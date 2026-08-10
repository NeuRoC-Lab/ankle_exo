#pragma once

#include <Arduino.h>
#include <array>
#include <cstdint>

#include "board.h"
#include "Encoder.h"
#include "LoadCell.h"
#include "CubeMarsMotor.h"

template<typename T>
class Driver
{
public:
    using ValueType = T;

    virtual ~Driver() = default;

    virtual bool begin() = 0;
    virtual T sample() = 0;
};


struct PowerReadings
{
    float batteryVoltage;
    float pcbCurrent;
    float pcbPower;
};

#if defined(PLATFORM_TEENSY)

#include <Wire.h>
#include <INA_Series_Sensor.h>

// Encoder ======

class EncoderDriver :
    public Driver<EncoderPositions>
{
public:
    EncoderDriver()
        : m_leftEncoder(Side::Left),
          m_rightEncoder(Side::Right)
    {}

    bool begin() override
    {
        pinMode(
            board::teensy41::pins.levelShifters.oe1,
            OUTPUT
        );

        pinMode(
            board::teensy41::pins.levelShifters.oe2,
            OUTPUT
        );

        digitalWrite(
            board::teensy41::pins.levelShifters.oe1,
            HIGH
        );

        digitalWrite(
            board::teensy41::pins.levelShifters.oe2,
            HIGH
        );

        m_leftEncoder.begin();
        m_rightEncoder.begin();

        return true;
    }

    EncoderPositions sample() override
    {
        EncoderPositions positions{};

        positions.left =
            m_leftEncoder.getPosition();

        delayMicroseconds(20);

        positions.right =
            m_rightEncoder.getPosition();

        if (positions.left == 1445) {
            positions.left = m_lastValid.left;
        } else {
            m_lastValid.left = positions.left;
        }

        if (positions.right == 1445) {
            positions.right = m_lastValid.right;
        } else {
            m_lastValid.right = positions.right;
        }

        return positions;
    }

private:
    Encoder m_leftEncoder;
    Encoder m_rightEncoder;
    EncoderPositions m_lastValid{};
};

// INA232 ======


class INA232Driver :
    public Driver<PowerReadings>
{
public:
    bool begin() override
    {
        m_sensor.begin(18, 19);

        m_sensor.setRshunt(0.015f); //TODO fine tune it
        m_sensor.setImax(10.0f);

        return true;
    }

    PowerReadings sample() override
    {
        if (m_sensor.dataReady())
        {
            return {
                m_sensor.readBusVoltage(),
                m_sensor.readCurrent(),
                m_sensor.readPower()
            };
        }

        return {
            -1.0f,
            -1.0f,
            -1.0f
        };
    }

private:
    InaBridge226 m_sensor{
        "INA232",
        0x40
    };
};



class MotorDriver
{
public:
    using CommandType = MotorCmd;
    using FeedbackType = MotorReply;

    MotorDriver(
        uint8_t canId,
        const AK60Params& limits,
        CanBus& bus)
        : m_motor(canId, limits),
          m_canBus(bus)
    {}

    bool enterMotorMode()
    {
        return m_canBus.send(
            m_motor.enterMotorMode()
        );
    }

    bool exitMotorMode()
    {
        return m_canBus.send(
            m_motor.exitMotorMode()
        );
    }

    bool zeroMotor()
    {
        return m_canBus.send(
            m_motor.setZeroPosition()
        );
    }
    bool begin()
    {
        if (!m_canBus.isReady() &&
            !m_canBus.begin())
        {
            return false;
        }

        if (!m_motor.begin()) {
            return false;
        }

        // Put drive into a known state first.
        if (!exitMotorMode()) {
            return false;
        }

        delay(200);

        if (!enterMotorMode()) {
            return false;
        }

        delay(100);

        return true;
    }

    bool apply(const MotorCmd& cmd)
    {
        return m_canBus.send(
            m_motor.packCommand(cmd)
        );
    }

    bool accepts(const CanFrame& frame) const
    {
        return frame.id == m_motor.canId();
    }

    MotorReply decode(const CanFrame& frame) const
    {
        return m_motor.unpackReply(frame);
    }

private:
    CubeMarsMotor m_motor;
    CanBus& m_canBus;
};

#endif

#if defined(PLATFORM_NANO)

class LoadCellDriver :
    public Driver<LoadCellForces>
{
public:
    bool begin() override
    {
        nrf_saadc_disable();

        for (uint8_t channel = 0;
             channel < NRF_SAADC_CHANNEL_COUNT;
             ++channel)
        {
            nrf_saadc_channel_input_set(
                channel,
                NRF_SAADC_INPUT_DISABLED,
                NRF_SAADC_INPUT_DISABLED
            );
        }

        nrf_saadc_resolution_set(
            NRF_SAADC_RESOLUTION_12BIT
        );

        nrf_saadc_oversample_set(
            NRF_SAADC_OVERSAMPLE_DISABLED
        );

        nrf_saadc_enable();

        calibrateSaadcHardwareOffset();

        for (auto& loadCell : m_loadCells)
        {
            loadCell.begin();

            if (!loadCell.valid()) {
                return false;
            }
        }

        calibrateOffsets();

        return true;
    }

    LoadCellForces sample() override
    {
        if (!performScan()) {
            return {};
        }

        LoadCellForces forces{};

        for (size_t i = 0; i < LoadCellCount; ++i)
        {
            m_loadCells[i].setLatestDiffVoltage(
                rawToDiffVoltage(m_rawBuffer[i])
            );

            forces[i] =
                m_loadCells[i].sampleForce();
        }

        return forces;
    }

private:
    static float rawToDiffVoltage(
        nrf_saadc_value_t raw)
    {
        return
            static_cast<float>(raw) *
            nanoSaadcReferenceVoltage /
            (
                nanoSaadcGain *
                nanoSaadcDifferentialCounts
            );
    }

    static void waitForEvent(
        nrf_saadc_event_t event)
    {
        while (!nrf_saadc_event_check(event))
        {
        }

        nrf_saadc_event_clear(event);
    }

    static void calibrateSaadcHardwareOffset()
    {
        nrf_saadc_event_clear(
            NRF_SAADC_EVENT_CALIBRATEDONE
        );

        nrf_saadc_task_trigger(
            NRF_SAADC_TASK_CALIBRATEOFFSET
        );

        waitForEvent(
            NRF_SAADC_EVENT_CALIBRATEDONE
        );
    }

    bool performScan()
    {
        if (!nrf_saadc_enable_check()) {
            return false;
        }

        nrf_saadc_buffer_init(
            m_rawBuffer,
            LoadCellCount
        );

        nrf_saadc_event_clear(
            NRF_SAADC_EVENT_STARTED
        );

        nrf_saadc_event_clear(
            NRF_SAADC_EVENT_END
        );

        nrf_saadc_event_clear(
            NRF_SAADC_EVENT_STOPPED
        );

        nrf_saadc_task_trigger(
            NRF_SAADC_TASK_START
        );

        waitForEvent(
            NRF_SAADC_EVENT_STARTED
        );

        nrf_saadc_task_trigger(
            NRF_SAADC_TASK_SAMPLE
        );

        waitForEvent(
            NRF_SAADC_EVENT_END
        );

        nrf_saadc_task_trigger(
            NRF_SAADC_TASK_STOP
        );

        waitForEvent(
            NRF_SAADC_EVENT_STOPPED
        );

        return true;
    }

    void calibrateOffsets(
        uint16_t sampleCount = 100)
    {
        std::array<double, LoadCellCount> sums{};
        uint16_t successfulSamples = 0;

        for (uint16_t sample = 0;
             sample < sampleCount;
             ++sample)
        {
            if (!performScan()) {
                continue;
            }

            ++successfulSamples;

            for (size_t i = 0;
                 i < LoadCellCount;
                 ++i)
            {
                sums[i] +=
                    rawToDiffVoltage(
                        m_rawBuffer[i]
                    );
            }

            delay(2);
        }

        if (successfulSamples == 0) {
            return;
        }

        for (size_t i = 0;
             i < LoadCellCount;
             ++i)
        {
            m_loadCells[i].setZeroOffset(
                static_cast<float>(
                    sums[i] /
                    successfulSamples
                )
            );
        }
    }

private:
    nrf_saadc_value_t
        m_rawBuffer[NRF_SAADC_CHANNEL_COUNT]{};

    std::array<LoadCell, LoadCellCount>
        m_loadCells{
            LoadCell{
                loadCellParams,
                LoadCellId::Left1
            },
            LoadCell{
                loadCellParams,
                LoadCellId::Left2
            },
            LoadCell{
                loadCellParams,
                LoadCellId::Right1
            },
            LoadCell{
                loadCellParams,
                LoadCellId::Right2
            }
        };
};

#endif
