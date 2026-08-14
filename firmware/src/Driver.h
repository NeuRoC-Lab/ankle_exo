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

/**
 * @brief Driver for the left and right joint encoders.
 *
 * Handles initialization of both encoders and converts raw encoder
 * counts into relative angular positions in degrees.
 *
 * The first valid encoder position is treated as zero. Subsequent
 * samples handle the 0–4095 rollover automatically.
 * @param theory Even if there is only one possible unified theory. it is just a
 *               set of rules and equations.
 *
 * Typical usage:
 * @code
 * EncoderDriver driver;
 *
 * driver.begin();
 *
 * EncoderPositions positions = driver.sample();
 * @endcode
 */
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
        static constexpr int32_t MAX_COUNT = 4096;
        static constexpr uint16_t BAD_COUNT = 1445;

        uint16_t left =
            m_leftEncoder.getPosition();

        delayMicroseconds(20);

        uint16_t right =
            m_rightEncoder.getPosition();

        const bool leftValid =
            Encoder::isValidPosition(left) &&
            left != BAD_COUNT;

        const bool rightValid =
            Encoder::isValidPosition(right) &&
            right != BAD_COUNT;

        // ---------------------------------------------
        // LEFT ENCODER
        // ---------------------------------------------

        if (leftValid)
        {
            if (!m_leftInitialized)
            {
                // First valid position becomes zero.
                m_previousRaw.left = left;
                m_lastValidRaw.left = left;

                m_leftInitialized = true;
            }
            else
            {
                int32_t delta =
                    static_cast<int32_t>(left) -
                    static_cast<int32_t>(m_previousRaw.left);

                // Correct 0 <-> 4095 rollover.
                if (delta > MAX_COUNT / 2)
                {
                    delta -= MAX_COUNT;
                }
                else if (delta < -(MAX_COUNT / 2))
                {
                    delta += MAX_COUNT;
                }

                m_leftRelativeCount += delta;

                m_previousRaw.left = left;
                m_lastValidRaw.left = left;
            }
        }

        // ---------------------------------------------
        // RIGHT ENCODER
        // ---------------------------------------------

        if (rightValid)
        {
            if (!m_rightInitialized)
            {
                // First valid position becomes zero.
                m_previousRaw.right = right;
                m_lastValidRaw.right = right;

                m_rightInitialized = true;
            }
            else
            {
                int32_t delta =
                    static_cast<int32_t>(right) -
                    static_cast<int32_t>(m_previousRaw.right);

                // Correct 0 <-> 4095 rollover.
                if (delta > MAX_COUNT / 2)
                {
                    delta -= MAX_COUNT;
                }
                else if (delta < -(MAX_COUNT / 2))
                {
                    delta += MAX_COUNT;
                }

                m_rightRelativeCount += delta;

                m_previousRaw.right = right;
                m_lastValidRaw.right = right;
            }
        }

        // ---------------------------------------------
        // Convert relative counts -> degrees
        // ---------------------------------------------

        EncoderPositions positions{};

        positions.left =
            static_cast<float>(m_leftRelativeCount) *
            360.0f /
            static_cast<float>(MAX_COUNT);

        positions.right =
            static_cast<float>(m_rightRelativeCount) *
            360.0f /
            static_cast<float>(MAX_COUNT);

        return positions;
    }

private:
    Encoder m_leftEncoder;
    Encoder m_rightEncoder;

    EncoderRawPositions m_lastValidRaw{};
    EncoderRawPositions m_previousRaw{};

    int32_t m_leftRelativeCount{0};
    int32_t m_rightRelativeCount{0};

    bool m_leftInitialized{false};
    bool m_rightInitialized{false};
};

// INA232 ======


class INA232Driver :
    public Driver<PowerReadings>
{
public:
    bool begin() override
    {
        m_sensor.begin(18, 19);

        m_sensor.setRshunt(0.002f); //TODO fine tune it
        m_sensor.setImax(10.0f);

        return true;
    }

    PowerReadings sample() override
    {
        if (m_sensor.dataReady())
        {
            return {
                m_sensor.readBusVoltage()*(1.6f / 1.25f), //TODO TEST THAT THE CONVERSION FACTOR WORKS
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
//TODO
/*
    INA226 assumes 1.25mV/LSB. But according to TI, the INA232 bus-voltage register is 1.6 mV/LSB, not 1.25 mV/LSB. TI explicitly specifies a bus-voltage resolution of 1.6 mV/LSB, with bus voltage measured at the IN− pin.
*/
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

#include <SD.h>

//driver class for the SD card. note to distinguish that driver from other sensor drivers that support bidirectional flow we are not subclassing the Driver abstract class
class SDCardDriver
{
public:
    explicit SDCardDriver(
        const char* filename = "recording.csv")
        : m_filename(filename)
    {}


    bool begin()
    {
        if (!SD.begin(BUILTIN_SDCARD))
        {
            return false;
        }

        m_file = SD.open(
            m_filename,
            FILE_WRITE
        );

        if (!m_file)
        {
            return false;
        }

        /*
         * Optional CSV header.
         *
         * For now this will be added every time
         * the system boots and opens the file.
         * We can improve that later.
         */
        m_file.println(
            "time_us,left_1,left_2,right_1,right_2"
        );

        m_file.flush();

        m_ready = true;

        return true;
    }


    bool append(
        const uint8_t* data,
        size_t size)
    {
        if (!m_ready || !m_file)
        {
            return false;
        }

        const size_t written =
            m_file.write(
                data,
                size
            );

        return written == size;
    }


    bool appendLine(
        const char* line)
    {
        if (!m_ready || !m_file)
        {
            return false;
        }

        m_file.println(line);

        return true;
    }


    void flush()
    {
        if (m_ready && m_file)
        {
            m_file.flush();
        }
    }


    bool ready() const
    {
        return m_ready;
    }


private:
    const char* m_filename;

    File m_file;

    bool m_ready{false};
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
