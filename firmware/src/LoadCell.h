#pragma once

#include <Arduino.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "board.h"

// Keep this type available on BOTH MCUs because the Teensy needs to
// deserialize LoadCellSnapshot messages even though it does not own the ADC.
using LoadCellForces = std::array<float, LoadCellCount>;

#if defined(PLATFORM_NANO)

#include <nrf_saadc.h>

inline constexpr float nanoSaadcReferenceVoltage = 0.6f;
inline constexpr float nanoSaadcGain = 1.0f / 5.0f;
inline constexpr float nanoSaadcDifferentialCounts = 2048.0f;

struct LoadCellParams
{
    static constexpr float sensitivity = 0.001f;
    static constexpr float ratedN = 1000.0f;
};

inline LoadCellParams loadCellParams{};

struct NanoLoadCellPins
{
    int voPin;
    int excPin;
};

class LoadCell
{
public:
    LoadCell(
        const LoadCellParams& params,
        LoadCellId id)
        : m_loadCellId(id),
          m_loadCellParams(params),
          m_channelIndex(static_cast<uint8_t>(id))
    {}

    void begin()
    {
        if (m_channelIndex >= NRF_SAADC_CHANNEL_COUNT)
        {
            m_valid = false;
            return;
        }

        const NanoLoadCellPins pins =
            pinsForLoadCell(m_loadCellId);

        const nrf_saadc_input_t positiveInput =
            arduinoPinToSaadcInput(pins.voPin);

        const nrf_saadc_input_t negativeInput =
            arduinoPinToSaadcInput(pins.excPin);

        if (positiveInput == NRF_SAADC_INPUT_DISABLED ||
            negativeInput == NRF_SAADC_INPUT_DISABLED)
        {
            m_valid = false;
            return;
        }

        nrf_saadc_channel_config_t config{};
        config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
        config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
        config.gain = NRF_SAADC_GAIN1_5;
        config.reference = NRF_SAADC_REFERENCE_INTERNAL;
        config.acq_time = NRF_SAADC_ACQTIME_3US;
        config.mode = NRF_SAADC_MODE_DIFFERENTIAL;
        config.pin_p = positiveInput;
        config.pin_n = negativeInput;

        nrf_saadc_channel_init(m_channelIndex, &config);

        m_valid = true;
    }

    void setLatestDiffVoltage(float voltage)
    {
        m_latestDiffVoltage = voltage;
    }

    void setZeroOffset(float offset)
    {
        m_zeroOffsetVoltage = offset;
    }

    bool valid() const
    {
        return m_valid;
    }

    float sampleForce() const
    {
        const float correctedVoltage =
            m_latestDiffVoltage - m_zeroOffsetVoltage;

        const float fullScaleDifferentialVoltage =
            inaParams.ampGain *
            m_loadCellParams.sensitivity *
            inaParams.Vexc;

        if (fullScaleDifferentialVoltage <= 0.0f)
        {
            return 0.0f;
        }

        return
            correctedVoltage *
            m_loadCellParams.ratedN /
            fullScaleDifferentialVoltage;
    }

private:
    static NanoLoadCellPins pinsForLoadCell(LoadCellId id)
    {
        const size_t index = static_cast<size_t>(id);

        return {
            board::nano::pins.loadCells.outputs[index],
            board::nano::pins.excitationPins.outputs[index]
        };
    }

    static nrf_saadc_input_t arduinoPinToSaadcInput(int arduinoPin)
    {
        if (arduinoPin < 0)
        {
            return NRF_SAADC_INPUT_DISABLED;
        }

        const PinName pinName =
            digitalPinToPinName(arduinoPin);

        const uint32_t gpioPin =
            static_cast<uint32_t>(pinName) & 0x1FU;

        switch (gpioPin)
        {
            case 2:  return NRF_SAADC_INPUT_AIN0;
            case 3:  return NRF_SAADC_INPUT_AIN1;
            case 4:  return NRF_SAADC_INPUT_AIN2;
            case 5:  return NRF_SAADC_INPUT_AIN3;
            case 28: return NRF_SAADC_INPUT_AIN4;
            case 29: return NRF_SAADC_INPUT_AIN5;
            case 30: return NRF_SAADC_INPUT_AIN6;
            case 31: return NRF_SAADC_INPUT_AIN7;
            default: return NRF_SAADC_INPUT_DISABLED;
        }
    }

private:
    LoadCellId m_loadCellId;
    LoadCellParams m_loadCellParams;
    float m_zeroOffsetVoltage{0.0f};
    uint8_t m_channelIndex{0};
    float m_latestDiffVoltage{0.0f};
    bool m_valid{false};
};

#endif
