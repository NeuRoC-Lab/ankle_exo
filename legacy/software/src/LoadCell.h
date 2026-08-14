#pragma once

#include <Arduino.h>
#include "Board.h"



inline constexpr float voltage_scale = 3.3f;

struct LoadCellParams
{
    static constexpr float sensitivity = 0.001f; // 1 mV/V full-scale
    static constexpr float ratedN = 1000.0f;
};




struct NanoLoadCellPins
{
    int voPin;
    int excPin;
};

// The use of a pointer-based array for ordering the load cells into the array gives us some extra flexibility : the order given into LoadCellHandler will be propagated to functions using it without ensuring that it follows a specific sequence.


class LoadCell
{
public:
    LoadCell(
        const LoadCellParams& loadCellParams
    )
        : m_loadCellParams(loadCellParams)
    {
    }

    virtual ~LoadCell() = default;

    virtual void begin() = 0;

    virtual float getDiffVoltage() = 0;

    float maxMeasurableTension() const
    {
        const float denominator =
            inaParams.ampGain
            * m_loadCellParams.sensitivity
            * inaParams.Vexc;

        if (denominator <= 0.0f)
        {
            return 0.0f;
        }

        const float availableVoltage =
            inaParams.IAref; // tension drives output from IAREF(=2.5V) toward 0 V

        return availableVoltage
             * m_loadCellParams.ratedN
             / denominator;
    }
    float sampleForce()
    {
        const float correctedVoltage =
            getDiffVoltage() - m_zeroOffsetVoltage;

        const float fullScaleDifferentialVoltage =
            inaParams.ampGain
            * m_loadCellParams.sensitivity
            * inaParams.Vexc;

        if (fullScaleDifferentialVoltage == 0.0f)
        {
            return 0.0f;
        }

        return std::clamp(correctedVoltage
             * m_loadCellParams.ratedN
             / fullScaleDifferentialVoltage, -maxMeasurableTension(), maxMeasurableTension());
    }
    // clamped to only valid values which we know will never be exceeded because the INA125U is physically limited to those ranges

    void calibrateOffset(
        uint16_t sampleCount = 100,
        uint16_t delayMs = 2
    )
    {
        if (sampleCount == 0)
        {
            return;
        }

        float sum = 0.0f;

        for (uint16_t i = 0; i < sampleCount; ++i)
        {
            sum += getDiffVoltage();

            if (delayMs > 0)
            {
                delay(delayMs);
            }
        }

        m_zeroOffsetVoltage =
            sum / static_cast<float>(sampleCount);
    }

    void setZeroOffset(float offset)
    {
        m_zeroOffsetVoltage = offset;
    }

protected:
    LoadCellParams m_loadCellParams;

    float m_zeroOffsetVoltage = 0.0f;
};


class LoadCellHandler
{
public:
    LoadCellHandler(LoadCell** lcArray, uint8_t lcCount,float* forceBuffer)
        : m_lcArray(lcArray),
          m_lcCount(lcCount),
          m_forceBuffer(forceBuffer)
    {
    }

    virtual ~LoadCellHandler() = default;

    virtual void begin()
    {
        for (uint8_t i = 0; i < m_lcCount; ++i)
        {
            if (m_lcArray[i] != nullptr)
            {
                m_lcArray[i]->begin();
                // no need to call begin() on every individual load cell. The handler does it for you
            }
        }
    }

    uint8_t count() const
    {
        return m_lcCount;
    }

    virtual const float* sampleAll()
    {
    // the handler owns the float* array so that the function simply returns a pointer to the updated values
        for (uint8_t i = 0; i < m_lcCount; ++i)
        {
            //m_forceBuffer[i] = m_lcArray[i]->getDiffVoltage(); // temporarily get diff voltage for debug purposes
            m_forceBuffer[i] = m_lcArray[i]->sampleForce();

        }

        return m_forceBuffer;
    }
    // returns an array of force readings from all the load cells


    virtual void calibrateAllOffsets(
        uint16_t sampleCount = 100
    ) = 0;

protected:
    LoadCell** m_lcArray;
    uint8_t m_lcCount;
    float* m_forceBuffer;
};

#if defined(PLATFORM_TEENSY41) && HW_VERSION_AT_MOST(1,1,0)
#pragma message "Compiling for Board versions <v1.1.1. This version assumes that load cell voltages are sampled through the Teensy. In later versions the Nano does it"


inline constexpr uint16_t adc_resolution_bits = 12;
inline constexpr uint32_t adc_max_value =
    (1UL << adc_resolution_bits) - 1UL;

class LoadCell_Teensy41 final : public LoadCell
{
public:

    LoadCell_Teensy41(LoadCellParams loadCellParams,LoadCellId id) : LoadCell(loadCellParams), m_VoPin(board::teensy41::pins.loadCells.outputs[static_cast<std::size_t>(id)]) {}

    void begin() override {
        // initialize the array
        pinMode(m_VoPin,INPUT);
    }

    float getDiffVoltage() {
        // gets the RAW differential voltage. It does so by substracting the preset IAREF voltage from the voltage read at pin Vo
        return analogRead(m_VoPin)*voltage_scale/ static_cast<float>(adc_max_value) - inaParams.IAref;
        }

private:
    Pin m_VoPin;
};

class LoadCellHandler_Teensy41 : public LoadCellHandler
{
public:
    LoadCellHandler_Teensy41(
        LoadCell** lcArray,
        uint8_t lcCount,
        float* forceBuffer
    )
        : LoadCellHandler(
              lcArray,
              lcCount,
              forceBuffer
          )
    {
    }

    void begin() override
    {
        // Shared Teensy ADC configuration.
        analogReadResolution(12);
        analogReadAveraging(16);

        // Run the generic per-load-cell initialization.
        LoadCellHandler::begin();
    }

    const float* sampleAll() override
    {
        return LoadCellHandler::sampleAll();
    }

    void calibrateAllOffsets(
        uint16_t sampleCount = 100
    ) override
    {
        for (uint8_t i = 0; i < m_lcCount; ++i)
        {
            if (m_lcArray[i] != nullptr)
            {
                m_lcArray[i]->calibrateOffset(sampleCount);
            }
        }
    }
};

#elif defined(PLATFORM_NORDIC) && HW_VERSION_AT_LEAST(1, 1, 1)

// ============================================================
// Arduino Nano 33 BLE Rev2 / nRF52840
// ============================================================

/*
use : #include <nrf_saadc.h>

ANALOG_REF_INTERNAL_VAL : macro for the internal reference voltage
`nrfx_analog_input_t` is an ENUM that catalogs references to all AIN channels

8 analog inputs:          AIN0 ... AIN7
8 hardware ADC channels:  CH[0] ... CH[7]
So two different categories of channels

struct nrfx_saadc_channel_t {
nrf_saadc_channel_config_t channel_config; // see below
nrfx_analog_input_t pin_p; // selecting positive input
nrfx_analog_input_t pin_n; // selecting negative input
uint8_t channel_index; // what is that for ???
}

struct nrf_saadc_channel_config_t {
    nrf_saadc_resistor_t resistor_p;
    nrf_saadc_resistor_t resistor_n;
    nrf_saadc_gain_t gain;
    nrf_saadc_reference_t reference;
    nrf_saadc_acqtime_t acq_time;
    nrf_saadc_mode_t mode = NRF_SAADC_MODE_DIFFERENTIAL; // for differential mode
    nrf_saadc_burst_t burst;
    nrf_saadc_chopping_t chopping;
    nrf_saadc_highspeed_t highspeed;
    uint8_t conv_time;
}

*/

// ============================================================
// Arduino Nano 33 BLE Rev2 / nRF52840
// ============================================================

#include <nrf_saadc.h>

/*
 * SAADC configuration:
 *
 * Internal reference: 0.6 V
 * Gain:               1/5
 * Resolution:         12-bit differential
 *
 * Differential full scale:
 *
 *     0.6 V / (1/5) = 3.0 V
 *
 * Differential 12-bit output:
 *
 *     -2048 ... +2047
 */

inline constexpr float nanoSaadcReferenceVoltage = 0.6f;
inline constexpr float nanoSaadcGain = 1.0f / 5.0f;
inline constexpr float nanoSaadcDifferentialCounts = 2048.0f;

class LoadCell_NanoBLE final : public LoadCell
{
public:
    LoadCell_NanoBLE(
    const LoadCellParams& loadCellParams,
    LoadCellId id
)
    : LoadCell(loadCellParams),
      m_loadCellId(id),
      m_channelIndex(static_cast<std::uint8_t>(id))
    {
    }

    void begin() override
    {
        if (m_channelIndex >= NRF_SAADC_CHANNEL_COUNT)
        {
            Serial.print("Invalid SAADC channel index: ");
            Serial.println(m_channelIndex);

            m_valid = false;
            return;
        }

        const NanoLoadCellPins pins =
            pinsForLoadCell(m_loadCellId);

        const nrf_saadc_input_t positiveInput =
            arduinoPinToSaadcInput(pins.voPin);

        const nrf_saadc_input_t negativeInput =
            arduinoPinToSaadcInput(pins.excPin);

        if (
            positiveInput == NRF_SAADC_INPUT_DISABLED
            || negativeInput == NRF_SAADC_INPUT_DISABLED
        )
        {
            Serial.println(
                "Invalid SAADC input pair for load cell."
            );

            m_valid = false;
            return;
        }

        m_pairConfig.resistor_p =
            NRF_SAADC_RESISTOR_DISABLED;

        m_pairConfig.resistor_n =
            NRF_SAADC_RESISTOR_DISABLED;

        m_pairConfig.gain =
            NRF_SAADC_GAIN1_5;

        m_pairConfig.reference =
            NRF_SAADC_REFERENCE_INTERNAL;

        m_pairConfig.acq_time =
            NRF_SAADC_ACQTIME_3US; // recommended for max 10k Ohms input resistance (ref : Table 95: Acquisition time)

        m_pairConfig.mode =
            NRF_SAADC_MODE_DIFFERENTIAL;

        /*
         * Differential measurement:
         *
         *     pin_p - pin_n
         *     Vo    - Exc/IAref
         */
        m_pairConfig.pin_p = positiveInput;
        m_pairConfig.pin_n = negativeInput;

        nrf_saadc_channel_init(
            m_channelIndex,
            &m_pairConfig
        );

        m_valid = true;
    }

    float getDiffVoltage() override
    {
        return m_latestDiffVoltage;
    }

    void setLatestDiffVoltage(float voltage)
    {
        m_latestDiffVoltage = voltage;
    }

    uint8_t channelIndex() const
    {
        return m_channelIndex;
    }

    LoadCellId id() const
    {
        return m_loadCellId;
    }

    bool valid() const
    {
        return m_valid;
    }

private:
    static NanoLoadCellPins pinsForLoadCell(
    LoadCellId id
    )
    {
        const std::size_t index =
            static_cast<std::size_t>(id);

        return {
            board::nano::pins.loadCells.outputs[index],
            board::nano::pins.excitationPins.outputs[index]
        };
    }

    static nrf_saadc_input_t arduinoPinToSaadcInput(
        int arduinoPin
    )
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
            case 2:
                return NRF_SAADC_INPUT_AIN0;

            case 3:
                return NRF_SAADC_INPUT_AIN1;

            case 4:
                return NRF_SAADC_INPUT_AIN2;

            case 5:
                return NRF_SAADC_INPUT_AIN3;

            case 28:
                return NRF_SAADC_INPUT_AIN4;

            case 29:
                return NRF_SAADC_INPUT_AIN5;

            case 30:
                return NRF_SAADC_INPUT_AIN6;

            case 31:
                return NRF_SAADC_INPUT_AIN7;

            default:
                Serial.print("Arduino pin ");
                Serial.print(arduinoPin);
                Serial.println(
                    " is not connected to an nRF52840 SAADC input."
                );

                return NRF_SAADC_INPUT_DISABLED;
        }
    }

    LoadCellId m_loadCellId;
    uint8_t m_channelIndex;

    nrf_saadc_channel_config_t m_pairConfig{};

    float m_latestDiffVoltage = 0.0f;
    bool m_valid = false;
};

class LoadCellHandler_NanoBLE final : public LoadCellHandler
{
public:
    LoadCellHandler_NanoBLE(
        LoadCell** lcArray,
        uint8_t lcCount,
        float* forceBuffer
    )
        : LoadCellHandler(
              lcArray,
              lcCount,
              forceBuffer
          )
    {
    }

    void begin() override
    {
        if (m_lcCount == 0)
        {
            Serial.println(
                "No load cells configured for the SAADC."
            );

            return;
        }

        if (m_lcCount > NRF_SAADC_CHANNEL_COUNT)
        {
            Serial.println(
                "Too many load cells for the SAADC."
            );

            return;
        }

        /*
         * Disable the SAADC before changing global configuration.
         */
        nrf_saadc_disable();

        /*
         * Disconnect every hardware channel first. This prevents
         * an old channel configuration from unexpectedly appearing
         * in the scan.
         */
        for (
            uint8_t channel = 0;
            channel < NRF_SAADC_CHANNEL_COUNT;
            ++channel
        )
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
            NRF_SAADC_OVERSAMPLE_DISABLED // oversampling is not recommended when using all 8 channels as effective sampling frequency would significantly drop
        );

        nrf_saadc_enable();

        calibrateSaadcHardwareOffset();

        /*
         * Configure every load-cell channel.
         */
        LoadCellHandler::begin();

        /*
         * This implementation expects:
         *
         * lcArray[0] -> SAADC CH[0]
         * lcArray[1] -> SAADC CH[1]
         * ...
         *
         * That guarantees the DMA result order matches lcArray.
         */
        for (uint8_t i = 0; i < m_lcCount; ++i)
        {
            LoadCell_NanoBLE* loadCell =
                nanoLoadCellAt(i);

            if (loadCell == nullptr)
            {
                Serial.println(
                    "Null Nano load-cell pointer."
                );

                continue;
            }

            if (loadCell->channelIndex() != i)
            {
                Serial.print(
                    "SAADC channel order mismatch at index "
                );

                Serial.println(i);
            }
        }
    }

    const float* sampleAll() override
    {
        if (!performScan())
        {
            /*
             * Preserve the force-buffer length and return safe values
             * if a conversion fails.
             */
            for (uint8_t i = 0; i < m_lcCount; ++i)
            {
                m_forceBuffer[i] = 0.0f;
            }

            return m_forceBuffer;
        }

        updateLoadCellVoltages();

        /*
         * The common implementation now calls sampleForce() for each
         * load cell. sampleForce() obtains the cached differential
         * voltage through getDiffVoltage().
         */
        return LoadCellHandler::sampleAll();
    }

    void calibrateAllOffsets(
        uint16_t sampleCount = 100
    ) override
    {
        if (sampleCount == 0)
        {
            return;
        }

        Serial.println(
            "Calibrating all load cells at zero force. "
            "Do not change cable tension."
        );

        /*
         * Double is useful here for accumulation, even though each
         * individual SAADC result is converted to float.
         */
        double sums[NRF_SAADC_CHANNEL_COUNT]{};

        uint16_t successfulSamples = 0;

        for (
            uint16_t sample = 0;
            sample < sampleCount;
            ++sample
        )
        {
            if (!performScan())
            {
                continue;
            }

            ++successfulSamples;

            for (uint8_t i = 0; i < m_lcCount; ++i)
            {
                sums[i] += rawToDiffVoltage(
                    m_rawBuffer[i]
                );
            }

            delay(2);
        }

        if (successfulSamples == 0)
        {
            Serial.println(
                "SAADC offset calibration failed: no samples."
            );

            return;
        }

        for (uint8_t i = 0; i < m_lcCount; ++i)
        {
            LoadCell_NanoBLE* loadCell =
                nanoLoadCellAt(i);

            if (loadCell == nullptr)
            {
                continue;
            }

            const float offset =
                static_cast<float>(
                    sums[i]
                    / static_cast<double>(
                        successfulSamples
                    )
                );

            loadCell->setZeroOffset(offset);
            loadCell->setLatestDiffVoltage(offset);
        }
    }

private:
    /*
     * The SAADC supports up to eight active channel results.
     * Only the first m_lcCount entries are used.
     */
    nrf_saadc_value_t
        m_rawBuffer[NRF_SAADC_CHANNEL_COUNT]{};

    static void waitForEvent(
        nrf_saadc_event_t event
    )
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
        if (!nrf_saadc_enable_check())
        {
            return false;
        }

        /*
         * One result is generated for every active channel.
         */
        nrf_saadc_buffer_init(
            m_rawBuffer,
            m_lcCount
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

        /*
         * In scan mode, one SAMPLE task samples every enabled
         * hardware channel.
         */
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

    static float rawToDiffVoltage(
        nrf_saadc_value_t raw
    )
    {
        return
            static_cast<float>(raw)
            * nanoSaadcReferenceVoltage
            / (
                nanoSaadcGain
                * nanoSaadcDifferentialCounts
            );
    }

    void updateLoadCellVoltages()
    {
        for (uint8_t i = 0; i < m_lcCount; ++i)
        {
            LoadCell_NanoBLE* loadCell =
                nanoLoadCellAt(i);

            if (loadCell == nullptr)
            {
                continue;
            }

            loadCell->setLatestDiffVoltage(
                rawToDiffVoltage(m_rawBuffer[i])
            );
        }
    }

    LoadCell_NanoBLE* nanoLoadCellAt(uint8_t index)
    {
        if (
            index >= m_lcCount
            || m_lcArray[index] == nullptr
        )
        {
            return nullptr;
        }

        /*
         * LoadCellHandler_NanoBLE must only be constructed with
         * LoadCell_NanoBLE objects.
         *
         * static_cast is used instead of dynamic_cast because RTTI is
         * often disabled in embedded builds.
         */
        return static_cast<LoadCell_NanoBLE*>(
            m_lcArray[index]
        );
    }
};

#elif defined(PLATFORM_TEENSY41) && HW_VERSION_AT_LEAST(1,1,1)
#pragma message "WARNING : not linking anything from LoadCell.h since using version v1.1.1+ on the Teensy"
#elif defined(PLATFORM_NORDIC) && HW_VERSION_AT_MOST(1, 1, 0)
#pragma message "WARNING : not linking anything from LoadCell.h since using version v1.1.0- on the Nano"
#else

    #error "No supported load-cell platform selected"

#endif

//TODO FIX THAT
// ============================================================
// Compile-time analog-range checks
// ============================================================
/*
inline constexpr float fullScaleBridgeVoltage =
    LoadCellParams::sensitivity
    * INA125UParams::Vexc;

inline constexpr float fullScaleOutputSpan =
    INA125UParams::ampGain
    * fullScaleBridgeVoltage;

inline constexpr float minOutput =
    INA125UParams::IAref - fullScaleOutputSpan;

inline constexpr float maxOutput =
    INA125UParams::IAref;

#if defined(DEBUG)
static_assert(
    minOutput >= 0.0f,
    "INA125 output goes below 0 V at full-scale force"
);
#endif

#if defined(PLATFORM_NORDIC)


 * Internal reference = 0.6 V
 * SAADC gain = 1/5
 *
 * Differential full scale:
 *
 *     0.6 / (1/5) = 3.0 V

inline constexpr float nanoDifferentialFullScale = 3.0f;

/*
static_assert(
    fullScaleOutputSpan <= nanoDifferentialFullScale,
    "INA125 differential output span exceeds the Nano BLE SAADC range"
);

#else

static_assert(
    maxOutput <= voltage_scale,
    "INA125 output exceeds the ADC measurement range"
);

#endif
*/