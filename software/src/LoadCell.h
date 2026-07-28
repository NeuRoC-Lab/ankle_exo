#pragma once

#include <Arduino.h>
#include "Board.h"

#if defined(PLATFORM_NORDIC)
    #include "NanoBLE33LoadCellADC.h"
#endif

struct LoadCellParams
{
    static constexpr float sensitivity = 0.001f; // 1 mV/V full-scale
    static constexpr float ratedN = 1000.0f;
};

class LoadCell
{
public:
    LoadCellParams m_lc_params;
    INA125UParams m_ina_params;

    int m_VoPin;
    float m_offset = 0.0f;

    explicit LoadCell(int VoPin)
        : m_VoPin(VoPin)
    {
    }

    virtual ~LoadCell() = default;

    virtual void initialize() = 0;
    virtual float getVo() = 0;
    virtual void calibrateOffset() = 0;

    /*
     * Default implementation for platforms that measure the absolute
     * INA125 output voltage relative to GND.
     */
    virtual float voltageToN()
    {
        const float correctedVo = getVo() - m_offset;

        return
            (m_ina_params.IAref - correctedVo)
            * m_lc_params.ratedN
            / (
                m_ina_params.ampGain
                * m_lc_params.sensitivity
                * m_ina_params.Vexc
            );
    }

    float rawVoltage()
    {
        return getVo();
    }
};


#if defined(PLATFORM_TEENSY41)

// ============================================================
// Teensy 4.1
// ============================================================

inline constexpr float voltage_scale = 3.3f;
inline constexpr uint16_t adc_resolution_bits = 12;
inline constexpr uint32_t adc_max_value =
    (1UL << adc_resolution_bits) - 1UL;

class LoadCell_Teensy41 : public LoadCell
{
public:
    explicit LoadCell_Teensy41(int VoPin)
        : LoadCell(VoPin)
    {
    }

    void initialize() override
    {
        analogReadResolution(adc_resolution_bits);
        analogReadAveraging(16);
    }

    float getVo() override
    {
        return static_cast<float>(analogRead(m_VoPin))
             * voltage_scale
             / static_cast<float>(adc_max_value);
    }

    void calibrateOffset() override
    {
        analogReadAveraging(32);
        delay(1000);

        Serial.println(
            "Calibrating the load cell for 0 N force. "
            "Do not change the tension in the cable."
        );

        /*
         * getVo() is an absolute voltage.
         *
         * At zero force:
         *     Vo = IAref + offset
         *
         * Therefore:
         *     offset = Vo - IAref
         */
        m_offset = getVo() - m_ina_params.IAref;

        analogReadAveraging(16);
    }
};


#elif defined(PLATFORM_RENESAS_RA)

// ============================================================
// Arduino UNO R4
// ============================================================

inline constexpr bool usingInternalRef = false;

inline constexpr float voltage_scale =
    usingInternalRef ? 1.5f : 5.0f;

inline constexpr uint16_t adc_resolution_bits = 14;
inline constexpr uint32_t adc_max_value =
    (1UL << adc_resolution_bits) - 1UL;

static_assert(
    !usingInternalRef || INA125UParams::IAref <= 1.5f,
    "Cannot use the UNO R4 1.5 V internal ADC reference when "
    "the INA125 output offset is greater than 1.5 V"
);

class LoadCell_Renesas : public LoadCell
{
public:
    explicit LoadCell_Renesas(int VoPin)
        : LoadCell(VoPin)
    {
    }

    void initialize() override
    {
        analogReadResolution(adc_resolution_bits);

        if constexpr (usingInternalRef)
        {
            analogReference(AR_INTERNAL);
        }
    }

    float getVo() override
    {
        return static_cast<float>(analogRead(m_VoPin))
             * voltage_scale
             / static_cast<float>(adc_max_value);
    }

    void calibrateOffset() override
    {
        delay(1000);

        Serial.println(
            "Calibrating the load cell for 0 N force. "
            "Do not change the tension in the cable."
        );

        m_offset = getVo() - m_ina_params.IAref;
    }
};


#elif defined(PLATFORM_ATMEL_AVR)

// ============================================================
// Arduino UNO R3 / AVR
// ============================================================

inline constexpr uint16_t adc_resolution_bits = 10;
inline constexpr uint32_t adc_max_value =
    (1UL << adc_resolution_bits) - 1UL;

inline constexpr float voltage_scale = 5.0f;

class LoadCell_AtmelAVR : public LoadCell
{
public:
    explicit LoadCell_AtmelAVR(int VoPin)
        : LoadCell(VoPin)
    {
    }

    void initialize() override
    {
        // Nothing required.
    }

    float getVo() override
    {
        return static_cast<float>(analogRead(m_VoPin))
             * voltage_scale
             / static_cast<float>(adc_max_value);
    }

    void calibrateOffset() override
    {
        delay(1000);

        Serial.println(
            "Calibrating the load cell for 0 N force. "
            "Do not change the tension in the cable."
        );

        m_offset = getVo() - m_ina_params.IAref;
    }
};


#elif defined(PLATFORM_NORDIC)

// ============================================================
// Arduino Nano 33 BLE Rev2 / nRF52840
// ============================================================

class LoadCell_NanoBLE : public LoadCell
{
public:
    using ADCChannel = NanoBLE33LoadCellADC::Channel;

    explicit LoadCell_NanoBLE(ADCChannel channel)
        : LoadCell(-1),
          m_channel(channel)
    {
    }

    void initialize() override
    {
        /*
         * The SAADC peripheral is shared by all LoadCell_NanoBLE
         * instances. Calling begin() repeatedly is unnecessary.
         *
         * initializeADC() ensures that initialization only happens once.
         */
        initializeADC();
    }

    float getVo() override
    {
        /*
         * Despite the inherited function name, this does not return the
         * absolute Vo voltage on the Nano BLE implementation.
         *
         * It directly returns:
         *
         *     Vo - Vexc
         */
        return adc_.sampleDifferential(m_channel);
    }

    void calibrateOffset() override
    {
        delay(1000);

        Serial.println(
            "Calibrating the load cell for 0 N force. "
            "Do not change the tension in the cable."
        );

        constexpr uint16_t sampleCount = 100;
        constexpr uint16_t sampleDelayMs = 2;

        float sum = 0.0f;

        for (uint16_t i = 0; i < sampleCount; ++i)
        {
            sum += getVo();
            delay(sampleDelayMs);
        }

        /*
         * getVo() already returns Vo - Vexc.
         *
         * At zero force:
         *
         *     m_offset = Vo_zero - Vexc
         */
        m_offset = sum / static_cast<float>(sampleCount);

        Serial.print("Measured differential zero offset: ");
        Serial.print(m_offset, 6);
        Serial.println(" V");
    }

    float voltageToN() override
    {
        /*
         * Differential SAADC reading:
         *
         *     measured = Vo - Vexc
         *
         * Corrected differential reading:
         *
         *     corrected = measured - zeroOffset
         *
         * With your polarity, Vo decreases under positive force, so
         * corrected is negative. Negate it to obtain positive force.
         */
        const float correctedDifferential =
            getVo() - m_offset;

        return
            -correctedDifferential
            * m_lc_params.ratedN
            / (
                m_ina_params.ampGain
                * m_lc_params.sensitivity
                * m_ina_params.Vexc
            );
    }

private:
    ADCChannel m_channel;

    inline static NanoBLE33LoadCellADC adc_{};
    inline static bool adcInitialized_ = false;

    static void initializeADC()
    {
        if (!adcInitialized_)
        {
            adc_.begin();
            adcInitialized_ = true;
        }
    }
};


#else

    #error "No supported load-cell platform selected"

#endif


// ============================================================
// Compile-time analog-range checks
// ============================================================

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

/*
 * Internal reference = 0.6 V
 * SAADC gain = 1/5
 *
 * Differential full scale:
 *
 *     0.6 / (1/5) = 3.0 V
 */
inline constexpr float nanoDifferentialFullScale = 3.0f;

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