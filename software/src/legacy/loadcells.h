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


#elif defined(PLATFORM_ATMEL_AVR) // deprecated and no longer used in production

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

