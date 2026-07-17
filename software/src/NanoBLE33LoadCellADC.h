#pragma once

#include <Arduino.h>
#include <nrf.h>

#include "Board.h"

#if !defined(PLATFORM_NORDIC)
    #error "NanoBLE33LoadCellADC is exclusively for the Arduino Nano 33 BLE Rev2"
#endif

class NanoBLE33LoadCellADC
{
public:
    enum class Channel : uint8_t
    {
        Left1,
        Left2,
        Right1,
        Right2
    };

    void begin()
    {
        NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled;

        // Disable all SAADC channels. Channel 0 will be remapped before
        // each conversion.
        for (uint8_t channel = 0; channel < 8; ++channel)
        {
            NRF_SAADC->CH[channel].PSELP =
                SAADC_CH_PSELP_PSELP_NC
                << SAADC_CH_PSELP_PSELP_Pos;

            NRF_SAADC->CH[channel].PSELN =
                SAADC_CH_PSELN_PSELN_NC
                << SAADC_CH_PSELN_PSELN_Pos;
        }

        NRF_SAADC->RESOLUTION =
            SAADC_RESOLUTION_VAL_12bit
            << SAADC_RESOLUTION_VAL_Pos;

        NRF_SAADC->OVERSAMPLE =
            SAADC_OVERSAMPLE_OVERSAMPLE_Bypass
            << SAADC_OVERSAMPLE_OVERSAMPLE_Pos;

        NRF_SAADC->ENABLE =
            SAADC_ENABLE_ENABLE_Enabled
            << SAADC_ENABLE_ENABLE_Pos;

        calibrateSaadcOffset();
    }

    float sampleLeft1()
    {
        return sampleDifferential(Channel::Left1);
    }

    float sampleLeft2()
    {
        return sampleDifferential(Channel::Left2);
    }

    float sampleRight1()
    {
        return sampleDifferential(Channel::Right1);
    }

    float sampleRight2()
    {
        return sampleDifferential(Channel::Right2);
    }

    float sampleDifferential(Channel channel)
    {
        return rawToVolts(sampleDifferentialRaw(channel));
    }

    int16_t sampleDifferentialRaw(Channel channel)
    {
        const InputPair inputs = getInputPair(channel);

        configureDifferential(
            arduinoPinToSaadcInput(inputs.voPin),
            arduinoPinToSaadcInput(inputs.excitationPin)
        );

        return performConversion();
    }

    float calibrateChannelOffset(
        Channel channel,
        uint16_t sampleCount = 100,
        uint16_t delayBetweenSamplesMs = 2
    )
    {
        if (sampleCount == 0)
        {
            return 0.0f;
        }

        float sum = 0.0f;

        for (uint16_t i = 0; i < sampleCount; ++i)
        {
            sum += sampleDifferential(channel);

            if (delayBetweenSamplesMs > 0)
            {
                delay(delayBetweenSamplesMs);
            }
        }

        return sum / static_cast<float>(sampleCount);
    }

private:
    struct InputPair
    {
        int voPin;
        int excitationPin;
    };

    static constexpr float referenceVoltage_ = 0.6f;
    static constexpr float gain_ = 1.0f / 5.0f;
    static constexpr float differentialCounts_ = 2048.0f;

    inline static volatile int16_t sampleBuffer_ = 0;

    static constexpr uint32_t commonChannelConfiguration()
    {
        return
            (SAADC_CH_CONFIG_RESP_Bypass
                << SAADC_CH_CONFIG_RESP_Pos) |

            (SAADC_CH_CONFIG_RESN_Bypass
                << SAADC_CH_CONFIG_RESN_Pos) |

            (SAADC_CH_CONFIG_GAIN_Gain1_5
                << SAADC_CH_CONFIG_GAIN_Pos) |

            (SAADC_CH_CONFIG_REFSEL_Internal
                << SAADC_CH_CONFIG_REFSEL_Pos) |

            (SAADC_CH_CONFIG_TACQ_10us
                << SAADC_CH_CONFIG_TACQ_Pos) |

            (SAADC_CH_CONFIG_MODE_Diff
                << SAADC_CH_CONFIG_MODE_Pos) |

            (SAADC_CH_CONFIG_BURST_Disabled
                << SAADC_CH_CONFIG_BURST_Pos);
    }

    static InputPair getInputPair(Channel channel)
    {
        switch (channel)
        {
            case Channel::Left1:
                return {
                    boardConfig.LC_L_1_Vo,
                    boardConfig.LC_L_1_Exc
                };

            case Channel::Left2:
                return {
                    boardConfig.LC_L_2_Vo,
                    boardConfig.LC_L_2_Exc
                };

            case Channel::Right1:
                return {
                    boardConfig.LC_R_1_Vo,
                    boardConfig.LC_R_1_Exc
                };

            case Channel::Right2:
                return {
                    boardConfig.LC_R_2_Vo,
                    boardConfig.LC_R_2_Exc
                };
        }

        return {
            boardConfig.LC_L_1_Vo,
            boardConfig.LC_L_1_Exc
        };
    }

    static uint32_t arduinoPinToSaadcInput(int arduinoPin)
    {
        const PinName pinName = digitalPinToPinName(arduinoPin);
        const int gpio = static_cast<int>(pinName) & 0x1F;

        switch (gpio)
        {
            case 2:
                return SAADC_CH_PSELP_PSELP_AnalogInput0
                       << SAADC_CH_PSELP_PSELP_Pos;

            case 3:
                return SAADC_CH_PSELP_PSELP_AnalogInput1
                       << SAADC_CH_PSELP_PSELP_Pos;

            case 4:
                return SAADC_CH_PSELP_PSELP_AnalogInput2
                       << SAADC_CH_PSELP_PSELP_Pos;

            case 5:
                return SAADC_CH_PSELP_PSELP_AnalogInput3
                       << SAADC_CH_PSELP_PSELP_Pos;

            case 28:
                return SAADC_CH_PSELP_PSELP_AnalogInput4
                       << SAADC_CH_PSELP_PSELP_Pos;

            case 29:
                return SAADC_CH_PSELP_PSELP_AnalogInput5
                       << SAADC_CH_PSELP_PSELP_Pos;

            case 30:
                return SAADC_CH_PSELP_PSELP_AnalogInput6
                       << SAADC_CH_PSELP_PSELP_Pos;

            case 31:
                return SAADC_CH_PSELP_PSELP_AnalogInput7
                       << SAADC_CH_PSELP_PSELP_Pos;

            default:
                Serial.print("Error: Arduino pin ");
                Serial.print(arduinoPin);
                Serial.println(" is not connected to an nRF52840 SAADC input.");

                while (true)
                {
                    delay(1000);
                }
        }
    }

    static void configureDifferential(
        uint32_t positiveInput,
        uint32_t negativeInput
    )
    {
        NRF_SAADC->CH[0].PSELP = positiveInput;
        NRF_SAADC->CH[0].PSELN = negativeInput;
        NRF_SAADC->CH[0].CONFIG = commonChannelConfiguration();
    }

    static int16_t performConversion()
    {
        sampleBuffer_ = 0;

        NRF_SAADC->RESULT.PTR =
            reinterpret_cast<uint32_t>(&sampleBuffer_);

        NRF_SAADC->RESULT.MAXCNT = 1;

        NRF_SAADC->EVENTS_STARTED = 0;
        NRF_SAADC->EVENTS_END = 0;
        NRF_SAADC->EVENTS_STOPPED = 0;

        NRF_SAADC->TASKS_START = 1;
        waitForEvent(NRF_SAADC->EVENTS_STARTED);

        NRF_SAADC->TASKS_SAMPLE = 1;
        waitForEvent(NRF_SAADC->EVENTS_END);

        NRF_SAADC->TASKS_STOP = 1;
        waitForEvent(NRF_SAADC->EVENTS_STOPPED);

        return sampleBuffer_;
    }

    static void calibrateSaadcOffset()
    {
        NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
        NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;

        waitForEvent(NRF_SAADC->EVENTS_CALIBRATEDONE);
    }

    static void waitForEvent(volatile uint32_t& event)
    {
        while (event == 0)
        {
        }

        event = 0;
    }

    static float rawToVolts(int16_t raw)
    {
        return static_cast<float>(raw)
             * referenceVoltage_
             / (gain_ * differentialCounts_);
    }
};