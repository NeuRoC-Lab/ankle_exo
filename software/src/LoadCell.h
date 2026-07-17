//
// Created by Oscar Tesniere on 23/06/2026.
#include "Board.h"
#include <Arduino.h>
// #TODO Move this to Board.h

 struct LoadCellParams{
    static constexpr float sensitivity = 0.001f; // 1 mV/V full-scale
    static constexpr float ratedN = 1000.0f; // defined on the load cell package
};


class LoadCell {
// superclass for loadCell. Then implementation will differ from board to board (i.e Arduino Uno vs Teensy)
 public:
    LoadCellParams m_lc_params;
    INA125UParams m_ina_params;
    int m_Vo;

    LoadCell(const int Vo){
        m_Vo = Vo;
    }
    virtual void initialize();
    virtual float getVo();

    float voltageToN() {
        return (m_ina_params.IAref - getVo()) * m_lc_params.ratedN
               / (m_ina_params.ampGain * m_lc_params.sensitivity * m_ina_params.Vexc);
    }
    float rawVoltage() {
        return getVo();
    }
};

#if defined(PLATFORM_TEENSY41) // Using the Teensy 41
#define ADC_REF_V 3.3f
constexpr float voltage_scale = 3.3f;
constexpr uint16_t adc_resolution_bits = 12; // later try 14 bits ?
constexpr uint32_t adc_max_value = (1UL << adc_resolution_bits) - 1;
class LoadCell_Teensy41 : public LoadCell {

public:
    LoadCell_Teensy41(const int m_Vo) : LoadCell(m_Vo) {}

    virtual void initialize(){
        analogReadResolution(adc_resolution_bits);
        analogReadAveraging(16); // Optional, moving window ?
    }
    virtual float getVo(){
        return analogRead(m_Vo)*voltage_scale/adc_max_value;
    }
};

#elif defined(PLATFORM_RENESAS_RA) // using the Arduino Uno R4 (Renesas) f
constexpr bool usingInternalRef = false; // for that the voltage must be adjusted below 1.24V (BG reference)
constexpr float voltage_scale = usingInternalRef ? 1.5f : 5.0f;
constexpr uint16_t adc_resolution_bits = 14;
constexpr uint32_t adc_max_value = (1UL << adc_resolution_bits) - 1;

static_assert(!usingInternalRef || INA125UParams::IAref == 1.5, "Cannot use the Arduino UNO R4 internal reference voltage of 1.5V if the output offset is greater than the bandgap reference");

class LoadCell_Renesas : public LoadCell {

public:
    LoadCell_Renesas(const int m_Vo) : LoadCell(m_Vo) {}

    virtual void initialize(){
     analogReadResolution(adc_resolution_bits);
        // from https://www.pjrc.com/store/teensy41.html#analog :
        // 18 pins can be used an analog inputs, for reading sensors or other analog signals. Basic analog input is done with the analogRead function. The default resolution is 10 bits (input range 0 to 1023), but can be adjusted with analogReadResolution. The hardware allows up to 12 bits of resolution, but in practice only up to 10 bits are normally usable due to noise. More advanced use is possible with the ADC library.
    if(usingInternalRef){
     analogReference(AR_INTERNAL);
    // set the alaog reference to 1.5V
    }
    }
 virtual float getVo(){
    return analogRead(m_Vo)*voltage_scale/adc_max_value;
    }
};

#elif defined(PLATFORM_ATMEL_AVR) // using the Arduino Uno R3 (AVR based) => to connect directly to the external amplifier
constexpr uint16_t adc_resolution_bits = 10;
constexpr uint32_t adc_max_value = (1UL << adc_resolution_bits) - 1;
constexpr float voltage_scale = 5.0f;
class LoadCell_AtmelAVR : public LoadCell {

public:
    LoadCell_AtmelAVR(const int m_Vo) : LoadCell(m_Vo) {}

    virtual void initialize(){
        return; // nothing to do here
    }
    virtual float getVo(){
        return analogRead(m_Vo)*voltage_scale/adc_max_value;
    }
};

#else
#error "No CAN platform selected. Make sure to use Arduino UNO R4 or Teensy 4.1 or Atmel AVR (Uno R3)"
#endif


constexpr float fullScaleBridgeVoltage = LoadCellParams::sensitivity * INA125UParams::Vexc;
constexpr float fullScaleOutputSpan = INA125UParams::ampGain * fullScaleBridgeVoltage;

constexpr float minOutput = INA125UParams::IAref - fullScaleOutputSpan;
constexpr float maxOutput = INA125UParams::IAref;
static_assert(minOutput >= 0.0f, "INA output goes below 0 V at full-scale force");
static_assert(maxOutput <= voltage_scale, "INA output exceeds ADC reference voltage");

