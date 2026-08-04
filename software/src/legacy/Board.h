#pragma once
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#ifndef HW_VERSION_MAJOR
    #define HW_VERSION_MAJOR 1
#endif

#ifndef HW_VERSION_MINOR
    #define HW_VERSION_MINOR 0
#endif

#ifndef HW_VERSION_PATCH
    #define HW_VERSION_PATCH 0
#endif

#pragma message "Using HW version " STR(HW_VERSION_MAJOR) "." STR(HW_VERSION_MINOR) "." STR(HW_VERSION_PATCH)

//#pragma message "HW_VERSION_MAJOR = " STR(HW_VERSION_MAJOR)
//#pragma message "HW_VERSION_MINOR = " STR(HW_VERSION_MINOR)
//#pragma message "HW_VERSION_PATCH = " STR(HW_VERSION_PATCH)

// default to first (v1.0.0) version if the compiler is missing the hardware version. That way our code is "safe" and there is no ambiguity

#define HW_VERSION_ENCODE(major, minor, patch) \
((major) * 10000 + (minor) * 100 + (patch))

#define HARDWARE_VERSION \
HW_VERSION_ENCODE(HW_VERSION_MAJOR, HW_VERSION_MINOR, HW_VERSION_PATCH)

#define HW_VERSION_AT_LEAST(major, minor, patch) \
(HARDWARE_VERSION >= HW_VERSION_ENCODE(major, minor, patch))

#define HW_VERSION_AT_MOST(major, minor, patch) \
(HARDWARE_VERSION <= HW_VERSION_ENCODE(major, minor, patch))

#define HW_VERSION_EQUALS(major, minor, patch) \
(HARDWARE_VERSION == HW_VERSION_ENCODE(major, minor, patch))

#include <Arduino.h>

enum class INA125Ref {
    Bandgap_1V24, // JP: 2-3
    Ref_2V5       // JP: 2-1
};

enum class BoardSupply {
    V3V3, // JP: 2-3
    V5V   // JP: 2-1
};


struct BoardConfig {
    // LOAD CELL DEFINITIONS :

    INA125Ref ina125ExcitationRef;
    INA125Ref ina125IARef;
    BoardSupply ina125Supply;
    // load cell Analog Read pins
    int LC_L_1_pin;
    int LC_L_2_pin;
    int LC_R_1_pin;
    int LC_R_2_pin;

    // ENCODER DEFINITIONS

    int ENCODER_LEFT_CS;
    int ENCODER_RIGHT_CS;

    // output enable for the voltage shifters
    int OE1;
    int OE2;

    // analog pins to read the excitation voltage (IAREF) of the INA125U in order to validate the settings and proper operation of the board
    int left_amp_1_exc;
    int left_amp_2_exc;
    int right_amp_1_exc;
    int right_amp_2_exc;

    // for the Arduino Nano BLE 33 Rev 2
    int LC_L_1_Exc;
    int LC_L_1_Vo;
    int LC_L_2_Exc;
    int LC_L_2_Vo;
    int LC_R_1_Exc;
    int LC_R_1_Vo;
    int LC_R_2_Exc;
    int LC_R_2_Vo;
};

// CHANGE THESE DEPENDING ON YOUR SETUP
constexpr INA125Ref INA125_SELECTED_REF = INA125Ref::Ref_2V5;

#if defined(PLATFORM_TEENSY41)

#if HW_VERSION_AT_LEAST(1, 1, 0)
#pragma message("Defining OE1 and OE2")
#endif
constexpr BoardConfig boardConfig {

    #if HW_VERSION_AT_MOST(1,1,0) // version 1.2 remaps ina125 to Nano to use differential ADC
    .ina125ExcitationRef = INA125_SELECTED_REF,
    .ina125IARef = INA125_SELECTED_REF,
    .ina125Supply = BoardSupply::V5V,

    .LC_L_1_pin = A6,    // physical/digital pin 20
    .LC_L_2_pin = A7,    // physical/digital pin 21
    .LC_R_1_pin = A11,   // physical/digital pin 25
    .LC_R_2_pin = A10,    // physical/digital pin 24
    #endif

    .ENCODER_LEFT_CS = 0, // Pin no 0 on Teensy (CS2)
    .ENCODER_RIGHT_CS = 7, // Pin no 7 on Teensy (CS1)
    //TODO replace usage with .CS1 and .CS2 instead because this is more meaningful


    #if HW_VERSION_AT_LEAST(1,1,0)
    // output enable pins added after rev v1.0.0
    .OE1 = 6,
    .OE2 = 3,
    #endif

    // Note : only available past HW version 1.1.0
    #if HW_VERSION_EQUALS(1,1,0)
    .left_amp_1_exc = 15,
    .left_amp_2_exc = 14,
    .right_amp_1_exc = 39,
    .right_amp_2_exc = 40,
    #endif

};

#elif defined(PLATFORM_RENESAS_RA) || defined(PLATFORM_ATMEL_AVR)

constexpr BoardConfig boardConfig {
    .ina125ExcitationRef = INA125_SELECTED_REF,
    .ina125IARef = INA125_SELECTED_REF,
    .ina125Supply = BoardSupply::V5V,

    .LC_L_1_pin = A0,
    .LC_L_2_pin = A1,
    .LC_R_1_pin = A2,
    .LC_R_2_pin = A3,

    .ENCODER_LEFT_CS = 10, // digital Pin 10 on Arduino Uno
    .ENCODER_RIGHT_CS = 9

};


#elif defined(PLATFORM_NORDIC)

inline constexpr BoardConfig boardConfig {
    // INA125 configuration
    INA125_SELECTED_REF, // ina125ExcitationRef
    INA125_SELECTED_REF, // ina125IARef
    BoardSupply::V5V,    // ina125Supply

    // Conventional load-cell analog pins
    -1, // LC_L_1_pin
    -1, // LC_L_2_pin
    -1, // LC_R_1_pin
    -1, // LC_R_2_pin

    // Encoder chip-select pins
    -1, // ENCODER_LEFT_CS
    -1, // ENCODER_RIGHT_CS

    // Voltage-shifter output-enable pins
    -1, // OE1
    -1, // OE2

    // INA125 excitation measurement pins
    -1, // left_amp_1_exc
    -1, // left_amp_2_exc
    -1, // right_amp_1_exc
    -1, // right_amp_2_exc

    // Nordic differential ADC pins
    A1, // LC_L_1_Exc
    A0, // LC_L_1_Vo

    A3, // LC_L_2_Exc
    A2, // LC_L_2_Vo

    A5, // LC_R_1_Exc
    A4, // LC_R_1_Vo

    A7, // LC_R_2_Exc
    A6  // LC_R_2_Vo
};


#else
#error "No platform selected for BoardConfig"
#endif
// =================================

constexpr float refVoltage(INA125Ref ref) {
    return ref == INA125Ref::Bandgap_1V24 ? 1.24f :
           ref == INA125Ref::Ref_2V5      ? 2.5f  :
                                            0.0f;
}

constexpr float ina125SupplyVoltage(BoardSupply supply) {
    return supply == BoardSupply::V3V3 ? 3.3f :
           supply == BoardSupply::V5V  ? 5.0f :
                                         0.0f;
}

/*
constexpr bool uses2V5Reference =
    boardConfig.ina125ExcitationRef == INA125Ref::Ref_2V5 ||
    boardConfig.ina125IARef == INA125Ref::Ref_2V5;
*/
struct INA125UParams {
    static constexpr float gainR = 10.0f;
    static constexpr float ampGain = 4.0f + 60000.0f / gainR;

    static constexpr float IAref = refVoltage(boardConfig.ina125IARef);
    static constexpr float Vexc  = refVoltage(boardConfig.ina125ExcitationRef);
    // change Vexc to equal 5.0f or 3.3f if using an external excitation voltage (coming for example from the regulated 5V/3.3V on the PCB)
};

static_assert(
    boardConfig.ina125Supply == BoardSupply::V5V ||
    (
        boardConfig.ina125ExcitationRef != INA125Ref::Ref_2V5 &&
        boardConfig.ina125IARef != INA125Ref::Ref_2V5
    ),
    "INA125U must be powered from +5V to use the +2.5V internal reference"
);

static_assert(INA125UParams::gainR > 0.0f, "Gain resistor must be positive");
static_assert(INA125UParams::ampGain > 0.0f, "Gain value must be positive");

#if defined(PLATFORM_TEENSY41) && HW_VERSION_AT_LEAST(1,1,0)

void check_excitation_voltages() {
    const int exc_pins[] = {
        boardConfig.left_amp_1_exc,
        boardConfig.left_amp_2_exc,
        boardConfig.right_amp_1_exc,
        boardConfig.right_amp_2_exc
    };

    const float expected = refVoltage(boardConfig.ina125IARef);

    for (int exc_pin : exc_pins) {
        float measured = analogRead(exc_pin) * 3.3f / 4095.0f;

        if (fabs(measured - expected) > 0.2f) {
            Serial.print("Error: reference voltage for EXC pin ");
            Serial.print(exc_pin);
            Serial.print(" is ");
            Serial.print(measured, 3);
            Serial.print(" V, expected ");
            Serial.print(expected, 3);
            Serial.println(" V. Make sure the solder jumpers are configured properly.");

            while (1); // abort code execution
        }
    }
}

#endif