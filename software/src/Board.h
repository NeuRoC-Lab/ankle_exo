#pragma once

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
};

// CHANGE THESE DEPENDING ON YOUR SETUP
constexpr INA125Ref INA125_SELECTED_REF = INA125Ref::Bandgap_1V24;

#if defined(PLATFORM_TEENSY41)

constexpr BoardConfig boardConfig {
    .ina125ExcitationRef = INA125_SELECTED_REF,
    .ina125IARef = INA125_SELECTED_REF,
    .ina125Supply = BoardSupply::V5V,

    .LC_L_1_pin = A6,    // physical/digital pin 20
    .LC_L_2_pin = A7,    // physical/digital pin 21
    .LC_R_1_pin = A11,   // physical/digital pin 25
    .LC_R_2_pin = A10,    // physical/digital pin 24

    .ENCODER_LEFT_CS = 1, // Pin no 1 on Teensy
    .ENCODER_RIGHT_CS = 7 // Pin no 7 on Teensy

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

constexpr bool uses2V5Reference =
    boardConfig.ina125ExcitationRef == INA125Ref::Ref_2V5 ||
    boardConfig.ina125IARef == INA125Ref::Ref_2V5;

struct INA125UParams {
    static constexpr float gainR = 470.0f;
    static constexpr float ampGain = 4.0f + 60000.0f / gainR;

    static constexpr float IAref = refVoltage(boardConfig.ina125IARef);
    static constexpr float Vexc  = refVoltage(boardConfig.ina125ExcitationRef);
    // change Vexc to equal 5.0f or 3.3f if using an external excitation voltage (coming for example from the regulated 5V/3.3V on the PCB)
};

static_assert(
    boardConfig.ina125Supply == BoardSupply::V5V || !uses2V5Reference,
    "INA125U must be powered from +5V to use the +2.5V internal reference"
);
static_assert(INA125UParams::gainR > 0.0f, "Gain resistor must be positive");
static_assert(INA125UParams::ampGain > 0.0f, "Gain value must be positive");