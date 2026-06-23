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
    INA125Ref ina125ExcitationRef;
    INA125Ref ina125IARef;
    BoardSupply ina125Supply;

    int LC_L_1_pin;
    int LC_L_2_pin;
    int LC_R_1_pin;
    int LC_R_2_pin;
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
    .LC_R_2_pin = A10    // physical/digital pin 24
};

#elif defined(PLATFORM_RENESAS_RA) || defined(PLATFORM_ATMEL_AVR)

constexpr BoardConfig boardConfig {
    .ina125ExcitationRef = INA125_SELECTED_REF,
    .ina125IARef = INA125_SELECTED_REF,
    .ina125Supply = BoardSupply::V5V,

    .LC_L_1_pin = A0,
    .LC_L_2_pin = A1,
    .LC_R_1_pin = A2,
    .LC_R_2_pin = A3
};

#else
#error "No platform selected for BoardConfig"
#endif
// =================================

constexpr float refVoltage(INA125Ref ref) {
    switch (ref) {
        case INA125Ref::Bandgap_1V24:
            return 1.24f;
        case INA125Ref::Ref_2V5:
            return 2.5f;
    }

    return 0.0f;
}

constexpr float ina125SupplyVoltage(BoardSupply supply) {
    switch (supply) {
        case BoardSupply::V3V3:
            return 3.3f;
        case BoardSupply::V5V:
            return 5.0f;
    }

    return 0.0f;
}

constexpr bool uses2V5Reference =
    boardConfig.ina125ExcitationRef == INA125Ref::Ref_2V5 ||
    boardConfig.ina125IARef == INA125Ref::Ref_2V5;

struct INA125UParams {
    static constexpr float gainR = 470.0f;
    static constexpr float ampGain = 4.0f + 60000.0f / gainR;

    static constexpr float IAref = refVoltage(boardConfig.ina125IARef);
    static constexpr float Vexc  = refVoltage(boardConfig.ina125ExcitationRef);
};

static_assert(
    boardConfig.ina125Supply == BoardSupply::V5V || !uses2V5Reference,
    "INA125U must be powered from +5V to use the +2.5V internal reference"
);
static_assert(INA125UParams::gainR > 0.0f, "Gain resistor must be positive");
static_assert(INA125UParams::ampGain > 0.0f, "Gain value must be positive");