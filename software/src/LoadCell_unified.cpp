//
// Created by Oscar Tesniere on 23/06/2026.
//

#include "LoadCell.h"
#include "Arduino.h"


#if defined(PLATFORM_TEENSY41)
using LoadCellController = LoadCell_Teensy41;
#elif defined(PLATFORM_RENESAS_RA)
using LoadCellController = LoadCell_Renesas;
#elif defined(PLATFORM_ATMEL_AVR)
using LoadCellController = LoadCell_AtmelAVR;
#elif !defined(PLATFORM_NORDIC)
#error "No platform selected: define PLATFORM_TEENSY41, PLATFORM_RENESAS_RA, or PLATFORM_ATMEL_AVR (or PLATFORM_NORDIC)"
#endif

#if defined(PLATFORM_NORDIC)

#include "LoadCell.h"

LoadCell_NanoBLE leftLoadCell1(
    NanoBLE33LoadCellADC::Channel::Left1
);

LoadCell_NanoBLE leftLoadCell2(
    NanoBLE33LoadCellADC::Channel::Left2
);

LoadCell_NanoBLE rightLoadCell1(
    NanoBLE33LoadCellADC::Channel::Right1
);

LoadCell_NanoBLE rightLoadCell2(
    NanoBLE33LoadCellADC::Channel::Right2
);

void setup()
{
    Serial.begin(115200);

    while (!Serial && millis() < 5000)
    {
    }

    leftLoadCell1.initialize();
    leftLoadCell2.initialize();
    rightLoadCell1.initialize();
    rightLoadCell2.initialize();

    Serial.println("Keep all load cells unloaded.");

    leftLoadCell1.calibrateOffset();
    leftLoadCell2.calibrateOffset();
    rightLoadCell1.calibrateOffset();
    rightLoadCell2.calibrateOffset();
}

void loop()
{
    const float left1Force = leftLoadCell1.voltageToN();
    const float left2Force = leftLoadCell2.voltageToN();
    const float right1Force = rightLoadCell1.voltageToN();
    const float right2Force = rightLoadCell2.voltageToN();

    Serial.print("L1: ");
    Serial.print(left1Force, 2);

    Serial.print(" N, L2: ");
    Serial.print(left2Force, 2);

    Serial.print(" N, R1: ");
    Serial.print(right1Force, 2);

    Serial.print(" N, R2: ");
    Serial.print(right2Force, 2);

    Serial.println(" N");

    delay(100);
}

#else

LoadCellController LC_L_1(boardConfig.LC_L_1_pin);
LoadCellController LC_L_2(boardConfig.LC_L_2_pin);
LoadCellController LC_R_2(boardConfig.LC_R_2_pin);
LoadCellController LC_R_1(boardConfig.LC_R_1_pin);

void setup()
    {
    #if defined(PLATFORM_TEENSY41) && HW_VERSION_AT_LEAST(1, 1, 0) //TODO remove that for v >1.1.0
    Serial.println("[Startup] checking the INA125U reference voltages...");
    delay(1000);
    check_excitation_voltages();
    #endif
    Serial.begin(115200);
    Serial.println("Starting script to interface load cells over Serial with uniformized script");
    delay(1000);

    LC_L_1.initialize();
    LC_L_2.initialize();
    LC_R_2.initialize();
    LC_R_1.initialize();

}

void loop() {
    Serial.print("L1:");
    Serial.print(LC_L_1.rawVoltage());
    Serial.print("\t");

    Serial.print("L2:");
    Serial.print(LC_L_2.rawVoltage());
    Serial.print("\t");

    Serial.print("R1:");
    Serial.print(LC_R_1.rawVoltage());
    Serial.print("\t");

    Serial.print("R2:");
    Serial.println(LC_R_2.rawVoltage());

    delay(100);
}
#endif