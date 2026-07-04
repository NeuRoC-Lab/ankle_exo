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
#else
#error "No platform selected: define PLATFORM_TEENSY41, PLATFORM_RENESAS_RA, or PLATFORM_ATMEL_AVR"
#endif

LoadCellController LC_L_1(boardConfig.LC_L_1_pin);
LoadCellController LC_L_2(boardConfig.LC_L_2_pin);
LoadCellController LC_R_2(boardConfig.LC_R_2_pin);
LoadCellController LC_R_1(boardConfig.LC_R_1_pin);

void setup()
    {
    #if defined(PLATFORM_TEENSY41) && HW_VERSION_AT_LEAST(1, 1, 0)
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
    Serial.print("Left Load cell 1 force (N) : ");
    Serial.print(LC_L_1.voltageToN());
    Serial.print("Left Load cell 2 force (N) : ");
    Serial.print(LC_L_2.voltageToN());
    Serial.print("Right Load cell 1 force (N) : ");
    Serial.print(LC_R_1.voltageToN());
    Serial.print("Right Load cell 2 force (N) : ");
    Serial.println(LC_R_2.voltageToN());
    delay(100);
}