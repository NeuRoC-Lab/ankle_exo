#include <Arduino.h>
#include "Encoder.h"
#include "Board.h"

#if (defined(PLATFORM_TEENSY41) && HW_VERSION_AT_LEAST(1, 0, 1)) || \
defined(PLATFORM_RENESAS_RA) || \
defined(PLATFORM_ATMEL_AVR)

Encoder encoders(true, true); // left=true, right=true

void printPosition(const char* label, uint16_t pos)
{
    Serial.print(label);

    if (pos == INVALID_ENCODER_POSITION)
    {
        Serial.print("INVALID");
    }
    else
    {
        Serial.print(pos);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("Tester script for the two encoders");

    encoders.begin();
    delay(1000);
}

void loop()
{
    delay(100);

    EncoderPositions positions = encoders.getPositions();

    printPosition("Left encoder position: ", positions.left_position);
    Serial.print("\t");
    printPosition("Right encoder position: ", positions.right_position);
    Serial.println();
}
#else
#error "You need at least board rev v1.0.1 in order to use the SPI on the Teensy. The version v1.0.0 does not support SPI for teensy, you must use an Arduino Uno R4 or R3 for that"
#endif