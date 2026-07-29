//
// Created by Oscar Tesniere on 28/07/2026.
//

#include <Arduino.h>

constexpr uint8_t dacPin = A0;

void setup()
{
    analogWriteResolution(12);

    // 1.000 V approximately, assuming a 5 V DAC full scale.
    constexpr float requestedVoltage = 1.0f;
    constexpr float dacFullScale = 5.0f;
    constexpr uint16_t dacMaximum = 4095;

    const uint16_t dacCode =
        static_cast<uint16_t>(
            requestedVoltage
            * static_cast<float>(dacMaximum)
            / dacFullScale
        );

    analogWrite(dacPin, dacCode);
}

void loop()
{
}