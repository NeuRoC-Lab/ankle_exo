#include <Arduino.h>

#include "NanoBLE33LoadCellADC.h"

NanoBLE33LoadCellADC loadCellAdc;

void setup()
{
    Serial.begin(115200);

    while (!Serial && millis() < 3000)
    {
    }

    loadCellAdc.begin();

    Serial.println("Differential ADC test");
}

void loop()
{
    const float voltage = loadCellAdc.sampleLeft1();

    Serial.print("Differential voltage: ");
    Serial.print(voltage, 6);
    Serial.println(" V");

    delay(100);
}