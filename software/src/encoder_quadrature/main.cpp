//
// Created by Oscar Tesniere on 18/06/2026.
//

// This script will attempt to read the quadrature output from the encoder
#include <Arduino.h>

const int encoderPinA = 2;

volatile bool interruptFlag = false;
volatile uint32_t interruptCount = 0;

void encoderISR() {
    interruptCount++;
    interruptFlag = true;
}

void setup() {
    Serial.begin(115200);

    pinMode(encoderPinA, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);
    // Other modes: RISING, FALLING, LOW, CHANGE
}

void loop() {
    if (interruptFlag) {
        noInterrupts();
        uint32_t countCopy = interruptCount;
        interruptFlag = false;
        interrupts();

        Serial.print("Interrupt count: ");
        Serial.println(countCopy);
    }
}