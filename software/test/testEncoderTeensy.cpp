//
// Created by Oscar Tesniere on 24/07/2026.
//
#if !defined(PLATFORM_TEENSY41)
#error "This must run on the Teensy 4.1"
#endif

#include <Arduino.h>
#include "Board.h"
#include "Encoder.h"

// we are using CS2, which corresponds to LEFT_ENCODER_CS in Board.h

Encoder encoders(true, false);

void setup(){
    Serial.begin(115200);
    Serial.println("Tester script for using the encoder with the new v1.1.0 board that features integrated voltage shifters");
    // OUTPUT ENABLE ON THE SHIFTERS
    pinMode(boardConfig.OE1, OUTPUT);
    pinMode(boardConfig.OE2, OUTPUT);
    digitalWrite(boardConfig.OE1,HIGH);
    digitalWrite(boardConfig.OE2,HIGH);
    delay(2000);
    encoders.begin();
    EncoderPositions positions = encoders.getPositions();
}

void loop(){
    EncoderPositions positions = encoders.getPositions();
    Serial.print("Left encoder position : " );
    Serial.println(positions.left_position);
    delay(10);
}