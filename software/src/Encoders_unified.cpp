//
// Created by Oscar Tesniere on 23/06/2026.
//
#include <Arduino.h>
#include "Encoder.h"
#include "Board.h"

Encoder left_encoder(false,true); // using only the left encoder

void setup() {
Serial.begin(115200);
Serial.println("Tester script for the two encoders");
left_encoder.begin();
delay(1000);
}

void loop() {
EncoderPositions positions = left_encoder.getPositions();
Serial.print("Left encoder position : " );
Serial.print(positions.left_position);
Serial.print("Right encoder position : " );
Serial.println(positions.right_position);
}
