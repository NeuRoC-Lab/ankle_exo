//
// Created by Oscar Tesniere on 23/06/2026.
// Modified to compute ankle velocity using EWMA
// Double check for right encoder code is not completed
//
#include <Arduino.h>
#include "Encoder.h"
#include "Board.h"

Encoder left_encoder(false,true); // using only the left encoder
const int delay_time = 100; //ms
const float dt = delay_time / 1000.0f; //time interval in s
const float alpha = 0.15f; //EWMA Coefficient for filtering velocity, test with different values
float previous_left_position = 0.0f;
float previous_right_position = 0.0f;
float filtered_left_velocity = 0.0f;
float filtered_right_velocity = 0.0f;

void setup()
{
    Serial.begin(115200);
    Serial.println("Tester script for the two encoders");
    left_encoder.begin();
    delay(1000);

    EncoderPositions positions = left_encoder.getPositions();
    previous_left_position = positions.left_position;
}

void loop()
{
    delay(delay_time); //time interval of 0.1s between each recorded position
    //TODO : add a setter for the encoder update frequency (the Serial.print rate) must be below 20khz, the frequency at which the encoder register updates
    EncoderPositions positions = left_encoder.getPositions();
    Serial.print("Left encoder position : " );
    Serial.print(positions.left_position);
    Serial.print("Right encoder position : " );
    Serial.println(positions.right_position);

    // Calculate raw velocity using v = delta/dt
    float current_left_position = positions.left_position;
    float current_right_position = positions.right_position;

    float left_delta = current_left_position - previous_left_position;
    float right_delta = current_right_position - previous_right_position;

    float raw_left_velocity = left_delta / dt;
    float raw_right_velocity = right_delta / dt;

    Serial.print("Left raw velocity: ");
    Serial.print(raw_left_velocity);
    Serial.print("Right raw velocity: ");
    Serial.print(raw_right_velocity);

    // Apply EWMA filter
    filtered_left_velocity += alpha * (raw_left_velocity - filtered_left_velocity);
    filtered_right_velocity += alpha * (raw_right_velocity - filtered_right_velocity);

    Serial.print("Left filtered velocity: ");
    Serial.print(filtered_left_velocity);
    Serial.print(" Right filtered velocity: ");
    Serial.print(filtered_right_velocity);

    // Update previous position
    previous_left_position = current_left_position;
    previous_right_position = current_right_position;

}

// OLD CODE IN CASE I BROKE SOMETHING

// #include <Arduino.h>
// #include "Encoder.h"
// #include "Board.h"
//
// Encoder left_encoder(false,true); // using only the left encoder
//
// void setup() {
//     Serial.begin(115200);
//     Serial.println("Tester script for the two encoders");
//     left_encoder.begin();
//     delay(1000);
// }
//
// void loop() {
//     delay(100);
//     //TODO : add a setter for the encoder update frequency (the Serial.print rate) must be below 20khz, the frequency at which the encoder register updates
//     EncoderPositions positions = left_encoder.getPositions();
//     Serial.print("Left encoder position : " );
//     Serial.print(positions.left_position);
//     Serial.print("Right encoder position : " );
//     Serial.println(positions.right_position);
// }
