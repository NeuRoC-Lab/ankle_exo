//
// Created by Oscar Tesniere on 17/07/2026.
// Integration script for interfacing both the motor, the
//

// This script will be used for the one-leg setup testing

// Here the Teensy will communicate with the Arduino through SPI as a temporary fix

// Using the UART on the Arduino

#if defined(PLATFORM_RENESAS_RA)
#include "Encoder.h"
// upload the Arduino Uno R4 code here
#include "Board.h"

Encoder encoders(true, false); // left enabled, right disabled

void setup(){
encoders.begin();
Serial1.begin(1000000); //1Mbaud
// periodically send the encoder positions on Serial
EncoderPositions positions = encoders.getPositions();
// then send data over serial. Should we encode in binary for faster communication ?
}
void loop() {
    Serial1.write(
            reinterpret_cast<uint8_t*>(&positions),
            sizeof(positions)
        );
    delay(10);
}

#elif defined(PLATFORM_TEENSY41)
#include <Arduino.h>
#include "Encoder.h"
#include "CANMotorMIT.h"
#include "LoadCell.h"
using CANController = CANMotorMIT_Teensy;
using LoadCellController = LoadCell_Teensy41;

LoadCellController LC_L_1(boardConfig.LC_L_1_pin);
LoadCellController LC_L_2(boardConfig.LC_L_2_pin);
LoadCellController LC_R_2(boardConfig.LC_R_2_pin);
LoadCellController LC_R_1(boardConfig.LC_R_1_pin);
//NOTE : only two load cells are actually used. Remove the two extras when testing (after identifying which is which)

EncoderPositions positions;

MotorCmd motor1Cmd {
    .position = 0.0f,
    .velocity = 0.0f,
    .torque = 0.0f,
    .kp = 0.0f,
    .kd = 0.0f
};
// define SOFT stop values here
constexpr AK60Params motorSoftwareConstraints = {

    -8.5f, 8.5f,   // position (rad)
    -20.0f,  20.0f,   // velocity
    -2.0f,  2.0f    // torque
};

constexpr AK60Params motorRunningConstraints = {

    -10.0f, 10.0f,   // position (rad)
    -25.0f,  25.0f,   // velocity
    -4.0f,  4.0f    // torque
};

static_assert(
    constraintsInside(motorSoftwareConstraints, motorParams),
    "Software constraints exceed nominal motor limits"
);

static_assert(
    constraintsInside(motorRunningConstraints, motorParams),
    "Running constraints exceed nominal motor limits"
);

static_assert(
    constraintsInside(
        motorSoftwareConstraints,
        motorRunningConstraints
    ),
    "Software constraints must be inside running constraints"
);
constexpr byte MOTOR_ID = 0x01;

CANMotorMIT_Teensy motor(MOTOR_ID, &motorParams,&motorSoftwareConstraints,&motorRunningConstraints,motor1Cmd,5);//NO_MOTOR_UPDATE);

#include "SerialMotorControl.h"
SerialMotorControl serialControl(Serial, motor1Cmd, motor);

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing Serial communication with the Arduino UNO R4");

    Serial1.begin(1000000);
    Serial.println("Initializing the motor in MIT mode");
    motor.begin();
    if(!motor.resetMotor()){
        Serial.println("Failed to start motor");
    }
    else {
        Serial.println("Successfully started motor");
    }
    Serial.println("Initializing Load Cells");

    LC_L_1.initialize();
    LC_L_2.initialize();
    LC_R_2.initialize();
    LC_R_1.initialize();
}

void loop()
{
    if (Serial1.available() >= sizeof(positions))
    {
        Serial1.readBytes(
            reinterpret_cast<char*>(&positions),
            sizeof(positions)
        );

        Serial.print("LeftEncoder:");
        Serial.print(positions.left_position);
        Serial.print("\t");

        Serial.print("RightEncoder:");
        Serial.println(positions.right_position);
        Serial.print("\t");
        // reinterpret the position from the serialized struct
    }
    motor.update();
    serialControl.update();

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

    delay(10);
}


#endif

