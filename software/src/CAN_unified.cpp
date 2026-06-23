//
// Created by Oscar Tesniere on 23/06/2026.
//
#include "CANMotor.h"
#include "SerialMotorControl.h"

MotorCmd cmd {
    .can_id = 1,
    .position = 0.0f,
    .velocity = 0.0f,
    .torque = 0.0f,
    .kp = 0.0f,
    .kd = 0.0f
};

SerialMotorControl serialControl(Serial, cmd);

constexpr byte MOTOR_ID = 0x01;

#if defined(PLATFORM_TEENSY41)
CANMotor_Teensy motor(MOTOR_ID, &motorParams);
#elif defined(PLATFORM_RENESAS_RA)
CANMotor_Renesas motor(MOTOR_ID, &motorParams);
#endif

void setup() {
    Serial.begin(115200);
    Serial.println("Starting script to interface motor over CAN in MIT mode");
    delay(1000);

    motor.begin();
    motor.resetMotor(1);
}

void loop() {
    /*
    MotorCmd cmd;

    cmd.can_id = 1;
    cmd.position = 0.0f;
    cmd.velocity = 0.0f;
    cmd.kp = 0.5f;
    cmd.kd = 0.5f;
    cmd.torque = 0.0f;
    */

    serialControl.update();
    motor.update(cmd);

    MotorReply reply;

    while (motor.readMessages(reply)) {
        motor.print_can_msg(reply);
    }

    delay(10);
}