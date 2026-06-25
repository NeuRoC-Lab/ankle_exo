//
// Created by Oscar Tesniere on 23/06/2026.
//
#include <Arduino.h>

#if defined(MIT_MODE)
#include "CANMotorMIT.h"
#elif defined(SERVO_MODE)
#include "ServoCANMotor.h"
#endif



#if defined(PLATFORM_TEENSY41)
#if defined(SERVO_MODE)
using CANController = ServoCANMotor_Teensy;
#elif defined(MIT_MODE)
using CANController = CANMotorMIT_Teensy;
#else
#error "You need to select either MIT or Servo mode for operating the motor"
#endif

#elif defined(PLATFORM_RENESAS_RA)

#if defined(SERVO_MODE)
using CANController = ServoCANMotor_Renesas;
#elif defined(MIT_MODE)
using CANController = CANMotorMIT_Renesas;
#else
#error "You need to select either MIT or Servo mode for operating the motor"
#endif
#else
#error "You need to select either PLATFORM_TEENSY41 or PLATFORM_RENESAS_RA"
#endif

// this is for motor1. CAN id is set here :

constexpr byte MOTOR_ID = 0x01;
CANController motor(MOTOR_ID, &motorParams);

#include "SerialMotorControl.h" // only include that library ONCE we defined the CANController object


#if defined(MIT_MODE)

MotorCmd motor1Cmd {
    .position = 0.0f,
    .velocity = 0.0f,
    .torque = 0.0f,
    .kp = 0.5f,
    .kd = 0.5f
};

#elif defined(SERVO_MODE)

MotorCmd motor1Cmd {
    .packetID = CAN_PACKET_SET_DUTY,
    .data = {0},
    .len = 4
};

#endif

SerialMotorControl serialControl(Serial, motor1Cmd, motor);


void setup() {
    Serial.begin(115200);
    Serial.println("Starting script to interface motor over CAN in MIT mode");
    delay(1000);

    motor.begin();
    if(!motor.resetMotor()){
    Serial.println("Failed to start motor");
    }
    else {
    Serial.println("Successfully started motor");
    }
}

void loop() {
    /*
    motor1Cmd.position = 0.0f;
    motor1Cmd.velocity = 10.0f;
    motor1Cmd.kp = 0.5f;
    motor1Cmd.kd = 0.5f;
    */

    motor.update();
    serialControl.update();
    delay(10);

}