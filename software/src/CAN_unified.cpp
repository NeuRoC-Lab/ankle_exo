//
// Created by Oscar Tesniere on 23/06/2026.
//
#include "CANMotor.h"
#include "SerialMotorControl.h"

MotorCmd cmd {
    .can_id = 1,
    .position = 0.0f,
    .velocity = 10.0f,
    .torque = 0.0f,
    .kp = 0.5f,
    .kd = 0.5f
};

constexpr byte MOTOR_ID = 0x01;

#if defined(PLATFORM_TEENSY41)
using CANController = CANMotor_Teensy;
#elif defined(PLATFORM_RENESAS_RA)
using CANController = CANMotor_Renesas;
#endif
CANController motor(MOTOR_ID, &motorParams);

SerialMotorControl serialControl(Serial, cmd, motor);

unsigned long update = millis();

void setup() {
    Serial.begin(115200);
    Serial.println("Starting script to interface motor over CAN in MIT mode");
    delay(1000);

    motor.begin();
    if(!motor.resetMotor(MOTOR_ID)){
    Serial.println("Failed to start motor");
    }
    else {
    Serial.println("Successfully started motor");
    }
}

void loop() {
    /*
    MotorCmd cmd;

    cmd.can_id = 1;
    cmd.position = 0.0f;
    cmd.velocity = 10.0f;
    cmd.kp = 0.5f;
    cmd.kd = 0.5f;
    */

    /*
    if(millis() - update > 1000){
    if(cmd.torque == 0.1f){
        cmd.torque = 0.0f;
    }
    else{
        cmd.torque = 0.1f;
    }
    update = millis();
    }
    */
    motor.update(cmd);
    serialControl.update();
    //Serial.println("Updating command");

    MotorReply reply;
    while (motor.readMessages(reply)) {
        //motor.print_can_msg(reply);
    }
    delay(10);

}