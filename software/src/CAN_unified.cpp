//
// Created by Oscar Tesniere on 23/06/2026.
//
#include <Arduino.h>

// AK60 Motor parameter definitions


unsigned long debug_update = millis();

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

constexpr int NO_MOTOR_UPDATE = -1;

#if defined(MIT_MODE)

constexpr byte MOTOR_ID = 0x01; // 0x02 for the one mounted ; 0x01 for the one that is free

MotorCmd motor1Cmd {
    .position = 0.0f,
    .velocity = 0.0f,
    .torque = 0.0f,
    .kp = 0.0f,
    .kd = 0.0f
};

CANController motor(MOTOR_ID, &motorParams,&motorConstraints,motor1Cmd,5);//NO_MOTOR_UPDATE);

#elif defined(SERVO_MODE)

constexpr byte MOTOR_ID = 0x68; //decimal 104

MotorCmd motor1Cmd {
    .packetID = CAN_PACKET_SET_DUTY,
    .data = {0},
    .len = 4
};

CANController motor(MOTOR_ID, &motorParams,motor1Cmd,10);//NO_MOTOR_UPDATE);

#endif


#include "SerialMotorControl.h" // only include that library ONCE we defined the CANController object



SerialMotorControl serialControl(Serial, motor1Cmd, motor);

void setup() {
    //pinMode(29,OUTPUT); // this is temporarily used as a OE pin for TXU0304

    Serial.begin(115200);
      // from https://www.pjrc.com/store/teensy41.html#analog :
      // 18 pins can be used an analog inputs, for reading sensors or other analog signals. Basic analog input is done with the analogRead function. The default resolution is 10 bits (input range 0 to 1023), but can be adjusted with analogReadResolution. The hardware allows up to 12 bits of resolution, but in practice only up to 10 bits are normally usable due to noise. More advanced use is possible with the ADC library.
    analogReadResolution(12); // enable 12 bit ADC resolution because the Teensy defaults otherwise to 10 bits
    Serial.println("Starting script to interface motor over CAN in MIT mode");
    delay(1000);

    //digitalWrite(29,HIGH);

    motor.begin();
    if(!motor.resetMotor()){
    Serial.println("Failed to start motor");
    }
    else {
    Serial.println("Successfully started motor");
    }
}

bool toturnOn;
unsigned long update = millis();
void loop() {
  /*
    motor1Cmd.position = 0.0f;
    motor1Cmd.velocity = 10.0f;
    motor1Cmd.kp = 0.5f;
    motor1Cmd.kd = 0.5f;
    // TESTER CODE FOR THE SERVO
    /*
    for(int i = 1000 ; i < 3000;i=i+100){
        motor.can_set_rpm(i);
        motor.update();
    delay(100);
    }
    for(int i = 3000;i>1000;i=i-10){
        motor.can_set_rpm(i);
        motor.update();
        delay(100);
    }
*/
    //motor.can_set_rpm(1000);
    //motor.update();
    //serialControl.update();


    delay(10); // this delay is NECESSARY OTHERWISE COMMAND UPDATES DONT SHOW UP
    //TODO understand why that is necessary
    /*
    if(millis() - debug_update > 1000){
    Serial.print("Velocity");
    Serial.println(motor.m_cmd.velocity);
    Serial.print("Kd");
    Serial.println(motor.m_cmd.kd);
    debug_update = millis();
    }
    */
    motor.update();
    serialControl.update();
    delay(10);

    if(millis() - update > 1000){
    // Serial.print("AMP L1 : ");
    // Serial.print(analogRead(24)*3.3f/4095.0f);
    // Serial.print(" AMP L2 : ");
    // Serial.print(analogRead(25)*3.3f/4095.0f);
    // Serial.println();
    update = millis();
    }





}