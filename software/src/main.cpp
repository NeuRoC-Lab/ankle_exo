//
// Created by Oscar Tesniere on 17/07/2026.
// Integration script for interfacing both the motor, the
//

// This script will be used for the one-leg setup testing

// Here the Teensy will communicate with the Arduino through SPI as a temporary fix

// Using the UART on the Arduino
#include <Arduino.h>
#include "MotorConfig.h"
#include "Encoder.h"
#include "Board.h"
#include <ArduinoJson.h>
#include "SerialConfig.h"

#if defined(PLATFORM_RENESAS_RA)

Encoder encoders(false, true); // left enabled, right disabled

void setup()
{
    //Serial.begin(115200);
    delay(1000);

    encoders.begin();
    Serial1.begin(57600);
}
void loop()
{
    EncoderPositions positions = encoders.getPositions();

    Serial.print("Left encoder position : " );
    Serial.print(positions.left_position);
    Serial.print("Right encoder position : " );
    Serial.println(positions.right_position);

    Serial1.write(
        reinterpret_cast<const uint8_t*>(&positions),
        sizeof(positions)
    );

    delay(10);
}

#elif defined(PLATFORM_TEENSY41)

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
constexpr byte MOTOR_ID = 0x02;

CANMotorMIT_Teensy motor(MOTOR_ID, &motorParams,&motorSoftwareConstraints,&motorRunningConstraints,motor1Cmd,5);//NO_MOTOR_UPDATE);

#include "SerialMotorControl.h"
SerialMotorControl serialControl(Serial, motor1Cmd, motor);

JsonDocument createTelemetryPacket() {
    JsonDocument doc;

    doc[TelemetryKey::LeftLoadCell1] = LC_L_1.rawVoltage();
    doc[TelemetryKey::LeftLoadCell2] = LC_L_2.rawVoltage();
    doc[TelemetryKey::RightLoadCell1] = LC_R_1.rawVoltage();
    doc[TelemetryKey::RightLoadCell2] = LC_R_2.rawVoltage();

    doc[TelemetryKey::LeftEncoder] = positions.left_position;
    doc[TelemetryKey::RightEncoder] = positions.right_position;

    JsonArray motors =
        doc[TelemetryKey::Motors].to<JsonArray>();

    JsonObject motor1Object = motors.add<JsonObject>();
    motor.writeReplyToJson(motor1Object);
    // only one motor for now

    //JsonObject motor2Object = motors.add<JsonObject>();
    //motor2.writeReplyToJson(motor2Object);

    return doc;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing Serial communication with the Arduino UNO R4");

    Serial8.begin(57600); // using Serial 8 here, not serial 1
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
// switching to newline-delimited JSON (NDJSON) as it is easier to retrieve data later on python

    if (Serial8.available() >= sizeof(positions))
    {
        Serial8.readBytes(
            reinterpret_cast<char*>(&positions),
            sizeof(positions)
        );
// update the position object
        /*
        Serial.print("LENC:");
        Serial.print(positions.left_position);
        Serial.print("\t");

        Serial.print("RENC:");
        Serial.println(positions.right_position);
        Serial.print("\t");
        */

        // reinterpret the position from the serialized struct
    }
    motor.update();
    serialControl.update();
    delay(10);
    /*
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
    */

    JsonDocument doc = createTelemetryPacket();
    serializeJson(doc, Serial);
    //Serial.println(positions.right_position);
    //delay(500);
    Serial.write('\n');

}


#endif

