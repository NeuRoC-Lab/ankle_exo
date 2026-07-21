//
// Created by Oscar Tesniere on 17/07/2026.
// Integration script for interfacing both the motor, the
//

// This script will be used for the one-leg setup testing

// Here the Teensy will communicate with the Arduino through SPI as a temporary fix

#include <Arduino.h>
#include "Board.h"
#include "Encoder.h"
#include "CANMotorMIT.h"
#include "SerialProtocol.h"

#if defined(PLATFORM_RENESAS_RA)

// Renesas-only includes
#include "Encoder.h"

#elif defined(PLATFORM_TEENSY41)

// Teensy-only CAN includes
#include "MotorConfig.h"
#include "LoadCell.h"
#include <ArduinoJson.h>

#elif defined(PLATFORM_NORDIC)

// Nano-only includes
#include <ArduinoBLE.h>

#endif

#if defined(PLATFORM_RENESAS_RA)

Encoder encoders(true, true);

void setup()
{
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
    ); // here it would be a waste to use sendPayload() because teh Arduino UNO R4 only sends encoder positions. Also this is temporary

    delay(10);
}

#elif defined(PLATFORM_TEENSY41)


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
// to change running and software constraints head to CANMotorMIT.h

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
    // THIS IS TEMPORARY
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

    // to debug : the Teensy will stream data to your laptop through USB Serial, using ArduinoJSON for ease of unpacking
    #if defined(DEBUG)
    #pragma message "DEBUG mode is enabled, streaming data from Teensy to computer with ArduinoJSON"
    JsonDocument doc = createTelemetryPacket();
    serializeJson(doc, Serial);
    Serial.write('\n');
    #else
    LoadCellVoltages loadCells =
    {
    LC_L_1.rawVoltage(),
    LC_L_2.rawVoltage(),
    LC_R_1.rawVoltage(),
    LC_R_2.rawVoltage(),
    };

    DataPayload payload = {
    loadCells,
    positions,
    motor.m_reply,
    };
    sendPayload(payload, Serial8); // send data from Teensy's Serial8 to the Arduino Nano's Serial over UART
    delay(10);
    #endif

}
#elif defined(PLATFORM_NORDIC)

// code for the Arduino Nano
DataPayload payload;
BLEHandler ble(payload);

void setup(){
    Serial.begin(9600);
    Serial.println("Starting Arduino Nano script");
    if(!ble.begin()){
    Serial.println("BLE Initialization failed. Aborting");
    while(1){}
    }
}

void loop(){
    while(readPayload(ble.m_payload,Serial1))
    { // Serial1 is the UART port used by the Arduino Nano
        ble.update(); // update the "bulletin board" on bluetooth
    }
}


#endif

