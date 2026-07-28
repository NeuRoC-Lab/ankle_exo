//
// Created by Oscar Tesniere on 17/07/2026.
// Integration script for interfacing both the motor, the
//

// This script will be used for the one-leg setup testing

// Here the Teensy will communicate with the Arduino through SPI as a temporary fix

#include <Arduino.h>
#include "Board.h"
#include "CANMotorMIT.h"
#include "Encoder.h"
#include "SerialProtocol.h"



#if defined(PLATFORM_RENESAS_RA)

// Renesas-only includes

#elif defined(PLATFORM_TEENSY41)

// Teensy-only CAN includes
#include "MotorConfig.h"
#include "LoadCell.h"
#include <ArduinoJson.h>



#elif defined(PLATFORM_NORDIC)

// Nano-only includes
#include <ArduinoBLE.h>
#include <ArduinoJson.h>

#endif


#if defined(PLATFORM_RENESAS_RA)

Encoder encoders(true, false);

void setup()
{
    Serial.begin(230400);
    encoders.begin();
    Serial1.begin(230400);
}
void loop()
{
    EncoderPositions positions = encoders.getPositions();

    Serial.print("Left encoder position : " );
    Serial.print(positions.left_position);
    Serial.print("Right encoder position : " );
    Serial.println(positions.right_position);

    /*
    Serial1.write(
        reinterpret_cast<const uint8_t*>(&positions),
        sizeof(positions)
    ); // here it would be a waste to use sendPayload() because teh Arduino UNO R4 only sends encoder positions. Also this is temporary
    */
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



Encoder encoders(true, false);

constexpr uint32_t TELEMETRY_PERIOD_US = 5000; // 20 ms = 50 Hz ; 5ms = 200Hz
unsigned long now = millis();
unsigned long previousSend = millis();

void handleMotorCommand(
    void* context,
    const CommandPayload& command
)
{
    auto* motor = static_cast<CANMotorMIT*>(context);

    if (motor != nullptr) {
        motor->handleSerialCommand(command);
    }
}

UARTHandler uart(
    Serial8,
    handleMotorCommand,
    &motor
);

void setup()
{
    // encoder setup
    pinMode(boardConfig.OE1, OUTPUT);
    pinMode(boardConfig.OE2, OUTPUT);
    digitalWrite(boardConfig.OE1,HIGH);
    digitalWrite(boardConfig.OE2,HIGH);
    delay(2000);
    uart.begin();
    Serial.println("Initializing UART communication with Nano");
    encoders.begin();

    Serial.begin(230400);
    Serial.println("Initializing Serial communication with the Arduino UNO R4");

    Serial8.begin(230400); // using Serial 8 here, not serial 1
    Serial.println("Initializing the motor in MIT mode");
    motor.begin();
    if(!motor.resetMotor()){
        Serial.println("Failed to start motor");
    }
    else {
        Serial.println("Successfully started motor");
    }
    Serial.println("Stopping motor for now");
    //motor.sendMessage(exitMotorMode);
    Serial.println("Initializing Load Cells");

    LC_L_1.initialize();
    LC_L_2.initialize();
    LC_R_2.initialize();
    LC_R_1.initialize();
    delay(100);
    LC_L_1.calibrateOffset();
    LC_L_2.calibrateOffset();
    LC_R_1.calibrateOffset();
    LC_R_2.calibrateOffset();
    Serial.println("Calibrated Load Cell offset");
}

void loop()
{
    uart.update();
    motor.update();
    serialControl.update();
    delay(10);
    static uint32_t previousSend = 0;
    const uint32_t now = micros();

    // to debug : the Teensy will stream data to your laptop through USB Serial, using ArduinoJSON for ease of unpacking
    #if defined(DEBUG)
    #pragma message "DEBUG mode is enabled, streaming data from Teensy to computer with ArduinoJSON"
    JsonDocument doc = createTelemetryPacket();
    serializeJson(doc, Serial);
    Serial.write('\n');
    #else
    if (now - previousSend >= TELEMETRY_PERIOD_US) {
        previousSend = now;

        EncoderPositions positions = encoders.getPositions();

        LoadCellVoltages loadCells {
            LC_L_1.voltageToN(),
            LC_L_2.voltageToN(),
            LC_R_1.voltageToN(),
            LC_R_2.voltageToN(),
        };

        DataPayload payload {
            loadCells,
            positions,
            motor.m_reply,
        };

        uart.sendTelemetryPacket(payload);

    }
    #endif

}
#elif defined(PLATFORM_NORDIC)

unsigned long lastStatus = 0;


JsonDocument createTelemetryPacket(const DataPayload& payload) {
    JsonDocument doc;

    doc[TelemetryKey::LeftLoadCell1] = payload.loadCells.LeftLoadCell1;
    doc[TelemetryKey::LeftLoadCell2] = payload.loadCells.LeftLoadCell2;
    doc[TelemetryKey::RightLoadCell1] = payload.loadCells.RightLoadCell1;
    doc[TelemetryKey::RightLoadCell2] = payload.loadCells.RightLoadCell2;

    doc[TelemetryKey::LeftEncoder] = payload.encoders.left_position;
    doc[TelemetryKey::RightEncoder] = payload.encoders.right_position;

    JsonArray motors =
        doc[TelemetryKey::Motors].to<JsonArray>();

    JsonObject motor1Object = motors.add<JsonObject>();
    motor1Object[TelemetryKey::MotorId] = payload.motorRep.can_id;
    motor1Object[TelemetryKey::MotorPos] = payload.motorRep.position;
    motor1Object[TelemetryKey::MotorVel] = payload.motorRep.velocity;
    motor1Object[TelemetryKey::MotorTrq] = payload.motorRep.torque;
    motor1Object[TelemetryKey::MotorTemp] = payload.motorRep.temperature;
    motor1Object[TelemetryKey::MotorErr] = payload.motorRep.error;

    return doc;
}
void printPayload(const DataPayload& payload)
{
    Serial.println("----- DataPayload -----");

    Serial.println("Load cells:");
    Serial.print("  Left 1:  ");
    Serial.println(payload.loadCells.LeftLoadCell1, 6);

    Serial.print("  Left 2:  ");
    Serial.println(payload.loadCells.LeftLoadCell2, 6);

    Serial.print("  Right 1: ");
    Serial.println(payload.loadCells.RightLoadCell1, 6);

    Serial.print("  Right 2: ");
    Serial.println(payload.loadCells.RightLoadCell2, 6);

    Serial.println("Encoders:");
    Serial.print("  Left position:  ");
    Serial.println(payload.encoders.left_position);

    Serial.print("  Right position: ");
    Serial.println(payload.encoders.right_position);

    Serial.println("Motor:");
    Serial.print("  CAN ID:      ");
    Serial.println(payload.motorRep.can_id);

    Serial.print("  Position:    ");
    Serial.println(payload.motorRep.position, 6);

    Serial.print("  Velocity:    ");
    Serial.println(payload.motorRep.velocity, 6);

    Serial.print("  Torque:      ");
    Serial.println(payload.motorRep.torque, 6);

    Serial.print("  Temperature: ");
    Serial.println(payload.motorRep.temperature);

    Serial.print("  Error:       ");
    Serial.println(payload.motorRep.error);

    Serial.println("-----------------------");
}

// code for the Arduino Nano
DataPayload payload {};

UARTHandler uart(
    Serial1,
    payload
);

BLEHandler ble(
    payload,
    uart
);

void setup(){
    //randomSeed(analogRead(A0));
    delay(1000); // add a delay because sometimes the Serial connection takes more time to be established and the arduino code has already moved on to execution of loop
    Serial.begin(230400);
    uart.begin();
    Serial1.begin(230400);
    delay(1000);

    while (Serial1.available() > 0) {
        Serial1.read();
    }
    Serial.println("Starting Arduino Nano script");
    if(!ble.begin()){
    Serial.println("BLE Initialization failed. Aborting");
    while(1){}
    }
    Serial.println("BLE initialization successful");
}

void loop(){
    /*
    WHEN NOT RUNNING THE TEENSY (I.E NANO AND COMPUTER ALONE) COMMENT THIS
    // TODO ADD A CHECK LIKE Serial1.available() to check if the teensy is actually sending valid and updated data
    while(readPayload(ble.m_payload,Serial1))
    { // Serial1 is the UART port used by the Arduino Nano
        ble.update(); // update the "bulletin board" on bluetooth
    }
    */
    // Read one complete payload from the Teensy.
    uart.update();

    #if defined(USING_BLE)
        ble.update();
    #endif

    #if defined(USING_SERIAL)
    JsonDocument doc = createTelemetryPacket(payload);
    serializeJson(doc, Serial);
    Serial.write('\n');
    #endif
}

#endif

