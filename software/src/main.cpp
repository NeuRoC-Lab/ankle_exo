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



#if defined(PLATFORM_TEENSY41)

// Teensy-only CAN includes
#include "MotorConfig.h"
#include "LoadCell.h"
#include <ArduinoJson.h>



#elif defined(PLATFORM_NORDIC)

// Nano-only includes
#include <ArduinoBLE.h>
#include <ArduinoJson.h>

#endif


// TEENSY SPECIFIC CONFIGURATION



// LOAD CELL CONFIGURATION
constexpr uint8_t loadCellCount = 4;

// Load Cell configuration in version up to (and including) 1.1.0 and past (but including) 1.1.1
#if defined(PLATFORM_TEENSY41) && HW_VERSION_AT_MOST(1,1,0)

INA125UParams inaParams{};
LoadCellParams loadCellParams{};

LoadCell_Teensy41 LC_L_1(
    inaParams,
    loadCellParams,
    boardConfig.LC_L_1_Vo
);

LoadCell_Teensy41 LC_L_2(
    inaParams,
    loadCellParams,
    boardConfig.LC_L_2_Vo
);

LoadCell_Teensy41 LC_R_1(
    inaParams,
    loadCellParams,
    boardConfig.LC_R_1_Vo
);

LoadCell_Teensy41 LC_R_2(
    inaParams,
    loadCellParams,
    boardConfig.LC_R_2_Vo
);

LoadCell* loadCells[loadCellCount] = {
    &LC_L_1,
    &LC_L_2,
    &LC_R_1,
    &LC_R_2
};

float forceBuffer[loadCellCount]{};

LoadCellHandler_Teensy41 loadCellController(
    loadCells,
    loadCellCount,
    forceBuffer
);

#elif defined(PLATFORM_NORDIC) && HW_VERSION_AT_LEAST(1,1,1)
#pragma message "Enabling Arduino Nano ADC"
#include "LoadCell.h"
INA125UParams inaParams{};
LoadCellParams loadCellParams{};

LoadCell_NanoBLE LC_L_1(
    inaParams,
    loadCellParams,
    LoadCellId::Left1,
    0
);

LoadCell_NanoBLE LC_L_2(
    inaParams,
    loadCellParams,
    LoadCellId::Left2,
    1
);

LoadCell_NanoBLE LC_R_1(
    inaParams,
    loadCellParams,
    LoadCellId::Right1,
    2
);

LoadCell_NanoBLE LC_R_2(
    inaParams,
    loadCellParams,
    LoadCellId::Right2,
    3
);

LoadCell* loadCells[loadCellCount] = {
    &LC_L_1,
    &LC_L_2,
    &LC_R_1,
    &LC_R_2
};

float forceBuffer[loadCellCount]{};

LoadCellHandler_NanoBLE loadCellController(
    loadCells,
    loadCellCount,
    forceBuffer
);

#endif


//NOTE : only two load cells are actually used. Remove the two extras when testing (after identifying which is which)




/*
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
*/

// TEENSY CODE

#if defined(PLATFORM_TEENSY41)

using CANController = CANMotorMIT_Teensy;

EncoderPositions positions;

MotorCmd motor1Cmd {
    .position = 0.0f,
    .velocity = 0.0f,
    .torque = 0.0f,
    .kp = 0.0f,
    .kd = 0.0f
};
// to change running and software constraints head to CANMotorMIT.h

Encoder encoders(true, false);

constexpr byte MOTOR_ID = 0x02;

CANMotorMIT_Teensy motor(MOTOR_ID, &motorParams,&motorSoftwareConstraints,&motorRunningConstraints,motor1Cmd,5);//NO_MOTOR_UPDATE);

#include "SerialMotorControl.h"
SerialMotorControl serialControl(Serial, motor1Cmd, motor);

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
     #if HW_VERSION_AT_MOST(1,1,0)
    Serial.println("Initializing Load Cell Controller (PCB version v1.1.0 and -)");
    loadCellController.begin();

    Serial.println(
        "Keep all load cells unloaded during calibration."
    );

    delay(1000);

    loadCellController.calibrateAllOffsets(100);
    Serial.println("Calibrated Load Cell offset");
    #endif
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

        #if HW_VERSION_AT_MOST(1,1,0)
        // versions before and up to v1.1.0 : sample the load cells from the Teensy
        const float* forces = loadCellController.sampleAll();
        LoadCellVoltages loadCells {
            forces[0],
            forces[1],
            forces[2],
            forces[3],
        };
        #else
        // versions above (and including) v1.1.1 : leave it up to the Arduino Nano to populate the load cell voltages
        LoadCellVoltages loadCells {
            0.0f,
            0.0f,
            0.0f,
            0.0f,
        };
        #endif

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
    #if HW_VERSION_AT_LEAST(1,1,1)
    Serial.println("Initializing Arduino Nano LoadCell Handler (PCB v1.1.1+)");
    loadCellController.begin();
    loadCellController.calibrateAllOffsets(100);
    Serial.println("Do not move the load cells ! Calibration in progress");
    delay(1000);
    #endif
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
    #if HW_VERSION_AT_LEAST(1,1,1)
    // version v1.1.1 and onwards : the Arduino Nano is the one sampling the load cell voltages so it directly modifies the payload object sent from the Teensy before posting it on BLE
    const float* forces = loadCellController.sampleAll();

    payload.loadCells = {
        forces[0],
        forces[1],
        forces[2],
        forces[3],
    };
    #endif

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

