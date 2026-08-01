//
// Created by Oscar Tesniere on 17/07/2026.
// Integration script for interfacing both the motor, the
//

// This script will be used for the one-leg setup testing

// Here the Teensy will communicate with the Arduino through SPI as a temporary fix

#include <cstdint>
#include <cmath>

#include <Arduino.h>


#include "Board.h"
#include "SerialProtocol.h"

#if defined(PLATFORM_TEENSY41)

#include "Encoder.h"
#include "CANMotorMIT.h"
#include "MotorConfig.h"
#include "LoadCell.h"
#include <ArduinoJson.h>

#elif defined(PLATFORM_NORDIC)

#include <ArduinoBLE.h>
#include <ArduinoJson.h>

#endif


// Load Cell configuration in version up to (and including) 1.1.0 and past (but including) 1.1.1

// ==================================== LOAD CELL CONFIGURATION =========================================
#include "LoadCell.h"
#if defined(PLATFORM_TEENSY41) && HW_VERSION_AT_MOST(1,1,0)

using LoadCellObj = LoadCell_Teensy41;
using LoadCellHandlerObj = LoadCellHandler_Teensy41;


#elif defined(PLATFORM_NORDIC) && HW_VERSION_AT_LEAST(1,1,1)
#pragma message "Enabling Arduino Nano ADC"

using LoadCellObj = LoadCell_NanoBLE;
using LoadCellHandlerObj = LoadCellHandler_NanoBLE;
#endif
#if defined(PLATFORM_NORDIC) && HW_VERSION_AT_LEAST(1,1,1) || defined(PLATFORM_TEENSY41) && HW_VERSION_AT_MOST(1,1,0)
LoadCellParams loadCellParams{};

LoadCellObj LC_L_1(
    loadCellParams,
    LoadCellId::Left1
);

LoadCellObj LC_L_2(
    loadCellParams,
    LoadCellId::Left2
);

LoadCellObj LC_R_1(
    loadCellParams,
    LoadCellId::Right1
);

LoadCellObj LC_R_2(
    loadCellParams,
    LoadCellId::Right2
);

LoadCell* loadCells[loadCellCount] = {
    &LC_L_1,
    &LC_L_2,
    &LC_R_1,
    &LC_R_2
};

float forceBuffer[loadCellCount]{};

LoadCellHandlerObj loadCellController(
    loadCells,
    loadCellCount,
    forceBuffer
);
#endif


#if defined(PLATFORM_TEENSY41)

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
CANMotorMIT_Handler motorHandler(motor,nullptr);

//TODO remove that later after Motor Handler class has been validated, but keep it for debugging in one leg setup
#include "SerialMotorControl.h"
SerialMotorControl serialControl(Serial, motor1Cmd, motor);

constexpr uint32_t TELEMETRY_PERIOD_US = 5000; // 20 ms = 50 Hz ; 5ms = 200Hz
unsigned long now = millis();
unsigned long previousSend = millis();

UARTHandler_Teensy uart(Serial8,motorHandler);


void setup()
{
    uart.begin();
    Serial.println("Initializing UART communication with Nano");
    encoders.begin();

    Serial.begin(230400);
    Serial.println("Initializing Serial communication with the Arduino UNO R4");

    Serial8.begin(230400); // using Serial 8 here, not serial 1
    Serial.println("Initializing the motor in MIT mode");
    motorHandler.begin();
    //motor.sendMessage(exitMotorMode);
     #if HW_VERSION_AT_MOST(1,1,0)
    Serial.println("Initializing Load Cell Controller (PCB version v1.1.0 and -)");
    loadCellController.begin();

    Serial.println(
        "Keep all load cells unloaded during calibration."
    );

    loadCellController.calibrateAllOffsets(100);
    delay(1000);
    Serial.println("Calibrated Load Cell offset");
    #endif
}

void loop()
{
    uart.update();
    motor.update();
    serialControl.update();
    delay(10); //TODO try to remove that unless it blocks the motor control update. This will ensure our data rate is set by TELEMETRY_PERIOD_US
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

        uart.send(payload);

    }
    #endif

}
#elif defined(PLATFORM_NORDIC)

DataPayload payload {};
UARTHandler_Nano uart(Serial1,payload); //TODO ideally later on pass on reference to BLEHandler so it follows the same semantics as in UARTController uart(Serial8,motorHandler);

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

