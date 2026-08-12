//
// Created by Oscar Tesniere on 11/08/2026.
//


#include <functional>
#include <Arduino.h>
#include "board.h"
#include "MessageBus.h"
#include "Driver.h"
#include "BLE.h"
#include "MotorControllers.h"
#include "Scheduler.h"

Scheduler<12> scheduler;

// UART bridge to Arduino Nano
UARTHandler uart(
    board::teensy41::nanoUart
);

//TODO move MessagebUS AND Topics declaration to a "commons.h" to clear up main.cpp

// Messagebus to communicate with Nano
MessageBus messageBus(uart);

//*** TOPIC INITIALISATION **//

Topic<EncoderPositions> encoderTopic;
Topic<LoadCellForces> loadCellTopic;
Topic<MotorReply> leftMotorTopic;
Topic<PowerReadings> ina232Topic;
Topic<MotorCmd> leftMotorCommandTopic;
Topic<MotorMetaCommand> leftMotorMetaCommandTopic;

//*** ENCODER INITIALISATION **//

EncoderDriver encoderDriver;

PollingPublisher<
    EncoderDriver,
    EndpointId::EncoderSnapshot,
    5'000
> encoderPublisher(
    encoderDriver,
    messageBus
);

//** Power monitoring (INA232) Initialisation **//

INA232Driver ina232Driver;

// Recuring publishers for local teensy peripherals (encoder, power, CAN)
PollingPublisher<
    INA232Driver,
    EndpointId::Ina232Snapshot,
    10'000
> ina232Publisher(
    ina232Driver,
    messageBus
);

//** CAN BUS definitions **//

CanBus canBus;

constexpr uint8_t LEFT_MOTOR_CAN_ID = 0x02;
constexpr uint8_t RIGHT_MOTOR_CAN_ID = 0x03;

AK60Params leftMotorLimits = MotorParams;

MotorDriver leftMotor(
    LEFT_MOTOR_CAN_ID,
    leftMotorLimits,
    canBus
);

//TODO implement second motor (right)
/*
MotorDriver rightMotor(
    RIGHT_MOTOR_CAN_ID,
    rightMotorLimits,
    canBus
);
*/

MotorCommandTask leftMotorCommandTask(
    leftMotor,
    leftMotorCommandTopic
);

MotorCanReceiver motorReceiver(
    canBus,
    leftMotor,
    //rightMotor,
    messageBus
);

MotorMetaCommandTask
    leftMotorMetaCommandTask(
        leftMotor,
        leftMotorMetaCommandTopic,
        leftMotorCommandTask
);

/* DEBUG : prints sensor and motor data to Serial continuously
DummyController dummyController(
    encoderTopic,
    loadCellTopic,
    leftMotorTopic,
    ina232Topic
);
*/

JointLimitController jointController(
    encoderTopic,
    loadCellTopic,
    leftMotorTopic,
    leftMotorCommandTopic,
    leftMotorMetaCommandTopic
);

TransparentModeController transparentModeController(
    encoderTopic,
    loadCellTopic,
    leftMotorTopic,
    leftMotorCommandTopic
);

void setup()
{
    Serial.begin(230400);
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 3000)) {
        // wait
    }
    Serial.println("Teensy : beginning peripherals initialization");

    board::teensy41::nanoUart.begin(230400);

    if (!encoderDriver.begin())
    {
    Serial.println("Encoder initialization failed");
        while (true) {}
    }

    if (!canBus.begin())
    {
    Serial.println("CAN initialization failed");
    while(true){}
    }
    if (!leftMotor.begin())
    {
    Serial.println("Left motor initialization failed");
    while (true) {}
    }
    if (!ina232Driver.begin())
    {
    Serial.println("INA232 initialization failed");
    while (true) {}
    }
    //TODO initialise right motor later

    messageBus.addTopic<EndpointId::EncoderSnapshot>(encoderTopic);
    messageBus.addTopic<EndpointId::LoadCellSnapshot>(loadCellTopic);
    messageBus.addTopic<EndpointId::LeftMotorSnapshot>(leftMotorTopic);
    messageBus.addTopic<EndpointId::Ina232Snapshot>(ina232Topic);
    messageBus.addTopic<EndpointId::LeftMotorCommand>(leftMotorCommandTopic);
    messageBus.addTopic<EndpointId::LeftMotorMetaCommand>(leftMotorMetaCommandTopic);
    messageBus.begin();

    scheduler.add(messageBus);
    scheduler.add(encoderPublisher);
    scheduler.add(ina232Publisher);
    scheduler.add(motorReceiver);
    scheduler.add(transparentModeController);
    scheduler.add(jointController);
    scheduler.add(leftMotorCommandTask);
    scheduler.add(leftMotorMetaCommandTask);
    Serial.println("Teensy: ready");
}

void loop()
{
    scheduler.run(micros());
}

