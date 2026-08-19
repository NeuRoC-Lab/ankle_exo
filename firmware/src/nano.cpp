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

Scheduler<14> scheduler;

UARTHandler uart(
    board::nano::teensyUart
);

MessageBus messageBus(uart);

Topic<LoadCellTorques> loadCellTopic;

// make a virtual copy / "mirror" of the topics that are on the teensy to forward the communications to/from the BLE
Topic<EncoderPositions> encoderTopic;
Topic<MotorFeedback> leftMotorTopic;
Topic<MotorFeedback> rightMotorTopic;
Topic<PowerReadings> ina232Topic;
Topic<LoggingState> loggingStateTopic;
Topic<TransparentControllerParameters> leftMotorControllerParams;
Topic<TransparentControllerParameters> rightMotorControllerParams;
Topic<float> leftLegIntermediateTorque;
Topic<float> rightLegIntermediateTorque;
Topic<float> leftMotorCommandTopic;
Topic<float> rightMotorCommandTopic;

LoadCellDriver loadCellDriver;

PollingPublisher<
    LoadCellDriver,
    EndpointId::LoadCellSnapshot,
    1000 //TODO this is experimental, try a more conservative value like 1,000 (1kHz) if load cell values drop
> loadCellPublisher(
    loadCellDriver,
    messageBus
);

BLEBridge bleBridge(
    messageBus,
    encoderTopic,
    loadCellTopic,
    leftMotorTopic,
    rightMotorTopic,
    ina232Topic,
    leftLegIntermediateTorque,
    rightLegIntermediateTorque,
    leftMotorCommandTopic,
    rightMotorCommandTopic
);

void setup()
{
    Serial.begin(230400);
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 3000)) {
        // wait
    }
    Serial.println("Nano : beginning peripherals initialization");

    board::nano::teensyUart.begin(
        board::nano::teensyUartBaudrate
    );

    if (!loadCellDriver.begin())
    {
        while (true) {}
    }

    messageBus.addTopic<EndpointId::EncoderSnapshot>(encoderTopic);
    messageBus.addTopic<EndpointId::LoadCellSnapshot>(loadCellTopic);
    messageBus.addTopic<EndpointId::LeftMotorSnapshot>(leftMotorTopic);
    messageBus.addTopic<EndpointId::RightMotorSnapshot>(rightMotorTopic);
    messageBus.addTopic<EndpointId::Ina232Snapshot>(ina232Topic);
	messageBus.addTopic<EndpointId::LoggingState>(loggingStateTopic);
    messageBus.addTopic<EndpointId::LeftMotorTransparentTorque>(leftLegIntermediateTorque);
    //messageBus.addTopic<EndpointId::RightMotorTransparentTorque>(rightLegIntermediateTorque);
    messageBus.addTopic<EndpointId::LeftMotorTransparentCommand>(leftMotorCommandTopic);
    //messageBus.addTopic<EndpointId::RightMotorTransparentCommand>(rightMotorCommandTopic);
    messageBus.begin();

    if (!bleBridge.begin())
    {
        Serial.println(
            "BLE initialization failed"
        );

        while (true) {}
    }


    scheduler.add(messageBus);
    scheduler.add(loadCellPublisher);
    scheduler.add(bleBridge);
    Serial.println("Nano : ready");
}

void loop()
{
    scheduler.run(micros());
}