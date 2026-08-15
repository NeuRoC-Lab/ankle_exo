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

UARTHandler uart(
    board::nano::teensyUart
);

MessageBus messageBus(uart);

Topic<LoadCellForces> loadCellTopic;

// make a virtual copy / "mirror" of the topics that are on the teensy to forward the communications to/from the BLE
Topic<EncoderPositions> encoderTopic;
Topic<MotorReply> leftMotorTopic;
Topic<MotorReply> rightMotorTopic;
Topic<PowerReadings> ina232Topic;
Topic<LoggingState> loggingStateTopic;

LoadCellDriver loadCellDriver;

PollingPublisher<
    LoadCellDriver,
    EndpointId::LoadCellSnapshot,
    1'000
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
    ina232Topic
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