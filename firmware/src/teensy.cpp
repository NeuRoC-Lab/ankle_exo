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
Topic<MotorReply> rightMotorTopic;
Topic<PowerReadings> ina232Topic;
Topic<MotorCmd> leftMotorCommandTopic;
Topic<MotorCmd> rightMotorCommandTopic;
Topic<MotorMetaCommand> leftMotorMetaCommandTopic;
Topic<MotorMetaCommand> rightMotorMetaCommandTopic;
Topic<LoggingState> loggingStateTopic;

//*** ENCODER INITIALISATION **//

EncoderDriver encoderDriver;

PollingPublisher<
    EncoderDriver,
    EndpointId::EncoderSnapshot,
    1'000
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
    1'000
> ina232Publisher(
    ina232Driver,
    messageBus
);

//SD CARD Driver initialization

SDCardDriver sdCard(
    "sweep.csv"
);


SDLogger sdLogger(
    sdCard,
    loadCellTopic,
    encoderTopic,
    leftMotorTopic,
    rightMotorTopic,
    leftMotorCommandTopic,
    rightMotorCommandTopic,
    loggingStateTopic
);

//** CAN BUS definitions **//

CanBus canBus;

constexpr uint8_t LEFT_MOTOR_CAN_ID = 0x02;
constexpr uint8_t RIGHT_MOTOR_CAN_ID = 0x03;

AK60Params leftMotorLimits = MotorParams;
AK60Params rightMotorLimits = MotorParams;
//todo check whether we need to customize each motor's limits. Here we make them equal to the nominal limits

MotorDriver leftMotor(
    LEFT_MOTOR_CAN_ID,
    leftMotorLimits,
    canBus
);


MotorDriver rightMotor(
    RIGHT_MOTOR_CAN_ID,
    rightMotorLimits,
    canBus
);

MotorCommandTask leftMotorCommandTask(
    leftMotor,
    leftMotorCommandTopic
);

MotorCommandTask rightMotorCommandTask(
    rightMotor,
    rightMotorCommandTopic
);

MotorCanReceiver motorReceiver(
    canBus,
    leftMotor,
    rightMotor,
    messageBus
);

MotorMetaCommandTask
    leftMotorMetaCommandTask(
        leftMotor,
        leftMotorMetaCommandTopic,
        leftMotorCommandTask
);

MotorMetaCommandTask
    rightMotorMetaCommandTask(
        rightMotor,
        rightMotorMetaCommandTopic,
        rightMotorCommandTask
);

/* DEBUG : prints sensor and motor data to Serial continuously
DummyController dummyController(
    encoderTopic,
    loadCellTopic,
    leftMotorTopic,
    rightMotorTopic,
    ina232Topic
);
*/

//TODO PUT BACK
/*
JointLimitController<
    ExoSide::Left
> leftJointLimitController(
    encoderTopic,
    leftMotorMetaCommandTopic
);


JointLimitController<
    ExoSide::Right
> rightJointLimitController(
    encoderTopic,
    rightMotorMetaCommandTopic
);


TransparentModeController<
    ExoSide::Left
> leftTransparentModeController(
    encoderTopic,
    loadCellTopic,
    leftMotorTopic,
    leftMotorCommandTopic
);

TransparentModeController<
    ExoSide::Right
> rightTransparentModeController(
    encoderTopic,
    loadCellTopic,
    rightMotorTopic,
    rightMotorCommandTopic
);
*/


TorqueBandwidthController<
    ExoSide::Left
> leftBandwidthController(
    leftMotorCommandTopic,
    loggingStateTopic
);
//TODO TEMPORARY FOR DEBUG

volatile bool restartRequested = false; // New  : for reset functionality on "STOP"

void resetButtonISR() {
  restartRequested = true;
}

void setup()
{
	pinMode(board::teensy41::pins.stopPin, INPUT_PULLUP);
	attachInterrupt(
    digitalPinToInterrupt(board::teensy41::pins.stopPin),
    resetButtonISR,
    FALLING
  );

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
	if (!rightMotor.begin())
	{
    Serial.println(
        "Right motor initialization failed"
    );
    while (true) {}
	}

    if (!ina232Driver.begin())
    {
    Serial.println("INA232 initialization failed");
    while (true) {}
    }
	if (!sdCard.begin())
	{
    Serial.println("SD card initialization failed");
	}
	else{
	Serial.println("SD card initialization successful");
	}
    //TODO initialise right motor later

    messageBus.addTopic<EndpointId::EncoderSnapshot>(encoderTopic);
    messageBus.addTopic<EndpointId::LoadCellSnapshot>(loadCellTopic);
    messageBus.addTopic<EndpointId::LeftMotorSnapshot>(leftMotorTopic);
    messageBus.addTopic<EndpointId::RightMotorSnapshot>(rightMotorTopic);
    messageBus.addTopic<EndpointId::Ina232Snapshot>(ina232Topic);
    messageBus.addTopic<EndpointId::LeftMotorCommand>(leftMotorCommandTopic);
    messageBus.addTopic<EndpointId::LeftMotorMetaCommand>(leftMotorMetaCommandTopic);
    messageBus.addTopic<EndpointId::RightMotorCommand>(rightMotorCommandTopic);
    messageBus.addTopic<EndpointId::RightMotorMetaCommand>(rightMotorMetaCommandTopic);
	messageBus.addTopic<EndpointId::LoggingState>(loggingStateTopic);
    messageBus.begin();

    scheduler.add(messageBus);
    scheduler.add(encoderPublisher);
    scheduler.add(ina232Publisher);

	scheduler.add(leftBandwidthController);

	//scheduler.add(dummyController); //TODO debug only
   	//scheduler.add(leftTransparentModeController);
	//scheduler.add(rightTransparentModeController);
   	//scheduler.add(leftJointLimitController);
	//scheduler.add(rightJointLimitController);

    scheduler.add(motorReceiver);
    scheduler.add(leftMotorCommandTask);
    scheduler.add(leftMotorMetaCommandTask);
    scheduler.add(rightMotorCommandTask);
    scheduler.add(rightMotorMetaCommandTask);

	rightMotor.exitMotorMode();
	rightMotorCommandTask.setEnabled(false);
	//TODO TEMPORARILY TURN OFF THE RIGHT MOTOR

	scheduler.add(sdLogger);
    Serial.println("Teensy: ready");

	Serial.println("Bandwidth sweep starts in 5 seconds");

	delay(5000);

	leftBandwidthController.setAmplitude(
    0.5f
	);

	leftBandwidthController.start(
    micros()
	);
	Serial.println("Sweeping controller : started");
}

void loop()
{
    scheduler.run(micros());
	if (restartRequested) {
	Serial.println("STOP button has been pressed, restarting the Teensy");
	delay(20);
	SCB_AIRCR = 0x05FA0004;
	}
}

