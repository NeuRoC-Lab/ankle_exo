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

// UART bridge to Arduino Nano
UARTHandler uart(
    board::teensy41::nanoUart
);

//TODO move MessagebUS AND Topics declaration to a "commons.h" to clear up main.cpp

// Messagebus to communicate with Nano
MessageBus messageBus(uart);

//*** TOPIC INITIALISATION **//

Topic<EncoderPositions> encoderTopic;
Topic<LoadCellTorques> loadCellTopic;
Topic<MotorFeedback> leftMotorTopic;
Topic<MotorFeedback> rightMotorTopic;
Topic<PowerReadings> ina232Topic;
Topic<float> leftLegIntermediateTorque;
Topic<float> rightLegIntermediateTorque;
Topic<float> leftMotorCommandTopic;
Topic<float> rightMotorCommandTopic;
Topic<bool> leftMotorEnabled;
Topic<bool> rightMotorEnabled;
Topic<LoggingState> loggingStateTopic;
Topic<TransparentControllerParameters> leftMotorControllerParams;
Topic<TransparentControllerParameters> rightMotorControllerParams;


//*** ENCODER INITIALISATION **//

EncoderDriver encoderDriver;

PollingPublisher<
    EncoderDriver,
    EndpointId::EncoderSnapshot,
    1'000 // 1,000 µs, which corresponds to 1 kHz
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
    10'000 // 100,000 µs, which corresponds to 100 Hz. We don't need a fast sampling rate and we use I2C which might fail at higher frequencies
> ina232Publisher(
    ina232Driver,
    messageBus
);

//SD CARD Driver initialization

SDCardDriver sdCard(
    "NO_TRANSPARENT.bin"
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

constexpr uint8_t LEFT_MOTOR_CAN_ID = 0x03;
constexpr uint8_t RIGHT_MOTOR_CAN_ID = 0x02;

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
    leftMotorCommandTopic,
	leftMotorEnabled
);

MotorCommandTask rightMotorCommandTask(
    rightMotor,
    rightMotorCommandTopic,
	rightMotorEnabled
);

MotorCanReceiver motorCanReceiver(
    canBus,
    leftMotor,
    rightMotor,
    leftMotorTopic,
    rightMotorTopic
);


DummyController dummyController(
    encoderTopic,
    loadCellTopic,
    leftMotorTopic,
    rightMotorTopic,
    ina232Topic
);


/*
JointLimitController<
    Side::Left
> leftJointLimitController(
    encoderTopic,
    leftMotorMetaCommandTopic
);

JointLimitController<
    Side::Right
> rightJointLimitController(
    encoderTopic,
    rightMotorMetaCommandTopic
);
*/


TransparentModeController<
    Side::Left
> leftTransparentModeController(
    loadCellTopic,
    leftMotorTopic,
    leftMotorCommandTopic,
	leftMotorControllerParams,
	leftLegIntermediateTorque
);



TransparentModeController<
    Side::Right
> rightTransparentModeController(
    loadCellTopic,
    rightMotorTopic,
    rightMotorCommandTopic,
	rightMotorControllerParams,
	rightLegIntermediateTorque
);



/*
TorqueBandwidthController<
    Side::Left
> leftBandwidthController(
    leftMotorCommandTopic,
    loggingStateTopic
);
*/
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
    Serial.println("[ERROR] Encoders initialization failed");
        while (true) {}
    }
	else {
	Serial.println("[STARTUP] Encoders initialization succeeded");
	}


    if (!canBus.begin())
    {
    Serial.println("[ERROR] CAN initialization failed");
    while(true){}
    }
	else {
	Serial.println("[STARTUP] CAN initialization succeeded");
	}
    if (!leftMotor.begin())
    {
    Serial.println("[ERROR] Left motor initialization failed");
    while (true) {}
    }
	else {
	Serial.println("[STARTUP] Left motor initialization succeeded");
	}

	if (!rightMotor.begin())
	{
    Serial.println("[ERROR] Right motor initialization failed");
    while (true) {}
	}
	else {
	Serial.println("[STARTUP] Right motor initialization succeeded");
	}

    if (!ina232Driver.begin())
    {
    Serial.println("[ERROR] INA232 initialization failed");
    while (true) {}
    }
	else {
	Serial.println("[STARTUP] INA232 initialization succeeded");
	}
	if (!sdCard.begin())
	{
    Serial.println("[ERROR] SD card initialization failed");
	}
	else{
	Serial.println("[STARTUP] SD card initialization successful");
	}
    delay(1000);
    //TODO initialise right motor later

    messageBus.addTopic<EndpointId::EncoderSnapshot>(encoderTopic);
    messageBus.addTopic<EndpointId::LoadCellSnapshot>(loadCellTopic);
    messageBus.addTopic<EndpointId::LeftMotorSnapshot>(leftMotorTopic);
    messageBus.addTopic<EndpointId::RightMotorSnapshot>(rightMotorTopic);
    messageBus.addTopic<EndpointId::Ina232Snapshot>(ina232Topic);
    messageBus.addTopic<EndpointId::LeftMotorCommand>(leftMotorCommandTopic);
    messageBus.addTopic<EndpointId::LeftMotorEnabled>(leftMotorEnabled);
    messageBus.addTopic<EndpointId::RightMotorCommand>(rightMotorCommandTopic);
    messageBus.addTopic<EndpointId::RightMotorEnabled>(rightMotorEnabled);
	messageBus.addTopic<EndpointId::LoggingState>(loggingStateTopic);
    messageBus.addTopic<EndpointId::LeftMotorTransparentParams>(leftMotorControllerParams);
    messageBus.addTopic<EndpointId::RightMotorTransparentParams>(rightMotorControllerParams);
    messageBus.addTopic<EndpointId::LeftMotorTransparentTorque>(leftLegIntermediateTorque);
    messageBus.addTopic<EndpointId::RightMotorTransparentTorque>(rightLegIntermediateTorque);
    messageBus.begin();

    scheduler.add(messageBus);
    scheduler.add(encoderPublisher);
    scheduler.add(ina232Publisher);

	//scheduler.add(leftBandwidthController);

	//scheduler.add(dummyController); //TODO debug only
   	scheduler.add(leftTransparentModeController);
   scheduler.add(rightTransparentModeController);
   //scheduler.add(leftJointLimitController);
	//scheduler.add(rightJointLimitController);

    scheduler.add(motorCanReceiver);
    scheduler.add(leftMotorCommandTask);
    scheduler.add(rightMotorCommandTask);


	//rightMotor.exitMotorMode();
	//rightMotorCommandTask.setEnabled(false);
	//TODO TEMPORARILY TURN OFF THE RIGHT MOTOR

	scheduler.add(sdLogger);
    Serial.println("Teensy: ready");

// load default (safe) values for the transparent mode controllers
leftMotorControllerParams.publish(
    DEFAULT_TRANSPARENT_CONTROLLER_PARAMETERS
);

rightMotorControllerParams.publish(
    DEFAULT_TRANSPARENT_CONTROLLER_PARAMETERS
);
    /*
	Serial.println("Bandwidth sweep starts in 5 seconds");
	delay(5000);

	leftBandwidthController.setAmplitude(0.5f);
	leftBandwidthController.start(micros());
	Serial.println("Sweeping controller : started");
    */

}

void loop()
{
	if (restartRequested) {
	Serial.println("STOP button has been pressed, restarting the Teensy");
	delay(20);
	SCB_AIRCR = 0x05FA0004;
	}
    /*
    static uint32_t count = 0;
    static uint32_t previousUs = micros();

    ++count;
    */
    scheduler.run(micros());
    /*
    const uint32_t nowUs = micros();

    if (nowUs - previousUs >= 1'000'000)
    {
        Serial.print("Scheduler loops/s: ");
        Serial.println(count);

        count = 0;
        previousUs = nowUs;
    }
*/
}

