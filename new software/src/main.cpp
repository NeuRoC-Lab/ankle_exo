
#include <functional>
#include <Arduino.h>
#include "board.h"
#include "MessageBus.h"
#include "Driver.h"
#include "BLE.h"
#include "MotorControllers.h"


template<size_t MaxTasks>
class Scheduler
{
public:
    bool add(ITask& task)
    {
        if (m_count >= MaxTasks) {
            return false;
        }

        m_tasks[m_count++] = &task;
        return true;
    }

    void run(uint32_t nowUs)
    {
        for (size_t i = 0;
             i < m_count;
             ++i)
        {
            m_tasks[i]->update(nowUs);
        }
    }

private:
    ITask* m_tasks[MaxTasks]{};
    size_t m_count{0};
};

Scheduler<12> scheduler;

#if defined(PLATFORM_TEENSY)

UARTHandler uart(
    board::teensy41::nanoUart
);

MessageBus messageBus(uart);

Topic<EncoderPositions> encoderTopic;
Topic<LoadCellForces> loadCellTopic;
Topic<MotorReply> leftMotorTopic;
Topic<PowerReadings> ina232Topic;
//Topic<MotorReply> rightMotorTopic;
Topic<MotorCmd> leftMotorCommandTopic;
Topic<MotorMetaCommand> leftMotorMetaCommandTopic;


EncoderDriver encoderDriver;

PollingPublisher<
    EncoderDriver,
    EndpointId::EncoderSnapshot,
    5'000
> encoderPublisher(
    encoderDriver,
    messageBus
);

INA232Driver ina232Driver;

PollingPublisher<
    INA232Driver,
    EndpointId::Ina232Snapshot,
    10'000
> ina232Publisher(
    ina232Driver,
    messageBus
);


CanBus canBus;

constexpr uint8_t LEFT_MOTOR_CAN_ID = 0x02;
constexpr uint8_t RIGHT_MOTOR_CAN_ID = 0x02;

// For compile testing, use the nominal limits.
// Replace these with your real software limits later.
AK60Params leftMotorLimits = MotorParams;
//AK60Params rightMotorLimits = MotorParams;

MotorDriver leftMotor(
    LEFT_MOTOR_CAN_ID,
    leftMotorLimits,
    canBus
);

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




DummyController dummyController(
    encoderTopic,
    loadCellTopic,
    leftMotorTopic,
    ina232Topic
);

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

    board::teensy41::nanoUart.begin(
        230400
    );

    Serial.println(
        "Teensy: starting distributed bus test"
    );

    if (!encoderDriver.begin())
    {
        Serial.println(
            "Encoder initialization failed"
        );

        while (true) {}
    }

    if (!canBus.begin())
    {
        Serial.println(
            "CAN initialization failed"
        );

        while (true) {}
    }

    if (!leftMotor.begin())
    {
        Serial.println(
            "Left motor initialization failed"
        );

        while (true) {}
    }
    if (!ina232Driver.begin())
    {
        Serial.println(
            "INA232 initialization failed"
        );

        while (true) {}
    }
    /*
    if (!rightMotor.begin())
    {
        Serial.println(
            "Right motor initialization failed"
        );

        while (true) {}
    }
    */

    messageBus.addTopic<EndpointId::EncoderSnapshot>(encoderTopic);

    messageBus.addTopic<EndpointId::LoadCellSnapshot>(loadCellTopic);

    messageBus.addTopic<EndpointId::LeftMotorSnapshot>(leftMotorTopic);

    messageBus.addTopic<EndpointId::Ina232Snapshot>(ina232Topic);

    messageBus.addTopic<EndpointId::LeftMotorCommand>(leftMotorCommandTopic);

    messageBus.addTopic<EndpointId::LeftMotorMetaCommand>(leftMotorMetaCommandTopic);
    /*
    messageBus.addTopic(
        routedRightMotorTopic
    );
    */

    messageBus.begin();

    scheduler.add(messageBus);
    scheduler.add(encoderPublisher);
    scheduler.add(ina232Publisher);
    scheduler.add(motorReceiver);
    //scheduler.add(dummyController);
    scheduler.add(transparentModeController);
//since TransparentModeController publishes commands into leftMotorCommandTopic, and MotorCommandTask later reads that topic and sends the latest command to the motor, I’d schedule the transparent controller before leftMotorCommandTask:
    scheduler.add(jointController);
    scheduler.add(leftMotorCommandTask);
    scheduler.add(leftMotorMetaCommandTask);
    Serial.println("Teensy: ready");
}

void loop()
{
    scheduler.run(micros());
}

#elif defined(PLATFORM_NANO)

UARTHandler uart(
    board::nano::teensyUart
);

MessageBus messageBus(uart);

Topic<LoadCellForces> loadCellTopic;
// make a virtual copy / "mirror" of the topics that are on the teensy to forward the communications to/from the BLE
Topic<EncoderPositions> encoderTopic;
Topic<MotorReply> leftMotorTopic;
Topic<PowerReadings> ina232Topic;


LoadCellDriver loadCellDriver;

PollingPublisher<
    LoadCellDriver,
    EndpointId::LoadCellSnapshot,
    5'000
> loadCellPublisher(
    loadCellDriver,
    messageBus
);

BLEBridge bleBridge(
    messageBus,
    encoderTopic,
    loadCellTopic,
    leftMotorTopic,
    ina232Topic
);

void setup()
{
    Serial.begin(230400);

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
    messageBus.addTopic<EndpointId::Ina232Snapshot>(ina232Topic);
    messageBus.begin();

    if (!bleBridge.begin())
    {
        Serial.println(
            "BLE initialization failed"
        );

        while (true) {}
    }

    messageBus.begin();

    scheduler.add(messageBus);
    scheduler.add(loadCellPublisher);
    scheduler.add(bleBridge);
}

void loop()
{
    scheduler.run(micros());
}

#else
#error "Define PLATFORM_TEENSY or PLATFORM_NANO"
#endif
