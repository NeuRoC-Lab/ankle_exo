

#include <Arduino.h>
#include "board.h"
#include "MessageBus.h"
#include "Driver.h"
#include "BLE.h"

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

RoutedTopic<
    EndpointId::EncoderSnapshot
> routedEncoderTopic(encoderTopic);

RoutedTopic<
    EndpointId::LoadCellSnapshot
> routedLoadCellTopic(loadCellTopic);

RoutedTopic<
    EndpointId::LeftMotorSnapshot
> routedLeftMotorTopic(leftMotorTopic);

RoutedTopic<
    EndpointId::Ina232Snapshot
> routedIna232Topic(ina232Topic);

RoutedTopic<
    EndpointId::LeftMotorCommand
> routedLeftMotorCommandTopic(
    leftMotorCommandTopic
);

/*RoutedTopic<
    EndpointId::RightMotorSnapshot
> routedRightMotorTopic(rightMotorTopic);
*/

RoutedTopic<EndpointId::LeftMotorMetaCommand> routedLeftMotorMetaCommandTopic(leftMotorMetaCommandTopic);

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

constexpr uint8_t LEFT_MOTOR_CAN_ID = 0x03;
constexpr uint8_t RIGHT_MOTOR_CAN_ID = 0x01;

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

class MotorCanReceiver final :
    public ITask
{
public:
    MotorCanReceiver(
        CanBus& bus,
        MotorDriver& left,/*
        MotorDriver& right,*/
        MessageBus& messageBus)
        : m_bus(bus),
          m_left(left),
          /*m_right(right),*/
          m_messageBus(messageBus)
    {}

    void update(uint32_t) override
    {
        CanFrame frame{};

        while (m_bus.read(frame))
        {
            if (m_left.accepts(frame))
            {
                m_messageBus.publish<
                    EndpointId::LeftMotorSnapshot
                >(
                    m_left.decode(frame)
                );
            }
            /*
            else if (
                m_right.accepts(frame))
            {
                m_messageBus.publish<
                    EndpointId::RightMotorSnapshot
                >(
                    m_right.decode(frame)
                );
            }
            */
        }
    }

private:
    CanBus& m_bus;
    MotorDriver& m_left;
    //MotorDriver& m_right;
    MessageBus& m_messageBus;
};

class MotorCommandTask :
    public ITask
{
public:
    MotorCommandTask(
        MotorDriver& motor,
        Topic<MotorCmd>& command)
        : m_motor(motor),
          m_command(command)
    {}

    void update(uint32_t nowUs) override
    {
        static constexpr uint32_t
            PERIOD_US = 10'000; // 100 Hz

        if (
            nowUs - m_previousUs <
            PERIOD_US
        ) {
            return;
        }

        m_previousUs += PERIOD_US;

        /*
         * No command has been received yet.
         *
         * Send the neutral MIT command so we
         * can still obtain motor feedback.
         */
        if (!m_command.valid())
        {
            MotorCmd neutral{};

            neutral.position = 0.0f;
            neutral.velocity = 0.0f;
            neutral.torque   = 0.0f;
            neutral.kp       = 0.0f;
            neutral.kd       = 0.0f;

            m_motor.apply(neutral);

            return;
        }

        /*
         * Re-send the latest desired command.
         *
         * This is intentional: the motor's MIT
         * protocol expects regular commands,
         * and each command gives us fresh
         * feedback.
         */
        m_motor.apply(
            m_command.latest()
        );
    }

private:
    MotorDriver& m_motor;
    Topic<MotorCmd>& m_command;

    uint32_t m_previousUs{0};
};

MotorCanReceiver motorReceiver(
    canBus,
    leftMotor,
    //rightMotor,
    messageBus
);

MotorCommandTask leftMotorCommandTask(
    leftMotor,
    leftMotorCommandTopic
);

class MotorMetaCommandTask :
    public ITask
{
public:
    MotorMetaCommandTask(
        MotorDriver& motor,
        Topic<MotorMetaCommand>& command)
        : m_motor(motor),
          m_command(command)
    {}

    void update(uint32_t) override
    {
        if (!m_command.valid()) {
            return;
        }

        if (m_command.sequence() ==
            m_lastSequence)
        {
            return;
        }

        m_lastSequence =
            m_command.sequence();

        switch (m_command.latest())
        {
            case MotorMetaCommand::
                EnterMotorMode:
                m_motor.enterMotorMode();
                break;

            case MotorMetaCommand::
                ExitMotorMode:
                m_motor.exitMotorMode();
                break;

            case MotorMetaCommand::
                SetZero:
                m_motor.zeroMotor();
                break;
        }
    }

private:
    MotorDriver& m_motor;
    Topic<MotorMetaCommand>& m_command;

    uint32_t m_lastSequence{0};
};

MotorMetaCommandTask
    leftMotorMetaCommandTask(
        leftMotor,
        leftMotorMetaCommandTopic
    );

class DummyController final :
    public ITask
{
public:
    DummyController(
        Topic<EncoderPositions>& encoders,
        Topic<LoadCellForces>& loadCells,
        Topic<MotorReply>& leftMotor,
        Topic<PowerReadings>& power)
        : m_encoders(encoders),
          m_loadCells(loadCells),
          m_leftMotor(leftMotor),
          m_power(power)
    {}

    void update(uint32_t nowUs) override
    {
        static constexpr uint32_t
            PRINT_PERIOD_US = 100'000;

        if (nowUs - m_previousPrintUs <
            PRINT_PERIOD_US)
        {
            return;
        }

        m_previousPrintUs +=
            PRINT_PERIOD_US;

        Serial.println(
            "----- Distributed state -----"
        );

        printEncoder();
        printLoadCells();
        printMotor(
            "LEFT",
            m_leftMotor
        );
        printPower();
            /*
        printMotor(
            "RIGHT",
            m_rightMotor
        );
    */

        Serial.println();
    }

private:

    void printPower()
    {
        Serial.print("Power: ");

        if (!m_power.valid())
        {
            Serial.println("NO DATA");
            return;
        }

        const auto& value =
            m_power.latest();

        Serial.print("V=");
        Serial.print(value.batteryVoltage);

        Serial.print(" I=");
        Serial.print(value.pcbCurrent);

        Serial.print(" P=");
        Serial.println(value.pcbPower);
    }
    void printEncoder()
    {
        Serial.print("Encoder: ");

        if (!m_encoders.valid())
        {
            Serial.println("NO DATA");
            return;
        }

        const auto& value =
            m_encoders.latest();

        Serial.print("left=");
        Serial.print(value.left);

        Serial.print(" right=");
        Serial.println(value.right);
    }

    void printLoadCells()
    {
        Serial.print("Load cells: ");

        if (!m_loadCells.valid())
        {
            Serial.println("NO DATA");
            return;
        }

        const auto& value =
            m_loadCells.latest();

        for (size_t i = 0;
             i < value.size();
             ++i)
        {
            Serial.print(value[i]);

            if (i + 1 < value.size())
            {
                Serial.print(", ");
            }
        }

        Serial.println();
    }

    static void printMotor(
        const char* name,
        const Topic<MotorReply>& topic)
    {
        Serial.print(name);
        Serial.print(" motor: ");

        if (!topic.valid())
        {
            Serial.println("NO DATA");
            return;
        }

        const auto& value =
            topic.latest();

        Serial.print("pos=");
        Serial.print(value.position);

        Serial.print(" vel=");
        Serial.print(value.velocity);

        Serial.print(" torque=");
        Serial.print(value.torque);

        Serial.print(" temp=");
        Serial.print(value.temperature);

        Serial.print(" error=");
        Serial.println(value.error);
    }

private:
    Topic<EncoderPositions>& m_encoders;
    Topic<LoadCellForces>& m_loadCells;
    Topic<MotorReply>& m_leftMotor;
    Topic<PowerReadings>& m_power;
    /*Topic<MotorReply>& m_rightMotor;*/

    uint32_t m_previousPrintUs{0};
};



DummyController dummyController(
    encoderTopic,
    loadCellTopic,
    leftMotorTopic,
    ina232Topic
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

    messageBus.addTopic(
        routedLeftMotorMetaCommandTopic
    );

    messageBus.addTopic(
        routedEncoderTopic
    );

    messageBus.addTopic(
        routedLoadCellTopic
    );

    messageBus.addTopic(
        routedLeftMotorTopic
    );

    messageBus.addTopic(
        routedIna232Topic
    );

    messageBus.addTopic(
        routedLeftMotorCommandTopic
    );
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
    scheduler.add(dummyController);
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

RoutedTopic<
    EndpointId::EncoderSnapshot
> routedEncoderTopic(encoderTopic);

RoutedTopic<
    EndpointId::LoadCellSnapshot
> routedLoadCellTopic(loadCellTopic);

RoutedTopic<
    EndpointId::LeftMotorSnapshot
> routedLeftMotorTopic(leftMotorTopic);

RoutedTopic<
    EndpointId::Ina232Snapshot
> routedPowerTopic(ina232Topic);



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

    messageBus.addTopic(routedEncoderTopic);
    messageBus.addTopic(routedLoadCellTopic);
    messageBus.addTopic(routedLeftMotorTopic);
    messageBus.addTopic(routedPowerTopic);
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
