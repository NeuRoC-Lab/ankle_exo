#pragma once

#include <Arduino.h>
#include "MessageBus.h"

#if defined(PLATFORM_TEENSY)

class MotorCommandTask : public ITask
{
public:
    MotorCommandTask(
        MotorDriver& motor,
        Topic<MotorCmd>& command)
        : m_motor(motor),
          m_command(command)
    {}

    void setEnabled(bool enabled)
    {
        m_enabled = enabled;
    }

    void update(uint32_t nowUs) override
    {
        static constexpr uint32_t PERIOD_US = 10'000;

        if (nowUs - m_previousUs < PERIOD_US)
            return;

        m_previousUs += PERIOD_US;

        // IMPORTANT: send nothing after ExitMotorMode
        if (!m_enabled)
            return;

        if (!m_command.valid())
        {
            MotorCmd neutral{};
            m_motor.apply(neutral);
            return;
        }

        m_motor.apply(m_command.latest());
    }

private:
    MotorDriver& m_motor;
    Topic<MotorCmd>& m_command;

    bool m_enabled{false};
    uint32_t m_previousUs{0};
};


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

class MotorMetaCommandTask : public ITask
{
public:
    MotorMetaCommandTask(
        MotorDriver& motor,
        Topic<MotorMetaCommand>& command,
        MotorCommandTask& motorCommandTask)
        : m_motor(motor),
          m_command(command),
          m_motorCommandTask(motorCommandTask)
    {}

    void update(uint32_t) override
    {
        if (!m_command.valid())
            return;

        if (m_command.sequence() == m_lastSequence)
            return;

        m_lastSequence = m_command.sequence();

        switch (m_command.latest())
        {
            case MotorMetaCommand::EnterMotorMode:
                m_motor.enterMotorMode();
                m_motorCommandTask.setEnabled(true);
                break;

            case MotorMetaCommand::ExitMotorMode:
                // Stop regular MIT frames FIRST
                m_motorCommandTask.setEnabled(false);
                m_motor.exitMotorMode();
                break;

            case MotorMetaCommand::SetZero:
                m_motor.zeroMotor();
                break;
        }
    }

private:
    MotorDriver& m_motor;
    Topic<MotorMetaCommand>& m_command;
    MotorCommandTask& m_motorCommandTask;

    uint32_t m_lastSequence{0};
};

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

class JointLimitController final : public ITask {

const float softwareLimit = 15.0f; // the soft limit will be at 15.0f degrees
const float absoluteLimit = 25.0f; // the hard (absolute) limit will be at 25.0f degrees

public:
    JointLimitController(
        Topic<EncoderPositions>& encoders,
        Topic<LoadCellForces>& loadCells,
        Topic<MotorReply>& leftMotorFeed,
        Topic<MotorCmd>& leftMotorCmd,
        Topic<MotorMetaCommand>& leftMotorControl)
             : m_encoders(encoders),
               m_loadcells(loadCells),
               m_leftMotorFeed(leftMotorFeed),
               m_leftMotorCmd(leftMotorCmd),
               m_leftMotorControl(leftMotorControl)
             {}

    void update(uint32_t nowUs) override {
        if(m_encoders.valid()){
            const auto& value = m_encoders.latest().left;
            if (abs(value) > softwareLimit){
                Serial.println("You reached the software limit !");
            }
        }

    }

private:
   Topic<EncoderPositions>& m_encoders;
   Topic<LoadCellForces>& m_loadcells;
   Topic<MotorReply>& m_leftMotorFeed;
   Topic<MotorCmd>& m_leftMotorCmd;
   Topic<MotorMetaCommand>& m_leftMotorControl;

};
#else
#endif