#pragma once

#include <Arduino.h>
#include "MessageBus.h"
#include "Board.h"

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

        if (!m_enabled) {
            //Serial.println("MotorCommandTask DISABLED");
            return;
            }

        if (!m_command.valid())
        {
            //Serial.println("MOTOR CMD: no command");
            return;
        }

        const auto& cmd =
            m_command.latest();

        Serial.print("SEND p=");
        Serial.print(cmd.position);

        Serial.print(" v=");
        Serial.print(cmd.velocity);

        Serial.print(" kp=");
        Serial.print(cmd.kp);

        Serial.print(" kd=");
        Serial.print(cmd.kd);

        Serial.print(" tau=");
        Serial.println(cmd.torque);

        m_motor.apply(cmd);
    }

private:
    MotorDriver& m_motor;
    Topic<MotorCmd>& m_command;

    bool m_enabled{true};
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
                Serial.println("Entering");
                break;

            case MotorMetaCommand::ExitMotorMode:
                // Stop regular MIT frames FIRST
                Serial.println("Exciting");
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

class JointLimitController final : public ITask
{
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

    void update(uint32_t nowUs) override
    {
        static constexpr uint32_t PERIOD_US = 5'000;

        if (nowUs - m_previousUpdateUs < PERIOD_US)
            return;

        m_previousUpdateUs = nowUs;

        if (!m_encoders.valid() ||
            !m_loadcells.valid())
        {
            return;
        }

        const float position =
            m_encoders.latest().left;

        const auto& loadcellSnapshot =
            m_loadcells.latest();

        const float LC_left1 =
            loadcellSnapshot[
                static_cast<size_t>(LoadCellId::Left1)
            ];

        const float LC_left2 =
            loadcellSnapshot[
                static_cast<size_t>(LoadCellId::Left2)
            ];

        // -------------------------------------------------
        // Initialization
        // -------------------------------------------------

        if (!m_initialized)
        {
            m_previousPosition = position;
            m_previousUs = nowUs;

            m_initialized = true;
            return;
        }

        const float dt =
            static_cast<float>(
                nowUs - m_previousUs
            ) * 1e-6f;

        if (dt <= 0.0f)
            return;

        // -------------------------------------------------
        // Joint limits
        // -------------------------------------------------

        if (fabsf(position) >= absoluteLimit)
        {
            /*Serial.println(
                "ABSOLUTE JOINT LIMIT BREACHED!"
            );*/
            m_leftMotorControl.publish(MotorMetaCommand::ExitMotorMode);
        }
        else if (fabsf(position) >= softwareLimit)
        {
            /*Serial.println(
                "Software joint limit breached!"
            );*/
        }
        // -------------------------------------------------
        // Raw velocity
        // deg/s
        // -------------------------------------------------

        const float rawVelocity =
            (position - m_previousPosition)
            / dt;

        // -------------------------------------------------
        // Low-pass velocity
        // -------------------------------------------------

        m_filteredVelocity =
            VELOCITY_ALPHA * rawVelocity +
            (1.0f - VELOCITY_ALPHA)
                * m_filteredVelocity;

        // -------------------------------------------------
        // Acceleration from FILTERED velocity
        // deg/s^2
        // -------------------------------------------------

        const float rawAcceleration =
            (m_filteredVelocity -
             m_previousFilteredVelocity)
            / dt;

        // -------------------------------------------------
        // Low-pass acceleration
        // -------------------------------------------------

        m_filteredAcceleration =
            ACCELERATION_ALPHA *
                rawAcceleration +
            (1.0f - ACCELERATION_ALPHA) *
                m_filteredAcceleration;

        // Convert deg/s^2 -> rad/s^2
        const float accelerationRad =
            m_filteredAcceleration *
            PI / 180.0f;

        // -------------------------------------------------
        // Torque
        // -------------------------------------------------

        const float rawTorque =
            (LC_left2 - LC_left1)
            * 0.055f;

        // Optional torque filtering
        m_filteredTorque =
            TORQUE_ALPHA *
                rawTorque +
            (1.0f - TORQUE_ALPHA) *
                m_filteredTorque;

        // -------------------------------------------------
        // Inertia estimate
        //
        // I = tau / alpha
        // -------------------------------------------------

        if (fabsf(accelerationRad) >
            MIN_ACCELERATION_RAD)
        {
            const float inertia =
                m_filteredTorque /
                accelerationRad;

            // Reject obviously unreasonable spikes.
            if (inertia > 0 && fabsf(inertia) <
                MAX_REASONABLE_INERTIA)
            {
                //Serial.print("Position: ");
                //Serial.print(position);

                //Serial.print(" Vel: ");
                //Serial.print(m_filteredVelocity);

                Serial.print(" Accel: ");
                Serial.print(accelerationRad);

                Serial.print(" Torque: ");
                Serial.println(m_filteredTorque);

                //Serial.print(" Inertia: ");
                //Serial.println(inertia);
            }
        }

        // -------------------------------------------------
        // Save state
        // -------------------------------------------------

        m_previousPosition =
            position;

        m_previousFilteredVelocity =
            m_filteredVelocity;

        m_previousUs =
            nowUs;
    }

private:
    static constexpr float
        softwareLimit = 15.0f;

    static constexpr float
        absoluteLimit = 35.0f;

    // Smaller = more smoothing.
    static constexpr float
        VELOCITY_ALPHA = 0.15f;

    static constexpr float
        ACCELERATION_ALPHA = 0.08f;

    static constexpr float
        TORQUE_ALPHA = 0.15f;

    // Don't estimate inertia when acceleration
    // is too close to zero.
    static constexpr float
        MIN_ACCELERATION_RAD = 0.2f;

    // Example sanity filter.
    // Tune this based on your real mechanism.
    static constexpr float
        MAX_REASONABLE_INERTIA = 10.0f;

    Topic<EncoderPositions>& m_encoders;
    Topic<LoadCellForces>& m_loadcells;
    Topic<MotorReply>& m_leftMotorFeed;
    Topic<MotorCmd>& m_leftMotorCmd;
    Topic<MotorMetaCommand>& m_leftMotorControl;

    float m_previousPosition{0.0f};

    float m_filteredVelocity{0.0f};
    float m_previousFilteredVelocity{0.0f};

    float m_filteredAcceleration{0.0f};

    float m_filteredTorque{0.0f};

    uint32_t m_previousUs{0};
    uint32_t m_previousUpdateUs{0};

    bool m_initialized{false};
};

class TransparentModeController final : public ITask
{
public:
    TransparentModeController(
        Topic<EncoderPositions>& encoders,
        Topic<LoadCellForces>& loadCells,
        Topic<MotorReply>& leftMotorFeed,
        Topic<MotorCmd>& leftMotorCmd)
        : m_encoders(encoders),
          m_loadcells(loadCells),
          m_leftMotorFeed(leftMotorFeed),
          m_leftMotorCmd(leftMotorCmd)
    {}

    void update(uint32_t nowUs) override
    {
        static constexpr uint32_t PERIOD_US = 5'000; // 200 Hz

        if (nowUs - m_previousUpdateUs < PERIOD_US)
            return;

        m_previousUpdateUs = nowUs;

        if (!m_loadcells.valid())
            return;

        const auto& loadcellSnapshot =
            m_loadcells.latest();

        const float left1 =
            loadcellSnapshot[
                static_cast<size_t>(LoadCellId::Left1)
            ];

        const float left2 =
            loadcellSnapshot[
                static_cast<size_t>(LoadCellId::Left2)
            ];

        // -------------------------------------------------
        // Interaction torque measured by load cells
        // -------------------------------------------------

        const float measuredTorque =
            TORQUE_SIGN *
            (left1 - left2) *
            LOAD_CELL_LEVER_ARM;

        // -------------------------------------------------
        // First sample
        // -------------------------------------------------

        if (!m_initialized)
        {
            m_previousTorque = measuredTorque;
            m_previousUs = nowUs;
            m_initialized = true;

            return;
        }

        const float dt =
            static_cast<float>(
                nowUs - m_previousUs
            ) * 1e-6f;

        if (dt <= 0.0f)
            return;

        // -------------------------------------------------
        // Low-pass the measured interaction torque
        // -------------------------------------------------

        m_filteredTorque =
            TORQUE_FILTER_ALPHA *
                measuredTorque +
            (1.0f - TORQUE_FILTER_ALPHA) *
                m_filteredTorque;

        // -------------------------------------------------
        // Derivative of interaction torque
        //
        // Units: N.m / s
        // -------------------------------------------------

        const float rawTorqueDerivative =
            (m_filteredTorque -
             m_previousFilteredTorque)
            / dt;

        m_filteredTorqueDerivative =
            DERIVATIVE_FILTER_ALPHA *
                rawTorqueDerivative +
            (1.0f - DERIVATIVE_FILTER_ALPHA) *
                m_filteredTorqueDerivative;

        // -------------------------------------------------
        // PD transparent controller
        //
        // Desired interaction torque = 0
        //
        // error = 0 - measuredTorque
        // -------------------------------------------------

        const float error =
            -m_filteredTorque;

        const float errorDerivative =
            -m_filteredTorqueDerivative;

        float commandedTorque =
            KP * error +
            KD * errorDerivative;

        // -------------------------------------------------
        // Limit commanded assistance
        // -------------------------------------------------

        commandedTorque =
            constrain(
                commandedTorque,
                -MAX_TRANSPARENT_TORQUE,
                 MAX_TRANSPARENT_TORQUE
            );

        // -------------------------------------------------
        // MIT command:
        //
        // no position control
        // no velocity control
        // torque only
        // -------------------------------------------------

        MotorCmd command{};

        command.position = 0.0f;
        command.velocity = 0.0f;

        command.kp = 0.0f;
        command.kd = 0.0f;

        command.torque =
            commandedTorque;

        m_leftMotorCmd.publish(command);

        // -------------------------------------------------
        // Save state
        // -------------------------------------------------

        m_previousTorque =
            measuredTorque;

        m_previousFilteredTorque =
            m_filteredTorque;

        m_previousUs =
            nowUs;
    }

private:
    // -----------------------------------------------------
    // Controller tuning
    // -----------------------------------------------------

    static constexpr float KP = 0.5f;
    static constexpr float KD = 0.01f;

    // Distance between load-cell force line of action
    // and joint axis.
    static constexpr float LOAD_CELL_LEVER_ARM =
        0.055f;

    // Flip to -1.0f if your torque sign is backwards.
    static constexpr float TORQUE_SIGN =
        1.0f;

    // Start VERY conservatively.
    static constexpr float MAX_TRANSPARENT_TORQUE =
        0.4f;

    // Low-pass filters.
    static constexpr float TORQUE_FILTER_ALPHA =
        0.15f;

    static constexpr float DERIVATIVE_FILTER_ALPHA =
        0.05f;

    // -----------------------------------------------------
    // Topics
    // -----------------------------------------------------

    Topic<EncoderPositions>& m_encoders;
    Topic<LoadCellForces>& m_loadcells;
    Topic<MotorReply>& m_leftMotorFeed;
    Topic<MotorCmd>& m_leftMotorCmd;

    // -----------------------------------------------------
    // Controller state
    // -----------------------------------------------------

    float m_previousTorque{0.0f};

    float m_filteredTorque{0.0f};
    float m_previousFilteredTorque{0.0f};

    float m_filteredTorqueDerivative{0.0f};

    uint32_t m_previousUs{0};
    uint32_t m_previousUpdateUs{0};

    bool m_initialized{false};
};

#else
#endif