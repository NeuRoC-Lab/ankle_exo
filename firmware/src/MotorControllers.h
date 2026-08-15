#pragma once

#include <Arduino.h>
#include "MessageBus.h"
#include "Board.h"

#include <cmath>
#include <cstdint>

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
        static constexpr uint32_t PERIOD_US = 1'000; // 1kHz

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
		/*
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
		*/
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
        MotorDriver& left,
        MotorDriver& right,
        MessageBus& messageBus)
        : m_bus(bus),
          m_left(left),
          m_right(right),
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
            else if (
                m_right.accepts(frame))
            {
                m_messageBus.publish<
                    EndpointId::RightMotorSnapshot
                >(
                    m_right.decode(frame)
                );
            }
        }
    }

private:
    CanBus& m_bus;
    MotorDriver& m_left;
    MotorDriver& m_right;
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
            // to make this task event-driven we only take action on changing data
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
		Topic<MotorReply>& rightMotor,
        Topic<PowerReadings>& power)
        : m_encoders(encoders),
          m_loadCells(loadCells),
          m_leftMotor(leftMotor),
		  m_rightMotor(rightMotor),
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

        printMotor(
            "RIGHT",
            m_rightMotor
        );

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
    Topic<MotorReply>& m_rightMotor;

    uint32_t m_previousPrintUs{0};
};

enum class ExoSide : uint8_t
{
    Left,
    Right
};

template<ExoSide Side>
class JointLimitController final :
    public ITask
{
public:
    JointLimitController(
        Topic<EncoderPositions>& encoders,
        Topic<MotorMetaCommand>& motorControl)
        :
        m_encoders(encoders),
        m_motorControl(motorControl)
    {}


    void update(uint32_t nowUs) override
    {
        static constexpr uint32_t
            PERIOD_US = 5'000; // 200 Hz

        if (
            nowUs - m_previousUpdateUs <
            PERIOD_US
        )
        {
            return;
        }

        m_previousUpdateUs = nowUs;


        if (!m_encoders.valid())
        {
            return;
        }


        const float position =
            getPosition(
                m_encoders.latest()
            );


        /*
         * Absolute limit:
         * immediately request that the
         * selected motor exits motor mode.
         */
        if (
            fabsf(position) >=
            ABSOLUTE_LIMIT
        )
        {
            if (!m_limitTriggered)
            {
                m_motorControl.publish(
                    MotorMetaCommand::
                        ExitMotorMode
                );

                m_limitTriggered = true;

                Serial.print(
                    sideName()
                );

                Serial.println(
                    " absolute joint limit breached"
                );
            }

            return;
        }


        /*
         * Once the joint returns inside the
         * absolute limit, allow the safety
         * event to trigger again later.
         */
        m_limitTriggered = false;


        /*
         * Software limit.
         *
         * Currently only detects the condition.
         * You can later use this region for
         * torque limiting / soft-stop behavior.
         */
        if (
            fabsf(position) >=
            SOFTWARE_LIMIT
        )
        {
            // Optional:
            //
            // Serial.print(sideName());
            // Serial.println(
            //     " software joint limit reached"
            // );
        }
    }


private:

    static float getPosition(
        const EncoderPositions& encoders)
    {
        if constexpr (
            Side == ExoSide::Left
        )
        {
            return static_cast<float>(
                encoders.left
            );
        }
        else
        {
            return static_cast<float>(
                encoders.right
            );
        }
    }


    static constexpr const char*
    sideName()
    {
        if constexpr (
            Side == ExoSide::Left
        )
        {
            return "LEFT";
        }
        else
        {
            return "RIGHT";
        }
    }


private:

    static constexpr float
        SOFTWARE_LIMIT = 15.0f;

    static constexpr float
        ABSOLUTE_LIMIT = 35.0f;


    Topic<EncoderPositions>&
        m_encoders;

    Topic<MotorMetaCommand>&
        m_motorControl;


    uint32_t
        m_previousUpdateUs{0};

    bool
        m_limitTriggered{false};
};




template<ExoSide Side>
class TransparentModeController final :
    public ITask
{
public:
    TransparentModeController(
        Topic<EncoderPositions>& encoders,
        Topic<LoadCellForces>& loadCells,
        Topic<MotorReply>& motorFeed,
        Topic<MotorCmd>& motorCmd)
        :
        m_encoders(encoders),
        m_loadcells(loadCells),
        m_motorFeed(motorFeed),
        m_motorCmd(motorCmd)
    {}

    void update(uint32_t nowUs) override
    {
        static constexpr uint32_t
            PERIOD_US = 1'000;

        if (
            nowUs - m_previousUpdateUs <
            PERIOD_US
        )
        {
            return;
        }

        m_previousUpdateUs = nowUs;

        if (!m_loadcells.valid())
        {
            return;
        }

        const auto& loadcellSnapshot =
            m_loadcells.latest();

        const float force1 =
            loadcellSnapshot[
                static_cast<size_t>(
                    firstLoadCell()
                )
            ];

        const float force2 =
            loadcellSnapshot[
                static_cast<size_t>(
                    secondLoadCell()
                )
            ];

        const float measuredTorque =
            torqueSign()
            *
            (force1 - force2)
            *
            LOAD_CELL_LEVER_ARM;

        if (!m_initialized)
        {
            m_filteredTorque =
                measuredTorque;

            m_previousFilteredTorque =
                measuredTorque;

            m_filteredTorqueDerivative =
                0.0f;

            m_filteredFrictionComp =
                0.0f;

            m_previousUs =
                nowUs;

            m_initialized =
                true;

            return;
        }

        const float dt =
            static_cast<float>(
                nowUs - m_previousUs
            ) * 1e-6f;

        if (dt <= 0.0f)
        {
            return;
        }

        m_filteredTorque =
            TORQUE_FILTER_ALPHA *
                measuredTorque
            +
            (1.0f -
                TORQUE_FILTER_ALPHA)
                *
                m_filteredTorque;

        const float
            rawTorqueDerivative =
                (
                    m_filteredTorque
                    -
                    m_previousFilteredTorque
                ) / dt;

        m_filteredTorqueDerivative =
            DERIVATIVE_FILTER_ALPHA
                *
                rawTorqueDerivative
            +
            (
                1.0f -
                DERIVATIVE_FILTER_ALPHA
            )
                *
                m_filteredTorqueDerivative;

        const float error =
            -m_filteredTorque;

        const float errorDerivative =
            -m_filteredTorqueDerivative;

        const float feedbackTorque =
            KP * error
            +
            KD * errorDerivative;

        updateFrictionDirection(
            feedbackTorque
        );

        float targetFrictionComp =
            0.0f;

        if (
            m_frictionDirection > 0
        )
        {
            targetFrictionComp =
                STATIC_FRICTION_COMP_TORQUE;
        }
        else if (
            m_frictionDirection < 0
        )
        {
            targetFrictionComp =
                -STATIC_FRICTION_COMP_TORQUE;
        }

        m_filteredFrictionComp =
            FRICTION_FILTER_ALPHA
                *
                targetFrictionComp
            +
            (
                1.0f -
                FRICTION_FILTER_ALPHA
            )
                *
                m_filteredFrictionComp;

        float commandedTorque =
            feedbackTorque
            +
            m_filteredFrictionComp;

        commandedTorque =
            constrain(
                commandedTorque,
                -MAX_TRANSPARENT_TORQUE,
                MAX_TRANSPARENT_TORQUE
            );

        MotorCmd command{};

        command.position =
            0.0f;

        command.velocity =
            0.0f;

        command.kp =
            0.0f;

        command.kd =
            0.0f;

        command.torque =
            commandedTorque;

        m_motorCmd.publish(
            command
        );

        m_previousFilteredTorque =
            m_filteredTorque;

        m_previousUs =
            nowUs;
    }


private:

    static constexpr LoadCellId
    firstLoadCell()
    {
        if constexpr (
            Side == ExoSide::Left
        )
        {
            return LoadCellId::Left1;
        }
        else
        {
            return LoadCellId::Right1;
        }
    }


    static constexpr LoadCellId
    secondLoadCell()
    {
        if constexpr (
            Side == ExoSide::Left
        )
        {
            return LoadCellId::Left2;
        }
        else
        {
            return LoadCellId::Right2;
        }
    }


    static constexpr float
    torqueSign()
    {
        if constexpr (
            Side == ExoSide::Left
        )
        {
            return LEFT_TORQUE_SIGN;
        }
        else
        {
            return RIGHT_TORQUE_SIGN;
        }
    }


    void updateFrictionDirection(
        float feedbackTorque)
    {
        if (
            feedbackTorque >
            FRICTION_TRIGGER_ON_TORQUE
        )
        {
            m_frictionDirection = +1;
            return;
        }

        if (
            feedbackTorque <
            -FRICTION_TRIGGER_ON_TORQUE
        )
        {
            m_frictionDirection = -1;
            return;
        }

        if (
            std::abs(feedbackTorque) <
            FRICTION_TRIGGER_OFF_TORQUE
        )
        {
            m_frictionDirection = 0;
        }
    }


private:

    static constexpr float KP =
        0.5f;

    static constexpr float KD =
        0.01f;

    static constexpr float
        LOAD_CELL_LEVER_ARM =
            0.055f;

    static constexpr float
        LEFT_TORQUE_SIGN =
            1.0f;

    static constexpr float
        RIGHT_TORQUE_SIGN =
            -1.0f;

    static constexpr float
        STATIC_FRICTION_COMP_TORQUE =
            0.08f;

    static constexpr float
        FRICTION_TRIGGER_ON_TORQUE =
            0.025f;

    static constexpr float
        FRICTION_TRIGGER_OFF_TORQUE =
            0.010f;

    static constexpr float
        FRICTION_FILTER_ALPHA =
            0.10f;

    static constexpr float
        MAX_TRANSPARENT_TORQUE =
            0.4f;

    static constexpr float
        TORQUE_FILTER_ALPHA =
            0.15f;

    static constexpr float
        DERIVATIVE_FILTER_ALPHA =
            0.05f;


    Topic<EncoderPositions>&
        m_encoders;

    Topic<LoadCellForces>&
        m_loadcells;

    Topic<MotorReply>&
        m_motorFeed;

    Topic<MotorCmd>&
        m_motorCmd;


    float m_filteredTorque{0.0f};

    float
        m_previousFilteredTorque{0.0f};

    float
        m_filteredTorqueDerivative{0.0f};

    float
        m_filteredFrictionComp{0.0f};

    int m_frictionDirection{0};

    uint32_t m_previousUs{0};

    uint32_t m_previousUpdateUs{0};

    bool m_initialized{false};
};

// NEW FOR NOW : TORQUE BANDWIDTH CONTROLLER FOR BANDWIDTH EVALUATION

template<ExoSide Side>
class TorqueBandwidthController final :
    public ITask
{
public:

    explicit TorqueBandwidthController(
        Topic<MotorCmd>& motorCmd,
		Topic<LoggingState>& loggingState
		)
        :
        m_motorCmd(motorCmd),
		m_loggingState(loggingState)
    {}


    // =====================================================
    // CONFIGURATION
    // =====================================================

    void setAmplitude(
        float amplitudeNm)
    {
        m_amplitudeNm =
            constrain(
                amplitudeNm,
                0.0f,
                MAX_AMPLITUDE_NM
            );
    }


    float amplitude() const
    {
        return m_amplitudeNm;
    }


    float frequency() const
    {
        return FREQUENCIES_HZ[
            m_frequencyIndex
        ];
    }


    size_t frequencyIndex() const
    {
        return m_frequencyIndex;
    }


    // =====================================================
    // START SWEEP
    // =====================================================

    void start(uint32_t nowUs)
	{
    m_frequencyIndex = 0;

    m_frequencyStartUs =
        nowUs;

    m_previousUpdateUs =
        nowUs;

    m_running =
        true;

    // Start SD logging automatically.
    m_loggingState.publish(
        LoggingState::Recording
    );

    Serial.println(
        "Bandwidth sweep started"
    );

    printCurrentFrequency();
}


    // =====================================================
    // STOP SWEEP
    // =====================================================

    void stop()
	{
    if (!m_running)
    {
        return;
    }

    m_running =
        false;

    publishZeroTorque();

    // Stop and flush SD logging.
    m_loggingState.publish(
        LoggingState::Stopped
    );

    Serial.println(
        "Bandwidth sweep stopped"
    );
	}


    bool running() const
    {
        return m_running;
    }


    // =====================================================
    // TASK
    // =====================================================

    void update(
        uint32_t nowUs) override
    {
        if (!m_running)
        {
            return;
        }


        if (
            nowUs - m_previousUpdateUs <
            PERIOD_US
        )
        {
            return;
        }


        m_previousUpdateUs +=
            PERIOD_US;


        const float frequencyHz =
            FREQUENCIES_HZ[
                m_frequencyIndex
            ];


        const float elapsedSeconds =
            static_cast<float>(
                nowUs -
                m_frequencyStartUs
            )
            *
            1e-6f;


        // =================================================
        // CHECK IF CURRENT FREQUENCY IS COMPLETE
        // =================================================

        const float requiredTime =
            dwellTimeForFrequency(
                frequencyHz
            );


        if (
            elapsedSeconds >=
            requiredTime
        )
        {
            advanceFrequency(
                nowUs
            );

            return;
        }


        // =================================================
        // SINUSOIDAL TORQUE COMMAND
        // =================================================

        const float phase =
            TWO_PI
            *
            frequencyHz
            *
            elapsedSeconds;


        const float torque =
            torqueSign()
            *
            m_amplitudeNm
            *
            sinf(
                phase
            );


        MotorCmd command{};

        command.position =
            0.0f;

        command.velocity =
            0.0f;

        command.kp =
            0.0f;

        command.kd =
            0.0f;

        command.torque =
            torque;


        m_motorCmd.publish(
            command
        );
    }


private:

    // =====================================================
    // ADVANCE TO NEXT FREQUENCY
    // =====================================================

    void advanceFrequency(
        uint32_t nowUs)
    {
        // Zero command at transition.

        publishZeroTorque();


        ++m_frequencyIndex;


        if (
    m_frequencyIndex >=
    FREQUENCY_COUNT
)
{
    Serial.println(
        "Bandwidth sweep complete"
    );

    stop();

    return;
}


        // Reset time / phase for the next
        // frequency. Since sin(0) = 0,
        // frequency changes begin at zero torque.

        m_frequencyStartUs =
            nowUs;


        printCurrentFrequency();
    }


    // =====================================================
    // DWELL TIME
    // =====================================================

    static float dwellTimeForFrequency(
        float frequencyHz)
    {
        // Enough time for MIN_CYCLES at
        // every frequency.

        const float cycleTime =
            static_cast<float>(
                MIN_CYCLES
            )
            /
            frequencyHz;


        // But never use less than
        // MIN_DWELL_TIME_S.

        if (
            cycleTime >
            MIN_DWELL_TIME_S
        )
        {
            return cycleTime;
        }


        return MIN_DWELL_TIME_S;
    }


    // =====================================================
    // ZERO TORQUE
    // =====================================================

    void publishZeroTorque()
    {
        MotorCmd command{};

        command.position =
            0.0f;

        command.velocity =
            0.0f;

        command.kp =
            0.0f;

        command.kd =
            0.0f;

        command.torque =
            0.0f;


        m_motorCmd.publish(
            command
        );
    }


    // =====================================================
    // SIDE
    // =====================================================

    static constexpr float
    torqueSign()
    {
        if constexpr (
            Side == ExoSide::Left
        )
        {
            return LEFT_TORQUE_SIGN;
        }
        else
        {
            return RIGHT_TORQUE_SIGN;
        }
    }


    static constexpr const char*
    sideName()
    {
        if constexpr (
            Side == ExoSide::Left
        )
        {
            return "LEFT";
        }
        else
        {
            return "RIGHT";
        }
    }


    void printCurrentFrequency() const
    {
        Serial.print(
            sideName()
        );

        Serial.print(
            " bandwidth test: "
        );

        Serial.print(
            FREQUENCIES_HZ[
                m_frequencyIndex
            ],
            2
        );

        Serial.println(
            " Hz"
        );
    }


private:

    // =====================================================
    // TASK FREQUENCY
    // =====================================================

    static constexpr uint32_t
        PERIOD_US =
            1'000;       // 1 kHz


    // =====================================================
    // TEST PARAMETERS
    // =====================================================

    static constexpr float
        MAX_AMPLITUDE_NM =
            1.0f;


    // Minimum number of complete sinusoidal cycles
    // recorded at each frequency.

    static constexpr uint32_t
        MIN_CYCLES =
            10;


    // At higher frequencies, 10 cycles would
    // otherwise be extremely short.
    //
    // Example:
    //
    // 50 Hz -> 10 cycles = only 0.2 s.
    //
    // So keep recording each high-frequency
    // point for at least 3 seconds.

    static constexpr float
        MIN_DWELL_TIME_S =
            3.0f;


    // =====================================================
    // FREQUENCY SWEEP
    // =====================================================

    inline static constexpr float
        FREQUENCIES_HZ[] =
        {
            0.5f,
            1.0f,
            2.0f,
            3.0f,
            5.0f,
            7.0f,
            10.0f,
            15.0f,
            20.0f,
            30.0f,
            40.0f,
            50.0f
        };


    inline static constexpr size_t
        FREQUENCY_COUNT =
            sizeof(FREQUENCIES_HZ)
            /
            sizeof(FREQUENCIES_HZ[0]);


    // =====================================================
    // SIGNS
    // =====================================================

    static constexpr float
        LEFT_TORQUE_SIGN =
            1.0f;

    static constexpr float
        RIGHT_TORQUE_SIGN =
            -1.0f;


    // =====================================================
    // STATE
    // =====================================================

    Topic<MotorCmd>&
        m_motorCmd;


    float
        m_amplitudeNm{0.2f};


    size_t
        m_frequencyIndex{0};


    uint32_t
        m_frequencyStartUs{0};

    uint32_t
        m_previousUpdateUs{0};


    bool m_running{false};
	Topic<LoggingState>& m_loggingState;
};
#else
#endif
