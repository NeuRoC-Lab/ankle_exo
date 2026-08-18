#pragma once

#include <Arduino.h>
#include "MessageBus.h"
#include "Board.h"

#include <cmath>
#include <cstdint>


// default values for the transparent controller. By default disable it btw
// these values were tested safely on the test bench, however it hasn't been validated when the exo is worn.


#if defined(PLATFORM_TEENSY)

class MotorCommandTask : public ITask
/**
@brief Standalone task that periodically sends user command to motor

@param motorControl : the topic which holds the state of the motor (on,off)
@param command : the user commanded torque
@param motor : the CubeMars motor instance
**/
{
public:
    MotorCommandTask(
        MotorDriver& motor,
        Topic<float>& command,
        Topic<bool>& motorEnabled)
        : m_motor(motor),
          m_command(command),
          m_motorEnabled(motorEnabled)
    {}

    void update(uint32_t nowUs) override
    {
        static constexpr uint32_t PERIOD_US = 1'000; // update at 1kHz

        processmotorEnabled();

        if (nowUs - m_previousUs < PERIOD_US)
            return;

        m_previousUs += PERIOD_US;

        if (!m_enabled)
        {
            m_motor.apply(0.0f); // set a zero torque command to disable the motor
            return;
        }

        if (!m_command.valid())
            return;

        m_motor.apply(m_command.latest());
    }

	void setEnabled(const bool enable){
		m_enabled = enable;
		}

private:
    void processmotorEnabled()
    {
        if (!m_motorEnabled.valid())
            return;

        if (m_motorEnabled.sequence() == m_lastControlSequence)
            return; // only accept new commands

        m_lastControlSequence = m_motorEnabled.sequence();

        setEnabled(m_motorEnabled.latest());
    }

    MotorDriver& m_motor;
    Topic<float>& m_command;
    Topic<bool>& m_motorEnabled;

    bool m_enabled{true};
    bool m_lastControlSequence{0};
    uint32_t m_previousUs{0};
};

//to fix : find an alternative way to using a ring buffer or move it to a separate header file

template<typename T, size_t Capacity>
class RingBuffer
{
public:
    bool push(const T& value)
    {
        const size_t next =
            (m_head + 1) % Capacity;

        if (next == m_tail)
        {
            // Buffer full
            ++m_dropped;
            return false;
        }

        m_buffer[m_head] = value;
        m_head = next;

        return true;
    }


    bool pop(T& value)
    {
        if (m_tail == m_head)
        {
            return false;
        }

        value = m_buffer[m_tail];

        m_tail =
            (m_tail + 1) % Capacity;

        return true;
    }


    size_t dropped() const
    {
        return m_dropped;
    }


private:
    std::array<T, Capacity> m_buffer{};

    volatile size_t m_head{0};
    volatile size_t m_tail{0};

    volatile size_t m_dropped{0};
};


class MotorCanReceiver final : public ITask
{
public:

    MotorCanReceiver(
        CanBus& canBus,
        MotorDriver& left,
        MotorDriver& right,
        Topic<MotorFeedback>& leftFeedback,
        Topic<MotorFeedback>& rightFeedback)
        :
        m_bus(canBus),
        m_left(left),
        m_right(right),
        m_leftFeedback(leftFeedback),
        m_rightFeedback(rightFeedback)
    {}

    void update(uint32_t) override
    {
        CanFrame frame{};

        uint8_t processed = 0;

        while (
            processed < MAX_FRAMES_PER_UPDATE &&
            m_bus.read(frame)
        )
        {
            ++processed;

            if (m_left.accepts(frame))
            {
                m_leftFeedback.publish(m_left.decode(frame));
            }
            else if (m_right.accepts(frame))
            {
                m_rightFeedback.publish(m_right.decode(frame));
            }
        }
    }

private:

    static constexpr uint8_t
        MAX_FRAMES_PER_UPDATE = 8; // I would raise this

    CanBus& m_bus;

    MotorDriver& m_left;
    MotorDriver& m_right;

    Topic<MotorFeedback>& m_leftFeedback;
    Topic<MotorFeedback>& m_rightFeedback;
};

class DummyController final :
    public ITask
{
public:
    DummyController(
        Topic<EncoderPositions>& encoders,
        Topic<LoadCellTorques>& loadCells,
        Topic<MotorFeedback>& leftMotor,
		Topic<MotorFeedback>& rightMotor,
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
        const Topic<MotorFeedback>& topic)
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

        //Serial.print("pos=");
        //Serial.print(value.position);

        //Serial.print(" vel=");
        //Serial.print(value.velocity);

        Serial.print(" torque=");
        Serial.print(value.torque);

        Serial.print(" temp=");
        Serial.print(value.temperature);

        Serial.print(" error=");
        Serial.println(value.error);
    }

private:
    Topic<EncoderPositions>& m_encoders;
    Topic<LoadCellTorques>& m_loadCells;
    Topic<MotorFeedback>& m_leftMotor;
    Topic<PowerReadings>& m_power;
    Topic<MotorFeedback>& m_rightMotor;

    uint32_t m_previousPrintUs{0};
};


// =========== JOINT LIMIT CONTROLLER = POSITION CONTROL ============

template<Side side>
class JointLimitController final :
    public ITask
{
public:
    JointLimitController(
        Topic<EncoderPositions>& encoders,
        Topic<bool>& motorControl)
        :
        m_encoders(encoders),
        m_motorControl(motorControl)
    {}


    void update(uint32_t nowUs) override
    {
        static constexpr uint32_t
            PERIOD_US = 5'000; // 200 Hz

        if (nowUs - m_previousUpdateUs < PERIOD_US)
        {
            return;
        }

        m_previousUpdateUs = nowUs;


        if (!m_encoders.valid())
        {
            return;
        }


        const float position = getPosition(m_encoders.latest());
        /*
         * Absolute limit:
         * immediately request that the
         * selected motor exits motor mode.
         */
        if (fabsf(position) >= ABSOLUTE_LIMIT)
        {
            if (!m_limitTriggered)
            {
                m_motorControl.publish(false);
                m_limitTriggered = true;
                Serial.print(sideName());
                Serial.println("absolute joint limit breached");
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
        if (fabsf(position) >=SOFTWARE_LIMIT)
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
    //to fix : think about moving this to Encoders.h

    static float getPosition(const EncoderPositions& encoders)
    {
        if constexpr (
            side == Side::Left
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
            side == Side::Left
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

    Topic<bool>& m_motorControl;


    uint32_t
        m_previousUpdateUs{0};

    bool
        m_limitTriggered{false};
};


// NEW FOR NOW : TORQUE BANDWIDTH CONTROLLER FOR BANDWIDTH EVALUATION
/*
template<ExoSide Side>
class TorqueBandwidthController final :
    public ITask
{
public:

    explicit TorqueBandwidthController(
        Topic<float>& motorCmd,
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

        const float phase = TWO_PI*frequencyHz*elapsedSeconds;


        const float torque =
            m_amplitudeNm*sinf(phase);


        float command{};

        command = torque;


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

        m_motorCmd.publish(0.0f);
    }



    static constexpr const char*
    sideName()
    {
        if constexpr (
            Side == Side::Left
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
    // STATE
    // =====================================================

    Topic<float>&
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
*/
#else
#endif
