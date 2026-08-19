#pragma once

#include <Arduino.h>

#include <functional>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "BusTypes.h"
#include "Encoder.h"
#include "LoadCell.h"
//#include "CubeMarsMotor.h"
#include "CubeMarsMotorServo.h"
#include "UartHandler.h"
#include "Driver.h"


#if defined(PLATFORM_NANO)

inline constexpr PlatformId LocalPlatform =
    PlatformId::Nano;

#elif defined(PLATFORM_TEENSY)

inline constexpr PlatformId LocalPlatform =
    PlatformId::Teensy;

#else

#error "No supported platform selected"

#endif

class ITask
{
public:
    virtual ~ITask() = default;
    virtual void update(uint32_t nowUs) = 0;
};

template<typename T>
/**
 * @brief Publishes a new value to the topic.
 *
 * Replaces the previously stored value and increments the
 * topic sequence number.
 *
 * @param value Value to publish.
 */
class Topic
{
public:
    using ValueType = T;

    /**
     * @brief Publishes a value to the topic (e.g updating a sensor topic from the driver)
     *
     * @return None
     */
    void publish(const T& value)
    {
        m_value = value;
        ++m_sequence; // version coujnter for checking if value updated
        m_valid = true;
    }

    /**
     * @brief Returns the latest published value.
     *
     * @return Reference to the latest value stored in the topic.
     */
    const T& latest() const
    {
        return m_value;
    }
    /**
     * @brief Indicates if data is valid
     *
     * @return booleam
     */
    bool valid() const
    {
        return m_valid;
    }

    /**
     * @brief Sequence index
     *
     * @return 32bit unsigned int
     */
    uint32_t sequence() const
    {
        return m_sequence;
    }

private:
    T m_value{};
    uint32_t m_sequence{0};
    bool m_valid{false};
};



template<typename T, size_t Capacity>
class FixedQueue
{
public:
    bool push(const T& value)
    {
        if (m_count >= Capacity) {
            return false;
        }

        m_data[m_tail] = value;
        m_tail =
            (m_tail + 1) % Capacity;

        ++m_count;
        return true;
    }

    bool pop(T& value)
    {
        if (m_count == 0) {
            return false;
        }

        value = m_data[m_head];
        m_head =
            (m_head + 1) % Capacity;

        --m_count;
        return true;
    }

private:
    std::array<T, Capacity> m_data{};
    size_t m_head{0};
    size_t m_tail{0};
    size_t m_count{0};
};

class MessageBus final : public ITask
{
public:
    explicit MessageBus(
        UARTHandler& uart)
        : m_uart(uart)
    {
        m_uart.setReceiveCallback(
            [](void* context,
               const BusMessage& message)
            {
                auto* bus =
                    static_cast<MessageBus*>(
                        context
                    );

                bus->receiveRemote(
                    message
                );
            },
            this
        );
    }

    void begin()
    {
        m_uart.begin();
    }

    template<EndpointId Id>
    bool addTopic(
        Topic<
            typename EndpointTraits<Id>::Payload
        >& topic)
        {
            using Payload =
                typename EndpointTraits<
                    Id
                >::Payload;

            static_assert(
                std::is_trivially_copyable_v<
                    Payload
                >,
                "Payload must be trivially copyable"
            );

            const size_t index =
                static_cast<size_t>(Id);

            if (index >= m_handlers.size())
            {
                return false;
            }

            m_handlers[index] =
                [&topic](
                    const BusMessage& message)
                {
                    if (
                        message.header.topic != Id
                        ||
                        message.payloadSize !=
                            sizeof(Payload)
                    )
                    {
                        return false;
                    }

                    Payload value{};

                    std::memcpy(
                        &value,
                        message.payload.data(),
                        sizeof(Payload)
                    );

                    topic.publish(value);

                    return true;
                };

            return true;
        }

    template<EndpointId Id>
    bool publish(const typename EndpointTraits<Id>::Payload& value)
    {
        using Payload =
            typename EndpointTraits<Id>::Payload;

        static_assert(
            std::is_trivially_copyable_v<Payload>
        );

        static_assert(
            sizeof(Payload) <= MaxPayloadSize
        );

        BusMessage message{};

        message.header.topic = Id;
        message.header.origin = LocalPlatform;

        message.header.sequence =++m_sequences[static_cast<size_t>(Id)];

        message.payloadSize =sizeof(Payload);

        std::memcpy(
            message.payload.data(),
            &value,
            sizeof(Payload)
        );

        dispatchLocal(message); // local dispatching (i.e on teensy if this code runs on teensy)

        if (remoteNeeds(Id))
        {
            return m_uart.send(message);
        }

        return true;
    }

    void update(uint32_t) override
    {
        // PacketSerial must be serviced continuously.
        m_uart.update();

        BusMessage message{};

        while (m_rxQueue.pop(message))
        {
        //to fix : make a ring buffer or bounded queue because the while loop might spend a variable amount of time draining which could explain why the BLE connection hangs up randomly
            dispatchLocal(message);
        }
    }

private:
    static constexpr
    PlatformId remotePlatform()
    {
        if constexpr (
            LocalPlatform ==
            PlatformId::Teensy)
        {
            return PlatformId::Nano;
        }
        else
        {
            return PlatformId::Teensy;
        }
    }

    bool remoteNeeds(
        EndpointId id) const
    {
        const auto& route =
            TopicRoutes[
                static_cast<size_t>(id)
            ];

        return
            (route.subscribers & platformBit(remotePlatform())) != 0;
        }

    void dispatchLocal(const BusMessage& message)
        {
            const size_t index =
                static_cast<size_t>(
                    message.header.topic
                );

            if (index >= m_handlers.size())
            {
                return;
            }

            auto& handler =
                m_handlers[index];

            if (handler)
            {
                handler(message);
            }
        }

    void receiveRemote(const BusMessage& message)
    {
        if (message.header.origin ==LocalPlatform)
        {
            return;
        }

        m_rxQueue.push(message);
    }

private:
    UARTHandler& m_uart;

    using MessageHandler =
    std::function<
        bool(const BusMessage&)
    >;

    std::array<
        MessageHandler,
        EndpointCount
    > m_handlers{};

    std::array<uint16_t,EndpointCount> m_sequences{};

    FixedQueue<BusMessage,16> m_rxQueue; // could have used std::queue but that container is unbounded which is risky for out embedded platforms
};

template<typename DriverT,EndpointId Id,uint32_t PeriodUs>
class PollingPublisher final :
    public ITask
{
public:
    using Payload =
        typename EndpointTraits<
            Id
        >::Payload;

    PollingPublisher(
        DriverT& driver,
        MessageBus& bus)
        : m_driver(driver),
          m_bus(bus)
    {
        static_assert(
            std::is_same_v<
                typename DriverT::ValueType,
                Payload
            >,
            "Driver output does not match topic payload"
        );
    }

    void update(
        uint32_t nowUs) override
    {
        if (nowUs - m_previousUs <
            PeriodUs)
        {
            return;
        }

        m_previousUs += PeriodUs;

        m_bus.publish<Id>(
            m_driver.sample()
        );
    }

private:
    DriverT& m_driver;
    MessageBus& m_bus;

    uint32_t m_previousUs{0};
};


#if defined(PLATFORM_TEENSY)


class SDLogger final :
    public ITask
{
public:

    SDLogger(
        SDCardDriver& sdCard,
        Topic<LoadCellTorques>& loadCells,
        Topic<EncoderPositions>& encoders,
        Topic<MotorFeedback>& leftMotor,
        Topic<MotorFeedback>& rightMotor,
        Topic<float>& leftMotorCommand,
        Topic<float>& rightMotorCommand,
        Topic<LoggingState>& loggingState)
        :
        m_sdCard(sdCard),
        m_loadCells(loadCells),
        m_encoders(encoders),
        m_leftMotor(leftMotor),
        m_rightMotor(rightMotor),
        m_leftMotorCommand(leftMotorCommand),
        m_rightMotorCommand(rightMotorCommand),
        m_loggingState(loggingState)
    {}


    void update(
        uint32_t nowUs) override
    {
        if (!m_sdCard.ready())
        {
            return;
        }


        if (!m_loggingState.valid())
        {
            return;
        }


        const LoggingState state =
            m_loggingState.latest();


        // =================================================
        // STATE CHANGES
        // =================================================

        if (
            state !=
            m_previousState
        )
        {
            handleStateChange(
                state
            );

            m_previousState =
                state;
        }


        if (
            state !=
            LoggingState::Recording
        )
        {
            return;
        }


        // =================================================
        // LOG AT FIXED TARGET RATE
        // =================================================

        if (
            nowUs -
            m_lastSampleUs
            <
            SAMPLE_PERIOD_US
        )
        {
            return;
        }


        /*
         * Advance by the desired period instead of:
         *
         *     m_lastSampleUs = nowUs;
         *
         * This avoids accumulating timing drift.
         */

        m_lastSampleUs +=
            SAMPLE_PERIOD_US;


        // =================================================
        // REQUIRE VALID DATA
        // =================================================
		/*
        if (
            !m_loadCells.valid() ||
            !m_encoders.valid() ||
            !m_leftMotor.valid() ||
            !m_rightMotor.valid() ||
            !m_leftMotorCommand.valid() ||
            !m_rightMotorCommand.valid()
        )
        {
            return;
        }
		*/


        // =================================================
        // BUILD BINARY RECORD
        // =================================================

        BinaryLogRecord record{};


        record.timeUs =
            nowUs;


        // -------------------------------------------------
        // Sequences
        // -------------------------------------------------

        record.loadCellSequence =
            m_loadCells.sequence();

        record.encoderSequence =
            m_encoders.sequence();

        record.leftMotorSequence =
            m_leftMotor.sequence();

        record.rightMotorSequence =
            m_rightMotor.sequence();

        record.leftCommandSequence =
            m_leftMotorCommand.sequence();

        record.rightCommandSequence =
            m_rightMotorCommand.sequence();


        // -------------------------------------------------
        // Latest values
        // -------------------------------------------------

        const auto& forces =
            m_loadCells.latest();

        const auto& encoders =
            m_encoders.latest();

        const auto& leftMotor =
            m_leftMotor.latest();

        const auto& rightMotor =
            m_rightMotor.latest();

        const auto& leftCommand =
            m_leftMotorCommand.latest();

        const auto& rightCommand =
            m_rightMotorCommand.latest();


        // -------------------------------------------------
        // Load cells
        // -------------------------------------------------

        record.loadCells[0] =
            forces[0];

        record.loadCells[1] =
            forces[1];

        record.loadCells[2] =
            forces[2];

        record.loadCells[3] =
            forces[3];


        // -------------------------------------------------
        // Encoders
        // -------------------------------------------------

        record.encoderLeft =
            encoders.left;

        record.encoderRight =
            encoders.right;


        // -------------------------------------------------
        // Commands
        // -------------------------------------------------

        record.leftCommandTorque = leftCommand;

        record.rightCommandTorque = rightCommand;


        // -------------------------------------------------
        // Motor feedback
        // -------------------------------------------------

        record.leftMotorTorque =
            leftMotor.torque;

        record.rightMotorTorque =
            rightMotor.torque;


        record.leftMotorTemperature =
            leftMotor.temperature;

        record.leftMotorError =
            leftMotor.error;


        record.rightMotorTemperature =
            rightMotor.temperature;

        record.rightMotorError =
            rightMotor.error;


        // =================================================
        // COPY INTO RAM BUFFER
        // =================================================

        m_buffer[
            m_bufferCount
        ] = record;


        ++m_bufferCount;


        // =================================================
        // WRITE ONLY WHEN BUFFER IS FULL
        // =================================================

        if (
            m_bufferCount >=
            BUFFER_RECORD_COUNT
        )
        {
            writeBuffer();
        }
    }


private:

    void writeBuffer()
    {
        if (
            m_bufferCount == 0
        )
        {
            return;
        }


        const size_t bytes =
            m_bufferCount *
            sizeof(BinaryLogRecord);


        if (
            !m_sdCard.append(
                m_buffer.data(),
                bytes
            )
        )
        {
            ++m_writeErrors;

            Serial.println(
                "SD binary write failed"
            );

            /*
             * Don't retry forever here.
             *
             * A retry inside the realtime scheduler
             * could make timing even worse.
             */
        }


        m_bufferCount = 0;


        ++m_blockWriteCount;
    }


    void handleStateChange(
        LoggingState newState)
    {
        switch (newState)
        {
            case LoggingState::Recording:
            {
                Serial.println(
                    "SD binary logging started"
                );


                m_bufferCount = 0;

                m_lastSampleUs =
                    micros();

                break;
            }


            case LoggingState::Stopped:
            {
                Serial.println(
                    "SD binary logging stopped"
                );


                /*
                 * Write remaining records that did not
                 * fill the complete buffer.
                 */

                writeBuffer();


                /*
                 * Flush only when stopping.
                 *
                 * Avoid frequent flush() calls during
                 * realtime operation.
                 */

                m_sdCard.flush();

                break;
            }
        }
    }


private:

    // =====================================================
    // LOGGING RATE
    // =====================================================

    static constexpr uint32_t
        SAMPLE_PERIOD_US =
            1'000;        // 1 kHz


    // =====================================================
    // BUFFER
    // =====================================================

    static constexpr size_t
        BUFFER_RECORD_COUNT =
            128;


    std::array<
        BinaryLogRecord,
        BUFFER_RECORD_COUNT
    >
        m_buffer{};


    size_t
        m_bufferCount{0};


    // =====================================================
    // REFERENCES
    // =====================================================

    SDCardDriver& m_sdCard;
    Topic<LoadCellTorques>& m_loadCells;
    Topic<EncoderPositions>& m_encoders;
    Topic<MotorFeedback>& m_leftMotor;
	Topic<MotorFeedback>& m_rightMotor;
    Topic<float>& m_leftMotorCommand;
    Topic<float>& m_rightMotorCommand;
    Topic<LoggingState>& m_loggingState;


    // =====================================================
    // STATE
    // =====================================================

    uint32_t
        m_lastSampleUs{0};


    LoggingState
        m_previousState{
            LoggingState::Stopped
        };


    // =====================================================
    // DEBUG COUNTERS
    // =====================================================

    uint32_t
        m_blockWriteCount{0};


    uint32_t
        m_writeErrors{0};
};

template<EndpointId Id>
class TopicForwarder final : public ITask
{
public:
    using Payload =
        typename EndpointTraits<Id>::Payload;

    TopicForwarder(
        Topic<Payload>& topic,
        MessageBus& bus)
        :
        m_topic(topic),
        m_bus(bus)
    {}

    void update(uint32_t) override
    {
        if (!m_topic.valid()) {
            return;
        }

        if (m_topic.sequence() == m_lastSequence) {
            return;
        }

        m_lastSequence = m_topic.sequence();

        m_bus.publish<Id>(
            m_topic.latest()
        );
    }

private:
    Topic<Payload>& m_topic;
    MessageBus& m_bus;
    uint32_t m_lastSequence{0};
};

#endif