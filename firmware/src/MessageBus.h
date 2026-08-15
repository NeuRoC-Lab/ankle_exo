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
#include "CubeMarsMotor.h"
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

template<EndpointId Id>
struct EndpointTraits;

template<>
struct EndpointTraits<
    EndpointId::EncoderSnapshot>
{
    using Payload = EncoderPositions;
};

template<>
struct EndpointTraits<
    EndpointId::LoadCellSnapshot>
{
    using Payload = LoadCellForces;
};

template<>
struct EndpointTraits<
    EndpointId::LeftMotorSnapshot>
{
    using Payload = MotorReply;
};

template<>
struct EndpointTraits<
    EndpointId::RightMotorSnapshot>
{
    using Payload = MotorReply;
};

template<>
struct EndpointTraits<
    EndpointId::LeftMotorCommand>
{
    using Payload = MotorCmd;
};


template<>
struct EndpointTraits<
    EndpointId::Ina232Snapshot>
{
    using Payload = PowerReadings;
};

template<>
struct EndpointTraits<
    EndpointId::LeftMotorMetaCommand>
{
    using Payload = MotorMetaCommand;
};
template<>
struct EndpointTraits<
    EndpointId::LoggingState>
{
    using Payload = LoggingState;
};

template<>
struct EndpointTraits<
    EndpointId::RightMotorCommand>
{
    using Payload = MotorCmd;
};

template<>
struct EndpointTraits<
    EndpointId::RightMotorMetaCommand>
{
    using Payload = MotorMetaCommand;
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




using PlatformMask = uint8_t; // a mask for indicating if a topic belongs to teensy or nano

constexpr PlatformMask platformBit(
    PlatformId platform)
{
    return static_cast<PlatformMask>(1U << static_cast<uint8_t>(platform));
}

struct TopicRoute
{
    PlatformMask subscribers{0};
};

inline constexpr std::array<TopicRoute,EndpointCount> TopicRoutes = []()
{
    std::array<
        TopicRoute,
        EndpointCount
    > routes{};

    const PlatformMask both =
        platformBit(PlatformId::Teensy) |
        platformBit(PlatformId::Nano);

    routes[
        static_cast<size_t>(
            EndpointId::EncoderSnapshot
        )
    ].subscribers = both;

    routes[
        static_cast<size_t>(
            EndpointId::LoadCellSnapshot
        )
    ].subscribers = both;

    routes[
        static_cast<size_t>(
            EndpointId::LeftMotorSnapshot
        )
    ].subscribers = both;

    routes[
        static_cast<size_t>(
            EndpointId::RightMotorSnapshot
        )
    ].subscribers = both;

	routes[
    static_cast<size_t>(
        EndpointId::RightMotorCommand
    )
	].subscribers =
    platformBit(
        PlatformId::Teensy
    );

	routes[
    static_cast<size_t>(
        EndpointId::RightMotorMetaCommand
    )
	].subscribers =
    platformBit(
        PlatformId::Teensy
    );

    routes[
        static_cast<size_t>(
            EndpointId::LeftMotorCommand
        )
    ].subscribers =
        platformBit(PlatformId::Teensy);

    routes[
        static_cast<size_t>(
            EndpointId::Ina232Snapshot
        )
    ].subscribers =
        platformBit(PlatformId::Teensy) |
        platformBit(PlatformId::Nano);

    routes[
        static_cast<size_t>(
            EndpointId::LeftMotorMetaCommand
        )
    ].subscribers =
        platformBit(PlatformId::Teensy);

	routes[
    	static_cast<size_t>(
        	EndpointId::LoggingState
    	)
	].subscribers =
    	platformBit(PlatformId::Teensy) |
		platformBit(PlatformId::Nano);

    return routes;
}();

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
        Topic<LoadCellForces>& loadCells,
        Topic<EncoderPositions>& encoders,
        Topic<MotorReply>& leftMotor,
        Topic<MotorReply>& rightMotor,
        Topic<MotorCmd>& leftMotorCommand,
        Topic<MotorCmd>& rightMotorCommand,
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


        if (state != m_previousState)
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
        // CURRENT TOPIC SEQUENCES
        // =================================================

        const uint32_t loadCellSequence =
            m_loadCells.sequence();

        const uint32_t encoderSequence =
            m_encoders.sequence();

        const uint32_t leftMotorSequence =
            m_leftMotor.sequence();

        const uint32_t rightMotorSequence =
            m_rightMotor.sequence();

        const uint32_t leftCommandSequence =
            m_leftMotorCommand.sequence();

        const uint32_t rightCommandSequence =
            m_rightMotorCommand.sequence();


        // =================================================
        // LOG WHEN ANY RELEVANT TOPIC CHANGES
        // =================================================

        const bool dataChanged =
            loadCellSequence !=
                m_lastLoadCellSequence
            ||
            encoderSequence !=
                m_lastEncoderSequence
            ||
            leftMotorSequence !=
                m_lastLeftMotorSequence
            ||
            rightMotorSequence !=
                m_lastRightMotorSequence
            ||
            leftCommandSequence !=
                m_lastLeftCommandSequence
            ||
            rightCommandSequence !=
                m_lastRightCommandSequence;


        if (!dataChanged)
        {
            return;
        }


        // =================================================
        // SAVE CURRENT SEQUENCES
        // =================================================

        m_lastLoadCellSequence =
            loadCellSequence;

        m_lastEncoderSequence =
            encoderSequence;

        m_lastLeftMotorSequence =
            leftMotorSequence;

        m_lastRightMotorSequence =
            rightMotorSequence;

        m_lastLeftCommandSequence =
            leftCommandSequence;

        m_lastRightCommandSequence =
            rightCommandSequence;


        // =================================================
        // LATEST VALUES
        // =================================================

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


        // =================================================
        // CSV
        //
        // time_us,
        //
        // load cells,
        // encoders,
        //
        // commanded torque,
        //
        // motor feedback
        // =================================================

        char line[320];


        const int length =
            snprintf(
                line,
                sizeof(line),

                "%lu,"

                "%.4f,%.4f,%.4f,%.4f,"

                "%.4f,%.4f,"

                "%.4f,%.4f,"

                "%.4f,%.4f,%.4f,%u,%u,"

                "%.4f,%.4f,%.4f,%u,%u",


                // -----------------------------------------
                // Time
                // -----------------------------------------

                static_cast<unsigned long>(
                    nowUs
                ),


                // -----------------------------------------
                // Load cells
                // -----------------------------------------

                static_cast<double>(
                    forces[0]
                ),

                static_cast<double>(
                    forces[1]
                ),

                static_cast<double>(
                    forces[2]
                ),

                static_cast<double>(
                    forces[3]
                ),


                // -----------------------------------------
                // Encoders
                // -----------------------------------------

                static_cast<double>(
                    encoders.left
                ),

                static_cast<double>(
                    encoders.right
                ),


                // -----------------------------------------
                // Commanded motor torques
                // -----------------------------------------

                static_cast<double>(
                    leftCommand.torque
                ),

                static_cast<double>(
                    rightCommand.torque
                ),


                // -----------------------------------------
                // Left motor feedback
                // -----------------------------------------

                static_cast<double>(
                    leftMotor.position
                ),

                static_cast<double>(
                    leftMotor.velocity
                ),

                static_cast<double>(
                    leftMotor.torque
                ),

                static_cast<unsigned int>(
                    leftMotor.temperature
                ),

                static_cast<unsigned int>(
                    leftMotor.error
                ),


                // -----------------------------------------
                // Right motor feedback
                // -----------------------------------------

                static_cast<double>(
                    rightMotor.position
                ),

                static_cast<double>(
                    rightMotor.velocity
                ),

                static_cast<double>(
                    rightMotor.torque
                ),

                static_cast<unsigned int>(
                    rightMotor.temperature
                ),

                static_cast<unsigned int>(
                    rightMotor.error
                )
            );


        if (
            length <= 0 ||
            static_cast<size_t>(length)
                >= sizeof(line)
        )
        {
            Serial.println(
                "SD log line too long"
            );

            return;
        }


        if (!m_sdCard.appendLine(line))
        {
            Serial.println(
                "SD logging failed"
            );
        }


        if (
            nowUs - m_lastFlushUs
            >= FLUSH_PERIOD_US
        )
        {
            m_sdCard.flush();

            m_lastFlushUs =
                nowUs;
        }
    }


private:

    void handleStateChange(
        LoggingState newState)
    {
        switch (newState)
        {
            case LoggingState::Recording:
            {
                Serial.println(
                    "SD logging started"
                );


                // Force a fresh row when a
                // new recording starts.

                m_lastLoadCellSequence = 0;
                m_lastEncoderSequence = 0;

                m_lastLeftMotorSequence = 0;
                m_lastRightMotorSequence = 0;

                m_lastLeftCommandSequence = 0;
                m_lastRightCommandSequence = 0;

                break;
            }


            case LoggingState::Stopped:
            {
                Serial.println(
                    "SD logging stopped"
                );

                m_sdCard.flush();

                break;
            }
        }
    }


private:

    static constexpr uint32_t
        FLUSH_PERIOD_US =
            100'000;


    SDCardDriver&
        m_sdCard;


    Topic<LoadCellForces>&
        m_loadCells;

    Topic<EncoderPositions>&
        m_encoders;


    Topic<MotorReply>&
        m_leftMotor;

    Topic<MotorReply>&
        m_rightMotor;


    Topic<MotorCmd>&
        m_leftMotorCommand;

    Topic<MotorCmd>&
        m_rightMotorCommand;


    Topic<LoggingState>&
        m_loggingState;


    uint32_t
        m_lastLoadCellSequence{0};

    uint32_t
        m_lastEncoderSequence{0};

    uint32_t
        m_lastLeftMotorSequence{0};

    uint32_t
        m_lastRightMotorSequence{0};

    uint32_t
        m_lastLeftCommandSequence{0};

    uint32_t
        m_lastRightCommandSequence{0};


    uint32_t
        m_lastFlushUs{0};


    LoggingState m_previousState{
        LoggingState::Stopped
    };
};

#endif