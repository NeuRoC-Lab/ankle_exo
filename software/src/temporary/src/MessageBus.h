#pragma once

#include <Arduino.h>

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
    EndpointId::RightMotorCommand>
{
    using Payload = MotorCmd;
};

template<>
struct EndpointTraits<
    EndpointId::Ina232Snapshot>
{
    using Payload = PowerReadings;
};


template<typename T>
class Topic
{
public:
    using ValueType = T;

    void publish(const T& value)
    {
        m_value = value;
        ++m_sequence;
        m_valid = true;
    }

    const T& latest() const
    {
        return m_value;
    }

    bool valid() const
    {
        return m_valid;
    }

    uint32_t sequence() const
    {
        return m_sequence;
    }

private:
    T m_value{};
    uint32_t m_sequence{0};
    bool m_valid{false};
};

class IRoutedTopic
{
public:
    virtual ~IRoutedTopic() = default;

    virtual EndpointId id() const = 0;

    virtual bool receive(
        const BusMessage& message) = 0;
};

template<EndpointId Id>
class RoutedTopic final :
    public IRoutedTopic
{
public:
    using Payload =
        typename EndpointTraits<Id>::Payload;

    explicit RoutedTopic(
        Topic<Payload>& topic)
        : m_topic(topic)
    {}

    EndpointId id() const override
    {
        return Id;
    }

    bool receive(
        const BusMessage& message) override
    {
        static_assert(
            std::is_trivially_copyable_v<
                Payload
            >,
            "Payload must be trivially copyable"
        );

        if (message.header.topic != Id ||
            message.payloadSize !=
                sizeof(Payload))
        {
            return false;
        }

        Payload value{};

        std::memcpy(
            &value,
            message.payload.data(),
            sizeof(Payload)
        );

        m_topic.publish(value);

        return true;
    }

private:
    Topic<Payload>& m_topic;
};

using PlatformMask = uint8_t;

constexpr PlatformMask platformBit(
    PlatformId platform)
{
    return static_cast<PlatformMask>(
        1U <<
        static_cast<uint8_t>(platform)
    );
}

struct TopicRoute
{
    PlatformMask subscribers{0};
};

inline constexpr std::array<
    TopicRoute,
    EndpointCount
> TopicRoutes = []()
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
            EndpointId::LeftMotorCommand
        )
    ].subscribers =
        platformBit(PlatformId::Teensy);

    routes[
        static_cast<size_t>(
            EndpointId::RightMotorCommand
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

    bool addTopic(
        IRoutedTopic& topic)
    {
        const size_t index =
            static_cast<size_t>(
                topic.id()
            );

        if (index >=
            m_topics.size())
        {
            return false;
        }

        m_topics[index] = &topic;
        return true;
    }

    template<EndpointId Id>
    bool publish(
        const typename
            EndpointTraits<Id>::Payload&
                value)
    {
        using Payload =
            typename EndpointTraits<
                Id
            >::Payload;

        static_assert(
            std::is_trivially_copyable_v<
                Payload
            >
        );

        static_assert(
            sizeof(Payload) <=
            MaxPayloadSize
        );

        BusMessage message{};

        message.header.topic = Id;
        message.header.origin =
            LocalPlatform;

        message.header.sequence =
            ++m_sequences[
                static_cast<size_t>(Id)
            ];

        message.payloadSize =
            sizeof(Payload);

        std::memcpy(
            message.payload.data(),
            &value,
            sizeof(Payload)
        );

        dispatchLocal(message);

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
            (
                route.subscribers &
                platformBit(
                    remotePlatform()
                )
            )
            != 0;
    }

    void dispatchLocal(
        const BusMessage& message)
    {
        const size_t index =
            static_cast<size_t>(
                message.header.topic
            );

        if (index >=
            m_topics.size())
        {
            return;
        }

        IRoutedTopic* topic =
            m_topics[index];

        if (topic != nullptr)
        {
            topic->receive(message);
        }
    }

    void receiveRemote(
        const BusMessage& message)
    {
        if (message.header.origin ==
            LocalPlatform)
        {
            return;
        }

        m_rxQueue.push(message);
    }

private:
    UARTHandler& m_uart;

    std::array<
        IRoutedTopic*,
        EndpointCount
    > m_topics{};

    std::array<
        uint16_t,
        EndpointCount
    > m_sequences{};

    FixedQueue<
        BusMessage,
        16
    > m_rxQueue;
};

template<
    typename DriverT,
    EndpointId Id,
    uint32_t PeriodUs
>
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
