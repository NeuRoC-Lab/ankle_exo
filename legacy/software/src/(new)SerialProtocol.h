#pragma once

#include <Arduino.h>
#include <PacketSerial.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <variant>

#include "ProtocolTypes.h"

using Message =
    std::variant<DataPayload, CommandPayload>;

class UARTHandler
{
public:
    explicit UARTHandler(HardwareSerial& serial)
        : m_serial(serial)
    {
        s_instance = this;
    }

    virtual ~UARTHandler() = default;

    void begin()
    {
        m_packetSerial.setStream(&m_serial);
        m_packetSerial.setPacketHandler(
            packetReceivedCallback
        );
    }

    void update()
    {
        m_packetSerial.update();
    }

    virtual void onReceive(
        const Message& message
    ) = 0;

    template<typename Payload>
    bool send(const Payload& payload)
    {
        static_assert(
            std::is_same_v<Payload, DataPayload> ||
            std::is_same_v<Payload, CommandPayload>,
            "Unsupported UART payload type"
        );

        static_assert(
            std::is_trivially_copyable_v<Payload>,
            "Payload must be trivially copyable"
        );

        constexpr MessageType messageType =
            getMessageType<Payload>();

        constexpr size_t packetSize =
            sizeof(MessageHeader) +
            sizeof(Payload);

        std::array<uint8_t, packetSize> packet {};

        const MessageHeader header {
            .type = messageType,
            .payloadSize =
                static_cast<uint16_t>(sizeof(Payload))
        };

        std::memcpy(
            packet.data(),
            &header,
            sizeof(header)
        );

        std::memcpy(
            packet.data() + sizeof(header),
            &payload,
            sizeof(payload)
        );

        m_packetSerial.send(
            packet.data(),
            packet.size()
        );

        return true;
    }

protected:
    bool decodeMessage(
        const uint8_t* buffer,
        size_t size,
        Message& message
    )
    {
        if (
            buffer == nullptr ||
            size < sizeof(MessageHeader)
        ) {
            return false;
        }

        MessageHeader header {};

        std::memcpy(
            &header,
            buffer,
            sizeof(header)
        );

        const uint8_t* payloadBuffer =
            buffer + sizeof(MessageHeader);

        const size_t payloadSize =
            size - sizeof(MessageHeader);

        if (header.payloadSize != payloadSize) {
            return false;
        }

        switch (header.type) {
        case MessageType::Telemetry:
        {
            if (payloadSize != sizeof(DataPayload)) {
                return false;
            }

            DataPayload payload {};

            std::memcpy(
                &payload,
                payloadBuffer,
                sizeof(payload)
            );

            message = payload;
            return true;
        }

        case MessageType::Command:
        {
            if (payloadSize != sizeof(CommandPayload)) {
                return false;
            }

            CommandPayload payload {};

            std::memcpy(
                &payload,
                payloadBuffer,
                sizeof(payload)
            );

            message = payload;
            return true;
        }

        default:
            return false;
        }
    }

private:
    template<typename Payload>
    static consteval MessageType getMessageType()
    {
        if constexpr (
            std::is_same_v<Payload, DataPayload>
        ) {
            return MessageType::Telemetry;
        }
        else if constexpr (
            std::is_same_v<Payload, CommandPayload>
        ) {
            return MessageType::Command;
        }
    }

    static void packetReceivedCallback(
        const uint8_t* buffer,
        size_t size
    )
    {
        if (s_instance != nullptr) {
            s_instance->onPacketReceived(
                buffer,
                size
            );
        }
    }

    void onPacketReceived(
        const uint8_t* buffer,
        size_t size
    )
    {
        Message message {};

        if (!decodeMessage(buffer, size, message)) {
            return;
        }

        onReceive(message);
    }

    inline static UARTHandler* s_instance = nullptr;

    HardwareSerial& m_serial;
    PacketSerial m_packetSerial;
};

#if defined(PLATFORM_TEENSY41)

class UARTHandler_Teensy : public UARTHandler
{
public:
    UARTHandler_Teensy(
        HardwareSerial& serial,
        CANMotorMIT_Handler& motorHandler
    )
        : UARTHandler(serial),
          m_motorHandler(motorHandler)
    {
    }

    void onReceive(
        const Message& message
    ) override
    {
        const auto* command =
            std::get_if<CommandPayload>(&message);

        if (command == nullptr) {
            return;
        }

        m_motorHandler.handleSerialCommand(
            *command
        );

        m_motorHandler.update();
    }

private:
    CANMotorMIT_Handler& m_motorHandler;
};

#elif defined(PLATFORM_NANO)

class UARTHandler_Nano : public UARTHandler
{
public:
    UARTHandler_Nano(
        HardwareSerial& serial,
        DataPayload& dataBuffer
    )
        : UARTHandler(serial),
          m_dataBuffer(dataBuffer)
    {
    }

    void onReceive(
        const Message& message
    ) override
    {
        const auto* telemetry =
            std::get_if<DataPayload>(&message);

        if (telemetry == nullptr) {
            return;
        }

        m_dataBuffer = *telemetry;
    }

private:
    DataPayload& m_dataBuffer;
};

#endif