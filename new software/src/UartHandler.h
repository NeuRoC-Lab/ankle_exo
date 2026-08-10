#pragma once

#include <Arduino.h>
#include <PacketSerial.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "BusTypes.h"

class UARTHandler
{
public:
    using ReceiveCallback =
        void(*)(void* context, const BusMessage&);

    explicit UARTHandler(HardwareSerial& serial)
        : m_serial(serial)
    {
        s_instance = this;
    }

    ~UARTHandler()
    {
        if (s_instance == this) {
            s_instance = nullptr;
        }
    }

    void setReceiveCallback(
        ReceiveCallback callback,
        void* context)
    {
        m_callback = callback;
        m_context = context;
    }

    void begin()
    {
        m_packetSerial.setStream(&m_serial);
        m_packetSerial.setPacketHandler(
            packetReceivedCallback
        );
    }

    bool send(const BusMessage& msg)
    {
        constexpr size_t HeaderSize = 7;
        constexpr size_t MaxPacketSize =
            HeaderSize + MaxPayloadSize;

        if (msg.payloadSize > MaxPayloadSize) {
            return false;
        }

        std::array<uint8_t, MaxPacketSize> packet{};

        size_t offset = 0;

        packet[offset++] =
            static_cast<uint8_t>(
                msg.header.topic
            );

        packet[offset++] =
            static_cast<uint8_t>(
                msg.header.origin
            );

        packet[offset++] =
            static_cast<uint8_t>(
                msg.header.sequence & 0xFF
            );

        packet[offset++] =
            static_cast<uint8_t>(
                (msg.header.sequence >> 8) & 0xFF
            );

        packet[offset++] =
            static_cast<uint8_t>(
                msg.payloadSize & 0xFF
            );

        packet[offset++] =
            static_cast<uint8_t>(
                (msg.payloadSize >> 8) & 0xFF
            );

        // reserved byte for future protocol version/flags
        packet[offset++] = 0;

        std::memcpy(
            packet.data() + offset,
            msg.payload.data(),
            msg.payloadSize
        );

        offset += msg.payloadSize;

        m_packetSerial.send(
            packet.data(),
            offset
        );

        return true;
    }

    void update()
    {
        m_packetSerial.update();
    }

private:
    static void packetReceivedCallback(
        const uint8_t* buffer,
        size_t size)
    {
        if (s_instance != nullptr)
        {
            s_instance->onPacketReceived(
                buffer,
                size
            );
        }
    }

    void onPacketReceived(
        const uint8_t* buffer,
        size_t size)
    {
        constexpr size_t HeaderSize = 7;

        if (size < HeaderSize) {
            return;
        }

        BusMessage message{};

        size_t offset = 0;

        message.header.topic =
            static_cast<EndpointId>(
                buffer[offset++]
            );

        message.header.origin =
            static_cast<PlatformId>(
                buffer[offset++]
            );

        message.header.sequence =
            static_cast<uint16_t>(
                buffer[offset]
            )
            |
            (
                static_cast<uint16_t>(
                    buffer[offset + 1]
                )
                << 8
            );

        offset += 2;

        message.payloadSize =
            static_cast<uint16_t>(
                buffer[offset]
            )
            |
            (
                static_cast<uint16_t>(
                    buffer[offset + 1]
                )
                << 8
            );

        offset += 2;

        // reserved byte
        ++offset;

        if (message.payloadSize >
            MaxPayloadSize)
        {
            return;
        }

        if (size !=
            HeaderSize +
            message.payloadSize)
        {
            return;
        }

        std::memcpy(
            message.payload.data(),
            buffer + offset,
            message.payloadSize
        );

        if (m_callback != nullptr)
        {
            m_callback(
                m_context,
                message
            );
        }
    }

private:
    inline static UARTHandler* s_instance = nullptr;

    HardwareSerial& m_serial;
    PacketSerial m_packetSerial;

    ReceiveCallback m_callback{nullptr};
    void* m_context{nullptr};
};
