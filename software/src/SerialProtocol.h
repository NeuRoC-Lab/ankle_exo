#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <variant>
#include <cmath>

#include <Arduino.h>


#include <PacketSerial.h>
#include "ProtocolTypes.h"

#if defined(PLATFORM_TEENSY41)
#include "CANMotorMIT.h"
#endif

#if defined(PLATFORM_NORDIC)
#include <ArduinoBLE.h>
#endif

using Message =
    std::variant<DataPayload, CommandPayload>;
// std::variant was introduced in C++ 2017 and is a great way to combine DataPayload and CommandPayload into one entity

class UARTHandler
{
public:
    explicit UARTHandler(HardwareSerial& serial)
        : m_serial(serial)
    {
        if (s_instance == nullptr) {
            s_instance = this;
        }
    }

    virtual ~UARTHandler()
    {
        if (s_instance == this) {
            s_instance = nullptr;
        }
    }

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

    virtual void onReceive(const Message& message) = 0;

    template<typename Payload>
    void send(const Payload& payload)
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
         // copy header into memory address

        std::memcpy(
            packet.data() + sizeof(header),
            &payload,
            sizeof(payload)
        );
        // copy payload into memory address adjacent to header

        m_packetSerial.send(
            packet.data(),
            packet.size()
        );
    }

protected:
    bool decodeMessage(
        const uint8_t* buffer,
        size_t size,
        Message& message
    )
    {
        if (buffer == nullptr || size < sizeof(MessageHeader)) {
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
    static constexpr MessageType getMessageType()
    {
        if constexpr (std::is_same_v<Payload, DataPayload>) {
            return MessageType::Telemetry;
        }
        else {
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
    }

private:
    CANMotorMIT_Handler& m_motorHandler;
};

#elif defined(PLATFORM_NORDIC)

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

        if (telemetry == nullptr)
        {
            return;
        }

        // Start with the newly received payload.
        DataPayload validatedTelemetry = *telemetry;

        keepOldIfInvalid(
            validatedTelemetry.loadCells.LeftLoadCell1,
            m_dataBuffer.loadCells.LeftLoadCell1
        );

        keepOldIfInvalid(
            validatedTelemetry.loadCells.LeftLoadCell2,
            m_dataBuffer.loadCells.LeftLoadCell2
        );

        keepOldIfInvalid(
            validatedTelemetry.loadCells.RightLoadCell1,
            m_dataBuffer.loadCells.RightLoadCell1
        );

        keepOldIfInvalid(
            validatedTelemetry.loadCells.RightLoadCell2,
            m_dataBuffer.loadCells.RightLoadCell2
        );

        // Motor and encoder data are updated normally.
        // Invalid load-cell values have been replaced with old values.
        m_dataBuffer = validatedTelemetry;
    }

private:
    DataPayload& m_dataBuffer;

    static constexpr float minimumValidForce = -500.0f;
    static constexpr float maximumValidForce = 500.0f;

    static bool isValidForce(float force)
    {
        return std::isfinite(force)
            && force >= minimumValidForce
            && force <= maximumValidForce;
    }

    static void keepOldIfInvalid(
        float& receivedForce,
        float previousForce
    )
    {
        if (!isValidForce(receivedForce))
        {
            receivedForce = previousForce;
        }
    }

};

#endif

#if defined(PLATFORM_NORDIC)

// This is a Arduino Nano-specific method for forwarding byte-for-byte commands sent from the BLE Central.


// on mac generate 128bit UUIDs with uuidgen command
constexpr char dataServiceUUID[] = "CF45813E-4358-4903-B961-09996BB081FB";
constexpr char LLCCharacteristicUUID[] = "CA87289F-102B-4078-AD8C-8F53063547A6";
constexpr char motorCharacteristicUUID[] = "E0D883F6-705C-4A11-B117-E2B0909CC68E";
constexpr char encoderCharacteristicUUID[] = "094A717B-0C7F-4A23-BFD1-A4924E6E7DAB";

constexpr char commandCharacteristicUUID[] = "C94B7403-6BFB-4A06-BA12-6394765C328E";

void blePeripheralConnectHandler(BLEDevice central) {
    // central connected event handler
    digitalWrite(LED_BUILTIN,HIGH);
    Serial.print("Connected event, central: ");
    Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
    // central disconnected event handler
    digitalWrite(LED_BUILTIN,LOW);
    Serial.print("Disconnected event, central: ");
    Serial.println(central.address());
}


class BLEHandler
{
public:
    BLEHandler(
        DataPayload& payload,
        UARTHandler& uart
    )
        : m_payload(payload),
          m_uart(uart),
          m_dataService(dataServiceUUID),
          m_loadCellCharacteristic(
              LLCCharacteristicUUID,
              BLERead | BLENotify,
              sizeof(LoadCellVoltages)
          ),
          m_motorCharacteristic(
              motorCharacteristicUUID,
              BLERead | BLENotify,
              sizeof(MotorReply)
          ),
          m_encoderCharacteristic(
              encoderCharacteristicUUID,
              BLERead | BLENotify,
              sizeof(EncoderPositions)
          ),
          m_commandCharacteristic(
              commandCharacteristicUUID,
              BLEWrite | BLEWriteWithoutResponse,
              sizeof(CommandPayload)
          )
    {
        s_instance = this;
    }

    bool begin()
    {
        pinMode(LED_BUILTIN, OUTPUT);

        if (!BLE.begin()) {
            return false;
        }

        BLE.setEventHandler(
            BLEConnected,
            blePeripheralConnectHandler
        );

        BLE.setEventHandler(
            BLEDisconnected,
            blePeripheralDisconnectHandler
        );

        m_commandCharacteristic.setEventHandler(
            BLEWritten,
            commandWrittenCallback
        );

        BLE.setLocalName("AnkleExo");
        BLE.setAdvertisedService(m_dataService);

        m_dataService.addCharacteristic(
            m_loadCellCharacteristic
        );

        m_dataService.addCharacteristic(
            m_motorCharacteristic
        );

        m_dataService.addCharacteristic(
            m_encoderCharacteristic
        );

        m_dataService.addCharacteristic(
            m_commandCharacteristic
        );

        BLE.addService(m_dataService);
        BLE.advertise();

        return true;
    }

    void update()
    {
        BLE.poll();

        BLEDevice central = BLE.central();

        if (!central) {
            return;
        }

        m_motorCharacteristic.writeValue(
            reinterpret_cast<const uint8_t*>(
                &m_payload.motorRep
            ),
            sizeof(m_payload.motorRep)
        );

        m_loadCellCharacteristic.writeValue(
            reinterpret_cast<const uint8_t*>(
                &m_payload.loadCells
            ),
            sizeof(m_payload.loadCells)
        );

        m_encoderCharacteristic.writeValue(
            reinterpret_cast<const uint8_t*>(
                &m_payload.encoders
            ),
            sizeof(m_payload.encoders)
        );
    }

private:
    inline static BLEHandler* s_instance = nullptr;

    static void commandWrittenCallback(
        BLEDevice central,
        BLECharacteristic characteristic
    )
    {
        if (s_instance != nullptr) {
            s_instance->forwardCommand(
                central,
                characteristic
            );
        }
    }

    void forwardCommand(
        BLEDevice central,
        BLECharacteristic characteristic
    )
    {
        (void)central;

        CommandPayload command {};

        const int bytesRead = characteristic.readValue(
            reinterpret_cast<uint8_t*>(&command),
            sizeof(command)
        );

        if (
            bytesRead
            != static_cast<int>(sizeof(command))
        ) {
            Serial.print("Wrong command size: ");
            Serial.print(bytesRead);
            Serial.print(", expected: ");
            Serial.println(sizeof(command));
            return;
        }

        m_uart.send(command);

        Serial.println("Command forwarded to Teensy");
    }

    DataPayload& m_payload;
    UARTHandler& m_uart;

    BLEService m_dataService;

    BLECharacteristic m_loadCellCharacteristic;
    BLECharacteristic m_motorCharacteristic;
    BLECharacteristic m_encoderCharacteristic;
    BLECharacteristic m_commandCharacteristic;
};

#endif
