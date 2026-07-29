#pragma once

#include <Arduino.h>
#include <PacketSerial.h>
#include <ArduinoJson.h>
#include "ProtocolTypes.h"
#include "SerialConfig.h"

class UARTHandler
{
public:
    using CommandHandler = void (*)(
        void* context,
        const CommandPayload& command
    );

#if defined(PLATFORM_TEENSY41)

    UARTHandler(
        HardwareSerial& serial,
        CommandHandler commandHandler,
        void* commandContext
    )
        : m_serial(serial),
          m_commandHandler(commandHandler),
          m_commandContext(commandContext)
    {
        s_instance = this;
    }

#elif defined(PLATFORM_NORDIC)



    explicit UARTHandler(
        HardwareSerial& serial,
        DataPayload& payload
    )
    : m_serial(serial),
      m_payload(payload)
    {
        s_instance = this;
    }

#else
#error "Unsupported platform"
#endif

    void begin()
    {
        m_packetSerial.setStream(&m_serial);
        m_packetSerial.setPacketHandler(packetReceivedCallback);
    }

    void update()
    {
        m_packetSerial.update();
    }

#if defined(PLATFORM_TEENSY41)

    void sendTelemetryPacket(const DataPayload& payload)
    {
        m_packetSerial.send(
            reinterpret_cast<const uint8_t*>(&payload),
            sizeof(payload)
        );
    }

#elif defined(PLATFORM_NORDIC)

    void sendCommandPacket(const CommandPayload& command)
    {
        m_packetSerial.send(
            reinterpret_cast<const uint8_t*>(&command),
            sizeof(command)
        );
    }

#endif

private:
    inline static UARTHandler* s_instance = nullptr;

    static void packetReceivedCallback(
        const uint8_t* buffer,
        size_t size
    )
    {
        if (s_instance != nullptr) {
            s_instance->handlePacketReceived(buffer, size);
        }
    }

    void handlePacketReceived(
        const uint8_t* buffer,
        size_t size
    )
    {
#if defined(PLATFORM_TEENSY41)

        if (size != sizeof(CommandPayload)) {
            return;
        }

        CommandPayload command {};

        memcpy(
            &command,
            buffer,
            sizeof(command)
        );

        if (m_commandHandler != nullptr) {
            m_commandHandler(
                m_commandContext,
                command
            );
        }

#elif defined(PLATFORM_NORDIC)

        if (size != sizeof(DataPayload)) {
            return;
        }

        memcpy(
            &m_payload,
            buffer,
            sizeof(m_payload)
        );

#endif
    }

    HardwareSerial& m_serial;
    PacketSerial m_packetSerial;

#if defined(PLATFORM_NORDIC)
    DataPayload& m_payload;
#endif

#if defined(PLATFORM_TEENSY41)
    CommandHandler m_commandHandler = nullptr;
    void* m_commandContext = nullptr;
#endif
};


#if defined(PLATFORM_NORDIC)
#include <ArduinoBLE.h>

JsonDocument createTelemetryPacket(const DataPayload& payload) {
    JsonDocument doc;

    doc[TelemetryKey::LeftLoadCell1] = payload.loadCells.LeftLoadCell1;
    doc[TelemetryKey::LeftLoadCell2] = payload.loadCells.LeftLoadCell2;
    doc[TelemetryKey::RightLoadCell1] = payload.loadCells.RightLoadCell1;
    doc[TelemetryKey::RightLoadCell2] = payload.loadCells.RightLoadCell2;

    doc[TelemetryKey::LeftEncoder] = payload.encoders.left_position;
    doc[TelemetryKey::RightEncoder] = payload.encoders.right_position;

    JsonArray motors =
        doc[TelemetryKey::Motors].to<JsonArray>();

    JsonObject motor1Object = motors.add<JsonObject>();
    motor1Object[TelemetryKey::MotorId] = payload.motorRep.can_id;
    motor1Object[TelemetryKey::MotorPos] = payload.motorRep.position;
    motor1Object[TelemetryKey::MotorVel] = payload.motorRep.velocity;
    motor1Object[TelemetryKey::MotorTrq] = payload.motorRep.torque;
    motor1Object[TelemetryKey::MotorTemp] = payload.motorRep.temperature;
    motor1Object[TelemetryKey::MotorErr] = payload.motorRep.error;

    return doc;
}
void printPayload(const DataPayload& payload)
{
    Serial.println("----- DataPayload -----");

    Serial.println("Load cells:");
    Serial.print("  Left 1:  ");
    Serial.println(payload.loadCells.LeftLoadCell1, 6);

    Serial.print("  Left 2:  ");
    Serial.println(payload.loadCells.LeftLoadCell2, 6);

    Serial.print("  Right 1: ");
    Serial.println(payload.loadCells.RightLoadCell1, 6);

    Serial.print("  Right 2: ");
    Serial.println(payload.loadCells.RightLoadCell2, 6);

    Serial.println("Encoders:");
    Serial.print("  Left position:  ");
    Serial.println(payload.encoders.left_position);

    Serial.print("  Right position: ");
    Serial.println(payload.encoders.right_position);

    Serial.println("Motor:");
    Serial.print("  CAN ID:      ");
    Serial.println(payload.motorRep.can_id);

    Serial.print("  Position:    ");
    Serial.println(payload.motorRep.position, 6);

    Serial.print("  Velocity:    ");
    Serial.println(payload.motorRep.velocity, 6);

    Serial.print("  Torque:      ");
    Serial.println(payload.motorRep.torque, 6);

    Serial.print("  Temperature: ");
    Serial.println(payload.motorRep.temperature);

    Serial.print("  Error:       ");
    Serial.println(payload.motorRep.error);

    Serial.println("-----------------------");
}

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

        m_uart.sendCommandPacket(command);

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
