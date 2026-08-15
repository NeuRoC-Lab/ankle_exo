#pragma once

#if defined(PLATFORM_NANO)

#include <Arduino.h>
#include <ArduinoBLE.h>

#include "Driver.h"
#include "MessageBus.h"
#include "BusTypes.h"


// =========================================================
// BLE SERVICE
// =========================================================

constexpr char dataServiceUUID[] =
    "CF45813E-4358-4903-B961-09996BB081FB";


// =========================================================
// SHARED TELEMETRY UUIDs
// =========================================================

constexpr char loadCellCharacteristicUUID[] =
    "CA87289F-102B-4078-AD8C-8F53063547A6";

constexpr char encoderCharacteristicUUID[] =
    "094A717B-0C7F-4A23-BFD1-A4924E6E7DAB";

constexpr char powerCharacteristicUUID[] =
    "4D92C3F7-848C-42C2-B26A-9D1D15CB361A";


// =========================================================
// LEFT MOTOR UUIDs
// =========================================================

constexpr char leftMotorFeedbackCharacteristicUUID[] =
    "81DC2896-1B27-4195-A391-99A637FA50A4";

constexpr char leftMotorCommandCharacteristicUUID[] =
    "E0D883F6-705C-4A11-B117-E2B0909CC68E";

constexpr char leftMotorControlCharacteristicUUID[] =
    "99F02A66-065C-41BE-B05E-4BE2B9035A8B";


// =========================================================
// RIGHT MOTOR UUIDs
// =========================================================

constexpr char rightMotorFeedbackCharacteristicUUID[] =
    "2E38C871-902C-425F-8D3B-181CB21F0B67";

constexpr char rightMotorCommandCharacteristicUUID[] =
    "09DC04D0-BFC0-4D7C-A88D-96D60857FE64";

constexpr char rightMotorControlCharacteristicUUID[] =
    "12E89431-F2D4-4495-B984-A127A79D1591";


// =========================================================
// SD LOGGER UUID
// =========================================================

constexpr char loggingControlCharacteristicUUID[] =
    "A06EE428-0AB6-4CD4-AA8A-91619F1AF577";


// =========================================================
// BLE BRIDGE
// =========================================================

class BLEBridge final :
    public ITask
{
public:

    BLEBridge(
        MessageBus& bus,
        Topic<EncoderPositions>& encoder,
        Topic<LoadCellForces>& loadCells,
        Topic<MotorReply>& leftMotor,
        Topic<MotorReply>& rightMotor,
        Topic<PowerReadings>& power)
        :
        m_bus(bus),

        m_encoder(encoder),
        m_loadCells(loadCells),

        m_leftMotor(leftMotor),
        m_rightMotor(rightMotor),

        m_power(power),

        m_service(
            dataServiceUUID
        ),

        // -------------------------------------------------
        // Shared telemetry
        // -------------------------------------------------

        m_encoderCharacteristic(
            encoderCharacteristicUUID,
            BLERead | BLENotify,
            sizeof(EncoderPositions)
        ),

        m_loadCellCharacteristic(
            loadCellCharacteristicUUID,
            BLERead | BLENotify,
            sizeof(LoadCellForces)
        ),

        m_powerCharacteristic(
            powerCharacteristicUUID,
            BLERead | BLENotify,
            sizeof(PowerReadings)
        ),

        // -------------------------------------------------
        // Left motor
        // -------------------------------------------------

        m_leftMotorReplyCharacteristic(
            leftMotorFeedbackCharacteristicUUID,
            BLERead | BLENotify,
            sizeof(MotorReply)
        ),

        m_leftMotorCommandCharacteristic(
            leftMotorCommandCharacteristicUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(MotorCmd)
        ),

        m_leftMotorControlCharacteristic(
            leftMotorControlCharacteristicUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(MotorMetaCommand)
        ),

        // -------------------------------------------------
        // Right motor
        // -------------------------------------------------

        m_rightMotorReplyCharacteristic(
            rightMotorFeedbackCharacteristicUUID,
            BLERead | BLENotify,
            sizeof(MotorReply)
        ),

        m_rightMotorCommandCharacteristic(
            rightMotorCommandCharacteristicUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(MotorCmd)
        ),

        m_rightMotorControlCharacteristic(
            rightMotorControlCharacteristicUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(MotorMetaCommand)
        ),

        // -------------------------------------------------
        // SD logger
        // -------------------------------------------------

        m_loggingControlCharacteristic(
            loggingControlCharacteristicUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(LoggingState)
        )
    {
        s_instance = this;
    }


    // =====================================================
    // INITIALIZATION
    // =====================================================

    bool begin()
    {
        if (!BLE.begin())
        {
            return false;
        }


        BLE.setLocalName(
            "AnkleExo"
        );

        BLE.setAdvertisedService(
            m_service
        );


        // -------------------------------------------------
        // Shared telemetry
        // -------------------------------------------------

        m_service.addCharacteristic(
            m_encoderCharacteristic
        );

        m_service.addCharacteristic(
            m_loadCellCharacteristic
        );

        m_service.addCharacteristic(
            m_powerCharacteristic
        );


        // -------------------------------------------------
        // Left motor
        // -------------------------------------------------

        m_service.addCharacteristic(
            m_leftMotorReplyCharacteristic
        );

        m_service.addCharacteristic(
            m_leftMotorCommandCharacteristic
        );

        m_service.addCharacteristic(
            m_leftMotorControlCharacteristic
        );


        // -------------------------------------------------
        // Right motor
        // -------------------------------------------------

        m_service.addCharacteristic(
            m_rightMotorReplyCharacteristic
        );

        m_service.addCharacteristic(
            m_rightMotorCommandCharacteristic
        );

        m_service.addCharacteristic(
            m_rightMotorControlCharacteristic
        );


        // -------------------------------------------------
        // SD logger
        // -------------------------------------------------

        m_service.addCharacteristic(
            m_loggingControlCharacteristic
        );


        // -------------------------------------------------
        // Left motor callbacks
        // -------------------------------------------------

        m_leftMotorCommandCharacteristic
            .setEventHandler(
                BLEWritten,
                leftMotorCommandWrittenCallback
            );

        m_leftMotorControlCharacteristic
            .setEventHandler(
                BLEWritten,
                leftMotorControlWrittenCallback
            );


        // -------------------------------------------------
        // Right motor callbacks
        // -------------------------------------------------

        m_rightMotorCommandCharacteristic
            .setEventHandler(
                BLEWritten,
                rightMotorCommandWrittenCallback
            );

        m_rightMotorControlCharacteristic
            .setEventHandler(
                BLEWritten,
                rightMotorControlWrittenCallback
            );


        // -------------------------------------------------
        // SD logger callback
        // -------------------------------------------------

        m_loggingControlCharacteristic
            .setEventHandler(
                BLEWritten,
                loggingControlWrittenCallback
            );


        BLE.addService(
            m_service
        );

        BLE.advertise();

        return true;
    }


    // =====================================================
    // TASK UPDATE
    // =====================================================

    void update(
        uint32_t) override
    {
        BLE.poll();

        if (!BLE.connected())
        {
            return;
        }

        publishChangedTopics();
    }


private:

    // =====================================================
    // TELEMETRY PUBLISHING
    // =====================================================

    void publishChangedTopics()
    {
        publishIfChanged(
            m_encoder,
            m_encoderCharacteristic,
            m_encoderSequence
        );

        publishIfChanged(
            m_loadCells,
            m_loadCellCharacteristic,
            m_loadCellSequence
        );

        publishIfChanged(
            m_power,
            m_powerCharacteristic,
            m_powerSequence
        );

        publishIfChanged(
            m_leftMotor,
            m_leftMotorReplyCharacteristic,
            m_leftMotorSequence
        );

        publishIfChanged(
            m_rightMotor,
            m_rightMotorReplyCharacteristic,
            m_rightMotorSequence
        );
    }


    template<typename T>
    void publishIfChanged(
        Topic<T>& topic,
        BLECharacteristic& characteristic,
        uint32_t& lastSequence)
    {
        if (!topic.valid())
        {
            return;
        }

        if (
            topic.sequence() ==
            lastSequence
        )
        {
            return;
        }

        lastSequence =
            topic.sequence();

        const T& value =
            topic.latest();

        characteristic.writeValue(
            reinterpret_cast<
                const uint8_t*
            >(&value),
            sizeof(T)
        );
    }


    // =====================================================
    // GENERIC MOTOR COMMAND DECODER
    // =====================================================

    template<EndpointId Id>
    void handleMotorCommand(
        BLECharacteristic& characteristic)
    {
        MotorCmd command{};

        const int bytesRead =
            characteristic.readValue(
                reinterpret_cast<uint8_t*>(
                    &command
                ),
                sizeof(command)
            );

        if (
            bytesRead !=
            static_cast<int>(
                sizeof(command)
            )
        )
        {
            Serial.print(
                "Wrong MotorCmd size: "
            );

            Serial.println(
                bytesRead
            );

            return;
        }

        m_bus.publish<Id>(
            command
        );
    }


    // =====================================================
    // LEFT MOTOR COMMAND CALLBACK
    // =====================================================

    static void leftMotorCommandWrittenCallback(
        BLEDevice central,
        BLECharacteristic characteristic)
    {
        (void)central;

        if (s_instance == nullptr)
        {
            return;
        }

        s_instance
            ->handleMotorCommand<
                EndpointId::
                    LeftMotorCommand
            >(
                characteristic
            );
    }


    // =====================================================
    // RIGHT MOTOR COMMAND CALLBACK
    // =====================================================

    static void rightMotorCommandWrittenCallback(
        BLEDevice central,
        BLECharacteristic characteristic)
    {
        (void)central;

        if (s_instance == nullptr)
        {
            return;
        }

        s_instance
            ->handleMotorCommand<
                EndpointId::
                    RightMotorCommand
            >(
                characteristic
            );
    }


    // =====================================================
    // GENERIC MOTOR META-CONTROL DECODER
    // =====================================================

    template<EndpointId Id>
    void handleMotorControl(
        BLECharacteristic& characteristic)
    {
        uint8_t rawCommand{};

        const int bytesRead =
            characteristic.readValue(
                &rawCommand,
                sizeof(rawCommand)
            );

        if (bytesRead != 1)
        {
            Serial.print(
                "Wrong MotorMetaCommand size: "
            );

            Serial.println(
                bytesRead
            );

            return;
        }


        if (
            rawCommand >
            static_cast<uint8_t>(
                MotorMetaCommand::SetZero
            )
        )
        {
            Serial.print(
                "Invalid MotorMetaCommand: "
            );

            Serial.println(
                rawCommand
            );

            return;
        }


        const auto command =
            static_cast<
                MotorMetaCommand
            >(
                rawCommand
            );


        m_bus.publish<Id>(
            command
        );
    }


    // =====================================================
    // LEFT MOTOR CONTROL CALLBACK
    // =====================================================

    static void leftMotorControlWrittenCallback(
        BLEDevice central,
        BLECharacteristic characteristic)
    {
        (void)central;

        if (s_instance == nullptr)
        {
            return;
        }

        s_instance
            ->handleMotorControl<
                EndpointId::
                    LeftMotorMetaCommand
            >(
                characteristic
            );
    }


    // =====================================================
    // RIGHT MOTOR CONTROL CALLBACK
    // =====================================================

    static void rightMotorControlWrittenCallback(
        BLEDevice central,
        BLECharacteristic characteristic)
    {
        (void)central;

        if (s_instance == nullptr)
        {
            return;
        }

        s_instance
            ->handleMotorControl<
                EndpointId::
                    RightMotorMetaCommand
            >(
                characteristic
            );
    }


    // =====================================================
    // SD LOGGING CONTROL
    // =====================================================

    static void loggingControlWrittenCallback(
        BLEDevice central,
        BLECharacteristic characteristic)
    {
        (void)central;

        if (s_instance == nullptr)
        {
            return;
        }

        s_instance
            ->handleLoggingControl(
                characteristic
            );
    }


    void handleLoggingControl(
        BLECharacteristic& characteristic)
    {
        uint8_t rawState{};

        const int bytesRead =
            characteristic.readValue(
                &rawState,
                sizeof(rawState)
            );

        if (bytesRead != 1)
        {
            Serial.print(
                "Wrong LoggingState size: "
            );

            Serial.println(
                bytesRead
            );

            return;
        }


        if (
            rawState >
            static_cast<uint8_t>(
                LoggingState::Recording
            )
        )
        {
            Serial.print(
                "Invalid LoggingState: "
            );

            Serial.println(
                rawState
            );

            return;
        }


        const auto state =
            static_cast<
                LoggingState
            >(
                rawState
            );


        m_bus.publish<
            EndpointId::LoggingState
        >(
            state
        );


        Serial.print(
            "Logging state received: "
        );

        Serial.println(
            rawState
        );
    }


private:

    // =====================================================
    // SINGLE BLE INSTANCE FOR STATIC CALLBACKS
    // =====================================================

    inline static BLEBridge*
        s_instance = nullptr;


    // =====================================================
    // MESSAGE BUS
    // =====================================================

    MessageBus& m_bus;


    // =====================================================
    // TOPICS
    // =====================================================

    Topic<EncoderPositions>&
        m_encoder;

    Topic<LoadCellForces>&
        m_loadCells;

    Topic<MotorReply>&
        m_leftMotor;

    Topic<MotorReply>&
        m_rightMotor;

    Topic<PowerReadings>&
        m_power;


    // =====================================================
    // LAST BLE-PUBLISHED SEQUENCES
    // =====================================================

    uint32_t
        m_encoderSequence{0};

    uint32_t
        m_loadCellSequence{0};

    uint32_t
        m_powerSequence{0};

    uint32_t
        m_leftMotorSequence{0};

    uint32_t
        m_rightMotorSequence{0};


    // =====================================================
    // BLE SERVICE
    // =====================================================

    BLEService m_service;


    // =====================================================
    // SHARED TELEMETRY CHARACTERISTICS
    // =====================================================

    BLECharacteristic
        m_encoderCharacteristic;

    BLECharacteristic
        m_loadCellCharacteristic;

    BLECharacteristic
        m_powerCharacteristic;


    // =====================================================
    // LEFT MOTOR CHARACTERISTICS
    // =====================================================

    BLECharacteristic
        m_leftMotorReplyCharacteristic;

    BLECharacteristic
        m_leftMotorCommandCharacteristic;

    BLECharacteristic
        m_leftMotorControlCharacteristic;


    // =====================================================
    // RIGHT MOTOR CHARACTERISTICS
    // =====================================================

    BLECharacteristic
        m_rightMotorReplyCharacteristic;

    BLECharacteristic
        m_rightMotorCommandCharacteristic;

    BLECharacteristic
        m_rightMotorControlCharacteristic;


    // =====================================================
    // SD LOGGER CHARACTERISTIC
    // =====================================================

    BLECharacteristic
        m_loggingControlCharacteristic;
};

#endif