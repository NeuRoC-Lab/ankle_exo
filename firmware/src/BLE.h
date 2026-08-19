#pragma once

#if defined(PLATFORM_NANO)

#include <Arduino.h>
#include <ArduinoBLE.h>

#include "Driver.h"
#include "MessageBus.h"
#include "BusTypes.h"

// =========================================================
// AGGREGATED BLE TELEMETRY
// =========================================================
//
// All high-rate Nano -> PC telemetry is sent through ONE
// BLE notification characteristic.
//
// This avoids sending many small BLE notifications every
// 20 ms.
//
// IMPORTANT:
// The Bleak side must deserialize this structure using the
// exact same field layout.
// =========================================================

struct BLETelemetry
{
    EncoderPositions encoders;
    LoadCellTorques loadCells;

    MotorFeedback leftMotor;
    MotorFeedback rightMotor;

    PowerReadings power;

    float leftIntermediateTorque;
    float rightIntermediateTorque;

    float leftControllerOutputTorque;
    float rightControllerOutputTorque;
};


// =========================================================
// BLE SERVICE
// =========================================================

constexpr char dataServiceUUID[] =
    "CF45813E-4358-4903-B961-09996BB081FB";


// =========================================================
// HIGH-RATE TELEMETRY CHARACTERISTIC
// =========================================================

constexpr char telemetryUUID[] =
    "348B92F4-EB75-476B-A124-5D8C97C35907";


// =========================================================
// MOTOR COMMAND / CONTROL UUIDs
// =========================================================

constexpr char leftMotorCommandCharacteristicUUID[] =
    "E0D883F6-705C-4A11-B117-E2B0909CC68E";

constexpr char leftMotorControlCharacteristicUUID[] =
    "99F02A66-065C-41BE-B05E-4BE2B9035A8B";


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
// TRANSPARENT MODE CONTROLLER UUIDs
// =========================================================

constexpr char leftTransparentModeControllerUUID[] =
    "644CD587-A563-437E-8006-8B7F39559690";

constexpr char rightTransparentModeControllerUUID[] =
    "A8B58E7F-4B57-4C5F-85E4-55F38A6CE271";


// =========================================================
// BLE BRIDGE
// =========================================================

class BLEBridge final : public ITask
{
public:

    BLEBridge(
        MessageBus& bus,

        Topic<EncoderPositions>& encoder,
        Topic<LoadCellTorques>& loadCells,

        Topic<MotorFeedback>& leftMotor,
        Topic<MotorFeedback>& rightMotor,

        Topic<PowerReadings>& power,

        Topic<float>& leftIntermediateTorque,
        Topic<float>& rightIntermediateTorque,

        Topic<float>& leftControllerOutputTorque,
        Topic<float>& rightControllerOutputTorque)

        :
        m_bus(bus),

        m_encoder(encoder),
        m_loadCells(loadCells),

        m_leftMotor(leftMotor),
        m_rightMotor(rightMotor),

        m_power(power),

        m_leftIntermediateTorque(
            leftIntermediateTorque
        ),

        m_rightIntermediateTorque(
            rightIntermediateTorque
        ),

        m_leftControllerOutputTorque(
            leftControllerOutputTorque
        ),

        m_rightControllerOutputTorque(
            rightControllerOutputTorque
        ),

        // -------------------------------------------------
        // BLE service
        // -------------------------------------------------

        m_service(
            dataServiceUUID
        ),

        // -------------------------------------------------
        // High-rate telemetry
        // -------------------------------------------------

        m_telemetryCharacteristic(
            telemetryUUID,
            BLERead | BLENotify,
            sizeof(BLETelemetry)
        ),

        // -------------------------------------------------
        // Left motor
        // -------------------------------------------------

        m_leftMotorCommandCharacteristic(
            leftMotorCommandCharacteristicUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(float)
        ),

        m_leftMotorControlCharacteristic(
            leftMotorControlCharacteristicUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(uint8_t)
        ),

        // -------------------------------------------------
        // Right motor
        // -------------------------------------------------

        m_rightMotorCommandCharacteristic(
            rightMotorCommandCharacteristicUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(float)
        ),

        m_rightMotorControlCharacteristic(
            rightMotorControlCharacteristicUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(uint8_t)
        ),

        // -------------------------------------------------
        // SD logger
        // -------------------------------------------------

        m_loggingControlCharacteristic(
            loggingControlCharacteristicUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(uint8_t)
        ),

        // -------------------------------------------------
        // Transparent mode controllers
        // -------------------------------------------------

        m_leftTransparentModeControllerCharacteristic(
            leftTransparentModeControllerUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(TransparentControllerParameters)
        ),

        m_rightTransparentModeControllerCharacteristic(
            rightTransparentModeControllerUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(TransparentControllerParameters)
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
            Serial.println("Failed to start BLE");
            return false;
        }

        BLE.setLocalName("AnkleExo");
        BLE.setDeviceName("AnkleExo");

        BLE.setAdvertisedService(
            m_service
        );


        // -------------------------------------------------
        // ONE shared high-rate telemetry characteristic
        // -------------------------------------------------

        m_service.addCharacteristic(
            m_telemetryCharacteristic
        );


        // -------------------------------------------------
        // Left motor commands
        // -------------------------------------------------

        m_service.addCharacteristic(
            m_leftMotorCommandCharacteristic
        );

        m_service.addCharacteristic(
            m_leftMotorControlCharacteristic
        );


        // -------------------------------------------------
        // Right motor commands
        // -------------------------------------------------

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
        // Transparent mode controllers
        // -------------------------------------------------

        m_service.addCharacteristic(
            m_leftTransparentModeControllerCharacteristic
        );

        m_service.addCharacteristic(
            m_rightTransparentModeControllerCharacteristic
        );


        // -------------------------------------------------
        // Left motor callbacks
        // -------------------------------------------------

        m_leftMotorCommandCharacteristic.setEventHandler(
            BLEWritten,
            leftMotorCommandWrittenCallback
        );

        m_leftMotorControlCharacteristic.setEventHandler(
            BLEWritten,
            leftMotorControlWrittenCallback
        );


        // -------------------------------------------------
        // Right motor callbacks
        // -------------------------------------------------

        m_rightMotorCommandCharacteristic.setEventHandler(
            BLEWritten,
            rightMotorCommandWrittenCallback
        );

        m_rightMotorControlCharacteristic.setEventHandler(
            BLEWritten,
            rightMotorControlWrittenCallback
        );


        // -------------------------------------------------
        // SD logger callback
        // -------------------------------------------------

        m_loggingControlCharacteristic.setEventHandler(
            BLEWritten,
            loggingControlWrittenCallback
        );


        // -------------------------------------------------
        // Transparent mode callbacks
        // -------------------------------------------------

        m_leftTransparentModeControllerCharacteristic.setEventHandler(
            BLEWritten,
            leftTransparentModeControllerWrittenCallback
        );

        m_rightTransparentModeControllerCharacteristic.setEventHandler(
            BLEWritten,
            rightTransparentModeControllerWrittenCallback
        );


        // -------------------------------------------------
        // Register service
        // -------------------------------------------------

        BLE.addService(
            m_service
        );


        // -------------------------------------------------
        // Start advertising
        // -------------------------------------------------

        if (!BLE.advertise())
        {
            Serial.println("BLE.advertise FAILED");
            return false;
        }

        Serial.print("BLE: advertising, telemetry size = ");
        Serial.print(sizeof(BLETelemetry));
        Serial.println(" bytes");

        return true;
    }


    // =====================================================
    // TASK UPDATE
    // =====================================================

    void update(uint32_t nowUs) override
    {
        // Service the BLE stack on every scheduler pass.
        BLE.poll();

        const bool connected =
            BLE.connected();


        // -------------------------------------------------
        // Connection diagnostics
        // -------------------------------------------------

        if (!m_wasConnected && connected)
        {
            Serial.print("BLE CENTRAL CONNECTED at ");
            Serial.print(millis());
            Serial.println(" ms");
        }

        if (m_wasConnected && !connected)
        {
            Serial.print("BLE CENTRAL DISCONNECTED at ");
            Serial.print(millis());
            Serial.println(" ms");
        }

        m_wasConnected =
            connected;


        if (!connected)
        {
            // Avoid carrying timing debt across disconnects.
            m_lastBlePublishUs =
                nowUs;

            return;
        }


        // -------------------------------------------------
        // 50 Hz telemetry
        // -------------------------------------------------

        constexpr uint32_t BLE_PERIOD_US =
            20'000;

        if (
            static_cast<uint32_t>(
                nowUs - m_lastBlePublishUs
            ) < BLE_PERIOD_US
        )
        {
            return;
        }


        // Do not try to "catch up" by transmitting several
        // packets after a delayed scheduler iteration.
        m_lastBlePublishUs =
            nowUs;


        publishTelemetry();
    }


private:

    // =====================================================
    // TELEMETRY
    // =====================================================

    void publishTelemetry()
    {
        BLETelemetry telemetry{};


        // -------------------------------------------------
        // Read latest values from topics
        // -------------------------------------------------

        //
        // Topic::latest() is used directly here, matching
        // the behavior of your existing implementation.
        //

        telemetry.encoders =
            m_encoder.latest();

        telemetry.loadCells =
            m_loadCells.latest();


        telemetry.leftMotor =
            m_leftMotor.latest();

        telemetry.rightMotor =
            m_rightMotor.latest();


        telemetry.power =
            m_power.latest();


        telemetry.leftIntermediateTorque =
            m_leftIntermediateTorque.latest();

        telemetry.rightIntermediateTorque =
            m_rightIntermediateTorque.latest();


        telemetry.leftControllerOutputTorque =
            m_leftControllerOutputTorque.latest();

        telemetry.rightControllerOutputTorque =
            m_rightControllerOutputTorque.latest();


        // -------------------------------------------------
        // Send exactly ONE BLE notification/update
        // -------------------------------------------------

        const int result =
            m_telemetryCharacteristic.writeValue(
                reinterpret_cast<const uint8_t*>(
                    &telemetry
                ),
                sizeof(telemetry)
            );


        if (!result)
        {
            ++m_bleWriteFailureCount;

            Serial.print(
                "BLE telemetry write failed: size="
            );

            Serial.print(
                sizeof(BLETelemetry)
            );

            Serial.print(
                ", failures="
            );

            Serial.println(
                m_bleWriteFailureCount
            );
        }
    }


    // =====================================================
    // GENERIC MOTOR COMMAND DECODER
    // =====================================================

    template<EndpointId Id>
    void handleMotorCommand(
        BLECharacteristic& characteristic)
    {
        float command{};

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
                "Wrong float size: "
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
                EndpointId::LeftMotorCommand
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
                EndpointId::RightMotorCommand
            >(
                characteristic
            );
    }


    // =====================================================
    // GENERIC MOTOR CONTROL DECODER
    // =====================================================

    template<EndpointId Id>
    void handleMotorControl(
        BLECharacteristic& characteristic)
    {
        uint8_t rawEnabled{};

        const int bytesRead =
            characteristic.readValue(
                &rawEnabled,
                sizeof(rawEnabled)
            );


        if (bytesRead != 1)
        {
            Serial.print(
                "Wrong bool size: "
            );

            Serial.println(
                bytesRead
            );

            return;
        }


        if (rawEnabled > 1)
        {
            Serial.print(
                "Invalid motor enabled value: "
            );

            Serial.println(
                rawEnabled
            );

            return;
        }


        const bool enabled =
            rawEnabled != 0;


        m_bus.publish<Id>(
            enabled
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
                EndpointId::LeftMotorEnabled
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
                EndpointId::RightMotorEnabled
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
            static_cast<LoggingState>(
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


    // =====================================================
    // TRANSPARENT MODE CONTROLLER DECODER
    // =====================================================

    template<EndpointId Id>
    void handleTransparentModeController(
        BLECharacteristic& characteristic)
    {
        TransparentControllerParameters params{};

        const int bytesRead =
            characteristic.readValue(
                reinterpret_cast<uint8_t*>(
                    &params
                ),
                sizeof(params)
            );


        if (
            bytesRead !=
            static_cast<int>(
                sizeof(params)
            )
        )
        {
            Serial.print(
                "Wrong TransparentControllerParameters size: "
            );

            Serial.println(
                bytesRead
            );

            return;
        }


        m_bus.publish<Id>(
            params
        );
    }


    // =====================================================
    // LEFT TRANSPARENT MODE CALLBACK
    // =====================================================

    static void leftTransparentModeControllerWrittenCallback(
        BLEDevice central,
        BLECharacteristic characteristic)
    {
        (void)central;

        if (s_instance == nullptr)
        {
            return;
        }

        s_instance
            ->handleTransparentModeController<
                EndpointId::LeftMotorTransparentParams
            >(
                characteristic
            );
    }


    // =====================================================
    // RIGHT TRANSPARENT MODE CALLBACK
    // =====================================================

    static void rightTransparentModeControllerWrittenCallback(
        BLEDevice central,
        BLECharacteristic characteristic)
    {
        (void)central;

        if (s_instance == nullptr)
        {
            return;
        }

        s_instance
            ->handleTransparentModeController<
                EndpointId::RightMotorTransparentParams
            >(
                characteristic
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

    MessageBus&
        m_bus;


    // =====================================================
    // TOPICS
    // =====================================================

    Topic<EncoderPositions>&
        m_encoder;

    Topic<LoadCellTorques>&
        m_loadCells;


    Topic<MotorFeedback>&
        m_leftMotor;

    Topic<MotorFeedback>&
        m_rightMotor;


    Topic<PowerReadings>&
        m_power;


    Topic<float>&
        m_leftIntermediateTorque;

    Topic<float>&
        m_rightIntermediateTorque;


    Topic<float>&
        m_leftControllerOutputTorque;

    Topic<float>&
        m_rightControllerOutputTorque;


    // =====================================================
    // BLE STATE
    // =====================================================

    uint32_t
        m_lastBlePublishUs{0};

    uint32_t
        m_bleWriteFailureCount{0};

    bool
        m_wasConnected{false};


    // =====================================================
    // BLE SERVICE
    // =====================================================

    BLEService
        m_service;


    // =====================================================
    // HIGH-RATE TELEMETRY CHARACTERISTIC
    // =====================================================

    BLECharacteristic
        m_telemetryCharacteristic;


    // =====================================================
    // LEFT MOTOR EVENT-DRIVEN CHARACTERISTICS
    // =====================================================

    BLECharacteristic
        m_leftMotorCommandCharacteristic;

    BLECharacteristic
        m_leftMotorControlCharacteristic;


    // =====================================================
    // RIGHT MOTOR EVENT-DRIVEN CHARACTERISTICS
    // =====================================================

    BLECharacteristic
        m_rightMotorCommandCharacteristic;

    BLECharacteristic
        m_rightMotorControlCharacteristic;


    // =====================================================
    // SD LOGGER CHARACTERISTIC
    // =====================================================

    BLECharacteristic
        m_loggingControlCharacteristic;


    // =====================================================
    // TRANSPARENT MODE CONTROLLER CHARACTERISTICS
    // =====================================================

    BLECharacteristic
        m_leftTransparentModeControllerCharacteristic;

    BLECharacteristic
        m_rightTransparentModeControllerCharacteristic;
};

#endif