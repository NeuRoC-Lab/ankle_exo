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
    "99F02A66-065C-41BE-B05E-4BE2B9035A8B"; // basically a bool to turn on / off the motor (virtual state)


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
// Transparent Mode Controller UUIDs
// =========================================================

constexpr char leftTransparentModeControllerUUID[] =
    "644CD587-A563-437E-8006-8B7F39559690";

constexpr char rightTransparentModeControllerUUID[] =
    "A8B58E7F-4B57-4C5F-85E4-55F38A6CE271";

// TENMPORARY

constexpr char leftIntermediateTorqueCharacteristicUUID[] =
    "B50F6E44-AB02-4C7A-A801-74A85815B001";

constexpr char rightIntermediateTorqueCharacteristicUUID[] =
    "B50F6E44-AB02-4C7A-A801-74A85815B002";

constexpr char leftControllerOutputTorqueCharacteristicUUID[] =
    "B50F6E44-AB02-4C7A-A801-74A85815B003";

constexpr char rightControllerOutputTorqueCharacteristicUUID[] =
    "B50F6E44-AB02-4C7A-A801-74A85815B004";

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
        sizeof(LoadCellTorques)
    ),

    m_powerCharacteristic(
        powerCharacteristicUUID,
        BLERead | BLENotify,
        sizeof(PowerReadings)
    ),

    m_leftIntermediateTorqueCharacteristic(
        leftIntermediateTorqueCharacteristicUUID,
        BLERead | BLENotify,
        sizeof(float)
    ),

    m_rightIntermediateTorqueCharacteristic(
        rightIntermediateTorqueCharacteristicUUID,
        BLERead | BLENotify,
        sizeof(float)
    ),

    m_leftControllerOutputTorqueCharacteristic(
        leftControllerOutputTorqueCharacteristicUUID,
        BLERead | BLENotify,
        sizeof(float)
    ),

    m_rightControllerOutputTorqueCharacteristic(
        rightControllerOutputTorqueCharacteristicUUID,
        BLERead | BLENotify,
        sizeof(float)
    ),

    // -------------------------------------------------
    // Left motor
    // -------------------------------------------------

    m_leftMotorReplyCharacteristic(
        leftMotorFeedbackCharacteristicUUID,
        BLERead | BLENotify,
        sizeof(MotorFeedback)
    ),

    m_leftMotorCommandCharacteristic(
        leftMotorCommandCharacteristicUUID,
        BLEWrite | BLEWriteWithoutResponse,
        sizeof(float)
    ),

    m_leftMotorControlCharacteristic(
        leftMotorControlCharacteristicUUID,
        BLEWrite | BLEWriteWithoutResponse,
        sizeof(bool)
    ),

    // -------------------------------------------------
    // Right motor
    // -------------------------------------------------

    m_rightMotorReplyCharacteristic(
        rightMotorFeedbackCharacteristicUUID,
        BLERead | BLENotify,
        sizeof(MotorFeedback)
    ),

    m_rightMotorCommandCharacteristic(
        rightMotorCommandCharacteristicUUID,
        BLEWrite | BLEWriteWithoutResponse,
        sizeof(float)
    ),

    m_rightMotorControlCharacteristic(
        rightMotorControlCharacteristicUUID,
        BLEWrite | BLEWriteWithoutResponse,
        sizeof(bool)
    ),

    // -------------------------------------------------
    // SD logger
    // -------------------------------------------------

    m_loggingControlCharacteristic(
        loggingControlCharacteristicUUID,
        BLEWrite | BLEWriteWithoutResponse,
        sizeof(LoggingState)
    ),

    // -------------------------------------------------
    // Transparent mode controller
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

        BLE.setAdvertisedService(m_service);


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

		m_service.addCharacteristic(
   		 m_leftIntermediateTorqueCharacteristic
		);

		m_service.addCharacteristic(
    		m_rightIntermediateTorqueCharacteristic
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
        // Transparent Mode Controller
        // -------------------------------------------------

                m_service.addCharacteristic(
    	m_leftTransparentModeControllerCharacteristic
		);

		m_service.addCharacteristic(
    	m_rightTransparentModeControllerCharacteristic
		);

    m_service.addCharacteristic(
        m_leftControllerOutputTorqueCharacteristic
    );

    m_service.addCharacteristic(
        m_rightControllerOutputTorqueCharacteristic
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

        // -------------------------------------------------
        // Transparent Mode Controller Callback
        // -------------------------------------------------

    m_leftTransparentModeControllerCharacteristic
        .setEventHandler(
            BLEWritten,
            leftTransparentModeControllerWrittenCallback
        );

    m_rightTransparentModeControllerCharacteristic
        .setEventHandler(
            BLEWritten,
            rightTransparentModeControllerWrittenCallback
    );



        BLE.addService(
            m_service
        );

        if (!BLE.advertise()) {
            Serial.println("BLE.advertise FAILED");
            return false;
        }
    Serial.println("BLE: advertising");

        return true;
    }


    // =====================================================
    // TASK UPDATE
    // =====================================================

    void update(uint32_t nowUs) override
    {
        BLE.poll();

        static bool wasConnected = false;
        const bool connected = BLE.connected();

        if (wasConnected && !connected) {
            Serial.println("BLE CENTRAL DISCONNECTED");
        }

        wasConnected = connected;

        if (!connected) {
            return;
        }

        constexpr uint32_t BLE_PERIOD_US = 20'000; // 50 Hz

        if (nowUs - m_lastBlePublishUs < BLE_PERIOD_US) {
            return;
        }

        m_lastBlePublishUs += BLE_PERIOD_US;

        publishChangedTopics();
    }


private:

    // =====================================================
    // TELEMETRY PUBLISHING
    // =====================================================
    uint32_t m_lastBlePublishUs{0};

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

        publishIfChanged(
            m_leftIntermediateTorque,
            m_leftIntermediateTorqueCharacteristic,
            m_leftIntermediateTorqueSequence
        );

        publishIfChanged(
            m_rightIntermediateTorque,
            m_rightIntermediateTorqueCharacteristic,
            m_rightIntermediateTorqueSequence
        );

        publishIfChanged(
            m_leftControllerOutputTorque,
            m_leftControllerOutputTorqueCharacteristic,
            m_leftControllerOutputTorqueSequence
        );

        publishIfChanged(
            m_rightControllerOutputTorque,
            m_rightControllerOutputTorqueCharacteristic,
            m_rightControllerOutputTorqueSequence
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
        float command{};

        const int bytesRead = characteristic.readValue(reinterpret_cast<uint8_t*>(&command),sizeof(command));

        if (bytesRead != static_cast<int>(sizeof(command)))
        {
            Serial.print("Wrong float size: ");
            Serial.println(bytesRead);
            return;
        }

        m_bus.publish<Id>(command);
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
        s_instance->handleMotorCommand<EndpointId::LeftMotorCommand>(characteristic);
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

        s_instance->handleMotorCommand<EndpointId::RightMotorCommand>(characteristic);
    }


    // =====================================================
    // GENERIC MOTOR CONTROL CALLBACK
    // =====================================================

    template<EndpointId Id>
	void handleMotorControl(
    BLECharacteristic& characteristic)
{
    uint8_t rawEnabled{};

    const int bytesRead =
        characteristic.readValue(&rawEnabled,sizeof(rawEnabled));

    if (bytesRead != 1)
    {
        Serial.print("Wrong bool size: ");
        Serial.println(bytesRead);
        return;
    }

    if (rawEnabled > 1)
    {
        Serial.print("Invalid motor enabled value: ");
        Serial.println(rawEnabled);
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

        s_instance->handleMotorControl<EndpointId::LeftMotorEnabled>(characteristic);
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

        s_instance->handleMotorControl<EndpointId::RightMotorEnabled>(characteristic);
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

    MessageBus& m_bus;


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

    uint32_t
        m_leftIntermediateTorqueSequence{0};

    uint32_t
        m_rightIntermediateTorqueSequence{0};

    uint32_t
        m_leftControllerOutputTorqueSequence{0};

    uint32_t
        m_rightControllerOutputTorqueSequence{0};


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

	BLECharacteristic
    	m_leftIntermediateTorqueCharacteristic;

	BLECharacteristic
    	m_rightIntermediateTorqueCharacteristic;

    BLECharacteristic
        m_leftControllerOutputTorqueCharacteristic;

    BLECharacteristic
        m_rightControllerOutputTorqueCharacteristic;


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

    // =====================================================
    //TRANSPARENT MODE CONTROLLER CHARACTERISTIC
    // =====================================================

	BLECharacteristic
    	m_leftTransparentModeControllerCharacteristic;

	BLECharacteristic
    	m_rightTransparentModeControllerCharacteristic;
};

#endif