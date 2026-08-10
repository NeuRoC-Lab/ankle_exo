#pragma once

#if defined(PLATFORM_NANO)

#include <Arduino.h>
#include <ArduinoBLE.h>

#include "Driver.h"
#include "MessageBus.h"

constexpr char dataServiceUUID[] =
    "CF45813E-4358-4903-B961-09996BB081FB";

constexpr char loadCellCharacteristicUUID[] =
    "CA87289F-102B-4078-AD8C-8F53063547A6";

constexpr char motorCommandCharacteristicUUID[] =
    "E0D883F6-705C-4A11-B117-E2B0909CC68E";

constexpr char encoderCharacteristicUUID[] =
    "094A717B-0C7F-4A23-BFD1-A4924E6E7DAB";

constexpr char powerCharacteristicUUID[] =
    "4D92C3F7-848C-42C2-B26A-9D1D15CB361A";

constexpr char motorFeedbackCharacteristicUUID[] =
    "81DC2896-1B27-4195-A391-99A637FA50A4";

constexpr char motorControlCharacteristicUUID[] =
    "99F02A66-065C-41BE-B05E-4BE2B9035A8B";


class BLEBridge final :
    public ITask
{
public:

    BLEBridge(
        MessageBus& bus,
        Topic<EncoderPositions>& encoder,
        Topic<LoadCellForces>& loadCells,
        Topic<MotorReply>& leftMotor,
        Topic<PowerReadings>& power)
        :
        m_bus(bus),
        m_encoder(encoder),
        m_loadCells(loadCells),
        m_leftMotor(leftMotor),
        m_power(power),

        m_service(
            dataServiceUUID
        ),

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

        m_motorReplyCharacteristic(
            motorFeedbackCharacteristicUUID,
            BLERead | BLENotify,
            sizeof(MotorReply)
        ),

        m_powerCharacteristic(
            powerCharacteristicUUID,
            BLERead | BLENotify,
            sizeof(PowerReadings)
        ),

        m_motorCommandCharacteristic(
            motorCommandCharacteristicUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(MotorCmd)
        ),

        m_motorControlCharacteristic(
            motorControlCharacteristicUUID,
            BLEWrite | BLEWriteWithoutResponse,
            sizeof(MotorMetaCommand)
        )
    {
        s_instance = this;
    }


    bool begin()
    {
        if (!BLE.begin()) {
            return false;
        }

        BLE.setLocalName(
            "AnkleExo"
        );

        BLE.setAdvertisedService(
            m_service
        );

        /*
         * Telemetry characteristics
         */

        m_service.addCharacteristic(
            m_encoderCharacteristic
        );

        m_service.addCharacteristic(
            m_loadCellCharacteristic
        );

        m_service.addCharacteristic(
            m_motorReplyCharacteristic
        );

        m_service.addCharacteristic(
            m_powerCharacteristic
        );

        /*
         * Command characteristics
         */

        m_service.addCharacteristic(
            m_motorCommandCharacteristic
        );

        m_service.addCharacteristic(
            m_motorControlCharacteristic
        );


        /*
         * BLE write callbacks
         */

        m_motorCommandCharacteristic
            .setEventHandler(
                BLEWritten,
                motorCommandWrittenCallback
            );

        m_motorControlCharacteristic
            .setEventHandler(
                BLEWritten,
                motorControlWrittenCallback
            );


        BLE.addService(
            m_service
        );

        BLE.advertise();

        return true;
    }


    void update(
        uint32_t) override
    {
        BLE.poll();

        if (!BLE.connected()) {
            return;
        }

        publishChangedTopics();
    }


private:

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
            m_leftMotor,
            m_motorReplyCharacteristic,
            m_motorSequence
        );

        publishIfChanged(
            m_power,
            m_powerCharacteristic,
            m_powerSequence
        );
    }


    template<typename T>
    void publishIfChanged(
        Topic<T>& topic,
        BLECharacteristic& characteristic,
        uint32_t& lastSequence)
    {
        if (!topic.valid()) {
            return;
        }

        if (
            topic.sequence() ==
            lastSequence
        ) {
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


    /*
     * Motor command callback
     *
     * Continuous MIT command:
     * position, velocity, kp, kd, torque
     */

    static void motorCommandWrittenCallback(
        BLEDevice central,
        BLECharacteristic characteristic)
    {
        (void)central;

        if (s_instance == nullptr) {
            return;
        }

        s_instance->handleMotorCommand(
            characteristic
        );
    }


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

        m_bus.publish<
            EndpointId::LeftMotorCommand
        >(command);
    }


    /*
     * Motor control callback
     *
     * Discrete commands:
     * EnterMotorMode
     * ExitMotorMode
     * SetZero
     */

    static void motorControlWrittenCallback(
        BLEDevice central,
        BLECharacteristic characteristic)
    {
        (void)central;

        if (s_instance == nullptr) {
            return;
        }

        s_instance->handleMotorControl(
            characteristic
        );
    }


    void handleMotorControl(
        BLECharacteristic& characteristic)
    {
        uint8_t rawCommand{};

        const int bytesRead =
            characteristic.readValue(
                &rawCommand,
                sizeof(rawCommand)
            );

        if (
            bytesRead !=
            static_cast<int>(
                sizeof(rawCommand)
            )
        )
        {
            Serial.print(
                "Wrong MotorMetaCommand size: "
            );

            Serial.println(
                bytesRead
            );

            return;
        }

        /*
         * Validate before converting
         * the incoming BLE byte into
         * an enum.
         */
        if (
            rawCommand >
            static_cast<uint8_t>(
                MotorMetaCommand::SetZero
            )
        )
        {
            Serial.print(
                "Invalid motor control command: "
            );

            Serial.println(
                rawCommand
            );

            return;
        }

        const auto command =
            static_cast<
                MotorMetaCommand
            >(rawCommand);

        m_bus.publish<
            EndpointId::
                LeftMotorMetaCommand
        >(command);

        Serial.print(
            "Motor control command received: "
        );

        Serial.println(
            rawCommand
        );
    }


private:

    inline static BLEBridge*
        s_instance = nullptr;


    MessageBus& m_bus;

    Topic<EncoderPositions>&
        m_encoder;

    Topic<LoadCellForces>&
        m_loadCells;

    Topic<MotorReply>&
        m_leftMotor;

    Topic<PowerReadings>&
        m_power;


    /*
     * Last published BLE versions
     */

    uint32_t m_encoderSequence{0};
    uint32_t m_loadCellSequence{0};
    uint32_t m_motorSequence{0};
    uint32_t m_powerSequence{0};


    /*
     * BLE objects
     */

    BLEService m_service;

    BLECharacteristic
        m_encoderCharacteristic;

    BLECharacteristic
        m_loadCellCharacteristic;

    BLECharacteristic
        m_motorReplyCharacteristic;

    BLECharacteristic
        m_powerCharacteristic;

    BLECharacteristic
        m_motorCommandCharacteristic;

    BLECharacteristic
        m_motorControlCharacteristic;
};

#endif