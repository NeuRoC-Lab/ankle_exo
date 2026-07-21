//
// Created by Oscar Tesniere on 20/07/2026.
// for handling communication between the Teensy and the Nano over UART
// Requirements :
// Serialize and deserialize data to/from teensy and nano
// ease ofn implementation with both 1) direct serial communication b/w laptop and teensy (for debugging purposes) and 2) bluetooth interfacing with the Nano
// Ideally have a datapayload struct which is common to both the pre-serialization (when teensy sends data to nano for ex) and post-deserialization(ex after the nano has deserialized the data). This makes it more modular
//
#include <Arduino.h>

struct LoadCellVoltages {
    float LeftLoadCell1 {};
    float LeftLoadCell2 {};
    float RightLoadCell1 {};
    float RightLoadCell2 {};
};

struct DataPayload {
    // load cells
    // encoders
    LoadCellVoltages loadCells {};
    EncoderPositions encoders {};
    MotorReply motorRep {}; // to account for the two motors
};

inline size_t sendPayload(const DataPayload& payload,HardwareSerial& serial){
// serial can be any of Serial1, Serial8 etc
// first we need to reinterpret the poiunter to the struct as a pointer to an array of uint8_t
// we are using the signature Serial.write(buf, len)
return serial.write(reinterpret_cast<const uint8_t*>(&payload),
                    sizeof(payload)
                    );
}

inline bool readPayload(
    DataPayload& response,
    HardwareSerial& serial
) {

    constexpr size_t payloadSize = sizeof(DataPayload); // size resolved at compile time = safer

    if (serial.available() < payloadSize) {
        return false;
    }

    size_t bytesRead = serial.readBytes(
        reinterpret_cast<char*>(&response),
        payloadSize
    );

    return bytesRead == payloadSize;
}

// Bluetooth components for the Arduino Nano
#if defined(PLATFORM_NORDIC)
#include <ArduinoBLE.h>
// on mac generate 128bit UUIDs with uuidgen command
constexpr char dataServiceUUID[] = "CF45813E-4358-4903-B961-09996BB081FB";
constexpr char LLCCharacteristicUUID[] = "CA87289F-102B-4078-AD8C-8F53063547A6";
constexpr char motorCharacteristicUUID[] = "E0D883F6-705C-4A11-B117-E2B0909CC68E";
constexpr char encoderCharacteristicUUID[] = "094A717B-0C7F-4A23-BFD1-A4924E6E7DAB";

class BLEHandler {
public:
    DataPayload& m_payload;
    explicit BLEHandler(DataPayload& payload)
        : m_payload(payload),
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
          )
    {
    }

    bool begin()
    {
        pinMode(LED_BUILTIN, OUTPUT);

        if (!BLE.begin()) {
            return false;
        }

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

        BLE.addService(m_dataService);
        BLE.advertise();

        return true;
    }

    void update()
    {
        BLE.poll();

        BLEDevice central = BLE.central();

        if (!central) {
            digitalWrite(LED_BUILTIN, LOW);
            return;
        }

        digitalWrite(LED_BUILTIN, HIGH);

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

    DataPayload& payload()
    {
        return m_payload;
    }

private:

    BLEService m_dataService;

    BLECharacteristic m_loadCellCharacteristic;
    BLECharacteristic m_motorCharacteristic;
    BLECharacteristic m_encoderCharacteristic;
};

#endif