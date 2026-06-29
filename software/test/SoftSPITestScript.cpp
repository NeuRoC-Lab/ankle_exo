//
// AMT20 / AMT203 encoder test using SWSPI
//
// Software SPI master
// Based on Rob Tillaart SWSPI library
//

#include <Arduino.h>
#include <SWSPI.h>

// ------------------------------------------------------------
// Serial baud rate
// ------------------------------------------------------------
constexpr uint32_t BAUD_RATE = 115200;

// ------------------------------------------------------------
// AMT20 / AMT203 SPI commands
// ------------------------------------------------------------
constexpr uint8_t NOP            = 0x00;
constexpr uint8_t RD_POS         = 0x10;
constexpr uint8_t SET_ZERO_POINT = 0x70;

// ------------------------------------------------------------
// AMT20 response / timeout settings
// ------------------------------------------------------------
constexpr uint8_t TIMEOUT_LIMIT = 100;

// ------------------------------------------------------------
// Software SPI pins
//
// SWSPI(dataIn, dataOut, clock)
// dataIn  = MISO, encoder -> Arduino
// dataOut = MOSI, Arduino -> encoder
// clock   = SCK
// ------------------------------------------------------------
constexpr uint8_t SW_MISO = 3;
constexpr uint8_t SW_MOSI = 4;
constexpr uint8_t SW_SCK  = 5;

// Encoder chip select
constexpr uint8_t CSB = 6;

// Create software SPI object
SWSPI myspi(SW_MISO, SW_MOSI, SW_SCK);

// ------------------------------------------------------------
// Control flags
// ------------------------------------------------------------
bool running = false;

// Zero/reference position
uint16_t zeroPosition = 0;

// ------------------------------------------------------------
// Function prototypes
// ------------------------------------------------------------
uint8_t SW_SPIWrite(uint8_t sendByte);
uint16_t readEncoder();
bool setEncoderZero();

void setup()
{
    Serial.begin(BAUD_RATE);
    delay(1000);

    pinMode(CSB, OUTPUT);
    digitalWrite(CSB, HIGH);

    myspi.begin();

    Serial.println("AMT20 / AMT203 Encoder Test using SWSPI");
    Serial.println("y = start running encoder");
    Serial.println("n = stop running");
    Serial.println("z = set current position as software zero");
    Serial.println("h = send hardware set-zero command");
}

void loop()
{
    // Handle serial commands
    if (Serial.available())
    {
        char c = Serial.read();

        if (c == 'y')
        {
            running = true;
            Serial.println("Encoder run started");
        }
        else if (c == 'n')
        {
            running = false;
            Serial.println("Encoder run stopped");
        }
        else if (c == 'z')
        {
            uint16_t currentPosition = readEncoder();
            zeroPosition = currentPosition;

            Serial.print("Software zero set at count ");
            Serial.println(zeroPosition);
        }
        else if (c == 'h')
        {
            if (setEncoderZero())
            {
                Serial.println("Hardware zero command sent");
            }
            else
            {
                Serial.println("Hardware zero command may have failed");
            }
        }
    }

    if (!running)
    {
        return;
    }

    uint16_t currentPosition = readEncoder();

    // Relative position with wrap-around handling
    int16_t relativePosition =
        ((int32_t)currentPosition - (int32_t)zeroPosition + 2048) % 4096 - 2048;

    float angleDeg = relativePosition * 360.0f / 4096.0f;

    Serial.print("Raw: ");
    Serial.print(currentPosition);

    Serial.print("\tRelative: ");
    Serial.print(relativePosition);

    Serial.print("\tAngle: ");
    Serial.print(angleDeg, 2);
    Serial.println(" deg");

    delay(50);
}

uint16_t readEncoder()
{
    uint8_t data;
    uint8_t timeoutCounter = 0;
    uint16_t currentPosition;

    // Send read-position command
    data = SW_SPIWrite(RD_POS);

    // Wait until encoder echoes/acknowledges RD_POS
    while (data != RD_POS && timeoutCounter++ < TIMEOUT_LIMIT)
    {
        data = SW_SPIWrite(NOP);
    }

    if (timeoutCounter >= TIMEOUT_LIMIT)
    {
        Serial.println("Error obtaining position");
        return 0;
    }

    // Read 12-bit position
    currentPosition = (SW_SPIWrite(NOP) & 0x0F) << 8;
    currentPosition |= SW_SPIWrite(NOP);

    return currentPosition;
}

uint8_t SW_SPIWrite(uint8_t sendByte)
{
    uint8_t data;

    // Your working Arduino-slave test needed SPI_MODE3.
    // For the AMT20/AMT203, your previous hardware SPI code used SPI_MODE0.
    //
    // Start with SPI_MODE0 for the encoder.
    // If values are shifted/weird, try SPI_MODE3.
    myspi.beginTransaction(MSBFIRST, SPI_MODE0);

    digitalWrite(CSB, LOW);

    data = myspi.transfer(sendByte);

    digitalWrite(CSB, HIGH);

    myspi.endTransaction();

    delayMicroseconds(10);

    return data;
}

bool setEncoderZero()
{
    uint8_t response;

    response = SW_SPIWrite(SET_ZERO_POINT);

    // The exact response depends on the encoder variant.
    // For now, just send the command and wait.
    delay(250);

    return true;
}