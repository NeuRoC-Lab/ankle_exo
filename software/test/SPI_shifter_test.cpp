//
// Created by Oscar Tesniere on 23/07/2026.
//
// SPI communication test:
//
// Teensy:   SPI controller
// UNO R3:   SPI peripheral
//
// The UNO returns:
//     previously_received_byte + 1
//
// The first received byte is 0xA5 because SPDR is initialized to 0xA5.
//

#include <Arduino.h>
#include <SPI.h>

constexpr uint32_t SERIAL_BAUD_RATE = 115200;
constexpr uint32_t SPI_CLOCK_HZ = 500000;

#if defined(PLATFORM_TEENSY41)
#include "Board.h"
// Change this if boardConfig.ENCODER_LEFT_CS is not connected to the UNO's SS pin.
const uint8_t csPin = board::teensy41::pins.encoders.leftChipSelect; //CS2


uint8_t byteToSend = 0;
uint8_t previousByteSent = 0;
bool firstTransfer = true;

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    pinMode(board::teensy41::pins.levelShifters.oe1, OUTPUT);
    pinMode(board::teensy41::pins.levelShifters.oe2, OUTPUT);
    pinMode(csPin, OUTPUT);

    // Keep the peripheral deselected initially.
    digitalWrite(csPin, HIGH);

    // Enable the voltage shifter.
    digitalWrite(board::teensy41::pins.levelShifters.oe1, HIGH);
    digitalWrite(board::teensy41::pins.levelShifters.oe2, HIGH);

    SPI1.begin();

    delay(1000);

    Serial.println("Teensy SPI controller started");
    Serial.println("Sending one byte every 500 ms");
}

void loop()
{
/*
digitalWrite(csPin,HIGH);
delay(1000);
digitalWrite(csPin,LOW);
delay(1000);
*/
    SPI1.beginTransaction(
        SPISettings(
            SPI_CLOCK_HZ,
            MSBFIRST,
            SPI_MODE0
        )
    );

    digitalWrite(csPin, LOW);

    // Small setup delay for the level shifter and peripheral SS signal.
    delayMicroseconds(2);

    const uint8_t receivedByte = SPI1.transfer(byteToSend);

    delayMicroseconds(2);

    digitalWrite(csPin, HIGH);

    SPI1.endTransaction();

    Serial.print("TX: 0x");
    if (byteToSend < 0x10) {
        Serial.print('0');
    }
    Serial.print(byteToSend, HEX);

    Serial.print(" | RX: 0x");
    if (receivedByte < 0x10) {
        Serial.print('0');
    }
    Serial.print(receivedByte, HEX);

    if (firstTransfer) {
        if (receivedByte == 0xA5) {
            Serial.println(" | Initial response OK");
        } else {
            Serial.println(" | Unexpected initial response");
        }

        firstTransfer = false;
    } else {
        const uint8_t expectedResponse =
            static_cast<uint8_t>(previousByteSent + 1);

        Serial.print(" | Expected: 0x");
        if (expectedResponse < 0x10) {
            Serial.print('0');
        }
        Serial.print(expectedResponse, HEX);

        if (receivedByte == expectedResponse) {
            Serial.println(" | PASS");
        } else {
            Serial.println(" | FAIL");
        }
    }

    previousByteSent = byteToSend;
    byteToSend++;

    delay(500);
}
#elif defined(PLATFORM_ATMELAVR)

#include <avr/interrupt.h>

volatile uint8_t receivedByte = 0;
volatile bool newByteReceived = false;

/*
 * This interrupt runs after one complete SPI byte is received.
 *
 * Important:
 * SPI is full duplex. The value written to SPDR here will be returned
 * during the NEXT SPI transfer.
 */
ISR(SPI_STC_vect)
{
    const uint8_t incomingByte = SPDR;

    receivedByte = incomingByte;
    newByteReceived = true;

    // Prepare the response for the next transaction.
    SPDR = static_cast<uint8_t>(incomingByte + 1);
}

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    /*
     * UNO R3 hardware SPI pins:
     *
     * D10 = SS
     * D11 = MOSI
     * D12 = MISO
     * D13 = SCK
     */

    pinMode(SS, INPUT);
    pinMode(MOSI, INPUT);
    pinMode(SCK, INPUT);
    pinMode(MISO, OUTPUT);

    /*
     * SPE  = enable SPI
     * SPIE = enable SPI transfer-complete interrupt
     *
     * MSTR remains zero, so the ATmega328P operates as a peripheral.
     * CPOL and CPHA remain zero, selecting SPI mode 0.
     */
    SPCR = _BV(SPE) | _BV(SPIE);

    // First byte returned to the Teensy.
    SPDR = 0xA5;

    sei();

    Serial.println("UNO R3 SPI peripheral started");
}

void loop()
{
    if (newByteReceived) {
        /*
         * Copy shared ISR data while interrupts are temporarily disabled.
         */
        noInterrupts();
        const uint8_t value = receivedByte;
        newByteReceived = false;
        interrupts();

        Serial.print("Received: 0x");

        if (value < 0x10) {
            Serial.print('0');
        }

        Serial.print(value, HEX);

        Serial.print(" | Next response: 0x");

        const uint8_t response =
            static_cast<uint8_t>(value + 1);

        if (response < 0x10) {
            Serial.print('0');
        }

        Serial.println(response, HEX);
    }
}

#else

#error "Define either PLATFORM_TEENSY41 or PLATFORM_ATMELAVR"

#endif