//
// Created by Oscar Tesniere on 23/07/2026.
//
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

#if defined(PLATFORM_TEENSY41)
#include "Board.h"

constexpr int CS1 = 7;
constexpr int CIPO1 = 1;
constexpr int CS2 = 0;
constexpr int SCK1 = 27;
constexpr int COPI1 = 26;

void setup()
{
    Serial.begin(115200);

    pinMode(boardConfig.OE1, OUTPUT);
    pinMode(boardConfig.OE2, OUTPUT);
    pinMode(CS1,OUTPUT);
    pinMode(CS2,OUTPUT);
    pinMode(CIPO1,INPUT);
    pinMode(COPI1,OUTPUT);
    pinMode(SCK1,OUTPUT);

    // Enable the voltage shifter.
    digitalWrite(boardConfig.OE1, HIGH);
    digitalWrite(boardConfig.OE2, HIGH);

    delay(1000);

    Serial.println("Shifters are enabled");
}

void loop(){
// "Blink" like example to test the shifter
delay(1000);
digitalWrite(COPI1,HIGH);
delay(1000);
digitalWrite(COPI1,LOW);
}
#else
#error "Choose teensy please"
#endif