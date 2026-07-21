//
//
/*
#include <Arduino.h>
#include "Board.h"
#include <SPI.h>
// Encoder script

#if defined(PLATFORM_TEENSY41)
  #define SPI_PORT SPI1
#elif defined(PLATFORM_RENESAS_RA) || defined(PLATFORM_ATMEL_AVR)
  #define SPI_PORT SPI
#endif

constexpr uint16_t INVALID_ENCODER_POSITION = UINT16_MAX;

struct EncoderPositions {
    uint16_t left_position;
    uint16_t right_position;
};

constexpr byte wt_resp = 0x45; // wait response
constexpr byte rd_pos = 0x10; // read position command
constexpr byte nop = 0x00; // no operation
constexpr byte set_zero = 0x70;
constexpr unsigned long timeout = 200;

class Encoder {
public:
   bool m_usingRight;
   bool m_usingLeft;
   EncoderPositions m_encoders;
   Encoder(bool usingRight=true,bool usingLeft=true){
// we must define one class for both encoders because they share the SPI bus so having two instances would otherwise be complicated
     m_usingRight = usingRight;
     m_usingLeft = usingLeft;
   }
   void begin(){
    pinMode(boardConfig.ENCODER_RIGHT_CS,OUTPUT);
    pinMode(boardConfig.ENCODER_LEFT_CS,OUTPUT);
    digitalWrite(boardConfig.ENCODER_RIGHT_CS,HIGH);
    digitalWrite(boardConfig.ENCODER_LEFT_CS,HIGH);
    // initialize chip select (CS) pins as outputs and set the two encoders (even if one or none are used) in idle mode by giving a logic high on CS pins
    SPI_PORT.begin();
    SPI_PORT.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    }

    uint8_t SPIWrite(uint8_t sendByte, bool isLeft=true) // by default left
    {
           //holder for the received over SPI
           uint8_t data;
           //the AMT20 requires the release of the CS line after each byte
           digitalWrite(isLeft ? boardConfig.ENCODER_LEFT_CS : boardConfig.ENCODER_RIGHT_CS, LOW);
           data = SPI_PORT.transfer(sendByte);
           digitalWrite(isLeft ? boardConfig.ENCODER_LEFT_CS : boardConfig.ENCODER_RIGHT_CS, HIGH);
           //we will delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
           delayMicroseconds(10);

           return data;
    }
    uint16_t readEncoder(bool isLeft=true){
       unsigned long start = millis();
       data = SPIWrite(rd_pos,isLeft);
       delayMicroseconds(1);
       // is it better to use a numeric counter here ?
       while(SPIWrite(nop,isLeft) != rd_pos && millis() - start < timeout){
       }
       if(millis() - start >= timeout){
           //Serial.println("Error reading encoder");
           return INVALID_ENCODER_POSITION; // return NAN to signify invalid data
       }
       uint16_t encoder_position = (SPIWrite(nop,isLeft) & 0x0F) << 8; // read the first 4 bits, shift them up to make room for the next byte (lower eight bits)
       encoder_position |= SPIWrite(nop,isLeft); // read the next byte (=12 bits)

// Add relative angles computations here but then might need to find an alternative to INVALID_ENCODER_POSITION
       //int16_t relativePosition =
        //       ((int32_t)currentPosition - (int32_t)zeroPosition + 2048) % 4096 - 2048;
       return encoder_position;
   }
    EncoderPositions getPositions(){
       m_encoders.left_position = m_usingLeft ? readEncoder(true) : INVALID_ENCODER_POSITION;
       m_encoders.right_position = m_usingRight ? readEncoder(false) : INVALID_ENCODER_POSITION;
     return m_encoders;
    }
};
*/
#pragma once

#include <Arduino.h>
#include "Board.h"
#include <SPI.h>

struct EncoderPositions {
    uint16_t left_position;
    uint16_t right_position;
};

#if !defined(PLATFORM_TEENSY41) && !(defined(PLATFORM_RENESAS_RA) || defined(PLATFORM_ATMEL_AVR))
#else

#if defined(PLATFORM_TEENSY41)
  #define SPI_PORT SPI1
#elif defined(PLATFORM_RENESAS_RA) || defined(PLATFORM_ATMEL_AVR)
  #define SPI_PORT SPI
#else
  #error "No SPI port selected"
#endif

constexpr uint16_t INVALID_ENCODER_POSITION = UINT16_MAX; // when there is no valid data return maximum unsigned int16

constexpr byte rd_pos = 0x10;
constexpr byte nop = 0x00;
constexpr byte set_zero = 0x70;

constexpr uint16_t timeoutLimit = 100;

class Encoder {
public:
    bool m_usingLeft;
    bool m_usingRight;

    EncoderPositions m_encoders;

    Encoder(bool usingLeft = true, bool usingRight = true)
        : m_usingLeft(usingLeft),
          m_usingRight(usingRight),
          m_encoders{INVALID_ENCODER_POSITION, INVALID_ENCODER_POSITION}
    {}

    void begin()
    {
        pinMode(boardConfig.ENCODER_LEFT_CS, OUTPUT);
        pinMode(boardConfig.ENCODER_RIGHT_CS, OUTPUT);


        // idle BOTH encoders, even if only one is used
        digitalWrite(boardConfig.ENCODER_LEFT_CS, HIGH);
        digitalWrite(boardConfig.ENCODER_RIGHT_CS, HIGH);

        SPI_PORT.begin();
    }

    uint8_t SPIWrite(uint8_t sendByte, bool isLeft = true)
    {
        const int csPin = isLeft
            ? boardConfig.ENCODER_LEFT_CS
            : boardConfig.ENCODER_RIGHT_CS;

        uint8_t data;

        SPI_PORT.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0)); // 500Khz for debugging

        digitalWrite(csPin, LOW);
        data = SPI_PORT.transfer(sendByte);
        digitalWrite(csPin, HIGH);

        SPI_PORT.endTransaction();

        delayMicroseconds(10);

        return data;
    }

    uint16_t readEncoder(bool isLeft = true)
    {
        uint8_t data;
        uint16_t timeoutCounter = 0;


        data = SPIWrite(rd_pos, isLeft);

        while (data != rd_pos && timeoutCounter++ < timeoutLimit)
        {
            data = SPIWrite(nop, isLeft);
        }

        if (timeoutCounter >= timeoutLimit)
        {
            return INVALID_ENCODER_POSITION;
        }

        uint16_t encoder_position = 0;

        encoder_position = (SPIWrite(nop, isLeft) & 0x0F) << 8;
        encoder_position |= SPIWrite(nop, isLeft);

        return encoder_position;
    }

    EncoderPositions getPositions()
    {
        m_encoders.left_position =
            m_usingLeft ? readEncoder(true) : INVALID_ENCODER_POSITION;

        if (m_usingLeft && m_usingRight)
        {
            delayMicroseconds(20);
        }

        m_encoders.right_position =
            m_usingRight ? readEncoder(false) : INVALID_ENCODER_POSITION;

        return m_encoders;
    }
};
#endif