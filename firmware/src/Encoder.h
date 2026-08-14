#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <cstdint>

#include "board.h"

enum class Side : uint8_t
{
    Left,
    Right,
};

struct EncoderRawPositions
{
    uint16_t left;
    uint16_t right;
};

struct EncoderPositions
{
    float left;
    float right;
};

#if defined(PLATFORM_TEENSY)

class Encoder
{
public:
    static constexpr uint16_t INVALID_POSITION = UINT16_MAX;
    static constexpr uint8_t READ_POSITION_COMMAND = 0x10;
    static constexpr uint8_t NOP_COMMAND = 0x00;

    static constexpr uint16_t MAX_POLL_ATTEMPTS = 100;

    static constexpr uint32_t CS_SETUP_DELAY_US = 2;
    static constexpr uint32_t CS_HOLD_DELAY_US = 2;
    static constexpr uint32_t INTER_BYTE_DELAY_US = 20;

    explicit Encoder(Side side)
        : m_side(side)
    {}

    void begin()
    {
        m_csPin =
            (m_side == Side::Left)
                ? board::teensy41::pins.encoders.leftChipSelect
                : board::teensy41::pins.encoders.rightChipSelect;

        pinMode(m_csPin, OUTPUT);
        digitalWrite(m_csPin, HIGH);

        SPI1.begin();
        delayMicroseconds(100);
    }

    uint16_t getPosition()
    {
        SPI1.beginTransaction(
            SPISettings(
                board::teensy41::encoderSPIBaud,
                MSBFIRST,
                SPI_MODE0
            )
        );

        transferByte(READ_POSITION_COMMAND);

        bool acknowledged = false;

        for (uint16_t attempt = 0;
             attempt < MAX_POLL_ATTEMPTS;
             ++attempt)
        {
            const uint8_t response =
                transferByte(NOP_COMMAND);

            if (response == READ_POSITION_COMMAND)
            {
                acknowledged = true;
                break;
            }
        }

        if (!acknowledged)
        {
            SPI1.endTransaction();
            deselectEncoder();
            return INVALID_POSITION;
        }

        const uint8_t positionHighByte =
            transferByte(NOP_COMMAND);

        const uint8_t positionLowByte =
            transferByte(NOP_COMMAND);

        SPI1.endTransaction();
        deselectEncoder();

        return
            (static_cast<uint16_t>(
                positionHighByte & 0x0F
             ) << 8)
            |
            static_cast<uint16_t>(positionLowByte);
    }

    static bool isValidPosition(uint16_t position)
    {
        return position != INVALID_POSITION;
    }

private:
    void deselectEncoder()
    {
        digitalWrite(m_csPin, HIGH);
    }

    void selectEncoder()
    {
        digitalWrite(m_csPin, LOW);
    }

    uint8_t transferByte(uint8_t transmittedByte)
    {
        selectEncoder();

        delayMicroseconds(CS_SETUP_DELAY_US);

        const uint8_t receivedByte =
            SPI1.transfer(transmittedByte);

        delayMicroseconds(CS_HOLD_DELAY_US);

        deselectEncoder();

        delayMicroseconds(INTER_BYTE_DELAY_US);

        return receivedByte;
    }

private:
    Side m_side;
    Pin m_csPin{-1};
};

#endif
