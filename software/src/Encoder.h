#pragma once

#include <Arduino.h>
#include "Board.h"
#include "ProtocolTypes.h"
#include <SPI.h>

/*
 * AMT20 / AMT203 SPI encoder interface.
 *
 * Protocol:
 * 1. Send read-position command 0x10.
 * 2. Discard the byte received during that transfer.
 * 3. Repeatedly send NOP 0x00.
 * 4. Encoder returns 0xA5 while processing.
 * 5. Encoder returns 0x10 when the position is ready.
 * 6. Send two more NOP bytes to receive the 12-bit position.
 *
 * The AMT20 requires CS to return HIGH after every byte.
 */

#if defined(PLATFORM_TEENSY41)

#define ENCODER_SPI_PORT SPI1


class Encoder
{
public:
    static constexpr uint16_t INVALID_POSITION = UINT16_MAX;

    static constexpr uint8_t READ_POSITION_COMMAND = 0x10;
    static constexpr uint8_t WAIT_RESPONSE = 0xA5;
    static constexpr uint8_t NOP_COMMAND = 0x00;
    static constexpr uint8_t SET_ZERO_COMMAND = 0x70;

    static constexpr uint32_t SPI_FREQUENCY_HZ = 100000;
    static constexpr uint16_t MAX_POLL_ATTEMPTS = 100;

    static constexpr uint32_t CS_SETUP_DELAY_US = 2;
    static constexpr uint32_t CS_HOLD_DELAY_US = 2;
    static constexpr uint32_t INTER_BYTE_DELAY_US = 20;
    static constexpr uint32_t BETWEEN_ENCODERS_DELAY_US = 20;

    Encoder(
        bool usingLeft = true,
        bool usingRight = true
    )
        : m_usingLeft(usingLeft),
          m_usingRight(usingRight),
          m_encoders{
              INVALID_POSITION,
              INVALID_POSITION
          }
    {
    }

    void begin()
    {
        // Enable the onboard voltage shifter (otherwise it holds lines in high impedance)

        pinMode(board::teensy41::pins.levelShifters.oe1, OUTPUT);
        pinMode(board::teensy41::pins.levelShifters.oe2, OUTPUT);
        digitalWrite(board::teensy41::pins.levelShifters.oe1,HIGH);
        digitalWrite(board::teensy41::pins.levelShifters.oe2,HIGH);

        pinMode(board::teensy41::pins.encoders.leftChipSelect, OUTPUT);
        pinMode(board::teensy41::pins.encoders.rightChipSelect, OUTPUT);

        /*
         * Both encoders must remain deselected while the SPI
         * peripheral is initialized.
         */
        digitalWrite(board::teensy41::pins.encoders.leftChipSelect, HIGH);
        digitalWrite(board::teensy41::pins.encoders.rightChipSelect, HIGH);

        ENCODER_SPI_PORT.begin();

        delayMicroseconds(100);
    }

    uint16_t readEncoder(bool isLeft = true)
    {
        const uint8_t csPin = getChipSelectPin(isLeft);

        /*
         * Ensure the other encoder is deselected before starting.
         * This helps prevent both devices from driving MISO.
         */
        deselectBothEncoders();

        ENCODER_SPI_PORT.beginTransaction(
            SPISettings(
                SPI_FREQUENCY_HZ,
                MSBFIRST,
                SPI_MODE0
            )
        );

        /*
         * Send the read-position command.
         *
         * The byte received during this transfer does not correspond
         * to the command that is currently being sent, so discard it.
         */
        transferByte(
            READ_POSITION_COMMAND,
            csPin
        );

        bool acknowledged = false;
        uint8_t response = 0;

        for (
            uint16_t attempt = 0;
            attempt < MAX_POLL_ATTEMPTS;
            ++attempt
        )
        {
            response = transferByte(
                NOP_COMMAND,
                csPin
            );

            if (response == READ_POSITION_COMMAND)
            {
                acknowledged = true;
                break;
            }

            /*
             * 0xA5 is the normal response while the encoder
             * is still calculating its position.
             *
             * Other values are ignored here but may indicate
             * electrical corruption or bus contention.
             */
        }

        if (!acknowledged)
        {
            ENCODER_SPI_PORT.endTransaction();
            deselectBothEncoders();

            return INVALID_POSITION;
        }

        const uint8_t positionHighByte =
            transferByte(
                NOP_COMMAND,
                csPin
            );

        const uint8_t positionLowByte =
            transferByte(
                NOP_COMMAND,
                csPin
            );

        ENCODER_SPI_PORT.endTransaction();

        deselectBothEncoders();

        /*
         * Position is 12 bits:
         *
         * high byte: only bits 3:0 are position bits
         * low byte:  all 8 bits are position bits
         */
        const uint16_t position =
            (
                static_cast<uint16_t>(
                    positionHighByte & 0x0F
                )
                << 8
            )
            |
            static_cast<uint16_t>(
                positionLowByte
            );

        return position;
    }

    EncoderPositions getPositions()
    {
        if (m_usingLeft)
        {
            m_encoders.left_position =
                readEncoder(true);
        }
        else
        {
            m_encoders.left_position =
                INVALID_POSITION;
        }

        if (m_usingLeft && m_usingRight)
        {
            delayMicroseconds(
                BETWEEN_ENCODERS_DELAY_US
            );
        }

        if (m_usingRight)
        {
            m_encoders.right_position =
                readEncoder(false);
        }
        else
        {
            m_encoders.right_position =
                INVALID_POSITION;
        }

        return m_encoders;
    }

    bool setZero(bool isLeft = true)
    {
        const uint8_t csPin = getChipSelectPin(isLeft);

        deselectBothEncoders();

        ENCODER_SPI_PORT.beginTransaction(
            SPISettings(
                SPI_FREQUENCY_HZ,
                MSBFIRST,
                SPI_MODE0
            )
        );

        /*
         * As with the read command, discard the response received
         * while sending the set-zero command.
         */
        transferByte(
            SET_ZERO_COMMAND,
            csPin
        );

        bool acknowledged = false;

        for (
            uint16_t attempt = 0;
            attempt < MAX_POLL_ATTEMPTS;
            ++attempt
        )
        {
            const uint8_t response =
                transferByte(
                    NOP_COMMAND,
                    csPin
                );

            if (response == SET_ZERO_COMMAND)
            {
                acknowledged = true;
                break;
            }
        }

        ENCODER_SPI_PORT.endTransaction();

        deselectBothEncoders();

        return acknowledged;
    }

    static bool isValidPosition(uint16_t position)
    {
        return position != INVALID_POSITION;
    }

private:
    bool m_usingLeft;
    bool m_usingRight;

    EncoderPositions m_encoders;

    uint8_t getChipSelectPin(bool isLeft) const
    {
        return isLeft
            ? board::teensy41::pins.encoders.leftChipSelect
            : board::teensy41::pins.encoders.rightChipSelect;
    }

    void deselectBothEncoders()
    {
        digitalWrite(
            board::teensy41::pins.encoders.leftChipSelect,
            HIGH
        );

        digitalWrite(
            board::teensy41::pins.encoders.rightChipSelect,
            HIGH
        );
    }

    uint8_t transferByte(
        uint8_t transmittedByte,
        uint8_t csPin
    )
    {
        /*
         * The AMT20 protocol requires CS to be pulsed separately
         * for every transmitted byte.
         */
        digitalWrite(csPin, LOW);

        delayMicroseconds(
            CS_SETUP_DELAY_US
        );

        const uint8_t receivedByte =
            ENCODER_SPI_PORT.transfer(
                transmittedByte
            );

        delayMicroseconds(
            CS_HOLD_DELAY_US
        );

        digitalWrite(csPin, HIGH);

        delayMicroseconds(
            INTER_BYTE_DELAY_US
        );

        return receivedByte;
    }
};
#else
#endif