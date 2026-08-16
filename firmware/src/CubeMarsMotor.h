#pragma once

#include <Arduino.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

#include "board.h"

#if defined(PLATFORM_TEENSY)
#include <FlexCAN_T4.h>
#endif

// ====== Definitions =====
struct MotorReply
{
    uint8_t can_id{0};
    float position{0.0f};
    float velocity{0.0f};
    float torque{0.0f};
    uint8_t temperature{0};
    uint8_t error{0};
};

enum class MotorMetaCommand : uint8_t
{
    EnterMotorMode,
    ExitMotorMode,
    SetZero
};

struct MotorCmd
{
    float position{0.0f};
    float velocity{0.0f};
    float torque{0.0f};
    float kp{0.0f};
    float kd{0.0f};
};

struct CanFrame
{
    uint32_t id{0};
    uint8_t length{0};
    bool extended{false};
    std::array<uint8_t, 8> data{};
};

struct AK60Params
{
    float p_min, p_max;
    float v_min, v_max;
    float kp_min, kp_max;
    float kd_min, kd_max;
    float trq_min, trq_max;
};

inline constexpr AK60Params MotorParams{
    -12.5f, 12.5f,
    -45.0f, 45.0f,
    0.0f, 500.0f,
    0.0f, 5.0f,
    -15.0f, 15.0f
};

inline float uint_to_float(
    uint16_t code,
    float x_min,
    float x_max,
    int bits)
{
    const float span = x_max - x_min;
    const float max_int =
        static_cast<float>((1UL << bits) - 1UL);

    return
        static_cast<float>(code) *
        span / max_int +
        x_min;
}

inline uint16_t float_to_uint(
    float x,
    float x_min,
    float x_max,
    int bits)
{
    x = std::clamp(x, x_min, x_max);

    const float span = x_max - x_min;
    const float max_int =
        static_cast<float>((1UL << bits) - 1UL);

    return static_cast<uint16_t>(
        (x - x_min) * max_int / span
    );
}

#if defined(PLATFORM_TEENSY)

class CanBus
{
public:

    bool begin()
    {
        if (isReady()) {
            return true;
        }

        m_can.begin();

        m_can.setBaudRate(
            board::teensy41::motorCanBaud
        );

        m_can.setMaxMB(16);

        m_can.enableFIFO();

        // Enable interrupt-driven FIFO reception.
        m_can.enableFIFOInterrupt();

        // FlexCAN_T4 callback.
        s_instance = this;

        m_can.onReceive(
            FIFO,
            onReceiveStatic
        );

        m_isInitialized = true;

        return true;
    }


    bool send(
        const CanFrame& frame)
    {
        if (
            !m_isInitialized ||
            frame.length > 8
        )
        {
            return false;
        }

        CAN_message_t msg{};

        msg.id =
            frame.id;

        msg.len =
            frame.length;

        // Servo mode uses extended 29-bit IDs.
        msg.flags.extended = 1;

        std::memcpy(
            msg.buf,
            frame.data.data(),
            frame.length
        );

        return
            m_can.write(msg) > 0;
    }


    // =====================================================
    // Scheduler-side access
    // =====================================================
    //
    // This DOES NOT touch the CAN hardware.
    // It only removes a frame already copied into RAM.
    // =====================================================

    bool read(
        CanFrame& frame)
    {
        noInterrupts();

        if (m_tail == m_head)
        {
            interrupts();
            return false;
        }

        frame =
            m_rxBuffer[m_tail];

        m_tail =
            nextIndex(m_tail);

        interrupts();

        return true;
    }


    bool isReady() const
    {
        return m_isInitialized;
    }


    uint32_t droppedFrames() const
    {
        return m_droppedFrames;
    }


private:

    // =====================================================
    // FlexCAN callback
    // =====================================================

    static void onReceiveStatic(
        const CAN_message_t& msg)
    {
        if (s_instance != nullptr)
        {
            s_instance->onReceive(
                msg
            );
        }
    }


    void onReceive(
        const CAN_message_t& msg)
    {
        // Keep callback extremely short.

        if (msg.len != 8) {
            return;
        }

        const size_t next =
            nextIndex(m_head);


        // Buffer full:
        // drop newest frame rather than blocking.
        if (next == m_tail)
        {
            ++m_droppedFrames;
            return;
        }


        CanFrame& frame =
            m_rxBuffer[m_head];


        frame.id =
            msg.id;

        frame.length =
            msg.len;
		frame.extended =
    		msg.flags.extended;

        std::memcpy(
            frame.data.data(),
            msg.buf,
            msg.len
        );


        m_head =
            next;
    }


    static constexpr size_t
    nextIndex(size_t index)
    {
        return
            (index + 1) %
            RX_BUFFER_SIZE;
    }


private:

    static constexpr size_t
        RX_BUFFER_SIZE = 64;


    FlexCAN_T4<
        CAN1,
        RX_SIZE_256,
        TX_SIZE_16
    >
        m_can;


    std::array<
        CanFrame,
        RX_BUFFER_SIZE
    >
        m_rxBuffer{};


    volatile size_t
        m_head{0};


    volatile size_t
        m_tail{0};


    volatile uint32_t
        m_droppedFrames{0};


    bool
        m_isInitialized{false};


    static CanBus*
        s_instance;
};

// Cubemars protocol interface
class CubeMarsMotor
{
public:
    CubeMarsMotor(
        uint8_t canId,
        const AK60Params& motorSoftwareLimits)
        : m_canId(canId),
          m_softwareLimits(motorSoftwareLimits)
    {}

    CanFrame enterMotorMode() const
    {
        return specialFrame({
            0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFC
        });
    }

    CanFrame exitMotorMode() const
    {
        return specialFrame({
            0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFD
        });
    }

    CanFrame setZeroPosition() const
    {
        return specialFrame({
            0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFE
        });
    }

    bool begin()
    {
        return true;
    //TODO IMPORTANT : MAKE A "HEALTH" CHECK TO ENSURE THE MOTORS ARE PROPERLY CONNECTED.
    // IF THE CAN CABLES ARE MISCONNECTED / DISCONNECTED THE BEGIN WILL STILL WORK WHICH IS A SILENT ERROR. HARD TO TROUBLESHOOT
    }

    uint8_t canId() const
    {
        return m_canId;
    }

    CanFrame packCommand(const MotorCmd& cmd) const
    {
        const uint16_t position = float_to_uint(
            std::clamp(
                cmd.position,
                m_softwareLimits.p_min,
                m_softwareLimits.p_max
            ),
            MotorParams.p_min,
            MotorParams.p_max,
            16
        );

        const uint16_t velocity = float_to_uint(
            std::clamp(
                cmd.velocity,
                m_softwareLimits.v_min,
                m_softwareLimits.v_max
            ),
            MotorParams.v_min,
            MotorParams.v_max,
            12
        );

        const uint16_t kp = float_to_uint(
            std::clamp(
                cmd.kp,
                m_softwareLimits.kp_min,
                m_softwareLimits.kp_max
            ),
            MotorParams.kp_min,
            MotorParams.kp_max,
            12
        );

        const uint16_t kd = float_to_uint(
            std::clamp(
                cmd.kd,
                m_softwareLimits.kd_min,
                m_softwareLimits.kd_max
            ),
            MotorParams.kd_min,
            MotorParams.kd_max,
            12
        );

        const uint16_t trq = float_to_uint(
            std::clamp(
                cmd.torque,
                m_softwareLimits.trq_min,
                m_softwareLimits.trq_max
            ),
            MotorParams.trq_min,
            MotorParams.trq_max,
            12
        );

        CanFrame frame{};
        frame.length = 8;
        frame.id = m_canId;

        frame.data[0] = static_cast<uint8_t>(position >> 8);
        frame.data[1] = static_cast<uint8_t>(position & 0xFF);

        frame.data[2] = static_cast<uint8_t>(velocity >> 4);
        frame.data[3] =
            static_cast<uint8_t>(
                ((velocity & 0x0F) << 4) |
                ((kp >> 8) & 0x0F)
            );

        frame.data[4] = static_cast<uint8_t>(kp & 0xFF);
        frame.data[5] = static_cast<uint8_t>(kd >> 4);

        frame.data[6] =
            static_cast<uint8_t>(
                ((kd & 0x0F) << 4) |
                ((trq >> 8) & 0x0F)
            );

        frame.data[7] = static_cast<uint8_t>(trq & 0xFF);

        return frame;
    }

    MotorReply unpackReply(const CanFrame& frame) const
    {
        MotorReply reply{};
        reply.can_id = static_cast<uint8_t>(frame.id);

        // Kept from your current reply packing convention.
        const uint16_t position_raw =
            (static_cast<uint16_t>(frame.data[1]) << 8) |
            static_cast<uint16_t>(frame.data[2]);

        const uint16_t velocity_raw =
            (static_cast<uint16_t>(frame.data[3]) << 4) |
            (static_cast<uint16_t>(frame.data[4] & 0xF0) >> 4);

        const uint16_t trq_raw =
            (static_cast<uint16_t>(frame.data[4] & 0x0F) << 8) |
            static_cast<uint16_t>(frame.data[5]);

        reply.position = uint_to_float(
            position_raw,
            MotorParams.p_min,
            MotorParams.p_max,
            16
        );

        reply.velocity = uint_to_float(
            velocity_raw,
            MotorParams.v_min,
            MotorParams.v_max,
            12
        );

        reply.torque = uint_to_float(
            trq_raw,
            MotorParams.trq_min,
            MotorParams.trq_max,
            12
        );

        reply.temperature = frame.data[6];
        reply.error = frame.data[7];

        return reply;
    }

private:

    CanFrame specialFrame(
        const std::array<uint8_t, 8>& data) const
    {
        CanFrame frame{};

        frame.id = m_canId;
        frame.length = 8;
        frame.data = data;

        return frame;
    }


    uint8_t m_canId;
    AK60Params m_softwareLimits;
};

#endif
