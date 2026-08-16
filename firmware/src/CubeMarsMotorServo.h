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

//note for compatibility with MIT code
struct CanFrame
{
    uint32_t id{0};   // complete CAN identifier, including Servo EID
    uint8_t length{0};
    std::array<uint8_t, 8> data{};
	// bool extended {}; we assume that any can frame that makes it through the read() function is EXT. So no need to specify here
};

enum CAN_PACKET_ID : uint32_t {
    CAN_PACKET_SET_DUTY          = 0,
    CAN_PACKET_SET_CURRENT       = 1,
    CAN_PACKET_SET_CURRENT_BRAKE = 2,
    CAN_PACKET_SET_RPM           = 3,
    CAN_PACKET_SET_POS           = 4,
    CAN_PACKET_SET_ORIGIN_HERE   = 5,
    CAN_PACKET_SET_POS_SPD       = 6,
    CAN_PACKET_SET_MIT           = 8
};

struct ServoMotorReply {
    uint8_t motor_eid = 0;
    uint32_t func_id = 0;

    float position_deg = 0.0f;
    float speed_erpm = 0.0f;
    float current_a = 0.0f;
    int8_t temperature_c = 0;
    uint8_t error = 0;

    bool valid_feedback = false;
};

//note : for compatibility with MIT version
struct MotorReply
{
    uint8_t can_id{0};
    float position{0.0f};
    float velocity{0.0f};
    float torque{0.0f};
    uint8_t temperature{0};
    uint8_t error{0};
};

//note : a new struct which only transmits useful informations to the user. If you need position and/or velocity add it here
struct MotorFeedback
{
   float torque{0.0f};
   uint8_t temperature{0};
   uint8_t error{0};
};

// -----------------

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

inline constexpr float currentFactor = 0.468f; // τ = 0.468*I

struct AK60Params {
    float p_min;
    float p_max;

    float erpm_min;
    float erpm_max;

    float i_min;
    float i_max;

    float i_brake_min;
    float i_brake_max;
};

inline constexpr AK60Params MotorParams  {
    -3200.0f, 3200.0f,      // position, deg
    -320000.0f, 320000.0f,  // speed, ERPM
    -60.0f, 60.0f,          // current, A
    0.0f, 60.0f             // brake current, A
};

constexpr uint32_t SERVO_JUMP_START_STATE   = 0x09;
constexpr uint32_t SERVO_ENTER_MODE_FRAME   = 0x2C;
constexpr uint32_t SERVO_REALTIME_FEEDBACK  = 0x29;

#if defined(PLATFORM_TEENSY)

class CanBus
{
public:
    bool begin() {

        if(isReady()){
            return true;
        }
        m_can.begin();
        m_can.setBaudRate(board::teensy41::motorCanBaud);
        m_can.setMaxMB(16);
        m_can.enableFIFO();
        m_isInitialized = true;
        return true;
    }

    bool send(const CanFrame& frame)
    {
        if (!m_isInitialized || frame.length > 8) {
            return false;
        }

        CAN_message_t msg{};

        msg.id = frame.id;
        msg.flags.extended = 1;
        msg.len = frame.length;

        std::memcpy(
            msg.buf,
            frame.data.data(),
            frame.length
        );

        return m_can.write(msg) > 0;
    }

    bool read(CanFrame& frame)
    {
        CAN_message_t rxMsg{};

        while (m_can.read(rxMsg))
        {
            if (!rxMsg.flags.extended) {
                continue;
            }

            if (rxMsg.len > 8) {
                continue;
            }

            frame.id = rxMsg.id; //note this is the RAW id (EID) in servo mode this contains more than the simple can ID
            frame.length = rxMsg.len;

            std::memcpy(
                frame.data.data(),
                rxMsg.buf,
                rxMsg.len
            );

            return true;
        }

        return false;
    }


    bool isReady() const
    {
        return m_isInitialized;
    }

private:
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> m_can;
    bool m_isInitialized{false};
};

class CubeMarsMotor
{
public:
    CubeMarsMotor(
        uint8_t canId, //TODO ENSURE COMPATIBILITY B/W STD AND EXT ID
        const AK60Params& motorSoftwareLimits)
        : m_canId(canId),
          m_softwareLimits(motorSoftwareLimits)
    {}

    bool begin() { return true; }

    uint8_t canId() const
    {
        return m_canId;
    }


    CanFrame packTorqueCommand(float torqueNm) const
    {
        CanFrame frame{};

        // Torque -> Iq current
        float currentA = torqueNm / currentFactor;

        currentA = std::clamp(
            currentA,
            m_softwareLimits.i_min,
            m_softwareLimits.i_max
        );

        const int32_t currentRaw =
            static_cast<int32_t>(currentA * 1000.0f);

        frame.id =
            makeServoEid(CAN_PACKET_SET_CURRENT);

        frame.length = 4;

        write_i32_be(
            frame.data.data(),
            currentRaw
        );

        return frame;
    }

    CanFrame packCommand(const MotorCmd& cmd) const {
        //note : for compatibility with MIT version
        return packTorqueCommand(cmd.torque); // we only care about torque
    }

    CanFrame packCommand(const float torque) const {

        return packTorqueCommand(torque); // new method that overloads the other
    }


    MotorReply unpackReply(const CanFrame& frame) const
    {
        const ServoMotorReply servo =
            unpackServoReply(frame);

        MotorReply reply{};

        if (!servo.valid_feedback) {
            return reply;
        }

        reply.can_id = servo.motor_eid;

        // degrees -> radians
        reply.position =
            servo.position_deg * PI / 180.0f;

        // You'll need to decide exactly how you want
        // ERPM converted to your previous velocity units.
        reply.velocity = servo.speed_erpm;

        // Servo gives current; estimate output torque.
        reply.torque =
            servo.current_a * currentFactor;

        reply.temperature =
            static_cast<uint8_t>(servo.temperature_c);

        reply.error = servo.error;

        return reply;
    }

    ServoMotorReply unpackServoReply(const CanFrame& frame) const
    {
        ServoMotorReply reply{};

        const uint32_t eid =
            frame.id & 0x1FFFFFFF;

        reply.motor_eid =
            static_cast<uint8_t>(eid & 0xFF);

        reply.func_id =
            (eid >> 8) & 0x1FFFFF;

        if (frame.length != 8) {
            return reply;
        }

        if (reply.func_id != SERVO_REALTIME_FEEDBACK) {
            return reply;
        }

        const int16_t posRaw =
            read_i16_be(&frame.data[0]);

        const int16_t speedRaw =
            read_i16_be(&frame.data[2]);

        const int16_t currentRaw =
            read_i16_be(&frame.data[4]);

        reply.position_deg =
            static_cast<float>(posRaw) * 0.1f;

        reply.speed_erpm =
            static_cast<float>(speedRaw) * 10.0f;

        reply.current_a =
            static_cast<float>(currentRaw) * 0.01f;

        reply.temperature_c =
            static_cast<int8_t>(frame.data[6]);

        reply.error =
            frame.data[7];

        reply.valid_feedback = true;

        return reply;
    }

    void enterMotorMode(){return;};
    void exitMotorMode(){return;};
    void setZeroPosition() {return;};

private:

    uint32_t m_canId;
    const AK60Params  m_softwareLimits;

    uint32_t makeServoEid(CAN_PACKET_ID packetId) const
    {
        return
            (static_cast<uint32_t>(packetId) << 8) |
            static_cast<uint32_t>(m_canId);
    }

    static void write_i32_be(
        uint8_t* buf,
        int32_t value)
    {
        const uint32_t u =
            static_cast<uint32_t>(value);

        buf[0] = static_cast<uint8_t>(u >> 24);
        buf[1] = static_cast<uint8_t>(u >> 16);
        buf[2] = static_cast<uint8_t>(u >> 8);
        buf[3] = static_cast<uint8_t>(u);
    }

    static int16_t read_i16_be(const uint8_t* buf)
    {
        return static_cast<int16_t>(
            (static_cast<uint16_t>(buf[0]) << 8) |
             static_cast<uint16_t>(buf[1])
        );
    }
};

#endif