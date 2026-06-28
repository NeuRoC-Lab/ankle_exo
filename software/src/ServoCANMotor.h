#pragma once

#include <Arduino.h>

#if defined(PLATFORM_RENESAS_RA)
    #include <Arduino_CAN.h>
#endif

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


struct MotorCmd {
    CAN_PACKET_ID packetID = CAN_PACKET_SET_DUTY;
    uint8_t data[8] = {0};
    uint8_t len = 4;
}; // this will be overriden in MIT. this format only holds for Servo mode

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

static constexpr AK60Params motorParams = {
    -3200.0f, 3200.0f,      // position, deg
    -320000.0f, 320000.0f,  // speed, ERPM
    -60.0f, 60.0f,          // current, A
    0.0f, 60.0f             // brake current, A
};



constexpr uint32_t SERVO_JUMP_START_STATE   = 0x09;
constexpr uint32_t SERVO_ENTER_MODE_FRAME   = 0x2C;
constexpr uint32_t SERVO_REALTIME_FEEDBACK  = 0x29;

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

class ServoCANMotor {
public:
    ServoCANMotor(byte canId, const AK60Params* motorSettings,MotorCmd& cmd, uint32_t kPrintEvery = 20)
        : m_canId(canId),
          m_motorSettings(motorSettings),
          m_cmd(cmd),
          m_kPrintEvery(kPrintEvery)
    {}

    virtual ~ServoCANMotor() = default;

    uint8_t m_canId;
    MotorCmd& m_cmd;

    virtual void begin() = 0;
    virtual bool sendMessage(CAN_PACKET_ID packetId, const uint8_t* data, uint8_t len) = 0;
    virtual bool readMessages(ServoMotorReply& reply) = 0;

    void update() {
        ServoMotorReply reply;

        while (readMessages(reply)) {
            if (++m_printCounter >= m_kPrintEvery) {
                print_can_msg(reply);
                m_printCounter = 0;
            }
        }

        sendMessage(m_cmd.packetID, m_cmd.data, m_cmd.len);
    }

    bool resetMotor() {
        can_set_duty(0.0f);
        return true;
    }

    void can_set_duty(float duty) {
        m_cmd.packetID = CAN_PACKET_SET_DUTY;
        m_cmd.len = 4;
        memset(m_cmd.data, 0, sizeof(m_cmd.data));
        write_i32_be(m_cmd.data, (int32_t)(duty * 100000.0f));
    }

    void can_set_current(float current_a) {
        m_cmd.packetID = CAN_PACKET_SET_CURRENT;
        m_cmd.len = 4;
        memset(m_cmd.data, 0, sizeof(m_cmd.data));
        write_i32_be(m_cmd.data, (int32_t)(current_a * 1000.0f));
    }

    void can_set_current_brake(float brake_current_a) {
        m_cmd.packetID = CAN_PACKET_SET_CURRENT_BRAKE;
        m_cmd.len = 4;
        memset(m_cmd.data, 0, sizeof(m_cmd.data));
        write_i32_be(m_cmd.data, (int32_t)(brake_current_a * 1000.0f));
    }

    void can_set_rpm(float erpm) {
        m_cmd.packetID = CAN_PACKET_SET_RPM;
        m_cmd.len = 4;
        memset(m_cmd.data, 0, sizeof(m_cmd.data));
        write_i32_be(m_cmd.data, (int32_t)erpm);
    }

    void can_set_position(float position_deg) {
        m_cmd.packetID = CAN_PACKET_SET_POS;
        m_cmd.len = 4;
        memset(m_cmd.data, 0, sizeof(m_cmd.data));
        write_i32_be(m_cmd.data, (int32_t)(position_deg * 10000.0f));
    }

    bool can_set_origin(uint8_t origin_mode) {
        uint8_t buf[1] = { origin_mode };
        return sendMessage(CAN_PACKET_SET_ORIGIN_HERE, buf, sizeof(buf));
    }

    void can_set_position_speed(
        float position_deg,
        float speed_erpm,
        float accel_erpm_s2
    ) {
        m_cmd.packetID = CAN_PACKET_SET_POS_SPD;
        m_cmd.len = 8;

        memset(m_cmd.data, 0, sizeof(m_cmd.data));

        write_i32_be(&m_cmd.data[0], (int32_t)(position_deg * 10000.0f));
        write_i16_be(&m_cmd.data[4], (int16_t)(speed_erpm / 10.0f));
        write_i16_be(&m_cmd.data[6], (int16_t)(accel_erpm_s2 / 10.0f));
    }

protected:
    uint32_t m_printCounter = 0;
    uint32_t m_kPrintEvery;
    const AK60Params* m_motorSettings;

    uint32_t make_servo_eid(CAN_PACKET_ID packet_id) const {
        return ((uint32_t)packet_id << 8) | m_canId;
    }

    static int16_t read_i16_be(const uint8_t* buf) {
        return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    }

    static void write_i16_be(uint8_t* buf, int16_t value) {
        uint16_t u = (uint16_t)value;
        buf[0] = (uint8_t)(u >> 8);
        buf[1] = (uint8_t)(u);
    }

    static void write_i32_be(uint8_t* buf, int32_t value) {
        uint32_t u = (uint32_t)value;
        buf[0] = (uint8_t)(u >> 24);
        buf[1] = (uint8_t)(u >> 16);
        buf[2] = (uint8_t)(u >> 8);
        buf[3] = (uint8_t)(u);
    }

    ServoMotorReply unpack_servo_reply(uint32_t eid, const uint8_t data[8], uint8_t dlc) {
        ServoMotorReply reply;

        reply.motor_eid = eid & 0xFF;
        reply.func_id = (eid >> 8) & 0x1FFFFF;

        if (dlc != 8) {
            return reply;
        }

        if (reply.func_id == SERVO_REALTIME_FEEDBACK) {
            int16_t pos_raw = read_i16_be(&data[0]);
            int16_t spd_raw = read_i16_be(&data[2]);
            int16_t cur_raw = read_i16_be(&data[4]);

            reply.position_deg = (float)pos_raw * 0.1f;
            reply.speed_erpm = (float)spd_raw * 10.0f;
            reply.current_a = (float)cur_raw * 0.01f;
            reply.temperature_c = (int8_t)data[6];
            reply.error = data[7];
            reply.valid_feedback = true;
        }

        return reply;
    }

    void print_error_code(uint8_t error) {
        if (error == 0) {
            Serial.print("OK");
            return;
        }

        Serial.print("ERR_0x");
        Serial.print(error, HEX);
    }

    void print_can_msg(const ServoMotorReply& reply) {
        Serial.print("eid=");
        Serial.print(reply.motor_eid);

        Serial.print(" func_id=0x");
        Serial.print(reply.func_id, HEX);

        if (!reply.valid_feedback) {
            if (reply.func_id == SERVO_JUMP_START_STATE) {
                Serial.println(" jump-start state");
            } else if (reply.func_id == SERVO_ENTER_MODE_FRAME) {
                Serial.println(" enter-servo-mode frame");
            } else {
                Serial.println(" non-feedback frame");
            }
            return;
        }

        Serial.print(" pos_deg=");
        Serial.print(reply.position_deg, 2);

        Serial.print(" speed_erpm=");
        Serial.print(reply.speed_erpm, 1);

        Serial.print(" current_a=");
        Serial.print(reply.current_a, 3);

        Serial.print(" temp_c=");
        Serial.print(reply.temperature_c);

        Serial.print(" error=");
        Serial.print(reply.error);
        Serial.print(" ");
        print_error_code(reply.error);

        Serial.println();
    }
};

#if defined(PLATFORM_RENESAS_RA)

class ServoCANMotor_Renesas : public ServoCANMotor {
public:
    ServoCANMotor_Renesas(byte canId, const AK60Params* motorSettings,MotorCmd& cmd, uint32_t kPrintEvery = 20)
        : ServoCANMotor(canId, motorSettings,cmd,kPrintEvery)
    {}

    void begin() override {
        Serial.println("Initializing CAN communication for Servo Mode at 500 kbps");

        if (!CAN.begin(CanBitRate::BR_500k)) {
            Serial.println("CAN.begin(...) failed. Halting code execution");
            for (;;) {}
        }

        Serial.println("CAN ready.");
    }

    bool sendMessage(CAN_PACKET_ID packetId, const uint8_t* data, uint8_t len) override {
        CanMsg const msg(CanExtendedId(make_servo_eid(packetId)), len, data);
        return CAN.write(msg) >= 0;
    }

    bool readMessages(ServoMotorReply& reply) override {
        while (CAN.available()) {
            CanMsg const rxMsg = CAN.read();

            if (!rxMsg.isExtendedId()) {
                continue;
            }

            ServoMotorReply parsed = unpack_servo_reply(
                rxMsg.getExtendedId(),
                rxMsg.data,
                rxMsg.data_length
            );

            if (!parsed.valid_feedback) {
                continue;
            }

            if (parsed.motor_eid != m_canId) {
                continue;
            }

            reply = parsed;
            return true;
        }

        return false;
    }
};

#else
    #error "No CAN platform selected for ServoCANMotor. Define PLATFORM_RENESAS_RA or add another platform implementation."
#endif