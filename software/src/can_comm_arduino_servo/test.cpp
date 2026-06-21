//
// Created by Oscar Tesniere on 15/06/2026.
//

#include <Arduino.h>
#include <Arduino_CAN.h>

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

// Servo feedback function IDs
constexpr uint32_t SERVO_JUMP_START_STATE = 0x09;
constexpr uint32_t SERVO_ENTER_MODE_FRAME = 0x2C;
constexpr uint32_t SERVO_REALTIME_FEEDBACK = 0x29;

struct MotorReply {
  uint8_t motor_eid = 0; // lower byte in eid
  uint32_t func_id = 0; // upper byte in eid

  float position_deg = 0.0f;
  float speed_erpm = 0.0f;
  float current_a = 0.0f;
  int8_t temperature_c = 0;
  uint8_t error = 0;

  bool valid_feedback = false;
};

// ---------- byte helpers ----------

static int16_t read_i16_be(const uint8_t *buf) {
  return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

static void write_i16_be(uint8_t *buf, int16_t value) {
  uint16_t u = (uint16_t)value;
  buf[0] = (uint8_t)(u >> 8);
  buf[1] = (uint8_t)(u);
}

static void write_i32_be(uint8_t *buf, int32_t value) {
  uint32_t u = (uint32_t)value;
  buf[0] = (uint8_t)(u >> 24);
  buf[1] = (uint8_t)(u >> 16);
  buf[2] = (uint8_t)(u >> 8);
  buf[3] = (uint8_t)(u);
}

static uint32_t make_servo_eid(uint8_t motor_id, CAN_PACKET_ID packet_id) {
  // Extended CAN ID: [function/control mode in upper bits][motor id in low 8 bits]
  return ((uint32_t)packet_id << 8) | motor_id;
}

// ---------- feedback decode ----------

MotorReply unpack_servo_reply(uint32_t eid, const uint8_t data[8], uint8_t dlc) {
  // inspired from the "motor_receive_servo" methiod implemented in CubeMar's official documentation on Github
  MotorReply reply;

  reply.motor_eid = eid & 0xFF;
  reply.func_id = (eid >> 8) & 0x1FFFFF;

  if (dlc != 8) {
    Serial.println("Invalid Frame Length. Skipping");
    return reply;
  }

  if (reply.func_id == SERVO_REALTIME_FEEDBACK) {
    // assume the motor sends those values as SIGNED INT16 (=INT16 not UINT16) signed numbers. We first reconstructe the 16 bit numbers,
    // then cast them to signed int16
    // the read_i16_be function takes care of concatenating pairs of bytes into int16 by taking the data indexed at lower byte
    int16_t pos_raw = read_i16_be(&data[0]);
    int16_t spd_raw = read_i16_be(&data[2]);
    int16_t cur_raw = read_i16_be(&data[4]);

    reply.position_deg = (float)pos_raw * 0.1f;
    reply.speed_erpm = (float)spd_raw * 10.0f;
    reply.current_a = (float)cur_raw * 0.01f;
    reply.temperature_c = (int8_t)data[6]; // signed byte -> use SIGNED INT8
    reply.error = data[7];
    reply.valid_feedback = true;
  }

  return reply;
}

// ---------- command send helpers ----------

bool send_extended(uint32_t eid, const uint8_t *data, uint8_t len) {
  CanMsg const msg(CanExtendedId(eid), len, data);
  return CAN.write(msg) >= 0;
}

bool can_set_duty(uint8_t motor_id, float duty) {
  // Manual: int32 duty = duty * 100000
  // Typical duty range depends on configured limits, often about 0.005 to 0.95.
  uint8_t buf[4];
  write_i32_be(buf, (int32_t)(duty * 100000.0f));

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_DUTY),
    buf,
    sizeof(buf)
  );
}

bool can_set_current(uint8_t motor_id, float current_a) {
  // Manual: int32 current = current_A * 1000
  uint8_t buf[4];
  write_i32_be(buf, (int32_t)(current_a * 1000.0f));

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_CURRENT),
    buf,
    sizeof(buf)
  );
}

bool can_set_current_brake(uint8_t motor_id, float brake_current_a) {
  // Manual: int32 brake current = current_A * 1000
  uint8_t buf[4];
  write_i32_be(buf, (int32_t)(brake_current_a * 1000.0f));

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_CURRENT_BRAKE),
    buf,
    sizeof(buf)
  );
}

bool can_set_rpm(uint8_t motor_id, float erpm) {
  // Manual: int32 speed = ERPM directly
  uint8_t buf[4];
  write_i32_be(buf, (int32_t)erpm);

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_RPM),
    buf,
    sizeof(buf)
  );
}

bool can_set_position(uint8_t motor_id, float position_deg) {
  // Manual CAN Servo position mode:
  // int32 position = degrees * 10000
  uint8_t buf[4];
  write_i32_be(buf, (int32_t)(position_deg * 10000.0f));

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_POS),
    buf,
    sizeof(buf)
  );
}

bool can_set_origin(uint8_t motor_id, uint8_t origin_mode) {
  // 0 = temporary origin, lost after power cycle
  // 1 = permanent zero point, dual-encoder models only
  uint8_t buf[1] = { origin_mode };

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_ORIGIN_HERE),
    buf,
    sizeof(buf)
  );
}

bool can_set_position_speed(uint8_t motor_id,
                            float position_deg,
                            float speed_erpm,
                            float accel_erpm_s2) {
  // Manual:
  // position: int32 = deg * 10000
  // speed:    int16 = ERPM / 10
  // accel:    int16 = ERPM/s^2 / 10
  uint8_t buf[8];

  write_i32_be(&buf[0], (int32_t)(position_deg * 10000.0f));
  write_i16_be(&buf[4], (int16_t)(speed_erpm / 10.0f));
  write_i16_be(&buf[6], (int16_t)(accel_erpm_s2 / 10.0f));

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_POS_SPD),
    buf,
    sizeof(buf)
  );
}

// ---------- printing ----------

void print_error_code(uint8_t error) {
  switch (error) {
    case 0: Serial.print("none"); break;
    case 1: Serial.print("motor over-temp"); break;
    case 2: Serial.print("over-current"); break;
    case 3: Serial.print("over-voltage"); break;
    case 4: Serial.print("under-voltage"); break;
    case 5: Serial.print("encoder fault"); break;
    case 6: Serial.print("MOSFET over-temp"); break;
    case 7: Serial.print("motor stall"); break;
    default: Serial.print("unknown"); break;
  }
}

void print_reply(const MotorReply &reply) {
  Serial.print("motor_id=");
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

constexpr uint8_t MOTOR_ID = 0x02;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    if (!CAN.begin(CanBitRate::BR_1000k)) {
        Serial.println("CAN.begin failed");
        while (true) {}
    }

    delay(1000);

    Serial.println("Sending RPM command");
    bool ok = can_set_position(MOTOR_ID, 360.0f);
    Serial.print("CAN write ok = ");
    Serial.println(ok);
}

void loop() {

    while (CAN.available()) {
        CanMsg const rxMsg = CAN.read();

        if (!rxMsg.isExtendedId()) {
            continue;
        }

        MotorReply reply = unpack_servo_reply(
          rxMsg.getExtendedId(),
          rxMsg.data,
          rxMsg.data_length
        );

        print_reply(reply);
      can_set_position(MOTOR_ID, 180.0f);
    }
}