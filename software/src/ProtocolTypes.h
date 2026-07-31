//
// Created by Oscar Tesniere on 27/07/2026.
//

#pragma once

enum class MessageType : uint8_t
{
    Telemetry = 1,
    Command   = 2
};

struct MessageHeader
{
    MessageType type;
    uint8_t payloadSize;
};

typedef struct
{
    float position;
    float velocity;
    float torque;
    float kp;
    float kd;
} MotorCmd;

typedef struct
{
    uint8_t can_id;
    float position;
    float velocity;
    float torque;
    uint8_t temperature;
    uint8_t error;
} MotorReply;

struct EncoderPositions
{
    uint16_t left_position;
    uint16_t right_position;
};


struct LoadCellVoltages {
    float LeftLoadCell1 {};
    float LeftLoadCell2 {};
    float RightLoadCell1 {};
    float RightLoadCell2 {};
};

struct DataPayload {
    // load cells
    // encoders
    //MessageType cmd_type = MessageType::Telemetry; // telemetry header
    LoadCellVoltages loadCells {};
    EncoderPositions encoders {};
    MotorReply motorRep {}; // to account for the two motors
};

// =========== TELEMETRY DEFINITIONS ==================



// =================== COMMAND DEFINITIONS =======================

// make a "command payload" : it should have space for a MotorCmd object (reinterpreted into an array of bytes) as well as auxilliary commands like start,stop, and the ID to go with it.
enum class MotorCommandType : uint8_t {
    Start = 0,
    Stop, // stopping the motor
    Zero, // set current position to 0
    Set // command setting
};

// ONLY SUPPORTS ONE MOTOR PER PAYLOAD (command sent)
struct CommandPayload {
    //MessageType cmd_type = MessageType::Command; // command header
    MotorCommandType type; // set, zero, start, stop
    uint8_t motorId;
    MotorCmd cmd; // the main payload
};