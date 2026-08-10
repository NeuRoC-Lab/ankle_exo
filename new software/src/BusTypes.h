#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// to distinguish between Nano and
enum class PlatformId : uint8_t
{
    Teensy,
    Nano
};

enum class EndpointId : uint8_t
{
    EncoderSnapshot, // encoder positions (left/right) snapshot
    LoadCellSnapshot, // load cells (L1,L2,R1,R2) snapshots

    LeftMotorSnapshot, // left motor snapshot (its feedback)
    RightMotorSnapshot, // right motor snapshot (its feedback)

    LeftMotorCommand, // left motor command buffer (MotorCmd)
    LeftMotorMetaCommand, // to start,stop, zero the motor (MotorMetaCommand)
    //RightMotorCommand, // right motor command buffer (MotorCmd)
    RightMotorMetaCommand, // to start,stop, zero the motor (MotorMetaCommand)

    SafetyCommand,
    SafetyState,
    SystemCommand,

    Ina232Snapshot, // PCB voltage and current readings from INA232

    Count // DONT TOUCH
};

inline constexpr size_t EndpointCount =
    static_cast<size_t>(EndpointId::Count);
// returns the size of Endpoints

inline constexpr size_t MaxPayloadSize = 64;

struct MessageHeader
{
    EndpointId topic{};
    PlatformId origin{};
    uint16_t sequence{0};
};

struct BusMessage
{
    MessageHeader header{};
    uint16_t payloadSize{0};
    std::array<uint8_t, MaxPayloadSize> payload{}; // bounded array for safety
};
