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
    EncoderSnapshot,
    LoadCellSnapshot,

    LeftMotorSnapshot,
    RightMotorSnapshot,

    LeftMotorCommand,
    RightMotorCommand,

    SafetyCommand,
    SafetyState,
    SystemCommand,

    Ina232Snapshot,

    Count
};

inline constexpr size_t EndpointCount =
    static_cast<size_t>(EndpointId::Count);

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
    std::array<uint8_t, MaxPayloadSize> payload{};
};
