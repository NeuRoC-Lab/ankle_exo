#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// ENCODER TYPES

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

//LOAD CELL TYPES

// load cell torques are more useful than raw forces
using LoadCellTorques = std::array<float, 2>;

//MOTOR TYPES

//note : a new struct which only transmits useful informations to the user. If you need position and/or velocity add it here
struct MotorFeedback
{
   float torque{0.0f};
   uint8_t temperature{0};
   uint8_t error{0};
};

// INA232

struct PowerReadings
{
    float batteryVoltage;
    //float pcbCurrent; //temporary disabled these to save BLE bandwidth, also we don't need that for higher level controller
    //float pcbPower;
};

// Motor controllers

struct TransparentControllerParameters
{
    bool enabled; // enabled state => whenn set to false, transparent controller simply doesn't run

    float input_hp_cutoff_hz;
    float input_lp_cutoff_hz;

    float derivative_lp_cutoff_hz;
    float friction_lp_cutoff_hz;

    float kp;
    float kd;

    float comp_torque;

    float trigger_on_trq;
    float trigger_off_trq;

    float max_abs_out_trq;
};

// to distinguish between Nano and
enum class PlatformId : uint8_t
{
    Teensy,
    Nano
};

enum class LoggingState : uint8_t
{
    Stopped = 0,
    Recording = 1
};

enum class EndpointId : uint8_t
{
    EncoderSnapshot, // encoder positions (left/right) snapshot
    LoadCellSnapshot, // load cells (L1,L2,R1,R2) snapshots

    LeftMotorSnapshot, // left motor snapshot (its feedback)
    RightMotorSnapshot, // right motor snapshot (its feedback)

    LeftMotorCommand, // left motor torque command buffer (float)
    LeftMotorEnabled, // to enable / disable motor
    RightMotorCommand, // right motor torque command buffer (float)
    RightMotorEnabled, // to enable / disable motor

   	LeftMotorTransparentParams,
   	RightMotorTransparentParams,

    LeftMotorTransparentTorque,
    RightMotorTransparentTorque,

    LeftMotorTransparentCommand,
    RightMotorTransparentCommand,

    Ina232Snapshot, // PCB voltage and current readings from INA232
	LoggingState,

    Count // DONT TOUCH
};

inline constexpr size_t EndpointCount =
    static_cast<size_t>(EndpointId::Count);
// returns the size of Endpoints

inline constexpr size_t MaxPayloadSize = 128;

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


using PlatformMask = uint8_t; // a mask for indicating if a topic belongs to teensy or nano

constexpr PlatformMask platformBit(
    PlatformId platform)
{
    return static_cast<PlatformMask>(1U << static_cast<uint8_t>(platform));
}

struct TopicRoute
{
    PlatformMask subscribers{0};
};

#define ENDPOINT_LIST(X)                                                \
    X(EncoderSnapshot,              EncoderPositions,                Both)   \
    X(LoadCellSnapshot,             LoadCellTorques,                  Both)   \
    X(LeftMotorSnapshot,            MotorFeedback,                   Both)   \
    X(RightMotorSnapshot,           MotorFeedback,                   Both)   \
    X(LeftMotorCommand,             float,                           Both) \
    X(RightMotorCommand,            float,                           Both) \
    X(Ina232Snapshot,               PowerReadings,                   Both)   \
    X(LeftMotorEnabled,             bool,                            Teensy) \
    X(RightMotorEnabled,            bool,                            Teensy) \
    X(LoggingState,                 LoggingState,                    Both)   \
    X(LeftMotorTransparentParams,   TransparentControllerParameters, Both)   \
    X(RightMotorTransparentParams,  TransparentControllerParameters, Both)   \
    X(LeftMotorTransparentCommand,   float,                           Nano)   \
    X(RightMotorTransparentCommand,  float,                           Nano) \
    X(LeftMotorTransparentTorque,    float,                           Nano) \
    X(RightMotorTransparentTorque,   float,                           Nano)

template<EndpointId Id>
struct EndpointTraits;

#define DEFINE_ENDPOINT_TRAIT(name, payload, subscribers) \
    template<>                                            \
    struct EndpointTraits<EndpointId::name>               \
    {                                                     \
        using Payload = payload;                          \
    };

ENDPOINT_LIST(DEFINE_ENDPOINT_TRAIT)

#undef DEFINE_ENDPOINT_TRAIT

inline constexpr PlatformMask Teensy =
    platformBit(PlatformId::Teensy);

inline constexpr PlatformMask Nano =
    platformBit(PlatformId::Nano);

inline constexpr PlatformMask Both =
    platformBit(PlatformId::Teensy) |
    platformBit(PlatformId::Nano);

inline constexpr std::array<TopicRoute, EndpointCount> TopicRoutes = []()
{
    std::array<TopicRoute, EndpointCount> routes{};

#define DEFINE_ROUTE(name, payload, subscriber_mask)                \
    routes[static_cast<size_t>(EndpointId::name)].subscribers =     \
        subscriber_mask;

    ENDPOINT_LIST(DEFINE_ROUTE)

#undef DEFINE_ROUTE

    return routes;
}();