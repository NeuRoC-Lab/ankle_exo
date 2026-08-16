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

struct TransparentControllerParameters
{
	bool enabled;			// to enable / disable it

    float kp;               // Proportional gain
    float kd;               // Derivative gain

    float a_derivative;     // Derivative filter alpha
    float a_friction;       // Friction compensation filter alpha
    float a_torque;         // Measured torque filter alpha

    float comp_torque;      // Cable friction compensation torque

    float trigger_on_trq;   // Friction hysteresis ON threshold
    float trigger_off_trq;  // Friction hysteresis OFF threshold

    float max_abs_out_trq;  // Maximum absolute output torque
};

constexpr TransparentControllerParameters
    DEFAULT_TRANSPARENT_CONTROLLER_PARAMETERS
{
	.enabled 		= false,
    .kp              = 0.5f,
    .kd              = 0.01f,

    .a_derivative    = 0.05f,
    .a_friction      = 0.10f,
    .a_torque        = 0.15f,

    .comp_torque     = 0.08f,

    .trigger_on_trq  = 0.025f,
    .trigger_off_trq = 0.010f,

    .max_abs_out_trq = 0.4f
};