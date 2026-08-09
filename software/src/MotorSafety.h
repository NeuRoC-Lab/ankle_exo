#pragma once

#include <cstdint>
#include <Arduino.h>
#include "ProtocolTypes.h"
#include "MotorConfig.h"



float rawCountToDegrees(
    uint16_t rawCount,
    uint16_t zeroCount=0,
    uint16_t maxCount = 4096
)
{
    const int32_t halfCount =
        static_cast<int32_t>(maxCount) / 2;

    int32_t relativeCount =
        static_cast<int32_t>(rawCount) -
        static_cast<int32_t>(zeroCount);

    // Wrap into the range [0, maxCount).
    relativeCount %= static_cast<int32_t>(maxCount);

    // C++ modulo can be negative.
    if (relativeCount < 0) {
        relativeCount += maxCount;
    }

    // Convert to the signed range
    // [-halfCount, halfCount).
    if (relativeCount >= halfCount) {
        relativeCount -= maxCount;
    }

    return static_cast<float>(relativeCount) *
           360.0f /
           static_cast<float>(maxCount);
}

struct MotorLimits
{
    float p_min;
    float p_max;

    float v_min;
    float v_max;

    float trq_min;
    float trq_max;
};


enum class SafetyAction : uint8_t
{
    None,
    SendCommand,
    DisableMotor
};

struct SafetyResult
{
    MotorState state;
    SafetyAction action;
    MotorCmd command;
};


class MotorSafetyController
{
public:
    MotorSafetyController(
        const AK60Params& softwareLimits,
        const AK60Params& runningLimits,
        const DataPayload& payload
    )
        : m_softwareLimits(softwareLimits),
          m_runningLimits(runningLimits),
          m_payload(payload)
    {
    }

    SafetyResult evaluate(
        const MotorReply& feedback,
        const MotorCmd& requestedCommand
    )
    {
        //if (isOutsideRunningLimits(feedback)) {
        if(rawCountToDegrees(m_payload.encoders.left_position) > -90 && rawCountToDegrees(m_payload.encoders.left_position) < 90 && (rawCountToDegrees(m_payload.encoders.left_position) > EncoderAbsConstraints.p_max || rawCountToDegrees(m_payload.encoders.left_position) < EncoderAbsConstraints.p_min)) {
            Serial.println(rawCountToDegrees(m_payload.encoders.left_position));
                return {
                MotorState::HardStopped,
                SafetyAction::DisableMotor,
                {}
            };
        }

        if(rawCountToDegrees(m_payload.encoders.left_position) > -90 && rawCountToDegrees(m_payload.encoders.left_position) < 90 && (rawCountToDegrees(m_payload.encoders.left_position) > EncoderSoftConstraints.p_max || rawCountToDegrees(m_payload.encoders.left_position) < EncoderSoftConstraints.p_min)) {
            return {
                MotorState::Recovery,
                SafetyAction::SendCommand,
                makeRecoveryCommand(feedback)
            };
        }

        MotorCmd safeCommand = requestedCommand;
        safeCommand.torque =
            limitTorqueCommand(feedback, requestedCommand.torque);

        return {
            MotorState::Running,
            SafetyAction::SendCommand,
            safeCommand
        };
    }

private:
    const DataPayload& m_payload;
    bool isOutsideRunningLimits(
        const MotorReply& feedback
    ) const
    {
        return
            feedback.position < m_runningLimits.p_min ||
            feedback.position > m_runningLimits.p_max ||
            feedback.velocity < m_runningLimits.v_min ||
            feedback.velocity > m_runningLimits.v_max ||
            feedback.torque < m_runningLimits.trq_min ||
            feedback.torque > m_runningLimits.trq_max;
    }

    bool isOutsideSoftwareLimits(
        const MotorReply& feedback
    ) const
    {
        return
            feedback.position < m_softwareLimits.p_min ||
            feedback.position > m_softwareLimits.p_max;
    }

    MotorCmd makeRecoveryCommand(
        const MotorReply& feedback
    ) const
    {
        constexpr float recoveryMargin = 0.5f;

        MotorCmd command {};

        if (feedback.position > m_softwareLimits.p_max) {
            command.position =
                m_softwareLimits.p_max - recoveryMargin;
        }
        else if (feedback.position < m_softwareLimits.p_min) {
            command.position =
                m_softwareLimits.p_min + recoveryMargin;
        }

        command.velocity = 0.0f;
        command.kp = 4.0f;
        command.kd = 1.2f;
        command.torque = 0.0f;

        return command;
    }

    float limitTorqueCommand(
        const MotorReply& feedback,
        float requestedTorque
    ) const
    {
        constexpr float brakingDistance = 0.25f;

        if (requestedTorque > 0.0f) {
            const float distance =
                m_softwareLimits.p_max -
                feedback.position;

            const float scale = std::clamp(
                distance / brakingDistance,
                0.0f,
                1.0f
            );

            return requestedTorque * scale;
        }

        if (requestedTorque < 0.0f) {
            const float distance =
                feedback.position -
                m_softwareLimits.p_min;

            const float scale = std::clamp(
                distance / brakingDistance,
                0.0f,
                1.0f
            );

            return requestedTorque * scale;
        }

        return 0.0f;
    }

    const AK60Params& m_softwareLimits;
    const AK60Params& m_runningLimits;
};