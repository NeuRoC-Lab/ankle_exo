```cpp
#pragma once

#include <Arduino.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "ProtocolTypes.h"
#include "MotorConfig.h"

// ============================================================
// Encoder conversion
// ============================================================

inline float rawCountToDegrees(
    uint16_t rawCount,
    uint16_t zeroCount = 0,
    uint16_t maxCount = 4096
)
{
    const int32_t signedMaxCount =
        static_cast<int32_t>(maxCount);

    const int32_t halfCount =
        signedMaxCount / 2;

    int32_t relativeCount =
        static_cast<int32_t>(rawCount) -
        static_cast<int32_t>(zeroCount);

    relativeCount %= signedMaxCount;

    // C++ modulo may produce a negative result.
    if (relativeCount < 0) {
        relativeCount += signedMaxCount;
    }

    // Convert to the signed interval:
    // [-halfCount, halfCount)
    if (relativeCount >= halfCount) {
        relativeCount -= signedMaxCount;
    }

    return static_cast<float>(relativeCount) *
           360.0f /
           static_cast<float>(maxCount);
}

inline float degreesToRadians(float angleDegrees)
{
    constexpr float DEG_TO_RAD =
        0.01745329251994329577f;

    return angleDegrees * DEG_TO_RAD;
}

// ============================================================
// Safety result types
// ============================================================

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

// ============================================================
// Inertia estimator and bounce-controller configuration
// ============================================================

struct InertiaSafetyConfig
{
    // Local encoder configuration
    uint16_t encoderZeroCount = 0;
    uint16_t encoderMaxCount = 4096;

    // Change to -1 if encoder direction is reversed.
    float encoderDirection = 1.0f;

    /*
     * Load cells used to calculate differential tension.
     *
     * tau_net =
     *     (forceA - forceB) *
     *     momentArm *
     *     loadCellTorqueDirection
     */
    size_t loadCellAIndex = 0;
    size_t loadCellBIndex = 1;

    // Effective cable or pulley moment arm, in metres.
    float momentArm = 0.055f;

    // Change to -1 if the load-cell torque sign is reversed.
    float loadCellTorqueDirection = 1.0f;

    /*
     * Filtering factors.
     *
     * 0.0 = no response to new measurements
     * 1.0 = no filtering
     */
    float velocityFilterAlpha = 0.20f;
    float accelerationFilterAlpha = 0.10f;
    float inertiaFilterAlpha = 0.05f;

    // Ignore derivative updates made too quickly.
    uint32_t minimumKinematicPeriodUs = 1000;

    // Ignore inertia observations below these values.
    float minimumAccelerationRadPerSec2 = 1.0f;
    float minimumNetTorqueNm = 0.20f;

    // Plausible effective-inertia range, in kg*m^2.
    float minimumEffectiveInertia = 0.01f;
    float maximumEffectiveInertia = 5.0f;

    // Used until a reliable inertia estimate exists.
    float initialEffectiveInertia = 0.25f;

    /*
     * Start applying inertia braking this many degrees before
     * reaching EncoderSoftConstraints.
     */
    float bounceRegionDegrees = 10.0f;

    /*
     * Prevent torque from becoming arbitrarily large when the
     * remaining stopping distance approaches zero.
     */
    float minimumStoppingDistanceDegrees = 1.0f;

    // Maximum safety-generated torque.
    float maximumBounceTorqueNm = 8.0f;

    // Multiplier for the inertia-based braking term.
    float bounceGain = 1.0f;

    // Velocity damping, in N*m per rad/s.
    float dampingGain = 0.5f;

    /*
     * Restoring torque applied after crossing the soft boundary,
     * in N*m per degree.
     */
    float recoverySpringGain = 0.20f;
};

// ============================================================
// Motor safety controller
// ============================================================

template <typename LoadCellControllerT>
class MotorSafetyController
{
public:
    MotorSafetyController(
        const AK60Params& softwareLimits,
        const AK60Params& runningLimits,
        const DataPayload& payload,
        LoadCellControllerT& loadCellController,
        const InertiaSafetyConfig& config = {}
    )
        /*
         * These AK60Params references are intentionally retained
         * for compatibility with the existing CANMotorMIT
         * constructor.
         *
         * Position safety below uses EncoderSoftConstraints and
         * EncoderAbsConstraints instead.
         */
        : m_softwareLimits(softwareLimits),
          m_runningLimits(runningLimits),
          m_payload(payload),
          m_loadCellController(loadCellController),
          m_config(config),
          m_effectiveInertia(
              std::clamp(
                  config.initialEffectiveInertia,
                  config.minimumEffectiveInertia,
                  config.maximumEffectiveInertia
              )
          )
    {
    }

    SafetyResult evaluate(
        const MotorReply& feedback,
        const MotorCmd& requestedCommand
    )
    {
        // Kept in the signature for compatibility and possible
        // future motor-feedback checks.
        (void)feedback;

        const uint16_t rawPosition =
            m_payload.encoders.left_position;

        if (!isValidEncoderCount(rawPosition)) {
            return makeHardStopResult();
        }

        const float angleDegrees =
            m_config.encoderDirection *
            rawCountToDegrees(
                rawPosition,
                m_config.encoderZeroCount,
                m_config.encoderMaxCount
            );

        updateKinematics(
            degreesToRadians(angleDegrees),
            micros()
        );

        const float netTorque =
            sampleNetTorque();

        updateEffectiveInertia(netTorque);

        // EncoderAbsConstraints define the hard-stop boundary.
        if (isOutsideAbsoluteLimits(angleDegrees)) {
            Serial.print(
                "Absolute encoder limit exceeded: "
            );
            Serial.println(angleDegrees);

            return makeHardStopResult();
        }

        const BoundaryStatus boundary =
            evaluateSoftBoundary(angleDegrees);

        /*
         * The encoder has crossed a soft limit but remains inside
         * the absolute limits.
         */
        if (boundary.outsideSoftLimit) {
            return {
                MotorState::Recovery,
                SafetyAction::SendCommand,
                makeRecoveryCommand(boundary)
            };
        }

        /*
         * The joint is still inside the allowed region, but is
         * moving outward and is close enough to a soft boundary
         * for inertia braking to be useful.
         */
        if (
            boundary.movingOutward &&
            boundary.distanceToSoftLimitDegrees <=
                m_config.bounceRegionDegrees
        ) {
            return {
                MotorState::Running,
                SafetyAction::SendCommand,
                makeBounceCommand(
                    requestedCommand,
                    boundary
                )
            };
        }

        return {
            MotorState::Running,
            SafetyAction::SendCommand,
            requestedCommand
        };
    }

    float effectiveInertia() const
    {
        return m_effectiveInertia;
    }

    float estimatedVelocityRadPerSec() const
    {
        return m_filteredVelocity;
    }

    float estimatedAccelerationRadPerSec2() const
    {
        return m_filteredAcceleration;
    }

    float measuredNetTorqueNm() const
    {
        return m_lastNetTorque;
    }

private:
    struct BoundaryStatus
    {
        bool outsideSoftLimit = false;
        bool movingOutward = false;

        /*
         * +1 for the upper boundary.
         * -1 for the lower boundary.
         */
        float boundaryDirection = 0.0f;

        // Positive while inside the valid range.
        float distanceToSoftLimitDegrees = 0.0f;

        // Positive after crossing a soft limit.
        float penetrationDegrees = 0.0f;
    };

    /*
     * Retained only for constructor compatibility.
     * Position safety does not use these objects.
     */
    const AK60Params& m_softwareLimits;
    const AK60Params& m_runningLimits;

    const DataPayload& m_payload;
    LoadCellControllerT& m_loadCellController;

    InertiaSafetyConfig m_config;

    bool m_hasPreviousKinematicSample = false;

    uint32_t m_previousKinematicTimeUs = 0;

    float m_previousAngleRad = 0.0f;
    float m_previousRawVelocity = 0.0f;

    float m_filteredVelocity = 0.0f;
    float m_filteredAcceleration = 0.0f;

    float m_effectiveInertia = 0.0f;
    float m_lastNetTorque = 0.0f;

    bool isValidEncoderCount(
        uint16_t rawPosition
    ) const
    {
        return rawPosition != UINT16_MAX;
    }

    bool isOutsideAbsoluteLimits(
        float angleDegrees
    ) const
    {
        return
            angleDegrees <
                EncoderAbsConstraints.p_min ||
            angleDegrees >
                EncoderAbsConstraints.p_max;
    }

    BoundaryStatus evaluateSoftBoundary(
        float angleDegrees
    ) const
    {
        BoundaryStatus result {};

        if (
            angleDegrees >
            EncoderSoftConstraints.p_max
        ) {
            result.outsideSoftLimit = true;
            result.boundaryDirection = 1.0f;

            result.penetrationDegrees =
                angleDegrees -
                EncoderSoftConstraints.p_max;

            result.movingOutward =
                m_filteredVelocity > 0.0f;

            return result;
        }

        if (
            angleDegrees <
            EncoderSoftConstraints.p_min
        ) {
            result.outsideSoftLimit = true;
            result.boundaryDirection = -1.0f;

            result.penetrationDegrees =
                EncoderSoftConstraints.p_min -
                angleDegrees;

            result.movingOutward =
                m_filteredVelocity < 0.0f;

            return result;
        }

        /*
         * Inside the soft limits: identify the boundary in the
         * current direction of movement.
         */
        if (m_filteredVelocity > 0.0f) {
            result.boundaryDirection = 1.0f;

            result.distanceToSoftLimitDegrees =
                EncoderSoftConstraints.p_max -
                angleDegrees;

            result.movingOutward = true;
        }
        else if (m_filteredVelocity < 0.0f) {
            result.boundaryDirection = -1.0f;

            result.distanceToSoftLimitDegrees =
                angleDegrees -
                EncoderSoftConstraints.p_min;

            result.movingOutward = true;
        }

        return result;
    }

    void updateKinematics(
        float angleRad,
        uint32_t currentTimeUs
    )
    {
        if (!m_hasPreviousKinematicSample) {
            m_previousAngleRad = angleRad;
            m_previousKinematicTimeUs =
                currentTimeUs;

            m_hasPreviousKinematicSample = true;
            return;
        }

        const uint32_t elapsedUs =
            currentTimeUs -
            m_previousKinematicTimeUs;

        if (
            elapsedUs <
            m_config.minimumKinematicPeriodUs
        ) {
            return;
        }

        const float dt =
            static_cast<float>(elapsedUs) *
            1.0e-6f;

        if (dt <= 0.0f) {
            return;
        }

        const float angleDifference =
            wrappedAngleDifference(
                angleRad,
                m_previousAngleRad
            );

        const float rawVelocity =
            angleDifference / dt;

        const float rawAcceleration =
            (
                rawVelocity -
                m_previousRawVelocity
            ) / dt;

        m_filteredVelocity =
            lowPassFilter(
                m_filteredVelocity,
                rawVelocity,
                m_config.velocityFilterAlpha
            );

        m_filteredAcceleration =
            lowPassFilter(
                m_filteredAcceleration,
                rawAcceleration,
                m_config.accelerationFilterAlpha
            );

        m_previousAngleRad = angleRad;
        m_previousRawVelocity = rawVelocity;
        m_previousKinematicTimeUs =
            currentTimeUs;
    }

    float sampleNetTorque()
    {
        const float* forces =
            m_loadCellController.sampleAll();

        if (forces == nullptr) {
            m_lastNetTorque = 0.0f;
            return 0.0f;
        }

        const float forceA =
            forces[m_config.loadCellAIndex];

        const float forceB =
            forces[m_config.loadCellBIndex];

        if (
            !std::isfinite(forceA) ||
            !std::isfinite(forceB)
        ) {
            m_lastNetTorque = 0.0f;
            return 0.0f;
        }

        const float differentialForce =
            forceA - forceB;

        m_lastNetTorque =
            m_config.loadCellTorqueDirection *
            differentialForce *
            m_config.momentArm;

        return m_lastNetTorque;
    }

    void updateEffectiveInertia(
        float netTorque
    )
    {
        if (
            std::fabs(m_filteredAcceleration) <
                m_config.minimumAccelerationRadPerSec2
        ) {
            return;
        }

        if (
            std::fabs(netTorque) <
                m_config.minimumNetTorqueNm
        ) {
            return;
        }

        const float observedInertia =
            netTorque /
            m_filteredAcceleration;

        /*
         * A scalar inertia estimate should be positive.
         *
         * A negative result usually means that the encoder or
         * load-cell sign is reversed, or that external torque is
         * dominating the measurement.
         */
        if (
            !std::isfinite(observedInertia) ||
            observedInertia <= 0.0f
        ) {
            return;
        }

        const float boundedObservation =
            std::clamp(
                observedInertia,
                m_config.minimumEffectiveInertia,
                m_config.maximumEffectiveInertia
            );

        m_effectiveInertia =
            lowPassFilter(
                m_effectiveInertia,
                boundedObservation,
                m_config.inertiaFilterAlpha
            );
    }

    MotorCmd makeBounceCommand(
        const MotorCmd& requestedCommand,
        const BoundaryStatus& boundary
    ) const
    {
        MotorCmd command = requestedCommand;

        // Safety control is applied in torque mode.
        command.kp = 0.0f;
        command.kd = 0.0f;

        const float stoppingDistanceDegrees =
            std::max(
                boundary.distanceToSoftLimitDegrees,
                m_config.minimumStoppingDistanceDegrees
            );

        const float stoppingDistanceRad =
            degreesToRadians(
                stoppingDistanceDegrees
            );

        /*
         * From:
         *
         *     omega^2 = 2 * alpha * distance
         *
         * and:
         *
         *     torque = J * alpha
         *
         * the required counter-torque is:
         *
         *     -J * omega * |omega| / (2 * distance)
         */
        const float inertiaCounterTorque =
            -m_config.bounceGain *
            m_effectiveInertia *
            m_filteredVelocity *
            std::fabs(m_filteredVelocity) /
            (2.0f * stoppingDistanceRad);

        const float dampingTorque =
            -m_config.dampingGain *
            m_filteredVelocity;

        float userTorque =
            requestedCommand.torque;

        /*
         * Remove user torque that pushes farther toward the
         * boundary.
         */
        if (
            boundary.boundaryDirection > 0.0f &&
            userTorque > 0.0f
        ) {
            userTorque = 0.0f;
        }
        else if (
            boundary.boundaryDirection < 0.0f &&
            userTorque < 0.0f
        ) {
            userTorque = 0.0f;
        }

        const float combinedTorque =
            userTorque +
            inertiaCounterTorque +
            dampingTorque;

        command.torque =
            std::clamp(
                combinedTorque,
                -m_config.maximumBounceTorqueNm,
                m_config.maximumBounceTorqueNm
            );

        return command;
    }

    MotorCmd makeRecoveryCommand(
        const BoundaryStatus& boundary
    ) const
    {
        MotorCmd command {};

        command.position = 0.0f;
        command.velocity = 0.0f;
        command.kp = 0.0f;
        command.kd = 0.0f;

        /*
         * Upper-limit penetration requires negative torque.
         * Lower-limit penetration requires positive torque.
         */
        const float restoringTorque =
            -boundary.boundaryDirection *
            m_config.recoverySpringGain *
            boundary.penetrationDegrees;

        const float dampingTorque =
            -m_config.dampingGain *
            m_filteredVelocity;

        float inertiaCounterTorque = 0.0f;

        /*
         * Continue opposing inertia while the joint is still
         * travelling farther outside the soft boundary.
         */
        if (boundary.movingOutward) {
            const float minimumDistanceRad =
                degreesToRadians(
                    m_config.minimumStoppingDistanceDegrees
                );

            inertiaCounterTorque =
                -m_config.bounceGain *
                m_effectiveInertia *
                m_filteredVelocity *
                std::fabs(m_filteredVelocity) /
                (2.0f * minimumDistanceRad);
        }

        const float recoveryTorque =
            restoringTorque +
            dampingTorque +
            inertiaCounterTorque;

        command.torque =
            std::clamp(
                recoveryTorque,
                -m_config.maximumBounceTorqueNm,
                m_config.maximumBounceTorqueNm
            );

        return command;
    }

    SafetyResult makeHardStopResult() const
    {
        return {
            MotorState::HardStopped,
            SafetyAction::DisableMotor,
            {}
        };
    }

    static float lowPassFilter(
        float previousValue,
        float newValue,
        float alpha
    )
    {
        const float boundedAlpha =
            std::clamp(
                alpha,
                0.0f,
                1.0f
            );

        return previousValue +
               boundedAlpha *
               (newValue - previousValue);
    }

    static float wrappedAngleDifference(
        float currentAngle,
        float previousAngle
    )
    {
        constexpr float PI =
            3.14159265358979323846f;

        constexpr float TWO_PI =
            6.28318530717958647692f;

        float difference =
            currentAngle - previousAngle;

        while (difference > PI) {
            difference -= TWO_PI;
        }

        while (difference < -PI) {
            difference += TWO_PI;
        }

        return difference;
    }
};
```
