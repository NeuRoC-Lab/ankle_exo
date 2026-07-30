//
// Created by Oscar Tesniere on 17/07/2026.
//
#include <Arduino.h>
#include "CANMotorMIT.h"
// define SOFT stop amd HARD stop values here
// compile-time verifications are performed to ensure values are not out of range

constexpr bool constraintsInside(
    const AK60Params& inner,
    const AK60Params& outer)
{
    return
        inner.p_min   >= outer.p_min   &&
        inner.p_max   <= outer.p_max   &&
        inner.v_min   >= outer.v_min   &&
        inner.v_max   <= outer.v_max   &&
        inner.trq_min >= outer.trq_min &&
        inner.trq_max <= outer.trq_max;
}


constexpr AK60Params motorSoftwareConstraints = {
    -4.0f,  2.0f,    // position, rad
    -20.0f, 20.0f,   // velocity, rad/s
    0.0f,  500.0f,   // kp
    0.0f,  5.0f,     // kd
    -10.0f, 10.0f      // torque, N*m
};

constexpr AK60Params motorRunningConstraints = {
    -5.0f, 3.0f,   // position, rad
    -25.0f, 25.0f,   // velocity, rad/s
    0.0f,  500.0f,   // kp, not used for feedback checks
    0.0f,  5.0f,     // kd, not used for feedback checks
    -12.0f, 12.0f      // torque, N*m
};


static_assert(
    constraintsInside(motorSoftwareConstraints, motorParams),
    "Software constraints exceed nominal motor limits"
);

static_assert(
    constraintsInside(motorRunningConstraints, motorParams),
    "Running constraints exceed nominal motor limits"
);

static_assert(
    constraintsInside(
        motorSoftwareConstraints,
        motorRunningConstraints
    ),
    "Software constraints must be inside running constraints"
);

