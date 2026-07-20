//
// Created by Oscar Tesniere on 20/07/2026.
//

#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

namespace TelemetryKey {

    inline constexpr char LeftLoadCell1[]  = "LLC1";
    inline constexpr char LeftLoadCell2[]  = "LLC2";
    inline constexpr char RightLoadCell1[] = "RLC1";
    inline constexpr char RightLoadCell2[] = "RLC2";

    inline constexpr char Motors[]    = "MOTORS";
    inline constexpr char MotorId[]   = "MTR_ID_DEC";
    inline constexpr char MotorPos[]  = "MTR_POS_RAD";
    inline constexpr char MotorVel[]  = "MTR_VEL_RADS";
    inline constexpr char MotorTrq[]  = "MTR_TRQ_NM";
    inline constexpr char MotorErr[]  = "MTR_ERR_DEC";
    inline constexpr char MotorTemp[] = "MTR_TEMP_DEG";

    inline constexpr char LeftEncoder[]  = "LENC";
    inline constexpr char RightEncoder[] = "RENC";

}

// could have used #DEFINE directives here as well ....
// maybe do a Telemetry data class as well ?


