//
// Created by Oscar Tesniere on 20/07/2026.
//

#pragma once

#include <Arduino.h>

namespace TelemetryKey {

     constexpr char LeftLoadCell1[]  = "LLC1";
     constexpr char LeftLoadCell2[]  = "LLC2";
     constexpr char RightLoadCell1[] = "RLC1";
     constexpr char RightLoadCell2[] = "RLC2";

     constexpr char Motors[]    = "MOTORS";
     constexpr char MotorId[]   = "MTR_ID_DEC";
     constexpr char MotorPos[]  = "MTR_POS_RAD";
     constexpr char MotorVel[]  = "MTR_VEL_RADS";
     constexpr char MotorTrq[]  = "MTR_TRQ_NM";
     constexpr char MotorErr[]  = "MTR_ERR_DEC";
     constexpr char MotorTemp[] = "MTR_TEMP_DEG";

     constexpr char LeftEncoder[]  = "LENC";
     constexpr char RightEncoder[] = "RENC";

}

// could have used #DEFINE directives here as well ....
// maybe do a Telemetry data class as well ?


