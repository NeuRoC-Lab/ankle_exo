"""
Sensor and motor data models for the single-leg exoskeleton.

Includes:
- Encoder
- 4 load cells
- 1 CubeMars motor in MIT mode
- Motor commands
- Motor meta-commands
- SD logging control
"""

Motor_ID = 2 #TODO CHANGE LATER BY A SIDE TO ID MAPPING (easier)

import struct
import threading

from dataclasses import dataclass
from enum import IntEnum


# =========================================================
# GENERAL
# =========================================================

def clamp(
        value,
        minimum=-1000.0,
        maximum=1000.0,
):
    return max(
        minimum,
        min(value, maximum),
    )


# =========================================================
# ENCODER
# =========================================================

class Encoder:

    def __init__(self):

        self.raw_count = 0

        self.first_value = None

        self.angle_deg = 0.0

        self.raw_vel = 0.0
        self.filtered_vel = 0.0
        self.ankle_velocity = 0.0

        # EWMA filter coefficient
        self.alpha = 0.2

        self.first_velocity = True

        self.prev_angle = 0.0
        self.prev_time = None

        self.lock = threading.Lock()


    def update(
            self,
            raw_count,
            current_time,
    ):

        with self.lock:

            self.raw_count = raw_count

            # Save the first encoder reading
            # as the initial reference.
            if self.first_value is None:
                self.first_value = raw_count

            # Currently using the received
            # encoder value directly.
            new_angle = raw_count

            # ---------------------------------------------
            # Velocity calculation
            # ---------------------------------------------

            if self.prev_time is not None:

                dt = (
                        current_time
                        -
                        self.prev_time
                )

                if dt > 0:

                    self.raw_vel = (
                                           new_angle
                                           -
                                           self.prev_angle
                                   ) / dt


            self.prev_angle = new_angle
            self.prev_time = current_time

            self.angle_deg = new_angle


            # ---------------------------------------------
            # EWMA velocity filter
            # ---------------------------------------------

            if self.first_velocity:

                self.filtered_vel = (
                    self.raw_vel
                )

                self.first_velocity = False

            else:

                self.filtered_vel = (
                        self.alpha
                        *
                        self.raw_vel
                        +
                        (1.0 - self.alpha)
                        *
                        self.filtered_vel
                )


            self.ankle_velocity = (
                self.filtered_vel
            )


    def set_zero(self):

        with self.lock:

            self.first_value = (
                self.raw_count
            )

            self.angle_deg = 0.0


    def get_angle_deg(self):

        with self.lock:
            return self.angle_deg


    def get_raw_count(self):

        with self.lock:
            return self.raw_count


    def get_ankle_vel(self):

        with self.lock:
            return self.ankle_velocity


# =========================================================
# LOAD CELLS
# =========================================================

class LoadCells:

    def __init__(self):

        self.left1 = 0.0
        self.left2 = 0.0

        self.right1 = 0.0
        self.right2 = 0.0

        self.lock = threading.Lock()


    def update(
            self,
            left1,
            left2,
            right1,
            right2,
    ):

        with self.lock:

            self.left1 = clamp(left1)
            self.left2 = clamp(left2)

            self.right1 = clamp(right1)
            self.right2 = clamp(right2)


    def get_values(self):

        with self.lock:

            return (
                self.left1,
                self.left2,
                self.right1,
                self.right2,
            )


    def get_cable_tensions(self):

        with self.lock:

            return (
                self.left1,
                self.right2,
            )


# =========================================================
# MOTOR COMMAND
# =========================================================

# C++ MotorCmd:
#
# float position
# float velocity
# float torque
# float kp
# float kd
#
# 5 x float32 = 20 bytes

COMMAND_PACKET_FORMAT = "<5f"

COMMAND_PACKET_SIZE = (
    struct.calcsize(
        COMMAND_PACKET_FORMAT
    )
)


@dataclass
class MotorCommand:

    position: float = 0.0
    velocity: float = 0.0

    torque: float = 0.0

    kp: float = 0.0
    kd: float = 0.0


    def to_bytes(self) -> bytes:

        return struct.pack(
            COMMAND_PACKET_FORMAT,
            self.position,
            self.velocity,
            self.torque,
            self.kp,
            self.kd,
        )


# =========================================================
# MOTOR META CONTROL
# =========================================================

class MotorMetaCommand(IntEnum):

    ENTER_MOTOR_MODE = 0
    EXIT_MOTOR_MODE = 1
    SET_ZERO = 2


MOTOR_CONTROL_PACKET_FORMAT = "<B"

MOTOR_CONTROL_PACKET_SIZE = (
    struct.calcsize(
        MOTOR_CONTROL_PACKET_FORMAT
    )
)


@dataclass
class MotorControl:

    command: MotorMetaCommand


    def to_bytes(self) -> bytes:

        return struct.pack(
            MOTOR_CONTROL_PACKET_FORMAT,
            int(self.command),
        )


# =========================================================
# SD LOGGING CONTROL
# =========================================================

class LoggingState(IntEnum):

    STOPPED = 0
    RECORDING = 1


SD_CONTROL_PACKET_FORMAT = "<B"

SD_CONTROL_PACKET_SIZE = (
    struct.calcsize(
        SD_CONTROL_PACKET_FORMAT
    )
)


@dataclass
class SDControl:

    command: LoggingState


    def to_bytes(self) -> bytes:

        return struct.pack(
            SD_CONTROL_PACKET_FORMAT,
            int(self.command),
        )


# =========================================================
# MOTOR FEEDBACK
# =========================================================

class Motor:

    def __init__(self):

        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0

        self.temperature = 0
        self.error = 0

        self.lock = threading.Lock()


    def update(
            self,
            position,
            velocity,
            torque,
            temperature,
            error,
    ):

        with self.lock:

            self.position = position
            self.velocity = velocity
            self.torque = torque

            self.temperature = (
                temperature
            )

            self.error = error


    def get_values(self):

        with self.lock:

            return (
                self.position,
                self.velocity,
                self.torque,
                self.temperature,
                self.error,
            )