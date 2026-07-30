"""
This script contains the modules for the sensors and motor
- 2 loadcels
- 1 encoder
- 1 motor in MIT mode
"""

import struct
import threading
from dataclasses import dataclass
from enum import IntEnum

"""
ENCODER
"""

max_count = 4096
half_count = max_count // 2

class Encoder:
    def __init__(self):
        self.raw_count = 0
        self.first_value = None
        self.angle_deg = 0.0

        self.lock = threading.Lock()

    def update(self, raw_count):

        with self.lock:

            self.raw_count = raw_count

            if self.first_value is None: # zero the encoder at the beginning
                self.first_value = raw_count

            relative_count = (raw_count - self.first_value) % max_count 

            # Avoid jumps from max count to zero
            if relative_count >= half_count:
                relative_count -= max_count

            self.angle_deg = (relative_count * 360.0 / max_count)

    def get_angle_deg(self):
        with self.lock:
            return self.angle_deg

    def get_raw_count(self):
        with self.lock:
            return self.raw_count

    def set_zero(self):
        with self.lock:
            self.first_value = self.raw_count
            self.angle_deg = 0.0



"""
LOAD CELLS
"""

class LoadCells:

    def __init__(self):
        self.left1 = 0.0
        self.right2 = 0.0

        self.lock = threading.Lock()

    def update( # add left2 and right1 later when they are being used
            self,
            left1,
            right2,
    ):
        with self.lock:
            self.left1 = left1
            self.right2 = right2

    def get_values(self):
        with self.lock:
            return(self.left1, self.right2)

    def get_cable_tensions(self):
        with self.lock:
            return(self.left1, self.right2)

"""
MOTOR
"""
Motor_ID = 0x02

class MotorCommandType(IntEnum): # assign integer to each motor command
    START = 0
    STOP = 1
    ZERO = 2
    SET = 3

COMMAND_PACKET_FORMAT = "<BB2x5f"
COMMAND_PACKET_SIZE = struct.calcsize(COMMAND_PACKET_FORMAT)

@dataclass
class MotorCommand:
    command_type: MotorCommandType
    motor_id: int = Motor_ID

    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    kp: float = 0.0
    kd: float = 0.0

    def to_bytes(self) -> bytes:

        if not 0 <= self.motor_id <= 255:
            raise ValueError("motor_id must be between 0 and 255")

        return struct.pack(
            COMMAND_PACKET_FORMAT,
            int(self.command_type),
            self.motor_id,
            self.position,
            self.velocity,
            self.torque,
            self.kp,
            self.kd,
        )

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
            self.temperature = temperature
            self.error = error

    def get_values(self):
        with self.lock:
            return(
                self.position,
                self.velocity,
                self.torque,
                self.temperature,
                self.error,
            )

def parse_motor_command(text):

    tokens = text.strip().lower().strip()
    if not tokens:
        raise ValueError("Command is empty")

    command_name = tokens[0]

    if command_name in {"start", "stop", "zero"}:
        motor_id = (
            int(tokens[1],0)
            if len(tokens) >= 2
            else Motor_ID
        )

        command_types = {
            "start": MotorCommandType.START,
            "stop": MotorCommandType.STOP,
            "zero": MotorCommandType.ZERO,
        }

        return MotorCommand(
            command_type = command_types[command_name],
            motor_id=motor_id
        )

    if command_name != "set":
        raise ValueError("Motor commands are expected to begin with start, stop, or set")

    values = {
        "id": Motor_ID,
        "pos": 0.0,
        "vel": 0.0,
        "torque": 0.0,
        "kp": 0.0,
        "kd": 0.0,
    }

    aliases = {
        "trq": "torque",
        "position": "pos",
        "velocity": "vel"
    }

    index = 1

    while index < len(tokens):
        if index + 1 >= len(tokens):
            raise ValueError(
                f"Missing value after '{tokens[index]}'"
            )

        key = tokens[index]
        value = tokens[index + 1]

        # in case the full word is used instead of the shortened version and vice versa
        if key == "trq":
            key = "torque"

        if key == "position":
            key = "pos"

        if key == "velocity":
            key = "vel"

        if key not in values:
            raise ValueError(
                f"Unknown field '{key}'"
            )

        if key == "id":
            values[key] = int(value, 0)
        else:
            values[key] = float(value)

        index += 2

    return MotorCommand(
        command_type=MotorCommandType.SET,
        motor_id=int(values["id"]),
        position=float(values["pos"]),
        velocity=float(values["vel"]),
        torque=float(values["torque"]),
        kp=float(values["kp"]),
        kd=float(values["kd"])
    )