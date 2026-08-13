"""
This script contains the modules for the sensors and motor
- 2 load cells
- 1 encoder
    -
- 1 motor in MIT mode
"""

import struct # convert variables into raw bytes for the arduino
import threading
from dataclasses import dataclass
from enum import IntEnum

"""
ENCODER
"""
def clamp(value, minimum=-1000.0, maximum=1000.0):
    return max(minimum, min(value, maximum))


class Encoder:
    def __init__(self):
        self.raw_count = 0
        self.first_value = None
        self.angle_deg = 0.0

        self.raw_vel = 0.0
        self.filtered_vel = 0.0
        self.ankle_velocity = 0.0

        self.alpha = 0.2 # test and adjust EWMA filter coefficient (0 < alpha < 1)
        self.first_velocity = True

        # values used for velocity calculation and filtering
        self.prev_angle = 0.0
        self.prev_time = None

        self.lock = threading.Lock()

    def update(self, raw_count, current_time):

        with self.lock:

            self.raw_count = raw_count

            # encoder position
            if self.first_value is None: # zero the encoder at the beginning
                self.first_value = raw_count

            relative_count = raw_count#(raw_count - self.first_value) % max_count

            # Avoid jumps from max count to zero
            #if relative_count >= half_count:
               # relative_count -= max_count

            new_angle = raw_count#(relative_count * 360.0 / max_count)

            # encoder raw velocity

            if self.prev_time is not None:

                dt = current_time - self.prev_time

                if dt > 0:
                    self.raw_vel = (new_angle - self.prev_angle)/dt

            self.prev_angle = new_angle
            self.prev_time = current_time

            self.angle_deg = new_angle

            # EWMA filter on encoder raw velocity https://corporatefinanceinstitute.com/resources/uncategorized/exponentially-weighted-moving-average-ewma/

            if self.first_velocity: # do not apply EWMA on the first velocity reading
                self.filtered_vel = self.raw_vel
                self.first_velocity = False

            else:
                self.filtered_vel = (
                    self.alpha * self.raw_vel + (1 - self.alpha) * self.filtered_vel
                )

            self.ankle_velocity = self.filtered_vel

    def set_zero(self): # zero the encoder
        with self.lock:
            self.first_value = self.raw_count
            self.angle_deg = 0.0

    def get_angle_deg(self): # returns the encoder angle in degrees
        with self.lock:
            return self.angle_deg

    def get_raw_count(self): # returns the raw encoder count (debugging purpose in case angle conversion doesnt work)
        with self.lock:
            return self.raw_count

    def get_ankle_vel(self): # returns the ankle velocity in deg/s
        with self.lock:
            return self.ankle_velocity



"""
LOAD CELLS
"""

class LoadCells:

    def __init__(self):
        self.left1 = 0.0
        self.left2 = 0.0
        self.right1 = 0.0
        self.right2 = 0.0

        self.lock = threading.Lock()

    def update( # first leg setup uses left1 and right2
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
            return(
                self.left1,
                self.left2,
                self.right1,
                self.right2
            )

    def get_cable_tensions(self): # returns the cable tension on the current single leg exo setup
        with self.lock:
            return(self.left1, self.right2)

"""
MOTOR
"""
Motor_ID = 0x02 # current motor id, modify as needed


COMMAND_PACKET_FORMAT = "<5f" # 5 floats
COMMAND_PACKET_SIZE = struct.calcsize(COMMAND_PACKET_FORMAT)

@dataclass
class MotorCommand:
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    kp: float = 0.0
    kd: float = 0.0

    def to_bytes(self) -> bytes:

        #if not 0 <= self.motor_id <= 255:
            #raise ValueError("motor_id must be between 0 and 255")

        return struct.pack(
            COMMAND_PACKET_FORMAT,
            #int(self.command_type),
            #self.motor_id,
            self.position,
            self.velocity,
            self.torque,
            self.kp,
            self.kd,
        )

# import libraries for motor classes
from dataclasses import dataclass
from enum import IntEnum
import struct


class MotorMetaCommand(IntEnum):
    ENTER_MOTOR_MODE = 0
    EXIT_MOTOR_MODE = 1
    SET_ZERO = 2


MOTOR_CONTROL_PACKET_FORMAT = "<B"


@dataclass
class MotorControl:
    command: MotorMetaCommand

    def to_bytes(self) -> bytes:
        return struct.pack(
            MOTOR_CONTROL_PACKET_FORMAT,
            int(self.command),
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

"""
def parse_motor_command(text): # convert user text into MotorCommand object

    tokens = text.strip().lower().split()
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
"""