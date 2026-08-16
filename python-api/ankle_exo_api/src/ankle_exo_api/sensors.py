from typing import Any

import struct
from dataclasses import dataclass
from enum import IntEnum
import threading

class StructPacket:
    FORMAT: str

    def to_bytes(self) -> bytes:
        return struct.pack(
            self.FORMAT,
            *self._values(),
        )

    @classmethod
    def size(cls) -> int:
        return struct.calcsize(
            cls.FORMAT
        )

    def _values(self):
        raise NotImplementedError

class ByteCommand(IntEnum):

    def to_bytes(self) -> bytes:
        return struct.pack(
            "<B",
            int(self),
        )

    @classmethod
    def size(cls) -> int:
        return 1

# ==================== ENCODER LOGIC =======================

class Encoders:
    def __init__(self):

        self.left_angle = 0
        self.right_angle = 0
        self.lock = threading.Lock()

    def _update(self, left_angle: object, right_angle: object) -> Any:
        with self.lock:
            self.left_angle = left_angle
            self.right_angle = right_angle

    def get_values(self):
        with self.lock:
            return (
                self.left_angle,
                self.right_angle,
            )


# ==================== LOAD CELL LOGIC =======================

class LoadCells:
    def __init__(self):

        self.left1 = 0.0
        self.left2 = 0.0
        self.right1 = 0.0
        self.right2 = 0.0
        self.lock = threading.Lock()

    def _update(self,left1,left2,right1,right2):
        with self.lock:
            self.left1 = left1
            self.left2 = left2
            self.right1 = right1
            self.right2 = right2
    def get_values(self):
        """returns all four load cell forces in Newton (N)"""
        with self.lock:
            return (
                self.left1,
                self.left2,
                self.right1,
                self.right2,
            )

# ==================== MOTOR LOGIC =======================
class Motor:
    def __init__(self):
        self.id  = 0
        #self.position = 0.0
        #self.velocity = 0.0
        self.torque = 0.0
        self.temperature = 0
        self.error = 0
        self.lock = threading.Lock()

    def _update(self,torque,temperature,error):
        """Meant to be used internally by bluetooth manager to update motor state"""
        with self.lock:
            #self.position = position
            #self.velocity = velocity
            self.torque = torque
            self.temperature = temperature
            self.error = error
    def get_values(self):
        with self.lock:
            return (
                #self.position,
                #self.velocity,
                self.torque,
                self.temperature,
                self.error,
            )

class Power:
    def __init__(self):
        self.voltage = 0.0
        self.current = 0.0
        self.power = 0.0

        self.lock = threading.Lock()

    def _update(self,voltage,current,power):
        """Meant to be used internally by bluetooth manager to update motor state"""
        with self.lock:
            self.voltage = voltage
            self.current = current
            self.power = power
        return False
    def get_values(self):

        with self.lock:

            return (
                self.voltage,
                self.current,
                self.power,
            )
        return False

@dataclass
class MotorCommand(StructPacket):
    #position: float = 0.0
    #velocity: float = 0.0
    torque: float = 0.0
    #kp: float = 0.0
   # kd: float = 0.0

    FORMAT = "<1f"

    def _values(self):
        return (
            #self.position,
            #self.velocity,
            self.torque,
            #self.kp,
            #self.kd,
        )

class MotorControlCmd(ByteCommand):
    ENTER_MOTOR_MODE = 0
    EXIT_MOTOR_MODE = 1
    SET_ZERO = 2


class SDLoggerControlCmd(ByteCommand):
    STOP_RECORDING = 0
    START_RECORDING = 1
