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

        self.leftTorque = 0.0
        self.rightTorque = 0.0
        self.lock = threading.Lock()

    def _update(self,leftTorque,rightTorque):
        with self.lock:
            self.leftTorque = leftTorque
            self.rightTorque = rightTorque

    def get_values(self):
        """returns the net torque (Nm) for each side of the exo"""
        with self.lock:
            return (
                self.leftTorque,
                self.rightTorque,
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

@dataclass
class TransparentControlCommand(StructPacket):
    enabled : bool = False
    kp : float = 0.5
    kd : float = 0.01
    a_derivative : float = 0.05
    a_friction : float = 0.10
    a_torque : float = 0.15
    comp_torque : float = 0.08
    trigger_on_trq : float = 0.025
    trigger_off_trq : float = 0.010
    max_abs_out_trq : float = 0.4

    FORMAT = "<b3x9f"

    def _values(self):
        return (
            self.enabled,
            self.kp,
            self.kd,
            self.a_derivative,
            self.a_friction,
            self.a_torque,
            self.comp_torque,
            self.trigger_on_trq,
            self.trigger_off_trq,
            self.max_abs_out_trq,
        )


class MotorControlCmd(ByteCommand):
    EXIT_MOTOR_MODE = 0
    ENTER_MOTOR_MODE = 1


class SDLoggerControlCmd(ByteCommand):
    STOP_RECORDING = 0
    START_RECORDING = 1

# ==================== INTERMEDIATE TORQUE =======================

class IntermediateTorque:
    """
    Stores the HP-filtered interaction torque
    received from the embedded controller.
    """

    def __init__(self):

        self.left = 0.0
        self.right = 0.0

        self.lock = threading.Lock()


    def _update_left(self, value):
        with self.lock:
            self.left = float(value)


    def _update_right(self, value):
        with self.lock:
            self.right = float(value)


    def get_values(self):
        with self.lock:
            return (
                self.left,
                self.right,
            )