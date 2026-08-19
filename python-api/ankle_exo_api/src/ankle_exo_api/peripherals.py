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

    def _update(self, voltage, current=None, power=None):
        """Update available power telemetry.

        The current embedded PowerReadings packet only contains batteryVoltage.
        Optional current/power arguments keep the existing Python API compatible
        if those fields are restored on the embedded side later.
        """
        with self.lock:
            self.voltage = float(voltage)
            if current is not None:
                self.current = float(current)
            if power is not None:
                self.power = float(power)
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
class BLETelemetryPacket:
    """Decoded high-rate telemetry packet from the Nano.

    This layout must match BLETelemetry in the embedded BLE.h exactly.
    MotorFeedback has two bytes of C++ tail padding, represented by ``2x``.
    """

    left_encoder: float
    right_encoder: float
    left_loadcell_torque: float
    right_loadcell_torque: float

    left_motor_torque: float
    left_motor_temperature: int
    left_motor_error: int

    right_motor_torque: float
    right_motor_temperature: int
    right_motor_error: int

    battery_voltage: float

    left_intermediate_torque: float
    right_intermediate_torque: float
    left_controller_output_torque: float
    right_controller_output_torque: float

    FORMAT = "<4f fBB2x fBB2x 5f"

    @classmethod
    def size(cls) -> int:
        return struct.calcsize(cls.FORMAT)

    @classmethod
    def from_bytes(cls, data: bytes):
        if len(data) != cls.size():
            raise ValueError(
                f"Invalid BLE telemetry packet size: "
                f"expected {cls.size()}, got {len(data)}"
            )

        return cls(*struct.unpack(cls.FORMAT, data))


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
    enabled: bool = False

    input_hp_cutoff_hz: float = 0.1
    input_lp_cutoff_hz: float = 5.0
    derivative_lp_cutoff_hz: float = 3.7
    friction_lp_cutoff_hz: float = 5.0

    kp: float = 0.5
    kd: float = 0.01

    comp_torque: float = 0.08

    trigger_on_trq: float = 0.025
    trigger_off_trq: float = 0.010

    max_abs_out_trq: float = 0.4

    FORMAT = "<?3x10f"

    def _values(self):
        return (
            self.enabled,
            self.input_hp_cutoff_hz,
            self.input_lp_cutoff_hz,
            self.derivative_lp_cutoff_hz,
            self.friction_lp_cutoff_hz,
            self.kp,
            self.kd,
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