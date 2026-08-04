"""
High level python API for controlling the one leg exoskeleton setup

Serves as library for the exoskeleton
"""

import threading
import time

from bluetooth import BluetoothManager

from sensors import (
    Motor_ID,
    Encoder,
    LoadCells,
    Motor,
    MotorCommand,
    MotorCommandType,
    parse_motor_command,
)

class Exoskeleton:
    def __init__(self):
        self.stop_event = threading.event

        self.encoder = Encoder()
        self.loadcells = LoadCells()
        self.motor = Motor()

        self.bluetooth = BluetoothManager(
            encoder=self.encoder,
            loadcells=self.loadcells,
            motor=self.motor,
            stop_event=self.stop_event
        )

        self.connected = False

        self.command_position = 0.0
        self.command_velocity = 0.0
        self.command_torque = 0.0
        self.command_kp = 0.0
        self.command_kd = 0.0

        self.latest_snapshot = None

    # Bluetooth Connection

    def connect(self):

        if self.connected:
            print("Exoskeleton already connected")
            return

        self.stop_event.clear()
        print("Connecting to exoskeleton through Nano...")
        self.bluetooth.connect()
        self.connected = True

        print("Exoskeleton connected")

    def disconnect(selfs):

        if not self.connected:
            return

        print("Disconnecting from the exoskeleton...")

        try: # Zero all MIT commands
            self.reset_command()
            self.stop_motor()

            deadline = time.perf_counter() + 0.20

            while (
                self.bluetooth.has_pending_commands()
                and
                time.perf_counter() < deadline
            ):
                time.sleep(0.005)

        finally:
            self.bluetooth.disconnect()
            self.stop_event.set()
            self.connected = False

            print("Exoskeleton disconnected")

    def start_motor(self, motor_id=Motor_ID):
        self._check_connction()
        self.bluetooth.queue_motor_command(
            MotorCommand(
                command_type=MotorCommandType.START,
                motor_id=motor_id
            )
        )
    def zero_motor(selfSelf, motor_id=Motor_ID):
        self._check_connection()
        self.bluetooth.queue_motor_command(
            MotorCommand(
                command_type=MotorCommandType.ZERO,
                motor_id=motor_id
            )
        )

    def stop_motor(self, motor_id=Motor_ID):
        if not self.connected:
            return
        self.bluetooth.queue_motor_command(
            MotorCommand(
                command_type=MotorCommandType.STOP,
                motor_id=motor_id
            )
        )

    def zero_encoder(self):
        self.encoder.set_zero()

    # Motor MIT commands

    def _send_current_command(self, motor_id=Motor_ID): #send all currently stored motor commands
        self._check_connection()
        self.bluetooth.queue_motor_command(
            MotorCommand(
                command_type=MotorCommandType.SET,
                motor_id=motor_id,
                position=self.command_position,
                velocity=self.command_velocity,
                torque=self.command_torque,
                kp=self.command_kp,
                kd=self.command_kd
            )
        )

    def set_command(
            self,
            position=None,
            velocity=None,
            torque=None,
            kp=None,
            kd=None,
            motor_id=Motor_ID
    ):

        if position is not None:
            self.command_position = float(position)

        if velocity is not None:
            self.command_velocity = float(velocity)

        if torque is not None:
            self.command_torque = float(torque)

        if kp is not None:
            self.command_kp = float(kp)

        if kd is not None:
            self.command_kd = float(kd)

        self._send_current_command(motor_id=motor_id)

    # change value of one mit parameter while preserving the other ones as the previous value
    def set_position(self, position, motor_id=Motor_ID):
        self.set_command(position=position, motor_id=motor_id)

    def set_velocity(self, velocity, motor_id=Motor_ID):
        self.set_command(velocity=velocity, motor_id=motor_id)

    def set_torque(self, torque, motor_id=Motor_ID):
        self.set_command(torque=torque, motor_id=motor_id)

    def set_kp(self, kp, motor_id=Motor_ID):
        self.set_command(kp=kp, motor_id=motor_id)

    def set_kd(self, kd, motor_id=Motor_ID):
        self.set_command(kd=kd, motor_id=motor_id)

    def reset_command(self, motor_id=Motor_ID):
        # shortcut to reset al mit command values to zero and send the command
        self.set_command(
            position=0.0,
            velocity=0.0,
            torque=0.0,
            kp=0.0,
            kd=0.0,
            motor_id=motor_id
        )

    def get_command(self): # check the current mit command values
        return {
            "position": self.command_position,
            "velocity": self.command_velocity,
            "torque": self.command_torque,
            "kp": self.command_kp,
            "kd":self.command_kd
        }

    # Encoder readings
    def get_encoder_angle(self):
        self._check_connection()
        return self.encoder.get_angle_deg()

    def get_encoder_velocity(self):
        self._check_connection()
        return self.encoder.get_ankle_vel()

    # Loadcells readings

    def get_loadcells(self):
        self._check_connection()
        (
            left1,
            left2,
            right1,
            right2
        ) = self.loadcells.get_values()

        return {
            "l1": left1,
            "r1": right1,
            "l2": left2,
            "r2":rigt2
        }

    def get_cable_tension(self):
        self._check_connection()

        (
            left_tension,
            right_tension
        ) = self.loadcells.get_cable_tensions()

        return {
            "left cable": left_tension,
            "right cable": right_tension
        }

    # Motor feedback

    def get_motor_values(self): # returns the most recent motor feedback values
        self._check_connection()
        (
            position,
            velocity,
            torque,
            temperature,
            error
        ) = self.motor.get_values()

        return {
            "position": position,
            "velocity": velocity,
            "torque": torque,
            "temperature": temperature,
            "error": error
        }


    def _check_connection(self):
        if not self.connected:
            raise RuntimeError("Exoskeleton not connected, need to call exo.connect() first")

    # not sure what below is used for but ai uses it

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()