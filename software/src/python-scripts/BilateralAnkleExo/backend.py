# this script will apply successive tensions on both load cells through a motor commanded torque control

"""
Backend-only Bluetooth interface for the bilateral ankle exoskeleton.

This module:
- Connects to the Arduino Nano over BLE
- Sends motor commands
- Provides access to the latest sensor values
- Does not create plots or CSV files

Expected existing modules:
    bluetooth.py
    sensors.py
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from bluetooth import BluetoothManager

from sensors import (
    Motor_ID,
    Encoders,
    LoadCells,
    Motor,
    MotorCommand,
    MotorCommandType,
    parse_motor_command,
)


class AnkleExoBackend:
    def __init__(self) -> None:
        self.stop_event = threading.Event()

        self.left_encoder = Encoders()
        self.right_encoder = None

        self.loadCells = LoadCells()

        self.left_motor = Motor()
        self.right_motor = None#Motor()

        self.bluetooth = BluetoothManager(
            encoder=self.left_encoder,
            loadcells=self.loadCells,
            motor=self.left_motor,
            stop_event=self.stop_event,
        )

        self._connected = False
        self._latest_snapshot = None

    def connect(self) -> None:
        """Connect to the Arduino Nano over BLE."""
        if self._connected:
            return

        print("Connecting to ankle exoskeleton...")

        self.bluetooth.connect()
        self._connected = True

        print("Bluetooth connected")

    def disconnect(self, stop_motors: bool = True) -> None:
        """
        Disconnect from BLE.

        By default, a stop command is queued before disconnecting.
        """
        if not self._connected:
            return

        if stop_motors:
            self.stop_all_motors()
            self._wait_for_pending_commands(timeout=0.1)

        self.stop_event.set()
        self.bluetooth.disconnect()
        self._connected = False

        print("Bluetooth disconnected")

    def is_connected(self) -> bool:
        return self._connected

    def send_command(self, command: MotorCommand) -> None:
        """Queue a MotorCommand for transmission."""
        self._require_connection()
        self.bluetooth.queue_motor_command(command)

    def send_command_string(self, command_text: str) -> None:
        """
        Parse and send a text command.

        Example:
            "set id 1 pos 0.0 vel 0.0 kp 10.0 kd 0.5 trq 0.0"
        """
        command = parse_motor_command(command_text)
        self.send_command(command)

    def start_motor(self, motor_id: int) -> None:
        self.send_command(
            MotorCommand(
                command_type=MotorCommandType.START,
                motor_id=motor_id,
            )
        )

    def stop_motor(self, motor_id: int) -> None:
        self.send_command(
            MotorCommand(
                command_type=MotorCommandType.STOP,
                motor_id=motor_id,
            )
        )

    def zero_motor(self, motor_id: int) -> None:
        self.send_command(
            MotorCommand(
                command_type=MotorCommandType.ZERO,
                motor_id=motor_id,
            )
        )

    def start_all_motors(self) -> None:
        self.start_motor(Motor_ID)

    def stop_all_motors(self) -> None:
        self.stop_motor(Motor_ID)

    def set_motor_command(
            self,
            motor_id: int,
            position: float = 0.0,
            velocity: float = 0.0,
            kp: float = 0.0,
            kd: float = 0.0,
            torque: float = 0.0,
    ) -> None:
        """
        Send an MIT-mode motor command.

        Adjust the MotorCommand constructor field names here if your
        sensors.py implementation uses a nested `cmd` object.
        """
        command = MotorCommand(
            command_type=MotorCommandType.SET,
            motor_id=motor_id,
            position=position,
            velocity=velocity,
            kp=kp,
            kd=kd,
            torque=torque,
        )

        self.send_command(command)

    def update(self):
        """
        Drain received BLE snapshots.

        Returns the newest snapshot received since the previous call,
        or None if no new snapshot is available.
        """
        self._require_connection()

        snapshots = self.bluetooth.get_pending_snapshots()

        if snapshots:
            self._latest_snapshot = snapshots[-1]

        return self._latest_snapshot

    def get_latest_snapshot(self):
        """
        Retrieve the latest available sensor snapshot.

        This first drains any newly received BLE snapshots.
        """
        return self.update()

    def get_pending_snapshots(self) -> list:
        """
        Return every snapshot received since the previous call.

        Useful when every BLE sample matters.
        """
        self._require_connection()

        snapshots = self.bluetooth.get_pending_snapshots()

        if snapshots:
            self._latest_snapshot = snapshots[-1]

        return snapshots

    def get_sensor_values(self) -> Optional[dict]:
        """
        Return the latest values as a dictionary.

        The attribute names may need adjustment depending on the exact
        snapshot structure defined in bluetooth.py or sensors.py.
        """
        snapshot = self.get_latest_snapshot()

        if snapshot is None:
            return None

        return {
            "sample_time": snapshot.sample_time,

            "left_encoder": snapshot.left_encoder,
            "right_encoder": snapshot.right_encoder,

            "left_loadcells": snapshot.left_loadcells,
            "right_loadcells": snapshot.right_loadcells,

            "left_motor": snapshot.left_motor,
            "right_motor": snapshot.right_motor,
        }

    def wait_for_snapshot(
            self,
            timeout: float = 1.0,
            poll_interval: float = 0.005,
    ):
        """
        Wait for a new sensor snapshot.

        Returns:
            The newest snapshot, or None if the timeout expires.
        """
        self._require_connection()

        deadline = time.perf_counter() + timeout

        while time.perf_counter() < deadline:
            snapshots = self.bluetooth.get_pending_snapshots()

            if snapshots:
                self._latest_snapshot = snapshots[-1]
                return self._latest_snapshot

            time.sleep(poll_interval)

        return None

    def _wait_for_pending_commands(self, timeout: float) -> bool:
        deadline = time.perf_counter() + timeout

        while (
                self.bluetooth.has_pending_commands()
                and time.perf_counter() < deadline
        ):
            time.sleep(0.005)

        return not self.bluetooth.has_pending_commands()

    def _require_connection(self) -> None:
        if not self._connected:
            raise RuntimeError("Bluetooth backend is not connected")

    def __enter__(self) -> "AnkleExoBackend":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.disconnect()