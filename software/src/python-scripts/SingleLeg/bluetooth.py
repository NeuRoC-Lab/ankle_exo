"""
Bluetooth communication for the ankle exoskeleton.

This file handles:
- Connecting to Arduino Nano through Bluetooth
- Receiving motor, encoder, and load cell packets
- Sending motor commands
"""

import asyncio
import queue
import struct
import threading
import time
from dataclasses import dataclass

from bleak import BleakClient, BleakScanner

from sensors import (
    COMMAND_PACKET_SIZE,
    Encoder,
    LoadCells,
    Motor,
    MotorCommand,
    MotorControl,
)


# Bluetooth configuration

DEVICE_NAME = "AnkleExo"
SERVICE_UUID = "CF45813E-4358-4903-B961-09996BB081FB"

LOAD_CELL_UUID = "CA87289F-102B-4078-AD8C-8F53063547A6"
MOTOR_FEEDBACK_UUID = "81DC2896-1B27-4195-A391-99A637FA50A4" #CHANGED
ENCODER_UUID = "094A717B-0C7F-4A23-BFD1-A4924E6E7DAB"
MOTOR_COMMAND_UUID = "E0D883F6-705C-4A11-B117-E2B0909CC68E" #CHANGED
MOTOR_CONTROL_UUID = "99F02A66-065C-41BE-B05E-4BE2B9035A8B" # NEW


@dataclass(frozen=True)
class TelemetrySnapshot:
    sample_time: float

    encoder: int
    ankle_angle: float
    ankle_velocity: float

    loadcell1: float
    loadcell2: float

    motor_position: float
    motor_velocity: float
    motor_torque: float
    motor_temperature: int


class BluetoothManager:

    def __init__(
            self,
            encoder: Encoder,
            loadcells: LoadCells,
            motor: Motor,
            stop_event,
    ):
        self.encoder = encoder
        self.loadcells = loadcells
        self.motor = motor
        self.stop_event = stop_event

        # Queue containing every BLE update.
        # CSV logging drains this queue independently of plot refresh rate.
        self.data_queue = queue.Queue()

        # Motor commands entered in the plot window are placed here.
        # The BLE thread sends them without blocking Matplotlib.
        self.command_queue = queue.Queue()
        self.control_queue = queue.Queue()

        # Writable BLE command characteristic detected after connecting.
        self.command_characteristic_uuid = None

        self.thread = None


    # Bluetooth connection controls

    def connect(self):
        if self.thread is not None and self.thread.is_alive():
            return

        self.thread = threading.Thread(
            target=self._run_bluetooth,
            daemon=True
        )

        self.thread.start()

    def disconnect(self):
        self.stop_event.set()


    # Motor command queue

    def queue_motor_command(self, command: MotorCommand):
        self.command_queue.put(command)

    def queue_motor_control(self, command: MotorCommand):
        self.control_queue.put(command)

    def has_pending_commands(self):
        return (
                not self.command_queue.empty()
                or not self.control_queue.empty()
        )


    # Bluetooth data queue

    def get_pending_snapshots(self):
        snapshots = []

        while True:
            try:
                snapshots.append(
                    self.data_queue.get_nowait()
                )

            except queue.Empty:
                break

        return snapshots


    # Queue latest telemetry snapshot

    def _queue_telemetry_snapshot(self):
        (
            loadcell1,
            loadcell2,
            #TODO understand why you can't unpack the other two load cell values
            #_,
            #_,
        ) = self.loadcells.get_cable_tensions()


        (
            motor_position,
            motor_velocity,
            motor_torque,
            motor_temperature,
            motor_error,
        ) = self.motor.get_values()

        self.data_queue.put(
            TelemetrySnapshot(
                sample_time=time.perf_counter(),

                encoder=self.encoder.get_raw_count(),
                ankle_angle=self.encoder.get_angle_deg(),
                ankle_velocity=self.encoder.get_ankle_vel(),

                loadcell1=loadcell1,
                loadcell2=loadcell2,

                motor_position=motor_position,
                motor_velocity=motor_velocity,
                motor_torque=motor_torque,
                motor_temperature=motor_temperature,
            )
        )


    # BLE packet decoding

    def _motor_callback(self, sender, data):
        """
        Motor packet:

        uint8_t can_id
        float position
        float velocity
        float torque
        uint8_t temperature
        uint8_t error
        """

        try:
            values = struct.unpack(
                "<B3x3fBB2x",
                data
            )
            (
                can_id,
                position,
                velocity,
                torque,
                temperature,
                error
            ) = values

            self.motor.update(
                position=position,
                velocity=velocity,
                torque=torque,
                temperature=temperature,
                error=error,
            )
            self._queue_telemetry_snapshot()

        except Exception as e:
            print("Motor decode error:", e)


    def _loadcell_callback(self, sender, data):
        """
        Load cell packet:

        float left_1
        float left_2
        float right_1
        float right_2
        """

        try:
            values = struct.unpack(
                "<4f", # < for little endian, 4f meaning a struct of 4 floats
                data
            )

            (
                left1,
                left2,
                right1,
                right2
            ) = values

            self.loadcells.update(
                left1,
                left2,
                right1,
                right2,
            )

            self._queue_telemetry_snapshot()

        except Exception as e:
            print("Load cell decode error:", e)


    def _encoder_callback(self, sender, data):
        """
        Encoder packet:

        uint16_t left
        uint16_t right
        """

        try:
            values = struct.unpack(
                "<2H",
                data
            )

            (
                left,
                right
            ) = values

            # Existing single-leg code uses the left encoder.
            self.encoder.update(left, time.perf_counter())

            self._queue_telemetry_snapshot()

        except Exception as e:
            print("Encoder decode error:", e)


    # Find writable BLE command characteristic

    def _find_command_characteristic(self, client):
        """
        Find a writable GATT characteristic in the AnkleExo service.

        Telemetry characteristics are excluded first. If MOTOR_FEEDBACK_UUID itself is
        writable, it is accepted as a fallback.
        """

        telemetry_uuids = {
            LOAD_CELL_UUID.lower(),
            MOTOR_FEEDBACK_UUID.lower(),
            ENCODER_UUID.lower(),
        }

        writable = []
        motor_fallback = None

        for service in client.services:
            if service.uuid.lower() != SERVICE_UUID.lower():
                continue

            for characteristic in service.characteristics:
                properties = {
                    prop.lower()
                    for prop in characteristic.properties
                }

                can_write = (
                        "write" in properties
                        or
                        "write-without-response" in properties
                )

                if not can_write:
                    continue

                if characteristic.uuid.lower() == MOTOR_FEEDBACK_UUID.lower():
                    motor_fallback = characteristic.uuid
                    continue

                if characteristic.uuid.lower() not in telemetry_uuids:
                    writable.append(characteristic.uuid)

        if len(writable) == 1:
            return writable[0]

        if len(writable) > 1:
            print("Multiple writable BLE characteristics found:")

            for characteristic_uuid in writable:
                print("  ", characteristic_uuid)

            print(
                "Using the first writable characteristic. "
                "Set the command UUID explicitly if needed."
            )

            return writable[0]

        if motor_fallback is not None:
            print(
                "No separate command characteristic found; "
                "using MOTOR_FEEDBACK_UUID because it is writable."
            )

            return motor_fallback

        return None


    # Bluetooth motor command sending

    async def _send_pending_motor_commands(
            self,
            client: BleakClient
    ):
        characteristic = (
            client.services.get_characteristic(
                MOTOR_COMMAND_UUID
            )
        )

        if characteristic is None:
            print(
                "Motor command characteristic not found"
            )
            return

        properties = {
            prop.lower()
            for prop in characteristic.properties
        }

        use_response = (
                "write" in properties
        )

        while True:
            try:
                command = (
                    self.command_queue.get_nowait()
                )

            except queue.Empty:
                break

            try:
                packet = command.to_bytes()

                if len(packet) != COMMAND_PACKET_SIZE:
                    raise RuntimeError(
                        f"Incorrect MotorCommand size: "
                        f"{len(packet)}, "
                        f"expected "
                        f"{COMMAND_PACKET_SIZE}"
                    )

                await client.write_gatt_char(
                    characteristic,
                    packet,
                    response=use_response,
                )

                print(
                    "Motor command sent:",
                    f"size={len(packet)}",
                )

            except Exception as exc:
                print(
                    "Could not send motor command:",
                    exc
                )
    async def _send_pending_motor_controls(
            self,
            client: BleakClient
    ):
        characteristic = (
            client.services.get_characteristic(
                MOTOR_CONTROL_UUID
            )
        )

        if characteristic is None:
            print(
                "Motor control characteristic not found"
            )
            return

        properties = {
            prop.lower()
            for prop in characteristic.properties
        }

        use_response = (
                "write" in properties
        )

        while True:
            try:
                command = (
                    self.control_queue.get_nowait()
                )

            except queue.Empty:
                break

            try:
                packet = command.to_bytes()

                # Your MotorControl packet is one byte.
                if len(packet) != 1:
                    raise RuntimeError(
                        f"Incorrect MotorControl size: "
                        f"{len(packet)}, expected 1"
                    )

                await client.write_gatt_char(
                    characteristic,
                    packet,
                    response=use_response,
                )

                print(
                    "Motor control sent:",
                    command.command.name,
                    f"size={len(packet)}",
                )

            except Exception as exc:
                print(
                    "Could not send motor control:",
                    exc
                )
        # Bluetooth connection

    async def _bluetooth_connection(self):
        print("Searching for Bluetooth device... (15s timeout)")

        device = await BleakScanner.find_device_by_name(
            DEVICE_NAME,
            timeout=15.0
        )

        if device is None:
            print("Bluetooth device not found")
            self.stop_event.set()
            return

        async with BleakClient(device) as client:
            print("Connected to Bluetooth")

            await client.start_notify(
                MOTOR_FEEDBACK_UUID,
                self._motor_callback
            )

            await client.start_notify(
                ENCODER_UUID,
                self._encoder_callback
            )

            await client.start_notify(
                LOAD_CELL_UUID,
                self._loadcell_callback
            )

            self.command_characteristic_uuid = (
                self._find_command_characteristic(client)
            )

            if self.command_characteristic_uuid is None:
                print(
                    "WARNING: No writable BLE command characteristic "
                    "was found in the AnkleExo service."
                )

                print(
                    "Plotting and CSV logging will still work, "
                    "but motor commands cannot be sent over BLE."
                )

            else:
                print(
                    "Motor command characteristic:",
                    self.command_characteristic_uuid
                )

            print("Receiving Bluetooth data...")

            while not self.stop_event.is_set():

                await self._send_pending_motor_commands(
                    client
                )

                await self._send_pending_motor_controls(
                    client
                )

                # Small sleep keeps command latency low while allowing
                # BLE notification callbacks to run normally.
                await asyncio.sleep(0.01)


    def _run_bluetooth(self):
        try:
            asyncio.run(
                self._bluetooth_connection()
            )

        except Exception as exc:
            print("Bluetooth error:", exc)
            self.stop_event.set()