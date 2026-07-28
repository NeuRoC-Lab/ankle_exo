"""
Bluetooth interface for the ankle exoskeleton.

"""

import asyncio
import queue
import struct
import threading
import time

from bleak import BleakClient, BleakScanner

from core import (
    COMMAND_PACKET_SIZE,
    MotorCommand,
    Telemetry,
    TelemetrySnapshot,
)


# Bluetooth configuration

DEVICE_NAME = "AnkleExo"
SERVICE_UUID = "CF45813E-4358-4903-B961-09996BB081FB"

LOAD_CELL_UUID = "CA87289F-102B-4078-AD8C-8F53063547A6"
MOTOR_UUID = "E0D883F6-705C-4A11-B117-E2B0909CC68E"
ENCODER_UUID = "094A717B-0C7F-4A23-BFD1-A4924E6E7DAB"
COMMAND_UUID = "C94B7403-6BFB-4A06-BA12-6394765C328E"


class AnkleExoBluetooth:

    def __init__(self, stop_event):

        self.stop_event = stop_event

        self.telemetry = Telemetry()

        # Queue containing every BLE update.
        # CSV logging drains this queue independently of plot refresh rate.
        self._ble_data_queue = queue.Queue()

        # Motor commands entered in the plot window are placed here.
        # The BLE thread sends them without blocking Matplotlib.
        self._command_queue = queue.Queue()

        # The writable BLE command characteristic is detected after connecting.
        self.command_characteristic_uuid = None

        self.thread = None


    # Public interface used by the main program

    def connect(self):

        """
        Start the Bluetooth connection in a background thread.

        Matplotlib remains on the main thread while Bleak runs its own
        asyncio event loop in this thread.
        """

        if self.thread is not None and self.thread.is_alive():
            return self.thread

        self.thread = threading.Thread(
            target=self._run_bluetooth,
            daemon=True
        )

        self.thread.start()

        return self.thread


    def disconnect(self):

        """
        Tell the Bluetooth loop to stop.

        The BleakClient async context manager closes the BLE connection
        when the loop exits.
        """

        self.stop_event.set()


    def queue_motor_command(self, command: MotorCommand) -> None:
        self._command_queue.put(command)


    def has_pending_commands(self):
        return not self._command_queue.empty()


    def get_pending_snapshots(self):

        """
        Return all telemetry snapshots currently waiting in the BLE queue.

        This hides the queue implementation from the main script.
        """

        snapshots = []

        while True:

            try:
                snapshots.append(
                    self._ble_data_queue.get_nowait()
                )

            except queue.Empty:
                break

        return snapshots


    # Queue latest telemetry snapshot

    def _queue_telemetry_snapshot(self):

        """
        Add the latest complete telemetry state to the queue.

        The BLE callbacks call this after updating their own sensor values.
        CSV logging can therefore run independently of the plot refresh rate.
        """

        self._ble_data_queue.put(
            TelemetrySnapshot(
                sample_time=time.perf_counter(),

                encoder=self.telemetry.encoder,

                loadcell1=self.telemetry.loadcell1,
                loadcell2=self.telemetry.loadcell2,

                motor_position=self.telemetry.motor_position,
                motor_velocity=self.telemetry.motor_velocity,
                motor_torque=self.telemetry.motor_torque,
                motor_temperature=self.telemetry.motor_temperature,

                motor_packets=self.telemetry.motor_packets,
                loadcell_packets=self.telemetry.loadcell_packets,
                encoder_packets=self.telemetry.encoder_packets,
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


            with self.telemetry.lock:

                self.telemetry.motor_position = position
                self.telemetry.motor_velocity = velocity
                self.telemetry.motor_torque = torque
                self.telemetry.motor_temperature = temperature

                self.telemetry.motor_packets += 1

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
                "<4f",
                data
            )

            (
                left1,
                left2,
                right1,
                right2
            ) = values


            with self.telemetry.lock:

                self.telemetry.loadcell1 = left1
                self.telemetry.loadcell2 = right2

                self.telemetry.loadcell_packets += 1

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


            with self.telemetry.lock:

                self.telemetry.encoder = left
                self.telemetry.encoder_packets += 1

                self._queue_telemetry_snapshot()


        except Exception as e:
            print("Encoder decode error:", e)


    # Bluetooth motor command sending

    def _find_command_characteristic(self, client):

        """
        Find a writable GATT characteristic in the AnkleExo service.

        COMMAND_UUID is preferred if it is present and writable.
        Otherwise telemetry characteristics are excluded and another writable
        characteristic is selected. If MOTOR_UUID itself is writable, it is
        accepted as a final fallback.
        """

        telemetry_uuids = {
            LOAD_CELL_UUID.lower(),
            MOTOR_UUID.lower(),
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

                if characteristic.uuid.lower() == COMMAND_UUID.lower():
                    return characteristic.uuid

                if characteristic.uuid.lower() == MOTOR_UUID.lower():
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
                "using MOTOR_UUID because it is writable."
            )

            return motor_fallback

        return None


    async def _send_pending_commands(self, client: BleakClient) -> None:

        """
        Send all queued binary CommandPayload packets.
        """

        if self.command_characteristic_uuid is None:
            return

        characteristic = client.services.get_characteristic(
            self.command_characteristic_uuid
        )

        if characteristic is None:
            print("BLE command characteristic not found")
            return

        properties = {
            prop.lower()
            for prop in characteristic.properties
        }

        use_response = "write" in properties

        while True:

            try:
                command = self._command_queue.get_nowait()

            except queue.Empty:
                break

            try:

                packet = command.to_bytes()

                if len(packet) != COMMAND_PACKET_SIZE:
                    raise RuntimeError(
                        f"Incorrect command size: {len(packet)}, "
                        f"expected {COMMAND_PACKET_SIZE}"
                    )

                await client.write_gatt_char(
                    characteristic,
                    packet,
                    response=use_response,
                )

                print(
                    "BLE command sent:",
                    command.command_type.name,
                    f"id={command.motor_id}",
                    f"size={len(packet)}",
                )

            except Exception as exc:
                print("Could not send BLE command:", exc)


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
                MOTOR_UUID,
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

                await self._send_pending_commands(client)

                # Small sleep keeps command latency low while allowing
                # BLE notification callbacks to run normally.
                await asyncio.sleep(0.01)


    def _run_bluetooth(self):

        try:
            asyncio.run(self._bluetooth_connection())

        except Exception as exc:
            print("Bluetooth error:", exc)
            self.stop_event.set()