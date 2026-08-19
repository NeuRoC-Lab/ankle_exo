import queue
import threading
import struct
import asyncio
from enum import  Enum

from bleak import BleakClient, BleakScanner

from .peripherals import (
    Encoders,
    LoadCells,
    Motor,
    MotorCommand,
    MotorControlCmd,
    SDLoggerControlCmd,
    Power,
    TransparentControlCommand,
    IntermediateTorque,
    BLETelemetryPacket,
)

class Side(Enum):
    #a class that acts as an access modifier to distinguish between left/right motor, as well as encoders and load cells
    LEFT = "left"
    RIGHT = "right"

class CanId(Enum):
    LEFT = 3
    RIGHT = 2


DEVICE_NAME = "AnkleExo"
SERVICE_UUID = "CF45813E-4358-4903-B961-09996BB081FB"

TELEMETRY_UUID = "348B92F4-EB75-476B-A124-5D8C97C35907"

RIGHT_MOTOR_COMMAND_UUID = "09DC04D0-BFC0-4D7C-A88D-96D60857FE64"
LEFT_MOTOR_COMMAND_UUID = "E0D883F6-705C-4A11-B117-E2B0909CC68E"

RIGHT_MOTOR_CONTROL_UUID = "12E89431-F2D4-4495-B984-A127A79D1591"
LEFT_MOTOR_CONTROL_UUID = "99F02A66-065C-41BE-B05E-4BE2B9035A8B"

SD_LOGGER_UUID = "A06EE428-0AB6-4CD4-AA8A-91619F1AF577"

LEFT_TRANSPARENT_CONTROLLER_UUID = "644CD587-A563-437E-8006-8B7F39559690"
RIGHT_TRANSPARENT_CONTROLLER_UUID = "A8B58E7F-4B57-4C5F-85E4-55F38A6CE271"

MOTOR_COMMAND_UUIDS = {
    Side.LEFT:
        LEFT_MOTOR_COMMAND_UUID,

    Side.RIGHT:
        RIGHT_MOTOR_COMMAND_UUID,
}


MOTOR_CONTROL_UUIDS = {
    Side.LEFT:
        LEFT_MOTOR_CONTROL_UUID,

    Side.RIGHT:
        RIGHT_MOTOR_CONTROL_UUID,
}

TRANSPARENT_CONTROLLER_UUIDS = {
    Side.LEFT:
        LEFT_TRANSPARENT_CONTROLLER_UUID,
    Side.RIGHT:
        RIGHT_TRANSPARENT_CONTROLLER_UUID,
}




class BluetoothManager:
    """Owns the BLE connection with the Arduino Nano"""
    def __init__(
            self,
            encoders: Encoders,
            loadcells: LoadCells,
            power: Power,
            left_motor: Motor,
            right_motor: Motor,
            intermediate_torque: IntermediateTorque,
            controller_output_torque: IntermediateTorque,
            stop_event):

        self.encoders = encoders # two encoders
        self.loadcells = loadcells # four load cells
        self.power = power
        self.motors = {
            Side.LEFT :left_motor,
            Side.RIGHT : right_motor,
        }


        self.stop_event = stop_event


        self.command_queues = {
            Side.LEFT: queue.Queue(),
            Side.RIGHT: queue.Queue(),
        }

        self.control_queues = {
            Side.LEFT: queue.Queue(),
            Side.RIGHT: queue.Queue(),
        }

        self.sd_queue = queue.Queue()

        self.transparent_queues = {
            Side.LEFT : queue.Queue(),
            Side.RIGHT : queue.Queue(),
        }

        self.data_queue = queue.Queue()
        #TODO determine if that is really necessary

        self.ble_thread = None

        self.intermediate_torque = intermediate_torque
        self.controller_output_torque = controller_output_torque

        self.telemetry_packets_received = 0
        self.telemetry_packets_malformed = 0



    def connect(self):
        if (self.ble_thread is not None and self.ble_thread.is_alive()):
            return

        self.stop_event.clear()

        self.ble_thread = threading.Thread(
            target=self._run_bluetooth,
            daemon=True,
        )

        self.ble_thread.start()




    def disconnect(self):

        self.stop_event.set()

        if self.ble_thread is not None:

            self.ble_thread.join(
                timeout=2.0
            )

        self.ble_thread = None

    def _has_pending_commands(self):
        return (
                not self.command_queues[Side.LEFT].empty()
                or not self.command_queues[Side.RIGHT].empty()
                or not self.control_queues[Side.LEFT].empty()
                or not self.control_queues[Side.RIGHT].empty()
                or not self.sd_queue.empty()
                or not self.transparent_queues.values()[Side.LEFT].empty()
                or not self.transparent_queues.values()[Side.RIGHT].empty()
        )

    def _fetch_telemetry(self, sender, data):
        """Decode one aggregated Nano telemetry notification."""
        try:
            telemetry = BLETelemetryPacket.from_bytes(bytes(data))
        except (ValueError, struct.error) as exc:
            self.telemetry_packets_malformed += 1
            print(
                "Malformed BLE telemetry packet "
                f"({len(data)} bytes): {exc}"
            )
            return

        self.telemetry_packets_received += 1

        if self.encoders is not None:
            self.encoders._update(
                telemetry.left_encoder,
                telemetry.right_encoder,
            )

        if self.loadcells is not None:
            self.loadcells._update(
                telemetry.left_loadcell_torque,
                telemetry.right_loadcell_torque,
            )

        if self.motors[Side.LEFT] is not None:
            self.motors[Side.LEFT]._update(
                telemetry.left_motor_torque,
                telemetry.left_motor_temperature,
                telemetry.left_motor_error,
            )

        if self.motors[Side.RIGHT] is not None:
            self.motors[Side.RIGHT]._update(
                telemetry.right_motor_torque,
                telemetry.right_motor_temperature,
                telemetry.right_motor_error,
            )

        if self.power is not None:
            self.power._update(
                telemetry.battery_voltage
            )

        self.intermediate_torque._update_left(
            telemetry.left_intermediate_torque
        )
        self.intermediate_torque._update_right(
            telemetry.right_intermediate_torque
        )

        self.controller_output_torque._update_left(
            telemetry.left_controller_output_torque
        )
        self.controller_output_torque._update_right(
            telemetry.right_controller_output_torque
        )

    def queue_motor_command(
            self,
            side: Side,
            command: MotorCommand,
    ):
        self.command_queues[
            side
        ].put(
            command
        )

    def queue_motor_control(self,side: Side,command: MotorControlCmd):
        self.control_queues[side].put(command)

    def queue_sd_command(self,command: SDLoggerControlCmd):
        self.sd_queue.put(command)

    def queue_transparent_command(self,side: Side, command : TransparentControlCommand):
        self.transparent_queues[side].put(command)

    def has_pending_commands(self):

        return (
                any(
                    not q.empty()
                    for q in
                    self.command_queues.values()
                )
                or
                any(
                    not q.empty()
                    for q in
                    self.control_queues.values()
                )
                or
                any(
                    not q.empty()
                    for q in
                    self.transparent_queues.values()
                )
                or
                not self.sd_queue.empty()
        )

    def clear_pending_commands(self):

        for q in self.command_queues.values():
            self._clear_queue(q)

        for q in self.control_queues.values():
            self._clear_queue(q)

        for q in self.transparent_queues.values():
            self._clear_queue(q)

        self._clear_queue(
            self.sd_queue
        )

    @staticmethod
    def _clear_queue(q):

        while True:

            try:
                q.get_nowait()

            except queue.Empty:
                break

    async def _send_pending_motor_commands(
            self,
            client: BleakClient,
            side: Side,
    ):
        characteristic = (
            client.services
            .get_characteristic(
                MOTOR_COMMAND_UUIDS[
                    side
                ]
            )
        )
        properties = {
            prop.lower()
            for prop
            in characteristic.properties
        }

        use_response = (
                "write-without-response" not in properties
                and "write" in properties
        )

        command_queue = (self.command_queues[side])
        while True:
            try:
                command = (command_queue.get_nowait())
            except queue.Empty:
                break
            try:

                packet = (command.to_bytes())

                await client.write_gatt_char(
                    characteristic,
                    packet,
                    response=use_response,
                )

            except Exception as exc:

                print(
                    f"Could not send "
                    f"{side.value} "
                    f"motor command:",
                    exc,
                )


    async def _send_pending_motor_controls(
            self,
            client: BleakClient,
            side: Side,
    ):
        characteristic = (
            client.services
            .get_characteristic(
                MOTOR_CONTROL_UUIDS[
                    side
                ]
            )
        )


        properties = {
            prop.lower()
            for prop
            in characteristic.properties
        }

        use_response = (
                "write-without-response" not in properties
                and "write" in properties
        )


        control_queue = (
            self.control_queues[
                side
            ]
        )

        while True:
            try:
                command = (control_queue.get_nowait())
            except queue.Empty:
                break
            try:
                packet = (command.to_bytes())

                await client.write_gatt_char(
                    characteristic,
                    packet,
                    response=use_response,
                )

            except Exception as exc:

                print(
                    f"Could not send "
                    f"{side.value} "
                    f"motor control:",
                    exc,
                )


    async def _send_pending_sd_commands(
            self,
            client: BleakClient,
    ):
        characteristic = (
            client.services
            .get_characteristic(
                SD_LOGGER_UUID
            )
        )

        properties = {
            prop.lower()
            for prop
            in characteristic.properties
        }

        use_response = (
                "write-without-response" not in properties
                and "write" in properties
        )

        while True:

            try:
                command = (self.sd_queue.get_nowait())

            except queue.Empty:
                break
            try:
                packet = (command.to_bytes())

                await client.write_gatt_char(
                    characteristic,
                    packet,
                    response=use_response,
                )

            except Exception as exc:

                print(
                    "Could not send "
                    "SD logger control:",
                    exc,
                )

    async def _send_pending_transparent_commands(
            self,
            client: BleakClient,
            side: Side,
    ):
        characteristic = (
            client.services
            .get_characteristic(
                TRANSPARENT_CONTROLLER_UUIDS[
                    side
                ]
            )
        )


        properties = {
            prop.lower()
            for prop
            in characteristic.properties
        }

        use_response = (
                "write-without-response" not in properties
                and "write" in properties
        )


        transparent_queue = (
            self.transparent_queues[
                side
            ]
        )

        while True:
            try:
                command = (transparent_queue.get_nowait())
            except queue.Empty:
                break
            try:
                packet = (command.to_bytes())

                await client.write_gatt_char(
                    characteristic,
                    packet,
                    response=use_response,
                )

            except Exception as exc:

                print(
                    f"Could not send "
                    f"{side.value} "
                    f"transparent mode control:",
                    exc,
                )


    async def _bluetooth_connection(self):
        print("Searching for Bluetooth device...")
        device = (
            await
            BleakScanner.find_device_by_name(
                DEVICE_NAME,
                timeout=15.0,
            )
        )
        if device is None:
            print("Bluetooth device not found")
            self.stop_event.set()
            return

        async with BleakClient(device) as client:
            print("Connected to Bluetooth")

            telemetry_characteristic = (
                client.services.get_characteristic(
                    TELEMETRY_UUID
                )
            )

            if telemetry_characteristic is None:
                raise RuntimeError(
                    "Telemetry characteristic not found: "
                    f"{TELEMETRY_UUID}"
                )

            print(
                "BLE telemetry packet size expected: "
                f"{BLETelemetryPacket.size()} bytes"
            )

            await client.start_notify(
                telemetry_characteristic,
                self._fetch_telemetry,
            )

            print("Receiving Bluetooth data...")


            while not self.stop_event.is_set():
                for side in Side:
                    await self._send_pending_motor_commands(
                        client,
                        side,
                    )


                    await  self._send_pending_motor_controls(
                        client,
                        side,
                    )

                    await  self._send_pending_transparent_commands(
                        client,
                        side,
                    )

                await self._send_pending_sd_commands(
                    client)


                await asyncio.sleep(0.01)


    def _run_bluetooth(self):
        try:
            asyncio.run(self._bluetooth_connection())
        except Exception as exc:
            print("Bluetooth error:",exc)
            self.stop_event.set()