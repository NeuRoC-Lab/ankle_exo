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

LOAD_CELL_UUID = "CA87289F-102B-4078-AD8C-8F53063547A6"
ENCODER_UUID = "094A717B-0C7F-4A23-BFD1-A4924E6E7DAB"
POWER_UUID = "4D92C3F7-848C-42C2-B26A-9D1D15CB361A"

RIGHT_MOTOR_FEEDBACK_UUID = "2E38C871-902C-425F-8D3B-181CB21F0B67"
LEFT_MOTOR_FEEDBACK_UUID = "81DC2896-1B27-4195-A391-99A637FA50A4"

RIGHT_MOTOR_COMMAND_UUID = "09DC04D0-BFC0-4D7C-A88D-96D60857FE64"
LEFT_MOTOR_COMMAND_UUID = "E0D883F6-705C-4A11-B117-E2B0909CC68E"

RIGHT_MOTOR_CONTROL_UUID = "12E89431-F2D4-4495-B984-A127A79D1591"
LEFT_MOTOR_CONTROL_UUID = "99F02A66-065C-41BE-B05E-4BE2B9035A8B"

SD_LOGGER_UUID = "A06EE428-0AB6-4CD4-AA8A-91619F1AF577"

LEFT_TRANSPARENT_CONTROLLER_UUID = "644CD587-A563-437E-8006-8B7F39559690"
RIGHT_TRANSPARENT_CONTROLLER_UUID = "A8B58E7F-4B57-4C5F-85E4-55F38A6CE271"

MOTOR_FEEDBACK_UUIDS = {
    Side.LEFT:
        LEFT_MOTOR_FEEDBACK_UUID,

    Side.RIGHT:
        RIGHT_MOTOR_FEEDBACK_UUID,
}


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

LEFT_INTERMEDIATE_TORQUE_UUID = (
    "B50F6E44-AB02-4C7A-A801-74A85815B001"
)

RIGHT_INTERMEDIATE_TORQUE_UUID = (
    "B50F6E44-AB02-4C7A-A801-74A85815B002"
)


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

    def _fetch_motor(self, sender, data, side: Side):

        values = struct.unpack("<fBBxx", data)

        if self.motors[side] is None:
            print("Motor is undefined")
            return

        self.motors[side]._update(*values)

    def _fetch_loadcells(self, sender, data):
        values = struct.unpack("<4f", data)
        if self.loadcells is not None:
            self.loadcells._update(*values)

    def _fetch_left_intermediate_torque(self,sender,data):

        value, = struct.unpack(
            "<f",
            data
        )

        self.intermediate_torque._update_left(
            value
    )


    def _fetch_right_intermediate_torque(
            self,
            sender,
            data):

        value, = struct.unpack(
            "<f",
            data
        )

        self.intermediate_torque._update_right(
            value
        )

    def _fetch_power(self, sender, data):
        values = struct.unpack("<3f", data)
        if self.power is None:
            print("power is undefined")
            return
        self.power._update(*values)

    def _fetch_encoders(self,sender,data):
        values = struct.unpack("<2f",data)
        if self.encoders is None:
            print("power is undefined")
            return
        self.encoders._update(*values)

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

    def _make_motor_callback(self,side: Side):

        def callback(
                sender,
                data,
        ):
            self._fetch_motor(
                sender,
                data,
                side,
            )

        return callback

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
            "write" in properties
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
                "write" in properties
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

            use_response = ("write" in properties)

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
                "write" in properties
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

                await client.start_notify(
                    ENCODER_UUID,
                    self._fetch_encoders,
                )

                await client.start_notify(
                    LOAD_CELL_UUID,
                    self._fetch_loadcells,
                )

                await client.start_notify(
                    POWER_UUID,
                    self._fetch_power,
                )

                await client.start_notify(
                    LEFT_INTERMEDIATE_TORQUE_UUID,
                    self._fetch_left_intermediate_torque,
                )

                await client.start_notify(
                    RIGHT_INTERMEDIATE_TORQUE_UUID,
                    self._fetch_right_intermediate_torque,
                )

                for side in Side:
                    await client.start_notify(
                        MOTOR_FEEDBACK_UUIDS[
                            side
                        ],
                        self._make_motor_callback(
                            side
                        ),
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