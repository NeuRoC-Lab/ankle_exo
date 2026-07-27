"""
New bluetooth plotting script test

The script:
1. Finds the AnkleExo peripheral.
2. Connects and discovers its GATT characteristics.
3. Reads each characteristic once.
4. Subscribes to motor, load-cell, and encoder notifications.
5. Decodes and prints every incoming notification.
"""

import asyncio
import logging
import struct
import time
from dataclasses import dataclass, field

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice


SERVICE_UUID = "CF45813E-4358-4903-B961-09996BB081FB"
DEVICE_NAME = "AnkleExo"

LOAD_CELL_UUID = "CA87289F-102B-4078-AD8C-8F53063547A6"
MOTOR_UUID = "E0D883F6-705C-4A11-B117-E2B0909CC68E"
ENCODER_UUID = "094A717B-0C7F-4A23-BFD1-A4924E6E7DAB"

SCAN_TIMEOUT_SECONDS = 10.0
STATUS_INTERVAL_SECONDS = 5.0

# Change this to logging.DEBUG to see Bleak/CoreBluetooth diagnostics.
LOG_LEVEL = logging.INFO


@dataclass
class NotificationStatistics:
    name: str
    count: int = 0
    malformed_count: int = 0
    first_timestamp: float | None = None
    last_timestamp: float | None = None
    intervals: list[float] = field(default_factory=list)

    def record(self) -> None:
        now = time.monotonic()

        if self.first_timestamp is None:
            self.first_timestamp = now

        if self.last_timestamp is not None:
            interval = now - self.last_timestamp
            self.intervals.append(interval)

            # Do not let this list grow forever.
            if len(self.intervals) > 100:
                self.intervals.pop(0)

        self.last_timestamp = now
        self.count += 1

    @property
    def average_frequency_hz(self) -> float:
        if not self.intervals:
            return 0.0

        average_interval = sum(self.intervals) / len(self.intervals)

        if average_interval <= 0.0:
            return 0.0

        return 1.0 / average_interval


motor_stats = NotificationStatistics("motor")
load_cell_stats = NotificationStatistics("load cells")
encoder_stats = NotificationStatistics("encoders")


async def find_nano() -> BLEDevice:
    service_uuid_lower = SERVICE_UUID.lower()

    print(f"Searching for advertised service {SERVICE_UUID}...")

    device = await BleakScanner.find_device_by_filter(
        lambda scanned_device, advertisement: (
                service_uuid_lower
                in [uuid.lower() for uuid in advertisement.service_uuids]
        ),
        timeout=SCAN_TIMEOUT_SECONDS,
    )

    if device is not None:
        print(
            f"Found by service UUID: "
            f"{device.name or '<unnamed>'} [{device.address}]"
        )
        return device

    print(
        "Could not find the Nano through its advertised service UUID. "
        "Trying its local name..."
    )

    await asyncio.sleep(1)

    device = await BleakScanner.find_device_by_filter(
        lambda scanned_device, advertisement: (
                scanned_device.name == DEVICE_NAME
                or advertisement.local_name == DEVICE_NAME
        ),
        timeout=SCAN_TIMEOUT_SECONDS,
    )

    if device is None:
        raise RuntimeError(
            f"Could not find BLE device named {DEVICE_NAME!r}. "
            "Check that the Nano is powered, advertising, and not "
            "already connected to another central."
        )

    print(
        f"Found by name: "
        f"{device.name or '<unnamed>'} [{device.address}]"
    )

    return device


def timestamp_string() -> str:
    now = time.time()
    local_time = time.localtime(now)
    milliseconds = int((now % 1.0) * 1000)

    return (
        f"{local_time.tm_hour:02d}:"
        f"{local_time.tm_min:02d}:"
        f"{local_time.tm_sec:02d}."
        f"{milliseconds:03d}"
    )


def decode_motor(data: bytearray) -> dict[str, int | float]:
    """
    Decode the naturally aligned 20-byte C++ MotorReply structure:

        uint8_t can_id;
        3 bytes padding;
        float position;
        float velocity;
        float torque;
        uint8_t temperature;
        uint8_t error;
        2 bytes trailing padding;
    """

    packet_format = "<B3x3fBB2x"
    expected_size = struct.calcsize(packet_format)

    if len(data) != expected_size:
        raise ValueError(
            f"motor packet is {len(data)} bytes; "
            f"expected {expected_size}"
        )

    can_id, position, velocity, torque, temperature, error = (
        struct.unpack(packet_format, data)
    )

    return {
        "can_id": can_id,
        "position": position,
        "velocity": velocity,
        "torque": torque,
        "temperature": temperature,
        "error": error,
    }


def decode_load_cells(data: bytearray) -> dict[str, float]:
    packet_format = "<4f"
    expected_size = struct.calcsize(packet_format)

    if len(data) != expected_size:
        raise ValueError(
            f"load-cell packet is {len(data)} bytes; "
            f"expected {expected_size}"
        )

    left_1, left_2, right_1, right_2 = struct.unpack(
        packet_format,
        data,
    )

    return {
        "left_1": left_1,
        "left_2": left_2,
        "right_1": right_1,
        "right_2": right_2,
    }


def decode_encoders(data: bytearray) -> dict[str, int]:
    packet_format = "<2H"
    expected_size = struct.calcsize(packet_format)

    if len(data) != expected_size:
        raise ValueError(
            f"encoder packet is {len(data)} bytes; "
            f"expected {expected_size}"
        )

    left_position, right_position = struct.unpack(
        packet_format,
        data,
    )

    return {
        "left_position": left_position,
        "right_position": right_position,
    }


def motor_notification(
        characteristic: BleakGATTCharacteristic,
        data: bytearray,
) -> None:
    motor_stats.record()

    try:
        values = decode_motor(data)

        print(
            f"[{timestamp_string()}] "
            f"MOTOR #{motor_stats.count}: "
            f"ID={values['can_id']}, "
            f"position={values['position']:.4f}, "
            f"velocity={values['velocity']:.4f}, "
            f"torque={values['torque']:.4f}, "
            f"temperature={values['temperature']}, "
            f"error={values['error']}"
        )

    except (ValueError, struct.error) as error:
        motor_stats.malformed_count += 1

        print(
            f"[{timestamp_string()}] "
            f"MOTOR DECODE ERROR: {error}; "
            f"raw={data.hex()}"
        )


def load_cell_notification(
        characteristic: BleakGATTCharacteristic,
        data: bytearray,
) -> None:
    load_cell_stats.record()

    try:
        values = decode_load_cells(data)

        print(
            f"[{timestamp_string()}] "
            f"LOAD CELLS #{load_cell_stats.count}: "
            f"L1={values['left_1']:.6f}, "
            f"L2={values['left_2']:.6f}, "
            f"R1={values['right_1']:.6f}, "
            f"R2={values['right_2']:.6f}"
        )

    except (ValueError, struct.error) as error:
        load_cell_stats.malformed_count += 1

        print(
            f"[{timestamp_string()}] "
            f"LOAD-CELL DECODE ERROR: {error}; "
            f"raw={data.hex()}"
        )


def encoder_notification(
        characteristic: BleakGATTCharacteristic,
        data: bytearray,
) -> None:
    encoder_stats.record()

    try:
        values = decode_encoders(data)

        print(
            f"[{timestamp_string()}] "
            f"ENCODERS #{encoder_stats.count}: "
            f"left={values['left_position']}, "
            f"right={values['right_position']}"
        )

    except (ValueError, struct.error) as error:
        encoder_stats.malformed_count += 1

        print(
            f"[{timestamp_string()}] "
            f"ENCODER DECODE ERROR: {error}; "
            f"raw={data.hex()}"
        )


def find_characteristic(
        client: BleakClient,
        uuid: str,
) -> BleakGATTCharacteristic:
    characteristic = client.services.get_characteristic(uuid)

    if characteristic is None:
        raise RuntimeError(
            f"Characteristic {uuid} was not found on the peripheral."
        )

    return characteristic


def check_notify_support(
        name: str,
        characteristic: BleakGATTCharacteristic,
) -> None:
    properties = {
        property_name.lower()
        for property_name in characteristic.properties
    }

    if "notify" not in properties and "indicate" not in properties:
        raise RuntimeError(
            f"{name} characteristic does not support notifications. "
            f"Properties reported by the Nano: "
            f"{characteristic.properties}"
        )


async def print_periodic_status(
        client: BleakClient,
) -> None:
    while client.is_connected:
        await asyncio.sleep(STATUS_INTERVAL_SECONDS)

        print("\n--- Notification statistics ---")

        for stats in (
                motor_stats,
                load_cell_stats,
                encoder_stats,
        ):
            print(
                f"{stats.name}: "
                f"{stats.count} packets, "
                f"{stats.average_frequency_hz:.2f} Hz average, "
                f"{stats.malformed_count} malformed"
            )

        print("-------------------------------\n")


async def main() -> None:
    device = await find_nano()

    def disconnected_callback(client: BleakClient) -> None:
        print("\nBLE peripheral disconnected")

    async with BleakClient(
            device,
            disconnected_callback=disconnected_callback,
    ) as client:
        print(f"Connected: {client.is_connected}")

        print("\nDiscovered GATT services:")

        for service in client.services:
            print(f"Service: {service.uuid}")

            for characteristic in service.characteristics:
                print(
                    f"  Characteristic: {characteristic.uuid} "
                    f"handle={characteristic.handle} "
                    f"properties={characteristic.properties}"
                )

        motor_characteristic = find_characteristic(
            client,
            MOTOR_UUID,
        )

        load_cell_characteristic = find_characteristic(
            client,
            LOAD_CELL_UUID,
        )

        encoder_characteristic = find_characteristic(
            client,
            ENCODER_UUID,
        )

        check_notify_support(
            "Motor",
            motor_characteristic,
        )

        check_notify_support(
            "Load-cell",
            load_cell_characteristic,
        )

        check_notify_support(
            "Encoder",
            encoder_characteristic,
        )

        print("\nReading each characteristic once:")

        motor_data = await client.read_gatt_char(
            motor_characteristic
        )
        print("Initial motor value:", decode_motor(motor_data))

        load_cell_data = await client.read_gatt_char(
            load_cell_characteristic
        )
        print(
            "Initial load-cell value:",
            decode_load_cells(load_cell_data),
        )

        encoder_data = await client.read_gatt_char(
            encoder_characteristic
        )
        print(
            "Initial encoder value:",
            decode_encoders(encoder_data),
        )

        print("\nSubscribing to notifications...")

        await client.start_notify(
            motor_characteristic,
            motor_notification,
        )

        await client.start_notify(
            load_cell_characteristic,
            load_cell_notification,
        )

        await client.start_notify(
            encoder_characteristic,
            encoder_notification,
        )

        print("Subscribed to motor notifications")
        print("Subscribed to load-cell notifications")
        print("Subscribed to encoder notifications")
        print("\nWaiting for data. Press Ctrl+C to stop.\n")

        status_task = asyncio.create_task(
            print_periodic_status(client)
        )

        try:
            while client.is_connected:
                await asyncio.sleep(1)

        finally:
            status_task.cancel()

            try:
                await status_task
            except asyncio.CancelledError:
                pass

            if client.is_connected:
                print("\nStopping notifications...")

                await client.stop_notify(
                    motor_characteristic
                )
                await client.stop_notify(
                    load_cell_characteristic
                )
                await client.stop_notify(
                    encoder_characteristic
                )


if __name__ == "__main__":
    logging.basicConfig(
        level=LOG_LEVEL,
        format=(
            "%(asctime)s "
            "%(name)s "
            "%(levelname)s: "
            "%(message)s"
        ),
    )

    try:
        asyncio.run(main())

    except KeyboardInterrupt:
        print("\nStopped by user")

    except Exception as error:
        logging.exception("BLE subscriber test failed")
        print(f"\nBLE error: {error}")