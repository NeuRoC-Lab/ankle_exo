"""
Real-time BLE telemetry plotting for the ankle exoskeleton.

Architecture:
- Bleak/asyncio runs in a background thread.
- BLE callbacks decode notifications and update shared telemetry.
- Matplotlib runs in the main thread.
- The latest telemetry values are periodically copied to the plots and CSV.
"""

import asyncio
import csv
import logging
import os
import struct
import threading
import time
from dataclasses import dataclass, field

import matplotlib.pyplot as plt
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice


# =============================================================================
# BLE configuration
# =============================================================================

SERVICE_UUID = "CF45813E-4358-4903-B961-09996BB081FB"
DEVICE_NAME = "AnkleExo"

LOAD_CELL_UUID = "CA87289F-102B-4078-AD8C-8F53063547A6"
MOTOR_UUID = "E0D883F6-705C-4A11-B117-E2B0909CC68E"
ENCODER_UUID = "094A717B-0C7F-4A23-BFD1-A4924E6E7DAB"

SCAN_TIMEOUT_SECONDS = 10.0

LOG_LEVEL = logging.INFO


# =============================================================================
# Plot configuration
# =============================================================================

TIME_WINDOW_SECONDS = 10.0
PLOT_UPDATE_INTERVAL_SECONDS = 0.05
CSV_FILENAME = "SingleLegBLEData.csv"

# Set this to your encoder resolution if you want degree conversion.
ENCODER_COUNTS_PER_REVOLUTION = 4096.0


# =============================================================================
# Shared telemetry
# =============================================================================

@dataclass
class TelemetryState:
    left_load_cell_1: float = 0.0
    left_load_cell_2: float = 0.0
    right_load_cell_1: float = 0.0
    right_load_cell_2: float = 0.0

    left_encoder: int = 0
    right_encoder: int = 0

    motor_can_id: int = 0
    motor_position: float = 0.0
    motor_velocity: float = 0.0
    motor_torque: float = 0.0
    motor_temperature: int = 0
    motor_error: int = 0

    motor_packets: int = 0
    load_cell_packets: int = 0
    encoder_packets: int = 0

    last_update_time: float = 0.0

    lock: threading.Lock = field(
        default_factory=threading.Lock,
        repr=False,
    )


telemetry = TelemetryState()

# Signals that all activity should stop.
stop_event = threading.Event()

# Set when the BLE client connects.
connected_event = threading.Event()


# =============================================================================
# Packet decoding
# =============================================================================

def decode_motor(data: bytearray) -> tuple:
    """
    Decode the naturally aligned C++ MotorReply structure.

    Expected C++ structure:

        uint8_t can_id;
        float position;
        float velocity;
        float torque;
        uint8_t temperature;
        uint8_t error;

    With normal ARM alignment this is expected to occupy 20 bytes.
    """

    packet_format = "<B3x3fBB2x"
    expected_size = struct.calcsize(packet_format)

    if len(data) != expected_size:
        raise ValueError(
            f"Motor packet has {len(data)} bytes; "
            f"expected {expected_size}. Raw={data.hex()}"
        )

    return struct.unpack(packet_format, data)


def decode_load_cells(data: bytearray) -> tuple:
    packet_format = "<4f"
    expected_size = struct.calcsize(packet_format)

    if len(data) != expected_size:
        raise ValueError(
            f"Load-cell packet has {len(data)} bytes; "
            f"expected {expected_size}. Raw={data.hex()}"
        )

    return struct.unpack(packet_format, data)


def decode_encoders(data: bytearray) -> tuple:
    packet_format = "<2H"
    expected_size = struct.calcsize(packet_format)

    if len(data) != expected_size:
        raise ValueError(
            f"Encoder packet has {len(data)} bytes; "
            f"expected {expected_size}. Raw={data.hex()}"
        )

    return struct.unpack(packet_format, data)


# =============================================================================
# BLE notification callbacks
# =============================================================================

def motor_notification(
        characteristic: BleakGATTCharacteristic,
        data: bytearray,
) -> None:
    try:
        (
            can_id,
            position,
            velocity,
            torque,
            temperature,
            error,
        ) = decode_motor(data)

        with telemetry.lock:
            telemetry.motor_can_id = can_id
            telemetry.motor_position = position
            telemetry.motor_velocity = velocity
            telemetry.motor_torque = torque
            telemetry.motor_temperature = temperature
            telemetry.motor_error = error

            telemetry.motor_packets += 1
            telemetry.last_update_time = time.perf_counter()

    except (ValueError, struct.error) as error:
        print(f"Motor notification error: {error}")


def load_cell_notification(
        characteristic: BleakGATTCharacteristic,
        data: bytearray,
) -> None:
    try:
        left_1, left_2, right_1, right_2 = decode_load_cells(data)

        with telemetry.lock:
            telemetry.left_load_cell_1 = left_1
            telemetry.left_load_cell_2 = left_2
            telemetry.right_load_cell_1 = right_1
            telemetry.right_load_cell_2 = right_2

            telemetry.load_cell_packets += 1
            telemetry.last_update_time = time.perf_counter()

    except (ValueError, struct.error) as error:
        print(f"Load-cell notification error: {error}")


def encoder_notification(
        characteristic: BleakGATTCharacteristic,
        data: bytearray,
) -> None:
    try:
        left_position, right_position = decode_encoders(data)

        with telemetry.lock:
            telemetry.left_encoder = left_position
            telemetry.right_encoder = right_position

            telemetry.encoder_packets += 1
            telemetry.last_update_time = time.perf_counter()

    except (ValueError, struct.error) as error:
        print(f"Encoder notification error: {error}")


# =============================================================================
# BLE handling
# =============================================================================

async def find_nano() -> BLEDevice:
    target_service = SERVICE_UUID.lower()

    print(f"Searching for BLE service {SERVICE_UUID}...")

    device = await BleakScanner.find_device_by_filter(
        lambda scanned_device, advertisement: (
                target_service
                in [
                    uuid.lower()
                    for uuid in advertisement.service_uuids
                ]
        ),
        timeout=SCAN_TIMEOUT_SECONDS,
    )

    if device is not None:
        print(
            f"Found by service UUID: "
            f"{device.name or '<unnamed>'} [{device.address}]"
        )
        return device

    print("Service UUID search failed. Searching by local name...")

    device = await BleakScanner.find_device_by_filter(
        lambda scanned_device, advertisement: (
                scanned_device.name == DEVICE_NAME
                or advertisement.local_name == DEVICE_NAME
        ),
        timeout=SCAN_TIMEOUT_SECONDS,
    )

    if device is None:
        raise RuntimeError(
            f"Could not find BLE device {DEVICE_NAME!r}."
        )

    print(
        f"Found by name: "
        f"{device.name or '<unnamed>'} [{device.address}]"
    )

    return device


def require_characteristic(
        client: BleakClient,
        uuid: str,
) -> BleakGATTCharacteristic:
    characteristic = client.services.get_characteristic(uuid)

    if characteristic is None:
        raise RuntimeError(
            f"Characteristic {uuid} was not found."
        )

    properties = {
        property_name.lower()
        for property_name in characteristic.properties
    }

    if "notify" not in properties and "indicate" not in properties:
        raise RuntimeError(
            f"Characteristic {uuid} does not support notifications. "
            f"Properties: {characteristic.properties}"
        )

    return characteristic


async def ble_worker() -> None:
    try:
        device = await find_nano()

        def disconnected_callback(client: BleakClient) -> None:
            print("\nNano disconnected")
            connected_event.clear()
            stop_event.set()

        async with BleakClient(
                device,
                disconnected_callback=disconnected_callback,
        ) as client:
            print(f"Connected: {client.is_connected}")
            connected_event.set()

            motor_characteristic = require_characteristic(
                client,
                MOTOR_UUID,
            )

            load_cell_characteristic = require_characteristic(
                client,
                LOAD_CELL_UUID,
            )

            encoder_characteristic = require_characteristic(
                client,
                ENCODER_UUID,
            )

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

            print("Subscribed to all telemetry characteristics")

            while (
                    client.is_connected
                    and not stop_event.is_set()
            ):
                await asyncio.sleep(0.1)

            if client.is_connected:
                await client.stop_notify(motor_characteristic)
                await client.stop_notify(load_cell_characteristic)
                await client.stop_notify(encoder_characteristic)

    except Exception:
        logging.exception("BLE worker failed")
        connected_event.clear()
        stop_event.set()


def run_ble_thread() -> None:
    asyncio.run(ble_worker())


# =============================================================================
# Telemetry snapshot
# =============================================================================

def copy_telemetry() -> dict:
    """
    Copy the shared state while holding the lock briefly.

    The plotting code then works on the copied dictionary without preventing
    BLE callbacks from updating the original telemetry object.
    """

    with telemetry.lock:
        return {
            "left_load_cell_1": telemetry.left_load_cell_1,
            "left_load_cell_2": telemetry.left_load_cell_2,
            "right_load_cell_1": telemetry.right_load_cell_1,
            "right_load_cell_2": telemetry.right_load_cell_2,

            "left_encoder": telemetry.left_encoder,
            "right_encoder": telemetry.right_encoder,

            "motor_can_id": telemetry.motor_can_id,
            "motor_position": telemetry.motor_position,
            "motor_velocity": telemetry.motor_velocity,
            "motor_torque": telemetry.motor_torque,
            "motor_temperature": telemetry.motor_temperature,
            "motor_error": telemetry.motor_error,

            "motor_packets": telemetry.motor_packets,
            "load_cell_packets": telemetry.load_cell_packets,
            "encoder_packets": telemetry.encoder_packets,
        }


# =============================================================================
# Plot utilities
# =============================================================================

def encoder_count_to_degrees(count: int) -> float:
    return (
            float(count)
            * 360.0
            / ENCODER_COUNTS_PER_REVOLUTION
    )


def trim_history(
        current_time: float,
        time_data: list[float],
        series: list[list[float]],
) -> None:
    while (
            time_data
            and current_time - time_data[0] > TIME_WINDOW_SECONDS
    ):
        time_data.pop(0)

        for values in series:
            values.pop(0)


def update_axis(
        axis,
        line,
        time_data: list[float],
        y_data: list[float],
        current_time: float,
) -> None:
    line.set_data(time_data, y_data)

    axis.set_xlim(
        max(0.0, current_time - TIME_WINDOW_SECONDS),
        max(TIME_WINDOW_SECONDS, current_time),
    )

    axis.relim()
    axis.autoscale_view(
        scalex=False,
        scaley=True,
    )


# =============================================================================
# Main plotting program
# =============================================================================

def main() -> None:
    ble_thread = threading.Thread(
        target=run_ble_thread,
        name="BLEThread",
        daemon=True,
    )

    ble_thread.start()

    print("Waiting for BLE connection...")

    while (
            not connected_event.is_set()
            and not stop_event.is_set()
    ):
        time.sleep(0.05)

    if stop_event.is_set():
        raise RuntimeError(
            "BLE connection could not be established."
        )

    print("BLE connected. Starting plots.")

    output_path = os.path.abspath(CSV_FILENAME)
    print(f"Writing telemetry to: {output_path}")

    time_data = []

    left_encoder_data = []
    right_encoder_data = []

    load_cell_1_data = []
    load_cell_2_data = []

    motor_position_data = []
    motor_velocity_data = []

    all_series = [
        left_encoder_data,
        right_encoder_data,
        load_cell_1_data,
        load_cell_2_data,
        motor_position_data,
        motor_velocity_data,
    ]

    plt.ion()

    fig, axes = plt.subplots(
        ncols=2,
        nrows=3,
        figsize=(12, 8),
    )

    ax1 = axes[0, 0]
    ax2 = axes[0, 1]
    ax3 = axes[1, 0]
    ax4 = axes[1, 1]
    ax5 = axes[2, 0]
    ax6 = axes[2, 1]

    fig.suptitle(
        "Real-Time BLE Data — One-Leg Ankle Exoskeleton",
        fontsize=14,
    )

    line1, = ax1.plot([], [], linewidth=1.5)
    line2, = ax2.plot([], [], linewidth=1.5)
    line3, = ax3.plot([], [], linewidth=1.5)
    line4, = ax4.plot([], [], linewidth=1.5)
    line5, = ax5.plot([], [], linewidth=1.5)
    line6, = ax6.plot([], [], linewidth=1.5)

    ax1.set_title("Left Encoder Position")
    ax2.set_title("Right Encoder Position")
    ax3.set_title("Left Load Cell 1")
    ax4.set_title("Left Load Cell 2")
    ax5.set_title("Motor Position")
    ax6.set_title("Motor Velocity")

    ax1.set_ylabel("Position (deg)")
    ax2.set_ylabel("Position (deg)")
    ax3.set_ylabel("Voltage (V)")
    ax4.set_ylabel("Voltage (V)")
    ax5.set_ylabel("Position (rad)")
    ax6.set_ylabel("Velocity (rad/s)")

    for axis in (
            ax1,
            ax2,
            ax3,
            ax4,
            ax5,
            ax6,
    ):
        axis.set_xlabel("Time (s)")
        axis.grid(True)

    def close_plot(event) -> None:
        stop_event.set()

    fig.canvas.mpl_connect(
        "close_event",
        close_plot,
    )

    plt.tight_layout()
    plt.show(block=False)

    start_time = time.perf_counter()
    last_packet_counts = (-1, -1, -1)

    with open(
            output_path,
            "w",
            newline="",
    ) as csv_file:
        writer = csv.writer(csv_file)

        writer.writerow([
            "Time (s)",
            "Left Encoder Raw",
            "Right Encoder Raw",
            "Left Encoder (deg)",
            "Right Encoder (deg)",
            "Left Load Cell 1 (V)",
            "Left Load Cell 2 (V)",
            "Right Load Cell 1 (V)",
            "Right Load Cell 2 (V)",
            "Motor CAN ID",
            "Motor Position (rad)",
            "Motor Velocity (rad/s)",
            "Motor Torque (Nm)",
            "Motor Temperature (C)",
            "Motor Error",
        ])

        try:
            while (
                    not stop_event.is_set()
                    and plt.fignum_exists(fig.number)
            ):
                snapshot = copy_telemetry()

                packet_counts = (
                    snapshot["motor_packets"],
                    snapshot["load_cell_packets"],
                    snapshot["encoder_packets"],
                )

                # Do not append duplicate rows when no new BLE data arrived.
                if packet_counts == last_packet_counts:
                    plt.pause(PLOT_UPDATE_INTERVAL_SECONDS)
                    continue

                last_packet_counts = packet_counts

                current_time = (
                        time.perf_counter()
                        - start_time
                )

                left_encoder_degrees = encoder_count_to_degrees(
                    snapshot["left_encoder"]
                )

                right_encoder_degrees = encoder_count_to_degrees(
                    snapshot["right_encoder"]
                )

                writer.writerow([
                    current_time,
                    snapshot["left_encoder"],
                    snapshot["right_encoder"],
                    left_encoder_degrees,
                    right_encoder_degrees,
                    snapshot["left_load_cell_1"],
                    snapshot["left_load_cell_2"],
                    snapshot["right_load_cell_1"],
                    snapshot["right_load_cell_2"],
                    snapshot["motor_can_id"],
                    snapshot["motor_position"],
                    snapshot["motor_velocity"],
                    snapshot["motor_torque"],
                    snapshot["motor_temperature"],
                    snapshot["motor_error"],
                ])

                # Flush occasionally during testing so data survives a crash.
                csv_file.flush()

                time_data.append(current_time)

                left_encoder_data.append(
                    left_encoder_degrees
                )

                right_encoder_data.append(
                    right_encoder_degrees
                )

                load_cell_1_data.append(
                    snapshot["left_load_cell_1"]
                )

                load_cell_2_data.append(
                    snapshot["left_load_cell_2"]
                )

                motor_position_data.append(
                    snapshot["motor_position"]
                )

                motor_velocity_data.append(
                    snapshot["motor_velocity"]
                )

                trim_history(
                    current_time,
                    time_data,
                    all_series,
                )

                update_axis(
                    ax1,
                    line1,
                    time_data,
                    left_encoder_data,
                    current_time,
                )

                update_axis(
                    ax2,
                    line2,
                    time_data,
                    right_encoder_data,
                    current_time,
                )

                update_axis(
                    ax3,
                    line3,
                    time_data,
                    load_cell_1_data,
                    current_time,
                )

                update_axis(
                    ax4,
                    line4,
                    time_data,
                    load_cell_2_data,
                    current_time,
                )

                update_axis(
                    ax5,
                    line5,
                    time_data,
                    motor_position_data,
                    current_time,
                )

                update_axis(
                    ax6,
                    line6,
                    time_data,
                    motor_velocity_data,
                    current_time,
                )

                fig.canvas.draw_idle()
                plt.pause(PLOT_UPDATE_INTERVAL_SECONDS)

        except KeyboardInterrupt:
            print("\nCtrl+C received")

        finally:
            stop_event.set()
            plt.close("all")

            ble_thread.join(timeout=2.0)

            print("BLE plotting and logging stopped")


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
        main()

    except Exception:
        stop_event.set()
        logging.exception("Program failed")