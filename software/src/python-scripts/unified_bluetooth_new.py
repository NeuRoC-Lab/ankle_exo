"""
This code plots the data for the encoder, loadcells, and motor (MIT mode)
integrated together on the one leg setup.

Data is read through Bluetooth from Arduino Nano.

Features:
- Real-time 6 graph plotting
- Encoder zeroing on first valid reading
- Encoder conversion: count -> degrees
- Fixed-size buffers to prevent lag
- BLE reading separated from plotting
- No duplicate plotting when BLE data is unchanged

Run:
python software/src/python-scripts/unified_bluetooth_new.py
"""


import asyncio
import threading
import time
import csv
import os
import struct
import queue

import matplotlib
matplotlib.use("QtAgg")

import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button
from collections import deque
from dataclasses import dataclass, field
from bleak import BleakClient, BleakScanner


MOTOR_ID = 0x02

from enum import IntEnum


class MotorCommandType(IntEnum):
    START = 0
    STOP = 1
    ZERO = 2
    SET = 3

COMMAND_PACKET_FORMAT = "<BB2x5f"
COMMAND_PACKET_SIZE = struct.calcsize(COMMAND_PACKET_FORMAT)

print("Command packet size:", COMMAND_PACKET_SIZE)

@dataclass
class MotorCommand:
    command_type: MotorCommandType
    motor_id: int = 0

    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    kp: float = 0.0
    kd: float = 0.0

    def to_bytes(self) -> bytes:
        if not 0 <= self.motor_id <= 255:
            raise ValueError("motor_id must be between 0 and 255")

        return struct.pack(
            COMMAND_PACKET_FORMAT,
            int(self.command_type),
            self.motor_id,
            self.position,
            self.velocity,
            self.torque,
            self.kp,
            self.kd,
        )

# Bluetooth configuration

DEVICE_NAME = "AnkleExo"
SERVICE_UUID = "CF45813E-4358-4903-B961-09996BB081FB"

LOAD_CELL_UUID = "CA87289F-102B-4078-AD8C-8F53063547A6"
MOTOR_UUID = "E0D883F6-705C-4A11-B117-E2B0909CC68E"
ENCODER_UUID = "094A717B-0C7F-4A23-BFD1-A4924E6E7DAB"
COMMAND_UUID = "C94B7403-6BFB-4A06-BA12-6394765C328E"


# Plot configuration

time_window = 10
MAX_POINTS = 500

# Same visual refresh rate as the working UART code:
# 0.1 s = approximately 10 plot updates per second.
PLOT_INTERVAL = 0.1

encoder_max_count = 4096 # 12 bits resolution
encoder_half_count = encoder_max_count // 2

first_encoder_value = None


running = {"in_progress": True}
plotting = {"paused": False}


# Latest Bluetooth values

@dataclass
class Telemetry:

    motor_position: float = 0.0
    motor_velocity: float = 0.0
    motor_torque: float = 0.0
    motor_temperature: int = 0

    loadcell1: float = 0.0
    loadcell2: float = 0.0

    encoder: int = 0

    motor_packets: int = 0
    loadcell_packets: int = 0
    encoder_packets: int = 0

    lock: threading.Lock = field(
        default_factory=threading.Lock,
        repr=False
    )


telemetry = Telemetry()


# Queue containing every BLE update.
# CSV logging drains this queue independently of plot refresh rate.

ble_data_queue = queue.Queue()

# Motor commands entered in the plot window are placed here.
# The BLE thread sends them without blocking Matplotlib.
command_queue = queue.Queue()

# The writable BLE command characteristic is detected after connecting.
command_characteristic_uuid = {"uuid": None}


# Program time reference.
# This is defined before the Bluetooth thread starts.

start_time = time.perf_counter()


# Plot data buffers

time_data = deque(maxlen=MAX_POINTS)

ankle_pos_data = deque(maxlen=MAX_POINTS)
ankle_vel_data = deque(maxlen=MAX_POINTS)

loadcell1_data = deque(maxlen=MAX_POINTS)
loadcell2_data = deque(maxlen=MAX_POINTS)

motor_pos_data = deque(maxlen=MAX_POINTS)
motor_vel_data = deque(maxlen=MAX_POINTS)


# Encoder conversion

def count_to_deg_encoder(encoder):

    global first_encoder_value

    if first_encoder_value is None:
        first_encoder_value = encoder

    ankle_angle = (
        (encoder - first_encoder_value)
        % encoder_max_count
    )

    if ankle_angle >= encoder_half_count:
        ankle_angle -= encoder_max_count

    ankle_angle = (
        ankle_angle
        * 360.0
        / encoder_max_count
    )

    return ankle_angle


# Queue latest telemetry snapshot

def queue_telemetry_snapshot():

    """
    Add the latest complete telemetry state to the queue.

    The BLE callbacks call this after updating their own sensor values.
    CSV logging can therefore run independently of the plot refresh rate.
    """

    ble_data_queue.put(
        (
            time.perf_counter(),

            telemetry.encoder,

            telemetry.loadcell1,
            telemetry.loadcell2,

            telemetry.motor_position,
            telemetry.motor_velocity,
            telemetry.motor_torque,
            telemetry.motor_temperature,

            telemetry.motor_packets,
            telemetry.loadcell_packets,
            telemetry.encoder_packets,
        )
    )


# BLE packet decoding

def motor_callback(sender, data):

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


        with telemetry.lock:

            telemetry.motor_position = position
            telemetry.motor_velocity = velocity
            telemetry.motor_torque = torque
            telemetry.motor_temperature = temperature

            telemetry.motor_packets += 1

            queue_telemetry_snapshot()


    except Exception as e:
        print("Motor decode error:", e)


def loadcell_callback(sender, data):

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


        with telemetry.lock:

            telemetry.loadcell1 = left1
            telemetry.loadcell2 = right2

            telemetry.loadcell_packets += 1

            queue_telemetry_snapshot()


    except Exception as e:

        print("Load cell decode error:", e)


def encoder_callback(sender, data):

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


        with telemetry.lock:

            telemetry.encoder = left
            telemetry.encoder_packets += 1

            queue_telemetry_snapshot()


    except Exception as e:
        print("Encoder decode error:", e)



# Bluetooth motor command sending

def queue_motor_command(command: MotorCommand) -> None:
    command_queue.put(command)


def find_command_characteristic(client):

    """
    Find a writable GATT characteristic in the AnkleExo service.

    Telemetry characteristics are excluded first. If MOTOR_UUID itself is
    writable, it is accepted as a fallback.

    The Arduino Nano firmware still needs to expose a writable BLE
    characteristic and interpret the received command text.
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


async def send_pending_commands(client: BleakClient) -> None:
    """
    Send all queued binary CommandPayload packets.
    """

    command_uuid = command_characteristic_uuid["uuid"]

    if command_uuid is None:
        return

    characteristic = client.services.get_characteristic(command_uuid)

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
            command = command_queue.get_nowait()
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


def parse_motor_command(text: str) -> MotorCommand:
    """
    Accepted commands:

        start
        start 2

        stop
        stop 2

        zero
        zero 2

        set id 2 pos 0 vel 1 torque 0 kp 0 kd 0.15

    Short aliases:
        trq -> torque
    """

    tokens = text.strip().lower().split()

    if not tokens:
        raise ValueError("Command is empty")

    command_name = tokens[0]

    if command_name in {"start", "stop", "zero"}:
        motor_id = int(tokens[1], 0) if len(tokens) >= 2 else 0x02

        command_types = {
            "start": MotorCommandType.START,
            "stop": MotorCommandType.STOP,
            "zero": MotorCommandType.ZERO,
        }

        return MotorCommand(
            command_type=command_types[command_name],
            motor_id=motor_id,
        )

    if command_name != "set":
        raise ValueError(
            "Expected start, stop, zero, or set"
        )

    values = {
        "id": 0x02,
        "pos": 0.0,
        "vel": 0.0,
        "torque": 0.0,
        "kp": 0.0,
        "kd": 0.0,
    }

    aliases = {
        "trq": "torque",
        "position": "pos",
        "velocity": "vel",
    }

    index = 1

    while index < len(tokens):
        if index + 1 >= len(tokens):
            raise ValueError(
                f"Missing value after '{tokens[index]}'"
            )

        key = aliases.get(tokens[index], tokens[index])
        value_text = tokens[index + 1]

        if key not in values:
            raise ValueError(
                f"Unknown field '{tokens[index]}'"
            )

        if key == "id":
            values[key] = int(value_text, 0)
        else:
            values[key] = float(value_text)

        index += 2

    return MotorCommand(
        command_type=MotorCommandType.SET,
        motor_id=int(values["id"]),
        position=float(values["pos"]),
        velocity=float(values["vel"]),
        torque=float(values["torque"]),
        kp=float(values["kp"]),
        kd=float(values["kd"]),
    )


# Bluetooth connection

async def bluetooth_connection():

    print("Searching for Bluetooth device... (15s timeout)")

    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15.0)

    if device is None:
        print("Bluetooth device not found")
        running["in_progress"] = False
        return

    async with BleakClient(device) as client:
        print("Connected to Bluetooth")

        await client.start_notify(MOTOR_UUID, motor_callback)
        await client.start_notify(ENCODER_UUID, encoder_callback)
        await client.start_notify(LOAD_CELL_UUID, loadcell_callback)

        command_characteristic_uuid["uuid"] = (
            find_command_characteristic(client)
        )

        if command_characteristic_uuid["uuid"] is None:
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
                command_characteristic_uuid["uuid"]
            )

        print("Receiving Bluetooth data...")

        while running["in_progress"]:

            await send_pending_commands(client)

            # Small sleep keeps command latency low while allowing
            # BLE notification callbacks to run normally.
            await asyncio.sleep(0.01)


def start_bluetooth():
    asyncio.run(bluetooth_connection())


# Start BLE in background thread

bluetooth_thread = threading.Thread(
    target=start_bluetooth,
    daemon=True
)

bluetooth_thread.start()


# CSV setup

filename = "../../SingleLegData_BLE.csv"
path = os.path.abspath(filename)

print("CSV will save at:", path)


csv_file = open(path, "w", newline="")
writer = csv.writer(csv_file)

writer.writerow(
    [
        "Time (s)",
        "Ankle Position (deg)",
        "Ankle Velocity (deg/s)",
        "Cable 1 Tension",
        "Cable 2 Tension",
        "Motor Position (rad)",
        "Motor Velocity (rad/s)",
        "Motor Torque (Nm)",
        "Motor Temperature (C)",
    ]
)


# Plot setup

plt.ion()
print("Figure created")

width = 12
height = 7


fig, axes = plt.subplots(
    ncols=2,
    nrows=3,
    figsize=(width, height)
)


ax1 = axes[0, 0]
ax2 = axes[0, 1]
ax3 = axes[1, 0]
ax4 = axes[1, 1]
ax5 = axes[2, 0]
ax6 = axes[2, 1]

"""
Create plots in the format:

    +---------+---------+
    |   ax1   |   ax2   |
    +---------+---------+
    |   ax3   |   ax4   |
    +---------+---------+
    |   ax5   |   ax6   |
    +---------+---------+

"""

fig.suptitle(
    "Real Time Data Of One Ankle Exoskeleton",
    fontsize=14
)

# Reserve room at the bottom for motor controls.
fig.subplots_adjust(
    bottom=0.18,
    hspace=0.55
)

line1, = ax1.plot(
    [],
    [],
    linewidth=1.5,
    color="red"
)


line2, = ax2.plot(
    [],
    [],
    linewidth=1.5,
    color="green"
)


line3, = ax3.plot(
    [],
    [],
    linewidth=1.5,
    color="orange"
)


line4, = ax4.plot(
    [],
    [],
    linewidth=1.5,
    color="blue"
)


line5, = ax5.plot(
    [],
    [],
    linewidth=1.5,
    color="yellow"
)


line6, = ax6.plot(
    [],
    [],
    linewidth=1.5,
    color="purple"
)


# Figure titles

ax1.set_title("Ankle Relative Position Over Time")
ax2.set_title("Ankle Velocity Over Time")
ax3.set_title("Cable 1 Tension Over Time")
ax4.set_title("Cable 2 Tension Over Time")
ax5.set_title("Motor Position Over Time")
ax6.set_title("Motor Velocity Over Time")


# x-labels

ax1.set_xlabel("Time (s)")
ax2.set_xlabel("Time (s)")
ax3.set_xlabel("Time (s)")
ax4.set_xlabel("Time (s)")
ax5.set_xlabel("Time (s)")
ax6.set_xlabel("Time (s)")


# y-labels

ax1.set_ylabel("Ankle Angle (deg)")
ax2.set_ylabel("Ankle Velocity (deg/s)")
ax3.set_ylabel("Cable 1 Tension")
ax4.set_ylabel("Cable 2 Tension")
ax5.set_ylabel("Motor Position (rad)")
ax6.set_ylabel("Motor Velocity (rad/s)")


# Grid

ax1.grid(True)
ax2.grid(True)
ax3.grid(True)
ax4.grid(True)
ax5.grid(True)
ax6.grid(True)


axes_list = [
    ax1,
    ax2,
    ax3,
    ax4,
    ax5,
    ax6
]



# Motor control widgets

command_axis = fig.add_axes(
    [0.10, 0.055, 0.55, 0.045]
)

send_axis = fig.add_axes(
    [0.67, 0.055, 0.09, 0.045]
)

start_axis = fig.add_axes(
    [0.78, 0.055, 0.08, 0.045]
)

stop_axis = fig.add_axes(
    [0.88, 0.055, 0.08, 0.045]
)


command_box = TextBox(
    command_axis,
    "Motor command: ",
    initial=""
)

send_button = Button(
    send_axis,
    "Send"
)

start_button = Button(
    start_axis,
    "Start"
)

stop_button = Button(
    stop_axis,
    "STOP"
)


def send_command_from_box(_event=None):
    text = command_box.text.strip()

    if not text:
        return

    try:
        command = parse_motor_command(text)
        queue_motor_command(command)
        command_box.set_val("")

    except ValueError as exc:
        print("Invalid motor command:", exc)


def start_motor(_event=None):
    queue_motor_command(
        MotorCommand(
            command_type=MotorCommandType.START,
            motor_id=MOTOR_ID,
        )
    )


def stop_motor(_event=None):
    queue_motor_command(
        MotorCommand(
            command_type=MotorCommandType.STOP,
            motor_id=MOTOR_ID,
        )
    )


# Press Enter in the text box OR click Send.
command_box.on_submit(send_command_from_box)
send_button.on_clicked(send_command_from_box)

start_button.on_clicked(start_motor)
stop_button.on_clicked(stop_motor)


def close_plot(_event):

    # Ask the Nano to stop the motor before shutting down.
    queue_motor_command(
        MotorCommand(
            command_type=MotorCommandType.STOP,
            motor_id=MOTOR_ID,
        )
    )

    # Give the BLE loop one short opportunity to transmit it.
    # The final shutdown still must not depend on this succeeding.
    deadline = time.perf_counter() + 0.10

    while (
            not command_queue.empty()
            and
            time.perf_counter() < deadline
    ):
        plt.pause(0.005)

    running["in_progress"] = False


fig.canvas.mpl_connect(
    "close_event",
    close_plot
)

print(
    'Motor controls are in the plot window. '
    'Example: set id 1 pos 0 vel 1 kp 0 kd 0.15 trq 0'
)

plt.show(block=False)


# Main plotting loop

last_plot_time = 0.0
plot_counter = 0

pending_plot_snapshot = None


try:

    while (
            running["in_progress"]
            and
            plt.fignum_exists(fig.number)
    ):

        # Drain every Bluetooth update currently waiting.
        #
        # Every queued update is written to CSV.
        # Only the newest update is kept for the next graph refresh.

        while True:

            try:
                snapshot = ble_data_queue.get_nowait()

            except queue.Empty:
                break


            (
                sample_time,
                encoder,
                loadcell1,
                loadcell2,
                motor_position,
                motor_velocity,
                motor_torque,
                motor_temperature,
                motor_packets,
                loadcell_packets,
                encoder_packets,
            ) = snapshot


            current_time = sample_time - start_time

            ankle_angle = count_to_deg_encoder(
                encoder
            )


            # Save every queued BLE update to CSV

            writer.writerow(
                [
                    current_time,
                    ankle_angle,
                    0.0,
                    loadcell1,
                    loadcell2,
                    motor_position,
                    motor_velocity,
                    motor_torque,
                    motor_temperature,
                ]
            )


            # Save only newest received state for plotting

            pending_plot_snapshot = (
                current_time,
                ankle_angle,
                loadcell1,
                loadcell2,
                motor_position,
                motor_velocity,
            )


        # Plot only at the chosen visual refresh rate.
        #
        # This keeps Matplotlib from redrawing for every BLE packet.

        now = time.perf_counter()

        if (
                pending_plot_snapshot is not None
                and
                now - last_plot_time >= PLOT_INTERVAL
        ):

            if not plotting["paused"]:

                (
                    current_time,
                    ankle_angle,
                    loadcell1,
                    loadcell2,
                    motor_position,
                    motor_velocity,
                ) = pending_plot_snapshot


                # Update plot data

                time_data.append(current_time)

                ankle_pos_data.append(ankle_angle)

                ankle_vel_data.append(
                    0.0
                )  # Encoder velocity placeholder

                loadcell1_data.append(loadcell1)
                loadcell2_data.append(loadcell2)

                motor_pos_data.append(motor_position)
                motor_vel_data.append(motor_velocity)


                # Update plots

                line1.set_data(
                    time_data,
                    ankle_pos_data
                )

                line2.set_data(
                    time_data,
                    ankle_vel_data
                )

                line3.set_data(
                    time_data,
                    loadcell1_data
                )

                line4.set_data(
                    time_data,
                    loadcell2_data
                )

                line5.set_data(
                    time_data,
                    motor_pos_data
                )

                line6.set_data(
                    time_data,
                    motor_vel_data
                )


                # Keep x-axis moving

                for axis in axes_list:

                    axis.set_xlim(
                        max(
                            0,
                            current_time - time_window
                        ),
                        current_time
                    )


                # Update y-axis occasionally.
                #
                # Use a separate counter instead of len(time_data),
                # because len(time_data) stays at MAX_POINTS once full.

                plot_counter += 1

                if plot_counter % 10 == 0:

                    for axis in axes_list:

                        axis.relim()

                        axis.autoscale_view(
                            scalex=False,
                            scaley=True
                        )


                fig.canvas.draw_idle()
                fig.canvas.flush_events()


            pending_plot_snapshot = None
            last_plot_time = now


        # Keep GUI responsive without forcing a full graph redraw.

        plt.pause(0.005)


except KeyboardInterrupt:
    print("Ctrl+C pressed, closing figure")


finally:

    running["in_progress"] = False

    csv_file.close()

    plt.close("all")

    print("Plotting stopped")
    print(f"CSV saved at: {path}")
