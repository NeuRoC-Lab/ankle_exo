"""
This code plots the data for the encoder, loadcells, and motor (MIT mode)
integrated together on the one leg setup.

Data is read through Bluetooth GATT characteristics from Arduino Nano.

Features:
- Real-time 6 graph plotting
- Encoder zeroing on first valid reading
- Encoder conversion: count -> degrees
- Fixed-size buffers to prevent lag
- BLE reading separated from plotting
- No duplicate plotting when BLE data is unchanged

Run:
python software/src/python-scripts/unified-bluetooth.py
"""


import asyncio
import threading
import time
import csv
import os
import struct

import matplotlib.pyplot as plt
from collections import deque
from dataclasses import dataclass, field
from bleak import BleakClient, BleakScanner


# Bluetooth configuration

DEVICE_NAME = "AnkleExo"
SERVICE_UUID = "CF45813E-4358-4903-B961-09996BB081FB"

LOAD_CELL_UUID = "CA87289F-102B-4078-AD8C-8F53063547A6"
MOTOR_UUID = "E0D883F6-705C-4A11-B117-E2B0909CC68E"
ENCODER_UUID = "094A717B-0C7F-4A23-BFD1-A4924E6E7DAB"

# Plot configuration

time_window = 10
MAX_POINTS = 500

encoder_max_count = 65536


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

encoder_zero = None

# Plot data buffers

time_data = deque(maxlen=MAX_POINTS)

ankle_pos_data = deque(maxlen=MAX_POINTS)
ankle_vel_data = deque(maxlen=MAX_POINTS)

loadcell1_data = deque(maxlen=MAX_POINTS)
loadcell2_data = deque(maxlen=MAX_POINTS)

motor_pos_data = deque(maxlen=MAX_POINTS)
motor_vel_data = deque(maxlen=MAX_POINTS)



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
            telemetry.loadcell2 = left2

            telemetry.loadcell_packets += 1


    except Exception as e:

        print("Load cell decode error:", e)


def encoder_callback(sender, data):

    """
    Encoder packet:

    uint16_t left
    uint16_t right
    """

    global encoder_zero


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

            if encoder_zero is None:
                encoder_zero = right

            telemetry.encoder = right - encoder_zero
            telemetry.encoder_packets += 1


    except Exception as e:
        print("Encoder decode error:", e)


# Bluetooth connection

async def bluetooth_connection():

    print("Searching for Bluetooth device...")

    device = await BleakScanner.find_device_by_name(DEVICE_NAME)

    if device is None:
        print("Bluetooth device not found")
        running["in_progress"] = False
        return

    async with BleakClient(device) as client:
        print("Connected to Bluetooth")

        await client.start_notify(MOTOR_UUID,  motor_callback)
        await client.start_notify(ENCODER_UUID, encoder_callback)
        await client.start_notify(LOAD_CELL_UUID, loadcell_callback)

        print("Receiving Bluetooth data...")

        while running["in_progress"]:
            await asyncio.sleep(0.1)


def start_bluetooth():
    asyncio.run(bluetooth_connection())

# Start BLE in background thread

bluetooth_thread = threading.Thread(target=start_bluetooth, daemon=True)
bluetooth_thread.start()

# CSV setup

filename = "SingleLegData_BLE.csv"
path = os.path.abspath(filename)

print("CSV will save at:" ,path)


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
height = 6


fig, axes = plt.subplots(
    ncols=2,
    nrows=3,
    figsize=(width,height)
)


ax1 = axes[0,0]
ax2 = axes[0,1]
ax3 = axes[1,0]
ax4 = axes[1,1]
ax5 = axes[2,0]
ax6 = axes[2,1]

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

fig.suptitle("Real Time Data Of One Ankle Exoskeleton",fontsize=14)

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

axes_list = [ax1, ax2, ax3, ax4, ax5, ax6]

fig.canvas.mpl_connect(
    "close_event",
    lambda event: running.update({"in_progress":False})
)

plt.show(block=False)


# Main plotting loop

start_time = time.perf_counter()

last_packets = (-1, -1, -1)

try:

    while (
            running["in_progress"]
            and
            plt.fignum_exists(fig.number)
    ):

        # Copy latest Bluetooth data safely
        with telemetry.lock:

            snapshot = {

                "encoder":
                    telemetry.encoder,

                "loadcell1":
                    telemetry.loadcell1,

                "loadcell2":
                    telemetry.loadcell2,

                "motor_position":
                    telemetry.motor_position,

                "motor_velocity":
                    telemetry.motor_velocity,

                "motor_torque":
                    telemetry.motor_torque,

                "motor_temperature":
                    telemetry.motor_temperature,


                "packets":
                    (
                        telemetry.motor_packets,
                        telemetry.loadcell_packets,
                        telemetry.encoder_packets
                    )
            }


        # Ignore loop iterations where BLE has not updated

        if snapshot["packets"] == last_packets:
            plt.pause(0.01)
            continue

        last_packets = snapshot["packets"]

        if plotting["paused"]:
            plt.pause(0.01)
            continue

        # Current time
        current_time = time.perf_counter() - start_time

        # Encoder conversion
        ankle_angle = snapshot["encoder"] * 360.0/encoder_max_count

        # Save CSV
        writer.writerow([
            current_time,
            ankle_angle,
            0.0,
            snapshot["loadcell1"],
            snapshot["loadcell2"],
            snapshot["motor_position"],
            snapshot["motor_velocity"],
            snapshot["motor_torque"],
            snapshot["motor_temperature"],
        ])

        # Update data
        time_data.append(current_time)

        ankle_pos_data.append(ankle_angle)
        ankle_vel_data.append(0.0) # Encoder velocity placeholder
        loadcell1_data.append(snapshot["loadcell1"])
        loadcell2_data.append(snapshot["loadcell2"])
        motor_pos_data.append(snapshot["motor_position"])
        motor_vel_data.append(snapshot["motor_velocity"])


        # Update plots

        line1.set_data(time_data, ankle_pos_data)
        line2.set_data(time_data, ankle_vel_data)
        line3.set_data(time_data, loadcell1_data)
        line4.set_data(time_data, loadcell2_data)
        line5.set_data(time_data, motor_pos_data)
        line6.set_data(time_data, motor_vel_data)

        # Keep x-axis moving

        for axis in axes_list:
            axis.set_xlim(
                max(0, current_time-time_window),
                current_time
            )


        # Update y-axis occasionally
        # prevents matplotlib slowdown

        if len(time_data) % 20 == 0:

            for axis in axes_list:
                axis.relim()
                axis.autoscale_view(scalex=False, scaley=True)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        # Plot refresh rate
        plt.pause(0.03)

except KeyboardInterrupt:
    print( "Ctrl+C pressed, closing figure")

finally:
    running["in_progress"] = False
    csv_file.close()
    plt.close("all")
    print("Plotting stopped")
    print(f"CSV saved at: {path}")