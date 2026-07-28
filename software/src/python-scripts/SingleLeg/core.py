"""
This handles all features other than bluetooth connection for the one leg setup, including:
- Motor commands
- Telemetry data structures
- Encoder count -> degree conversion
- CSV logging
- Matplotlib plot creation and updating
"""

import csv
import os
import struct
import threading
from collections import deque
from dataclasses import dataclass, field
from enum import IntEnum

import matplotlib
matplotlib.use("QtAgg")

import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button


MOTOR_ID = 0x02


# Plot configuration

time_window = 10
MAX_POINTS = 500

# Same visual refresh rate as the working UART code:
# 0.1 s = approximately 10 plot updates per second.
PLOT_INTERVAL = 0.1


# Encoder configuration

encoder_max_count = 4096  # 12 bits resolution
encoder_half_count = encoder_max_count // 2


# Motor command configuration

class MotorCommandType(IntEnum):
    START = 0
    STOP = 1
    ZERO = 2
    SET = 3


COMMAND_PACKET_FORMAT = "<BB2x5f"
COMMAND_PACKET_SIZE = struct.calcsize(COMMAND_PACKET_FORMAT)


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


@dataclass(frozen=True)
class TelemetrySnapshot:

    sample_time: float

    encoder: int

    loadcell1: float
    loadcell2: float

    motor_position: float
    motor_velocity: float
    motor_torque: float
    motor_temperature: int

    motor_packets: int
    loadcell_packets: int
    encoder_packets: int


@dataclass(frozen=True)
class PlotSnapshot:

    current_time: float
    ankle_angle: float

    loadcell1: float
    loadcell2: float

    motor_position: float
    motor_velocity: float


# Encoder conversion

class EncoderConverter:

    def __init__(self):
        self.first_encoder_value = None


    def count_to_deg(self, encoder):

        if self.first_encoder_value is None:
            self.first_encoder_value = encoder

        ankle_angle = (
            (encoder - self.first_encoder_value) # Zero encoder at the start
            % encoder_max_count
        )

        if ankle_angle >= encoder_half_count:
            ankle_angle -= encoder_max_count

        ankle_angle = ( # count to degree conversion
            ankle_angle
            * 360.0
            / encoder_max_count
        )

        return ankle_angle


# Motor command parsing

def parse_motor_command(text: str) -> MotorCommand:


    tokens = text.strip().lower().split()

    if not tokens:
        raise ValueError("Command is empty")

    command_name = tokens[0]

    if command_name in {"start", "stop", "zero"}:

        if len(tokens) > 2:
            raise ValueError(
                f"Too many values after '{command_name}'"
            )

        motor_id = (
            int(tokens[1], 0)
            if len(tokens) >= 2
            else MOTOR_ID
        )

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
            "Expected start: stop, zero, or set"
        )

    values = {
        "id": MOTOR_ID,
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

        key = aliases.get(
            tokens[index],
            tokens[index]
        )

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


# CSV logging

class CSVLogger:

    def __init__(self, filename):

        self.filename = filename
        self.path = os.path.abspath(filename)

        self.csv_file = None
        self.writer = None


    def open(self):

        print("CSV will save at:", self.path)

        self.csv_file = open(
            self.path,
            "w",
            newline=""
        )

        self.writer = csv.writer(self.csv_file)

        self.writer.writerow(
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


    def write_snapshot(
            self,
            current_time,
            ankle_angle,
            loadcell1,
            loadcell2,
            motor_position,
            motor_velocity,
            motor_torque,
            motor_temperature,
    ):

        if self.writer is None:
            raise RuntimeError("CSV file is not open")

        self.writer.writerow(
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


    def close(self):

        if (
                self.csv_file is not None
                and
                not self.csv_file.closed
        ):
            self.csv_file.close()


# Plotting

class PlotManager:

    def __init__(self):

        self.fig = None
        self.axes_list = []

        self.line1 = None
        self.line2 = None
        self.line3 = None
        self.line4 = None
        self.line5 = None
        self.line6 = None

        self.command_box = None
        self.send_button = None
        self.start_button = None
        self.stop_button = None

        # Plot data buffers

        self.time_data = deque(maxlen=MAX_POINTS)

        self.ankle_pos_data = deque(maxlen=MAX_POINTS)
        self.ankle_vel_data = deque(maxlen=MAX_POINTS)

        self.loadcell1_data = deque(maxlen=MAX_POINTS)
        self.loadcell2_data = deque(maxlen=MAX_POINTS)

        self.motor_pos_data = deque(maxlen=MAX_POINTS)
        self.motor_vel_data = deque(maxlen=MAX_POINTS)

        self.plot_counter = 0


    def setup(
            self,
            send_command_callback,
            start_motor_callback,
            stop_motor_callback,
            close_callback,
    ):

        plt.ion()
        print("Figure created")

        width = 12
        height = 9

        self.fig, axes = plt.subplots(
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

        self.fig.suptitle(
            "Real Time Data Of One Ankle Exoskeleton",
            fontsize=14
        )

        # Reserve room at the bottom for motor controls.
        self.fig.subplots_adjust(
            bottom=0.18,
            hspace=0.55
        )

        self.line1, = ax1.plot(
            [],
            [],
            linewidth=1.5,
            color="red"
        )

        self.line2, = ax2.plot(
            [],
            [],
            linewidth=1.5,
            color="green"
        )

        self.line3, = ax3.plot(
            [],
            [],
            linewidth=1.5,
            color="orange"
        )

        self.line4, = ax4.plot(
            [],
            [],
            linewidth=1.5,
            color="blue"
        )

        self.line5, = ax5.plot(
            [],
            [],
            linewidth=1.5,
            color="yellow"
        )

        self.line6, = ax6.plot(
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

        self.axes_list = [
            ax1,
            ax2,
            ax3,
            ax4,
            ax5,
            ax6
        ]


        # Motor control widgets

        command_axis = self.fig.add_axes(
            [0.10, 0.055, 0.55, 0.045]
        )

        send_axis = self.fig.add_axes(
            [0.67, 0.055, 0.09, 0.045]
        )

        start_axis = self.fig.add_axes(
            [0.78, 0.055, 0.08, 0.045]
        )

        stop_axis = self.fig.add_axes(
            [0.88, 0.055, 0.08, 0.045]
        )

        self.command_box = TextBox(
            command_axis,
            "Motor command: ",
            initial=""
        )

        self.send_button = Button(
            send_axis,
            "Send"
        )

        self.start_button = Button(
            start_axis,
            "Start"
        )

        self.stop_button = Button(
            stop_axis,
            "STOP"
        )

        # Press Enter in the text box OR click Send.
        self.command_box.on_submit(send_command_callback)
        self.send_button.on_clicked(send_command_callback)

        self.start_button.on_clicked(start_motor_callback)
        self.stop_button.on_clicked(stop_motor_callback)

        self.fig.canvas.mpl_connect(
            "close_event",
            close_callback
        )

        print(
            'Motor controls are in the plot window. '
            'Example: set id 1 pos 0 vel 1 kp 0 kd 0.15 trq 0'
        )

        plt.show(block=False)


    def update(self, snapshot: PlotSnapshot):

        # Update plot data

        self.time_data.append(snapshot.current_time)

        self.ankle_pos_data.append(snapshot.ankle_angle)
        self.ankle_vel_data.append(0.0)  # Encoder velocity placeholder

        self.loadcell1_data.append(snapshot.loadcell1)
        self.loadcell2_data.append(snapshot.loadcell2)

        self.motor_pos_data.append(snapshot.motor_position)
        self.motor_vel_data.append(snapshot.motor_velocity)


        # Update plots

        self.line1.set_data(
            self.time_data,
            self.ankle_pos_data
        )

        self.line2.set_data(
            self.time_data,
            self.ankle_vel_data
        )

        self.line3.set_data(
            self.time_data,
            self.loadcell1_data
        )

        self.line4.set_data(
            self.time_data,
            self.loadcell2_data
        )

        self.line5.set_data(
            self.time_data,
            self.motor_pos_data
        )

        self.line6.set_data(
            self.time_data,
            self.motor_vel_data
        )


        # Keep x-axis moving

        for axis in self.axes_list:
            axis.set_xlim(
                max(0, snapshot.current_time - time_window),
                snapshot.current_time
            )


        # Update y-axis occasionally.
        #
        # Use a separate counter instead of len(time_data),
        # because len(time_data) stays at MAX_POINTS once full.

        self.plot_counter += 1

        if self.plot_counter % 10 == 0:

            for axis in self.axes_list:

                axis.relim()

                axis.autoscale_view(
                    scalex=False,
                    scaley=True
                )


        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


    def is_open(self):

        return (
            self.fig is not None
            and
            plt.fignum_exists(self.fig.number)
        )


    def pause(self, seconds):
        plt.pause(seconds)


    def close(self):
        plt.close("all")