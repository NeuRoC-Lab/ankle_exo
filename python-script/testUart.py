#!/usr/bin/env python3

import argparse
import json
import queue
import threading
import time
from collections import deque
from typing import Any

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

DEFAULT_BAUD_RATE = 115200
DEFAULT_WINDOW_SECONDS = 15.0
DEFAULT_MOTOR_ID = 2

# Start conservatively when testing.
DEFAULT_MIN_VELOCITY = -2.0
DEFAULT_MAX_VELOCITY = 2.0
DEFAULT_VELOCITY_STEP = 0.1
DEFAULT_KD = 1.0

# These keys must match SerialConfig.h.
LOAD_CELL_KEYS = {
    "LLC1": "Left load cell 1",
    "LLC2": "Left load cell 2"
    #"RLC1": "Right load cell 1",
    #"RLC2": "Right load cell 2",
}

# Only the right encoder is currently displayed.
ENCODER_KEYS = {
    "LENC": "Right encoder",
}

MOTORS_KEY = "MOTORS"

MOTOR_ID_KEY = "MTR_ID_DEC"
MOTOR_POSITION_KEY = "MTR_POS_RAD"

INVALID_ENCODER_POSITION = 65535


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------

def numeric_or_nan(value: Any) -> float:
    """Convert a numeric JSON value to float, otherwise return NaN."""
    if isinstance(value, bool):
        return float("nan")

    if isinstance(value, (int, float)):
        return float(value)

    return float("nan")


def find_motor(
        packet: dict[str, Any],
        desired_motor_id: int,
) -> dict[str, Any] | None:
    """Find one motor dictionary in the MOTORS array by CAN ID."""
    motors = packet.get(MOTORS_KEY)

    if not isinstance(motors, list):
        return None

    for motor in motors:
        if not isinstance(motor, dict):
            continue

        if motor.get(MOTOR_ID_KEY) == desired_motor_id:
            return motor

    return None


# ---------------------------------------------------------------------------
# Rolling telemetry storage
# ---------------------------------------------------------------------------

class TelemetryHistory:
    def __init__(self, window_seconds: float) -> None:
        self.window_seconds = window_seconds
        self.start_time = time.monotonic()

        self.times: deque[float] = deque()

        self.load_cells: dict[str, deque[float]] = {
            key: deque()
            for key in LOAD_CELL_KEYS
        }

        self.encoders: dict[str, deque[float]] = {
            key: deque()
            for key in ENCODER_KEYS
        }

        self.motor_position: deque[float] = deque()

    def append(
            self,
            packet: dict[str, Any],
            motor_id: int,
    ) -> None:
        elapsed = time.monotonic() - self.start_time
        self.times.append(elapsed)

        for key in self.load_cells:
            self.load_cells[key].append(
                numeric_or_nan(packet.get(key))
            )

        for key in self.encoders:
            encoder_value = packet.get(key)

            if encoder_value == INVALID_ENCODER_POSITION:
                encoder_value = None

            self.encoders[key].append(
                numeric_or_nan(encoder_value)
            )

        motor = find_motor(packet, motor_id)

        if motor is None:
            self.motor_position.append(float("nan"))
        else:
            self.motor_position.append(
                numeric_or_nan(
                    motor.get(MOTOR_POSITION_KEY)
                )
            )

        self.remove_old_samples(elapsed)

    def remove_old_samples(self, current_time: float) -> None:
        cutoff = current_time - self.window_seconds

        while self.times and self.times[0] < cutoff:
            self.times.popleft()

            for values in self.load_cells.values():
                values.popleft()

            for values in self.encoders.values():
                values.popleft()

            self.motor_position.popleft()


# ---------------------------------------------------------------------------
# Serial reader and command sender
# ---------------------------------------------------------------------------

class SerialReader(threading.Thread):
    def __init__(
            self,
            port: str,
            baud_rate: int,
            output_queue: queue.Queue[dict[str, Any]],
    ) -> None:
        super().__init__(daemon=True)

        self.port_name = port
        self.baud_rate = baud_rate
        self.output_queue = output_queue

        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()

        self.serial_port: serial.Serial | None = None

    def run(self) -> None:
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=0.5,
                write_timeout=0.5,
            )

            # Teensy may reset or reopen its USB serial interface.
            time.sleep(1.0)
            self.serial_port.reset_input_buffer()

            print(
                f"Reading {self.port_name} at "
                f"{self.baud_rate} baud"
            )

            while not self.stop_event.is_set():
                raw_line = self.serial_port.readline()

                if not raw_line:
                    continue

                try:
                    line = raw_line.decode(
                        "utf-8",
                        errors="strict",
                    ).strip()
                except UnicodeDecodeError:
                    continue

                if not line:
                    continue

                try:
                    packet = json.loads(line)
                except json.JSONDecodeError:
                    # Command acknowledgements and debug lines are not JSON.
                    print(f"RX text: {line}")
                    continue

                if not isinstance(packet, dict):
                    continue

                try:
                    self.output_queue.put_nowait(packet)
                except queue.Full:
                    # Drop the oldest queued packet instead of blocking
                    # the serial-reading thread.
                    try:
                        self.output_queue.get_nowait()
                    except queue.Empty:
                        pass

                    try:
                        self.output_queue.put_nowait(packet)
                    except queue.Full:
                        pass

        except serial.SerialException as error:
            print(f"Serial error: {error}")

        finally:
            if self.serial_port is not None:
                try:
                    self.serial_port.close()
                except serial.SerialException:
                    pass

    def send_line(self, command: str) -> bool:
        if (
                self.serial_port is None
                or not self.serial_port.is_open
        ):
            print("Cannot send command: serial port is not open")
            return False

        message = command.rstrip("\r\n") + "\n"

        try:
            with self.write_lock:
                self.serial_port.write(
                    message.encode("utf-8")
                )
                self.serial_port.flush()

        except serial.SerialException as error:
            print(f"Command transmission failed: {error}")
            return False

        print(f"TX: {command}")
        return True

    def send_mit_velocity(
            self,
            motor_id: int,
            velocity: float,
            kd: float = DEFAULT_KD,
    ) -> bool:
        command = (
            f"set id {motor_id} "
            f"pos 0 "
            f"vel {velocity:.4f} "
            f"kp 0 "
            f"kd {kd:.4f} "
            f"trq 0"
        )

        return self.send_line(command)

    def start_motor(self) -> bool:
        return self.send_line("start")

    def stop_motor(self) -> bool:
        return self.send_line("stop")

    def stop(self) -> None:
        self.stop_event.set()


# ---------------------------------------------------------------------------
# Plotting helper
# ---------------------------------------------------------------------------

def autoscale_vertical_axis(
        axis: plt.Axes,
        value_groups: list[list[float]],
        minimum_span: float,
        padding_fraction: float = 0.10,
) -> None:
    finite_values: list[float] = []

    for values in value_groups:
        finite_values.extend(
            value
            for value in values
            if value == value  # NaN is not equal to itself.
        )

    if not finite_values:
        return

    minimum = min(finite_values)
    maximum = max(finite_values)
    span = maximum - minimum

    if span < minimum_span:
        midpoint = (minimum + maximum) / 2.0
        half_span = minimum_span / 2.0

        minimum = midpoint - half_span
        maximum = midpoint + half_span
        span = minimum_span

    padding = span * padding_fraction

    axis.set_ylim(
        minimum - padding,
        maximum + padding,
        )


# ---------------------------------------------------------------------------
# Live telemetry interface
# ---------------------------------------------------------------------------

class LiveTelemetryPlot:
    def __init__(
            self,
            history: TelemetryHistory,
            packet_queue: queue.Queue[dict[str, Any]],
            serial_reader: SerialReader,
            motor_id: int,
            minimum_velocity: float,
            maximum_velocity: float,
    ) -> None:
        self.history = history
        self.packet_queue = packet_queue
        self.serial_reader = serial_reader
        self.motor_id = motor_id

        self.commanded_velocity = 0.0
        self.last_sent_velocity: float | None = None
        self.motor_started = False
        self.setting_slider_programmatically = False

        self.figure, axes = plt.subplots(
            3,
            1,
            figsize=(12, 9),
            sharex=True,
        )

        self.load_cell_axis = axes[0]
        self.encoder_axis = axes[1]
        self.motor_axis = axes[2]

        self.load_cell_lines = {
            key: self.load_cell_axis.plot(
                [],
                [],
                label=label,
            )[0]
            for key, label in LOAD_CELL_KEYS.items()
        }

        self.encoder_lines = {
            key: self.encoder_axis.plot(
                [],
                [],
                label=label,
            )[0]
            for key, label in ENCODER_KEYS.items()
        }

        self.motor_position_line = self.motor_axis.plot(
            [],
            [],
            label="Position",
        )[0]

        self.configure_axes()
        self.configure_controls(
            minimum_velocity=minimum_velocity,
            maximum_velocity=maximum_velocity,
        )

        self.figure.canvas.mpl_connect(
            "close_event",
            self.on_window_closed,
        )

        self.animation = FuncAnimation(
            self.figure,
            self.update,
            interval=50,
            blit=False,
            cache_frame_data=False,
        )

    def configure_axes(self) -> None:
        self.load_cell_axis.set_title("Load cells")
        self.load_cell_axis.set_ylabel("Raw voltage")
        self.load_cell_axis.grid(True)
        self.load_cell_axis.legend(
            loc="upper left",
            ncols=2,
        )

        self.encoder_axis.set_title("Encoder")
        self.encoder_axis.set_ylabel("Encoder counts")
        self.encoder_axis.grid(True)
        self.encoder_axis.legend(loc="upper left")

        self.motor_axis.set_title(
            f"Motor position — CAN ID {self.motor_id}"
        )
        self.motor_axis.set_xlabel("Time (s)")
        self.motor_axis.set_ylabel("Position (rad)")
        self.motor_axis.grid(True)
        self.motor_axis.legend(loc="upper left")

        self.figure.tight_layout()

        # Reserve room at the bottom for the slider and buttons.
        self.figure.subplots_adjust(bottom=0.18)

    def configure_controls(
            self,
            minimum_velocity: float,
            maximum_velocity: float,
    ) -> None:
        zero_axis = self.figure.add_axes(
            [0.04, 0.065, 0.11, 0.05]
        )

        slider_axis = self.figure.add_axes(
            [0.20, 0.075, 0.50, 0.035]
        )

        start_axis = self.figure.add_axes(
            [0.75, 0.065, 0.08, 0.05]
        )

        stop_axis = self.figure.add_axes(
            [0.85, 0.065, 0.08, 0.05]
        )

        self.zero_button = Button(
            zero_axis,
            "Zero velocity",
        )

        self.velocity_slider = Slider(
            ax=slider_axis,
            label="Velocity (rad/s)",
            valmin=minimum_velocity,
            valmax=maximum_velocity,
            valinit=0.0,
            valstep=DEFAULT_VELOCITY_STEP,
        )

        self.start_button = Button(
            start_axis,
            "Start",
        )

        self.stop_button = Button(
            stop_axis,
            "Stop",
        )

        self.zero_button.on_clicked(
            self.on_zero_clicked
        )

        self.velocity_slider.on_changed(
            self.on_velocity_changed
        )

        self.start_button.on_clicked(
            self.on_start_clicked
        )

        self.stop_button.on_clicked(
            self.on_stop_clicked
        )

    def set_slider_without_command(self, value: float) -> None:
        self.setting_slider_programmatically = True

        try:
            self.velocity_slider.set_val(value)
        finally:
            self.setting_slider_programmatically = False

    def send_current_velocity(self) -> None:
        self.serial_reader.send_mit_velocity(
            motor_id=self.motor_id,
            velocity=self.commanded_velocity,
            kd=DEFAULT_KD,
        )

    def on_velocity_changed(self, value: float) -> None:
        if self.setting_slider_programmatically:
            return

        velocity = float(value)

        if (
                self.last_sent_velocity is not None
                and abs(
            velocity - self.last_sent_velocity
        ) < 0.05
        ):
            return

        self.commanded_velocity = velocity
        self.last_sent_velocity = velocity

        # This sends the command even before Start is pressed.
        # Since m_enabled should still be false, the controller should
        # only store the command until MIT mode is started.
        self.send_current_velocity()

    def on_start_clicked(self, _event: Any) -> None:
        # Set the selected command before entering motor mode.
        self.send_current_velocity()

        if self.serial_reader.start_motor():
            self.motor_started = True

    def on_stop_clicked(self, _event: Any) -> None:
        self.commanded_velocity = 0.0
        self.last_sent_velocity = 0.0

        self.set_slider_without_command(0.0)

        # First request zero velocity.
        self.serial_reader.send_mit_velocity(
            motor_id=self.motor_id,
            velocity=0.0,
            kd=DEFAULT_KD,
        )

        time.sleep(0.05)
        self.serial_reader.stop_motor()

        self.motor_started = False

    def on_zero_clicked(self, _event: Any) -> None:
        self.commanded_velocity = 0.0
        self.last_sent_velocity = 0.0

        self.set_slider_without_command(0.0)

        self.serial_reader.send_mit_velocity(
            motor_id=self.motor_id,
            velocity=0.0,
            kd=DEFAULT_KD,
        )

    def on_window_closed(self, _event: Any) -> None:
        self.safe_stop_motor()

    def safe_stop_motor(self) -> None:
        if (
                self.serial_reader.serial_port is None
                or not self.serial_reader.serial_port.is_open
        ):
            return

        self.serial_reader.send_mit_velocity(
            motor_id=self.motor_id,
            velocity=0.0,
            kd=DEFAULT_KD,
        )

        time.sleep(0.05)
        self.serial_reader.stop_motor()

        self.motor_started = False

    def drain_packet_queue(self) -> None:
        while True:
            try:
                packet = self.packet_queue.get_nowait()
            except queue.Empty:
                break

            self.history.append(
                packet,
                self.motor_id,
            )

    def update(self, _frame_number: int) -> list[Any]:
        self.drain_packet_queue()

        if not self.history.times:
            return []

        times = list(self.history.times)

        for key, line in self.load_cell_lines.items():
            line.set_data(
                times,
                list(self.history.load_cells[key]),
            )

        for key, line in self.encoder_lines.items():
            line.set_data(
                times,
                list(self.history.encoders[key]),
            )

        motor_position = list(
            self.history.motor_position
        )

        self.motor_position_line.set_data(
            times,
            motor_position,
        )

        current_time = times[-1]

        left_limit = max(
            0.0,
            current_time - self.history.window_seconds,
            )

        right_limit = max(
            self.history.window_seconds,
            current_time,
        )

        for axis in (
                self.load_cell_axis,
                self.encoder_axis,
                self.motor_axis,
        ):
            axis.set_xlim(
                left_limit,
                right_limit,
            )

        autoscale_vertical_axis(
            self.load_cell_axis,
            [
                list(values)
                for values in self.history.load_cells.values()
            ],
            minimum_span=0.05,
        )

        autoscale_vertical_axis(
            self.encoder_axis,
            [
                list(values)
                for values in self.history.encoders.values()
            ],
            minimum_span=20.0,
        )

        autoscale_vertical_axis(
            self.motor_axis,
            [motor_position],
            minimum_span=0.1,
        )

        return [
            *self.load_cell_lines.values(),
            *self.encoder_lines.values(),
            self.motor_position_line,
        ]


# ---------------------------------------------------------------------------
# Command-line arguments
# ---------------------------------------------------------------------------

def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot Teensy NDJSON telemetry and control "
            "MIT motor velocity."
        )
    )

    parser.add_argument(
        "port",
        help=(
            "Serial port, for example /dev/ttyACM0, "
            "/dev/cu.usbmodem123456, or COM5"
        ),
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD_RATE,
        help=(
            f"Serial baud rate "
            f"(default: {DEFAULT_BAUD_RATE})"
        ),
    )

    parser.add_argument(
        "--window",
        type=float,
        default=DEFAULT_WINDOW_SECONDS,
        help=(
            "Visible rolling time window in seconds "
            f"(default: {DEFAULT_WINDOW_SECONDS})"
        ),
    )

    parser.add_argument(
        "--motor-id",
        type=int,
        default=DEFAULT_MOTOR_ID,
        help=(
            "CAN motor ID to display and control "
            f"(default: {DEFAULT_MOTOR_ID})"
        ),
    )

    parser.add_argument(
        "--min-velocity",
        type=float,
        default=DEFAULT_MIN_VELOCITY,
        help=(
            "Minimum velocity-slider value in rad/s "
            f"(default: {DEFAULT_MIN_VELOCITY})"
        ),
    )

    parser.add_argument(
        "--max-velocity",
        type=float,
        default=DEFAULT_MAX_VELOCITY,
        help=(
            "Maximum velocity-slider value in rad/s "
            f"(default: {DEFAULT_MAX_VELOCITY})"
        ),
    )

    arguments = parser.parse_args()

    if arguments.window <= 0:
        parser.error("--window must be greater than zero")

    if arguments.min_velocity >= arguments.max_velocity:
        parser.error(
            "--min-velocity must be smaller than --max-velocity"
        )

    return arguments


# ---------------------------------------------------------------------------
# Program entry point
# ---------------------------------------------------------------------------

def main() -> None:
    arguments = parse_arguments()

    packet_queue: queue.Queue[dict[str, Any]] = queue.Queue(
        maxsize=1000
    )

    history = TelemetryHistory(
        window_seconds=arguments.window
    )

    serial_reader = SerialReader(
        port=arguments.port,
        baud_rate=arguments.baud,
        output_queue=packet_queue,
    )

    serial_reader.start()

    live_plot = LiveTelemetryPlot(
        history=history,
        packet_queue=packet_queue,
        serial_reader=serial_reader,
        motor_id=arguments.motor_id,
        minimum_velocity=arguments.min_velocity,
        maximum_velocity=arguments.max_velocity,
    )

    try:
        plt.show()

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        live_plot.safe_stop_motor()

        time.sleep(0.05)

        serial_reader.stop()
        serial_reader.join(timeout=1.0)


if __name__ == "__main__":
    main()