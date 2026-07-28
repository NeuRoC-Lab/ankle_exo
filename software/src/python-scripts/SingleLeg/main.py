"""
This code plots the data for the encoder, loadcells, and motor (MIT mode)
integrated together on the one leg setup.

Data is read through Bluetooth from Arduino Nano.

The program is split into three files:

1. main.py
   Calls functions to run the one leg setup test

2. ble.py
   Bluetooth connection, callback, and command-sending code.

3. core.py
   Shared classes for commands, telemetry, encoder conversion, CSV, and plots.

Run:
python software/src/python-scripts/SingleLeg/main.py.py
"""

import threading
import time

from ble import AnkleExoBluetooth
from core import (
    MOTOR_ID,
    PLOT_INTERVAL,
    CSVLogger,
    EncoderConverter,
    MotorCommand,
    MotorCommandType,
    PlotManager,
    PlotSnapshot,
    parse_motor_command,
)


# Program time reference.
# This is defined before the Bluetooth thread starts.

start_time = time.perf_counter()


# Connect to Arduino Nano Bluetooth

def connect_to_arduino(stop_event):

    bluetooth = AnkleExoBluetooth(stop_event)
    bluetooth.connect()

    return bluetooth


# Set up CSV logging

def setup_csv():

    csv_logger = CSVLogger(
        "../../SingleLegData_BLE.csv"
    )

    csv_logger.open()

    return csv_logger


# Motor command callbacks

def send_command_from_box(plotter, bluetooth, _event=None):

    text = plotter.command_box.text.strip()

    if not text:
        return

    try:

        command = parse_motor_command(text)
        bluetooth.queue_motor_command(command)

        plotter.command_box.set_val("")

    except ValueError as exc:
        print("Invalid motor command:", exc)


# Start motor

def start_motor(bluetooth, _event=None):

    bluetooth.queue_motor_command(
        MotorCommand(
            command_type=MotorCommandType.START,
            motor_id=MOTOR_ID,
        )
    )


# Stop motor

def stop_motor(bluetooth, _event=None):

    bluetooth.queue_motor_command(
        MotorCommand(
            command_type=MotorCommandType.STOP,
            motor_id=MOTOR_ID,
        )
    )


# Set up plotting

def setup_plot(bluetooth, stop_event):

    plotter = PlotManager()

    def send_callback(event=None):
        send_command_from_box(
            plotter,
            bluetooth,
            event
        )

    def start_callback(event=None):
        start_motor(
            bluetooth,
            event
        )

    def stop_callback(event=None):
        stop_motor(
            bluetooth,
            event
        )

    def close_callback(event=None):
        close_plot(
            plotter,
            bluetooth,
            stop_event,
            event
        )

    plotter.setup(
        send_command_callback=send_callback,
        start_motor_callback=start_callback,
        stop_motor_callback=stop_callback,
        close_callback=close_callback,
    )

    return plotter


# Close plot

def close_plot(
        plotter,
        bluetooth,
        stop_event,
        _event=None,
):

    if stop_event.is_set():
        return

    # Ask the Nano to stop the motor before shutting down.
    stop_motor(bluetooth)

    # Give the BLE loop one short opportunity to transmit it.
    # The final shutdown still must not depend on this succeeding.
    deadline = time.perf_counter() + 0.10

    while (
            bluetooth.has_pending_commands()
            and
            time.perf_counter() < deadline
    ):
        plotter.pause(0.005)

    bluetooth.disconnect()


# Read Bluetooth data and save every update to CSV

def process_bluetooth_data(
        bluetooth,
        csv_logger,
        encoder_converter,
):

    pending_plot_snapshot = None

    # Get every Bluetooth update currently waiting.
    #
    # Every queued update is written to CSV.
    # Only the newest update is kept for the next graph refresh.

    snapshots = bluetooth.get_pending_snapshots()

    for snapshot in snapshots:

        current_time = snapshot.sample_time - start_time

        ankle_angle = encoder_converter.count_to_deg(
            snapshot.encoder
        )


        # Save every queued BLE update to CSV

        csv_logger.write_snapshot(
            current_time = current_time,
            ankle_angle = ankle_angle,
            loadcell1 = snapshot.loadcell1,
            loadcell2 = snapshot.loadcell2,
            motor_position = snapshot.motor_position,
            motor_velocity = snapshot.motor_velocity,
            motor_torque = snapshot.motor_torque,
            motor_temperature = snapshot.motor_temperature,
        )


        # Save only newest received state for plotting

        pending_plot_snapshot = PlotSnapshot(
            current_time = current_time,
            ankle_angle = ankle_angle,
            loadcell1 = snapshot.loadcell1,
            loadcell2 = snapshot.loadcell2,
            motor_position = snapshot.motor_position,
            motor_velocity = snapshot.motor_velocity,
        )

    return pending_plot_snapshot


# Update plots

def update_plot(plotter, snapshot):

    plotter.update(snapshot)


# Main plotting loop

def run_plotting_loop(
        bluetooth,
        csv_logger,
        encoder_converter,
        plotter,
        stop_event,
):

    last_plot_time = 0.0
    pending_plot_snapshot = None

    while (
            not stop_event.is_set()
            and
            plotter.is_open()
    ):

        newest_snapshot = process_bluetooth_data(
            bluetooth,
            csv_logger,
            encoder_converter,
        )

        if newest_snapshot is not None:
            pending_plot_snapshot = newest_snapshot


        # Plot only at the chosen visual refresh rate.
        #
        # This keeps Matplotlib from redrawing for every BLE packet.

        now = time.perf_counter()

        if (
                pending_plot_snapshot is not None
                and
                now - last_plot_time >= PLOT_INTERVAL
        ):

            update_plot(
                plotter,
                pending_plot_snapshot
            )

            pending_plot_snapshot = None
            last_plot_time = now


        # Keep GUI responsive without forcing a full graph redraw.

        plotter.pause(0.005)


# Shut everything down

def shutdown(
        bluetooth,
        csv_logger,
        plotter,
        stop_event,
):

    if not stop_event.is_set():
        close_plot(
            plotter,
            bluetooth,
            stop_event
        )

    bluetooth.disconnect()

    csv_logger.close()
    plotter.close()

    print("Plotting stopped")
    print(f"CSV saved at: {csv_logger.path}")


# Main program

def main():

    stop_event = threading.Event()

    encoder_converter = EncoderConverter()

    bluetooth = connect_to_arduino(stop_event)

    csv_logger = setup_csv()

    plotter = setup_plot(
        bluetooth,
        stop_event
    )

    try:

        run_plotting_loop(
            bluetooth,
            csv_logger,
            encoder_converter,
            plotter,
            stop_event,
        )

    except KeyboardInterrupt:
        print("Ctrl+C pressed, closing figure")

    finally:

        shutdown(
            bluetooth,
            csv_logger,
            plotter,
            stop_event,
        )


if __name__ == "__main__":
    main()