"""
This code plots the data for the encoder, loadcells, and motor (MIT mode)
integrated together on the one leg setup.

Data is read through Bluetooth from Arduino Nano.

The code is separated into:

sensors.py
    Sensor and motor classes

bluetooth.py
    Bluetooth communication

plotting_csv.py
    Real-time plotting and CSV logging

Run:
python software/src/python-scripts/SingleLeg/main-bluetooth.py
"""

import threading
import time

from bluetooth import BluetoothManager

from sensors import (
    Motor_ID,
    Encoder,
    LoadCells,
    Motor,
    MotorCommand,
    MotorCommandType,
    parse_motor_command,
)

from plottingCSV import (
    CSVLogger,
    PlotManager,
    PLOT_INTERVAL,
)


# Program time reference.
# This is defined before the Bluetooth thread starts.

start_time = time.perf_counter()


# Sensor setup

def setup_sensors(): # create objects for sensors
    encoder = Encoder()
    loadcells = LoadCells()
    motor = Motor()

    return (
        encoder,
        loadcells,
        motor,
    )


# Connect to Arduino Nano Bluetooth

def connect_to_arduino(
        encoder,
        loadcells,
        motor,
        stop_event,
):

    print("Creating BluetoothManager...")
    bluetooth = BluetoothManager(
        encoder=encoder,
        loadcells=loadcells,
        motor=motor,
        stop_event=stop_event,
    )

    bluetooth.connect()
    print("Bluetooth connected")

    return bluetooth


# CSV setup

def setup_csv():
    csv_logger = CSVLogger(
        "../../SingleLegData_BLE_Test.csv"
    )

    csv_logger.open()

    return csv_logger


# Motor controls

def send_command_from_box(
        plotter,
        bluetooth,
        _event=None,
):
    text = plotter.get_command_text()

    if not text:
        return

    try:
        command = parse_motor_command(text)
        bluetooth.queue_motor_command(command)
        plotter.clear_command_text()

    except ValueError as exc:
        print("Invalid motor command:", exc)


def start_motor(
        bluetooth,
        _event=None,
):
    bluetooth.queue_motor_command(
        MotorCommand(
            command_type=MotorCommandType.START,
            motor_id=MOTOR_ID,
        )
    )


def stop_motor(
        bluetooth,
        _event=None,
):
    bluetooth.queue_motor_command(
        MotorCommand(
            command_type=MotorCommandType.STOP,
            motor_id=MOTOR_ID,
        )
    )


# Plot setup

def setup_plot(
        bluetooth,
        stop_event,
):
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

    deadline = time.perf_counter() + 0.10

    while (
            bluetooth.has_pending_commands() # wait until all motor commands are sent
            and
            time.perf_counter() < deadline
    ):
        plotter.pause(0.005)

    bluetooth.disconnect()


# Process Bluetooth data

def process_bluetooth_data(
        bluetooth,
        csv_logger,
):
    pending_plot_snapshot = None

    # Drain every Bluetooth update currently waiting.
    #
    # Every queued update is written to CSV.
    # Only the newest update is kept for the next graph refresh.

    snapshots = bluetooth.get_pending_snapshots()

    for snapshot in snapshots:
        current_time = (snapshot.sample_time - start_time)


        # Save every queued BLE update to CSV

        csv_logger.write(
            current_time,
            snapshot
        )


        # Save only newest received state for plotting

        pending_plot_snapshot = (
            current_time,
            snapshot,
        )

    return pending_plot_snapshot


# Update plot

def update_plot(plotter, current_time,snapshot):
    plotter.update(current_time, snapshot)


# Main plotting loop

def run_plotting_loop(
        bluetooth,
        csv_logger,
        plotter,
        stop_event,
):
    last_plot_time = 0.0

    pending_plot_snapshot = None

    while (not stop_event.is_set() and plotter.is_open()):

        newest_snapshot = process_bluetooth_data(
            bluetooth,
            csv_logger,
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
            (
                current_time,
                snapshot,
            ) = pending_plot_snapshot

            update_plot(
                plotter,
                current_time,
                snapshot,
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
    stop_event = threading.Event() # to stop safely bluetooth and plotting

    (
        encoder,
        loadcells,
        motor,
    ) = setup_sensors()

    # start bluetooth communication
    bluetooth = connect_to_arduino(
        encoder,
        loadcells,
        motor,
        stop_event,
    )

    csv_logger = setup_csv()

    plotter = setup_plot(
        bluetooth,
        stop_event
    )

    try:
        run_plotting_loop(
            bluetooth,
            csv_logger,
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