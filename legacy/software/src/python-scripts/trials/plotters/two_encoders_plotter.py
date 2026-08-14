"""
This code is used to plot the relative angle of both encoders in degrees
relative to their respective neutral positions.

The neutral position is recorded at the start of the test.

The code also plots:
- Raw velocity
- Filtered velocity using EWMA

This class is called from main.py.

Arduino Uno Pin Connections:
SPI Clock SCK: pin 13
SPI MOSI: pin 11
SPI MISO: pin 12
SPI Chip Select CSB: pin 10
"""

import serial
import time
import matplotlib.pyplot as plt
import csv
import os


class TwoEncodersPlotter:

    def __init__(
        self,
        port="COM4",
        baud=115200,
        time_window=15
    ):

        self.port = port
        self.baud = baud
        self.time_window = time_window

        # Establish serial connection
        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=1
        )

        time.sleep(2)

        # Clear old data
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.running = True

        # Prepare data for six plots
        self.left_angle = []
        self.right_angle = []

        self.left_raw_velocity = []
        self.right_raw_velocity = []

        self.left_filtered_velocity = []
        self.right_filtered_velocity = []

        self.time_data = []

        # CSV file
        self.filename = "encoderData.csv"
        self.path = os.path.abspath(
            self.filename
        )

        self.fig = None

    def stop_encoders(self):
        """
        Stop encoder plotting and close serial port.
        """

        # Avoid stopping twice
        if self.running == False:
            return

        self.running = False

        if self.ser.is_open:

            self.ser.flush()
            self.ser.close()

        print("Encoders plotting stopped")

        print("CSV file saved here:")
        print(self.path)

    def close_plot(self, event):
        """
        Called automatically when the figure is closed.
        """

        self.stop_encoders()

    def run(self):

        time.sleep(1)

        start_time = time.perf_counter()

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

        plt.ion()

        print(
            "Figure Creating"
        )

        width = 16
        height = 12

        self.fig, axes = plt.subplots(
            ncols=2,
            nrows=3,
            figsize=(
                width,
                height
            )
        )

        ax1 = axes[0, 0]
        ax2 = axes[0, 1]

        ax3 = axes[1, 0]
        ax4 = axes[1, 1]

        ax5 = axes[2, 0]
        ax6 = axes[2, 1]

        # Main title
        self.fig.suptitle(
            "Real Time Encoder Position for "
            "Left and Right Ankle Exoskeletons",
            fontsize=14
        )

        # Plot lines
        line1, = ax1.plot(
            self.time_data,
            self.left_angle,
            linewidth=1.5,
            color="red"
        )

        line2, = ax2.plot(
            self.time_data,
            self.right_angle,
            linewidth=1.5,
            color="green"
        )

        line3, = ax3.plot(
            self.time_data,
            self.left_raw_velocity,
            linewidth=1.5,
            color="orange"
        )

        line4, = ax4.plot(
            self.time_data,
            self.right_raw_velocity,
            linewidth=1.5,
            color="blue"
        )

        line5, = ax5.plot(
            self.time_data,
            self.left_filtered_velocity,
            linewidth=1.5,
            color="yellow"
        )

        line6, = ax6.plot(
            self.time_data,
            self.right_filtered_velocity,
            linewidth=1.5,
            color="purple"
        )

        # Figure titles
        ax1.set_title("Left Encoder Position Over Time")
        ax2.set_title("Right Encoder Position Over Time")
        ax3.set_title("Left Encoder Raw Velocity Over Time")
        ax4.set_title("Right Encoder Raw Velocity Over Time")
        ax5.set_title("Left Encoder Filtered Velocity Over Time")
        ax6.set_title("Right Encoder Filtered Velocity Over Time")

        # Figure x-axes
        ax1.set_xlabel("Time (s)")
        ax2.set_xlabel("Time (s)")
        ax3.set_xlabel("Time (s)")
        ax4.set_xlabel("Time (s)")
        ax5.set_xlabel("Time (s)")
        ax6.set_xlabel("Time (s)")

        # Figure y-axes
        ax1.set_ylabel( "Left encoder angle (deg)")
        ax2.set_ylabel("Right encoder angle (deg)")
        ax3.set_ylabel("Left encoder raw velocity (deg/s)")
        ax4.set_ylabel("Right encoder raw velocity (deg/s)")
        ax5.set_ylabel("Left encoder filtered velocity (deg/s)")
        ax6.set_ylabel("Right encoder filtered velocity (deg/s)")

        # Grid
        ax1.grid(True)
        ax2.grid(True)
        ax3.grid(True)
        ax4.grid(True)
        ax5.grid(True)
        ax6.grid(True)

        self.fig.canvas.mpl_connect(
            "close_event",
            self.close_plot
        )

        plt.show(block=False)

        # Start plotting
        self.ser.flush()

        print(
            "Left and right encoder position "
            "and velocity plotting starts now"
        )

        # Open CSV file
        with open(self.path, "w", newline="") as csv_file:

            writer = csv.writer(csv_file)

            # Column titles
            writer.writerow([
                "Time (s)",
                "Left Position (deg)",
                "Right Position (deg)",
                "Left Raw Velocity (deg/s)",
                "Right Raw Velocity (deg/s)",
                "Left Filtered Velocity (deg/s)",
                "Right Filtered Velocity (deg/s)"
            ])

            try:

                while (
                    self.running == True
                    and plt.fignum_exists(
                        self.fig.number
                    ) == True
                ):

                    raw = (
                        self.ser
                        .readline()
                        .decode(errors="ignore")
                        .strip()
                    )

                    print(raw)

                    # Skip empty lines
                    if raw == "":
                        plt.pause(0.001)
                        continue

                    # Skip startup, command, or error messages
                    if raw.startswith("Left") == False:

                        print("This is most likely a command or error message")
                        continue

                    arr = raw.split()

                    try:

                        left_pos_index = 3
                        right_pos_index = 7

                        left_raw_index = 11
                        right_raw_index = 15

                        left_filtered_index = 19
                        right_filtered_index = 23

                        # Read newest values
                        new_left_angle = float(arr[left_pos_index])
                        new_right_angle = float(arr[right_pos_index])
                        new_left_raw_velocity = float(arr[left_raw_index])
                        new_right_raw_velocity = float(arr[right_raw_index])
                        new_left_filtered_velocity = float(arr[left_filtered_index])
                        new_right_filtered_velocity = float(arr[right_filtered_index])

                    except Exception as e:
                        print("Could not read line:", e)
                        continue

                    # x-axis data
                    current_time = (time.perf_counter() - start_time)

                    # Add new data
                    self.time_data.append(current_time)
                    self.left_angle.append(new_left_angle)
                    self.right_angle.append(new_right_angle)
                    self.left_raw_velocity.append(new_left_raw_velocity)
                    self.right_raw_velocity.append(new_right_raw_velocity)
                    self.left_filtered_velocity.append(new_left_filtered_velocity)
                    self.right_filtered_velocity.append(new_right_filtered_velocity)

                    # Write data to CSV file
                    writer.writerow([
                        current_time,
                        new_left_angle,
                        new_right_angle,
                        new_left_raw_velocity,
                        new_right_raw_velocity,
                        new_left_filtered_velocity,
                        new_right_filtered_velocity
                    ])

                    csv_file.flush()

                    # Keep only data within time window
                    while (
                        self.time_data
                        and (
                            current_time
                            - self.time_data[0]
                        ) > self.time_window
                    ):

                        self.time_data.pop(0)

                        self.left_angle.pop(0)
                        self.right_angle.pop(0)

                        self.left_raw_velocity.pop(0)
                        self.right_raw_velocity.pop(0)

                        self.left_filtered_velocity.pop(0)
                        self.right_filtered_velocity.pop(0)

                    # Update plots
                    line1.set_data(self.time_data, self.left_angle)
                    line2.set_data(self.time_data, self.right_angle)
                    line3.set_data(self.time_data, self.left_raw_velocity)
                    line4.set_data(self.time_data, self.right_raw_velocity)
                    line5.set_data(self.time_data, self.left_filtered_velocity)
                    line6.set_data(self.time_data, self.right_filtered_velocity)

                    # Update all axes
                    for ax in (ax1, ax2, ax3, ax4, ax5, ax6):

                        ax.set_xlim(
                            max(0, current_time - self.time_window),
                            current_time
                        )

                        ax.relim()
                        ax.autoscale_view(scalex=False, scaley=True)

                    self.fig.canvas.draw()
                    self.fig.canvas.flush_events()

                    plt.pause(0.001)

            except KeyboardInterrupt:
                print("Ctrl+C pressed, closing figure")

            finally:
                self.stop_encoders()
                plt.close("all")