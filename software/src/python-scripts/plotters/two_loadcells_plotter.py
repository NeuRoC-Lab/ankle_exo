"""
This code plots the voltage output of 2 load cells connected to the analog
amplifier board connected to an Arduino board.

The analog count value is converted to voltage on a 5V basis.

The left and right load cells need to be carefully defined to avoid confusion.

This class is called from main.py.
"""

import serial
import time
import matplotlib.pyplot as plt
import csv
import os


class TwoLoadCellsPlotter:

    def __init__(
        self,
        port="COM5",
        baud=115200,
        time_window=10
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

        # Prepare plot data
        self.time_data = []
        self.left_data = []
        self.right_data = []

        # CSV file
        self.filename = "loadcellData.csv"
        self.path = os.path.abspath(
            self.filename
        )

        self.fig = None

    def stop_loadcells(self):
        """
        Stop load cell test and close serial port.
        """

        # Avoid stopping twice
        if self.running == False:
            return

        self.running = False

        if self.ser.is_open:

            self.ser.flush()
            self.ser.close()

        print("Load Cells Test Stopped")

        print("CSV file saved here:")
        print(self.path)

    def close_plot(self, event):
        """
        Called automatically when the figure is closed.
        """

        self.stop_loadcells()

    def run(self):

        time.sleep(1)

        start_time = time.perf_counter()

        # Create plot
        plt.ion()

        print("Figure created")

        self.fig, ax = plt.subplots()

        line1, = ax.plot(
            self.time_data,
            self.left_data,
            linewidth=1.5,
            color="blue",
            label="Left Load Cell"
        )

        line2, = ax.plot(
            self.time_data,
            self.right_data,
            linewidth=1.5,
            color="orange",
            label="Right Load Cell"
        )

        ax.set_title("Load Cells Voltage vs Time")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Voltage (V)")
        ax.grid(True)
        ax.legend()

        self.fig.canvas.mpl_connect(
            "close_event",
            self.close_plot
        )

        plt.show(block=False)

        # Start load cells test
        self.ser.flush()

        print("Load cells test starts now")

        # Open CSV file
        with open(self.path, "w", newline="") as csv_file:

            writer = csv.writer(csv_file)

            # Column titles
            writer.writerow([
                "Time (s)",
                "Left load cell voltage (V)",
                "Right load cell voltage (V)"
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

                    # Skip empty lines
                    if raw == "":

                        plt.pause(0.001)
                        continue

                    arr = raw.split()

                    try:

                        # Find positions of "left" and "right"
                        left_index = next(
                            i for i, x in enumerate(arr)
                            if x.lower().rstrip(":") == "left"
                        )

                        right_index = next(
                            i for i, x in enumerate(arr)
                            if x.lower().rstrip(":") == "right"
                        )

                        # Find first numeric value after LEFT
                        left_value = None

                        for token in arr[left_index + 1:right_index]:

                            try:

                                left_value = float(token)
                                break

                            except ValueError:

                                pass

                        # Find first numeric value after RIGHT
                        right_value = None

                        for token in arr[right_index + 1:]:

                            try:

                                right_value = float(token)
                                break

                            except ValueError:

                                pass

                        if left_value is None or right_value is None:
                            continue

                    except (StopIteration, IndexError):

                        continue

                    # x-axis data
                    current_time = time.perf_counter() - start_time

                    # Add new data
                    self.time_data.append(current_time)
                    self.left_data.append(left_value)
                    self.right_data.append(right_value)

                    # Write data to CSV
                    writer.writerow([
                        current_time,
                        left_value,
                        right_value
                    ])

                    csv_file.flush()

                    # Keep only data within time window
                    while (self.time_data and (current_time - self.time_data[0]) > self.time_window):
                        self.time_data.pop(0)
                        self.left_data.pop(0)
                        self.right_data.pop(0)

                    # Update plots
                    line1.set_data(self.time_data, self.left_data)
                    line2.set_data(self.time_data, self.right_data)

                    ax.set_xlim(
                        max(0, current_time - self.time_window),
                        current_time
                    )

                    ax.relim()
                    ax.autoscale_view(scalex=False, scaley=True)

                    self.fig.canvas.draw_idle()

                    plt.pause(0.001)

            except KeyboardInterrupt:
                print("Ctrl+C pressed, closing figure")

            finally:

                self.stop_loadcells()

                plt.close("all")