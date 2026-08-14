"""
This code plots all CubeMars motors data from the following format:

motor id: 2 pos(rad): 0.1234 vel(rad/s): 0.5678 trq(N*m): 1.2345 temp(C): 32 err: 0

Terminal commands:

Local Python commands:
- pause
- resume
- exit

Motor control commands are sent to the Teensy:
- start id 1
- stop id 1
- set id 1 pos 0
- set id 1 vel 1

This class is called from main.py.
"""

import serial
import time
import matplotlib.pyplot as plt
import threading
import csv
from datetime import datetime


class MotorMITPlotter:

    def __init__(
        self,
        port="COM6",
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
        self.plotting_paused = False
        self.stopped = False

        # Prepare plot data
        self.time_data = []

        self.position_data = []
        self.velocity_data = []
        self.torque_data = []
        self.temperature_data = []

        # Create CSV file
        self.filename = (
            "MITmotorData_"
            + datetime.now().strftime("%Y%m%d_%H%M%S")
            + ".csv"
        )

        self.file = open(self.filename, "w", newline="")
        self.writer = csv.writer(self.file)

        # Column titles
        self.writer.writerow([
            "Time (s)",
            "Position (rad)",
            "Velocity (rad/s)",
            "Torque (Nm)",
            "Temperature (C)"
        ])

        self.start_time = None
        self.fig = None

    def stop_motor(self):
        """
        Stop motor plotting, save CSV, and close serial connection.
        """

        # Avoid stopping twice
        if self.stopped == True:
            return

        self.stopped = True
        self.running = False

        if not self.file.closed:

            self.file.close()

            print(f"CSV file saved as {self.filename}")

        if self.ser.is_open:

            self.ser.flush()
            self.ser.close()

        print("Motor Plotting Stopped")

    def close_plot(self, event):
        """
        Called automatically when the figure is closed.
        """

        self.stop_motor()

    def control_motor(self):
        """
        Allows the user to control motor parameters by sending commands
        from the terminal to the Teensy.

        Local Python commands:
        pause
        resume
        exit

        All other commands are sent to the Teensy.
        """

        while self.running == True:

            try:

                command = input()
                command = command.strip().lower()

                # Ignore empty commands
                if command == "":
                    continue

                # Stop Python plotting program
                if command in (
                    "exit",
                    "end",
                    "stop",
                    "stop plotting",
                    "stop motor plotting"
                ):

                    self.running = False
                    break

                # Pause plotting
                if command in (
                    "pause",
                    "pause plotting",
                    "pause motor",
                    "pause motor plotting",
                    "pause id 1"
                ):

                    self.plotting_paused = True

                    print(
                        "Plotting paused. Teensy data is still being read. "
                        'Enter "resume" to continue.'
                    )

                    continue

                # Resume plotting
                if command in (
                    "resume",
                    "resume plotting",
                    "resume motor",
                    "resume motor plotting",
                    "continue plotting",
                    "continue"
                ):

                    self.plotting_paused = False

                    print("Motor plotting resumes now")

                    continue

                # Send all other commands to Teensy
                if self.ser.is_open:

                    self.ser.write(
                        (command + "\n").encode("utf-8")
                    )

                    self.ser.flush()

                    print(
                        "Command sent to Teensy:",
                        command
                    )

            except EOFError:

                self.running = False
                break

            except KeyboardInterrupt:

                self.running = False
                break

            except serial.SerialException as e:

                print("Could not send command:", e)
                self.running = False
                break

    def run(self):

        time.sleep(1)

        # Create plot
        plt.ion()

        print("Figure created")

        width = 12
        height = 8

        self.fig, axes = plt.subplots(ncols=2, nrows=2, figsize=(width, height))

        ax1 = axes[0, 0]
        ax2 = axes[0, 1]
        ax3 = axes[1, 0]
        ax4 = axes[1, 1]

        # Main title
        self.fig.suptitle(
            "Real Time Motor Multi-Plot Dashboard",
            fontsize=14
        )

        # Plot lines
        posline, = ax1.plot(
            self.time_data,
            self.position_data,
            linewidth=1.5,
            color="#A8E6CF"
        )

        velline, = ax2.plot(
            self.time_data,
            self.velocity_data,
            linewidth=1.5,
            color="#A7C7E7"
        )

        torqline, = ax3.plot(
            self.time_data,
            self.torque_data,
            linewidth=1.5,
            color="purple"
        )

        templine, = ax4.plot(
            self.time_data,
            self.temperature_data,
            linewidth=1.5,
            color="pink"
        )

        # Figure titles
        ax1.set_title("CubeMars Motor Position Over Time")
        ax2.set_title("CubeMars Motor Velocity Over Time")
        ax3.set_title("CubeMars Motor Torque Over Time")
        ax4.set_title("CubeMars Motor Temperature Over Time")

        # Figure x-axes
        ax1.set_xlabel("Time (s)")
        ax2.set_xlabel("Time (s)")
        ax3.set_xlabel("Time (s)")
        ax4.set_xlabel("Time (s)")

        # Figure y-axes
        ax1.set_ylabel("Position (rad)")
        ax2.set_ylabel("Velocity (rad/s)")
        ax3.set_ylabel("Torque (Nm)")
        ax4.set_ylabel("Temperature (C)")

        # Grid
        ax1.grid(True)
        ax2.grid(True)
        ax3.grid(True)
        ax4.grid(True)

        self.fig.canvas.mpl_connect("close_event", self.close_plot)

        plt.show(block=False)

        # Start terminal control thread
        terminal_thread = threading.Thread(
            target=self.control_motor,
            daemon=True
        )

        terminal_thread.start()

        # Start motor test
        self.ser.flush()

        print("MIT mode motor test starts now")
        print("Local commands: pause, resume, exit")

        self.start_time = time.perf_counter()

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
                if raw.startswith("motor id:") == False:

                    print("!" + raw)

                    self.fig.canvas.flush_events()
                    continue

                arr = raw.split()

                try:

                    pos_index = arr.index("pos(rad):") + 1
                    vel_index = arr.index("vel(rad/s):") + 1
                    trq_index = arr.index("trq(N*m):") + 1
                    temp_index = arr.index("temp(C):") + 1

                    new_position = float(arr[pos_index])
                    new_velocity = float(arr[vel_index])
                    new_torque = float(arr[trq_index])
                    new_temperature = float(arr[temp_index])

                except Exception as e:

                    print("Could not read line:", e)
                    print(raw)
                    continue

                # x-axis data
                current_time = time.perf_counter() - self.start_time

                # Write data to CSV
                self.writer.writerow([
                    current_time,
                    new_position,
                    new_velocity,
                    new_torque,
                    new_temperature
                ])

                self.file.flush()

                # Continue CSV logging but pause plotting
                if self.plotting_paused == True:

                    self.fig.canvas.flush_events()

                    plt.pause(0.001)
                    continue

                # Add new data
                self.time_data.append(current_time)
                self.position_data.append(new_position)
                self.velocity_data.append(new_velocity)
                self.torque_data.append(new_torque)
                self.temperature_data.append(new_temperature)

                # Keep only data within time window
                while (self.time_data and (current_time - self.time_data[0]) > self.time_window):

                    self.time_data.pop(0)

                    self.position_data.pop(0)
                    self.velocity_data.pop(0)
                    self.torque_data.pop(0)
                    self.temperature_data.pop(0)

                # Update plots
                posline.set_data(self.time_data, self.position_data)
                velline.set_data(self.time_data, self.velocity_data)
                torqline.set_data(self.time_data, self.torque_data)
                templine.set_data(self.time_data, self.temperature_data)

                # Update all axes
                for ax in (ax1, ax2, ax3, ax4):

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
            self.stop_motor()
            plt.close("all")