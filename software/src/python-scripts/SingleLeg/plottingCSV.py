"""
Plotting and CSV logging for the ankle exoskeleton.

This file handles:
- Real-time 6 graph plotting
- Motor control widgets
- CSV logging
"""

import csv
import os
from collections import deque

import matplotlib
matplotlib.use("QtAgg")

import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button


# Plot configuration

time_window = 10 # plot x-axis only displays the last 10 seconds of data
MAX_POINTS = 500

# Same visual refresh rate as the working UART code:
# 0.1 s = approximately 10 plot updates per second.
PLOT_INTERVAL = 0.1


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

    def write(
            self,
            current_time,
            snapshot,
    ):
        self.writer.writerow(
            [
                current_time,
                snapshot.ankle_angle,
                snapshot.ankle_velocity,
                snapshot.loadcell1,
                snapshot.loadcell2,
                snapshot.motor_position,
                snapshot.motor_velocity,
                snapshot.motor_torque,
                snapshot.motor_temperature,
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


    # Plot setup

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

        self.fig.suptitle("Real Time Data Of One Ankle Exoskeleton", fontsize=14)

        # Leave room at the bottom for motor controls.
        self.fig.subplots_adjust(bottom=0.18, hspace=0.55)

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


        # Motor control widgets, values may be adjusted

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


    # Update plot

    def update(
            self,
            current_time,
            snapshot,
    ):
        # Update plot data

        self.time_data.append(current_time)

        self.ankle_pos_data.append(snapshot.ankle_angle)
        self.ankle_vel_data.append(snapshot.ankle_velocity)

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
                max(0, current_time - time_window),
                current_time
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


    # Plot controls

    def get_command_text(self):
        return self.command_box.text.strip()

    def clear_command_text(self):
        self.command_box.set_val("")

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