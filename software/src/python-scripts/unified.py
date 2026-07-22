

import serial
import time
import matplotlib.pyplot as plt
import threading
import os
import csv
import json
from time import sleep

# Teensy Connection
port = "/dev/cu.usbmodem198847901"  # Change port as necessary
baud = 115200
time_window = 10  # Plot x-axis window in seconds

ser = serial.Serial(port, baud, timeout=1)
time.sleep(2)

# Clear old data
ser.reset_input_buffer()
ser.reset_output_buffer()

running = {"in_progress": True}
plotting = {"paused": False}

def stop_all():

    running["in_progress"] = False

    if ser.is_open:
        ser.flush()
        ser.close()

    print("Plotting and data logging stopped")

def close_plot(event):
    stop_all()


def control_motor():
    """
    Allows user to control motor parameters by sending commands
    from the CLion terminal to the Teensy.
    """

    while running["in_progress"] == True:

        try:
            command = input()
            command = command.strip().lower()

            # Ignore empty commands
            if command == "":
                continue

            # Stop Python plotting program
            if command in (
                    "stop",
                    "stop motor",
                    "stop id 1",
                    "exit",
                    "end"
            ):
                running["in_progress"] = False
                break

            # Pause plotting
            if command in (
                    "pause",
                    "pause plotting",
                    "pause motor",
                    "pause motor plotting",
                    "pause id 1"
            ):
                plotting["paused"] = True
                print(
                    'Plotting paused. Teensy data is still being read. '
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
                plotting["paused"] = False
                print("Motor plotting resumes now")
                continue

            # Send all other commands to Teensy
            if ser.is_open:
                ser.write((command + "\n").encode("utf-8"))
                ser.flush()
                print("Command sent to Teensy:", command)

        except EOFError:
            running["in_progress"] = False
            break

        except KeyboardInterrupt:
            running["in_progress"] = False
            break

        except serial.SerialException as e:
            print("Could not send command:", e)
            running["in_progress"] = False
            break


time.sleep(1)

# Prepare CSV file
filename = "SingleLegData.csv"
path = os.path.abspath(filename)

# Prepare plots data
time_data = []
ankle_pos_data = []
ankle_vel_data = []
loadcell1_data = []
loadcell2_data = []
motor_pos_data = []
motor_vel_data = []

start_time = time.perf_counter()

# Create plot
plt.ion()
print("Figure created")
width = 12
height = 8

fig, axes = plt.subplots(ncols=2, nrows=3, figsize=(width,height))
ax1 = axes[0, 0]
ax2 = axes[0, 1]
ax3 = axes[1, 0]
ax4 = axes[1, 1]
ax5 = axes[2, 0]
ax6 = axes[2, 1]

"""
Create plots in the format of
    +---------+---------+
    |   ax1   |   ax2   |
    +---------+---------+
    |   ax3   |   ax4   |
    +---------+---------+
    |   ax5   |   ax6   |
    +---------+---------+
"""

# Main title
fig.suptitle("Real Time Data Of One Ankle Exoskeleton", fontsize=14)

line1, = ax1.plot(time_data, ankle_pos_data, linewidth=1.5, color="red")
line2, = ax2.plot(time_data, ankle_vel_data, linewidth=1.5, color="green")
line3, = ax3.plot(time_data, loadcell1_data, linewidth=1.5, color="orange")
line4, = ax4.plot(time_data, loadcell2_data, linewidth=1.5, color="blue")
line5, = ax5.plot(time_data, motor_pos_data, linewidth=1.5, color="yellow")
line6, = ax6.plot(time_data, motor_vel_data, linewidth=1.5, color="purple")

# Figure titles
ax1.set_title("Ankle Relative Position Over Time")
ax2.set_title("Ankle Filtered Velocity Over Time")
ax3.set_title("Cable 1 Tension Over Time")
ax4.set_title("Cable 2 Tension Over Time")
ax5.set_title("Motor Position Over Time")
ax6.set_title("Motor Velocity Over Time")

# Figure x-axes
ax1.set_xlabel("Time (s)")
ax2.set_xlabel("Time(s)")
ax3.set_xlabel("Time (s)")
ax4.set_xlabel("Time (s)")
ax5.set_xlabel("Time (s)")
ax6.set_xlabel("Time (s)")

# Figure y-axes
ax1.set_ylabel("Ankle Angle (deg)")
ax2.set_ylabel("Ankle Velocity (deg/s)")
ax3.set_ylabel("Cable 1 Tension (V)")
ax4.set_ylabel("Cable 2 Tension (V)")
ax5.set_ylabel("Motor position (rad)")
ax6.set_ylabel("Motor velocity (rad/s)")

ax1.grid(True)
ax2.grid(True)
ax3.grid(True)
ax4.grid(True)
ax5.grid(True)
ax6.grid(True)

fig.canvas.mpl_connect("close_event", close_plot)

plt.show(block=False)

# Start plotting encoder positions
ser.flush()
print("Left and right encoder position and velocity plotting starts now")

# Start Motor Test
ser.flush()
print("MIT mode motor test starts now")
print("Local commands: pause, resume, exit")

with open(path, "w", newline="") as csv_file:

    writer = csv.writer(csv_file)

    #Write sheet column titles by writing the first row
    writer.writerow([
        "Time (s)",
        "Ankle Position (deg)",
        "Ankle Velocity (deg/s)",
        "Cable 1 Tension (V)",
        "Cable 2 Tension (V)",
        "Motor position (rad)",
        "Motor velocity (rad/s)",
        #"Motor Kp",
        #"Motor Kd",
        "Motor Feedforward Torque (Nm)",
        "Motor Temperature (C)",
    ])

    try:

        while (
                running["in_progress"] == True
                and plt.fignum_exists(fig.number) == True
        ):
            raw = ser.readline().decode(errors="ignore").strip()
            while ser.in_waiting > 0:
                raw = ser.readline().decode(errors="ignore").strip()
            #print(raw)
            
            # Skip empty lines
            if raw == "":
                continue
            
            try:
                data = json.loads(raw)
            
            except json.JSONDecodeError:
                print("Could not read JSON:", raw)
                continue

            try:

                ankle_pos = int(data["LENC"]) # or RENC
                ankle_vel = 0.0 #TO CHANGE
                
                l1_voltage = float(data["LLC1"]) # OR RLC1
                l2_voltage = float(data["LLC2"]) # OR RLC2

                motor = data["MOTORS"][0] #if nested list, use motor = data["MOTORS][0]
                motor_pos = float(motor["MTR_POS_RAD"])
                motor_vel = float(motor["MTR_VEL_RADS"])
                #motor_kp = float(data[])
                #motor_kd = float(data[])
                motor_ff = float(motor["MTR_TRQ_NM"])
                motor_temp = float(motor["MTR_TEMP_DEG"])

                # x-axis data
                current_time = time.perf_counter() - start_time

                #Update csv file
                writer.writerow([
                    current_time,
                    ankle_pos,
                    ankle_vel,
                    l1_voltage,
                    l2_voltage,
                    motor_pos,
                    motor_vel,
                    #motor_kp,
                    #motor_kd,
                    motor_ff,
                    motor_temp
                ])

            except Exception as e:
                print("Could not read line:", e)
                print(raw)
                continue

            # When paused, continue reading serial but discard plotting data
            if plotting["paused"] == True:
                fig.canvas.flush_events()
                continue


            time_data.append(current_time)
            if not (ankle_pos == 65535):
                ankle_pos_data.append(ankle_pos)
                print("Skipping invalid data")
            else:
                ankle_pos_data.append(ankle_pos_data[-1])
            ankle_vel_data.append(ankle_vel)
            loadcell1_data.append(l1_voltage)
            loadcell2_data.append(l2_voltage)
            motor_pos_data.append(motor_pos)
            motor_vel_data.append(motor_vel)

            # Keep plot centered within time window
            while time_data and (current_time - time_data[0]) > time_window:
                time_data.pop(0)
                ankle_pos_data.pop(0)
                ankle_vel_data.pop(0)
                loadcell1_data.pop(0)
                loadcell2_data.pop(0)
                motor_pos_data.pop(0)
                motor_vel_data.pop(0)

            # Update plots
            line1.set_data(time_data, ankle_pos_data)
            line2.set_data(time_data, ankle_vel_data)
            line3.set_data(time_data, loadcell1_data)
            line4.set_data(time_data, loadcell2_data)
            line5.set_data(time_data, motor_pos_data)
            line6.set_data(time_data, motor_vel_data)

            ax1.set_xlim(max(0, current_time - time_window), current_time)
            ax1.relim()
            ax1.autoscale_view(scalex=False, scaley=True)

            ax2.set_xlim(max(0, current_time - time_window), current_time)
            ax2.relim()
            ax2.autoscale_view(scalex=False, scaley=True)

            ax3.set_xlim(max(0, current_time - time_window), current_time)
            ax3.relim()
            ax3.autoscale_view(scalex=False, scaley=True)

            ax4.set_xlim(max(0, current_time - time_window), current_time)
            ax4.relim()
            ax4.autoscale_view(scalex=False, scaley=True)

            ax5.set_xlim(max(0, current_time - time_window), current_time)
            ax5.relim()
            ax5.autoscale_view(scalex=False, scaley=True)

            ax6.set_xlim(max(0, current_time - time_window), current_time)
            ax6.relim()
            ax6.autoscale_view(scalex=False, scaley=True)

            fig.canvas.draw()
            fig.canvas.flush_events()

    except KeyboardInterrupt:
        print("Ctrl+C pressed, closing figure")

    finally:
        stop_all()
        plt.close("all")
'''
This code plots the data for the encoder, loadcells, and motor (MIT mode) integrated together on the one leg setup
This script extracts data from the following format:

    (to enter)

Check for port: python -m serial.tools.list_ports

Command types:
- Motor control command: start/stop id 1; set id 1 [param] [value]
- Pause
- Resume
- Exit


'''