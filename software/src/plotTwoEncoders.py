"""
This code is used to the plot the relative angle of both encoders in degrees relative to their respective neutral position
The neutral position is recorded at the start of the test
The code is now updated to also plot raw velocity and filtered velocity using EWMA

To run this script, type in terminal of CLion:

python software\src\plotTwoEncoders.py

The following pin connections are used when testing with the Arduino Uno:
    SPI Clock SCK: pin 13
    SPI MOSI: pin 11
    SPI MISO: pin 12
    SPI Chip Select CSB: pin 10
"""

import serial
import time
import matplotlib.pyplot as plt

from software.src.plotMotorMIT import start_time

# Establish serial connection
port = "COM7"
baud = 115200
time_window = 15

ser = serial.Serial(port, baud, timeout=1)
time.sleep(2)

# Clear old data

ser.reset_input_buffer()
ser.reset_output_buffer()



running = {"in_progress": True}

def stop_encoders():
    running["in_progress"] = False
    if ser.is_open:
        ser.flush()
        ser.close()
        print("Encoders plotting stopped")

def close_plot(event):
    stop_encoders()

time.sleep(1)

# Prepare data for both plots

left_angle = []
right_angle = []

left_raw_velocity = []
right_raw_velocity = []

left_filtered_velocity = []
right_filtered_velocity = []

time_data = []

start_time = time.perf_counter()

# Create plots

plt.ion()
print("Figure Creating")
width = 16
height = 12

fig, axes = plt.subplots(ncols=2, nrows=3, figsize=(width,height))
ax1 = axes[0, 0]
ax2 = axes[0, 1]
ax3 = axes[1, 0]
ax4 = axes[1, 1]
ax5 = axes[2, 0]
ax6 = axes[2, 1]

fig.suptitle("Real Time Encoder Position for Left and Right Ankle Exoskeletons", fontsize=14)

line1, = ax1.plot(time_data, left_angle, linewidth=1.5, color="red")
line2, = ax2.plot(time_data, right_angle, linewidth=1.5, color="green")
line3, = ax3.plot(time_data, left_raw_velocity, linewidth=1.5, color="orange")
line4, = ax4.plot(time_data, right_raw_velocity, linewidth=1.5, color="blue")
line5, = ax5.plot(time_data, left_filtered_velocity, linewidth=1.5, color="yellow")
line6, = ax6.plot(time_data, right_filtered_velocity, linewidth=1.5, color="purple")

ax1.set_title("Left Encoder Position Over Time")
ax2.set_title("Right Encoder Position Over Time")
ax3.set_title("Left Encoder Raw Velocity Over Time")
ax4.set_title("Right Encoder Raw Velocity Over Time")
ax5.set_title("Left Encoder Filtered Velocity Over Time")
ax6.set_title("Right Encoder Filtered Velocity Over Time")

ax1.set_xlabel("Time (s)")
ax2.set_xlabel("Time(s)")
ax3.set_xlabel("Time (s)")
ax4.set_xlabel("Time (s)")
ax5.set_xlabel("Time (s)")
ax6.set_xlabel("Time (s)")

ax1.set_ylabel("Left encoder angle (deg)")
ax2.set_ylabel("Right encoder angle (deg)")
ax3.set_ylabel("Left encoder raw velocity (deg/s)")
ax4.set_ylabel("Right encoder raw velocity (deg/s)")
ax5.set_ylabel("Left encoder filtered velocity (deg/s)")
ax6.set_ylabel("Right encoder filtered velocity (deg/s)")

ax1.grid(True)
ax2.grid(True)
ax3.grid(True)
ax4.grid(True)
ax5.grid(True)
ax6.grid(True)

fig.canvas.mp1_connect("close_event", close_plot)

plt.show(block=False)

# Start plotting encoder positions
ser.flush()
print("Left and right encoder position and velocity plotting starts now")

try:

    while running["in_progress"] == True and plt.fignum_exists(fig.number) == True:

        raw = ser.readline().decode(errors="ignore").strip()
        print(raw)

        # Skip empty lines
        if raw == "":
            continue

        # Skip startup / command / error messages
        if raw.startswith("Left") == False:
            print("This is most likely a command or error message")
            continue

        arr = raw.split

        try:
            left_pos_index = 3
            right_pos_index = 7
            left_raw_index = 11
            right_raw_index = 15
            left_filtered_index = 19
            right_filtered_index = 23

            # y-axis data
            left_angle.append(float(arr[left_pos_index]))
            right_angle.append(float(arr[right_pos_index]))
            left_raw_velocity.append(float(arr[left_raw_index]))
            right_raw_velocity.append(float(arr[right_raw_index]))
            left_filtered_velocity.append(float(arr[left_filtered_index]))
            right_filtered_velocity.append(float(arr[right_filtered_index]))

        except Exception as e:
            print("Could not read line:", e)
            continue

        # x-axis data
        current_time = time.perf_counter() - start_time
        time_data.append(current_time)

        # keep plot centered around only within the time window
        while time_data and (current_time - time_data[0]) > time_window:
            left_angle.pop(0)
            right_angle.pop(0)
            left_raw_velocity.pop(0)
            right_raw_velocity.pop(0)
            left_filtered_velocity.pop(0)
            right_filtered_velocity.pop(0)

        # update plots in real line
        line1.set_data(time_data, left_angle)
        line2.set_data(time_data, right_angle)
        line3.set_data(time_data, left_raw_velocity)
        line4.set_data(time_data, right_raw_velocity)
        line5.set_data(time_data, left_filtered_velocity)
        line6.set_data(time_data, right_filtered_velocity)

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


except KeyboardInterrupt:
    print("Ctrl+C pressed, closing figure")

finally:
    stop_encoders()
    plt.close("all")