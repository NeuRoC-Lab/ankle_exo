"""
This code is used to the plot the relative angle of both encoders in degrees relative to their respective neutral position
The neutral position is recorded at the start of the test

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
time_data = []

start_time = time.perf_counter()

# Create plots

plt.ion()
print("Figure Creating")
width = 8
height = 6

fig, axes = plt.subplots(ncols=2, nrows=1, figsize=(width,height))
ax1 = axes[0, 0]
ax2 = axes[0, 1]

fig.suptitle("Real Time Encoder Position for Left and Right Ankle Exoskeletons", fontsize=14)

line1, = ax1.plot(time_data, left_angle, linewidth=1.5, color="red")
line2, = ax2.plot(time_data, right_angle, linewidth=1.5, color="blue")

ax1.set_title("Left Encoder Position Over Time")
ax2.set_title("Right Encoder Position Over Time")

ax1.set_xlabel("Time (s)")
ax2.set_xlabel("Time(s)")

ax1.set_ylabel("Left encoder angle (deg)")
ax2.set_ylabel("Right encoder angle (deg)")

ax1.grid(True)
ax2.grid(True)

fig.canvas.mp1_connect("close_event", close_plot)

plt.show(block=False)

# Start plotting encoder positions
ser.flush()
print("Left and right encoder position plotting starts now")

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
            left_index = arr.index("Left") + 4
            right_index = arr.index("Right") + 4

            # y-axis data
            left_angle.append(float(arr[left_index]))
            right_angle.append(float(arr[right_index]))

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

        # update plots in real line
        line1.set_data(time_data, left_angle)
        line2.set_data(time_data, right_angle)

        ax1.set_xlim(max(0, current_time - time_window), current_time)
        ax1.relim()
        ax1.autoscale_view(scalex=False, scaley=True)

        ax2.set_xlim(max(0, current_time - time_window), current_time)
        ax2.relim()
        ax2.autoscale_view(scalex=False, scaley=True)


except KeyboardInterrupt:
    print("Ctrl+C pressed, closing figure")

finally:
    stop_encoders()
    plt.close("all")