"""
This code plots all CubeMars motors data from the following format:

    motor id: 2 pos (rad?): 0.1234 vel(rad/s?): 0.5678 trq(N*m): 1.2345 temp (C): 32 err: 0

Type in CLion terminal:
python software\src\cubemars_CAN_arduino\plotMotorMIT.py

"""

import serial
import time
import matplotlib.pyplot as plt

# Arduino Connection
port = "COM5" # Change port as necessary
baud = 115200
time_window = 10  # Plot x-axis window in seconds

ser = serial.Serial(port, baud, timeout=1)
time.sleep(2)

# Clear old data
ser.reset_input_buffer()
ser.reset_output_buffer()

running = {"in_progress": True}

def stop_motor():
    running["in_progress"] = False

    if ser.is_open:
        ser.flush()
        ser.close()
        print("Motor Plotting Stopped")

def close_plot(event):
    stop_motor()

time.sleep(1)

# Prepare 4 plots data
time_data = []
position_data = []
velocity_data = []
torque_data = []
temperature_data = []

start_time = time.perf_counter()

# Create plot
plt.ion()
print("Figure created")
width = 8
height = 6

fig, (ax1, ax2, ax3, ax4) = plt.subplots(2, 2, figsize=(width, height))
fig.suptitle("Real Time Motor Multi-Plot Dashboard", fontsize=14)

# Customize all 4 plots

posline, = ax1.plot(
    time_data,
    position_data,
    linewidth=1.5,
    color="#A8E6CF"
)

velline, = ax2.plot(
    time_data,
    velocity_data,
    linewidth=1.5,
    color="#A7C7E7"
)

torqline, = ax3.plot(
    time_data,
    torque_data,
    linewidth=1.5,
    color="purple"
)

templine, = ax4.plot(
    time_data,
    temperature_data,
    linewidth=1.5,
    color="pink"
)

ax1.set_title("CubeMars Motor Position Over Time")
ax2.set_title("CubeMars Motor Velocity Over Time")
ax3.set_title("CubeMars Motor Torque Over Time")
ax4.set_title("CubeMars Motor Temperature Over Time")

ax1.set_xlabel("Time (s)")
ax2.set_xlabel("Time (s)")
ax3.set_xlabel("Time (s)")
ax4.set_xlabel("Time (s)")

ax1.set_ylabel("Position (rad)")
ax2.set_ylabel("Velocity (rad/s)")
ax3.set_ylabel("Torque (Nm)")
ax4.set_ylabel("Temperature (C)")

ax1.grid(True)
ax2.grid(True)
ax3.grid(True)
ax4.grid(True)

fig.canvas.mpl_connect("close_event", close_plot)

# Show figure window
plt.show(block=False)

# Start Load Cells Test
ser.flush()
print("MIT mode motor test starts now")

try:

    while (running["in_progress"] == True) and (plt.fignum_exists(fig.number) == True):

        # y-axis data
        raw = ser.readline().decode().strip()
        arr = re.split(r"[ :,]", raw)

        pos_index = (arr.index("pos") + 2)
        vel_index = (arr.index("vel") + 2)
        trq_index = (arr.index("trq") + 2)
        temp_index = (arr.index("temp") + 2)

        position_data.append(float(arr[pos_index]))
        velocity_data.append(float(arr[vel_index]))
        torque_data.append(float(arr[trq_index]))
        temperature_data.append(float(arr[temp_index]))

        # x-axis data
        current_time = time.perf_counter() - start_time
        time_data.append(current_time)

        # keep plot centered around only within time window
        while time_data and (current_time - time_data[0]) > time_window:
            time_data.pop(0)
            position_data.pop(0)
            velocity_data.pop(0)
            torque_data.pop(0)
            temperature_data.pop(0)

        # update plots
        posline.set_data(time_data, position_data)
        velline.set_data(time_data, velocity_data)
        torqline.set_data(time_data, torque_data)
        templine.set_data(time_data, temperature_data)

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

        fig.canvas.draw()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    print("Ctrl+C pressed, closing figure")
    ser.close()

finally:
    stop_motor()
    plt.close("all")