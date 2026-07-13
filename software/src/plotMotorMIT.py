"""
This code plots all CubeMars motors data from the following format:

    motor id: 2 pos (rad): 0.1234 vel(rad/s?): 0.5678 trq(N*m): 1.2345 temp (C): 32 err: 0

Check for port: python -m serial.tools.list_ports

Command types:
- Motor control command: start/stop id 1; set id 1 [param] [value]
- Pause 
- Resume
- Exit

Type in CLion terminal to run code:
python software\src\plotMotorMIT.py
python src\plotMotorMIT.py
"""

import serial
import time
import matplotlib.pyplot as plt
import threading

# Arduino Connection
port = "COM6"  # Change port as necessary
baud = 115200
time_window = 10  # Plot x-axis window in seconds

ser = serial.Serial(port, baud, timeout=1)
time.sleep(2)

# Clear old data
ser.reset_input_buffer()
ser.reset_output_buffer()

running = {"in_progress": True}
plotting = {"paused": False}


def stop_motor():
    running["in_progress"] = False

    if ser.is_open:
        ser.flush()
        ser.close()
        print("Motor Plotting Stopped")


def close_plot(event):
    stop_motor()


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
width = 12
height = 8

fig, axes = plt.subplots(2, 2, figsize=(width, height))
ax1 = axes[0, 0]
ax2 = axes[0, 1]
ax3 = axes[1, 0]
ax4 = axes[1, 1]

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

terminal_thread = threading.Thread(
    target=control_motor,
    daemon=True
)
terminal_thread.start()

# Start Motor Test
ser.flush()
print("MIT mode motor test starts now")
print("Local commands: pause, resume, exit")

try:

    while (
        running["in_progress"] == True
        and plt.fignum_exists(fig.number) == True
    ):

        raw = ser.readline().decode(errors="ignore").strip()
        print(raw)

        # Skip empty lines
        if raw == "":
            continue

        # Skip Arduino startup / command / error messages
        if raw.startswith("motor id:") == False:
            print("!" + raw)
            fig.canvas.flush_events()
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

        # When paused, continue reading serial but discard plotting data
        if plotting["paused"] == True:
            fig.canvas.flush_events()
            continue

        # x-axis data
        current_time = time.perf_counter() - start_time

        time_data.append(current_time)
        position_data.append(new_position)
        velocity_data.append(new_velocity)
        torque_data.append(new_torque)
        temperature_data.append(new_temperature)

        # Keep plot centered within time window
        while time_data and (current_time - time_data[0]) > time_window:
            time_data.pop(0)
            position_data.pop(0)
            velocity_data.pop(0)
            torque_data.pop(0)
            temperature_data.pop(0)

        # Update plots
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

finally:
    stop_motor()
    plt.close("all")
