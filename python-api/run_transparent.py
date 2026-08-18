from ankle_exo_api import Exoskeleton, Side

import time
import matplotlib.pyplot as plt
from collections import deque
import importlib
import exo_params


# -------------------------------------------------
# Plot configuration
# -------------------------------------------------

WINDOW_SECONDS = 10.0
PLOT_PERIOD = 0.02       # ~50 Hz plotting
MAX_SAMPLES = int(WINDOW_SECONDS / PLOT_PERIOD)

times = deque(maxlen=MAX_SAMPLES)
left_values = deque(maxlen=MAX_SAMPLES)
right_values = deque(maxlen=MAX_SAMPLES)


# -------------------------------------------------
# Matplotlib setup
# -------------------------------------------------

plt.ion()

fig, ax = plt.subplots()

left_line, = ax.plot(
    [],
    [],
    label="Left intermediate torque"
)

right_line, = ax.plot(
    [],
    [],
    label="Right intermediate torque"
)

ax.set_xlabel("Time [s]")
ax.set_ylabel("HP-filtered interaction torque [Nm]")
ax.set_title("Intermediate Torque - Live")
ax.legend()
ax.grid(True)


# -------------------------------------------------
# Exoskeleton
# -------------------------------------------------

with (Exoskeleton() as exo):

    time.sleep(5)

    exo.start_motor(Side.LEFT)
    exo.start_motor(Side.RIGHT)

    time.sleep(5)

    importlib.reload(exo_params)
    params = exo_params.params
    exo.update_transparent_params(Side.LEFT, params)
    exo.update_transparent_params(Side.RIGHT, params)

    print("Voltage of the battery")
    print(exo.get_power()[0])

    time.sleep(1)

    start_time = time.perf_counter()

    try:

        while True:

            loop_start = time.perf_counter()

            elapsed = (
                    loop_start
                    - start_time
            )

            # -----------------------------------------
            # Read intermediate torque values
            # -----------------------------------------

            #left_torque = exo.get_loadcells()["left1"] #note just for debug here!
            left_torque = (
                exo.get_intermediate_torque(
                    Side.LEFT
                )
            )

            #right_torque = exo.get_loadcells()["left2"]  #note just for debug here!
            right_torque =  (
             exo.get_intermediate_torque(
                    Side.RIGHT
                )
            )



            # -----------------------------------------
            # Store values
            # -----------------------------------------

            times.append(
                elapsed
            )

            left_values.append(
                left_torque
            )

            right_values.append(
                right_torque
            )


            # -----------------------------------------
            # Update plot
            # -----------------------------------------

            left_line.set_data(
                times,
                left_values
            )

            right_line.set_data(
                times,
                right_values
            )


            # -----------------------------------------
            # Moving X axis
            # -----------------------------------------

            if elapsed > WINDOW_SECONDS:

                ax.set_xlim(
                    elapsed - WINDOW_SECONDS,
                    elapsed
                )

            else:

                ax.set_xlim(
                    0,
                    WINDOW_SECONDS
                )


            # -----------------------------------------
            # Auto-scale Y axis
            # -----------------------------------------

            ax.relim()

            ax.autoscale_view(
                scalex=False,
                scaley=True
            )


            # -----------------------------------------
            # Draw
            # -----------------------------------------

            fig.canvas.draw_idle()
            fig.canvas.flush_events()


            # -----------------------------------------
            # Maintain ~50 Hz
            # -----------------------------------------

            loop_duration = (
                    time.perf_counter()
                    - loop_start
            )

            sleep_time = (
                    PLOT_PERIOD
                    - loop_duration
            )

            if sleep_time > 0:
                time.sleep(
                    sleep_time
                )


    except KeyboardInterrupt:

        print("Stopping...")


    finally:

        # Safety
        exo.set_torque(
            Side.LEFT,
            0.0
        )

        exo.set_torque(
            Side.RIGHT,
            0.0
        )

        plt.ioff()
        plt.show()