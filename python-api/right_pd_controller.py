import time
import numpy as np

from scipy.signal import butter, sosfilt, sosfilt_zi

from ankle_exo_api import Side


class RightPDController:
    """
    Right-leg transparent PD controller.

    Signal path:

        right1 - right2
              |
              v
        interaction torque
              |
              v
        high-pass filter
          fc = 0.5 Hz
              |
              v
        low-pass filter
          fc = 2.0 Hz
              |
              v
        filtered torque
              |
          +---+---+
          |       |
          P       derivative
          |       |
          +---+---+
              |
              v
           PD torque
              |
              v
        exo.set_torque(RIGHT)
    """

    LOAD_CELL_LEVER_ARM = 0.055

    def __init__(
            self,
            exo,
            kp=0.5,
            kd=0.01,
            sample_rate_hz=50.0,
            hp_cutoff_hz=0.5,
            lp_cutoff_hz=2.0,
            max_abs_torque=0.6,
    ):

        self.exo = exo

        self.kp = kp
        self.kd = kd

        self.sample_rate_hz = sample_rate_hz
        self.sample_period = 1.0 / sample_rate_hz

        self.hp_cutoff_hz = hp_cutoff_hz
        self.lp_cutoff_hz = lp_cutoff_hz

        self.max_abs_torque = max_abs_torque

        # ---------------------------------------------
        # 1st-order Butterworth high-pass
        # ---------------------------------------------

        self.hp_sos = butter(
            1,
            hp_cutoff_hz,
            btype="highpass",
            fs=sample_rate_hz,
            output="sos",
        )

        # ---------------------------------------------
        # 2nd-order Butterworth low-pass
        # ---------------------------------------------

        self.lp_sos = butter(
            2,
            lp_cutoff_hz,
            btype="lowpass",
            fs=sample_rate_hz,
            output="sos",
        )

        # Filter states
        self.hp_state = None
        self.lp_state = None

        # Controller states
        self.previous_filtered_torque = None
        self.previous_time = None

        # Latest values, useful for plotting/debugging
        self.measured_torque = 0.0
        self.highpass_torque = 0.0
        self.filtered_torque = 0.0
        self.torque_derivative = 0.0
        self.commanded_torque = 0.0


    # =========================================================
    # Reset
    # =========================================================

    def reset(self):

        self.hp_state = None
        self.lp_state = None

        self.previous_filtered_torque = None
        self.previous_time = None

        self.measured_torque = 0.0
        self.highpass_torque = 0.0
        self.filtered_torque = 0.0
        self.torque_derivative = 0.0
        self.commanded_torque = 0.0


    # =========================================================
    # Update
    # =========================================================

    def update(self):

        now = time.perf_counter()

        # ---------------------------------------------
        # Read RIGHT load cells
        # ---------------------------------------------

        forces = self.exo.get_loadcells()

        force1 = forces["left1"]
        force2 = forces["left2"]

        # Same torque calculation as firmware
        self.measured_torque = (
                (force1 - force2)
                * self.LOAD_CELL_LEVER_ARM
        )


        # ---------------------------------------------
        # Initialize
        # ---------------------------------------------

        if self.hp_state is None:

            # Initialize HP around the current input,
            # preventing the static offset from creating
            # a large startup transient.
            self.hp_state = (
                    sosfilt_zi(self.hp_sos)
                    * self.measured_torque
            )

            # HP output should begin around zero.
            self.lp_state = (
                    sosfilt_zi(self.lp_sos)
                    * 0.0
            )

            self.previous_filtered_torque = 0.0
            self.previous_time = now

            # Initially command zero
            self.exo.set_torque(
                Side.RIGHT,
                0.0
            )

            return 0.0


        # ---------------------------------------------
        # dt
        # ---------------------------------------------

        dt = now - self.previous_time

        if dt <= 0.0:
            return self.commanded_torque


        # ---------------------------------------------
        # HIGH-PASS
        # ---------------------------------------------

        hp_output, self.hp_state = sosfilt(
            self.hp_sos,
            [self.measured_torque],
            zi=self.hp_state,
        )

        self.highpass_torque = float(
            hp_output[0]
        )


        # ---------------------------------------------
        # LOW-PASS
        # ---------------------------------------------

        lp_output, self.lp_state = sosfilt(
            self.lp_sos,
            [self.highpass_torque],
            zi=self.lp_state,
        )

        self.filtered_torque = float(
            lp_output[0]
        )


        # ---------------------------------------------
        # Torque derivative
        # ---------------------------------------------

        self.torque_derivative = (
                                         self.filtered_torque
                                         - self.previous_filtered_torque
                                 ) / dt


        # ---------------------------------------------
        # PD controller
        #
        # error = desired torque - measured torque
        # desired interaction torque = 0
        # ---------------------------------------------

        error = -self.filtered_torque

        error_derivative = (
            -self.torque_derivative
        )

        feedback_torque = (
                self.kp * error
                +
                self.kd * error_derivative
        )


        # ---------------------------------------------
        # Clamp motor command
        # ---------------------------------------------

        self.commanded_torque = float(
            np.clip(
                feedback_torque,
                -self.max_abs_torque,
                self.max_abs_torque,
            )
        )


        # ---------------------------------------------
        # Send RIGHT motor command
        # ---------------------------------------------

        self.exo.set_torque(
            Side.RIGHT,
            self.commanded_torque
        )


        # ---------------------------------------------
        # Update controller state
        # ---------------------------------------------

        self.previous_filtered_torque = (
            self.filtered_torque
        )

        self.previous_time = now

        return self.commanded_torque


    # =========================================================
    # Convenient debug information
    # =========================================================

    def get_state(self):

        return {
            "measured_torque":
                self.measured_torque,

            "highpass_torque":
                self.highpass_torque,

            "filtered_torque":
                self.filtered_torque,

            "torque_derivative":
                self.torque_derivative,

            "commanded_torque":
                self.commanded_torque,
        }