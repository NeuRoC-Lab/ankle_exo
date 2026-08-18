"""
High-level API for the ankle exoskeleton.

This class owns:
- shared encoder state
- shared load-cell state
- power state
- left and right motor state
- BluetoothManager

Bluetooth-specific details such as UUIDs and BLE queues remain
inside BluetoothManager.
"""

import threading
import time

from .bluetooth import (
    BluetoothManager,
    Side,
)

from .peripherals import (
    Encoders,
    LoadCells,
    Power,
    Motor,
    MotorCommand,
    MotorControlCmd,
    SDLoggerControlCmd,
    TransparentControlCommand,
    IntermediateTorque,
)


class Exoskeleton:

    def __init__(self):

        # -------------------------------------------------
        # Thread / connection state
        # -------------------------------------------------

        self.stop_event = threading.Event()
        self.connected = False


        # -------------------------------------------------
        # Shared sensor state
        # -------------------------------------------------

        self.encoders = Encoders()
        self.loadcells = LoadCells()
        self.intermediate_torque = (
            IntermediateTorque()
        )
        self.power = Power()


        # -------------------------------------------------
        # Motor state
        # -------------------------------------------------

        self.motors = {
            Side.LEFT: Motor(),
            Side.RIGHT: Motor(),
        }


        # -------------------------------------------------
        # Current desired MIT commands
        #
        # Keep one independent command state for each motor.
        # This allows:
        #
        #   set_torque(LEFT, ...)
        #
        # without overwriting RIGHT's position/kp/etc.
        # -------------------------------------------------

        self.commands = {
            Side.LEFT: MotorCommand(),
            Side.RIGHT: MotorCommand(),
        }

        self.transparent = {
            Side.LEFT : TransparentControlCommand(),
            Side.RIGHT : TransparentControlCommand(),
        }


        # -------------------------------------------------
        # Bluetooth transport
        # -------------------------------------------------

        self.bluetooth = BluetoothManager(
            encoders=self.encoders,
            loadcells=self.loadcells,
            power=self.power,

            left_motor=self.motors[
                Side.LEFT
            ],

            right_motor=self.motors[
                Side.RIGHT
            ],

            intermediate_torque=(
                self.intermediate_torque
            ),

            stop_event=self.stop_event,
        )


    # =====================================================
    # CONNECTION
    # =====================================================

    def connect(self):

        if self.connected:
            print(
                "Exoskeleton already connected"
            )
            return

        self.stop_event.clear()

        print(
            "Connecting to exoskeleton..."
        )

        self.bluetooth.connect()

        self.connected = True

        print(
            "Exoskeleton connection started"
        )


    def disconnect(self):

        if not self.connected:
            return

        print(
            "Disconnecting exoskeleton..."
        )

        try:

            # Remove old pending commands before
            # sending the final safety commands.
            self.bluetooth.clear_pending_commands()


            # Reset continuous MIT commands first.
            for side in Side:
                self.reset_command(side)


            # Then request both motors to exit motor mode.
            for side in Side:
                self.stop_motor(side)


            # Give BluetoothManager some time to send
            # queued shutdown commands.
            deadline = (
                    time.perf_counter()
                    +
                    5.0
            )

            while (
                    self.bluetooth.has_pending_commands()
                    and
                    time.perf_counter() < deadline
            ):
                time.sleep(
                    0.005
                )


            if (
                    self.bluetooth
                            .has_pending_commands()
            ):
                print(
                    "Warning: some BLE commands "
                    "were not sent before disconnect"
                )


        finally:

            self.bluetooth.disconnect()

            self.connected = False

            print(
                "Exoskeleton disconnected"
            )


    # =====================================================
    # MOTOR CONTROLS
    # =====================================================

    def start_motor(
            self,
            side: Side,
    ):
        self._check_connection()

        self.bluetooth.queue_motor_control(
            side,
            MotorControlCmd.ENTER_MOTOR_MODE,
        )


    def stop_motor(
            self,
            side: Side,
    ):
        self._check_connection()

        self.bluetooth.queue_motor_control(
            side,
            MotorControlCmd.EXIT_MOTOR_MODE,
        )


    # =====================================================
    # SD LOGGER
    # =====================================================

    def start_recording(self):

        self._check_connection()

        self.bluetooth.queue_sd_command(
            SDLoggerControlCmd.START_RECORDING
        )


    def stop_recording(self):

        self._check_connection()

        self.bluetooth.queue_sd_command(
            SDLoggerControlCmd.STOP_RECORDING
        )


    # =====================================================
    # TRANSPARENT MODE CONTROL
    # =====================================================

    def update_transparent_params(self, side: Side, updates: dict):
        self._check_connection()

        try:
            command = TransparentControlCommand() # CREATE A SNAPSHOT INSTEAD OF COPYING BY REFERENCE

            command.enabled = updates["enabled"]
            command.kp = updates["kp"]
            command.kd = updates["kd"]
            command.input_hp_cutoff_hz = updates["input_hp_cutoff_hz"]
            command.input_lp_cutoff_hz = updates["input_lp_cutoff_hz"]
            command.derivative_lp_cutoff_hz = updates["derivative_lp_cutoff_hz"]
            command.friction_lp_cutoff_hz = updates["friction_lp_cutoff_hz"]
            command.comp_torque = updates["comp_torque"]
            command.trigger_on_trq = updates["trigger_on_trq"]
            command.trigger_off_trq = updates["trigger_off_trq"]
            command.max_abs_out_trq = updates["max_abs_out_trq"]

            self.bluetooth.queue_transparent_command(
                side,
                command
            )

        except KeyError:
            print("Invalid or missing parameters")

    # =====================================================
    # MOTOR MIT COMMAND to fix (OUTDATED)
    # =====================================================

    def _send_current_command(
            self,
            side: Side,
    ):
        """
        Send the complete current MIT command for one side.
        """

        self._check_connection()

        command = self.commands[
            side
        ]

        self.bluetooth.queue_motor_command(
            side,
            command,
        )

    #DEPRECATED
    def set_command(
            self,
            side: Side,
            #position=None,
            #velocity=None,
            torque=None,
            #kp=None,
            #kd=None,
    ):
        """
        Update one or more components of the desired MIT
        command while preserving the previous values of the
        other components.
        """

        self._check_connection()

        command = self.commands[
            side
        ]

        """
        if position is not None:
            command.position = float(
                position
            )

        if velocity is not None:
            command.velocity = float(
                velocity
            )
        """
        if torque is not None:
            command.torque = float(
                torque
            )
        """
        if kp is not None:
            command.kp = float(
                kp
            )

        if kd is not None:
            command.kd = float(
                kd
            )
        """

        self._send_current_command(
            side
        )

    """
    def set_position(
            self,
            side: Side,
            position,
    ):
        self.set_command(
            side,
            position=position,
        )


    def set_velocity(
            self,
            side: Side,
            velocity,
    ):
        self.set_command(
            side,
            velocity=velocity,
        )
    """

    def set_torque(
            self,
            side: Side,
            torque,
    ):
        self.set_command(
            side,
            torque=torque,
        )

    """
    def set_kp(
            self,
            side: Side,
            kp,
    ):
        self.set_command(
            side,
            kp=kp,
        )


    def set_kd(
            self,
            side: Side,
            kd,
    ):
        self.set_command(
            side,
            kd=kd,
        )
    """

    def reset_command(
            self,
            side: Side,
    ):
        """
        Reset all MIT command parameters to zero.
        """

        self.commands[
            side
        ] = MotorCommand()

        self._send_current_command(
            side
        )


    def get_command(
            self,
            side: Side,
    ):
        """
        Return the current desired MIT command.
        """

        command = self.commands[
            side
        ]

        return {
            """
            "position":
                command.position,

            "velocity":
                command.velocity,
            "kp":
                command.kp,

            "kd":
                command.kd,
            """
            "torque":
                command.torque,
        }


    # =====================================================
    # MOTOR FEEDBACK
    # =====================================================

    def get_motor_values(
            self,
            side: Side,
    ):
        self._check_connection()

        motor = self.motors[
            side
        ]

        (
            #position,
            #velocity,
            torque,
            temperature,
            error,
        ) = motor.get_values()

        return {
            "torque":
                torque,

            "temperature":
                temperature,

            "error":
                error,
        }


    # =====================================================
    # ENCODERS
    # =====================================================

    # =====================================================
# INTERMEDIATE / FILTERED TORQUE
# =====================================================

    def get_intermediate_torque(
            self,
            side: Side):

        self._check_connection()

        left, right = (
            self.intermediate_torque
            .get_values()
        )

        if side == Side.LEFT:
            return left

        return right


    def get_controller_output_torque(
                self,
                side: Side,
        ):
        if side == Side.LEFT:
                return self.bluetooth.controller_output_torque.left

        if side == Side.RIGHT:
            return self.bluetooth.controller_output_torque.right

        raise ValueError(f"Invalid side: {side}")
    def get_encoder(
            self,
            side: Side,
    ):
        """
        Return encoder state for one side.

        Adjust this depending on the exact public interface
        you give Encoders.
        """

        return (
            self.encoders
            .get_values()[side]
        )


    # =====================================================
    # LOAD CELLS
    # =====================================================

    def get_loadcells(self):

        self._check_connection()

        (
            leftTorque,
            rightTorque,
        ) = self.loadcells.get_values()

        return {
            "left": leftTorque,
            "right": rightTorque,
        }

    """to fix 
    def get_loadcells_for_side(
            self,
            side: Side,
    ):
        '''
        Convenience method for retrieving the two load cells
        associated with a single side.
        '''

        values = (
            self.get_loadcells()
        )

        if side == Side.LEFT:

            return {
                "1": values[
                    "left1"
                ],
                "2": values[
                    "left2"
                ],
            }

        return {
            "1": values[
                "right1"
            ],
            "2": values[
                "right2"
            ],
        }
    """

    # =====================================================
    # POWER
    # =====================================================

    def get_power(self):

        self._check_connection()

        return self.power.get_values()




    # =====================================================
    # UTILITIES
    # =====================================================

    def _check_connection(self):

        if not self.connected:

            raise RuntimeError(
                "Exoskeleton is not connected. "
                "Call exo.connect() first."
            )


    # =====================================================
    # CONTEXT MANAGER
    # =====================================================

    def __enter__(self):

        self.connect()

        return self


    def __exit__(
            self,
            exc_type,
            exc_value,
            traceback,
    ):

        if exc_type is KeyboardInterrupt:
            print(
                "Keyboard interrupt: "
                "shutting down exoskeleton"
            )

        self.disconnect()