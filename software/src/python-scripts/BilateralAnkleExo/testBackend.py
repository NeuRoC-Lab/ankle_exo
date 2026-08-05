from backend import AnkleExoBackend
from time import sleep
import time
import math

class PDControl:
    def __init__(self, p_gain=1.0, d_gain=1.0):
        self.prev_error = None
        self.prev_time = None

        self.p_gain = p_gain
        self.d_gain = d_gain
        self.curr_torque = 0.0
        self.prev_torque = 0.0

    def update(self, error):
        current_time = time.perf_counter()

        if self.prev_time is None:
            dt = 0.0
        else:
            dt = current_time - self.prev_time

        proportional = self.p_gain * error

        if self.prev_error is None or dt <= 0:
            derivative = 0.0
        else:
            derivative = self.d_gain * (error - self.prev_error) / dt

        output = proportional + derivative

        self.prev_error = error
        self.prev_time = current_time

        return output

    def diff_contribution(self, dt):
        if dt <= 0:
            return 0.0

        torque_rate = (self.curr_torque - self.prev_torque) / dt
        return self.d_gain * torque_rate
    def prop_contribution(self):
        return self.p_gain * self.curr_torque


# maybe implement the method to have a context manager so I dont have to bother with connec() / disconnect() and checking if its properly connected
# look online what special methods are required for that


def symmetric_sigmoid(x):
    return (2.0 / (1.0 + math.exp(-x)) - 1.0)*0.1

with AnkleExoBackend() as my_exo:
    controller = PDControl(p_gain=1.0, d_gain=1.0)
    my_exo.zero_motor(2)
    my_exo.start_motor(2)
    try:
        while True:
            my_exo.get_pending_snapshots()
            if my_exo.get_sensor_values() is not None:
                l1 = my_exo.get_sensor_values()["left_loadcells"]
                l2 = my_exo.get_sensor_values()["right_loadcells"]
                # implementing a very simple PD control loop here
                torque_error = l1 - l2 - 18
                clamped_torque = symmetric_sigmoid(torque_error)
                fix_torque = controller.update(clamped_torque)
                my_exo.set_motor_command(2,0.0,0.0,0.0,0.0,-clamped_torque)
                #print(torque_error)
                time.sleep(0.01)
                #my_exo.set_motor_command(2,0.0,0.0,0.0,0.0,.0)

    except KeyboardInterrupt:
        my_exo.set_motor_command(2,0.0,0.0,0.0,0.0,0.0)
        my_exo.stop_motor(2)
        pass

