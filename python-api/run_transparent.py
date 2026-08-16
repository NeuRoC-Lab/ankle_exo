from ankle_exo_api import Exoskeleton, Side
import time

params = {
    "enabled": 1,
    "kp": 0.5,
    "kd": 0.01,
    "a_derivative": 0.1,
    "a_friction": 0.10,
    "a_torque": 0.15,
    "comp_torque": 0.08,
    "trigger_on_trq": 0.025,
    "trigger_off_trq": 0.010,
    "max_abs_out_trq": 0.6,
}


with Exoskeleton() as exo:
    #exo.update_transparent_params(Side.LEFT,params)
    time.sleep(5)
    exo.start_motor(Side.LEFT)
    exo.start_motor(Side.RIGHT)
    exo.start_recording()
    exo.update_transparent_params(Side.LEFT,params)
    exo.update_transparent_params(Side.RIGHT,params)
    time.sleep(10)
    exo.stop_recording()
    time.sleep(5)

