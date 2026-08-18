params = {
    "enabled": 1,
    "kp": 0.8, #proportional gain
    "kd": 0.0001, #derivative gain
    "a_derivative":  0.05, # alpha filter term for the derivative contribution of PD
    "a_friction": 0.10, # alpha filter term for the friction compensation output
    "a_torque": 0.15, # alpha filter term for the raw load cell input signal
    "comp_torque": 0.08, # the compensatory torque provided by the friction module
    "trigger_on_trq": 0.025, # trigger on (hysterisis)
    "trigger_off_trq": 0.010, # trigger off (hysterisis)
    "max_abs_out_trq": 0.5, # max output torque (handle with care)
}