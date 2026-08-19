params = {
    "enabled": 0,

    # Input torque filtering
    "input_hp_cutoff_hz": 3.7,       # high-pass cutoff for bias/drift removal
    "input_lp_cutoff_hz": 5.0,       # low-pass cutoff for load-cell torque signal

    # Controller filtering
    "derivative_lp_cutoff_hz": 3.7,  # low-pass cutoff for torque derivative
    "friction_lp_cutoff_hz": 5.0,    # low-pass cutoff for friction compensation

    # PD gains
    "kp": 0.1,
    "kd": 0.0001,

    # Friction compensation
    "comp_torque": 0.08,
    "trigger_on_trq": 0.025,
    "trigger_off_trq": 0.010,

    # Output safety limit
    "max_abs_out_trq": 0.5,
}