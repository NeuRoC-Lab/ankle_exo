#!/usr/bin/env python3
"""
Plot interaction torque and encoder position, detect gait cycles from the encoder,
and generate an average interaction-torque profile over 0-100% gait.

Interaction torque:
    left:  tau = (load0 - load1) * 0.035
    right: tau = (load2 - load3) * 0.035

Gait cycles are detected peak-to-peak (or trough-to-trough) from the selected
encoder. Each detected cycle is interpolated to a common 0-100% gait axis,
then the mean and standard deviation of interaction torque are calculated.
"""

import argparse
import struct
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks


HEADER_FORMAT = "<8sHHI"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

# 7 uint32:
# timeUs
# loadCellSequence, encoderSequence
# leftMotorSequence, rightMotorSequence
# leftCommandSequence, rightCommandSequence
#
# Then:
# 4 load-cell floats
# 2 encoder floats
# 2 command-torque floats
# 2 motor-torque floats
# 4 uint8 motor status bytes
RECORD_FORMAT = "<7I4f2f2f2f4B"
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)

EXPECTED_MAGIC = b"ANKLOG01"
EXPECTED_VERSION = 1
EXPECTED_RECORD_SIZE = 72


def read_log(filename: Path):
    raw = filename.read_bytes()

    if len(raw) < HEADER_SIZE:
        raise ValueError("File is too small to contain the 16-byte log header.")

    magic, version, record_size, reserved = struct.unpack_from(
        HEADER_FORMAT, raw, 0
    )

    if magic != EXPECTED_MAGIC:
        raise ValueError(
            f"Bad magic: {magic!r}; expected {EXPECTED_MAGIC!r}. "
            "This may not be an ANKLOG01 file."
        )

    if version != EXPECTED_VERSION:
        raise ValueError(
            f"Unsupported log version {version}; expected {EXPECTED_VERSION}."
        )

    if record_size != EXPECTED_RECORD_SIZE:
        raise ValueError(
            f"Header says record size is {record_size} bytes; "
            f"expected {EXPECTED_RECORD_SIZE}."
        )

    if RECORD_SIZE != EXPECTED_RECORD_SIZE:
        raise RuntimeError(
            f"Python struct definition is {RECORD_SIZE} bytes, expected 72."
        )

    payload = raw[HEADER_SIZE:]

    n_records = len(payload) // RECORD_SIZE
    trailing = len(payload) % RECORD_SIZE

    if n_records == 0:
        raise ValueError("No complete data records found.")

    if trailing:
        print(
            f"Warning: ignoring {trailing} trailing byte(s) after "
            f"{n_records} complete records."
        )

    # Preallocate arrays.
    time_us = np.empty(n_records, dtype=np.uint32)
    load_cells = np.empty((n_records, 4), dtype=float)
    encoder_left = np.empty(n_records, dtype=float)
    encoder_right = np.empty(n_records, dtype=float)
    left_command_torque = np.empty(n_records, dtype=float)
    right_command_torque = np.empty(n_records, dtype=float)
    left_motor_torque = np.empty(n_records, dtype=float)
    right_motor_torque = np.empty(n_records, dtype=float)

    for i in range(n_records):
        offset = HEADER_SIZE + i * RECORD_SIZE
        r = struct.unpack_from(RECORD_FORMAT, raw, offset)

        time_us[i] = r[0]

        # r[1:7] are sequence numbers.
        load_cells[i, :] = r[7:11]

        encoder_left[i] = r[11]
        encoder_right[i] = r[12]

        left_command_torque[i] = r[13]
        right_command_torque[i] = r[14]

        left_motor_torque[i] = r[15]
        right_motor_torque[i] = r[16]

    return {
        "time_us": time_us,
        "load0": load_cells[:, 0],
        "load1": load_cells[:, 1],
        "load2": load_cells[:, 2],
        "load3": load_cells[:, 3],
        "encoderLeft": encoder_left,
        "encoderRight": encoder_right,
        "leftCommandTorque": left_command_torque,
        "rightCommandTorque": right_command_torque,
        "leftMotorTorque": left_motor_torque,
        "rightMotorTorque": right_motor_torque,
    }


def unwrap_time_us(time_us):
    """
    Convert uint32 microsecond timestamps to continuous seconds,
    including support for micros() wraparound.
    """
    t = time_us.astype(np.uint64)

    corrected = np.empty_like(t)
    corrected[0] = t[0]

    wrap_add = np.uint64(0)
    wrap_value = np.uint64(2**32)

    for i in range(1, len(t)):
        if t[i] < t[i - 1]:
            wrap_add += wrap_value
        corrected[i] = t[i] + wrap_add

    seconds = corrected.astype(np.float64) * 1e-6
    seconds -= seconds[0]

    return seconds


def smooth_signal(x, window):
    """
    Simple centered moving-average smoother with edge padding.

    Set window=1 for no smoothing.
    """
    if window <= 1:
        return x.copy()

    if window % 2 == 0:
        window += 1

    pad = window // 2
    padded = np.pad(x, (pad, pad), mode="edge")
    kernel = np.ones(window, dtype=float) / window

    return np.convolve(padded, kernel, mode="valid")



def moving_average(x, window):
    """Centered moving average used for gait-event detection."""
    window = int(window)

    if window <= 1:
        return x.copy()

    if window % 2 == 0:
        window += 1

    pad = window // 2
    padded = np.pad(x, (pad, pad), mode="edge")
    kernel = np.ones(window, dtype=float) / float(window)

    return np.convolve(padded, kernel, mode="valid")


def normalize_cycles(time_s, torque, event_indices, max_cycle_s, gait_points):
    """Interpolate accepted event-to-event torque cycles onto 0-100% gait."""
    gait_percent = np.linspace(0.0, 100.0, gait_points)
    cycles = []
    durations = []
    accepted_pairs = []

    for start_idx, end_idx in zip(event_indices[:-1], event_indices[1:]):
        duration = time_s[end_idx] - time_s[start_idx]

        if duration <= 0.0 or duration > max_cycle_s:
            continue

        cycle_t = time_s[start_idx:end_idx + 1]
        cycle_tau = torque[start_idx:end_idx + 1]

        finite = np.isfinite(cycle_t) & np.isfinite(cycle_tau)
        cycle_t = cycle_t[finite]
        cycle_tau = cycle_tau[finite]

        if len(cycle_t) < 3:
            continue

        cycle_phase = (
                (cycle_t - cycle_t[0])
                / (cycle_t[-1] - cycle_t[0])
                * 100.0
        )

        tau_normalized = np.interp(
            gait_percent,
            cycle_phase,
            cycle_tau,
        )

        cycles.append(tau_normalized)
        durations.append(duration)
        accepted_pairs.append((start_idx, end_idx))

    if not cycles:
        raise RuntimeError(
            "No valid gait cycles were found. Try changing --cycle-event, "
            "--min-cycle-s, --max-cycle-s, or --peak-prominence."
        )

    return (
        gait_percent,
        np.asarray(cycles),
        np.asarray(durations),
        accepted_pairs,
    )

def main():
    parser = argparse.ArgumentParser(
        description="Plot interaction torque and encoder position."
    )

    parser.add_argument("binfile", type=Path, help="ANKLOG01 binary log file")

    parser.add_argument(
        "--side",
        choices=("left", "right"),
        default="left",
        help="Which encoder to use. Default: left",
    )




    parser.add_argument(
        "--encoder-units",
        choices=("rad", "deg"),
        default="rad",
        help="Units stored in encoderLeft/Right. Default: rad",
    )

    parser.add_argument(
        "--angle-scale",
        type=float,
        default=1.0,
        help=(
            "Additional multiplier applied to encoder angle before computing velocity. "
            "Useful if encoder values are counts or require calibration. Default: 1."
        ),
    )




    parser.add_argument(
        "--cycle-event",
        choices=("peak", "trough"),
        default="peak",
        help=(
            "Encoder event used to define gait-cycle boundaries. "
            "Default: peak."
        ),
    )

    parser.add_argument(
        "--min-cycle-s",
        type=float,
        default=0.6,
        help=(
            "Minimum allowed time between encoder events in seconds. "
            "Default: 0.6 s."
        ),
    )

    parser.add_argument(
        "--max-cycle-s",
        type=float,
        default=2.0,
        help=(
            "Maximum accepted gait-cycle duration in seconds. "
            "Default: 2.0 s."
        ),
    )

    parser.add_argument(
        "--encoder-smooth",
        type=int,
        default=31,
        help=(
            "Moving-average window, in samples, used only for gait-event "
            "detection. Default: 31."
        ),
    )

    parser.add_argument(
        "--peak-prominence",
        type=float,
        default=None,
        help=(
            "Required encoder peak prominence. By default it is chosen "
            "automatically from the encoder range."
        ),
    )

    parser.add_argument(
        "--gait-points",
        type=int,
        default=201,
        help="Number of points in the normalized 0-100%% gait profile. Default: 201.",
    )

    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Optional path to save the plot, e.g. impedance.png",
    )

    args = parser.parse_args()

    data = read_log(args.binfile)
    time_s = unwrap_time_us(data["time_us"])

    encoder_name = "encoderLeft" if args.side == "left" else "encoderRight"
    angle = data[encoder_name].astype(float) * args.angle_scale

    # Convert only for display. Event detection works in either degrees or radians.
    if args.encoder_units == "deg":
        angle_display = np.rad2deg(angle)
        angle_label = "Encoder position [deg]"
    else:
        angle_display = angle
        angle_label = "Encoder position [rad]"

    # ---------------------------------------------------------
    # Interaction torque from the two load cells on each side.
    # ---------------------------------------------------------

    LOAD_CELL_LEVER_ARM_M = 0.035

    if args.side == "left":
        force1 = data["load0"].astype(float)
        force2 = data["load1"].astype(float)
        loadcell_names = "load0 - load1"
    else:
        force1 = data["load2"].astype(float)
        force2 = data["load3"].astype(float)
        loadcell_names = "load2 - load3"

    torque = (force1 - force2) * LOAD_CELL_LEVER_ARM_M

    # ---------------------------------------------------------
    # Detect gait-cycle boundaries from encoder extrema.
    # ---------------------------------------------------------

    angle_for_detection = moving_average(
        angle,
        args.encoder_smooth,
    )

    dt = np.diff(time_s)
    positive_dt = dt[np.isfinite(dt) & (dt > 0.0)]

    if positive_dt.size == 0:
        raise RuntimeError("Could not determine sampling period from timestamps.")

    median_dt = np.median(positive_dt)
    min_distance_samples = max(
        1,
        int(round(args.min_cycle_s / median_dt)),
    )

    if args.peak_prominence is None:
        # Robust automatic choice based on the central 98% encoder range.
        lo, hi = np.nanpercentile(angle_for_detection, [1.0, 99.0])
        encoder_span = hi - lo
        prominence = 0.20 * encoder_span
    else:
        prominence = args.peak_prominence

    detection_signal = (
        angle_for_detection
        if args.cycle_event == "peak"
        else -angle_for_detection
    )

    event_indices, properties = find_peaks(
        detection_signal,
        distance=min_distance_samples,
        prominence=prominence,
    )

    if len(event_indices) < 2:
        raise RuntimeError(
            f"Only {len(event_indices)} gait event(s) detected. "
            "Try reducing --peak-prominence or --min-cycle-s, "
            "or switch --cycle-event."
        )

    gait_percent, cycles, durations, accepted_pairs = normalize_cycles(
        time_s=time_s,
        torque=torque,
        event_indices=event_indices,
        max_cycle_s=args.max_cycle_s,
        gait_points=args.gait_points,
    )

    mean_torque = np.mean(cycles, axis=0)
    std_torque = np.std(cycles, axis=0, ddof=1) if len(cycles) > 1 else np.zeros_like(mean_torque)

    print(f"Records:             {len(time_s)}")
    print(f"Duration:            {time_s[-1]:.3f} s")
    print(f"Side:                {args.side}")
    print(f"Encoder:             {encoder_name}")
    print(f"Load-cell pair:      {loadcell_names}")
    print(f"Lever arm:           {LOAD_CELL_LEVER_ARM_M:.3f} m")
    print(f"Cycle event:         {args.cycle_event}")
    print(f"Peak prominence:     {prominence:.4g}")
    print(f"Detected events:     {len(event_indices)}")
    print(f"Accepted gait cycles:{len(cycles)}")
    print(f"Mean cycle duration: {np.mean(durations):.3f} s")
    print(f"Mean cadence:        {60.0 / np.mean(durations):.1f} cycles/min")

    # ---------------------------------------------------------
    # Figure 1: raw data with detected gait boundaries.
    # ---------------------------------------------------------

    fig1, ax_torque = plt.subplots(figsize=(11, 6))
    ax_angle = ax_torque.twinx()

    torque_line, = ax_torque.plot(
        time_s,
        torque,
        linewidth=1.0,
        label="Interaction torque",
    )

    angle_line, = ax_angle.plot(
        time_s,
        angle_display,
        linewidth=1.0,
        alpha=0.75,
        label="Encoder position",
    )

    accepted_event_indices = sorted(
        set(
            [pair[0] for pair in accepted_pairs]
            + [pair[1] for pair in accepted_pairs]
        )
    )

    for idx in accepted_event_indices:
        ax_torque.axvline(
            time_s[idx],
            linewidth=0.8,
            alpha=0.35,
        )

    ax_angle.scatter(
        time_s[event_indices],
        angle_display[event_indices],
        s=28,
        zorder=5,
        label=f"Detected {args.cycle_event}s",
    )

    ax_torque.set_xlabel("Time [s]")
    ax_torque.set_ylabel("Interaction torque [N m]")
    ax_angle.set_ylabel(angle_label)

    ax_torque.set_title(
        f"Detected gait cycles — {args.side} ankle"
    )

    ax_torque.grid(True, alpha=0.3)

    lines = [torque_line, angle_line]
    labels = [line.get_label() for line in lines]
    ax_torque.legend(lines, labels, loc="best")

    fig1.tight_layout()

    # ---------------------------------------------------------
    # Figure 2: normalized gait torque profiles.
    # ---------------------------------------------------------

    fig2, ax = plt.subplots(figsize=(9, 6))

    for cycle in cycles:
        ax.plot(
            gait_percent,
            cycle,
            linewidth=0.8,
            alpha=0.20,
        )

    ax.plot(
        gait_percent,
        mean_torque,
        linewidth=2.2,
        label=f"Mean torque, n={len(cycles)}",
    )

    ax.fill_between(
        gait_percent,
        mean_torque - std_torque,
        mean_torque + std_torque,
        alpha=0.20,
        label="±1 SD",
        )

    ax.axhline(0.0, linewidth=0.8, alpha=0.5)

    ax.set_xlim(0.0, 100.0)
    ax.set_xlabel("Gait cycle [%]")
    ax.set_ylabel("Interaction torque [N m]")
    ax.set_title(
        f"Average gait interaction torque — {args.side} ankle"
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    fig2.tight_layout()

    if args.save is not None:
        save_path = args.save
        stem = save_path.stem
        suffix = save_path.suffix if save_path.suffix else ".png"

        raw_path = save_path.with_name(f"{stem}_segmentation{suffix}")
        gait_path = save_path.with_name(f"{stem}_gait_average{suffix}")

        fig1.savefig(raw_path, dpi=200)
        fig2.savefig(gait_path, dpi=200)

        print(f"Saved segmentation plot to: {raw_path}")
        print(f"Saved gait-average plot to: {gait_path}")

    plt.show()


if __name__ == "__main__":
    main()
