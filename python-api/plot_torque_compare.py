#!/usr/bin/env python3

import struct
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks


HEADER_SIZE = 16
RECORD_FORMAT = "<7I4f2f2f2f4B"
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)

LEVER_ARM = 0.055


def read_log(filename):
    raw = Path(filename).read_bytes()
    n = (len(raw) - HEADER_SIZE) // RECORD_SIZE

    time_us = []
    load_cells = []
    encoder_left = []
    encoder_right = []

    for i in range(n):
        r = struct.unpack_from(
            RECORD_FORMAT,
            raw,
            HEADER_SIZE + i * RECORD_SIZE
        )

        time_us.append(r[0])
        load_cells.append(r[7:11])
        encoder_left.append(r[11])
        encoder_right.append(r[12])

    time_us = np.array(time_us, dtype=np.uint64)
    time = (time_us - time_us[0]) * 1e-6

    return (
        time,
        np.array(load_cells),
        np.array(encoder_left),
        np.array(encoder_right)
    )


def analyze_file(filename, side):
    time, loads, enc_left, enc_right = read_log(filename)

    if side == "left":
        encoder = enc_left
        torque = (loads[:, 0] - loads[:, 1]) * LEVER_ARM
    else:
        encoder = enc_right
        torque = (loads[:, 2] - loads[:, 3]) * LEVER_ARM

    # Smooth encoder for gait detection
    window = 31
    kernel = np.ones(window) / window
    encoder_smooth = np.convolve(encoder, kernel, mode="same")

    # Estimate sampling period
    dt = np.median(np.diff(time))

    # Detect gait-cycle peaks
    prominence = 0.2 * (
            np.percentile(encoder_smooth, 99)
            - np.percentile(encoder_smooth, 1)
    )

    peaks, _ = find_peaks(
        encoder_smooth,
        distance=int(0.6 / dt),
        prominence=prominence
    )

    # Normalize every gait cycle to 0–100%
    gait = np.linspace(0, 100, 201)
    cycles = []

    for start, end in zip(peaks[:-1], peaks[1:]):

        duration = time[end] - time[start]

        if duration <= 0 or duration > 2.0:
            continue

        phase = np.linspace(0, 100, end - start + 1)

        cycle = np.interp(
            gait,
            phase,
            torque[start:end + 1]
        )

        cycles.append(cycle)

    cycles = np.array(cycles)

    if len(cycles) == 0:
        raise RuntimeError(
            f"No gait cycles found in {filename}"
        )

    mean = np.mean(cycles, axis=0)
    std = np.std(cycles, axis=0)

    print(
        f"{filename}: {len(cycles)} gait cycles"
    )

    return gait, mean, std


parser = argparse.ArgumentParser()

parser.add_argument("file1")
parser.add_argument("file2")

parser.add_argument(
    "--side",
    choices=["left", "right"],
    default="left"
)

args = parser.parse_args()


gait1, mean1, std1 = analyze_file(
    args.file1,
    args.side
)

gait2, mean2, std2 = analyze_file(
    args.file2,
    args.side
)


# Plot

plt.plot(
    gait1,
    mean1,
    linewidth=2,
    label=Path(args.file1).stem
)

plt.fill_between(
    gait1,
    mean1 - std1,
    mean1 + std1,
    alpha=0.2
)

plt.plot(
    gait2,
    mean2,
    linewidth=2,
    label=Path(args.file2).stem
)

plt.fill_between(
    gait2,
    mean2 - std2,
    mean2 + std2,
    alpha=0.2
)

plt.axhline(0, linewidth=0.8)

plt.xlim(0, 100)

plt.xlabel("Gait cycle [%]")
plt.ylabel("Interaction torque [N m]")

plt.title(
    f"{args.side.capitalize()} interaction torque"
)

plt.legend()
plt.grid()

plt.show()