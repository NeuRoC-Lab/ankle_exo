#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt

# Import this from your spectrum script.
from rigol_spectrum import read_rigol_csv


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot voltage versus time from a Rigol CSV file."
    )
    parser.add_argument(
        "csv_file",
        type=Path,
        help="Path to the Rigol CSV file.",
    )
    parser.add_argument(
        "--milliseconds",
        action="store_true",
        help="Display time in milliseconds instead of seconds.",
    )
    parser.add_argument(
        "--save",
        type=Path,
        help="Optional output image path, such as voltage_time.png.",
    )
    args = parser.parse_args()

    time, voltage = read_rigol_csv(args.csv_file)

    if args.milliseconds:
        plotted_time = time * 1000
        time_label = "Time (ms)"
    else:
        plotted_time = time
        time_label = "Time (s)"

    plt.figure(figsize=(11, 6))
    plt.plot(plotted_time, voltage, linewidth=0.8)
    plt.xlabel(time_label)
    plt.ylabel("Voltage (V)")
    plt.title(f"Voltage vs time — {args.csv_file.name}")
    plt.grid(True)
    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=200)
        print(f"Saved plot to: {args.save}")

    plt.show()


if __name__ == "__main__":
    main()