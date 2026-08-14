
#!/usr/bin/env python3

import argparse
import math
import re
import statistics
import time
from collections import Counter

import matplotlib.pyplot as plt
import serial


BAUD_RATE = 115200

# 65535 = UINT16_MAX
# 32767 = INT16_MAX
BAD_VALUES = {65535, 32767}

RIGHT_ENCODER_PATTERN = re.compile(
    r"Right\s+encoder\s+position\s*:\s*(\d+)",
    re.IGNORECASE,
)


def plot_histogram(valid_values: list[int]) -> None:
    if not valid_values:
        print("No valid values available for histogram.")
        return

    minimum_value = min(valid_values)
    maximum_value = max(valid_values)
    mean_value = statistics.fmean(valid_values)
    median_value = statistics.median(valid_values)
    standard_deviation = statistics.pstdev(valid_values)

    count_range = maximum_value - minimum_value

    # Encoder values are integers, so bins are centered on integer counts.
    # For a very wide range, limit the number of bins so the figure
    # remains readable.
    if count_range <= 200:
        bins = [
            value - 0.5
            for value in range(
                minimum_value,
                maximum_value + 2,
                )
        ]
    else:
        bins = min(100, max(10, int(math.sqrt(len(valid_values)))))

    figure, axis = plt.subplots(figsize=(11, 7))

    axis.hist(
        valid_values,
        bins=bins,
        edgecolor="black",
        alpha=0.8,
    )

    axis.axvline(
        mean_value,
        linestyle="--",
        linewidth=2,
        label=f"Mean = {mean_value:.3f}",
    )

    axis.axvline(
        median_value,
        linestyle=":",
        linewidth=2,
        label=f"Median = {median_value:.3f}",
    )

    if standard_deviation > 0:
        axis.axvline(
            mean_value - standard_deviation,
            linestyle="--",
            linewidth=1.5,
            label=f"Mean - 1 SD = {mean_value - standard_deviation:.3f}",
            )

        axis.axvline(
            mean_value + standard_deviation,
            linestyle="--",
            linewidth=1.5,
            label=f"Mean + 1 SD = {mean_value + standard_deviation:.3f}",
            )

    axis.set_title(
        "Stationary Right Encoder Count Distribution"
    )
    axis.set_xlabel("Encoder count")
    axis.set_ylabel("Number of samples")
    axis.grid(True, axis="y", alpha=0.3)
    axis.legend()

    statistics_text = (
        f"Samples: {len(valid_values)}\n"
        f"Mean: {mean_value:.3f}\n"
        f"Std. dev.: {standard_deviation:.3f}\n"
        f"Min: {minimum_value}\n"
        f"Max: {maximum_value}\n"
        f"Peak-to-peak: {count_range}"
    )

    axis.text(
        0.98,
        0.97,
        statistics_text,
        transform=axis.transAxes,
        horizontalalignment="right",
        verticalalignment="top",
        bbox={
            "boxstyle": "round",
            "facecolor": "white",
            "alpha": 0.85,
        },
    )

    figure.tight_layout()
    plt.show()


def print_statistics(
        valid_values: list[int],
        invalid_values: list[int],
        malformed_lines: int,
        start_time: float,
) -> None:
    duration = time.perf_counter() - start_time

    valid_samples = len(valid_values)
    invalid_samples = len(invalid_values)
    total_samples = valid_samples + invalid_samples

    valid_percentage = (
        100.0 * valid_samples / total_samples
        if total_samples > 0
        else 0.0
    )

    invalid_percentage = (
        100.0 * invalid_samples / total_samples
        if total_samples > 0
        else 0.0
    )

    overall_sample_rate = (
        total_samples / duration
        if duration > 0.0
        else 0.0
    )

    valid_sample_rate = (
        valid_samples / duration
        if duration > 0.0
        else 0.0
    )

    print("\n========== Encoder Statistics ==========")
    print(f"Recording duration:          {duration:.3f} s")
    print(f"Parsed samples:              {total_samples}")
    print(f"Valid samples:               {valid_samples}")
    print(f"Invalid samples:             {invalid_samples}")
    print(f"Unrecognized lines:          {malformed_lines}")
    print(f"Valid-value percentage:      {valid_percentage:.3f}%")
    print(f"Invalid-value percentage:    {invalid_percentage:.3f}%")
    print(f"Overall sample rate:         {overall_sample_rate:.3f} samples/s")
    print(f"Valid sample rate:           {valid_sample_rate:.3f} samples/s")

    if not valid_values:
        print("\nNo valid values were recorded.")
        print("========================================")
        return

    mean_value = statistics.fmean(valid_values)
    median_value = statistics.median(valid_values)

    minimum_value = min(valid_values)
    maximum_value = max(valid_values)
    peak_to_peak = maximum_value - minimum_value

    population_variance = statistics.pvariance(valid_values)
    population_std_dev = statistics.pstdev(valid_values)

    if valid_samples >= 2:
        sample_variance = statistics.variance(valid_values)
        sample_std_dev = statistics.stdev(valid_values)
    else:
        sample_variance = 0.0
        sample_std_dev = 0.0

    value_counts = Counter(valid_values)
    most_common_value, most_common_count = value_counts.most_common(1)[0]

    most_common_percentage = (
            100.0 * most_common_count / valid_samples
    )

    distinct_values = len(value_counts)

    differences = [
        current - previous
        for previous, current in zip(
            valid_values,
            valid_values[1:],
        )
    ]

    absolute_differences = [
        abs(difference)
        for difference in differences
    ]

    changed_samples = sum(
        difference != 0
        for difference in differences
    )

    unchanged_samples = len(differences) - changed_samples

    changed_percentage = (
        100.0 * changed_samples / len(differences)
        if differences
        else 0.0
    )

    if absolute_differences:
        mean_absolute_change = statistics.fmean(
            absolute_differences
        )

        median_absolute_change = statistics.median(
            absolute_differences
        )

        maximum_jump = max(absolute_differences)

        rms_change = math.sqrt(
            statistics.fmean(
                difference * difference
                for difference in differences
            )
        )
    else:
        mean_absolute_change = 0.0
        median_absolute_change = 0.0
        maximum_jump = 0
        rms_change = 0.0

    print("\n---------- Stationary Signal -----------")
    print(f"Mean:                        {mean_value:.6f} counts")
    print(f"Median:                      {median_value:.6f} counts")
    print(f"Minimum:                     {minimum_value} counts")
    print(f"Maximum:                     {maximum_value} counts")
    print(f"Peak-to-peak fluctuation:    {peak_to_peak} counts")
    print(f"Population std. deviation:   {population_std_dev:.6f} counts")
    print(f"Population variance:         {population_variance:.6f} counts²")
    print(f"Sample std. deviation:       {sample_std_dev:.6f} counts")
    print(f"Sample variance:             {sample_variance:.6f} counts²")
    print(f"Distinct valid values:       {distinct_values}")
    print(f"Most common value:           {most_common_value} counts")
    print(f"Most common value frequency: {most_common_percentage:.3f}%")

    print("\n---------- Sample-to-Sample ------------")
    print(f"Compared sample pairs:       {len(differences)}")
    print(f"Changed sample pairs:        {changed_samples}")
    print(f"Unchanged sample pairs:      {unchanged_samples}")
    print(f"Change rate:                 {changed_percentage:.3f}%")
    print(f"Mean absolute change:        {mean_absolute_change:.6f} counts")
    print(f"Median absolute change:      {median_absolute_change:.6f} counts")
    print(f"RMS sample change:           {rms_change:.6f} counts")
    print(f"Maximum single-sample jump:  {maximum_jump} counts")

    print("\n---------- Most Common Values ----------")

    for value, count in value_counts.most_common(10):
        percentage = 100.0 * count / valid_samples

        print(
            f"{value:5d} counts: "
            f"{count:7d} samples "
            f"({percentage:7.3f}%)"
        )

    if invalid_values:
        print("\n---------- Invalid Values --------------")

        invalid_counts = Counter(invalid_values)

        for value, count in invalid_counts.most_common():
            percentage = 100.0 * count / invalid_samples

            print(
                f"{value:5d}: "
                f"{count:7d} samples "
                f"({percentage:7.3f}% of invalid samples)"
            )

    print("========================================")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Measure stationary right-encoder stability and invalid-value "
            "frequency from an Arduino serial stream."
        )
    )

    parser.add_argument(
        "port",
        help=(
            "Serial port, such as /dev/cu.usbmodem14201 "
            "or COM6."
        ),
    )

    parser.add_argument(
        "--show-values",
        action="store_true",
        help="Print every parsed right-encoder value.",
    )

    parser.add_argument(
        "--show-changes",
        action="store_true",
        help=(
            "Print a message whenever the valid encoder value changes."
        ),
    )

    parser.add_argument(
        "--save-histogram",
        metavar="FILE",
        help=(
            "Save the histogram to a file, for example "
            "encoder_histogram.png."
        ),
    )

    args = parser.parse_args()

    valid_values: list[int] = []
    invalid_values: list[int] = []
    malformed_lines = 0

    previous_valid_value: int | None = None

    print(f"Opening {args.port} at {BAUD_RATE} baud...")

    with serial.Serial(
            port=args.port,
            baudrate=BAUD_RATE,
            timeout=1.0,
    ) as serial_port:
        time.sleep(2.0)
        serial_port.reset_input_buffer()

        print("Recording stationary right-encoder values.")
        print(f"Invalid values: {sorted(BAD_VALUES)}")
        print("Press Ctrl+C to stop and show statistics.\n")

        start_time = time.perf_counter()

        try:
            while True:
                raw_line = serial_port.readline()

                if not raw_line:
                    continue

                line = raw_line.decode(
                    "utf-8",
                    errors="replace",
                ).strip()

                match = RIGHT_ENCODER_PATTERN.search(line)

                if match is None:
                    malformed_lines += 1
                    continue

                right_encoder = int(match.group(1))

                if right_encoder in BAD_VALUES:
                    invalid_values.append(right_encoder)

                    if args.show_values:
                        print(f"{right_encoder:5d}  INVALID")

                    continue

                valid_values.append(right_encoder)

                if args.show_values:
                    print(f"{right_encoder:5d}  VALID")

                if (
                        args.show_changes
                        and previous_valid_value is not None
                        and right_encoder != previous_valid_value
                ):
                    change = right_encoder - previous_valid_value

                    print(
                        f"CHANGE: "
                        f"{previous_valid_value} -> {right_encoder} "
                        f"(delta={change:+d})"
                    )

                previous_valid_value = right_encoder

        except KeyboardInterrupt:
            print_statistics(
                valid_values=valid_values,
                invalid_values=invalid_values,
                malformed_lines=malformed_lines,
                start_time=start_time,
            )

    if valid_values:
        if args.save_histogram:
            # Build the figure without blocking, save it, then show it.
            minimum_value = min(valid_values)
            maximum_value = max(valid_values)
            count_range = maximum_value - minimum_value

            if count_range <= 200:
                bins = [
                    value - 0.5
                    for value in range(
                        minimum_value,
                        maximum_value + 2,
                        )
                ]
            else:
                bins = min(
                    100,
                    max(10, int(math.sqrt(len(valid_values)))),
                )

            plt.figure(figsize=(11, 7))
            plt.hist(
                valid_values,
                bins=bins,
                edgecolor="black",
                alpha=0.8,
            )
            plt.title("Stationary Right Encoder Count Distribution")
            plt.xlabel("Encoder count")
            plt.ylabel("Number of samples")
            plt.grid(True, axis="y", alpha=0.3)
            plt.tight_layout()
            plt.savefig(
                args.save_histogram,
                dpi=200,
            )
            plt.close()

            print(
                f"Histogram saved to: {args.save_histogram}"
            )

        plot_histogram(valid_values)


if __name__ == "__main__":
    main()

