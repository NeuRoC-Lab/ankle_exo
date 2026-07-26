#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def parse_number(value: str) -> float:
    """
    Parse a number while tolerating decimal commas.
    """
    value = value.strip().replace("\ufeff", "")

    if "," in value and "." not in value:
        value = value.replace(",", ".")

    return float(value)


def read_rigol_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """
    Read time and voltage data from a Rigol CSV file.

    The function searches for rows whose first two columns are numeric,
    allowing it to skip metadata and column headings automatically.
    """
    time_values: list[float] = []
    voltage_values: list[float] = []

    with path.open("r", encoding="utf-8-sig", errors="replace") as file:
        sample = file.read(4096)
        file.seek(0)

        try:
            dialect = csv.Sniffer().sniff(sample, delimiters=",;\t")
            delimiter = dialect.delimiter
        except csv.Error:
            delimiter = ","

        reader = csv.reader(file, delimiter=delimiter)

        for row in reader:
            if len(row) < 2:
                continue

            try:
                time_value = parse_number(row[0])
                voltage_value = parse_number(row[1])
            except ValueError:
                continue

            if np.isfinite(time_value) and np.isfinite(voltage_value):
                time_values.append(time_value)
                voltage_values.append(voltage_value)

    if len(time_values) < 2:
        raise ValueError(
            "Could not find at least two numeric time/voltage rows in the CSV."
        )

    time_array = np.asarray(time_values, dtype=float)
    voltage_array = np.asarray(voltage_values, dtype=float)

    order = np.argsort(time_array)
    time_array = time_array[order]
    voltage_array = voltage_array[order]

    return time_array, voltage_array


def calculate_spectrogram(
        time: np.ndarray,
        voltage: np.ndarray,
        window_samples: int,
        overlap_fraction: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Calculate a one-sided peak-amplitude spectrogram.

    Each column in the returned amplitude matrix represents the spectrum of
    one time window.

    Returns:
        frequencies:
            Frequency of each FFT bin in Hz.

        window_times:
            Centre time of each FFT window in seconds.

        amplitudes:
            Matrix with shape (frequency_bins, time_windows).

        sample_rate:
            Estimated sample rate in samples per second.
    """
    sample_intervals = np.diff(time)
    sample_interval = float(np.median(sample_intervals))

    if sample_interval <= 0:
        raise ValueError("The CSV contains an invalid time axis.")

    sample_rate = 1.0 / sample_interval

    relative_jitter = np.std(sample_intervals) / sample_interval

    if relative_jitter > 0.01:
        print(
            "Warning: sample spacing is not uniform. "
            f"Relative interval variation: {100 * relative_jitter:.2f}%"
        )

    if window_samples < 2:
        raise ValueError("The FFT window must contain at least two samples.")

    if window_samples > len(voltage):
        raise ValueError(
            f"The requested FFT window contains {window_samples} samples, "
            f"but the recording contains only {len(voltage)} samples."
        )

    if not 0.0 <= overlap_fraction < 1.0:
        raise ValueError("Overlap must be at least 0 and less than 1.")

    hop_samples = max(
        1,
        int(round(window_samples * (1.0 - overlap_fraction))),
    )

    window = np.hanning(window_samples)
    coherent_gain = float(np.mean(window))

    frequencies = np.fft.rfftfreq(
        window_samples,
        d=sample_interval,
    )

    window_start_indices = np.arange(
        0,
        len(voltage) - window_samples + 1,
        hop_samples,
        )

    if len(window_start_indices) == 0:
        raise ValueError("No complete FFT windows could be generated.")

    amplitude_columns: list[np.ndarray] = []
    window_times: list[float] = []

    for start_index in window_start_indices:
        stop_index = start_index + window_samples

        signal_segment = voltage[start_index:stop_index]

        # Remove the local DC offset separately from each time window.
        signal_segment = signal_segment - np.mean(signal_segment)

        windowed_signal = signal_segment * window

        spectrum = np.fft.rfft(windowed_signal)

        amplitudes = (
                np.abs(spectrum)
                / (window_samples * coherent_gain)
        )

        # Convert to a one-sided peak-amplitude spectrum.
        if window_samples % 2 == 0:
            amplitudes[1:-1] *= 2.0
        else:
            amplitudes[1:] *= 2.0

        amplitude_columns.append(amplitudes)

        centre_index = start_index + (window_samples - 1) / 2
        centre_time = time[0] + centre_index * sample_interval
        window_times.append(centre_time)

    amplitude_matrix = np.column_stack(amplitude_columns)

    return (
        frequencies,
        np.asarray(window_times),
        amplitude_matrix,
        sample_rate,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a frequency-versus-time spectrogram from a "
            "Rigol oscilloscope CSV."
        )
    )

    parser.add_argument(
        "csv_file",
        type=Path,
        help="Path to the Rigol CSV file.",
    )

    window_group = parser.add_mutually_exclusive_group()

    window_group.add_argument(
        "--window-samples",
        type=int,
        default=None,
        help=(
            "Number of samples in each FFT window. Larger values improve "
            "frequency resolution but reduce time resolution."
        ),
    )

    window_group.add_argument(
        "--window-duration",
        type=float,
        default=None,
        help=(
            "Duration of each FFT window in seconds. For example, 0.01 "
            "uses 10 ms windows."
        ),
    )

    parser.add_argument(
        "--overlap",
        type=float,
        default=0.75,
        help=(
            "Fractional overlap between FFT windows. "
            "Default: 0.75."
        ),
    )

    parser.add_argument(
        "--min-frequency",
        type=float,
        default=0.0,
        help="Minimum displayed frequency in Hz. Default: 0.",
    )

    parser.add_argument(
        "--max-frequency",
        type=float,
        default=None,
        help="Maximum displayed frequency in Hz.",
    )

    parser.add_argument(
        "--linear",
        action="store_true",
        help=(
            "Display amplitude in volts instead of dBV. "
            "dBV is used by default."
        ),
    )

    parser.add_argument(
        "--dynamic-range",
        type=float,
        default=80.0,
        help=(
            "Displayed dynamic range in dB below the strongest component. "
            "Default: 80 dB. Used only in dBV mode."
        ),
    )

    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help=(
            "Optional path at which to save the spectrogram, "
            "such as spectrogram.png."
        ),
    )

    args = parser.parse_args()

    time, voltage = read_rigol_csv(args.csv_file)

    sample_intervals = np.diff(time)
    sample_interval = float(np.median(sample_intervals))
    sample_rate = 1.0 / sample_interval

    if args.window_duration is not None:
        if args.window_duration <= 0:
            raise ValueError("Window duration must be positive.")

        window_samples = int(round(
            args.window_duration * sample_rate
        ))
    elif args.window_samples is not None:
        window_samples = args.window_samples
    else:
        # A reasonable default for many recordings.
        # This is capped so that recordings with fewer samples still work.
        window_samples = min(4096, len(voltage))

    frequencies, window_times, amplitudes, sample_rate = (
        calculate_spectrogram(
            time=time,
            voltage=voltage,
            window_samples=window_samples,
            overlap_fraction=args.overlap,
        )
    )

    hop_samples = max(
        1,
        int(round(window_samples * (1.0 - args.overlap))),
    )

    window_duration = window_samples / sample_rate
    hop_duration = hop_samples / sample_rate
    frequency_resolution = sample_rate / window_samples

    print(f"Samples:                  {len(voltage)}")
    print(f"Acquisition duration:     {time[-1] - time[0]:.9g} s")
    print(f"Sample rate:              {sample_rate:.9g} Sa/s")
    print(f"Nyquist frequency:        {sample_rate / 2:.9g} Hz")
    print(f"FFT window samples:       {window_samples}")
    print(f"FFT window duration:      {window_duration:.9g} s")
    print(f"Window overlap:           {100 * args.overlap:.1f}%")
    print(f"Time step between FFTs:   {hop_duration:.9g} s")
    print(f"Frequency resolution:     {frequency_resolution:.9g} Hz")
    print(f"Number of FFT windows:    {len(window_times)}")

    minimum_frequency = max(0.0, args.min_frequency)

    if args.max_frequency is None:
        maximum_frequency = sample_rate / 2
    else:
        maximum_frequency = min(
            args.max_frequency,
            sample_rate / 2,
            )

    if maximum_frequency <= minimum_frequency:
        raise ValueError(
            "Maximum frequency must be greater than minimum frequency."
        )

    frequency_mask = (
            (frequencies >= minimum_frequency)
            & (frequencies <= maximum_frequency)
    )

    displayed_frequencies = frequencies[frequency_mask]
    displayed_amplitudes = amplitudes[frequency_mask, :]

    if len(displayed_frequencies) == 0:
        raise ValueError(
            "The selected frequency range contains no FFT bins."
        )

    if args.linear:
        plotted_amplitudes = displayed_amplitudes
        colour_label = "Peak amplitude (V)"
        colour_minimum = None
        colour_maximum = None
    else:
        minimum_amplitude = np.finfo(float).tiny

        plotted_amplitudes = 20.0 * np.log10(
            np.maximum(displayed_amplitudes, minimum_amplitude)
        )

        colour_label = "Peak amplitude (dBV)"

        colour_maximum = float(np.max(plotted_amplitudes))
        colour_minimum = colour_maximum - args.dynamic_range

    plt.figure(figsize=(12, 7))

    image = plt.pcolormesh(
        window_times,
        displayed_frequencies,
        plotted_amplitudes,
        shading="auto",
        vmin=colour_minimum,
        vmax=colour_maximum,
    )

    colour_bar = plt.colorbar(image)
    colour_bar.set_label(colour_label)

    plt.xlabel("Time (s)")
    plt.ylabel("Frequency (Hz)")
    plt.title(f"Spectrogram — {args.csv_file.name}")

    plt.ylim(minimum_frequency, maximum_frequency)
    plt.tight_layout()

    if args.save is not None:
        plt.savefig(args.save, dpi=200)
        print(f"Saved spectrogram to: {args.save}")

    plt.show()


if __name__ == "__main__":
    main()