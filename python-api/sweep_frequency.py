from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# CONFIGURATION
# ============================================================

FILE = Path("/Volumes/EXO_LOG/sweep.csv")

# Frequencies used by TorqueBandwidthController.
FREQUENCIES_HZ = np.array([
    0.5,
    1.0,
    2.0,
    3.0,
    5.0,
    7.0,
    10.0,
    15.0,
    20.0,
    30.0,
    40.0,
    50.0,
])

LEVER_ARM_M = 0.055

# Left-side convention from the embedded controller.
TORQUE_SIGN = 1.0

# Ignore this fraction at the beginning/end of each frequency
# section to reduce contamination from transitions.
TRIM_FRACTION = 0.15

# Command must have at least this RMS value to be considered
# part of the sweep.
MIN_COMMAND_RMS = 0.01


# ============================================================
# LOAD CSV
# ============================================================

def load_data(filename: Path) -> pd.DataFrame:

    if not filename.exists():
        raise FileNotFoundError(
            f"Could not find:\n{filename}"
        )

    df = pd.read_csv(filename)

    # Remove completely empty rows.
    df = df.dropna(how="all")

    # Handle repeated CSV headers caused by append mode.
    for column in df.columns:
        df[column] = pd.to_numeric(
            df[column],
            errors="coerce",
        )

    df = df.dropna(
        subset=["time_us"]
    ).reset_index(drop=True)

    required = [
        "time_us",
        "left_1",
        "left_2",
        "left_command_torque",
    ]

    missing = [
        column
        for column in required
        if column not in df.columns
    ]

    if missing:
        raise RuntimeError(
            "Missing required CSV columns: "
            + ", ".join(missing)
        )

    if len(df) < 10:
        raise RuntimeError(
            "Not enough samples in sweep.csv"
        )

    # Relative time in seconds.
    df["time_s"] = (
                           df["time_us"]
                           - df["time_us"].iloc[0]
                   ) * 1e-6

    # Interaction torque from the two load cells.
    df["interaction_torque"] = (
            TORQUE_SIGN
            * (
                    df["left_1"]
                    - df["left_2"]
            )
            * LEVER_ARM_M
    )

    return df


# ============================================================
# SINE FIT
# ============================================================

def fit_sine(
        time_s: np.ndarray,
        signal: np.ndarray,
        frequency_hz: float,
):
    """
    Least-squares fit:

        y(t) =
            a sin(wt)
          + b cos(wt)
          + c

    Returns:
        amplitude
        phase [rad]
        offset
    """

    omega = (
            2.0
            * np.pi
            * frequency_hz
    )

    design = np.column_stack([
        np.sin(omega * time_s),
        np.cos(omega * time_s),
        np.ones_like(time_s),
    ])

    coefficients, *_ = np.linalg.lstsq(
        design,
        signal,
        rcond=None,
    )

    a, b, offset = coefficients

    amplitude = np.sqrt(
        a**2 + b**2
    )

    # a sin(wt) + b cos(wt)
    # =
    # A sin(wt + phi)
    phase = np.arctan2(
        b,
        a,
    )

    fitted = (
            design
            @ coefficients
    )

    residual = (
            signal
            - fitted
    )

    rms_error = np.sqrt(
        np.mean(
            residual**2
        )
    )

    return (
        amplitude,
        phase,
        offset,
        fitted,
        rms_error,
    )


# ============================================================
# FREQUENCY IDENTIFICATION
# ============================================================

def estimate_command_frequency(
        time_s: np.ndarray,
        command: np.ndarray,
) -> float:
    """
    Estimate dominant command frequency using an FFT.
    """

    if len(time_s) < 10:
        return np.nan

    dt = np.median(
        np.diff(time_s)
    )

    if dt <= 0:
        return np.nan

    sample_rate = (
            1.0 / dt
    )

    x = (
            command
            - np.mean(command)
    )

    window = np.hanning(
        len(x)
    )

    spectrum = np.fft.rfft(
        x * window
    )

    frequencies = np.fft.rfftfreq(
        len(x),
        d=1.0 / sample_rate,
    )

    magnitude = np.abs(
        spectrum
    )

    if len(magnitude) < 2:
        return np.nan

    # Ignore DC.
    magnitude[0] = 0.0

    index = np.argmax(
        magnitude
    )

    return frequencies[index]


# ============================================================
# BUILD EXPECTED SWEEP WINDOWS
# ============================================================

def dwell_time(
        frequency_hz: float,
        min_cycles=10,
        min_dwell_s=3.0,
):
    return max(
        min_cycles / frequency_hz,
        min_dwell_s,
        )


def find_sweep_start(
        df: pd.DataFrame,
):
    command = (
        df["left_command_torque"]
        .to_numpy()
    )

    active = (
            np.abs(command)
            >
            MIN_COMMAND_RMS
    )

    indices = np.flatnonzero(
        active
    )

    if len(indices) == 0:
        raise RuntimeError(
            "Could not find any non-zero "
            "left motor command."
        )

    return float(
        df["time_s"].iloc[
            indices[0]
        ]
    )


# ============================================================
# ANALYZE EACH FREQUENCY
# ============================================================

def analyze_sweep(
        df: pd.DataFrame,
) -> pd.DataFrame:

    sweep_start = find_sweep_start(
        df
    )

    print(
        f"Sweep begins at approximately "
        f"{sweep_start:.3f} s"
    )

    results = []

    section_start = sweep_start

    for frequency in FREQUENCIES_HZ:

        duration = dwell_time(
            frequency
        )

        section_end = (
                section_start
                + duration
        )

        # Trim transitions from both ends.
        trim = (
                duration
                * TRIM_FRACTION
        )

        analysis_start = (
                section_start
                + trim
        )

        analysis_end = (
                section_end
                - trim
        )

        section = df[
            (
                    df["time_s"]
                    >= analysis_start
            )
            &
            (
                    df["time_s"]
                    < analysis_end
            )
            ].copy()

        if len(section) < 20:
            print(
                f"{frequency:6.2f} Hz: "
                "not enough data"
            )

            section_start = (
                section_end
            )

            continue

        t = (
            section["time_s"]
            .to_numpy()
        )

        # Improve numerical conditioning.
        t = (
                t - t[0]
        )

        commanded = (
            section[
                "left_command_torque"
            ]
            .to_numpy()
        )

        measured = (
            section[
                "interaction_torque"
            ]
            .to_numpy()
        )


        # ---------------------------------------------
        # Fit the command
        # ---------------------------------------------

        (
            command_amplitude,
            command_phase,
            _,
            _,
            command_error,
        ) = fit_sine(
            t,
            commanded,
            frequency,
        )


        # ---------------------------------------------
        # Fit measured interaction torque
        # ---------------------------------------------

        (
            measured_amplitude,
            measured_phase,
            _,
            _,
            measured_error,
        ) = fit_sine(
            t,
            measured,
            frequency,
        )


        if (
                command_amplitude
                <= 1e-6
        ):
            print(
                f"{frequency:6.2f} Hz: "
                "command amplitude too small"
            )

            section_start = (
                section_end
            )

            continue


        # ---------------------------------------------
        # Frequency response
        # ---------------------------------------------

        gain = (
                measured_amplitude
                /
                command_amplitude
        )

        gain_db = (
                20.0
                * np.log10(gain)
        )

        phase_rad = (
                measured_phase
                - command_phase
        )

        # Wrap to [-pi, pi].
        phase_rad = (
            np.arctan2(
                np.sin(phase_rad),
                np.cos(phase_rad),
            )
        )

        phase_deg = (
            np.degrees(
                phase_rad
            )
        )


        # Check actual command frequency.
        measured_command_frequency = (
            estimate_command_frequency(
                t,
                commanded,
            )
        )


        results.append({
            "frequency_hz":
                frequency,

            "detected_frequency_hz":
                measured_command_frequency,

            "command_amplitude_nm":
                command_amplitude,

            "interaction_amplitude_nm":
                measured_amplitude,

            "gain":
                gain,

            "gain_db":
                gain_db,

            "phase_deg":
                phase_deg,

            "command_fit_rms":
                command_error,

            "interaction_fit_rms":
                measured_error,

            "samples":
                len(section),
        })


        print(
            f"{frequency:6.2f} Hz | "
            f"gain = {gain:7.3f} | "
            f"{gain_db:7.2f} dB | "
            f"phase = {phase_deg:8.2f} deg | "
            f"N = {len(section)}"
        )


        section_start = (
            section_end
        )


    if not results:
        raise RuntimeError(
            "No frequency points could be analyzed."
        )

    return pd.DataFrame(
        results
    )


# ============================================================
# ESTIMATE -3 dB BANDWIDTH
# ============================================================

def estimate_bandwidth(
        results: pd.DataFrame,
):
    frequency = (
        results["frequency_hz"]
        .to_numpy()
    )

    gain_db = (
        results["gain_db"]
        .to_numpy()
    )

    if len(gain_db) < 2:
        return None

    # Normalize relative to the first measured
    # frequency rather than assuming DC gain = 1.
    normalized_db = (
            gain_db
            - gain_db[0]
    )

    target = -3.0

    for i in range(
            1,
            len(frequency),
    ):

        if (
                normalized_db[i]
                <= target
                and
                normalized_db[i - 1]
                > target
        ):

            # Linear interpolation in log-frequency.
            x1 = np.log10(
                frequency[i - 1]
            )

            x2 = np.log10(
                frequency[i]
            )

            y1 = normalized_db[
                i - 1
                ]

            y2 = normalized_db[
                i
            ]

            fraction = (
                    (target - y1)
                    /
                    (y2 - y1)
            )

            log_bandwidth = (
                    x1
                    +
                    fraction
                    * (x2 - x1)
            )

            return (
                    10.0
                    ** log_bandwidth
            )

    return None


# ============================================================
# PLOTS
# ============================================================

def plot_raw_data(
        df: pd.DataFrame,
):

    plt.figure(
        figsize=(12, 5)
    )

    plt.plot(
        df["time_s"],
        df["left_command_torque"],
        label="Commanded torque",
    )

    plt.plot(
        df["time_s"],
        df["interaction_torque"],
        label="Interaction torque",
    )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "Torque [Nm]"
    )

    plt.title(
        "Bandwidth sweep — raw torque signals"
    )

    plt.grid(
        True
    )

    plt.legend()

    plt.tight_layout()


def plot_gain(
        results: pd.DataFrame,
        bandwidth,
):

    frequency = (
        results["frequency_hz"]
    )

    normalized_gain_db = (
            results["gain_db"]
            -
            results["gain_db"].iloc[0]
    )


    plt.figure(
        figsize=(9, 5)
    )

    plt.semilogx(
        frequency,
        normalized_gain_db,
        marker="o",
    )


    plt.axhline(
        -3.0,
        linestyle="--",
        label="-3 dB",
    )


    if bandwidth is not None:

        plt.axvline(
            bandwidth,
            linestyle="--",
            label=(
                f"Bandwidth ≈ "
                f"{bandwidth:.2f} Hz"
            ),
        )


    plt.xlabel(
        "Frequency [Hz]"
    )

    plt.ylabel(
        "Normalized gain [dB]"
    )

    plt.title(
        "Locked-output torque frequency response — gain"
    )

    plt.grid(
        True,
        which="both",
    )

    plt.legend()

    plt.tight_layout()


def plot_phase(
        results: pd.DataFrame,
):

    plt.figure(
        figsize=(9, 5)
    )

    plt.semilogx(
        results["frequency_hz"],
        results["phase_deg"],
        marker="o",
    )

    plt.xlabel(
        "Frequency [Hz]"
    )

    plt.ylabel(
        "Phase [deg]"
    )

    plt.title(
        "Locked-output torque frequency response — phase"
    )

    plt.grid(
        True,
        which="both",
    )

    plt.tight_layout()


# ============================================================
# MAIN
# ============================================================

def main():

    print(
        f"Loading:\n{FILE}"
    )

    df = load_data(
        FILE
    )


    print(
        f"\nSamples: {len(df)}"
    )

    duration = (
            df["time_s"].iloc[-1]
            -
            df["time_s"].iloc[0]
    )

    print(
        f"Duration: {duration:.2f} s"
    )


    dt = (
        df["time_s"]
        .diff()
        .dropna()
    )

    if len(dt):

        sample_rate = (
                1.0
                /
                dt.median()
        )

        print(
            f"Approximate logging rate: "
            f"{sample_rate:.1f} Hz"
        )


    print(
        "\n=== FREQUENCY RESPONSE ==="
    )


    results = analyze_sweep(
        df
    )


    bandwidth = estimate_bandwidth(
        results
    )


    print()
    print(
        "=== RESULT ==="
    )

    if bandwidth is None:

        print(
            "No -3 dB crossing was found "
            "inside the tested frequency range."
        )

    else:

        print(
            "Estimated locked-output torque "
            f"bandwidth: {bandwidth:.2f} Hz"
        )


    print()
    print(results.to_string(index=False))


    # Save numerical results next to sweep.csv.
    output_file = (
            FILE.parent
            /
            "frequency_response.csv"
    )

    results.to_csv(
        output_file,
        index=False,
    )

    print(
        f"\nSaved results to:\n"
        f"{output_file}"
    )


    plot_raw_data(
        df
    )

    plot_gain(
        results,
        bandwidth,
    )

    plot_phase(
        results
    )

    plt.show()


if __name__ == "__main__":
    main()