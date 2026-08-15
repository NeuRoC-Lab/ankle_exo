from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


FILE = Path(
    "/Volumes/EXO_LOG/loadcell.csv"
)


# =========================================================
# LOAD DATA
# =========================================================

def load_data(filename: Path) -> pd.DataFrame:

    if not filename.exists():
        raise FileNotFoundError(
            f"Could not find:\n{filename}"
        )

    df = pd.read_csv(filename)

    if df.empty:
        raise RuntimeError(
            "CSV contains no samples."
        )

    if "time_us" not in df.columns:
        raise RuntimeError(
            "CSV does not contain a 'time_us' column."
        )

    # -----------------------------------------------------
    # Remove completely empty rows
    # -----------------------------------------------------

    df = df.dropna(
        how="all"
    )

    # -----------------------------------------------------
    # Convert all columns to numeric
    #
    # Repeated CSV header lines such as:
    #
    # time_us,left_1,left_2,...
    #
    # become NaN and are removed below.
    # -----------------------------------------------------

    for column in df.columns:
        df[column] = pd.to_numeric(
            df[column],
            errors="coerce",
        )

    # A valid recording row must have a timestamp.
    df = df.dropna(
        subset=["time_us"]
    ).reset_index(
        drop=True
    )

    if df.empty:
        raise RuntimeError(
            "No valid numeric samples found in CSV."
        )

    # -----------------------------------------------------
    # Convert microseconds -> relative seconds
    # -----------------------------------------------------

    df["time_s"] = (
                           df["time_us"]
                           -
                           df["time_us"].iloc[0]
                   ) * 1e-6

    return df

# =========================================================
# TIMING ANALYSIS
# =========================================================

def print_timing_info(
        df: pd.DataFrame,
):

    print()
    print("=== RECORDING ===")

    print(
        f"Samples: {len(df)}"
    )

    duration = (
            df["time_s"].iloc[-1]
            -
            df["time_s"].iloc[0]
    )

    print(
        f"Duration: {duration:.3f} s"
    )


    if len(df) < 2:
        return


    dt = (
            df["time_us"]
            .diff()
            .dropna()
            *
            1e-6
    )


    mean_dt = dt.mean()
    median_dt = dt.median()


    if mean_dt > 0:

        print(
            "Average sample rate: "
            f"{1.0 / mean_dt:.2f} Hz"
        )


    if median_dt > 0:

        print(
            "Median sample rate: "
            f"{1.0 / median_dt:.2f} Hz"
        )


    print(
        "Mean dt: "
        f"{mean_dt * 1000:.3f} ms"
    )

    print(
        "Median dt: "
        f"{median_dt * 1000:.3f} ms"
    )

    print(
        "Largest dt: "
        f"{dt.max() * 1000:.3f} ms"
    )


    # Anything > 2.5 times median interval
    # is worth inspecting as a possible missed sample
    # or long SD write.
    if median_dt > 0:

        gaps = dt[
            dt >
            2.5 * median_dt
            ]

        print(
            f"Large timing gaps: {len(gaps)}"
        )


# =========================================================
# COLUMN INFORMATION
# =========================================================

def print_columns(
        df: pd.DataFrame,
):

    print()
    print("=== COLUMNS ===")

    for column in df.columns:
        print(
            f"  {column}"
        )


# =========================================================
# GENERIC PLOTTER
# =========================================================

def plot_columns(
        df: pd.DataFrame,
        columns,
        title,
        ylabel,
):

    available = [
        column
        for column in columns
        if column in df.columns
    ]

    if not available:
        print(
            f"Skipping '{title}': "
            "no matching columns"
        )
        return


    plt.figure(
        figsize=(11, 5)
    )

    for column in available:

        plt.plot(
            df["time_s"],
            df[column],
            label=column,
        )


    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        ylabel
    )

    plt.title(
        title
    )

    plt.grid(
        True,
        alpha=0.3
    )

    plt.legend()

    plt.tight_layout()


# =========================================================
# ANALYSIS
# =========================================================

def main():

    df = load_data(
        FILE
    )


    print(
        f"Loaded: {FILE}"
    )

    print_columns(
        df
    )

    print_timing_info(
        df
    )


    # -----------------------------------------------------
    # Load cells
    # -----------------------------------------------------

    plot_columns(
        df,
        [
            "left_1",
            "left_2",
            "right_1",
            "right_2",
        ],
        title="Load Cell Forces",
        ylabel="Force",
    )


    # -----------------------------------------------------
    # Encoders
    # -----------------------------------------------------

    plot_columns(
        df,
        [
            "encoder_left",
            "encoder_right",
        ],
        title="Joint Encoder Positions",
        ylabel="Position [deg]",
    )


    # -----------------------------------------------------
    # Left motor
    # -----------------------------------------------------

    plot_columns(
        df,
        [
            "left_motor_position",
        ],
        title="Left Motor Position",
        ylabel="Position",
    )


    plot_columns(
        df,
        [
            "left_motor_velocity",
        ],
        title="Left Motor Velocity",
        ylabel="Velocity",
    )


    plot_columns(
        df,
        [
            "left_motor_torque",
        ],
        title="Left Motor Torque",
        ylabel="Torque [Nm]",
    )


    # -----------------------------------------------------
    # Right motor
    # -----------------------------------------------------

    plot_columns(
        df,
        [
            "right_motor_position",
        ],
        title="Right Motor Position",
        ylabel="Position",
    )


    plot_columns(
        df,
        [
            "right_motor_velocity",
        ],
        title="Right Motor Velocity",
        ylabel="Velocity",
    )


    plot_columns(
        df,
        [
            "right_motor_torque",
        ],
        title="Right Motor Torque",
        ylabel="Torque [Nm]",
    )


    # -----------------------------------------------------
    # Motor temperatures
    # -----------------------------------------------------

    plot_columns(
        df,
        [
            "left_motor_temperature",
            "right_motor_temperature",
        ],
        title="Motor Temperatures",
        ylabel="Temperature [°C]",
    )


    plt.show()


if __name__ == "__main__":
    main()