
from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Any

from backend import AnkleExoBackend
from sensors import Motor_ID


OUTPUT_FILE = Path("motor_loadcell_torque_log.csv")

# Each tuple contains:
# (commanded torque in Nm, duration in seconds)
TORQUE_SEQUENCE = [
    (0.0, 1.0),
    (0.1, 3.0),
    (0.2, 3.0),
    (0.3, 3.0),
    (0.4, 3.0),
    (0.5, 3.0),
    (0.6, 3.0),
    (0.7, 3.0),
    (0.8, 3.0),
    (0.9, 3.0),
    (1.0, 3.0),
    (0.8, 3.0),
    (0.4, 3.0),
    (0.3, 3.0),
    (0.2, 3.0),
    (0.1, 3.0),
    (0.0, 1.0),
]

POLL_INTERVAL_S = 0.005


def snapshot_to_row(
        snapshot: Any,
        elapsed_time_s: float,
        stage_elapsed_time_s: float,
        stage_number: int,
        commanded_torque_nm: float,
) -> dict[str, float | int]:
    """Convert one TelemetrySnapshot into one CSV row."""

    return {
        "elapsed_time_s": elapsed_time_s,
        "stage_elapsed_time_s": stage_elapsed_time_s,
        "stage_number": stage_number,
        "device_sample_time": snapshot.sample_time,
        "commanded_motor_torque_nm": commanded_torque_nm,
        "measured_motor_torque_nm": snapshot.motor_torque,
        "motor_position_rad": snapshot.motor_position,
        "motor_velocity_rad_s": snapshot.motor_velocity,
        "motor_temperature_c": snapshot.motor_temperature,
        "encoder_raw": snapshot.encoder,
        "ankle_angle_deg": snapshot.ankle_angle,
        "ankle_velocity_deg_s": snapshot.ankle_velocity,
        "loadcell1": snapshot.loadcell1,
        "loadcell2": snapshot.loadcell2,
    }


def send_torque(
        backend: AnkleExoBackend,
        torque_nm: float,
) -> None:
    """Send a pure MIT-mode feedforward torque command."""

    backend.set_motor_command(
        motor_id=Motor_ID,
        position=0.0,
        velocity=0.0,
        kp=0.0,
        kd=0.0,
        torque=torque_nm,
    )


def main() -> None:
    fieldnames = [
        "elapsed_time_s",
        "stage_elapsed_time_s",
        "stage_number",
        "device_sample_time",
        "commanded_motor_torque_nm",
        "measured_motor_torque_nm",
        "motor_position_rad",
        "motor_velocity_rad_s",
        "motor_temperature_c",
        "encoder_raw",
        "ankle_angle_deg",
        "ankle_velocity_deg_s",
        "loadcell1",
        "loadcell2",
    ]

    samples_written = 0

    with AnkleExoBackend() as backend:
        backend.start_motor(Motor_ID)
        time.sleep(0.2)

        experiment_start = time.perf_counter()

        try:
            with OUTPUT_FILE.open(
                    mode="w",
                    newline="",
                    encoding="utf-8",
            ) as csv_file:
                writer = csv.DictWriter(
                    csv_file,
                    fieldnames=fieldnames,
                )
                writer.writeheader()

                for stage_number, (torque_nm, duration_s) in enumerate(
                        TORQUE_SEQUENCE,
                        start=1,
                ):
                    print(
                        f"Stage {stage_number}/{len(TORQUE_SEQUENCE)}: "
                        f"{torque_nm:+.3f} Nm for {duration_s:.1f} s"
                    )

                    # Remove any old snapshots before starting the stage.
                    backend.get_pending_snapshots()

                    send_torque(
                        backend=backend,
                        torque_nm=torque_nm,
                    )

                    stage_start = time.perf_counter()
                    stage_end = stage_start + duration_s

                    while time.perf_counter() < stage_end:
                        snapshots = backend.get_pending_snapshots()

                        if not snapshots:
                            time.sleep(POLL_INTERVAL_S)
                            continue

                        for snapshot in snapshots:
                            current_time = time.perf_counter()

                            elapsed_time_s = (
                                    current_time - experiment_start
                            )

                            stage_elapsed_time_s = (
                                    current_time - stage_start
                            )

                            row = snapshot_to_row(
                                snapshot=snapshot,
                                elapsed_time_s=elapsed_time_s,
                                stage_elapsed_time_s=stage_elapsed_time_s,
                                stage_number=stage_number,
                                commanded_torque_nm=torque_nm,
                            )

                            writer.writerow(row)
                            samples_written += 1

                            print(
                                f"t={elapsed_time_s:7.3f} s | "
                                f"cmd={torque_nm:+.3f} Nm | "
                                f"motor={snapshot.motor_torque:+.3f} Nm | "
                                f"LC1={snapshot.loadcell1:+.3f} | "
                                f"LC2={snapshot.loadcell2:+.3f}"
                            )

                    # Save the file after every torque stage.
                    csv_file.flush()

        except KeyboardInterrupt:
            print("\nExperiment interrupted by user")

        finally:
            print("Sending zero torque...")

            send_torque(
                backend=backend,
                torque_nm=0.0,
            )

            time.sleep(0.2)
            backend.stop_motor(Motor_ID)

    print(
        f"Saved {samples_written} samples to "
        f"{OUTPUT_FILE.resolve()}"
    )


if __name__ == "__main__":
    main()
