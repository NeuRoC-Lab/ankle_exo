import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("motor_loadcell_torque_log.csv")

print(df.columns.tolist())

for stage_number, stage_df in df.groupby("stage_number"):
    commanded_torque = stage_df["commanded_motor_torque_nm"].iloc[0]

    plt.figure()

    plt.plot(
        stage_df["stage_elapsed_time_s"],
        stage_df["loadcell1"],
        label="Load cell 1",
    )

    plt.plot(
        stage_df["stage_elapsed_time_s"],
        stage_df["loadcell2"],
        label="Load cell 2",
    )

    plt.xlabel("Stage time (s)")
    plt.ylabel("Force (N)")
    plt.title(
        f"Stage {stage_number}: "
        f"{commanded_torque:.2f} Nm commanded torque"
    )

    plt.legend()
    plt.grid()
    plt.tight_layout()

    plt.show()