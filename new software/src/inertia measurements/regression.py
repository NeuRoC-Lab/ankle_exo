import re
import numpy as np
import matplotlib.pyplot as plt

FILE_PATH = "data.txt"

accelerations = []
torques = []

pattern = re.compile(
    r"Accel:\s*([-+]?\d*\.?\d+)\s+Torque:\s*([-+]?\d*\.?\d+)"
)

with open(FILE_PATH, "r") as f:
    for line in f:
        match = pattern.search(line)

        if match:
            accel = float(match.group(1))
            torque = float(match.group(2))

            accelerations.append(accel)
            torques.append(torque)

accelerations = np.array(accelerations)
torques = np.array(torques)

print(f"Loaded {len(accelerations)} samples")

# --------------------------------------------------
# Linear regression:
#
# torque = I * acceleration + intercept
# --------------------------------------------------

I, intercept = np.polyfit(
    accelerations,
    torques,
    1
)

predicted_torque = (
        I * accelerations + intercept
)

# R²
ss_res = np.sum(
    (torques - predicted_torque) ** 2
)

ss_tot = np.sum(
    (torques - np.mean(torques)) ** 2
)

r_squared = 1.0 - ss_res / ss_tot

print()
print(f"Estimated inertia I = {I:.6f} kg.m^2")
print(f"Intercept           = {intercept:.6f} Nm")
print(f"R^2                 = {r_squared:.4f}")

# --------------------------------------------------
# Plot
# --------------------------------------------------

plt.scatter(
    accelerations,
    torques,
    label="Measurements"
)

# Sort x so the fitted line is drawn cleanly
order = np.argsort(accelerations)

plt.plot(
    accelerations[order],
    predicted_torque[order],
    label=(
        f"Fit: torque = "
        f"{I:.5f} * accel "
        f"+ {intercept:.5f}"
    )
)

plt.xlabel("Angular acceleration [rad/s²]")
plt.ylabel("Torque [N.m]")
plt.title("Joint inertia estimation")

plt.grid(True)
plt.legend()

plt.show()