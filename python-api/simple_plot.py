#!/usr/bin/env python3

import struct
import sys
import numpy as np
import matplotlib.pyplot as plt

HEADER_SIZE = 16
RECORD_FORMAT = "<7I4f2f2f2f4B"
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)

filename = sys.argv[1]
side = sys.argv[2] if len(sys.argv) > 2 else "left"

raw = open(filename, "rb").read()
n = (len(raw) - HEADER_SIZE) // RECORD_SIZE

time = []
torque = []

for i in range(n):
    r = struct.unpack_from(
        RECORD_FORMAT,
        raw,
        HEADER_SIZE + i * RECORD_SIZE
    )

    time.append(r[0] * 1e-6)

    if side == "left":
        tau = (r[7] - r[8]) * 0.055
    else:
        tau = (r[9] - r[10]) * 0.055

    torque.append(tau)

time = np.array(time)
time -= time[0]

plt.plot(time, torque)
plt.xlabel("Time [s]")
plt.ylabel("Interaction torque [N m]")
plt.grid()
plt.show()