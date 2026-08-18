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

n = (
            len(raw) - HEADER_SIZE
    ) // RECORD_SIZE


time = []
position = []


for i in range(n):

    r = struct.unpack_from(
        RECORD_FORMAT,
        raw,
        HEADER_SIZE + i * RECORD_SIZE
    )

    # Time in seconds
    time.append(
        r[0] * 1e-6
    )

    # Encoder position
    if side == "left":
        pos = r[11]
    else:
        pos = r[12]

    position.append(pos)


time = np.array(time)
position = np.array(position)

# Start time at zero
time -= time[0]


plt.plot(
    time,
    position
)

plt.xlabel("Time [s]")
plt.ylabel("Encoder position")
plt.title(
    f"{side.capitalize()} encoder position"
)

plt.grid()
plt.show()