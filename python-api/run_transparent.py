from ankle_exo_api import Exoskeleton, Side

import time
import matplotlib.pyplot as plt
from collections import deque
import importlib
import exo_params



with Exoskeleton() as exo:

    time.sleep(5)

    exo.start_motor(Side.LEFT)
    exo.start_motor(Side.RIGHT)

    time.sleep(5)

    importlib.reload(exo_params)
    params = exo_params.params

    exo.update_transparent_params(
        Side.LEFT,
        params
    )

    exo.update_transparent_params(
        Side.RIGHT,
        params
    )

    print("Voltage of the battery")
    print(exo.get_power()[0])

    try:
        while True:

            importlib.reload(exo_params)
            params = exo_params.params

            exo.update_transparent_params(
                Side.LEFT,
                params
            )

            exo.update_transparent_params(
                Side.RIGHT,
                params
            )

            time.sleep(1)

    except KeyboardInterrupt:

        print("Stopping...")

    finally:

        exo.stop_recording()

        exo.set_torque(
            Side.LEFT,
            0.0
        )

        exo.set_torque(
            Side.RIGHT,
            0.0
        )
