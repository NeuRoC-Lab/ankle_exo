from ankle_exo_api import Exoskeleton, Side
import time


with Exoskeleton() as exo:
    exo.start_recording()
    i = input("Enter to stop")
    exo.stop_recording()
    exo.stop_motor(Side.LEFT)
    exo.stop_motor(Side.RIGHT)


