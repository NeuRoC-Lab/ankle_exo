from ankle_exo_api import Exoskeleton, Side
import time


with Exoskeleton() as exo:
    time.sleep(4)
    exo.start_recording()
    time.sleep(15)
    exo.stop_recording()
    print("Stopped recording")
    time.sleep(4)




