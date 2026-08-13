from ankle_exo_api import SingleExoskeleton
import time


with SingleExoskeleton() as exo:
    while True:
        s = input("Press enter to start recording")
        exo.start_recording()
        print("Recording started")
