import importlib
import time

my_module = importlib.import_module("single-exo")

Exoskeleton = my_module.Exoskeleton

print("Starting script to monitor load cell values from Python API")
with Exoskeleton() as exo:
    while True:
        if lc_values := exo.get_loadcells():
            print(lc_values)
            time.sleep(0.5)

