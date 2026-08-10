import importlib
import time

my_module = importlib.import_module("single-exo")

Exoskeleton = my_module.Exoskeleton

print("Starting script to monitor load cell values from Python API")
started = False
with Exoskeleton() as exo:
    #TODO to prevent commands from being discarded on startup make some async stuff to make sure the commands get sent once the BLE connection is properly established
    exo.start_motor(3)
    while True:
        if load_cell_values := exo.get_loadcells():
            #print(load_cell_values)
            #print(exo.get_encoder_angle())
            exo.set_command(0,1,0,0,1,3)
            time.sleep(1)
            exo.set_command(0,0,0,0,1,3)
            time.sleep(1)

