import importlib
import time

my_module = importlib.import_module("single-exo")

Exoskeleton = my_module.Exoskeleton

print("Starting script to monitor load cell values from Python API")
started = False
with Exoskeleton() as exo:
    #TODO to prevent commands from being discarded on startup make some async stuff to make sure the commands get sent once the BLE connection is properly established
    #exo.start_motor(3)
    exo.start_motor(2)
    while True:
        print(exo.get_loadcells()["l2"] - exo.get_loadcells()["l1"])
        #print(exo.get_encoder_angle())
        exo.set_command(0,0,0.2,0,0,2)



