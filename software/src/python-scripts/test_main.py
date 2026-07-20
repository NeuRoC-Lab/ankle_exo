"""
Main Plotting and Data Logging Script for multiple sensors and motor at the same time
Purpose for single leg implementation test

Enter choice of sensors and actuators
"""

from multiprocessing import Process

from plotters.mit_motor_plotter.py import MotorMITPlotter
from plotters.single_encoder_plotter.py import EncoderPlotter
from plotters.two_loadcells_plotter.py import LoadcellsPlotter

# Adjust COM port as necessary by first checking on the Arduino IDE
# or by running python -m serial.tools.list_ports
def run_plotter(choice):
    if choice == "motor":
        plotter = MotorMITPlotter(port="COM6", baud=115200, time_window=10)

    elif choice == "encoder":
        plotter = EncoderPlotter(port="COM4", baud=115200, time_window=10)

    elif choice == "loadcells":
        plotter = LoadcellsPlotter(port="COM5", baud=115200, time_window=10)

    else:
        return

    plotter.run()

def main():

    print("Please choose everything you want to plot: ")
    print("Options: motor, encoder, loadcells")
    print("Enter the name of all the sensors/actuators to plot and log data, separated by a a space")
    print("For example: motor encoder loadcells")
    print("Note that 'loadcell' will not work, use 'loadcells'")

    choices = input("Enter your choices: ").split()

    processes = []

    for choice in choices:
        if choice in ("motor", "encoder", "loadcells"):
            process = Process(target=run_plotter, args=(choice,))
            processes.append(process)
            process.start()

    for process in processes:
        process.join()

if __name__ == "__main__":
    main()