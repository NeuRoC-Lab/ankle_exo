import serial
import re
import time
import matplotlib.pyplot as plt

class SerialConnect
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud

    def connect(self):
        ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(2)
        # Clear old data
        ser.reset_input_buffer()
        ser.reset_output_buffer()


class ConfigPlot
    def __init__(self, xlabel, ylabel, title):
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.title = title
        self.timewindow = timewindow

    def create_plot(self):
        plt.ion()
        print("Figure created")

        fig, ax = plt.subplots()

        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True)

        fig.canvas.mpl_connect("close_event", close_plot)