"""
This code is used to plot the relative angle of one encoder in degrees
relative to the original position the ankle is at when the test just starts

python src/plotEncoder.py

Arduino Pin Connection:
SPI Clock (SCK): Pin 13
SPI MOSI:        Pin 11
SPI MISO:        Pin 12
SPI Chip Select: Pin 10

"""

import serial
import time
import matplotlib.pyplot as plt
import csv
import os

class EncoderPlotter:

    def __init__(self, port="COM4", baud=115200, time_window=10):
        self.port = port
        self.baud = baud
        self.time_window = time_window

        #Establish serial connection
        self.ser = serial.Serial(self.port, self.baud, timeout=1)

        time.sleep(2)

        #Clear old data
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.running = True

        #Prepare plot data
        self.time_data = []
        self.angle_data = []

        #Prepare CSV file
        self.filename = "EncoderData.csv"
        self.path = os.path.abspath(self.filename)

        self.fig = None
        self.ax = None
        self.line = None

    def stop_encoder(self):

        if self.running == False:
            return

        self.running = False
        if self.ser.is_open:
            self.ser.write(b"n")
            self.ser.flush()
            self.ser.close()

        print("Encoder Plotting and Data Logging Stopped")
        print("CSV file is saved here: ")
        print(self.path)

    def close_plot(self, event):
        self.stop_encoder()

    def run_plot(self):

        #Set current encoder position at zero whenever the code is ran
        self.ser.write(b"z")
        self.set.flush()

        print("Current ankle position is set at zero")
        print("The encoder will return angle values relative to the zero position")

        time.sleep(1)

        #Create plot
        plt.ion()
        width = 16
        height = 12
        self.fig, self.ax = plt.subplots()
        line, = ax.plot(time_data, angle_data, linewidth=1.5)

        self.ax.set_title("Encoder angle vs Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Angle (deg")
        self.ax.grid(True)

        self.fig.canvas.mpl_connect("close_event", close_plot)

        # Show figure window
        plt.show(block=False)

        #Start
        self.ser.write(b"y")
        self.ser.flush()
        print("Encoder Starting Now")

        start_time = time.perf_counter()

        # Open CSV file
        with open (self.path, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "Time (s)",
                "Angle (deg)"
            ])

            try:
                while (self.running == True) and (plt.fignum_exists(self.fig.number) == True):
                    # y-axis data
                    while self.ser.in_waiting > 0:
                        raw = self.ser.readline().decode().strip()

                    # take most recent data in serial buffer, at the expense of discarding intermediate data. This ensures lowest latency
                    # for more accurate logging (which will however introduce a delay) remove the while ser.in_waiting > 0 loop

                    #print(raw) not needed as it may flood the terminal
                    arr = raw.split()

                    try:
                        i = arr.index("Angle:")
                        angle = float(arr[i+1])
                        self.angle_data.append(angle)

                    except ValueError:
                        continue

                    # x-axis data
                    current_time = time.perf_counter() - start_time
                    self.time_data.append(current_time)

                    # CSV writing
                    writer.writerow([
                        current_time,
                        angle
                    ])

                    csv_file.flush()

                    # keep only last 30 seconds
                    while self.time_data and (current_time - self.time_data[0]) > self.time_window:
                        self.time_data.pop(0)
                        self.angle_data.pop(0)

                    # update plot
                    self.line.set_data(self.time_data, self.angle_data)

                    self.ax.set_xlim(max(0, current_time - self.time_window), current_time)
                    self.ax.relim()
                    self.ax.autoscale_view(scalex=False, scaley=True)

                    self.fig.canvas.draw()
                    self.fig.canvas.flush_events()

            except KeyboardInterrupt:
                print("Ctrl+C pressed, closing figure")
                ser.close()

            finally:
            stop_encoder()
            plt.close("all")