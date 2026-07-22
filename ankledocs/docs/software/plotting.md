## Reference Page for Real-Time Sensors Plotting Scripts

The real-time plotting of the exoskeleton sensors and motors is achieved using Python 
To check for the list of ports in CLion IDE, use python -m serial.tools.list_ports

The result for a one leg setup can be found through the unified.py script under software/src/python-scripts
It plots 6 graphs: 

- encoder angle
- encoder velocity
- cable tension 1
- cable tension 2
- motor position
- motor velocity

Challenges encountered and solution process:

1. Find a way to control the motor while the plot is running

- UI slider
  - Pros: easy to use for testing purposes
  - Cons: only temporarily needed feature
- Import threading to send commands via the terminal
  - Pros: same way to control motor as the motor script alone
  - Cons: not practical at all if printing data on the terminal 
- Using Teensy and Nano at the same time (plot from reading Nano data, control motor by sending commands to Teensy)

2. Plot fast and smoothly enough

- Replace 
  raw = ser.readline().decode(errors="ignore").strip()
  with
  while ser.in_waiting:
  raw = ser.readline().decode(errors="ignore").strip()
- Telemetry library?

3. Noise in one load cell caused by crosstalk and wiring/PCB issues

4. Encoder noise when motor is turned on but with a zero velocity


5. Encoder random spikes caused by invalid reading returns the uint_16_max
- When there is an invalid reading, plot the previous valid reading