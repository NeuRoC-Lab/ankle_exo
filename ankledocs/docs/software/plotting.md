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
- Import threading

2. 