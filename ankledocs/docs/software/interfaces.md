## Software layout for the project 

### CANMotor class

The `CANMotor` class is the main class used to interface the motor. 

As of now because we are having issues interfacing the encoders with the Teensy over SPI (voltage conversion related) I gave the possibility 
for the user to control the AnkleExo PCB using 1) the Teensy and 2) with the Arduino Uno R4. It is necessary to split the implementation for the Uno R4 and the Teensy 4.1 because these two boards use different CAN Controller libraries.
The Arduino uses `Arduino_CAN.h` while the Teensy currently uses `FlexCAN_T4`. For that reason I created two subclasses for each board so that one can use one or the other board as they wish.

Note that because only the Teensy 4.1 and Arduino UNO R4 boards have an integrated CAN controller other regular boards like the Arduino UNO R3 _cannot_ be used for this class.

For the wiring, please refer to `board.h` and the `electrical` section of this documentation. It differs from board to board.

A representative OOP diagram of the `CANMotor` class and it's two subclasses `CANMotor_Teensy41`, `CANMotor_Renesas` is shown below 

![OOP Diagram](../images/CANMotor_OOP.png)

To run the unified CAN controller example code (`CAN_unified.cpp`) you will need to run the pio environment `LC_teensy41` or `LC_uno_r4` depending on which board you are using. 
As a reminder, the general format to run an environment in platformio is `pio run -e yourenvname` where `yourenvname` is the target environment. 

### LoadCell class

The `LoadCell` class is the main class used to interface the motor. 

For the same reason as mentionned earlier, I gave the possibility to run this code on different board achitectures, namely 1) The Arduino UNO R3 (AtmelAVR), 2) The Arduino UNO R4 (Renesas) and 3) the Teensy 4.1
Because the `LoadCell` class simply requires to read a voltage from an Analog pin all of the boards can be used. However the implementation slightly differs from board to board. For example, the Teensy can only support voltages in the 0-3.3V range while Arduino UNO R3/R4 have an extended ADC input voltage range of 0-5V. 
Please keep that in mind when setting up the strain gauge amplifier as driving voltages beyond the GPIO limit can permanently damage the board.

Specific boards may be used for specific needs : 

1. The Arduino UNO R3 is the simplest option and most straightforward. I recommend starting with this
2. All boards can be used to interface the integrated INA125U on the [PCB](/electrical/pcb) however _pay attention to the voltage ranges_. Assert statements during compile time will throw errors if the theoretical `Vo` range goes beyond the ADC voltage range.


![OOP Diagram](../images/LoadCells_OOP.png)
