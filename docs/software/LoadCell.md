### LoadCell class

The `LoadCell` class is the main class used to interface the motor.

For the same reason as mentionned earlier, I gave the possibility to run this code on different board achitectures, namely 1) The Arduino UNO R3 (AtmelAVR), 2) The Arduino UNO R4 (Renesas) and 3) the Teensy 4.1
Because the `LoadCell` class simply requires to read a voltage from an Analog pin all of the boards can be used. However the implementation slightly differs from board to board. For example, the Teensy can only support voltages in the 0-3.3V range while Arduino UNO R3/R4 have an extended ADC input voltage range of 0-5V.
Please keep that in mind when setting up the strain gauge amplifier as driving voltages beyond the GPIO limit can permanently damage the board.

Specific boards may be used for specific needs :

1. The Arduino UNO R3 is the simplest option and most straightforward. I recommend starting with this
2. All boards can be used to interface the integrated INA125U on the [PCB](/electrical/pcb) however _pay attention to the voltage ranges_. Assert statements during compile time will throw errors if the theoretical `Vo` range goes beyond the ADC voltage range.

