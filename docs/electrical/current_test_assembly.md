## IMPORTANT notes for using the current setup for the Exo testbench

The setup looks like this from the top : 

![Image of the boards for test bench](./images/Test%20bench%20configuration.png)

The main components in the configuration are :

1. The _Main_ board with most of the components soldered on (screwed onto the orange socket). It handles :
    - hardware interface and circuitry to/from the teensy
    - load cell amplification _for the two load cells currently mounted on the test bench_ (for more information about the connections for that header refer to [the load cell header pinout page](/electrical/pcb))

* The header must be put in such a way that the "OUT" inscription in dark ink appears on the outside of the board edge (see picture below)
![load cell encoder](./images/testbench%20loadcell%20header.png)

2. The _Secondary_ and temporary "CAN only board" in the white socket. Because I ran into a hardware issue with trying to make the CAN chip work on the main PCB, I temporarily used another separate PCB that will exclusively handle CAN.
   - The Teensy being on the _Main_ board, I also had to solder on-the-fly cables to interface the CAN chip on the remote board. _Please do not disconnect them_

![temporary soldered cables for powering remote CAN board](./images/teensy%20temporary%20CAN%20extension.png)
![temporary soldered cables for powering remote CAN board](./images/teensy%20temporary%20CAN%20extension%202.png)

Side note for SPI : 
* The current PCB version (rev. 1.0.0) _DOES NOT_ support using SPI through the Teensy, as I realized after ordering the boards that the connections were mismatched for the level shifters raising / lowering 3.3V / 5V to/from the Teensy and the Encoders.

As such you must use the Arduino Uno R3 or R4 in order to read the encoder value over SPI. The Teensy won't work.

![Arduino Uno SPI setup for dual encoder reading](./images/encoder%20breakout%20connector.png)