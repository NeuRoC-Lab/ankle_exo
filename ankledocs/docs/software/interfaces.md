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

To run the unified CAN controller example code (`CAN_unified.cpp`) you will need to run the pio environment `CAN_teensy41` or `CAN_uno_r4` depending on which board you are using. 
As a reminder, the general format to run an environment in platformio is `pio run -e yourenvname` where `yourenvname` is the target environment. 

#### Design update (25/06)

After the Servo mode was successfully tested on June 24th we are now implementing a separate class for the MIT and Servo mode. Those supercless are `CANMotorMIT` (MIT mode) and `ServoCANMotor` (Servo mode). Note that these two classes need to been validated again 


| Class                   | Mode | Board          | Platformio Environment        |
|-------------------------|-----:|:---------------|-------------------------------|
| `CANMotorMIT_Teensy`    |  MIT | Teensy 4.1     | `pio run -e MIT_CAN_teensy41` |
| `CANMotorMIT_Renesas`   |  MIT | Arduino Uno R4 | `pio run -e MIT_CAN_uno_r4`   |
| `ServoCANMotor_Renesas` | Servo | Arduino Uno R4 | `pio run -e Servo_CAN_uno_r4` |
| `ServoCANMotor_Teensy`  | Servo | Teensy 4.1     | NOT IMPLEMENTED YET           |

The following commands can be run (rightmost command) to execute the MIT/Servo demo code on the Teensy/Arduino Uno R4. 
Note the following : 

* You must wire the CANTX/CANRX wires accordingly to the Teensy or Arduino UNO R4.

* For the Arduino 

  | OpenExo PCB                     | Arduino |
  |---------------------------------|--------:|
  | CANTX _through voltage divider_ |     D10 |
  | CANRX                           |     D13 | 
  | GND                             |     GND | 
  | 3.3V                            |   3.3V  | 

* For the Teensy


| OpenExo PCB                     | Arduino |
|---------------------------------|--------:|
| CANTX  |     D22 |
| CANRX                           |     D23 | 
| GND                             |     GND | 
| 3.3V                            |    3.3V | 


#### Serial communication 

The serial communication to control the motor works as follow : 

##### MIT mode
* `start id xx` : start the motor (it will otherwise remain deaf to commands)
* `stop id xx` : stop the motor and put it in idle mode
* `zero id xx` : set the motor's zero

`set` | `id xx`: 

  - `pos x.x` : position (in radians from -12.0 to +12.0)
  - `vel x.x` : velocity (in radians/s from -45.0 to +45.0)
  - `kp x.x` : proportional gain (for position)
  - `kd x.x` : derivative gain (for velocity)
  - `trq x.x`  : feedforward torque (be careful with it when using the motor with no load attached to the shaft as it will spin freely while accelerating)

_Note that the order the parameters are given does not matter. One / more parameter can be updated at once_
##### Servo Mode

`set` | `id xx`  :

  - `duty x.x` : set duty cycle mode
  - `current x.x` : set current mode (proportional to torque T = kI)
  - `brake x.x` : set brake current mode
  - `rpm x.x` : set rpm mode
  - `pos x.x` : set position mode
  - `pos x.x rpm x.x acc x.x` : set position velocity acceleration loop mode
  -  `origin 0` : set encoder origin

  - _Note that only ONE mode can be active at a time_
### LoadCell class

The `LoadCell` class is the main class used to interface the motor. 

For the same reason as mentionned earlier, I gave the possibility to run this code on different board achitectures, namely 1) The Arduino UNO R3 (AtmelAVR), 2) The Arduino UNO R4 (Renesas) and 3) the Teensy 4.1
Because the `LoadCell` class simply requires to read a voltage from an Analog pin all of the boards can be used. However the implementation slightly differs from board to board. For example, the Teensy can only support voltages in the 0-3.3V range while Arduino UNO R3/R4 have an extended ADC input voltage range of 0-5V. 
Please keep that in mind when setting up the strain gauge amplifier as driving voltages beyond the GPIO limit can permanently damage the board.

Specific boards may be used for specific needs : 

1. The Arduino UNO R3 is the simplest option and most straightforward. I recommend starting with this
2. All boards can be used to interface the integrated INA125U on the [PCB](/electrical/pcb) however _pay attention to the voltage ranges_. Assert statements during compile time will throw errors if the theoretical `Vo` range goes beyond the ADC voltage range.


![OOP Diagram](../images/LoadCells_OOP.png)

