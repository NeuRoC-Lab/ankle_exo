### Usage of `CANMotorMIT.h` and `ServoCANMotor.h` erre— CAN Motor API

1. In both MIT and Servo modes, auxiliary libraries such as `SerialMotorControl.h` use the generic `CANController` type name for the selected motor controller.

   Therefore, after including the appropriate controller header, define `CANController` as an alias for the platform-specific implementation.

   For MIT mode with a Teensy 4.1:
```Cpp
#include "CANMotorMIT.h"
using CANController = CANMotorMIT_Teensy;  
```

   When using an Arduino UNO R4 instead:

```Cpp
using CANController = CANMotorMIT_Renesas;
```

   For Servo mode with a Teensy 4.1:

```Cpp
#include "ServoCANMotor.h"
using CANController = ServoCANMotor_Teensy;
```

   When using an Arduino UNO R4 instead:

```c++
using CANController = ServoCANMotor_Renesas;
```

2. In MIT mode specifically, the user must define:

    * the software command constraints;
    * the motor running constraints.

   These values should be configured in `MotorConfig.h`.

3. Create a `MotorCmd` object that will store the commands sent to the motor.

   Example for MIT mode:

```c++
MotorCmd motor1Cmd {
      .position = 0.0f,
      .velocity = 0.0f,
      .torque = 0.0f,
      .kp = 0.0f,
      .kd = 0.0f
   };
```

4. Instantiate a `CANController` object.

   Example for MIT mode:

```c++
CANController motor(
       MOTOR_ID,
       &motorParams,
       &motorSoftwareConstraints,
       &motorRunningConstraints,
       motor1Cmd,
       printRate
   );
```

   The arguments are:

    * `MOTOR_ID`: the motor CAN ID. It can be checked or configured using the CubeMars Upper Computer software.
    * `motorParams`: the nominal motor communication parameters.
    * `motorSoftwareConstraints`: the limits applied to commands before they are sent to the motor.
    * `motorRunningConstraints`: the safety limits checked against motor feedback.
    * `motor1Cmd`: the `MotorCmd` object containing the current user command.
    * `printRate`: the motor status will be printed to the Serial Monitor once every `printRate` received frames.

   The Servo-mode constructor may use a different set of arguments because software and running constraints are currently specific to the MIT controller.

5. Include `SerialMotorControl.h` to enable control through the serial interface:

```c++
   #include "SerialMotorControl.h"
```

   Include this header only after the `CANController` alias and motor object have been defined, if the library depends on them.

6. Create the serial controller object:

```c++
   SerialMotorControl serialControl(Serial, motor1Cmd, motor);
```

7. In `setup()`, initialize the motor controller:

```c++
motor.begin();
```

   Reset and enable the motor using:

```c++
   if (!motor.resetMotor()) {
       Serial.println("Failed to start motor");
   } else {
       Serial.println("Motor started successfully");
   }
```

8. In `loop()`, call the following functions to process CAN communication and serial commands:

```c++
   void loop()
   {
       motor.update();
       serialControl.update();

       delay(10);
   }
```

   The delay is currently required for reliable serial command handling and should not be removed until the timing behavior has been investigated.
