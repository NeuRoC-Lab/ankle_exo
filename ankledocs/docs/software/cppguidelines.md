An effort was made to avoid having to either`

1. Reference the four load cells by their hardware pins. An abstraction was used in `Board.h` with a `enum` called `LoadCellId` which maps the position of the load cells (`Left1`,`Right2`, etc) to their hardware pins so the programer doesn't have to deal with low level pinout

2. Remember the order in which the load cells are defined. As much as possible the order is defined once then we use a pointer to refer to the array of load cells so that a mismatch in the order doesn't happen (*TODO: make sure this is properly enforced*)


!!! note "Cppiso guideline F.60"
    _[F.60](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#f60-prefer-t-over-t-when-no-argument-is-a-valid-option): Prefer T* over T& when “no argument” is a valid option_

>This recommendation is applied with the `CANMotorMIT_Handler` signature which takes one `CANMotorMIT&` reference for the first motor, while the second motor is made optional with the `CANMotorMIT* rightMotor = nullptr` parameter which by default will deactivate the second motor.