### C++ Programing Guidelines

#### MISC Code simplification

An effort was made to avoid having to either

1. Reference the four load cells by their hardware pins. An abstraction was used in `Board.h` with a `enum` called `LoadCellId` which maps the position of the load cells (`Left1`,`Right2`, etc) to their hardware pins so the programer doesn't have to deal with low level pinout

2. Remember the order in which the load cells are defined. As much as possible the order is defined once then we use a pointer to refer to the array of load cells so that a mismatch in the order doesn't happen (*TODO: make sure this is properly enforced*)

#### C++ ISO Core Guidelines remarks

!!! note "CppISO guideline F.60"
    _[F.60](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#f60-prefer-t-over-t-when-no-argument-is-a-valid-option): Prefer T* over T& when “no argument” is a valid option_

>This recommendation is applied with the `CANMotorMIT_Handler` signature which takes one `CANMotorMIT&` reference for the first motor, while the second motor is made optional with the `CANMotorMIT* rightMotor = nullptr` parameter which by default will deactivate the second motor.

!!! note "C++ Core Guideline SF.2"
    _[SF.2](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#rs-inline)
    A header file must not contain object definitions or non-inline function definitions._

When a header contains a function definition and that header is included by multiple `.cpp` files, each `.cpp` file receives its own copy of the definition. If those object files are later linked into the same program, the linker may report a multiple-definition error because the same function has been defined in more than one translation unit.

`#pragma once` does not solve this problem. It only prevents the same header from being included multiple times within a single translation unit. It does not prevent the header from being included once by several different `.cpp` files.

Functions may still be defined in a header when they are declared `inline`, defined inside a class definition, or implemented as templates. In those cases, the language permits the same definition to appear in multiple translation units, provided that the definitions are identical.

Therefore, there are two common approaches:

* Place declarations in `.h` files and non-inline definitions in corresponding `.cpp` files.
* Keep implementations in header files, while ensuring that functions are `inline`, defined inside a class, or implemented as templates.

Although header-only code can reduce the number of source files, separating declarations and implementations can reduce compilation time, hide implementation details, and avoid recompiling every translation unit that includes a header whenever its implementation changes.

For this project, we chose to keep as much code as reasonably possible within the header files:

* This reduces the number of files and makes the project structure easier to navigate. `main.cpp` contains the program entry points, while the remaining files define the components used by the application.
* Many of the implementations are class-defined, templates, or explicitly marked `inline`, which allows them to be safely defined in headers without violating the One Definition Rule.

`#pragma once` is still used in every header, but only to prevent the same header from being included multiple times within a single translation unit. It does not, by itself, prevent multiple-definition linker errors across different `.cpp` files.

To validate this header-oriented workflow, an automated test was implemented following guideline SF.11.

!!! note "C++ Core Guideline SF.11"
    _[SF.11](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#sf11-header-files-should-be-self-contained): Header files should be self-contained._

A post-build Python script compiles each header independently using the appropriate platform definitions, such as `PLATFORM_TEENSY41` or `PLATFORM_NORDIC`.

This test detects:

1. Types, functions, constants, or macros that are used by a header but are not declared by one of its own direct includes.
2. Dependencies that were accidentally made available through the inclusion order of another header.
3. Platform-specific compilation errors that may otherwise remain hidden until the header is included from a different source file.

The test therefore verifies that each header can be included independently under every platform configuration for which it is intended.
