#pragma once
#include <cstdint>
#include <Arduino.h>
#include <SPI.h>
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#ifndef HW_VERSION_MAJOR
    #define HW_VERSION_MAJOR 1
#endif

#ifndef HW_VERSION_MINOR
    #define HW_VERSION_MINOR 0
#endif

#ifndef HW_VERSION_PATCH
    #define HW_VERSION_PATCH 0
#endif

#pragma message "Using HW version " STR(HW_VERSION_MAJOR) "." STR(HW_VERSION_MINOR) "." STR(HW_VERSION_PATCH)

//#pragma message "HW_VERSION_MAJOR = " STR(HW_VERSION_MAJOR)
//#pragma message "HW_VERSION_MINOR = " STR(HW_VERSION_MINOR)
//#pragma message "HW_VERSION_PATCH = " STR(HW_VERSION_PATCH)

// default to first (v1.0.0) version if the compiler is missing the hardware version. That way our code is "safe" and there is no ambiguity

#define HW_VERSION_ENCODE(major, minor, patch) \
((major) * 10000 + (minor) * 100 + (patch))

#define HARDWARE_VERSION \
HW_VERSION_ENCODE(HW_VERSION_MAJOR, HW_VERSION_MINOR, HW_VERSION_PATCH)

#define HW_VERSION_AT_LEAST(major, minor, patch) \
(HARDWARE_VERSION >= HW_VERSION_ENCODE(major, minor, patch))

#define HW_VERSION_AT_MOST(major, minor, patch) \
(HARDWARE_VERSION <= HW_VERSION_ENCODE(major, minor, patch))

#define HW_VERSION_EQUALS(major, minor, patch) \
(HARDWARE_VERSION == HW_VERSION_ENCODE(major, minor, patch))

#if defined(PLATFORM_TEENSY41) && defined(PLATFORM_NORDIC)
#error "You specified both PLATFORM_TEENSY41 and PLATFORM_NORDIC. Please select ONLY ONE platform to compile"
#endif


constexpr uint8_t loadCellCount = 4;

using Pin = int; // defining a type for pins to make intent clearer

// load cell pins can be defined on both Teensy and Nano depending on hardware revision
struct SingleEndedLoadCellPins
{
    std::array<Pin, 4> outputs; // these are the Vo pins referenced to GND
};

// a enum class to do the mapping between load cell id and corresponding load cell location. All the code uses
// This ensures that we can address the load cells by their location and not by their raw pin
enum class LoadCellId : std::uint8_t
{
 Left1, // 0
 Left2, // 1
 Right1, // 2
 Right2 // 3
};

// these are now FIXED

struct INA125UParams {
 static constexpr float gainR = 50.0f; // used to be 10 Ohms on V1.1.0, now 50Ohms on V1.1.1
 static constexpr float ampGain = 4.0f + 60000.0f / gainR;

 static constexpr float IAref = 2.5f;
 static constexpr float Vexc  = 5.0f;
 // change Vexc to equal 5.0f or 3.3f if using an external excitation voltage (coming for example from the regulated 5V/3.3V on the PCB)
} inaParams;

// ============== Teensy definitions  ==============
#if defined(PLATFORM_TEENSY41)

namespace board::teensy41
{

 struct LevelShifterPins
 {
  Pin oe1;
  Pin oe2;
 };

 struct EncoderPins
 {
  Pin leftChipSelect;
  Pin rightChipSelect;
 };

 struct TeensyPins
 {
  EncoderPins encoders;
  LevelShifterPins levelShifters;

#if HW_VERSION_AT_MOST(1, 1, 0)
  SingleEndedLoadCellPins loadCells;
#endif

 };

 inline constexpr TeensyPins pins{
  .encoders = {
   .leftChipSelect = 0,
   .rightChipSelect = 7
},

.levelShifters = {
   .oe1 = 6,
   .oe2 = 3
},

#if HW_VERSION_AT_MOST(1, 1, 0)
.loadCells = {
   .outputs = {A6, A7, A11, A10}
}
#endif
 };
 inline SPIClass& nanoSpi = SPI;
 inline HardwareSerial& nanoUart = Serial8;
 // CAN configurations
 //inline FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> motorCan;
 inline constexpr std::uint32_t motorCanBaud = 1'000'000U;
// Encoder SPI baudrate
 inline constexpr std::uint32_t encoderSPIBaud = 500'000U;
}

// ============== Arduino Nano definitions  ==============
#elif defined(PLATFORM_NORDIC)
namespace board::nano
{
 struct ExcitationLoadCellPins
 {
  std::array<Pin, 4> outputs;
 };
// excitationloadcell pins are only used in version 1.1.1+
struct NanoPins {
// in a struct order matters for in-place initialization so put the latest hardware revisions at the end
#if HW_VERSION_AT_LEAST(1, 1, 1)
 ExcitationLoadCellPins excitationPins;
 SingleEndedLoadCellPins loadCells;
#endif
};

 inline constexpr NanoPins pins{
#if HW_VERSION_AT_LEAST(1, 1, 1)
  ExcitationLoadCellPins{
   std::array<Pin, 4>{A1, A3, A5, A7}
  },
  SingleEndedLoadCellPins{
   std::array<Pin, 4>{A0, A2, A4, A6}
  }
#endif
 };

#if HW_VERSION_AT_LEAST(1,1,1)
 inline SPIClass& teensySpi = SPI;
#endif
 inline HardwareSerial& teensyUart = Serial1;
 inline constexpr std::uint32_t teensyUartBaudrate = 230'400;
}
#endif