#pragma once
#include <cstdint>
#include <array>
#include <Arduino.h>
#include <SPI.h>

#if defined(PLATFORM_TEENSY) && defined(PLATFORM_NANO)
#error "You specified both PLATFORM_TEENSY and PLATFORM_NANO. Please select ONLY ONE platform to compile"
#endif


constexpr uint8_t LoadCellCount = 4;

using Pin = int; // defining a type for pins to make intent clearer


struct SingleEndedLoadCellPins
{
    std::array<Pin, 4> outputs; // these are the Vo pins referenced to GND (array of 4)
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
#if defined(PLATFORM_TEENSY)

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
 };
 inline SPIClass& nanoSpi = SPI;
 inline HardwareSerial& nanoUart = Serial8;
 // CAN configurations
 //inline FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> motorCan;
 inline constexpr std::uint32_t motorCanBaud = 1'000'000U;
// Encoder SPI baudrate
 inline constexpr std::uint32_t encoderSPIBaud = 1'000'000U;
}

// ============== Arduino Nano definitions  ==============
#elif defined(PLATFORM_NANO)
namespace board::nano
{
 struct ExcitationLoadCellPins
 {
  std::array<Pin, 4> outputs;
 };
// excitationloadcell pins are only used in version 1.1.1+
struct NanoPins {
// in a struct order matters for in-place initialization so put the latest hardware revisions at the end
 ExcitationLoadCellPins excitationPins;
 SingleEndedLoadCellPins loadCells;
};

 inline constexpr NanoPins pins{

  ExcitationLoadCellPins{
   std::array<Pin, 4>{A1, A3, A5, A7}
  },
  SingleEndedLoadCellPins{
   std::array<Pin, 4>{A0, A2, A4, A6}
  }

 };


 inline SPIClass& teensySpi = SPI;

 inline HardwareSerial& teensyUart = Serial1;
 inline constexpr std::uint32_t teensyUartBaudrate = 230'400;
//TODO decide whether to keep this or simply assume every Serial interface uses 230'400 bauds
}
#endif