# Encoders 

the AMT20 encoders series comes in two version : incremental encoders (through quadrature outputs) or absolute encoders (through SPI). Our encoder model (The AMT203-V) is an absolute position encoder and is thus interfaced over SPI.

## Absolute encoder : 

The encoder features en internal register which updates the absolute position at a rate of 40us (25 kHz). It is interfaced over SPI

![SPI Wiring diagram](../images/SPI-encoder.png)

The encoder is both 3.3V and 5V tolerant, however the CIPO (controller-in peripheral out) pin will be 5V voltage level, therefore when using a 3.3V logic MCUs (like the Teensy or the Nano) you MUST shift down the voltage from CIPO before connecting it to the MCU. 

When using an Arduino Uno R3 / R4, this is not an issue. However a typical setup for a 3.3V MCU is : 

![Voltage divider for CIPO](../images/SPI-voltdiv.png)

Using the voltage divider equation : 

\[
V_{out} = V_{in} \times \frac{R_2}{R_1 + R_2} \iff
V_{out} = 5 \times \frac{20k}{10k + 20k}  \iff
V_{out} = 5 \times \frac{20}{30} \iff
V_{out} = 3.33V
\]

We can use setup a shared SPI bus and assign a CS pin to each of the two encoders (left and right) so that only one encoder uses the SPI bus at a time. This means that to sync the two encoders's update we have to reduce the overall update rate by a factor of 2. 

![SPI dual encoder configuration](../images/SPI-dual-encoder.png)

### Arduino UNO SPI pinout for testing the encoder

When using the Arduino UNO R3/R4 to interface the encoder over SPI, use the following table to connect the SPI pins (CIPO/COPI/CS/SCK)

| COPI (Cont. Out Per. In) | CIPO (Cont. In Per. Out) | SCK (Serial Clock) | CS (Chip Select) |
|-------------------------:|-------------------------:|-------------------:|:----------------:|
|                      D11 |                      D12 |                D13 |       D10        |

## Incremental encoder : 

We also have access to the encoder's raw quadrature outputs (A/B/Z). The outputs A/B are used to count the increments of the encoder and tell the direction of rotation to update the counter (A→B vs B→A) while Z is the index, and generates a pulse once every full rotation

All of the Teensy 4.1's digital pins support interrupts, which makes the incremental encoder method a good candidate for fast encoder resolution and reading.

Somehow the quadrature signals don't work ???

![Quadrature signals](../images/quadrature signals.png)


## Ressources 

The encoder datasheet can be found here : [encoder datasheet](../files/amt20-datasheet.pdf)