As explained in [the software/encoder page](/old%20software/encoder), the two encoders will share one SPI bus

The wiring of the two AMT-14C-0-036-1 cables to the main board was done as follows :

![wiring of encoder cables](../images/Encoder_wiring.png)

The two cables merge together and on the other side is a 1x7 2.54mm female jumper header which will be connected on the PCB at the location specified with the label `SPIx2`

![KiCAD rendering of SPI header](../images/SPIheader_kicad.png)

![Pinout of the female header](../images/Encoder_header_pinout.png)

Note that the side with black ink on the red heatshrink should be put on the out-side of the board i.e face back from the PCB