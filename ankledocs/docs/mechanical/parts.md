
## Ankle Exoskeleton Mechanical Design Modifications

The original mechanical design assembly can be found in the NeuroC Lab OneDrive folder following:

### Pulley Cable Attach

The original pulley cable attach design had multiple issues:

- the screws used to assemble them to the pulley were touching, leaving no free space and thus making assembling the load cells difficult
- no fillets to reinforce the thin walls
- the side thin wall was only present on one side, making the part unnecessarily asymmetric (takes longer to adjust)

The new design takes the following into consideration:

- the cable stays tangential to its first point of contact to the pulley
- there is some free space in between the 2 screws
- the increased dimensions still make sure that no part will touch the floor
- fillets added in inner edges
- printed using CFRP (?)

Original Part: ![Original PulleyCableAttach](../../mechanical/images/motor-mount.png)
Revised Part: ![Revised PulleyCableAttach](../../mechanical/images/motor-mount.png)

## Waist Belt Design

General design considerations:

- One belt that can be used for unilateral and bilateral exoskeletons
- Can hold the weight of the exoskeleton + batteries + hardware with minimal load on the user

### PCB Pocket

The PCB design can be found here:

Specific design considerations: 

- Hold PCB and PCB case
- Allow space for cables and connections

The CAD file for the PCB pocket can be found here: ![Belt PCB Pocket](../CAD-files/pcb-pocket.prt)

### Battery Pockets (4)

Specific design considerations:

- Hold one or two batteries at a time
- Can attach and detach from belt
- Allow space for two large cables
- Thermal safety

The CAD file for the battery pocket can be found here: ![Belt Battery Pocket](../CAD-files/battery-pocket.prt)

## Test Bench Parts

### Fixing the Motors (3D Print)

The current part takes into all these considerations:

- uses the holes on the motor's back to screw the motor to the part
- leaves ample space at the bottom for the motor cable connections
- ensures unobstructed motor rotation facing outward the test bench
- minimizes vibrations at high torque/velocity motor testing operations
- holds the weight of the motor
- fixes to the test bench with screws through one hole and one slot


![Motor Test Bench Part](../../mechanical/images/motor-mount.png) DOESN'T WORK YET; ADD IMAGE IN ISOMETRIC VIEW


The CAD file can be found in SolidWorks using: [Motor Test Bench Part](../CAD-files/motor-mount.sldprt)


### Fixing The Exo Horizontally (3D Prints)

The current part takes into all these considerations:

- leaves space to adjust by hand the two bowden cable knobs
- keeps the exoskeleton horizontally placed without slipping or rotations
- minimizes non-necessary plastic use while ensuring enough part strength
- fixes to the test bench with screws through one hole and one slot

The following consists of 3 parts:

- The base, which is fixed to the test bench through 2 screws
- The cap, which sandwiches the exo in between itself and the base. It is fixed to the base through 2 screws and 2 nuts
- The clip, which is placed in between the knobs to prevent sliding along the exoskeleton's parallel axis

Each of the he CAD files can be found in Siemens NX (TO UPLOAD)
Base: [Ankle Test Bench Part - Base](../CAD-files/ankle-mount-base.prt)
Cap: [Ankle Test Bench Part - Cap](../CAD-files/ankle-mount-cap.prt)
Clip: [Ankle Test Bench Part - Clip](../CAD-files/ankle-mount-clip.prt)