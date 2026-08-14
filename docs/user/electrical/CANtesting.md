### Validation of the onboard CAN

An external CAN module was used to test the proper operation of the CAN bus. The CAN bus was then made of two nodes : the SN65HVD230 transceiver on the PCB, and another SN65HVD230 transceiver on the external module.
> Note that because the PCB already includes a 120 \(\Omega\) termination, you MUST unsolder the 60 \(\Omega\) resistor on the external CAN transceiver module (use a voltmeter to identify which resistor to remove among the two. The other one is usually 10k \(\Omega\))