"""
For testing purposes, such as comparing measured loadcell torque vs commanded motor torque

This test allows the user to send motor torque commands through the high-level single-exo.py API
It records the values of the two load cells associated to the motor

Used load cells:
left 1
right 2

To run the test:
python software/src/python-scripts/SingleLeg/torque-test.py
"""

import time

from single-exo import Exoskeleton

sample_interval = 0.01

# select the two load cells used for this ankle
l1 = "left1"
l2 = "right2"

def record_loadcells(exo, rec_time):
    """
    Records the force measured by both load cells
    :param exo:
    :param rec_time:
    :return:
    """