#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Libraries
from math import pi

__author__ = "Emre Cemal Gonen"
__copyright__ = "Copyright (C) 2019, Emre Cemal Gonen"
__credits__ = ["Prof. Dr. Volkan Patoglu", "Sabanci University"]
__version__ = "1.0"
__email__ = "emrecemal@sabanciuniv.edu"


class RobotData:
    """
        These are robot hard parameters
        All dimensions are in SI units.
            meter for length
            kilograms for mass
            second for time
    """

    # Radius of Pelvis of the robot
    r = 0.05

    # Length of Upper Limb
    l1 = 0.08

    # Length of Lower Limb
    l2 = 0.04

    # Total Mass of the robot
    m = 3.

    # Mass of each motor, note that there are 8 motors
    m_motor = 0.25

    # Attach Angles of Each Limb
    attach_ang = [pi / 6, 5 * pi / 6, 7 * pi / 6, -pi / 6]
