#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    This file contains an example map.
"""

# Libraries
import numpy as np

# Own Modules
from Hold import Hold

__author__ = "Emre Cemal Gonen"
__copyright__ = "Copyright (C) 2019, Emre Cemal Gonen"
__credits__ = ["Prof. Dr. Volkan Patoglu", "Sabanci University"]
__version__ = "1.0"
__email__ = "emrecemal@sabanciuniv.edu"

# Number of holds
hold_number = 10

# Random Seed
np.random.seed(42)

# Coefficient of Static Friction
mu = np.random.uniform(0.2, 1, hold_number)


def get_map():
    h0 = Hold(0, (0.35, 0.35), 0.03, 0.01, np.pi / 8, mu[0])
    h1 = Hold(1, (0.35, 0.58), 0.03, 0.01, np.pi / 8.1, mu[1])
    h2 = Hold(2, (0.45, 0.35), 0.03, 0.01, np.pi / 8.3, mu[2])
    h3 = Hold(3, (0.47, 0.59), 0.03, 0.01, np.pi / 8.2, mu[3])
    h4 = Hold(4, (0.55, 0.38), 0.03, 0.01, -np.pi / 7, mu[4])
    h5 = Hold(5, (0.53, 0.63), 0.03, 0.01, np.pi / 8, mu[5])
    h6 = Hold(6, (0.64, 0.42), 0.03, 0.01, -np.pi / 7.3, mu[6])
    h7 = Hold(7, (0.62, 0.64), 0.03, 0.01, np.pi / 6, mu[7])
    h8 = Hold(8, (0.74, 0.43), 0.03, 0.01, -np.pi / 7.5, mu[8])
    h9 = Hold(9, (0.72, 0.65), 0.03, 0.01, np.pi / 6.2, mu[9])

    return [h0, h1, h2, h3, h4, h5, h6, h7, h8, h9]

