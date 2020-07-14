#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Libraries
import numpy as np
from scipy.optimize import linprog

# Own Modules
from RobotData import RobotData

__author__ = "Emre Cemal Gonen"
__copyright__ = "Copyright (C) 2019, Emre Cemal Gonen"
__credits__ = ["Prof. Dr. Volkan Patoglu", "Sabanci University"]
__version__ = "1.0"
__email__ = "emrecemal@sabanciuniv.edu"


def calculate_support_polygon(hold_arr, force_limit=(0., 15.)):
    """
        This function calculates support polygon for given holds.
        It uses Linear Programming from Scipy

    :param hold_arr: An array containing holds
    :param force_limit: A defined force limit
    :return: A range of Support Polygon
    """

    # Get the mass of the robot
    mass = RobotData.m

    # Number of Holds which contributes for the polygon
    # Number of rows
    n = len(hold_arr)

    # Number of columns
    m = 2 * n + 1

    # Creating Matrices
    # Upper-Bound Arrays
    Aub1 = np.zeros(n * m)
    for i in range(n):
        Aub1[(m + 2) * i] = -1
        Aub1[(m + 2) * i + 1] = -1
    Aub1 = Aub1.reshape((n, m))

    Aub2 = np.zeros(n * m)
    for i in range(n):
        Aub2[(m + 2) * i] = 1
        Aub2[(m + 2) * i + 1] = 1
    Aub2 = Aub2.reshape((n, m))

    Aub = np.vstack((Aub1, Aub2))
    bub = np.hstack((np.ones(n) * -force_limit[0], np.ones(n) * force_limit[1]))

    # Equivalence Arrays
    Aeq1 = np.array([[], [], []])
    for h in hold_arr:
        # Including Minimum and Maximum Edges of Friction Cone using
        # get_f_matrix function defined below
        F = get_f_matrix((h.x, h.y), h.fr_cone_min, h.fr_cone_max)
        Aeq1 = np.hstack((Aeq1, F))

    Aeq2 = np.array([[0], [0], [-mass * 9.81]])

    Aeq = np.hstack((Aeq1, Aeq2))
    beq = np.array([0, mass * 9.81, 0])

    c_min = np.hstack((np.zeros(2 * n), np.array([-1])))
    c_max = np.hstack((np.zeros(2 * n), np.array([1])))

    # Calculating Minimum and Maximum Results
    res_min = linprog(c=c_min, A_ub=Aub, b_ub=bub, A_eq=Aeq, b_eq=beq)
    res_max = linprog(c=c_max, A_ub=Aub, b_ub=bub, A_eq=Aeq, b_eq=beq)

    # Printing Result
    print(res_min)

    return np.array([-res_min.fun, res_max.fun])


def get_f_matrix(r, min_vec, max_vec):
    """
        This function used in internal step to have a compact form

    :param r: position vector of an hold
    :param min_vec: The edge of the friction cone as unit vector whose slope is negative
    :param max_vec: The edge of the friction cone as unit vector whose slope is positive
    :return: F matrix to be used in the above function
    """
    return np.array([[min_vec[0], max_vec[0]],
                     [min_vec[1], max_vec[1]],
                     [np.cross(r, min_vec).item(0), np.cross(r, max_vec).item(0)]])