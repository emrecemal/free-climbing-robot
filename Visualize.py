#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Libraries
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# Own Modules
from RobotData import RobotData


__author__ = "Emre Cemal Gonen"
__copyright__ = "Copyright (C) 2019, Emre Cemal Gonen"
__credits__ = ["Prof. Dr. Volkan Patoglu", "Sabanci University"]
__version__ = "1.0"
__email__ = "emrecemal@sabanciuniv.edu"


def plot(holds=None, robot_sketch=None, sp_points=None, int_points=None, desired_holds=None):
    """
        This function plots what it asked, if the input arguments are not stated it doesn't draw.
        In order to draw them please give the proper input.

    :param holds: a list of all holds on the map
    :param robot_sketch: a list of sketch info of robot [pelvis, head, limb_lst, com]
    :param sp_points: support polygon points [x_min, x_max]
    :param int_points: a list of intersection points from circle and the middle
                        point of intersections [x_int, y_int, x_m, y_m]
    :param desired_holds: a list of desired holds, this can be three or four element
    :return: Nothing numerically, but gives the plot
    """

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 10))

    # Dimensions are 1m to 1m, set aspect ratio to 1
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    ax.set_aspect(1)

    # Label axes and titles
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    # plt.suptitle('FREE CLIMBING ROBOT: Insert Super Title Here')
    plt.suptitle('FREE CLIMBING ROBOT')
    # plt.title('Insert Title Here')
    plt.title('Combination of stability and reachability check')

    # Drawing Robot
    if robot_sketch is not None:
        # Add Robot
        pelvis, head, limb_lst, com = robot_sketch
        ax.add_artist(pelvis)
        ax.add_line(head)
        ax.plot(com[0], com[1], marker='+', ms=10, mec='m', c='m')
        for limb in limb_lst:
            ax.add_line(limb)

    # Drawing Holds
    if holds is not None:
        # Add Holds
        i = 0
        for h in holds:
            ax.add_line(h.get_hold_plot())
            ax.add_line(h.get_normal_plot())
            ax.add_line(h.get_cone_plot())
            text = h.get_index_on_plot()
            ax.text(x=text[0], y=text[1], s=text[2], fontsize=text[3])

    # Drawing Support Polygon
    if sp_points is not None:
        # Add Support Polygon
        y_d = np.linspace(0, 1, 100)
        ax.fill_betweenx(y_d, sp_points[0], sp_points[1], color=(0.1, 0.2, 0.5, 0.2))

    # Drawing Circles and the intersection points
    # if int_points is not None:
    #     # Add Circles and intersections
    #     plt.scatter(int_points[0], int_points[1])
    #     for h in desired_holds:
    #         c = patches.Circle((h.x, h.y), radius=(RobotData.r + RobotData.l1 + RobotData.l2),
    #                            ec=(0, 0, 1, 0.5), fc=(0.1725, 0.4588, 0.9412, 0.2))
    #         ax.add_patch(c)

    # Show the plot
    plt.show()