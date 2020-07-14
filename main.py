#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Own Modules
from Robot import Robot
import SampleMap
import SupportPolygon
import ReachabilityCircleCheck
import Visualize


__author__ = "Emre Cemal Gonen"
__copyright__ = "Copyright (C) 2020, Emre Cemal Gonen"
__credits__ = ["Prof. Dr. Volkan Patoglu", "Sabanci University"]
__version__ = "1.0"
__email__ = "emrecemal@sabanciuniv.edu"


def main(indices):
    """
        This function controls each module.Please refer inside on them
        for further information

    :param indices: an array containing indices of holds
    """

    # Get Map from the sample map
    all_holds = SampleMap.get_map()

    # Choose the desired holds for the indices coming from input
    desired_holds = [all_holds[i] for i in indices]

    # Draw circles and find the intersection points for reachability
    dxs, dys, x, y = ReachabilityCircleCheck.circle_intersections(desired_holds[1:4])

    # Get the robot, place the robot at the center of intersection points with an arbitrary orientation 0.1 rad
    robot = Robot((x, y), 0.1)

    # Placed robot's end effectors to the desired holds
    q = robot.get_ik([desired_holds[0], desired_holds[1], desired_holds[2], desired_holds[3]])

    # Get the sketch info for the robot
    pelvis, head, limb_lst = robot.get_sketch_plot()

    # Get the center of mass
    com = robot.get_com()

    # Calculate support polygon
    sp_points = SupportPolygon.calculate_support_polygon(desired_holds[1:4])

    # Plot everything
    # Visualize.plot(holds=all_holds, sp_points=sp_points, int_points=[dxs, dys],
    #                desired_holds=desired_holds, robot_sketch=[pelvis, head, limb_lst, com])

    Visualize.plot(holds=all_holds, int_points=[dxs, dys], desired_holds=desired_holds[1:4], sp_points=sp_points)

    # If you don't want to plot everything delete the related argument
    # For example if you want to draw just holds, use:
    # Visualize.plot(holds=all_holds)

    # Or if you want to draw support polygon and the robot only, use:
    # Visualize.plot(sp_points=sp_points, robot_sketch=[pelvis, head, limb_lst, com])

    # You can add holds to the previous one
    # Visualize.plot(sp_points=sp_points, robot_sketch=[pelvis, head, limb_lst, com], holds=all_holds)


if __name__ == '__main__':
    """
        The inside the function represents the first limb moves to hold 3, 
        second limb to hold 1, and third to 0, fourth to 2
    """
    main([3, 1, 0, 2])

    # You can try other configuration
    # main([7, 5, 4, 6])

