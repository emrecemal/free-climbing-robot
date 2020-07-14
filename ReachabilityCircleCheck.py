#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    This function finds intersection region of circles whose radius
    is the wing length of robot. Although it doesn't say exact workspace
    of the robot, but it gives an estimate about where it is
"""

# Libraries
from sympy.geometry import Circle, Point

# Own Modules
from RobotData import RobotData

__author__ = "Emre Cemal Gonen"
__copyright__ = "Copyright (C) 2019, Emre Cemal Gonen"
__credits__ = ["Prof. Dr. Volkan Patoglu", "Sabanci University"]
__version__ = "1.0"
__email__ = "emrecemal@sabanciuniv.edu"


def circle_intersections(holds):
    """
        This returns intersection points of the circles
    :param holds: an array containing holds
    :return: intersection points (x coordinates, y coordinates and middle point)
    """
    def _check_point_in_circle(px, py):
        """
            This sub-function checks whether a point is in a circle or not
        :param px: X coordinate of the point
        :param py: Y coordinate of the point
        :return: Boolean
        """
        distance_to_holds = []
        for h in holds:
            d_sq = (px - h.x) ** 2 + (py - h.y) ** 2
            distance_to_holds.append(round(d_sq, 6))

        return all(i <= round(circle_radius ** 2, 6) for i in distance_to_holds)

    # Circle Radius same for all circles
    circle_radius = RobotData.l1 + RobotData.l2 + RobotData.r

    # Number of holds
    hold_num = len(holds)

    # Create of circles as Symbolic because Sympy has solver to find intersection points
    circle_list = []
    for i in range(hold_num):
        circle_list.append(Circle(Point(holds[i].x, holds[i].y), circle_radius))

    # Find Intersections
    intersection_list = []
    for i in range(hold_num):
        circle = circle_list[i]
        check_circles = circle_list[i + 1:]
        for check in check_circles:
            intersection_list.append(circle.intersection(check))

    # Get Intersection Points
    points = []
    for intersections in intersection_list:
        if len(intersections) == 1:
            intersections.append(intersections[0])
        p0 = intersections[0]
        p1 = intersections[1]
        points.append([(float(p0.x), float(p0.y)), (float(p1.x), float(p1.y))])

    # Now all intersection points are found, but we desire only intersections of all circles
    # Get the points only desired
    xs = []
    ys = []
    for inter_points in points:
        p0x = inter_points[0][0]
        p0y = inter_points[0][1]
        p1x = inter_points[1][0]
        p1y = inter_points[1][1]

        if _check_point_in_circle(p0x, p0y):
            xs.append(p0x)
            ys.append(p0y)

        if _check_point_in_circle(p1x, p1y):
            xs.append(p1x)
            ys.append(p1y)

    return xs, ys, sum(xs) / len(xs), sum(ys) / len(ys)