#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Libraries
import numpy as np
import matplotlib.lines as lines

__author__ = "Emre Cemal Gonen"
__copyright__ = "Copyright (C) 2019, Emre Cemal Gonen"
__credits__ = ["Prof. Dr. Volkan Patoglu", "Sabanci University"]
__version__ = "1.0"
__email__ = "emrecemal@sabanciuniv.edu"


class Hold:
    def __init__(self, id, position, length, height, slope, coeff_fric):
        """
            This class defines a hold in a map. The user can get plot objects using its methods.
            The geometry of a hold is an isosceles triangle. The height is just defines the height of the triangle.
            The length is the length of the edge where the robot can hold.

        :param id: Hold ID Number
        :param position: Hold position as (x, y) on the map in meters
        :param length: Hold length in meters
        :param height: Hold height in meters
        :param slope: Hold slope
        :param coeff_fric: Coefficient of Static Friction
        """
        # Robot ID Number
        self.id = id

        # Robot Position in x and y
        self.x = position[0]
        self.y = position[1]

        # Length of the edge where the robot can hold
        self.l = length

        # Height of the triangle
        self.h = height

        # Slope angle
        self.th = np.arctan(slope)

        # Half Friction Cone Angle using coefficient of friction
        self.gamma = np.arctan(coeff_fric)

        # Angle of Normal of the surface
        if slope == 0.:
            self.n_ang = np.pi / 2
        else:
            self.n_ang = np.arctan(-1. / slope)

        # Normalizing the normal angle
        if self.n_ang < 0:
            self.n_ang += np.pi

        # Maximum and Minimum unit vectors of limits of the friction cone
        self.fr_cone_min = np.array([np.cos(self.n_ang - self.gamma), np.sin(self.n_ang - self.gamma)])
        self.fr_cone_max = np.array([np.cos(self.n_ang + self.gamma), np.sin(self.n_ang + self.gamma)])

    def get_hold_plot(self):
        """1
            This function creates a triangle which is a hold. It finds the three edges of the triangle and return
            as a line2D object
        """
        # Calculating the corners of the triangle
        corners = np.array([
            [self.x - self.l / 2 * np.cos(self.th), self.y - self.l / 2 * np.sin(self.th)],
            [self.x + self.l / 2 * np.cos(self.th), self.y + self.l / 2 * np.sin(self.th)],
            [self.x - self.h * np.cos(np.pi / 2 + self.th), self.y - self.h * np.sin(np.pi / 2 + self.th)],
            [self.x - self.l / 2 * np.cos(self.th), self.y - self.l / 2 * np.sin(self.th)]])

        return lines.Line2D(corners[:, 0], corners[:, 1], c='k', linewidth=2)

    def get_normal_plot(self):
        """
            This function creates normal vector and returns it as a Line2D object.
            The line length which will be plotted is assumed as 2 times of the height of the triangle.
        """
        # Assumed normal length
        normal_length = 5

        # Calculating the points
        points = np.array([[self.x, self.y],
                           [self.x + normal_length * self.h * np.cos(self.n_ang),
                            self.y + normal_length * self.h * np.sin(self.n_ang)]])

        return lines.Line2D(points[:, 0], points[:, 1], c='#ffa500')

    def get_cone_plot(self):
        """
            This function creates friction cone and returns it as a Line2D object.
            The line length which will be plotted is assumed as 10 times of the height of the triangle.
        """
        # Assumed Cone Length
        cone_length = 10

        # Calculating the points
        points = np.array([[self.x, self.y],
                           [self.x + cone_length * self.h * self.fr_cone_min[0],
                            self.y + cone_length * self.h * self.fr_cone_min[1]],
                           [self.x, self.y],
                           [self.x + cone_length * self.h * self.fr_cone_max[0],
                            self.y + cone_length * self.h * self.fr_cone_max[1]]])

        return lines.Line2D(points[:, 0], points[:, 1], ls=':', c='0.5')

    def get_index_on_plot(self):

        # Assumed Font Size
        fontsize = 12

        # Create Text Object
        t = self.x - self.l, self.y - self.l, str(self.id), fontsize

        return t