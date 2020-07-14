#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Libraries
import numpy as np
import matplotlib.patches as patches
import matplotlib.lines as lines

# Own Modules
from RobotData import RobotData

__author__ = "Emre Cemal Gonen"
__copyright__ = "Copyright (C) 2019, Emre Cemal Gonen"
__credits__ = ["Prof. Dr. Volkan Patoglu", "Sabanci University"]
__version__ = "2.0"
__email__ = "emrecemal@sabanciuniv.edu"


class Robot:
    def __init__(self, pelvis_position, pelvis_orientation):

        # Get Robot Hard Data from its class
        self.r = RobotData.r
        self.l1 = RobotData.l1
        self.l2 = RobotData.l2
        self.m = RobotData.m
        self.m_motor = RobotData.m_motor
        self.attach_ang = RobotData.attach_ang

        # Assign Inputs
        self.x = pelvis_position[0]
        self.y = pelvis_position[1]
        self.ori = pelvis_orientation

        # Create arrays for all joint positions in a limb
        # This will be calculated if direct kinematics method is called
        self.rf = None
        self.lf = None
        self.lr = None
        self.rr = None

        # Create Joint Angles Array, which will be filled after IK or DK solutions
        self.q = None

    def _calc_end_effector(self, start_point, length, angle, leg_number, limb_parent_angle):
        """
            This function calculates the position and the orientation of the end-effector.

        :param start_point: Initial Point of a Vector, numpy array 2 x 1
        :param length: Norm of the vector in meters
        :param angle: Angle of the vector in radians
        :param leg_number: Leg Index in CCW direction
        :param limb_parent_angle: Vector Parent Angles
        :return: position and orientation numpy array
        """
        x = start_point[0] + length * np.cos(self.ori + self.attach_ang[leg_number] + limb_parent_angle + angle)
        y = start_point[1] + length * np.sin(self.ori + self.attach_ang[leg_number] + limb_parent_angle + angle)
        th = self.ori + self.attach_ang[leg_number] + limb_parent_angle + angle
        return np.array([x, y, th])

    def get_dk(self, q):
        """
            This function calculates direct kinematic of the robot.
            It takes joint angles as input and transforms to end effector positions
        :param q: joint angles numpy array (8 x 1)
        :return: end-effector positions numpy array
        """

        # Assign joint values to current joint
        self.q = q

        # Right Front
        # Calculate Each Vector
        leg_num = 0
        att_rf = self._calc_end_effector((self.x, self.y), self.r, 0., leg_num, 0.)
        limb_rf1 = self._calc_end_effector(att_rf, self.l1, q[0], leg_num, 0.)
        limb_rf2 = self._calc_end_effector(limb_rf1, self.l2, q[1], leg_num, q[0])

        # Left Front
        # Calculate Each Vector
        leg_num = 1
        att_lf = self._calc_end_effector((self.x, self.y), self.r, 0., leg_num, 0.)
        limb_lf1 = self._calc_end_effector(att_lf, self.l1, q[2], leg_num, 0.)
        limb_lf2 = self._calc_end_effector(limb_lf1, self.l2, q[3], leg_num, q[2])

        # Left Rear
        # Calculate Each Vector
        leg_num = 2
        att_lr = self._calc_end_effector((self.x, self.y), self.r, 0., leg_num, 0.)
        limb_lr1 = self._calc_end_effector(att_lr, self.l1, q[4], leg_num, 0.)
        limb_lr2 = self._calc_end_effector(limb_lr1, self.l2, q[5], leg_num, q[4])

        # Right Rear
        # Calculate Each Vector
        leg_num = 3
        att_rr = self._calc_end_effector((self.x, self.y), self.r, 0., leg_num, 0.)
        limb_rr1 = self._calc_end_effector(att_rr, self.l1, q[6], leg_num, 0.)
        limb_rr2 = self._calc_end_effector(limb_rr1, self.l2, q[7], leg_num, q[6])

        self.rf = np.array([att_rf, limb_rf1, limb_rf2])
        self.lf = np.array([att_lf, limb_lf1, limb_lf2])
        self.lr = np.array([att_lr, limb_lr1, limb_lr2])
        self.rr = np.array([att_rr, limb_rr1, limb_rr2])

        return np.array([limb_rf2, limb_lf2, limb_lr2, limb_rr2])

    def _calc_ik(self, leg_number, desired_point):
        """
            This function calculates inverse kinematics for a robot leg.
            First, it finds attachment point, then it assigns to base vector
            After, it creates a vector between desired point and base point
            Then, it calculates IK for 2R robot arm

        :param leg_number: Index Number of a robot leg
        :param desired_point: Desired point for a leg, numpy array 2 x 1
        :return: Although two solutions are calculated, just return one solution
        """
        # Create Base Vector, just need position information
        base_vector = self._calc_end_effector((self.x, self.y), self.r, 0., leg_number, 0.)[:2]

        # Create Desired Vector
        desired_vector = desired_point - base_vector

        # Find Cosine of Second Angle From Cosine Theorem
        cos_q2 = (desired_vector[0] ** 2 + desired_vector[1] ** 2 - self.l1 ** 2 - self.l2 ** 2) / 2 / self.l1 / self.l2

        # Calculate Two Different Angles for the second angle
        q2_1 = np.arctan2(-np.sqrt(1 - cos_q2 ** 2), cos_q2)
        q2_2 = np.arctan2(np.sqrt(1 - cos_q2 ** 2), cos_q2)

        # Using the above equation, calculate the first angle
        q1_1 = np.arctan2(desired_vector[1], desired_vector[0]) - \
               np.arctan2(self.l2 * np.sin(q2_1), self.l1 + self.l2 * np.cos(q2_1)) - \
               self.attach_ang[leg_number] - self.ori
        q1_2 = np.arctan2(desired_vector[1], desired_vector[0]) - \
               np.arctan2(self.l2 * np.sin(q2_2), self.l1 + self.l2 * np.cos(q2_2)) - \
               self.attach_ang[leg_number] - self.ori

        # Return just first solution
        # NOTE: IF THE OTHER SOLUTION IS NEEDED, MODIFY THIS PART!
        return np.array([q1_1, q2_1])

    def get_ik(self, x):
        """
            This function calculates inverse kinematics for all limbs
        :param x: desired positions for end-effectors, numpy array 4 x 2
        :return: joint angle arrays, numpy array 8 x 1
        """
        # Create an empty array to fill
        q = np.array([])

        # In a loop
        for l in range(len(x)):
            # Calculate End-Effector for each limb
            limb = self._calc_ik(l, np.array([x[l].x, x[l].y]))

            # Append
            q = np.append(q, limb)

        # Assign Joint Values to current values
        self.q = q
        self.get_dk(q)

        return q

    def get_com(self, cd=0.2):
        """
            This function calculates Center of Mass (CoM) location of the whole robot.
            To find it, it takes each CoM location on limb as 20% far from the motor location
        :param cd: CoM location percentage from motor
        :return: CoM Position, numpy array 2 x 1
        """
        # Vector Normalization
        rf1 = (self.rf[1][:2] - self.rf[0][:2]) / np.linalg.norm((self.rf[1][:2] - self.rf[0][:2]))
        rf2 = (self.rf[2][:2] - self.rf[1][:2]) / np.linalg.norm((self.rf[2][:2] - self.rf[1][:2]))

        lf1 = (self.lf[1][:2] - self.lf[0][:2]) / np.linalg.norm((self.lf[1][:2] - self.lf[0][:2]))
        lf2 = (self.lf[2][:2] - self.lf[1][:2]) / np.linalg.norm((self.lf[2][:2] - self.lf[1][:2]))

        lr1 = (self.lr[1][:2] - self.lr[0][:2]) / np.linalg.norm((self.lr[1][:2] - self.lr[0][:2]))
        lr2 = (self.lr[2][:2] - self.lr[1][:2]) / np.linalg.norm((self.lr[2][:2] - self.lr[1][:2]))

        rr1 = (self.rr[1][:2] - self.rr[0][:2]) / np.linalg.norm((self.rr[1][:2] - self.rr[0][:2]))
        rr2 = (self.rr[2][:2] - self.rr[1][:2]) / np.linalg.norm((self.rr[2][:2] - self.rr[1][:2]))

        pelvis_mass = self.m - 8 * self.m_motor

        # Calculating CoM
        x_com = (self.x * pelvis_mass +
                 (self.rf[0][:2] + cd * rf1)[0] * self.m_motor +
                 (self.rf[1][:2] + cd * rf2)[0] * self.m_motor +
                 (self.lf[0][:2] + cd * lf1)[0] * self.m_motor +
                 (self.lf[1][:2] + cd * lf2)[0] * self.m_motor +
                 (self.lr[0][:2] + cd * lr1)[0] * self.m_motor +
                 (self.lr[1][:2] + cd * lr2)[0] * self.m_motor +
                 (self.rr[0][:2] + cd * rr1)[0] * self.m_motor +
                 (self.rr[1][:2] + cd * rr2)[0] * self.m_motor) / self.m

        y_com = (self.y * pelvis_mass +
                 (self.rf[0][:2] + cd * rf1)[1] * self.m_motor +
                 (self.rf[1][:2] + cd * rf2)[1] * self.m_motor +
                 (self.lf[0][:2] + cd * lf1)[1] * self.m_motor +
                 (self.lf[1][:2] + cd * lf2)[1] * self.m_motor +
                 (self.lr[0][:2] + cd * lr1)[1] * self.m_motor +
                 (self.lr[1][:2] + cd * lr2)[1] * self.m_motor +
                 (self.rr[0][:2] + cd * rr1)[1] * self.m_motor +
                 (self.rr[1][:2] + cd * rr2)[1] * self.m_motor) / self.m

        return np.array([x_com, y_com])

    def get_sketch_plot(self):
        pelvis = patches.RegularPolygon((self.x, self.y), 6, radius=self.r, orientation=self.ori,
                                        ec=(1, 0, 0, 0.5), fc=(0.9412, 0.4588, 0.1725, 0.5))
        head = lines.Line2D([self.x + self.r * np.cos(np.pi / 2 + self.ori)],
                            [self.y + self.r * np.sin(np.pi / 2 + self.ori)], marker='o', ms=10, mec='#00008b', c='y')

        self.get_dk(self.q)

        return pelvis, head, [lines.Line2D(self.rf[0:2, 0], self.rf[0:2, 1], zorder=2, c='g', lw=1.5, marker='o'),
                              lines.Line2D(self.rf[1:3, 0], self.rf[1:3, 1], zorder=2, c='g', lw=1.5, marker='o'),
                              lines.Line2D(self.lf[0:2, 0], self.lf[0:2, 1], zorder=2, c='r', lw=1.5, marker='o'),
                              lines.Line2D(self.lf[1:3, 0], self.lf[1:3, 1], zorder=2, c='r', lw=1.5, marker='o'),
                              lines.Line2D(self.lr[0:2, 0], self.lr[0:2, 1], zorder=2, c='r', lw=1.5, marker='o'),
                              lines.Line2D(self.lr[1:3, 0], self.lr[1:3, 1], zorder=2, c='r', lw=1.5, marker='o'),
                              lines.Line2D(self.rr[0:2, 0], self.rr[0:2, 1], zorder=2, c='g', lw=1.5, marker='o'),
                              lines.Line2D(self.rr[1:3, 0], self.rr[1:3, 1], zorder=2, c='g', lw=1.5, marker='o')]

    def is_contacted(self, leg_number, hold, dt=1e-8):
        """
            This functions calculates if the desired length is in contact or not.
            It calculates both vertical and horizontal distances and check if they are in tolerance range or not.
        :param leg_number: Leg Index number between 0-4
        :param hold: hold object to check if there is a contact with the leg
        :param dt: tolerance value to check the contact
        :return: True or False depends on existence of contact
        """
        # Get desired leg locations
        if leg_number == 0:
            leg = self.rf[2]
        elif leg_number == 1:
            leg = self.lf[2]
        elif leg_number == 2:
            leg = self.lr[2]
        elif leg_number == 3:
            leg = self.rr[2]
        else:
            raise IndexError('Enter a valid robot leg number between 0-4')

        # Calculate difference
        dx = np.abs(hold.x - leg[0])
        dy = np.abs(hold.y - leg[1])

        # Check the differences in range or not
        if dx <= dt and dy <= dt:
            return True
        else:
            return False
