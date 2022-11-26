# Copyright 2022 Ilario Antonio Azzollini.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import numpy as np
from geometry_msgs.msg import Twist

from create3_control.controllers.controller_interface import ControllerInterface
import create3_control.utilities as utils


class PolarCoordinatesController(ControllerInterface):

    def __init__(self, k_r, k_g, k_d):
        self.convergence_radius = 0.01  # m
        self.k_r = k_r
        self.k_g = k_g
        self.k_d = k_d

        self.absolute_goal_pose = None
        self.oriented_towards_goal = False
        self.validated_algorithm = False
        self.do_rotate_write_rotate = False

    def setup_goal(self, goal_pose, current_pose):
        self.absolute_goal_pose = \
            utils.add_relative_to_absolute_pose(goal_pose.position, current_pose)
        self.oriented_towards_goal = False
        self.validated_algorithm = False
        self.do_rotate_write_rotate = False

        print(f"current pose: {current_pose}")
        print(f"absolute goal: {self.absolute_goal_pose}")

    def step_function(self, current_pose):
        assert self.absolute_goal_pose

        _, _, current_yaw = utils.euler_from_quaternion(current_pose.orientation)
        _, _, goal_yaw = utils.euler_from_quaternion(self.absolute_goal_pose.orientation)

        e_x = self.absolute_goal_pose.position.x - current_pose.position.x
        e_y = self.absolute_goal_pose.position.y - current_pose.position.y
        e_yaw = utils.angles_radians_difference(goal_yaw, current_yaw)

        x_reached = abs(e_x) <= self.convergence_radius
        y_reached = abs(e_y) <= self.convergence_radius
        position_reached = x_reached and y_reached
        orientation_reached = abs(e_yaw) <= 0.05

        if position_reached and orientation_reached:
            return None

        rho = math.sqrt(e_x * e_x + e_y * e_y)
        error_direction = np.arctan2(e_y, e_x)
        gamma = utils.angles_radians_difference(error_direction, current_yaw)
        delta = gamma + current_yaw

        if not self.oriented_towards_goal and abs(gamma) > np.pi / 3.0:
            # Orient towards the goal
            v, omega = self.turn_in_place(self.k_g, gamma)
        else:
            if not self.validated_algorithm:
                heading_error = utils.angles_radians_difference(error_direction, goal_yaw)
                is_facing_goal_orientation = heading_error < (np.pi / 3.0)
                if not is_facing_goal_orientation:
                    self.do_rotate_write_rotate = True
                self.oriented_towards_goal = True
                self.validated_algorithm = True

            if self.do_rotate_write_rotate:
                if not position_reached:
                    v, omega = self.pose_regulation(rho, gamma, 0.0)
                else:
                    v, omega = self.turn_in_place(self.k_d, e_yaw)
            else:
                v, omega = self.pose_regulation(rho, gamma, delta)

        msg = Twist()
        msg.linear.x = v  # m/s
        msg.angular.z = omega  # rad/s

        return msg

    def turn_in_place(self, gain, orientation_error):
        v = 0.0
        omega = gain * orientation_error
        return v, omega

    def pose_regulation(self, rho, gamma, delta):
        v = self.k_r * rho
        omega = self.k_g * gamma + self.k_d * delta
        return v, omega
        '''
        sing = np.sin(gamma)
        cosg = np.cos(gamma)
        v = self.k_r * rho * cosg
        omega = \
            (self.k_g * gamma) + self.k_r * sing * cosg * (gamma + self.k_d * delta) / gamma
        return v, omega
        '''
