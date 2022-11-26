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

import numpy as np

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

from create3_control.controllers.controller_interface import ControllerInterface
import create3_control.utilities as utils


class FBLinearizationController(ControllerInterface):

    def __init__(self, gain, length):
        self.gain = gain
        self.convergence_radius = 0.01  # m

        self.front_bumper_position = Vector3()
        self.front_bumper_position.x = length

        self.relative_goal_pose = None
        self.absolute_goal_pose = None

    def setup_goal(self, goal_pose, current_pose):
        self.relative_goal_pose = goal_pose
        relative_goal_position = self.relative_goal_pose.position

        self.absolute_goal_pose = \
            utils.add_relative_to_absolute_pose(relative_goal_position, current_pose)

        print(f'current pose: {current_pose}')
        print(f'absolute goal: {self.absolute_goal_pose}')

    def step_function(self, current_pose):
        front_bumper_abs_pose = \
            utils.add_relative_to_absolute_pose(self.front_bumper_position, current_pose)

        assert self.absolute_goal_pose

        e_x = front_bumper_abs_pose.position.x - self.absolute_goal_pose.position.x
        e_y = front_bumper_abs_pose.position.y - self.absolute_goal_pose.position.y

        if abs(e_x) <= self.convergence_radius and abs(e_y) <= self.convergence_radius:
            return None

        # print(f"e_x: {e_x}, e_y: {e_y}")

        _, _, yaw = utils.euler_from_quaternion(current_pose.orientation)

        # print(f"yaw: {yaw}")

        fb_x = - self.gain * e_x
        fb_y = - self.gain * e_y
        length_const = self.front_bumper_position.x

        v = fb_x * np.cos(yaw) + fb_y * np.sin(yaw)
        omega = (1 / length_const) * ((-fb_x * np.sin(yaw)) + (fb_y * np.cos(yaw)))

        # print(f"v: {v}, omega:{omega}")

        msg = Twist()
        msg.linear.x = v  # m/s
        msg.angular.z = omega  # rad/s

        return msg
