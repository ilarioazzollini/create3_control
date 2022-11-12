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
from geometry_msgs.msg import Pose, Quaternion


def euler_from_quaternion(quaternion):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.

    quaternion = [x, y, z, w]
    Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def add_relative_to_absolute_pose(relative_position, absolute_origin_pose):
    _, _, yaw = euler_from_quaternion(absolute_origin_pose.orientation)

    # Apply rotation matrix
    delta_x = np.cos(yaw) * relative_position.x - np.sin(yaw) * relative_position.y
    delta_y = np.sin(yaw) * relative_position.x + np.cos(yaw) * relative_position.y

    # absolute pose
    absolute_pose = Pose()
    absolute_pose.position.x = absolute_origin_pose.position.x + delta_x
    absolute_pose.position.y = absolute_origin_pose.position.y + delta_y
    absolute_pose.position.z = absolute_origin_pose.position.z
    absolute_pose.orientation = absolute_origin_pose.orientation

    return absolute_pose


def unit_quaternions_difference(q1, q2):

    diff_quaternion = Quaternion()

    diff_quaternion.w = q2.w*-q1.w - q2.x*q1.x - q2.y*q1.y - q2.z*q1.z
    diff_quaternion.x = q2.w*q1.x + q2.x*-q1.w + q2.y*q1.z - q2.z*q1.y
    diff_quaternion.y = q2.w*q1.y + q2.y*-q1.w + q2.z*q1.x - q2.x*q1.z
    diff_quaternion.z = q2.w*q1.z + q2.z*-q1.w + q2.x*q1.y - q2.y*q1.x

    return diff_quaternion


def angles_radians_difference(angle1, angle2):
    # diff_angle = angle1 - angle2
    # angle1 and angle2 in [-pi, pi]

    diff_angle = (angle1 - angle2 + np.pi) % (2 * np.pi) - np.pi

    return diff_angle
