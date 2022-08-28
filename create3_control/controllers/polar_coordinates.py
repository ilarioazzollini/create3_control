import math
import numpy as np

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

from create3_control.controllers.controller_interface import ControllerInterface
import create3_control.utilities as utils

class PolarCoordinatesController(ControllerInterface):

    def __init__(self, k_r, k_g, k_d, rotate_to_goal):
        self.convergence_radius = 0.01 # cm
        self.k_r = k_r
        self.k_g = k_g
        self.k_d = k_d
        self.rotate_to_goal = rotate_to_goal

        self.absolute_goal_pose = None
        self.oriented_towards_goal = False

    def setup_goal(self, goal_pose, current_pose):
        self.absolute_goal_pose = utils.add_relative_to_absolute_pose(goal_pose.position, current_pose)
        self.oriented_towards_goal = False

        print(f"current pose: {current_pose}")
        print(f"absolute goal: {self.absolute_goal_pose}")

    def step_function(self, current_pose):
        assert self.absolute_goal_pose

        _, _, current_yaw = utils.euler_from_quaternion(current_pose.orientation)
        _, _, goal_yaw = utils.euler_from_quaternion(self.absolute_goal_pose.orientation)

        e_x = self.absolute_goal_pose.position.x - current_pose.position.x
        e_y = self.absolute_goal_pose.position.y - current_pose.position.y
        
        e_yaw = utils.angles_radians_difference(goal_yaw, current_yaw)

        position_reached = abs(e_x) <= self.convergence_radius and abs(e_y) <= self.convergence_radius
        orientation_reached = abs(e_yaw) <= 0.05

        if position_reached and orientation_reached:
            return None

        error_direction = np.arctan2(e_y, e_x)
        gamma = utils.angles_radians_difference(error_direction, current_yaw)

        if abs(gamma) > np.pi / 3.0 and not self.oriented_towards_goal:
            v = 0.0
            omega = self.k_g * gamma
        else:
            self.oriented_towards_goal = True
            rho = math.sqrt(e_x * e_x + e_y * e_y)
            delta = gamma + current_yaw

            v = self.k_r * rho
            # If rotate_to_goal: use e_yaw if position reached, gamma otherwise
            # If only arc motions: use gamma and delta
            if self.rotate_to_goal:
                if position_reached:
                    omega = self.k_d * e_yaw
                else:
                    omega = self.k_g * gamma
            else:
                omega = self.k_g * gamma + self.k_d * delta

        # Saturations
        if abs(v) >= 0.33:
            v = math.copysign(0.33, v)
        if position_reached:
            v = 0.0

        msg = Twist()
        msg.linear.x = v # m/s
        msg.angular.z = omega # rad/s
        
        return msg
