import numpy as np
import math

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

from create3_control.controllers.controller_interface import ControllerInterface
import create3_control.utilities as utils

class RotateDriveRotateController(ControllerInterface):

    def __init__(self):
        self.convergence_radius = 0.01 # cm

        self.relative_goal_pose = None
        self.absolute_goal_pose = None

    def setup_goal(self, goal_pose, current_pose):
        self.relative_goal_pose = goal_pose

        self.absolute_goal_pose = utils.add_relative_to_absolute_pose(self.relative_goal_pose.position, current_pose)

        print(f"current pose: {current_pose}")
        print(f"absolute goal: {self.absolute_goal_pose}")


    def step_function(self, current_pose):

        assert self.absolute_goal_pose
        _, _, current_yaw = utils.euler_from_quaternion(current_pose.orientation)

        e_x = self.absolute_goal_pose.position.x - current_pose.position.x
        e_y = self.absolute_goal_pose.position.y - current_pose.position.y

        position_reached = abs(e_x) <= self.convergence_radius and abs(e_y) <= self.convergence_radius

        if not position_reached:

            drive_yaw = np.arctan2(e_y, e_x)
            diff_yaw = utils.angles_radians_difference(drive_yaw, current_yaw)
            diff_linear = math.sqrt(e_x * e_x + e_y * e_y)

            v = 0.0
            omega = diff_yaw
            if abs(diff_yaw) <= np.pi/3:
                v = diff_linear
        
        else:
            
            _, _, goal_yaw = utils.euler_from_quaternion(self.absolute_goal_pose.orientation)
            diff_yaw = utils.angles_radians_difference(goal_yaw, current_yaw)
            
            if abs(diff_yaw) < 0.05:
                return None
            
            v = 0.0
            omega = diff_yaw

        # Saturations
        if abs(v) >= 0.33:
            v = math.copysign(0.33,v)

        msg = Twist()
        msg.linear.x = v # m/s
        msg.angular.z = omega # rad/s
        
        return msg
