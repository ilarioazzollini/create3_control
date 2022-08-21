import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose

from rclpy.qos import qos_profile_sensor_data

from rclpy.action import ActionServer, CancelResponse, GoalResponse

from irobot_create_msgs.action import NavigateToPosition

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import time

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
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


class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        # Subscribe to odom
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup())
        self.subscription  # prevent unused variable warning
        self._goal_handle = None
        self._action_server = ActionServer(
            self,
            NavigateToPosition,
            'drive_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup())

        # Relative goal
        self.relative_goal = Vector3()
        self.relative_goal.x = 1.0
        self.relative_goal.y = 1.0
        self.relative_goal.z = 0.0

        self.gain_x = 1.0
        self.gain_y = 1.0

        self.front_bumper_position = Vector3()
        self.front_bumper_position.x = 0.175

        self.last_odom_msg = None

        # Publish velocities
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_profile_sensor_data)
        self.timer_period = 0.01  # seconds

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        # This server only allows one goal at a time
        if self._goal_handle is not None and self._goal_handle.is_active:
            self.get_logger().info('Aborting previous goal')
            # Abort the existing goal
            self._goal_handle.abort()
        self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        absolute_goal = None

        # Start executing the action
        done = False
        while not done:
            time.sleep(self.timer_period)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return NavigateToPosition.Result()

            if not self.last_odom_msg:
                continue

            front_bumper_absolute_pose = add_relative_to_absolute_pose(self.front_bumper_position, self.last_odom_msg.pose.pose)

            if not absolute_goal:
                absolute_goal = add_relative_to_absolute_pose(goal_handle.request.goal_pose.pose.position, front_bumper_absolute_pose)
                print(f"current pose: {self.last_odom_msg.pose.pose}")
                print(f"absolute goal: {absolute_goal}")

            e_x = front_bumper_absolute_pose.position.x - absolute_goal.position.x
            e_y = front_bumper_absolute_pose.position.y - absolute_goal.position.y
            e_z = front_bumper_absolute_pose.position.z - absolute_goal.position.z

            if abs(e_x) <= 0.01 and abs(e_y) <= 0.01 and abs(e_z) <= 0.01:
                done = True
                break

            print(f"e_x: {e_x}, e_y: {e_y}")

            _, _, yaw = euler_from_quaternion(self.last_odom_msg.pose.pose.orientation)

            # print(f"yaw: {yaw}")

            fb_x = - self.gain_x * e_x
            fb_y = - self.gain_y * e_y

            v = fb_x * np.cos(yaw) + fb_y * np.sin(yaw)
            omega = (1 / self.front_bumper_position.x) * ((-fb_x * np.sin(yaw)) + (fb_y * np.cos(yaw)))

            # print(f"v: {v}, omega:{omega}")

            msg = Twist()
            msg.linear.x = v # m/s
            msg.angular.z = omega # rad/s
            self.publisher_.publish(msg)

        goal_handle.succeed()
        self.get_logger().info('Goal reached.')

        # Populate result message
        result = NavigateToPosition.Result()
        result.pose.pose = self.last_odom_msg.pose.pose

        return result

    def listener_callback(self, msg):
        self.last_odom_msg = msg


def main(args=None):
    rclpy.init(args=args)

    controller_node = Controller()

    executor = MultiThreadedExecutor()
    rclpy.spin(controller_node, executor=executor)

    rclpy.shutdown()


if __name__ == '__main__':
    main()