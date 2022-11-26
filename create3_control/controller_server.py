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

import time
from threading import Lock

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import NavigateToPosition
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionServer, CancelResponse, GoalResponse

import create3_control.controllers.factory as controllers_factory


class ControllerServer(Node):

    def __init__(self):
        super().__init__('controller')

        # Setup ROS 2 parameters
        self.declare_parameter('pose_topic', 'odom')
        self.declare_parameter('controller_type', 'polar_coordinates')
        self.declare_parameter('control_period', 0.015)

        # Create mutex
        self.odom_lock = Lock()

        # Subscribe to robot pose topic
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.odom_subscription = self.create_subscription(
            Odometry,
            pose_topic,
            self.odom_callback,
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup())

        # Create action server
        self.action_server = ActionServer(
            self,
            NavigateToPosition,
            'drive_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup())

        # Create velocity publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_profile_sensor_data)

        # Setup member variables
        self.control_period = \
            self.get_parameter('control_period').get_parameter_value().double_value
        self.last_odom_msg = None
        self.goal_handle = None
        self.controller = None

        # prevent unused variable warnings
        self.odom_subscription
        self.action_server

        self.get_logger().info('Node created, ready to receive action goals')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        # This server only allows one goal at a time
        if self.goal_handle is not None and self.goal_handle.is_active:
            self.get_logger().info('Aborting previous goal')
            # Abort the existing goal
            self.goal_handle.abort()
        self.goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        current_pose = Pose()
        while True:
            time.sleep(self.control_period)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return NavigateToPosition.Result()

            self.odom_lock.acquire()
            if not self.last_odom_msg:
                self.odom_lock.release()
                continue

            current_pose = self.last_odom_msg.pose.pose
            self.odom_lock.release()

            if not self.controller:
                controller_type = \
                    self.get_parameter('controller_type').get_parameter_value().string_value
                self.controller = controllers_factory.construct(controller_type, self)
                self.controller.setup_goal(goal_handle.request.goal_pose.pose, current_pose)

            cmd_msg = self.controller.step_function(current_pose)
            if not cmd_msg:
                break

            self.publisher_.publish(cmd_msg)

        goal_handle.succeed()
        self.get_logger().info('Goal reached.')

        # Cleanup
        self.goal_handle = None
        self.controller = None

        # Populate result message
        result = NavigateToPosition.Result()
        result.pose.pose = current_pose

        return result

    def odom_callback(self, msg):
        self.odom_lock.acquire()
        self.last_odom_msg = msg
        self.odom_lock.release()


def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(controller_node, executor=executor)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
