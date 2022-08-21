import numpy as np

from geometry_msgs.msg import Pose

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
