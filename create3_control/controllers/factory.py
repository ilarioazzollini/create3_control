from create3_control.controllers.fb_linearization import FBLinearizationController
from create3_control.controllers.polar_coordinates import PolarCoordinatesController

def declare_parameter_if_not_declared(node, parameter_name, default_value):
    if not node.has_parameter(parameter_name):
        node.declare_parameter(parameter_name, default_value)

def construct_fb_linearization_controller(ros2_node):
    controller_name = 'fb_linearization'
    ros2_node.get_logger().info(f"Creating controller {controller_name}")

    # Declare controller-specific ROS 2 parameters
    gain_param_name = controller_name + '.gain'
    lenght_param_name = controller_name + '.length'
    declare_parameter_if_not_declared(ros2_node, gain_param_name, 1.0)
    declare_parameter_if_not_declared(ros2_node, lenght_param_name, 0.01)

    # Construct controller
    gain = ros2_node.get_parameter(gain_param_name).get_parameter_value().double_value
    length = ros2_node.get_parameter(lenght_param_name).get_parameter_value().double_value
    controller = FBLinearizationController(gain, length)

    return controller

def construct_polar_coordinates_controller(ros2_node):
    controller_name = 'polar_coordinates'
    ros2_node.get_logger().info(f"Creating controller {controller_name}")

    # Declare controller-specific ROS 2 parameters
    k_r_param_name = controller_name + '.k_r'
    k_g_param_name = controller_name + '.k_g'
    k_d_param_name = controller_name + '.k_d'
    declare_parameter_if_not_declared(ros2_node, k_r_param_name, 1.0)
    declare_parameter_if_not_declared(ros2_node, k_g_param_name, 8.0)
    declare_parameter_if_not_declared(ros2_node, k_d_param_name, 3.5)

    # Construct controller
    k_r = ros2_node.get_parameter(k_r_param_name).get_parameter_value().double_value
    k_g = ros2_node.get_parameter(k_g_param_name).get_parameter_value().double_value
    k_d = ros2_node.get_parameter(k_d_param_name).get_parameter_value().double_value
    controller = PolarCoordinatesController(k_r, k_g, k_d)

    return controller

controllers_map = {
    'fb_linearization' : lambda node: construct_fb_linearization_controller(node),
    'polar_coordinates' : lambda node: construct_polar_coordinates_controller(node),
}

def construct(controller_type, node):
    return controllers_map[controller_type](node)
