from create3_control.controllers.fb_linearization import FBLinearizationController

def construct_fb_linearization_controller(ros2_node):
    controller_name = 'fb_linearization'
    ros2_node.get_logger().info(f"Creating controller {controller_name}")

    # Declare controller-specific ROS 2 parameters
    gain_param_name = controller_name + '.gain'
    lenght_param_name = controller_name + '.length'
    ros2_node.declare_parameter(gain_param_name, 1.0)
    ros2_node.declare_parameter(lenght_param_name, 0.01)

    # Construct controller
    gain = ros2_node.get_parameter(gain_param_name).get_parameter_value().double_value
    length = ros2_node.get_parameter(lenght_param_name).get_parameter_value().double_value
    controller = FBLinearizationController(gain, length)

    return controller

controllers_map = {
    'fb_linearization' : lambda node: construct_fb_linearization_controller(node),
}

def construct(controller_type, node):
    return controllers_map[controller_type](node)
