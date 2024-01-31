# import package dependencies
import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor # OPTIONAL

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        # OPTIONAL parameter description
        my_parameter_desc = ParameterDescriptor(description="Example parameter.")

        # declare parameter with default value, infer type
        self.declare_parameter('my_parameter', 'world', my_parameter_desc)
        # execute callback every second
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        # store node parameter value
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info('Hello %s!' % my_param)

        # reset parameter to the default value
        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)


def main():
    rclpy.init()
    node = MinimalParam()
    # start processing data from node
    rclpy.spin(node)

if __name__ == "__main__":
    main()