from tutorial_interfaces.srv import AddThreeInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        # creates node with name 'minimal_service'
        super().__init__('minimal_service')
        # creates service, defining name, type and callback
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)

    def add_three_ints_callback(self, request, response):
        # take the integers from the request to sum and store the result in the response
        response.sum = request.a + request.b + request.c
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c))

        return response


def main():
    # initialize ROS python client library
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
