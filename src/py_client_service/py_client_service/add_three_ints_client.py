import sys

from tutorial_interfaces.srv import AddThreeInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        # create node with name 'minimal_client_async'
        super().__init__('minimal_client_async')
        # create client with same service name and type
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')
        # check the network every second for a server node matching the name and type of the service of the client
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # create the request to send
        self.req = AddThreeInts.Request()

    # send request and wait for response
    def send_request(self, a, b, c):
        self.req.a = a
        self.req.b = b
        self.req.c = c
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    # create request from command line arguments, send request and store response
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]))
    minimal_client.get_logger().info(
        'Result of add_three_ints: for %d + %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
