#!/usr/bin/env python3

"""
Simple service-server and client example for ROS 2.
This demonstrates the request-response communication pattern.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceServer(Node):
    """
    Service server that adds two integers.
    """
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Sending back response: [{response.sum}]')
        return response


class ServiceClient(Node):
    """
    Service client that requests to add two integers.
    """
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future


def main(args=None):
    rclpy.init(args=args)

    # Run server example
    server = ServiceServer()

    # Create client and send request
    client = ServiceClient()

    # Send a request
    future = client.send_request(42, 3)

    # Wait for response
    rclpy.spin_until_future_complete(server, future)

    if future.result() is not None:
        response = future.result()
        client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    else:
        client.get_logger().info('Service call failed')

    # Clean up
    server.destroy_node()
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()