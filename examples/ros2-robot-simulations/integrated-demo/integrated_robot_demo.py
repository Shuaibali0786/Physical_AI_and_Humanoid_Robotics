#!/usr/bin/env python3

"""
Comprehensive integration example combining all concepts:
- ROS 2 nodes, topics, services
- Python agents
- URDF robot model
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from example_interfaces.srv import AddTwoInts
import math


class IntegratedRobotNode(Node):
    """
    A comprehensive robot node integrating multiple ROS 2 concepts.
    """
    def __init__(self):
        super().__init__('integrated_robot')

        # Publishers
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)
        self.sensor_publisher = self.create_publisher(Float32, 'simulated_sensor', 10)

        # Subscribers
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        # Service server
        self.service = self.create_service(AddTwoInts, 'robot_calculation', self.calculation_callback)

        # Timer for publishing simulated sensor data
        self.sensor_timer = self.create_timer(0.5, self.publish_sensor_data)

        # Timer for robot behavior
        self.behavior_timer = self.create_timer(0.1, self.robot_behavior)

        # Robot state
        self.obstacle_distance = float('inf')
        self.sensor_value = 0.0
        self.sim_time = 0.0

        self.get_logger().info("Integrated Robot Node initialized")

    def scan_callback(self, msg):
        """
        Process laser scan data to detect obstacles.
        """
        if len(msg.ranges) > 0:
            # Get the front-facing range (middle of the scan)
            front_idx = len(msg.ranges) // 2
            self.obstacle_distance = msg.ranges[front_idx] if not math.isinf(msg.ranges[front_idx]) else float('inf')

    def calculation_callback(self, request, response):
        """
        Service callback to perform calculations for the robot.
        """
        result = request.a + request.b
        response.sum = result
        self.get_logger().info(f"Calculation service: {request.a} + {request.b} = {result}")
        return response

    def publish_sensor_data(self):
        """
        Publish simulated sensor data based on time.
        """
        # Simulate a sensor value that oscillates
        self.sensor_value = 5.0 + 3.0 * math.sin(self.sim_time)
        self.sim_time += 0.1

        msg = Float32()
        msg.data = self.sensor_value
        self.sensor_publisher.publish(msg)

    def robot_behavior(self):
        """
        Main robot behavior logic combining all concepts.
        """
        cmd = Twist()

        # Navigation behavior based on obstacle detection
        if self.obstacle_distance < 0.8:
            # Obstacle detected - turn
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            status = f"OBSTACLE_AVOIDANCE: Distance={self.obstacle_distance:.2f}"
        else:
            # Clear path - move forward
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            status = f"FORWARD_MOVEMENT: Distance={self.obstacle_distance:.2f}"

        # Also respond to internal sensor value
        if self.sensor_value > 7.0:
            # If sensor value is high, turn more
            cmd.angular.z += 0.3

        # Publish command and status
        self.cmd_publisher.publish(cmd)

        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)

        self.get_logger().info(f"Command: linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f} | Status: {status}")


def main(args=None):
    rclpy.init(args=args)

    robot_node = IntegratedRobotNode()

    try:
        # Call the service to demonstrate service usage
        client = robot_node.create_client(AddTwoInts, 'robot_calculation')

        # Wait for service to be available
        while not client.wait_for_service(timeout_sec=1.0):
            robot_node.get_logger().info('Service not available, waiting again...')

        # Create a request
        request = AddTwoInts.Request()
        request.a = 10
        request.b = 20

        # Call the service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(robot_node, future)

        if future.result() is not None:
            robot_node.get_logger().info(f'Service result: {future.result().sum}')
        else:
            robot_node.get_logger().info('Service call failed')

        # Run the main robot behavior
        rclpy.spin(robot_node)

    except KeyboardInterrupt:
        pass
    finally:
        robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()