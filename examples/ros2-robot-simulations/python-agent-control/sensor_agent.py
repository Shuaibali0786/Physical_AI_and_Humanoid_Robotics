#!/usr/bin/env python3

"""
Advanced Python agent with sensor integration.
This agent processes multiple sensor inputs and makes more complex decisions.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class SensorAgent(Node):
    """
    An agent that processes laser scan data to navigate and avoid obstacles.
    """
    def __init__(self):
        super().__init__('sensor_agent')

        # Subscribe to laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        # Publish movement commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publish status messages
        self.status_publisher = self.create_publisher(String, 'agent_status', 10)

        # Store latest scan
        self.latest_scan = None

        # Create timer for processing
        self.timer = self.create_timer(0.1, self.process_scan_data)

        # Agent state
        self.state = "SEARCHING"  # SEARCHING, AVOIDING, STOPPED

    def scan_callback(self, msg):
        """
        Callback for laser scan data.
        """
        self.latest_scan = msg
        self.get_logger().info(f'Received scan with {len(msg.ranges)} readings')

    def process_scan_data(self):
        """
        Process laser scan data and make navigation decisions.
        """
        if self.latest_scan is None:
            return

        # Find minimum distance in front of the robot (forward 90 degrees range)
        ranges = self.latest_scan.ranges
        num_readings = len(ranges)

        # Define forward sector (front 90 degrees)
        start_idx = num_readings // 2 - num_readings // 8  # 315 degrees
        end_idx = num_readings // 2 + num_readings // 8    # 45 degrees

        # Handle circular indexing
        if start_idx < 0:
            start_idx = 0
        if end_idx >= num_readings:
            end_idx = num_readings - 1

        # Get forward distances
        forward_distances = [ranges[i] for i in range(start_idx, end_idx) if not math.isinf(ranges[i]) and not math.isnan(ranges[i])]

        if not forward_distances:
            # If no valid readings, stop
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_publisher.publish(cmd)
            self.state = "STOPPED"
        else:
            min_distance = min(forward_distances)

            cmd = Twist()

            if min_distance < 0.5:  # Obstacle too close
                # Turn to avoid obstacle
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5
                self.state = "AVOIDING"
            elif min_distance < 1.0:  # Obstacle at medium distance
                # Slow down and turn slightly
                cmd.linear.x = 0.2
                cmd.angular.z = 0.2
                self.state = "AVOIDING"
            else:  # Clear path
                # Move forward
                cmd.linear.x = 0.5
                cmd.angular.z = 0.0
                self.state = "SEARCHING"

            # Publish command
            self.cmd_publisher.publish(cmd)

        # Publish status
        status_msg = String()
        status_msg.data = f"State: {self.state}, Min dist: {min_distance if 'min_distance' in locals() else 'N/A'}"
        self.status_publisher.publish(status_msg)

        self.get_logger().info(f'State: {self.state}, Command: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}')


def main(args=None):
    rclpy.init(args=args)

    agent = SensorAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()