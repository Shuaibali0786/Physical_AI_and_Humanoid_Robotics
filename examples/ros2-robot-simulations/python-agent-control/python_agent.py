#!/usr/bin/env python3

"""
Simple Python agent that demonstrates agent behavior in ROS 2.
This agent listens to a sensor topic and responds by publishing commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist


class SimpleAgent(Node):
    """
    A simple agent that responds to sensor input.
    """
    def __init__(self):
        super().__init__('simple_agent')

        # Subscribe to sensor data (simulated sensor readings)
        self.sensor_subscription = self.create_subscription(
            Float32,
            'sensor_input',
            self.sensor_callback,
            10)

        # Publish movement commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Store latest sensor reading
        self.latest_sensor_reading = 0.0

        # Create a timer to periodically process sensor data
        self.timer = self.create_timer(0.1, self.process_sensor_data)  # 10 Hz

    def sensor_callback(self, msg):
        """
        Callback function for sensor data.
        """
        self.latest_sensor_reading = msg.data
        self.get_logger().info(f'Sensor reading: {msg.data}')

    def process_sensor_data(self):
        """
        Process the latest sensor data and publish appropriate commands.
        """
        cmd = Twist()

        # Simple behavior: if sensor reading is above threshold, move forward
        # otherwise turn in place
        if self.latest_sensor_reading > 5.0:
            cmd.linear.x = 0.5  # Move forward
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn right

        self.cmd_publisher.publish(cmd)
        self.get_logger().info(f'Published command: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}')


def main(args=None):
    rclpy.init(args=args)

    agent = SimpleAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()