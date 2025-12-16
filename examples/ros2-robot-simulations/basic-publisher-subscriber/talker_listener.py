#!/usr/bin/env python3

"""
Simple publisher-subscriber example for ROS 2.
This demonstrates the basic concepts of nodes, topics, and message passing.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    """
    Publisher node that sends messages to a topic.
    """
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


class ListenerNode(Node):
    """
    Subscriber node that receives messages from a topic.
    """
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    talker = TalkerNode()
    listener = ListenerNode()

    try:
        # Run both nodes
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        talker.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()