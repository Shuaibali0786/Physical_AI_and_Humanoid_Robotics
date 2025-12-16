# rclpy Basics

## Introduction to rclpy

`rclpy` is the Python client library for ROS 2. It provides the standard interface for Python programs to interact with ROS 2, allowing Python developers to create nodes, publish and subscribe to topics, provide and use services, and more.

## Setting Up rclpy

To use rclpy in your Python project, you need to install it as part of the ROS 2 distribution. If you have ROS 2 installed, rclpy should already be available.

```python
import rclpy
from rclpy.node import Node
```

## Creating a Node

The basic structure of a Python ROS 2 node is:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishers

To create a publisher in rclpy:

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

## Subscribers

To create a subscriber in rclpy:

```python
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Services

To create a service server:

```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

## Clients

To create a service client:

```python
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Parameters

Nodes can also handle parameters:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        self.declare_parameter('my_parameter', 'default_value')

    def get_parameter_value(self):
        param = self.get_parameter('my_parameter')
        return param.value
```

## Best Practices

1. **Always call rclpy.init()** before creating nodes
2. **Use proper node naming** to avoid conflicts
3. **Handle exceptions** in callbacks appropriately
4. **Use appropriate queue sizes** for publishers and subscribers
5. **Clean up resources** by destroying nodes properly
6. **Use logging** to help with debugging