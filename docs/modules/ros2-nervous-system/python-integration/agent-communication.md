# Python Agent Communication

## Introduction

In robotics, agents are autonomous entities that perceive their environment and take actions to achieve goals. In ROS 2, Python agents typically implement this by subscribing to sensor data topics, processing the information, and publishing commands or service requests.

## Agent Architecture

A typical Python agent in ROS 2 follows this pattern:

1. **Perception**: Subscribe to sensor topics and service responses
2. **Processing**: Analyze data and make decisions
3. **Action**: Publish commands or request services

## Example: Simple Sensor-Action Agent

Here's a basic agent that responds to sensor input:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleAgent(Node):
    def __init__(self):
        super().__init__('simple_agent')

        # Subscribe to sensor data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)

        # Publish movement commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Store sensor data
        self.latest_scan = None

    def laser_callback(self, msg):
        self.latest_scan = msg
        self.process_sensor_data()

    def process_sensor_data(self):
        if self.latest_scan is None:
            return

        # Simple obstacle avoidance logic
        min_distance = min(self.latest_scan.ranges)

        cmd = Twist()
        if min_distance > 1.0:  # No obstacle nearby
            cmd.linear.x = 0.5  # Move forward
            cmd.angular.z = 0.0
        else:  # Obstacle detected
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn right

        self.publisher.publish(cmd)
```

## Complex Agent with State Management

For more sophisticated agents, you might want to maintain state:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum

class AgentState(Enum):
    IDLE = 1
    MOVING_TO_GOAL = 2
    AVOIDING_OBSTACLE = 3
    COMPLETED_TASK = 4

class StatefulAgent(Node):
    def __init__(self):
        super().__init__('stateful_agent')
        self.state = AgentState.IDLE
        self.goal_reached = False

        # Setup publishers, subscribers, etc.

    def update_state(self):
        if self.state == AgentState.IDLE:
            self.start_new_task()
        elif self.state == AgentState.MOVING_TO_GOAL:
            if self.detect_obstacle():
                self.state = AgentState.AVOIDING_OBSTACLE
            elif self.reached_goal():
                self.state = AgentState.COMPLETED_TASK
        elif self.state == AgentState.AVOIDING_OBSTACLE:
            if not self.detect_obstacle():
                self.state = AgentState.MOVING_TO_GOAL
```

## Communication Patterns for Agents

### Event-Driven Communication
Agents respond to events from their environment:

```python
def sensor_callback(self, msg):
    # Process sensor event
    self.react_to_environment(msg)
```

### Goal-Driven Communication
Agents work toward specific objectives:

```python
def request_navigation_goal(self, x, y):
    # Request navigation to specific coordinates
    pass
```

### Collaborative Communication
Multiple agents coordinate with each other:

```python
def broadcast_agent_status(self):
    # Share agent state with other agents
    status_msg = AgentStatus()
    status_msg.id = self.agent_id
    status_msg.state = self.state
    self.status_publisher.publish(status_msg)
```

## Best Practices for Agent Communication

1. **Maintain clear state machines** to manage agent behavior
2. **Use appropriate QoS settings** for reliable communication
3. **Implement proper error handling** for sensor failures
4. **Design for modularity** to enable code reuse
5. **Consider timing constraints** for real-time behavior
6. **Implement graceful degradation** when sensors fail

## Integration with AI Libraries

Python agents can easily integrate with AI libraries:

```python
import tensorflow as tf
# or
import torch
# or
import sklearn

class AIEnabledAgent(Node):
    def __init__(self):
        super().__init__('ai_agent')
        # Load pre-trained model
        self.model = self.load_model()

    def process_with_ai(self, sensor_data):
        # Use AI model to process sensor data
        prediction = self.model.predict(sensor_data)
        return prediction
```

This enables sophisticated behavior like object recognition, path planning, and decision making in robotic agents.