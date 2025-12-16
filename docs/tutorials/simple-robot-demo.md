---
sidebar_position: 1
---

# Simple Robot Demo: Integrating All Concepts

This tutorial demonstrates how to integrate all the concepts covered in the ROS 2 module: nodes, topics, services, Python agents, and URDF modeling.

## Overview

In this tutorial, we'll create a complete robot system that combines:
- Basic ROS 2 communication (nodes, topics, services)
- Python-based agents with sensor processing
- A URDF robot model
- Simulation and visualization

## Prerequisites

- Completed all previous sections on ROS 2 basics, Python integration, and URDF modeling
- ROS 2 Humble Hawksbill installed
- Basic Python programming knowledge

## Step 1: Understanding the Integrated Demo

The integrated demo combines all concepts into a single system:

1. **Robot Node**: Contains publishers, subscribers, and services
2. **Sensor Processing**: Responds to simulated sensor data
3. **Navigation Logic**: Avoids obstacles based on sensor input
4. **Service Integration**: Performs calculations on request

## Step 2: Running the Integrated Demo

1. Make sure your ROS 2 environment is sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Navigate to the examples directory:
   ```bash
   cd examples/ros2-robot-simulations/integrated-demo/
   ```

3. Run the integrated demo:
   ```bash
   python3 integrated_robot_demo.py
   ```

## Step 3: Understanding the Code

Let's examine the key components of the integrated demo:

### Publishers and Subscribers
```python
# Publishers
self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
self.status_publisher = self.create_publisher(String, 'robot_status', 10)

# Subscribers
self.scan_subscription = self.create_subscription(
    LaserScan,
    'scan',
    self.scan_callback,
    10)
```

### Service Server
```python
# Service server
self.service = self.create_service(AddTwoInts, 'robot_calculation', self.calculation_callback)
```

### Behavior Integration
The robot integrates multiple concepts in its behavior function:
- Processes sensor data to detect obstacles
- Responds to internal sensor values
- Publishes appropriate commands
- Maintains its state

## Step 4: Extending the Demo

Try modifying the demo to:
1. Add more complex navigation behaviors
2. Integrate additional sensor types
3. Create multiple agents that coordinate
4. Add more sophisticated decision-making logic

## Step 5: Connecting to URDF Model

The robot behavior can be visualized with the URDF model:

1. Load the URDF in RViz:
   ```bash
   ros2 launch urdf_tutorial display.launch.py model:=../urdf-humanoid-model/simple_humanoid.urdf
   ```

2. Visualize the robot's movement as it responds to simulated sensor data

## Key Takeaways

This tutorial demonstrates how all the individual concepts from this module work together in a cohesive system:

- **Nodes and Topics**: Enable communication between different components
- **Services**: Provide synchronous request-response capabilities
- **Python Agents**: Implement intelligent behavior and decision-making
- **URDF Models**: Provide the physical representation for simulation

## Troubleshooting

- If the demo doesn't run, ensure all dependencies are installed
- Check that your ROS 2 environment is properly sourced
- Verify that the necessary message types are available

## Next Steps

After completing this tutorial, you should have a solid understanding of how to:
- Combine multiple ROS 2 concepts in a single system
- Create intelligent agents that respond to sensor data
- Integrate URDF models with behavior code
- Design complete robotic systems

Continue to explore more advanced topics in robotics and ROS 2 development.