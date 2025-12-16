# Quickstart Guide: ROS 2 Robotic Nervous System

## Overview
This guide provides a quick introduction to the ROS 2 educational module, allowing you to get up and running with the core concepts quickly.

## Prerequisites
- ROS 2 Humble Hawksbill installed (or later LTS version)
- Python 3.8 or higher
- Docusaurus development environment (Node.js, npm)
- Basic understanding of Python and Linux command line

## Setup ROS 2 Environment
1. Source your ROS 2 installation:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Create a workspace for examples:
   ```bash
   mkdir -p ~/ros2_book_examples/src
   cd ~/ros2_book_examples
   ```

## Running the Basic Publisher-Subscriber Example
1. Navigate to the example directory:
   ```bash
   cd examples/ros2-robot-simulations/basic-publisher-subscriber
   ```

2. Build the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

3. Run the publisher in one terminal:
   ```bash
   ros2 run demo_nodes_py talker
   ```

4. Run the subscriber in another terminal:
   ```bash
   ros2 run demo_nodes_py listener
   ```

## Running Python Agent Example
1. Navigate to the Python agent example:
   ```bash
   cd examples/ros2-robot-simulations/python-agent-control
   ```

2. Run the Python agent:
   ```bash
   python3 python_agent.py
   ```

## Working with URDF Models
1. Navigate to the URDF example:
   ```bash
   cd examples/ros2-robot-simulations/urdf-humanoid-model
   ```

2. Validate the URDF file:
   ```bash
   check_urdf simple_humanoid.urdf
   ```

3. Visualize the model in RViz:
   ```bash
   ros2 launch urdf_tutorial display.launch.py model:=simple_humanoid.urdf
   ```

## Building the Documentation
1. Navigate to the documentation directory:
   ```bash
   cd docs
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm start
   ```

## Validating Citations
All citations in this module follow APA format. You can find the complete bibliography in the references section. Each citation has been verified to meet the project's constitution requirement of minimum 50% peer-reviewed sources or official documentation.

## Next Steps
- Read the ROS 2 Basics module to understand fundamental concepts
- Try the hands-on Python integration examples
- Explore humanoid robot modeling with URDF
- Practice with the simulation examples provided