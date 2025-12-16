# Simulation Setup for Humanoid Robots

## Overview

Setting up humanoid robots for simulation involves configuring the URDF model for use in simulation environments like Gazebo. This includes adding plugins for physics, sensors, and controllers.

## Gazebo Integration

To integrate your URDF with Gazebo, add Gazebo-specific extensions:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Red</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>
```

### Adding Gazebo Plugins

For physics simulation, controllers, and sensors, you need to add plugins:

```xml
<gazebo>
  <plugin name="robot_state_publisher" filename="libgazebo_ros_robot_state_publisher.so">
    <tf_topic>tf</tf_topic>
    <robot_param>robot_description</robot_param>
  </plugin>
</gazebo>
```

## Controller Configuration

To control your humanoid robot in simulation, you need controller configurations:

### Joint State Controller
```yaml
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50
```

### Position Controllers
```yaml
arm_controller:
  type: position_controllers/JointPositionController
  joints: [shoulder_joint, elbow_joint, wrist_joint]
```

## Launch Files

Create launch files to start your simulation:

```xml
<launch>
  <!-- Load robot description -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find package)/urdf/robot.urdf.xacro'" />

  <!-- Spawn robot in Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model robot_name" />

  <!-- Start robot state publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
</launch>
```

## RViz Visualization

For visualization in RViz, create a configuration file:

```yaml
Panels:
  - Class: rviz/Displays
    Name: Displays
  - Class: rviz/Views
    Name: Views
Visualization Manager:
  Displays:
    - Class: rviz/RobotModel
      Name: RobotModel
      Topic: /robot_description
      Update Interval: 0
```

## Basic Humanoid URDF Example

Here's a minimal humanoid URDF with simulation setup:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Fixed base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Joint connecting base to torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <!-- Gazebo extensions -->
  <gazebo>
    <plugin name="ground_truth" filename="libgazebo_ros_p3d.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>30.0</updateRate>
      <bodyName>base_link</bodyName>
      <topicName>base_pose_ground_truth</topicName>
      <gaussianNoise>0.01</gaussianNoise>
      <frameName>map</frameName>
    </plugin>
  </gazebo>
</robot>
```

## Simulation Best Practices

1. **Start Simple**: Begin with a basic model and add complexity gradually
2. **Validate Physics**: Ensure inertial properties are realistic
3. **Tune Parameters**: Adjust friction, damping, and other physics parameters
4. **Use Appropriate Meshes**: For visualization, use simplified meshes for collision
5. **Consider Performance**: Balance detail with simulation speed
6. **Test Controllers**: Verify that controllers work properly with your model

## Debugging Simulation Issues

Common issues and solutions:

- **Robot falls through the ground**: Check collision geometries and inertial properties
- **Joints behave strangely**: Verify joint limits and origins
- **Simulation is unstable**: Adjust solver parameters or reduce complexity
- **Robot doesn't respond to commands**: Check controller configurations and joint names

## Testing URDF Models

1. **Visual inspection**: Load in RViz to check structure
2. **Validation**: Use `check_urdf` to verify syntax
3. **Simulation**: Test in Gazebo for physics behavior
4. **Kinematics**: Verify joint ranges and motion patterns