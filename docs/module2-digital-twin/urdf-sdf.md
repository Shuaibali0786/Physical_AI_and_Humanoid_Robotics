# URDF and SDF Formats

## Introduction to Robot Description Formats

URDF (Unified Robot Description Format) and SDF (Simulation Description Format) are the two primary formats used to describe robots and environments in robotics simulation. Understanding both formats and their relationship is crucial for effective humanoid robot development in Gazebo and other simulation environments.

## URDF (Unified Robot Description Format)

### URDF Overview

URDF is an XML-based format used primarily in ROS to describe robot models. It defines the physical and visual properties of a robot including links, joints, and other elements necessary for simulation and visualization.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define rigid parts of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_upper_body" type="fixed">
    <parent link="base_link"/>
    <child link="upper_body"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <link name="upper_body">
    <!-- Additional link definition -->
  </link>
</robot>
```

### URDF Elements

#### Links

Links represent rigid parts of the robot with three main components:

- **Visual**: How the link appears in visualization
- **Collision**: Geometry used for collision detection
- **Inertial**: Mass properties for physics simulation

```xml
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Shape can be box, cylinder, sphere, or mesh -->
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
  </inertial>
</link>
```

#### Joints

Joints connect links and define their relative motion:

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

Joint types:
- **fixed**: No motion (rigid connection)
- **revolute**: Rotational joint with limits
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint with limits
- **floating**: 6 DOF motion
- **planar**: Motion on a plane

### URDF Gazebo Extensions

To use URDF models in Gazebo, special Gazebo-specific tags are added:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.9</mu1>
  <mu2>0.9</mu2>
</gazebo>

<gazebo>
  <plugin name="control_plugin" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
  </plugin>
</gazebo>
```

## SDF (Simulation Description Format)

### SDF Overview

SDF is Gazebo's native format for describing simulation environments. It's more comprehensive than URDF and can describe entire simulation worlds, including physics, lighting, models, and plugins.

### Basic SDF Structure

```xml
<sdf version="1.7">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <model name="humanoid_robot" canonical_link="base_link">
      <pose>0 0 1 0 0 0</pose>

      <link name="base_link">
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>

        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.6</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Blue</name>
            </script>
          </material>
        </visual>

        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.6</length>
            </cylinder>
          </geometry>
        </collision>
      </link>

      <joint name="base_to_upper_body" type="revolute">
        <parent>base_link</parent>
        <child>upper_body</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-1.57</lower>
            <upper>1.57</upper>
            <effort>100</effort>
            <velocity>1</velocity>
          </limit>
        </axis>
      </joint>
    </model>
  </world>
</sdf>
```

### SDF World Elements

#### Physics Configuration

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

#### Lighting and Environment

```xml
<light name='sun' type='directional'>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>1000</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <direction>-0.1 0.1 -1.0</direction>
</light>
```

## Converting Between URDF and SDF

### URDF to SDF Conversion

Gazebo can automatically convert URDF to SDF when loading models:

```bash
gz sdf -p model.urdf
```

This command prints the SDF representation of a URDF file.

### Embedding URDF in SDF

You can include URDF content within SDF:

```xml
<sdf version="1.7">
  <model name="humanoid_with_urdf">
    <!-- Include URDF content here -->
    <include>
      <uri>model://humanoid_robot/model.urdf</uri>
    </include>
  </model>
</sdf>
```

## Best Practices for Format Selection

### When to Use URDF

- When working primarily with ROS/ROS 2
- For robot description that will be used across multiple tools
- When you need simpler, more focused robot descriptions
- For kinematic and dynamic analysis

### When to Use SDF

- When working specifically with Gazebo
- For complex simulation scenarios
- When you need to describe entire worlds
- For advanced physics and rendering configurations

## Practical Examples

### Complete Humanoid Robot URDF

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

### Gazebo-Specific Extensions for URDF

```xml
<gazebo reference="base_link">
  <material>Gazebo/Grey</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<gazebo reference="torso">
  <material>Gazebo/Blue</material>
</gazebo>

<gazebo reference="head">
  <material>Gazebo/Yellow</material>
</gazebo>

<gazebo>
  <plugin name="robot_state_publisher" filename="libgazebo_ros_robot_state_publisher.so">
    <ros>
      <namespace>/humanoid_robot</namespace>
    </ros>
  </plugin>
</gazebo>
```

## Validation and Tools

### URDF Validation

Validate URDF files:

```bash
check_urdf my_robot.urdf
```

This checks for syntax errors and structural issues.

### Visualization Tools

- **RViz**: Visualize URDF models in ROS
- **Gazebo**: Visualize both URDF and SDF models
- **URDF Viewer**: Standalone URDF visualization tools

## Common Pitfalls and Solutions

### URDF Issues

- **Mass/inertia**: Always define realistic mass and inertia values
- **Joint limits**: Set appropriate joint limits to prevent damage
- **Collision geometry**: Ensure collision geometry matches visual geometry
- **Fixed joints**: Use fixed joints appropriately to connect static parts

### SDF Issues

- **Coordinate systems**: Ensure consistent coordinate system usage
- **Units**: Use consistent units throughout the description
- **Performance**: Optimize collision geometry for better performance
- **Plugins**: Properly configure simulation plugins

Understanding both URDF and SDF formats and their appropriate use cases is essential for effective humanoid robot simulation and development in modern robotics frameworks.