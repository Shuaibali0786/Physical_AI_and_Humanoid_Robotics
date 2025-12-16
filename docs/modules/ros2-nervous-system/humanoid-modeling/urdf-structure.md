# URDF Structure for Humanoid Robots

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links, joints, and other elements necessary for simulation and visualization.

## Basic URDF Structure

A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links -->
  <link name="link_name">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="color">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="joint_name" type="revolute">
    <parent link="parent_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Links

Links represent rigid parts of the robot. Each link has:

- **Visual**: How the link appears in visualization
- **Collision**: Geometry used for collision detection
- **Inertial**: Mass properties for physics simulation

### Visual Properties
```xml
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
```

### Collision Properties
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

### Inertial Properties
```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1"/>
  <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
</inertial>
```

## Joints

Joints connect links and define their relative motion. Common joint types:

- **revolute**: Rotational joint with limits
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint with limits
- **fixed**: No motion (rigid connection)
- **floating**: 6 DOF motion
- **planar**: Motion on a plane

### Joint Definition
```xml
<joint name="joint_name" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.2 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

## Humanoid Robot Structure

A humanoid robot typically has the following structure:

- **Base**: Usually the pelvis or torso
- **Upper body**: Torso, neck, head
- **Arms**: Shoulders, upper arms, forearms, hands
- **Lower body**: Hips, thighs, shins, feet

### Kinematic Chain Example
```
        head
         |
       neck
         |
       torso
      /     \
   shoulder  shoulder
     |         |
   upperarm  upperarm
     |         |
   forearm   forearm
     |         |
    hand      hand
     |
   torso
  /      \
hip      hip
 |        |
thigh   thigh
 |        |
shin    shin
 |        |
foot    foot
```

## URDF Best Practices

1. **Use meaningful names** for links and joints
2. **Maintain a proper kinematic tree** - no loops (except with mimic joints or transmissions)
3. **Define proper origins** for joints to connect links correctly
4. **Include inertial properties** for physics simulation
5. **Use consistent units** (meters for distance, radians for angles)
6. **Validate URDF** with tools like check_urdf
7. **Use xacro** for complex models to avoid repetition

## Common URDF Elements

- `<robot>`: Root element
- `<link>`: Rigid body element
- `<joint>`: Connection between links
- `<material>`: Visual material properties
- `<gazebo>`: Gazebo-specific extensions
- `<transmission>`: Actuator interfaces
- `<gripper>`: Gripper-specific elements

## Validating URDF Files

Before using a URDF file, validate it with:

```bash
check_urdf your_robot.urdf
```

This command will verify the syntax and structure of your URDF file.