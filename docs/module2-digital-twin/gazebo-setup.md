# Gazebo Environment Setup

## Introduction to Gazebo Environment Configuration

Setting up comprehensive Gazebo environments is crucial for effective humanoid robot simulation. A well-configured environment provides realistic physics, appropriate lighting, and accurate sensor simulation that closely matches real-world conditions.

## Gazebo World File Structure

### Basic World File Components

A Gazebo world file is an XML document that defines the complete simulation environment:

```xml
<sdf version='1.7'>
  <world name='default'>
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Scene lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Models and objects -->
    <!-- Robot models, furniture, etc. -->

  </world>
</sdf>
```

### Physics Configuration

#### Real-time Performance Settings

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Simulation time step -->
  <real_time_factor>1.0</real_time_factor>  <!-- Simulation speed -->
  <real_time_update_rate>1000.0</real_time_update_rate>  <!-- Update frequency -->
  <gravity>0 0 -9.8</gravity>  <!-- Gravity vector -->
</physics>
```

#### Solver Parameters

- **Step size**: Smaller for accuracy, larger for performance
- **Real-time factor**: 1.0 for real-time, higher for faster simulation
- **Update rate**: Higher for more responsive simulation
- **Gravity**: Standard Earth gravity (0, 0, -9.8) or custom

## Environment Elements

### Lighting Setup

#### Sun Model Configuration

```xml
<model name='sun'>
  <pose>0 0 10 0 0 0</pose>
  <link name='link'>
    <visual name='visual'>
      <geometry>
        <sphere><radius>1000.0</radius></sphere>
      </geometry>
      <material>
        <script>Gazebo/Blue</script>
      </material>
    </visual>
  </link>
</model>
```

#### Custom Light Sources

For indoor environments, add custom lights:

```xml
<light name='room_light' type='point'>
  <pose>0 0 3 0 0 0</pose>
  <diffuse>1 1 1 1</diffuse>
  <specular>0.5 0.5 0.5 1</specular>
  <attenuation>
    <range>10</range>
    <linear>0.1</linear>
    <quadratic>0.01</quadratic>
  </attenuation>
</light>
```

### Ground and Terrain

#### Ground Plane

The default ground plane:

```xml
<include>
  <uri>model://ground_plane</uri>
</include>
```

#### Custom Terrain

For complex terrain:

```xml
<model name='terrain'>
  <static>true</static>
  <link name='link'>
    <collision name='collision'>
      <geometry>
        <heightmap>
          <uri>model://terrain/heightmap.png</uri>
          <size>100 100 20</size>
        </heightmap>
      </geometry>
    </collision>
    <visual name='visual'>
      <geometry>
        <heightmap>
          <uri>model://terrain/heightmap.png</uri>
          <size>100 100 20</size>
        </heightmap>
      </geometry>
    </visual>
  </link>
</model>
```

## Model Placement and Configuration

### Robot Model Integration

Include your humanoid robot model:

```xml
<include>
  <name>humanoid_robot</name>
  <pose>0 0 1 0 0 0</pose>
  <uri>model://my_humanoid_robot</uri>
</include>
```

### Environment Objects

Add furniture, obstacles, and other objects:

```xml
<include>
  <name>table</name>
  <pose>2 0 0 0 0 0</pose>
  <uri>model://table</uri>
</include>

<include>
  <name>box</name>
  <pose>-1 1 0.5 0 0 0</pose>
  <uri>model://box</uri>
</include>
```

## Advanced Environment Features

### Plugins for Enhanced Functionality

#### World Control Plugin

```xml
<plugin name="world_control" filename="libgazebo_ros_world.so">
  <ros>
    <namespace>/gazebo</namespace>
  </ros>
  <update_rate>1.0</update_rate>
</plugin>
```

#### Joint State Publisher

```xml
<plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
  <ros>
    <namespace>/robot</namespace>
    <remapping>~/out:=joint_states</remapping>
  </ros>
  <update_rate>30</update_rate>
  <joint_name>joint1</joint_name>
  <joint_name>joint2</joint_name>
</plugin>
```

### Camera and Sensor Worlds

Create worlds optimized for sensor simulation:

```xml
<world name='sensor_test_world'>
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>

  <!-- Calibrated targets for sensor validation -->
  <include>
    <name>calibration_board</name>
    <pose>1 0 1 0 0 0</pose>
    <uri>model://calibration_board</uri>
  </include>

  <!-- Various materials for sensor testing -->
  <include>
    <name>material_test_grid</name>
    <pose>-2 0 0 0 0 0</pose>
    <uri>model://material_test_grid</uri>
  </include>
</world>
```

## Environment Customization

### Indoor Environments

For indoor humanoid robot testing:

```xml
<world name='indoor_office'>
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>

  <!-- Indoor lighting -->
  <light name='ceiling_light' type='point'>
    <pose>0 0 2.5 0 0 0</pose>
    <diffuse>0.8 0.8 0.8 1</diffuse>
  </light>

  <!-- Office furniture -->
  <include>
    <name>desk</name>
    <pose>1 0 0 0 0 0</pose>
    <uri>model://desk</uri>
  </include>

  <include>
    <name>chair</name>
    <pose>1.5 -0.8 0 0 0 1.57</uri>
    <uri>model://chair</uri>
  </include>

  <!-- Walls -->
  <model name='wall_1'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry><box><size>5 0.2 2.5</size></box></geometry>
      </collision>
      <visual name='visual'>
        <geometry><box><size>5 0.2 2.5</size></box></geometry>
      </visual>
    </link>
  </model>
</world>
```

### Outdoor Environments

For outdoor humanoid robot testing:

```xml
<world name='outdoor_park'>
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>

  <!-- Terrain with varying elevations -->
  <model name='hilly_terrain'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <heightmap>
            <uri>file://terrain/hilly_heightmap.png</uri>
            <size>50 50 5</size>
          </heightmap>
        </geometry>
      </collision>
    </link>
  </model>

  <!-- Trees and vegetation -->
  <include>
    <name>tree_1</name>
    <pose>5 3 0 0 0 0</pose>
    <uri>model://tree</uri>
  </include>

  <include>
    <name>bench</name>
    <pose>-2 -4 0 0 0 0.785</pose>
    <uri>model://bench</uri>
  </include>
</world>
```

## Environment Validation

### Performance Testing

Test your environment setup:

- **Real-time factor**: Ensure it stays close to 1.0
- **Physics stability**: Check for jittering or instability
- **Sensor accuracy**: Validate sensor outputs
- **Collision detection**: Verify proper collision behavior

### Multi-Robot Environments

For testing multiple humanoid robots:

```xml
<world name='multi_robot'>
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>

  <!-- Robot 1 -->
  <include>
    <name>robot_1</name>
    <pose>0 0 1 0 0 0</pose>
    <uri>model://humanoid_robot</uri>
  </include>

  <!-- Robot 2 -->
  <include>
    <name>robot_2</name>
    <pose>2 0 1 0 0 0</pose>
    <uri>model://humanoid_robot</uri>
  </include>

  <!-- Shared environment objects -->
  <include>
    <name>shared_table</name>
    <pose>1 1 0 0 0 0</pose>
    <uri>model://table</uri>
  </include>
</world>
```

## Best Practices

### Environment Design Guidelines

- **Start simple**: Begin with basic environments and add complexity
- **Validate physics**: Ensure stable and realistic physics simulation
- **Test sensor outputs**: Verify that sensors behave as expected
- **Optimize performance**: Balance realism with computational efficiency

### Model Management

- **Organize models**: Keep custom models in a structured directory
- **Version control**: Track changes to environment configurations
- **Documentation**: Document environment parameters and assumptions
- **Reusability**: Design modular environments that can be combined

### Safety Considerations

- **Simulation limits**: Define operational boundaries
- **Emergency stops**: Implement simulation pause capabilities
- **Failure modes**: Test robot behavior under various failure conditions
- **Validation**: Compare simulation results with real-world data

Proper Gazebo environment setup is essential for effective humanoid robot development, providing a safe, controllable, and repeatable testing environment.