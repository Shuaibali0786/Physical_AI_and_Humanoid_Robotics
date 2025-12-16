import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get the urdf file path
    urdf_file_path = os.path.join(
        get_package_share_directory('ros2_robot_simulations'),
        'urdf-humanoid-model',
        'simple_humanoid.urdf'
    )

    # If the file doesn't exist in the package, use the direct path
    if not os.path.exists(urdf_file_path):
        urdf_file_path = os.path.join(
            os.getcwd(),
            'examples',
            'ros2-robot-simulations',
            'urdf-humanoid-model',
            'simple_humanoid.urdf'
        )

    # Launch configuration for the URDF file path
    model_path = LaunchConfiguration('model')

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=urdf_file_path,
        description='Absolute path to robot urdf file'
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', model_path])
        }]
    )

    # Joint State Publisher node (GUI for testing joint movements)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('ros2_robot_simulations'), 'rviz', 'urdf_config.rviz')]
    )

    # Create launch description and add actions
    ld = LaunchDescription()

    ld.add_action(model_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    # Note: RViz config file might not exist, so we'll comment this out for now
    # ld.add_action(rviz_node)

    return ld