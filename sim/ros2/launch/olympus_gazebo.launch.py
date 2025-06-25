#!/usr/bin/env python3
"""
Olympus Simulation Gazebo Launch File
This launch file starts Gazebo Harmonic and loads the Olympus simulation world and models
Using ros_gz integration for ROS 2 Jazzy
Includes mmWave sensor ROS2 bridge
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    """Generate launch description for Olympus Gazebo simulation."""

    # Get the path to the Olympus simulation directory
    olympus_sim_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

    # Paths for models and world files
    gazebo_models_path = os.path.join(olympus_sim_dir, 'sim', 'gazebo', 'models')
    gazebo_worlds_path = os.path.join(olympus_sim_dir, 'sim', 'gazebo', 'worlds')

    # Construct Gazebo paths, prepending custom paths to existing ones
    gazebo_plugin_path = os.path.join(olympus_sim_dir, 'sim', 'gazebo', 'plugins', 'build')
    
    # Get existing environment variables
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    existing_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')

    # Prepend our custom paths
    new_resource_path = f"{gazebo_models_path}:{existing_resource_path}"
    new_plugin_path = f"{gazebo_plugin_path}:{existing_plugin_path}"

    print(f"[OlympusLaunch] GZ_SIM_RESOURCE_PATH: {new_resource_path}")
    print(f"[OlympusLaunch] GZ_SIM_SYSTEM_PLUGIN_PATH: {new_plugin_path}")

    # --- Launch Arguments ---
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='mmwave_test.sdf',
        description='Name of the world file in sim/gazebo/worlds to launch.'
    )
    use_sensor_visualizer = DeclareLaunchArgument(
        'use_sensor_visualizer',
        default_value='false',
        description='Whether to launch the sensor visualizer'
    )
    use_mmwave_bridge = DeclareLaunchArgument(
        'use_mmwave_bridge',
        default_value='true',
        description='Whether to launch the mmWave sensor ROS2 bridge'
    )

    # Construct the full path to the world file
    world_file_path = PathJoinSubstitution([
        gazebo_worlds_path,
        LaunchConfiguration('world')
    ])

    # --- Gazebo Process ---
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file_path],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': new_resource_path,
            'GZ_SIM_SYSTEM_PLUGIN_PATH': new_plugin_path
        }
    )

    # --- ROS2 Nodes ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': '',
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        arguments=[
            # Bridge Gazebo clock to ROS clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Bridge Gazebo poses to ROS TF
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ]
    )

    mmwave_bridge_script = os.path.join(olympus_sim_dir, 'sim', 'ros2', 'mmwave_ros2_bridge.py')
    mmwave_bridge = Node(
        executable=mmwave_bridge_script,
        name='mmwave_ros2_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mmwave_bridge'))
    )

    # Static transform publisher for the mmWave sensor
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_mmwave',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'mmwave_sensor_link']
    )

    # --- Launch Description ---
    return LaunchDescription([
        # Launch Arguments
        world_arg,
        use_sensor_visualizer,
        use_mmwave_bridge,

        # Processes and Nodes
        gazebo,
        bridge,
        mmwave_bridge,
        static_tf_publisher,
    ])
