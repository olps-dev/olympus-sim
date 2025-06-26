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
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    """Generate launch description for Olympus Gazebo simulation."""

    # This assumes the launch file is in <root>/sim/ros2/launch
    olympus_sim_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

    # Paths for models and world files
    gazebo_worlds_path = os.path.join(olympus_sim_dir, 'sim', 'gazebo', 'worlds')

    # --- Launch Arguments ---
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='mmwave_test.sdf',
        description='Name of the world file in sim/gazebo/worlds to launch.'
    )
    use_mmwave_bridge_arg = DeclareLaunchArgument(
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
    # Environment variables should be set by the calling script (run_simulation.sh)
    gazebo = ExecuteProcess(
        # Launch Gazebo Sim in server-only mode with completely headless configuration
        cmd=[
            'gz', 'sim', 
            '-s',  # Server only mode
            '-r',  # Run simulation
            '--headless-rendering',  # Force headless rendering
            world_file_path
        ],
        output='screen',
    )

    # --- ROS2 Nodes ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
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

    # Static transform for the sensor, to place it in the world
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_mmwave',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'mmwave_sensor_link']
    )

    # --- Launch Description ---
    # Collect all the defined actions and return them to be executed.
    return LaunchDescription([
        world_arg,
        use_mmwave_bridge_arg,
        gazebo,
        bridge,
        mmwave_bridge,
        static_transform_publisher
    ])
