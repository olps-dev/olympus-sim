#!/usr/bin/env python3
"""
Olympus Simulation Gazebo Launch File
This launch file starts Gazebo Harmonic and loads the Olympus simulation world and models
Using ros_gz integration for ROS 2 Jazzy
Includes mmWave sensor ROS2 bridge and optional RViz2 visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    """Generate launch description for Olympus Gazebo simulation."""

    # This assumes the launch file is in <root>/sim/ros2/launch
    olympus_sim_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

    # Paths for models and world files
    gazebo_worlds_path = os.path.join(olympus_sim_dir, 'sim', 'gazebo', 'worlds')
    world_file = os.path.join(gazebo_worlds_path, 'mmwave_test.sdf')

    # --- Launch Arguments ---
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Whether to launch Gazebo with GUI'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz2 for visualization'
    )

    # --- Gazebo Process ---
    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '-r',  # Run simulation
            '-v', '4',  # Verbose output for debugging
            world_file
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'QT_X11_NO_MITSHM': '1',
            'QT_QUICK_BACKEND': 'software',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
            'MESA_GLSL_VERSION_OVERRIDE': '330'
        }
    )
    
    # Headless Gazebo for when GUI is disabled
    gazebo_headless = ExecuteProcess(
        cmd=[
            'gz', 'sim', 
            '-s',  # Server only mode
            '-r',  # Run simulation
            '--headless-rendering',  # Force headless rendering
            '-v', '4',  # Verbose output for debugging
            world_file
        ],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        additional_env={
            'GZ_SIM_SERVER_CONFIG_PATH': '',
            'LIBGL_ALWAYS_SOFTWARE': '1'
        }
    )

    # --- ROS2 Nodes ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )

    # mmWave sensor ROS2 bridge
    mmwave_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/mmwave/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/mmwave/points', '/mmwave/pointcloud')
        ],
        output='screen'
    )

    # Static transform publisher for mmWave sensor
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '1', '0', '0', '0', 'world', 'mmwave']
    )

    # RViz2 node for visualization (optional)
    rviz_config_path = os.path.join(olympus_sim_dir, 'sim', 'ros2', 'config', 'olympus_rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{
            'use_sim_time': True
        }],
        additional_env={
            'QT_X11_NO_MITSHM': '1',
            'QT_QUICK_BACKEND': 'software'
        }
    )

    # mmWave MQTT Bridge - converts ROS2 pointcloud to MQTT presence detection
    mmwave_mqtt_bridge = Node(
        package='olympus_sim',
        executable='mmwave_mqtt_bridge.py',
        name='mmwave_mqtt_bridge',
        output='screen',
        parameters=[{
            'mqtt_broker': 'localhost',
            'mqtt_port': 1883,
            'sensor_id': 'mmwave1',
            'detection_threshold': 0.1,
            'max_detection_range': 10.0,
            'min_points_for_detection': 5,
            'use_sim_time': True
        }]
    )

    # --- Launch Description ---
    return LaunchDescription([
        gui_arg,
        rviz_arg,
        gazebo,
        gazebo_headless,
        bridge,
        mmwave_bridge,
        static_transform_publisher,
        rviz_node,
        mmwave_mqtt_bridge
    ])
