#!/usr/bin/env python3
"""
Launch file for Olympus Simulation with ROS2 and Gazebo integration
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    mqtt_broker = LaunchConfiguration('mqtt_broker', default='localhost')
    mqtt_port = LaunchConfiguration('mqtt_port', default='1883')
    
    # Get the Gazebo paths
    gazebo_ros_pkg_prefix = FindPackageShare('gazebo_ros')
    
    # Determine the path to the world file
    # This assumes the world file is in the sim/gazebo/worlds directory
    olympus_sim_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    world_file = os.path.join(olympus_sim_dir, 'sim', 'gazebo', 'worlds', 'olympus_fixed.world')
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_mqtt_broker = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='localhost',
        description='MQTT broker address'
    )
    
    declare_mqtt_port = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT broker port'
    )
    
    # Set up environment variables for Gazebo
    plugin_path = os.path.join(olympus_sim_dir, 'sim', 'gazebo', 'plugins', 'build')
    resource_path = os.path.join(olympus_sim_dir, 'sim', 'gazebo', 'models')
    
    # Create environment variable dict with existing env plus our additions
    env = dict(os.environ)
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = f"{plugin_path}:{env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"
    env['GZ_SIM_RESOURCE_PATH'] = f"{resource_path}:/usr/share/gz/gz-sim8/models:{env.get('GZ_SIM_RESOURCE_PATH', '')}"
    env['LIBGL_ALWAYS_SOFTWARE'] = '1'  # Force software rendering for WSL
    
    # Launch Gazebo with environment variables
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros_pkg_prefix, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items(),
        environment=env
    )
    
    # Launch MQTT-ROS2 Bridge
    mqtt_ros2_bridge = Node(
        package='olympus_sim',
        executable='mqtt_ros2_bridge.py',
        name='mqtt_ros2_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'mqtt_broker': mqtt_broker,
            'mqtt_port': mqtt_port,
            'mqtt_topic_prefix': 'olympus/'
        }]
    )
    
    # Launch Time Sync Bridge (if needed)
    time_sync_bridge = Node(
        package='olympus_sim',
        executable='time_sync_bridge.py',
        name='time_sync_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Launch mmWave ROS2 Bridge
    mmwave_ros2_bridge = Node(
        package='olympus_sim',
        executable='mmwave_ros2_bridge.py',
        name='mmwave_ros2_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'mmwave_gazebo_topic': '/mmwave/points',
            'mmwave_ros2_topic': '/mmwave/pointcloud'
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_mqtt_broker,
        declare_mqtt_port,
        gazebo,
        mqtt_ros2_bridge,
        time_sync_bridge,
        mmwave_ros2_bridge  # Add the mmWave ROS2 bridge
    ])
