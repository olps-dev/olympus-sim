#!/usr/bin/env python3
"""
Olympus Simulation Gazebo Launch File
This launch file starts Gazebo and loads the Olympus simulation world and models
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate launch description for Olympus Gazebo simulation."""
    
    # Get the path to the Olympus simulation directory
    olympus_sim_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    
    # Paths for models and world files
    gazebo_models_path = os.path.join(olympus_sim_dir, 'sim', 'gazebo', 'models')
    gazebo_worlds_path = os.path.join(olympus_sim_dir, 'sim', 'gazebo', 'worlds')
    
    # Default world file
    world_file = os.path.join(gazebo_worlds_path, 'olympus.world')
    if not os.path.exists(world_file):
        # If the world file doesn't exist, use an empty world
        world_file = ''
    
    # Set Gazebo model path environment variable
    os.environ['GAZEBO_MODEL_PATH'] = f"{gazebo_models_path}:{os.environ.get('GAZEBO_MODEL_PATH', '')}"
    
    # Launch Gazebo with our world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file],
        output='screen'
    )
    
    # Launch ROS2 nodes for Gazebo integration
    sensor_visualizer = Node(
        package='olympus_gazebo',
        executable='sensor_visualizer',
        name='sensor_visualizer',
        output='screen',
        parameters=[{
            'update_rate': 10.0,
        }],
        condition=LaunchConfiguration('use_sensor_visualizer', default=False)
    )
    
    # Return the launch description
    return LaunchDescription([
        gazebo,
        # Uncomment the line below when the sensor_visualizer package is available
        # sensor_visualizer,
    ])
