#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to RViz configuration file
    rviz_config_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'mmwave_rviz.rviz'
    )
    
    # Check if config exists, if not we'll use default config
    use_default_config = not os.path.exists(rviz_config_path)
    
    # Create a LaunchDescription object
    ld = LaunchDescription([
        # Launch RViz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path] if not use_default_config else [],
            output='screen',
        )
    ])
    
    return ld
