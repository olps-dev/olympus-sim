#!/usr/bin/env python3
"""
Olympus Simulation Gazebo Launch File
This launch file starts Gazebo Harmonic and loads the Olympus simulation world and models
Using ros_gz integration for ROS 2 Jazzy
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
    # Get the path to the Olympus simulation directory
    olympus_sim_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    
    # Paths for models and world files
    gazebo_models_path = os.path.join(olympus_sim_dir, 'sim', 'gazebo', 'models')
    gazebo_worlds_path = os.path.join(olympus_sim_dir, 'sim', 'gazebo', 'worlds')
    
    # Set environment variable for Gazebo models (include both standard and custom models)
    # Also try to include standard Gazebo model paths
    standard_paths = ["/usr/share/gz/gz-sim8/models", "/usr/share/gz/gz-sim8/worlds"]
    
    # Make sure our custom models path is first in the list
    model_path_value = gazebo_models_path
    
    # Add standard paths if they exist
    for path in standard_paths:
        if os.path.exists(path):
            model_path_value += ":{}".format(path)
    
    # Set the environment variable for models
    os.environ["GZ_SIM_RESOURCE_PATH"] = model_path_value
    
    # Plugin paths for custom plugins
    gazebo_plugin_path = os.path.join(olympus_sim_dir, 'sim', 'gazebo', 'plugins', 'build')
    standard_plugin_paths = ["/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins", "/gz-sim-8/plugins"]
    
    # Create the plugin path value
    plugin_path_value = gazebo_plugin_path
    
    # Add standard plugin paths if they exist
    for path in standard_plugin_paths:
        if os.path.exists(path):
            plugin_path_value += ":{}".format(path)
    
    # Set the environment variable for plugins
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = plugin_path_value
    print(f"[OlympusLaunch] Setting GZ_SIM_SYSTEM_PLUGIN_PATH to: {plugin_path_value}")
    
    # Default world file
    world_file = os.path.join(gazebo_worlds_path, 'olympus.world')
    print(f"[OlympusLaunch] Checking for world file at: {world_file}") # Added for debugging
    if not os.path.exists(world_file):
        print(f"[OlympusLaunch] World file NOT FOUND at: {world_file}. Gazebo will use an empty world.") # Added for debugging
        # If the world file doesn't exist, use an empty world
        world_file = ''
    else:
        print(f"[OlympusLaunch] World file FOUND at: {world_file}") # Added for debugging
    
    
    # Launch arguments
    use_sensor_visualizer = DeclareLaunchArgument(
        'use_sensor_visualizer',
        default_value='false',
        description='Whether to launch the sensor visualizer'
    )
    
    # Launch Gazebo Harmonic with our world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Bridge between ROS 2 and Gazebo
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
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock]gz.msgs.Clock',
            # TF (Gazebo -> ROS2)
            '/tf@tf2_msgs/msg/TFMessage]gz.msgs.Pose_V',
            # TF Static (Gazebo -> ROS2)
            '/tf_static@tf2_msgs/msg/TFMessage]gz.msgs.Pose_V'
        ]
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
        condition=IfCondition(LaunchConfiguration('use_sensor_visualizer'))
    )
    
    # Return the launch description
    return LaunchDescription([
        use_sensor_visualizer,
        gazebo,
        bridge,
        sensor_visualizer
    ])
