#!/bin/bash

# Script to check if RViz2 is installed and launch it for mmWave visualization

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Check if rviz2 is available
if ! command -v rviz2 &> /dev/null; then
    echo "RViz2 is not installed. Installing RViz2..."
    sudo apt-get update && sudo apt-get install -y ros-jazzy-rviz2
    
    # Check if installation was successful
    if ! command -v rviz2 &> /dev/null; then
        echo "Failed to install RViz2. Please install it manually with:"
        echo "sudo apt-get install ros-jazzy-rviz2"
        exit 1
    fi
fi

echo "Launching RViz2 for mmWave pointcloud visualization..."

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Execute the RViz2 launch file
ros2 launch "${SCRIPT_DIR}/launch/rviz_mmwave.launch.py"
