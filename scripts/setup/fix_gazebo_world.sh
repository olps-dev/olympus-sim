#!/bin/bash

# Define paths
ROS2_WS_PATH="$HOME/ros2_olympus_ws"
PACKAGE_NAME="olympus_gazebo"
WORLD_PATH="$ROS2_WS_PATH/install/$PACKAGE_NAME/share/$PACKAGE_NAME/worlds/apartment_sensors.world"

echo "Checking for world file at: $WORLD_PATH"

if [ -f "$WORLD_PATH" ]; then
    echo "Found world file. Fixing material script tags..."
    # Fix material script tags
    sed -i -e 's|<n>Gazebo/Red</n>|<name>Gazebo/Red</name>|g' "$WORLD_PATH"
    sed -i -e 's|<n>Gazebo/Green</n>|<name>Gazebo/Green</name>|g' "$WORLD_PATH"
    echo "Fixed material script tags in world file"
else
    echo "World file not found at: $WORLD_PATH"
    echo "Searching for world files..."
    find "$ROS2_WS_PATH" -name "apartment_sensors.world" 2>/dev/null
fi

# Ensure Python dependencies are installed
echo "Installing Python dependencies..."
pip install PyYAML numpy

# Set up Gazebo model paths
echo "Setting up Gazebo model paths..."
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/jazzy/share/gazebo_ros/models:$GAZEBO_MODEL_PATH
export GZ_SIM_RESOURCE_PATH=/usr/share/gazebo-11/models:/opt/ros/jazzy/share/gazebo_ros/models:$GZ_SIM_RESOURCE_PATH

echo "Fix script completed."
