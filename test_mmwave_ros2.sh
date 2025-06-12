#!/bin/bash

# test_mmwave_ros2.sh
# Script to run mmWave sensor simulation with ROS2 integration in WSL Ubuntu
# This script launches Gazebo with the mmWave test world using ROS2 launch files

echo "==== Starting Environment Information ====" 
if grep -q Microsoft /proc/version; then
    echo "Running in WSL distribution: $(cat /etc/os-release | grep "^NAME" | cut -d= -f2 | tr -d '"')"
    echo "Setting LIBGL_ALWAYS_SOFTWARE=1 to force software rendering in WSL environment"
    export LIBGL_ALWAYS_SOFTWARE=1
fi

# Check if software rendering is enabled
if [ -n "$LIBGL_ALWAYS_SOFTWARE" ] && [ "$LIBGL_ALWAYS_SOFTWARE" -eq 1 ]; then
    echo "Software rendering is ENABLED for this Gazebo instance (LIBGL_ALWAYS_SOFTWARE=1)"
else
    echo "Software rendering is DISABLED for this Gazebo instance"
fi

# Source ROS2 Jazzy if not already sourced
if [ -z "$ROS_DISTRO" ] || [ "$ROS_DISTRO" != "jazzy" ]; then
    echo "Sourcing ROS2 Jazzy environment..."
    source /opt/ros/jazzy/setup.bash
    if [ $? -ne 0 ]; then
        echo "ERROR: Failed to source ROS2 Jazzy. Please ensure it is installed."
        exit 1
    fi
fi
echo "Using ROS2 distribution: $ROS_DISTRO"

# Set up paths for Gazebo resources and plugins
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/sim/gazebo/models:/usr/share/gz/gz-sim8/models:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$SCRIPT_DIR/sim/gazebo/plugins/build:/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:/gz-sim-8/plugins"

# Verify plugin exists
echo "Plugin path: $GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "Checking if plugin file exists:"
ls -la "$SCRIPT_DIR/sim/gazebo/plugins/build/libMmWaveSensorPlugin.so"

# Ensure the world file exists and copy it if needed
WORLD_FILE="$SCRIPT_DIR/sim/gazebo/worlds/mmwave_test.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: World file not found at $WORLD_FILE"
    exit 1
fi

# Create a ROS2 compatible world file by copying mmwave_test.sdf to olympus.world
# This ensures the ROS2 launch file can find it
cp "$WORLD_FILE" "$SCRIPT_DIR/sim/gazebo/worlds/olympus.world"
echo "Copied mmWave test world to olympus.world for ROS2 compatibility"

echo "==== Starting Gazebo with mmWave test world via ROS2 ====" 
echo "Software rendering status: ${LIBGL_ALWAYS_SOFTWARE:-DISABLED}"
echo "Starting simulation with ROS2 now..."

# Add the mmWave point cloud topic to the ros_gz_bridge
echo "Setting up bridge for mmWave point cloud topic..."
ros2 run ros_gz_bridge parameter_bridge /mmwave/points@sensor_msgs/msg/PointCloud2]gz.msgs.PointCloudPacked &
BRIDGE_PID=$!

# Run the simulation with ROS2 and Gazebo enabled
python3 "$SCRIPT_DIR/run_olympus.py" --ros2 --gazebo

# Clean up
echo "Simulation ended. Cleaning up..."
kill $BRIDGE_PID 2>/dev/null
rm -f "$SCRIPT_DIR/sim/gazebo/worlds/olympus.world"

# Print instructions for viewing point cloud data
echo ""
echo "To view mmWave point cloud data in another terminal, run:"
echo "ros2 topic echo /mmwave/points"
echo ""
echo "To visualize the point cloud in RViz2:"
echo "ros2 run rviz2 rviz2 -d $SCRIPT_DIR/sim/ros2/config/mmwave_pointcloud.rviz"

