#!/bin/bash

# Set environment variables
export GZ_VERBOSE=4

# Ensure Gazebo plugins are found
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/aliza/olympus-sim/sim/gazebo/plugins/build:/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:/gz-sim-8/plugins

# Ensure Gazebo models are found by setting the resource path
export GZ_SIM_RESOURCE_PATH=/usr/share/gz/gz-sim8/models:$HOME/olympus-sim/sim/gazebo/models:${GZ_SIM_RESOURCE_PATH}

# Log OpenGL info (helpful for debugging)
echo "==== Starting Environment Information ===="
if [ -n "$WSL_DISTRO_NAME" ]; then
    echo "Running in WSL distribution: $WSL_DISTRO_NAME"
    echo "Setting LIBGL_ALWAYS_SOFTWARE=1 to force software rendering in WSL environment"
    # Force software rendering for the entire Gazebo process
    export LIBGL_ALWAYS_SOFTWARE=1
fi

# Check and report if software rendering is enabled
if [ "$LIBGL_ALWAYS_SOFTWARE" = "1" ]; then
    echo "Software rendering is ENABLED for this Gazebo instance (LIBGL_ALWAYS_SOFTWARE=1)"
else
    echo "WARNING: Software rendering is NOT enabled. Ray casting may not work in WSL."
    echo "Setting LIBGL_ALWAYS_SOFTWARE=1 now to force software rendering"
    export LIBGL_ALWAYS_SOFTWARE=1
fi

# Print out the plugin path to confirm it's correct
echo "Plugin path: $GZ_SIM_SYSTEM_PLUGIN_PATH"

# Test if the plugin file exists
echo "Checking if plugin file exists:"
ls -la /home/aliza/olympus-sim/sim/gazebo/plugins/build/libMmWaveSensorPlugin.so

# Launch Gazebo with mmWave test world
echo -e "\n==== Starting Gazebo with mmWave test world ===="
echo "Software rendering status: ${LIBGL_ALWAYS_SOFTWARE:+ENABLED}${LIBGL_ALWAYS_SOFTWARE:-NOT ENABLED}"
echo "Starting simulation now..."

# Use CD to make sure we're using proper relative paths
cd /home/aliza/olympus-sim
gz sim --verbose 4 -r /home/aliza/olympus-sim/sim/gazebo/worlds/mmwave_test.sdf

# Note: The MmWaveSensorPlugin will automatically set LIBGL_ALWAYS_SOFTWARE=1 if running in WSL
