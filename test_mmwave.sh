#!/bin/bash

# Set environment variables
export GZ_VERBOSE=4
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/aliza/olympus-sim/sim/gazebo/plugins/build:/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:/gz-sim-8/plugins

# Print out the plugin path to confirm it's correct
echo "Plugin path: $GZ_SIM_SYSTEM_PLUGIN_PATH"

# Test if the plugin file exists
echo "Checking if plugin file exists:"
ls -la /home/aliza/olympus-sim/sim/gazebo/plugins/build/libMmWaveSensorPlugin.so

# Launch Gazebo with mmWave test world
echo "Starting Gazebo with mmWave test world"
gz sim --verbose 4 -r /home/aliza/olympus-sim/sim/gazebo/worlds/mmwave_test.sdf
