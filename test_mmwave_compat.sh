#!/bin/bash

# Set environment variables for better debugging
export GZ_VERBOSE=3
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/aliza/olympus-sim/sim/gazebo/plugins/build:/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins
export GZ_SIM_RESOURCE_PATH=/usr/share/gz/gz-sim8/models:$HOME/olympus-sim/sim/gazebo/models:${GZ_SIM_RESOURCE_PATH}

# Force software rendering for WSL environment
export LIBGL_ALWAYS_SOFTWARE=1

# Run the simulation with extra logging for MmWaveSensorPlugin
echo "==== Starting Test with Fixed WSL Compatibility Mode Activation ===="
echo "Running test to verify WSL compatibility mode activates after 5 attempts..."

# Use grep to filter just the MmWaveSensorPlugin messages related to WSL compatibility
gz sim -v 4 -r /home/aliza/olympus-sim/sim/gazebo/worlds/mmwave_test.sdf 2>&1 | grep -E "(MmWaveSensorPlugin|wslCompatMode|attempt|WSL)"
