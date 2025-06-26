#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

echo "--- Sourcing ROS 2 Jazzy ---"
# Source ROS 2 Jazzy environment. This also sets up Gazebo Harmonic.
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "Error: ROS 2 Jazzy setup script not found. Please ensure ROS 2 is installed correctly." >&2
  exit 1
fi


echo "--- Setting Gazebo environment variables ---"

# Define the project root directory using the script's location
OLYMPUS_SIM_ROOT=$(realpath "$(dirname "${BASH_SOURCE[0]}")")

# Set Gazebo resource path cleanly to avoid duplication.
# It should point to the project's models and the system's models.
# ROS 2 Jazzy uses Gazebo Harmonic (gz-sim8).
export GZ_SIM_RESOURCE_PATH="$OLYMPUS_SIM_ROOT/sim/gazebo/models"
if [ -d "/usr/share/gz/gz-sim8/models" ]; then
    export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:/usr/share/gz/gz-sim8/models"
fi

# Set Gazebo plugin path for custom plugins.
export GZ_SIM_SYSTEM_PLUGIN_PATH="$OLYMPUS_SIM_ROOT/sim/gazebo/plugins/build"

# Force software rendering for WSL compatibility
export LIBGL_ALWAYS_SOFTWARE=1

# Additional headless rendering configuration
export GZ_SIM_RENDER_ENGINE=ogre
export OGRE_RTT_MODE=Copy
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330

# Force Mesa software driver and disable hardware acceleration
export MESA_LOADER_DRIVER_OVERRIDE=swrast
export GALLIUM_DRIVER=llvmpipe
export LIBGL_ALWAYS_INDIRECT=1
export __GLX_VENDOR_LIBRARY_NAME=mesa

# Completely disable graphics and X11 for true headless operation
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=softpipe
export MESA_LOADER_DRIVER_OVERRIDE=swrast
export GZ_SIM_RENDER_ENGINE_TYPE=NONE
export OGRE_SKIP_DLL_PLUGINS=1
export QT_QPA_PLATFORM=offscreen

# For debugging, let's print the variables.
echo "DISPLAY is set to: $DISPLAY"
echo "GZ_SIM_RESOURCE_PATH is: $GZ_SIM_RESOURCE_PATH"
echo "GZ_SIM_SYSTEM_PLUGIN_PATH is: $GZ_SIM_SYSTEM_PLUGIN_PATH"


echo "--- Launching Olympus Gazebo simulation with virtual display ---"

# Use xvfb-run to provide a virtual display for Gazebo
# This resolves OpenGL/GLX context issues in WSL environments
xvfb-run -a -s "-screen 0 1024x768x24 -ac +extension GLX +render -noreset" \
    ros2 launch "$OLYMPUS_SIM_ROOT/sim/ros2/launch/olympus_gazebo.launch.py"
