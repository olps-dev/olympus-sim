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

echo "--- Launching Olympus Simulation ---"
echo "World contains: Red box (2,2), Green box (-2,3), Blue cylinder (3,-2)"
echo "mmWave sensor at origin should detect these obstacles"

# Check for GUI flag
GUI_MODE=false
if [[ "$1" == "--gui" ]]; then
    GUI_MODE=true
    echo "GUI mode requested - attempting Gazebo GUI"
    # Set up GUI environment
    export LIBGL_ALWAYS_SOFTWARE=1
    export QT_X11_NO_MITSHM=1
    export QT_QUICK_BACKEND=software
    export MESA_GL_VERSION_OVERRIDE=3.3
    export MESA_GLSL_VERSION_OVERRIDE=330
else
    echo "Running in headless mode (use --gui flag for GUI mode)"
fi

# Launch simulation
if [ "$GUI_MODE" = true ]; then
    ros2 launch "$OLYMPUS_SIM_ROOT/sim/ros2/launch/olympus_gazebo.launch.py" gui:=true rviz:=true
else
    ros2 launch "$OLYMPUS_SIM_ROOT/sim/ros2/launch/olympus_gazebo.launch.py" gui:=false rviz:=true
fi

# For debugging, let's print the variables.
echo "GZ_SIM_RESOURCE_PATH is: $GZ_SIM_RESOURCE_PATH"
echo "GZ_SIM_SYSTEM_PLUGIN_PATH is: $GZ_SIM_SYSTEM_PLUGIN_PATH"
