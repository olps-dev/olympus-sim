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

# Configure for WSL GUI operation
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330

# WSL GUI setup - check for different display options
echo "--- Configuring WSL Display ---"
if [ -n "$DISPLAY" ]; then
    echo "DISPLAY already set to: $DISPLAY"
elif [ -n "$WAYLAND_DISPLAY" ]; then
    echo "Using Wayland display: $WAYLAND_DISPLAY"
    export QT_QPA_PLATFORM=wayland
else
    echo "Setting up X11 display for WSL"
    export DISPLAY=:0
fi

# Additional WSL GUI environment variables
export QT_X11_NO_MITSHM=1
export QT_QUICK_BACKEND=software

# For debugging, let's print the variables.
echo "DISPLAY is set to: $DISPLAY"
echo "GZ_SIM_RESOURCE_PATH is: $GZ_SIM_RESOURCE_PATH"
echo "GZ_SIM_SYSTEM_PLUGIN_PATH is: $GZ_SIM_SYSTEM_PLUGIN_PATH"

echo "--- Launching Olympus Simulation with GUI ---"
echo "World contains: Red box (2,2), Green box (-2,3), Blue cylinder (3,-2)"
echo "mmWave sensor at origin should detect these obstacles"
echo "Note: Running Gazebo headless due to WSL OpenGL issues, but RViz2 will show the scene"

# Run Gazebo headless (which works) but with RViz2 for visualization
if command -v wslg &> /dev/null || [ -n "$WAYLAND_DISPLAY" ]; then
    echo "Using WSLg for RViz2 visualization"
    ros2 launch "$OLYMPUS_SIM_ROOT/sim/ros2/launch/olympus_gazebo.launch.py" gui:=false rviz:=true
elif command -v xvfb-run &> /dev/null; then
    echo "Using Xvfb virtual display for RViz2"
    xvfb-run -a -s "-screen 0 1920x1080x24 -ac +extension GLX +render -noreset" \
        ros2 launch "$OLYMPUS_SIM_ROOT/sim/ros2/launch/olympus_gazebo.launch.py" gui:=false rviz:=true
else
    echo "Running headless Gazebo with RViz2 visualization"
    ros2 launch "$OLYMPUS_SIM_ROOT/sim/ros2/launch/olympus_gazebo.launch.py" gui:=false rviz:=true
fi
