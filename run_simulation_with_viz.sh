#!/bin/bash
#
# Olympus Simulation Launcher
#
# The previous automated script had issues with process control due to how `ros2 launch` works.
# This new version provides the exact commands to run in separate terminals for reliable control.
#

set -e

# --- Environment and Path Setup ---
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

# --- Instructions ---
echo "-------------------------------------------------------------------"
echo " Olympus Simulation Launcher"
echo "-------------------------------------------------------------------"
echo "
INSTRUCTIONS:

1. Open a NEW terminal.
2. Copy and run the [GAZEBO COMMAND] provided below.
3. Wait for Gazebo to start.
4. Open ANOTHER new terminal.
5. Copy and run the [RVIZ COMMAND] provided below.

To shut down, press Ctrl+C in EACH terminal.
"

# --- GAZEBO COMMAND ---
GAZEBO_LAUNCH_FILE="$SCRIPT_DIR/sim/ros2/launch/olympus_gazebo.launch.py"
echo "[GAZEBO COMMAND]:"
echo "source /opt/ros/jazzy/setup.bash && ros2 launch $GAZEBO_LAUNCH_FILE world:=mmwave_test.sdf"
echo ""

# --- RVIZ COMMAND ---
RVIZ_CONFIG_FILE="$SCRIPT_DIR/sim/ros2/config/mmwave.rviz"
echo "[RVIZ COMMAND]:"
echo "source /opt/ros/jazzy/setup.bash && rviz2 -d $RVIZ_CONFIG_FILE"
echo "-------------------------------------------------------------------"
