#!/bin/bash

# test_mmwave_ros2_pointcloud.sh
# Script to run mmWave sensor simulation with ROS2 integration and verify pointcloud data

echo "==== Starting mmWave ROS2 PointCloud Test ===="
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

# Set up paths for Gazebo resources and plugins
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/sim/gazebo/models:/usr/share/gz/gz-sim8/models:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$SCRIPT_DIR/sim/gazebo/plugins/build:/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:/gz-sim-8/plugins"

# Verify plugin exists
echo "Plugin path: $GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "Checking if plugin file exists:"
ls -la "$SCRIPT_DIR/sim/gazebo/plugins/build/libMmWaveSensorPlugin.so"

# Ensure the world file exists
WORLD_FILE="$SCRIPT_DIR/sim/gazebo/worlds/mmwave_test.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: World file not found at $WORLD_FILE"
    exit 1
fi

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 environment not sourced. Please run 'source /opt/ros/jazzy/setup.bash' first."
    exit 1
fi

# Make the mmwave_ros2_bridge.py executable
chmod +x "$SCRIPT_DIR/sim/ros2/mmwave_ros2_bridge.py"
echo "Made mmwave_ros2_bridge.py executable"

# Terminal 1: Run the simulation with ROS2 and Gazebo enabled
echo "==== Starting Gazebo with mmWave test world via ROS2 ===="
echo "Software rendering status: ${LIBGL_ALWAYS_SOFTWARE:-DISABLED}"
echo "Starting simulation with ROS2 now..."

# Run the simulation with ROS2 and Gazebo enabled in the background
python3 "$SCRIPT_DIR/run_olympus.py" --ros2 --gazebo &
SIM_PID=$!

# Wait for the simulation to start
sleep 10

# Launch the mmWave bridge node using ROS2 run to ensure proper node registration
echo "==== Starting mmWave ROS2 bridge node ===="
ros2 run --prefix "python3" "$SCRIPT_DIR/sim/ros2/mmwave_ros2_bridge.py" --output "bridge_output.log" &
BRIDGE_PID=$!
echo "Bridge node started with PID: $BRIDGE_PID"

# Wait longer for the bridge node to fully initialize and connect
sleep 5

# Check if bridge node is running
if ! ps -p $BRIDGE_PID > /dev/null; then
    echo "ERROR: Bridge node failed to start or terminated prematurely!"
    echo "Checking log output:"
    tail -n 50 bridge_output.log 2>/dev/null || echo "No log file found"
else
    echo "Bridge node is running with PID: $BRIDGE_PID"
fi

# Check Gazebo topics to confirm points are being published
echo "\n==== Checking Gazebo topics ===="
gazebo_topics=$(gz topic -l)
echo "$gazebo_topics" | grep -E "/mmwave/points"
if echo "$gazebo_topics" | grep -q "/mmwave/points"; then
    echo "SUCCESS: Gazebo is publishing on /mmwave/points topic"
else
    echo "WARNING: Gazebo topic /mmwave/points not found"
fi

# Terminal 2: Monitor and verify ROS2 topics
echo "==== Monitoring ROS2 Environment ===="

# List all available ROS2 nodes
echo "\n==== ROS2 Nodes ===="
ros2 node list

# List all available ROS2 topics
echo "\n==== ROS2 Topics ===="
ros2 topic list

# Check specifically for the mmWave topic
echo "\n==== Checking for /mmwave/pointcloud topic ===="
topic_count=$(ros2 topic list | grep -c "/mmwave/pointcloud")
if [ "$topic_count" -gt 0 ]; then
    echo "SUCCESS: /mmwave/pointcloud topic found!"
    
    # Get info about the topic
    echo "\n==== Topic Info ===="
    ros2 topic info /mmwave/pointcloud
    
    # Try to get one message
    echo "\n==== Attempting to get one message ===="
    timeout 10 ros2 topic echo --once /mmwave/pointcloud 2>/dev/null || echo "No message received within timeout"
    
    # Monitor message rate
    echo "\n==== Monitoring message rate ===="
    timeout 15 ros2 topic hz /mmwave/pointcloud --window 5 || echo "No messages detected for rate calculation"
else
    echo "ERROR: /mmwave/pointcloud topic NOT found"
    
    # Debug: Check if bridge node is running
    echo "\n==== Debug: Checking if bridge node is running ===="
    ps aux | grep mmwave_ros2_bridge | grep -v grep
    
    # Print recent logs from the bridge
    echo "\n==== Debug: Recent bridge logs from journal ===="
    journalctl -n 30 | grep mmwave_ros2_bridge || echo "No bridge logs found in journal"
fi

# Launch RViz to visualize pointcloud data
echo "\n==== Launching RViz to visualize pointcloud data ===="
echo "Starting RViz with PointCloud2 display configured for /mmwave/pointcloud topic..."

# Create a temporary RViz config file
RVIZ_CONFIG="$SCRIPT_DIR/mmwave_rviz_config.rviz"
cat > "$RVIZ_CONFIG" << EOF
Visualized Topics:
  - /mmwave/pointcloud:
      Color: {R: 0.333, G: 0.7, B: 0.9}
      Decay Time: 0
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size: 0.009999999776482582
      Style: Points
Global Options:
  Fixed Frame: world
  Frame Rate: 30
EOF

# Launch RViz in the background
rviz2 -d "$RVIZ_CONFIG" &
RVIZ_PID=$!

# Function to clean up all processes
cleanup() {
    echo "\nCleaning up all processes..."
    
    # First, terminate main processes gracefully with SIGTERM
    echo "Step 1: Sending SIGTERM to known processes"
    for pid in $SIM_PID $BRIDGE_PID $RVIZ_PID; do
        if [ -n "$pid" ]; then
            ps -p $pid >/dev/null 2>&1 && {
                echo "Terminating PID $pid with SIGTERM"
                kill -TERM $pid 2>/dev/null || true
            }
        fi
    done
    
    # Short wait before more aggressive termination
    echo "Waiting briefly for graceful termination..."
    sleep 2
    
    # Now use SIGKILL for any processes that didn't terminate
    echo "Step 2: Sending SIGKILL to remaining processes"
    for pid in $SIM_PID $BRIDGE_PID $RVIZ_PID; do
        if [ -n "$pid" ]; then
            ps -p $pid >/dev/null 2>&1 && {
                echo "Force killing PID $pid with SIGKILL"
                kill -9 $pid 2>/dev/null || true
            }
        fi
    done
    
    # Kill all related ROS2 nodes by name
    echo "Step 3: Terminating all ROS2 nodes"
    ros2 node list 2>/dev/null | grep -E 'mmwave|olympus' | while read node; do
        echo "Killing ROS2 node: $node"
        ros2 lifecycle set $node shutdown 2>/dev/null || true
    done
    sleep 1
    
    # Kill all related processes forcefully by pattern
    echo "Step 4: Force killing all related processes by pattern"
    pkill -9 -f "ros2.*mmwave" 2>/dev/null || true
    pkill -9 -f "run_olympus\.py" 2>/dev/null || true
    pkill -9 -f "mmwave_ros2_bridge" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "_ros2_daemon" 2>/dev/null || true
    
    # Check if Gazebo is still running and try to terminate it properly
    gazebo_pids=$(pgrep -f "gz sim")
    if [ -n "$gazebo_pids" ]; then
        echo "Step 5: Terminating remaining Gazebo processes"
        echo "$gazebo_pids" | xargs kill -9 2>/dev/null || true
    fi
    
    # Clean up temporary files
    echo "Step 6: Cleaning up temporary files"
    rm -f "$RVIZ_CONFIG" 2>/dev/null || true
    rm -f "$SCRIPT_DIR/sim/gazebo/worlds/olympus.world" 2>/dev/null || true
    rm -f "bridge_output.log" 2>/dev/null || true
    
    # Verify cleanup
    remaining=$(pgrep -f "mmwave|olympus|gz sim" | wc -l)
    if [ "$remaining" -gt 0 ]; then
        echo "WARNING: $remaining related processes still running after cleanup"
        pgrep -fa "mmwave|olympus|gz sim"
    else
        echo "All processes successfully terminated"
    fi
    
    echo "Cleanup complete."
    exit 0
}

# Register cleanup function for various signals
trap cleanup INT TERM EXIT

# Wait for user to press Ctrl+C
echo "Press Ctrl+C to stop monitoring and exit"
wait $SIM_PID
