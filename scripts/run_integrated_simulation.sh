#!/bin/bash
# Run the integrated Olympus simulation with ROS2 and Gazebo

# Set up environment
echo "Setting up ROS2 environment..."

# Check if ROS2 is installed
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "ROS2 Humble not found at /opt/ros/humble/setup.bash"
    
    # Try to find any ROS2 installation
    ROS_SETUP=$(find /opt/ros -name setup.bash 2>/dev/null | head -1)
    
    if [ -n "$ROS_SETUP" ]; then
        echo "Found ROS2 at $ROS_SETUP, using that instead"
        source $ROS_SETUP
    else
        echo "ERROR: No ROS2 installation found. Please install ROS2 or update the script with the correct path."
        exit 1
    fi
fi

# Check if MQTT broker is running
if ! pgrep -x "mosquitto" > /dev/null; then
    echo "Starting MQTT broker..."
    mosquitto -d
    sleep 2
fi

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Create a temporary directory for ROS2 launch
TEMP_DIR=$(mktemp -d)
echo "Using temporary directory: $TEMP_DIR"

# Copy ROS2 files
cp -r "$PROJECT_ROOT/sim/ros2"/* "$TEMP_DIR/"

# Create a ROS2 package structure
mkdir -p "$TEMP_DIR/olympus_sim"
cp "$TEMP_DIR"/*.py "$TEMP_DIR/olympus_sim/"
chmod +x "$TEMP_DIR/olympus_sim"/*.py

# Make Python scripts executable
chmod +x "$TEMP_DIR/olympus_sim/mqtt_ros2_bridge.py" 2>/dev/null || true
chmod +x "$TEMP_DIR/olympus_sim/time_sync_bridge.py" 2>/dev/null || true

# Launch ROS2 and Gazebo in the background
echo "Launching ROS2 and Gazebo..."
cd "$TEMP_DIR"

# Try to launch with ros2 command
if command -v ros2 &> /dev/null; then
    python3 "$TEMP_DIR/olympus_sim/mqtt_ros2_bridge.py" &
    ROS_PID=$!
    echo "Started ROS2 bridge with PID $ROS_PID"
    
    # For now, skip Gazebo launch as it's causing errors
    echo "Note: Gazebo launch is temporarily disabled until launch file issues are fixed"
else
    echo "ERROR: ros2 command not found. Make sure ROS2 is properly installed."
    exit 1
fi

# Wait for Gazebo to start
echo "Waiting for Gazebo to initialize..."
sleep 10

# Run the simulation
echo "Starting Olympus simulation with ROS2 integration..."
python3 "$PROJECT_ROOT/sim/python/core/main_simulation_ros2.py" --ros2 --duration 3600

# Clean up
echo "Cleaning up..."
kill $ROS_PID
rm -rf $TEMP_DIR

echo "Simulation complete!"
