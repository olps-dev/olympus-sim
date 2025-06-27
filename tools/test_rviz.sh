#!/bin/bash

echo "Testing RViz2 GUI launch..."

# Set up environment
export DISPLAY=:0.0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/tmp/runtime-$(id -u)
export PULSE_RUNTIME_PATH=/tmp/pulse-runtime-$(id -u)

# Create runtime directories
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
mkdir -p "$PULSE_RUNTIME_PATH"
chmod 700 "$PULSE_RUNTIME_PATH"

# Source ROS2
source /opt/ros/jazzy/setup.bash

echo "Environment:"
echo "  DISPLAY=$DISPLAY"
echo "  WAYLAND_DISPLAY=$WAYLAND_DISPLAY"
echo "  XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"

echo "Testing simple GUI app first..."
timeout 3s xclock &
XCLOCK_PID=$!
sleep 1
if kill -0 $XCLOCK_PID 2>/dev/null; then
    echo "✅ xclock started successfully"
    kill $XCLOCK_PID 2>/dev/null
else
    echo "❌ xclock failed to start"
fi

echo "Now testing RViz2..."
cd /home/aliza/olympus-sim
ros2 run rviz2 rviz2 -d sim/ros2/config/olympus_rviz.rviz &
RVIZ_PID=$!

echo "RViz2 PID: $RVIZ_PID"
sleep 3

if kill -0 $RVIZ_PID 2>/dev/null; then
    echo "✅ RViz2 is running (PID: $RVIZ_PID)"
    echo "If you don't see the window, there may be a display issue"
    echo "RViz2 will continue running in background..."
else
    echo "❌ RViz2 failed to start or crashed"
fi
