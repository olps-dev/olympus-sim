#\!/bin/bash
echo "Starting Three.js Simulation..."
echo ""

# Check if bridge is already running
if curl -s http://localhost:5555 >/dev/null 2>&1; then
    echo "Bridge already running at http://localhost:5555"
else
    echo "Starting Three.js ROS2 bridge..."
    cd "$(dirname "$0")"
    python3 sim/threejs/threejs_ros2_bridge.py &
    BRIDGE_PID=$\!
    
    echo "Waiting for bridge to start..."
    sleep 3
    
    if curl -s http://localhost:5555 >/dev/null 2>&1; then
        echo "Bridge started successfully (PID: $BRIDGE_PID)"
    else
        echo "Failed to start bridge"
        kill $BRIDGE_PID 2>/dev/null
        exit 1
    fi
fi

echo ""
echo "SIMULATION READY\!"
echo "=================="
echo ""
echo "Available interfaces:"
echo "1. Main simulation: http://localhost:5555"
echo "   - 3D scene with interactive objects"
echo "   - Left click: drag objects"
echo "   - Right click: rotate camera"
echo "   - Mouse wheel: zoom"
echo ""
echo "2. Point cloud viewer: http://localhost:5555/pointcloud-viewer"
echo "   - RViz-like real-time point cloud visualization"
echo "   - Professional sensor data display"
echo "   - Performance metrics and FPS"
echo ""
echo "Monitor MQTT data:"
echo "  mosquitto_sub -t sensor/#  < /dev/null |  grep human_present"
echo ""
echo "Stop simulation:"
echo "  ./kill_olympus.sh"
