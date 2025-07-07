#!/bin/bash
echo "🌐 Opening Three.js Simulation in Browser..."
echo ""
echo "This will automatically:"
echo "1. Start the Three.js ROS2 bridge on http://localhost:5555"
echo "2. Open your default browser to run the simulation"
echo "3. Show you real-time sensor data"
echo ""

# Function to open URL in browser
open_browser() {
    local url=$1
    
    # Try different browser commands based on platform
    if command -v xdg-open > /dev/null; then
        # Linux
        xdg-open "$url"
    elif command -v open > /dev/null; then
        # macOS
        open "$url"
    elif command -v cmd.exe > /dev/null; then
        # WSL
        cmd.exe /c start "$url"
    else
        echo "❌ Could not detect browser. Please manually open: $url"
        return 1
    fi
    
    echo "✅ Browser opened to $url"
}

# Check if bridge is already running
if curl -s http://localhost:5555 >/dev/null 2>&1; then
    echo "✅ Three.js bridge already running"
    echo "🌐 Opening browser..."
    open_browser "http://localhost:5555"
else
    echo "🚀 Starting Three.js simulation..."
    echo ""
    echo "The simulation will run in the background."
    echo "Use ./kill_olympus.sh to stop it when done."
    echo ""
    
    # Start the bridge in background
    cd "$(dirname "$0")"
    python3 sim/threejs/threejs_ros2_bridge.py &
    BRIDGE_PID=$!
    
    # Wait a moment for it to start
    echo "Waiting for bridge to start..."
    sleep 3
    
    # Check if it started successfully
    if curl -s http://localhost:5555 >/dev/null 2>&1; then
        echo "✅ Bridge started successfully (PID: $BRIDGE_PID)"
        echo "🌐 Opening browser..."
        open_browser "http://localhost:5555"
        echo ""
        echo "📊 Monitor sensor data with:"
        echo "   mosquitto_sub -t sensor/# | grep human_present"
        echo ""
        echo "🛑 Stop simulation with:"
        echo "   ./kill_olympus.sh"
    else
        echo "❌ Failed to start bridge"
        kill $BRIDGE_PID 2>/dev/null
        exit 1
    fi
fi