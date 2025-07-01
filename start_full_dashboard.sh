#!/bin/bash

echo "🚀 Starting Olympus Professional IoT Development Platform..."

# Function to cleanup background processes on exit
cleanup() {
    echo "🛑 Shutting down Olympus Platform..."
    
    # Kill all background processes
    if [ ! -z "$BACKEND_PID" ]; then
        kill $BACKEND_PID 2>/dev/null
        echo "✅ Backend bridge stopped"
    fi
    
    if [ ! -z "$REACT_PID" ]; then
        kill $REACT_PID 2>/dev/null
        echo "✅ React dashboard stopped"
    fi
    
    # Kill any remaining processes
    pkill -f "backend_bridge.py" 2>/dev/null
    pkill -f "react-scripts start" 2>/dev/null
    
    echo "🏁 Olympus Platform shutdown complete"
    exit 0
}

# Set trap to cleanup on script exit
trap cleanup EXIT INT TERM

echo "🔧 Starting Backend Bridge Server (Port 3001)..."
cd olympus-dashboard

# Start simple backend for better compatibility
python3 simple_backend.py > ../backend.log 2>&1 &
BACKEND_PID=$!

# Wait for backend to start and check if it's running
sleep 3

if ! kill -0 $BACKEND_PID 2>/dev/null; then
    echo "❌ Backend bridge failed to start. Check backend.log for details."
    exit 1
fi

echo "✅ Backend bridge started successfully"

echo "🎨 Starting React Dashboard (Port 3000)..."
npm start &
REACT_PID=$!

echo ""
echo "🌟 Olympus Platform Started Successfully!"
echo ""
echo "📋 Services:"
echo "   • React Dashboard:  http://localhost:3000"
echo "   • Backend Bridge:   http://localhost:3001"
echo ""
echo "✨ Features Available:"
echo "   • Professional Node-Based IoT Architecture"
echo "   • Real-time 3D Simulation with Three.js"
echo "   • 8 Sensor Types (mmWave, Temperature, Audio, Camera, PIR, Pressure, Light, Humidity)"
echo "   • Zeus/Mid-tier/Low-power Node Hierarchy"
echo "   • Drag-and-Drop Node/Sensor Placement"
echo "   • Real-time Sensor Data Simulation"
echo "   • System Performance Monitoring"
echo "   • Save/Load Configuration Files"
echo ""
echo "🔧 Controls:"
echo "   • Press Ctrl+C to stop all services"
echo "   • Add nodes and sensors via the control panel"
echo "   • Start simulation to begin data generation"
echo ""

# Wait for user to stop
echo "⏳ Platform running... Press Ctrl+C to stop"
wait