#!/bin/bash

echo "🐛 Starting Olympus Dashboard in Debug Mode..."

# Start backend
echo "🔧 Starting backend server..."
cd olympus-dashboard
python3 simple_backend.py &
BACKEND_PID=$!

echo "⏳ Waiting for backend to start..."
sleep 3

# Test backend
echo "🧪 Testing backend connection..."
curl -s http://localhost:3001/health > /dev/null
if [ $? -eq 0 ]; then
    echo "✅ Backend is running on port 3001"
else
    echo "❌ Backend failed to start"
    kill $BACKEND_PID 2>/dev/null
    exit 1
fi

# Test workflow
echo "🧪 Testing backend with sample data..."
cd ..
python3 test_complete_workflow.py

echo ""
echo "🎨 Backend is ready! Now you can:"
echo "   1. Start React dashboard: cd olympus-dashboard && npm start"
echo "   2. Or run full system: ./start_full_dashboard.sh"
echo ""
echo "🔧 Debug URLs:"
echo "   • Backend API: http://localhost:3001/simulation_state"
echo "   • Health Check: http://localhost:3001/health"
echo ""
echo "⏳ Backend running in background (PID: $BACKEND_PID)"
echo "   Press Ctrl+C to stop backend"

# Keep backend running
wait $BACKEND_PID