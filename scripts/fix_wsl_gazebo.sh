#!/bin/bash
# Fix Gazebo for WSL

echo "Setting up Gazebo for WSL..."

# Set software rendering
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export GALLIUM_DRIVER=llvmpipe

# Set Gazebo to use OGRE with OpenGL
export GZ_RENDERING_ENGINE=ogre
export OGRE_RTT_MODE=Copy

# Disable some advanced features that cause issues in WSL
export GZ_SIM_DISABLE_COMPONENT_NAMES=1
export GZ_GUI_RENDER_ENGINE=ogre

# Create a simple test
echo "Testing Gazebo..."
gz sim -s -r /home/aliza/olympus-sim/sim/gazebo/worlds/mmwave_test_wsl.sdf &
GZ_PID=$!

sleep 5

if ps -p $GZ_PID > /dev/null; then
    echo "Gazebo is running successfully!"
    kill $GZ_PID
else
    echo "Gazebo crashed. Checking logs..."
    echo "Try running with: gz sim -v 4 -s /home/aliza/olympus-sim/sim/gazebo/worlds/mmwave_test_wsl.sdf"
fi