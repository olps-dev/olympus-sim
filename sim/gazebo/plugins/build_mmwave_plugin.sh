#!/bin/bash

# Build script for MmWaveSensorPlugin
# Navigate to the plugins directory (where this script is located)
cd "$(dirname "$0")"

# Remove previous build artifacts 
rm -rf build
mkdir build
cd build

# Configure and build the plugin
cmake ..
make

# Display successful build message if make succeeded
if [ $? -eq 0 ]; then
  echo "Build successful!"
  echo "To use the plugin, make sure to set the GZ_SIM_SYSTEM_PLUGIN_PATH environment variable:"
  echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=\$GZ_SIM_SYSTEM_PLUGIN_PATH:$(pwd)"
else
  echo "Build failed!"
fi
