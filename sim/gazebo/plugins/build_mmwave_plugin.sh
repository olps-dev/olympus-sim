#!/bin/bash

# Build and Run script for MmWaveSensorPlugin
# Navigate to the plugins directory (where this script is located)
cd "$(dirname "$0")"

# Function to display usage information
usage() {
  echo "Usage: $0 [options]"
  echo "Options:"
  echo "  -b, --build-only    Only build the plugin, don't install or run"
  echo "  -i, --install       Build and install the plugin to system directory"
  echo "  -r, --run           Build, install, and run the simulation"
  echo "  -h, --help          Show this help message"
}

# Default mode is build-only
BUILD_ONLY=true
INSTALL=false
RUN=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  key="$1"
  case $key in
    -b|--build-only)
      BUILD_ONLY=true
      INSTALL=false
      RUN=false
      shift
      ;;
    -i|--install)
      BUILD_ONLY=false
      INSTALL=true
      RUN=false
      shift
      ;;
    -r|--run)
      BUILD_ONLY=false
      INSTALL=true
      RUN=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      usage
      exit 1
      ;;
  esac
done

# Remove previous build artifacts 
rm -rf build
mkdir build
cd build

# Configure and build the plugin
echo "Building MmWaveSensorPlugin..."
cmake ..
make

# Check if build succeeded
if [ $? -ne 0 ]; then
  echo "Build failed!"
  exit 1
fi

echo "Build successful!"

# If only building, show the environment variable info and exit
if [ "$BUILD_ONLY" = true ]; then
  echo "To use the plugin without installing, set the GZ_SIM_SYSTEM_PLUGIN_PATH environment variable:"
  echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=\$GZ_SIM_SYSTEM_PLUGIN_PATH:$(pwd)"
  exit 0
fi

# Install the plugin to the system directory if requested
if [ "$INSTALL" = true ]; then
  echo "Installing plugin to system directory..."
  sudo cp libMmWaveSensorPlugin.so /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/
  sudo chmod 755 /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/libMmWaveSensorPlugin.so
  echo "Plugin installed successfully"
fi

# Run the simulation if requested
if [ "$RUN" = true ]; then
  cd ../../..
  echo "Running Gazebo with mmWave test world..."
  echo "To view point cloud data, run in a separate terminal: gz topic -e -t /mmwave"
  gz sim -v 4 /home/aliza/olympus-sim/sim/gazebo/worlds/mmwave_test.sdf
fi

