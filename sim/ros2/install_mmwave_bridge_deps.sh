#!/bin/bash
# Installation script for mmWave ROS2 Bridge dependencies
# This script installs the necessary dependencies for the mmWave ROS2 Bridge

set -e  # Exit on error

echo "Installing mmWave ROS2 Bridge dependencies..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root or with sudo"
  exit 1
fi

# Check if ROS2 is installed
if [ -z "$ROS_DISTRO" ]; then
  echo "ROS2 environment not detected."
  echo "Please install ROS2 and source it before running this script."
  echo "See https://docs.ros.org/en/humble/Installation.html for installation instructions."
  exit 1
fi

echo "Detected ROS2 distribution: $ROS_DISTRO"

# Install ROS2 dependencies
echo "Installing ROS2 dependencies..."
apt-get update
apt-get install -y \
  python3-rosdep \
  python3-colcon-common-extensions \
  ros-$ROS_DISTRO-sensor-msgs \
  ros-$ROS_DISTRO-std-msgs \
  python3-numpy

# Install Gazebo dependencies based on availability
echo "Checking for Gazebo packages..."

# Try to install Gazebo Fortress packages
if apt-cache search --names-only "python3-gz-transport14" | grep -q "python3-gz-transport14"; then
  echo "Installing Gazebo Fortress (14) packages..."
  apt-get install -y \
    python3-gz-transport14 \
    python3-gz-msgs11
elif apt-cache search --names-only "python3-gz-transport13" | grep -q "python3-gz-transport13"; then
  echo "Installing Gazebo Harmonic (13) packages..."
  apt-get install -y \
    python3-gz-transport13 \
    python3-gz-msgs10
else
  echo "Warning: Could not find Gazebo Python packages."
  echo "The bridge will run in test mode only (no Gazebo integration)."
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install numpy

# Install the mmWave bridge package
echo "Installing mmWave bridge package..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/mmwave_bridge"
pip3 install -e .

echo "Installation complete!"
echo ""
echo "To use the mmWave bridge:"
echo "1. Source your ROS2 installation: source /opt/ros/$ROS_DISTRO/setup.bash"
echo "2. Run the bridge: python3 $SCRIPT_DIR/launch_mmwave_bridge.py"
echo ""
echo "For testing without Gazebo:"
echo "  python3 $SCRIPT_DIR/test_mmwave_bridge.py --test-mode ros2-only"
