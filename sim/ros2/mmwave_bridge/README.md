# mmWave ROS2 Bridge

This package provides a bridge between Gazebo's mmWave sensor data and ROS2 PointCloud2 topics. It's designed to be modular, maintainable, and robust.

## Package Structure

The package is organized into the following modules:

```
mmwave_bridge/
├── __init__.py             # Main package exports
├── version.py              # Version information
├── bridge_node.py          # Core ROS2 node implementation
├── gazebo/                 # Gazebo transport integration
│   ├── __init__.py
│   └── transport.py        # Gazebo transport handling
├── pointcloud/             # PointCloud utilities
│   ├── __init__.py
│   └── pointcloud_utils.py # PointCloud creation and processing
└── utils/                  # Utility modules
    ├── __init__.py
    ├── dummy_ros2.py       # Dummy ROS2 classes for testing
    └── logging_utils.py    # Logging configuration
```

## Usage

### Basic Usage

The bridge can be used directly from the main entry point:

```bash
# Source ROS2 first
source /opt/ros/humble/setup.bash

# Run the bridge
python3 mmwave_ros2_bridge.py
```

### Testing

A test script is provided to verify the bridge functionality:

```bash
# Test with ROS2 only (publishes test data)
python3 test_mmwave_bridge.py --test-mode ros2-only

# Test with Gazebo integration
python3 test_mmwave_bridge.py --test-mode with-gazebo
```

## Integration with Olympus Simulation

The mmWave bridge is designed to work with the Olympus simulation system. When the simulation is launched with Gazebo support, the bridge will automatically connect to the mmWave sensor in Gazebo and publish the data to ROS2.

## Dependencies

- ROS2 (Humble or later)
- Gazebo (Fortress or Harmonic)
- Python 3.8+

## Troubleshooting

If the bridge fails to connect to Gazebo, check:

1. Ensure Gazebo is running and the mmWave sensor plugin is loaded
2. Verify the topic names match between Gazebo and the bridge
3. Check the logs for any error messages
4. Try running in `ros2-only` mode to verify ROS2 functionality

## Development

To extend or modify the bridge:

1. The core ROS2 node is in `bridge_node.py`
2. Gazebo transport handling is isolated in `gazebo/transport.py`
3. PointCloud utilities are in `pointcloud/pointcloud_utils.py`
4. Logging and other utilities are in the `utils/` directory
