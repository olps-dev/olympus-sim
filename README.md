# Olympus Simulation System

A comprehensive multi-sensor simulation platform that integrates MQTT-based hierarchical communication with Gazebo physics simulation and ROS2 visualization. The system features advanced mmWave radar sensor simulation with WSL compatibility and real-time point cloud visualization.

## Overview

The simulation system provides:

- **Multi-Protocol Communication**: MQTT for internal coordination, ROS2 for visualization
- **Advanced mmWave Radar Simulation**: Physics-based radar point cloud generation
- **Cross-Platform Compatibility**: Native Linux and WSL2 support with automatic detection
- **Real-Time Visualization**: Gazebo 3D simulation and RViz2 sensor data visualization
- **Interactive Scene Manipulation**: Programmatic object movement and sensor testing
- **Automation Integration**: Complete sensor-to-actuator automation pipeline

## Quick Start

The Olympus simulation has a single, clean entry point:

```bash
# Simple full simulation (headless)
./olympus

# Full simulation with GUI and RViz
./olympus full --gui --rviz --automation

# Just the sensor simulation
./olympus sensor --rviz

# List all available modes
./olympus --list-modes
```

## Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04+ (native) or WSL2 with Ubuntu
- **ROS2**: Jazzy Jalopy
- **Gazebo**: Gazebo Sim 8 (Harmonic)
- **Python**: 3.10+
- **Memory**: 4GB+ RAM recommended
- **Graphics**: OpenGL 3.3+ (software rendering supported for WSL)

### Dependencies Installation

```bash
# ROS2 Jazzy
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-jazzy-desktop

# Gazebo Sim 8 (Harmonic)
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic

# ROS2-Gazebo Bridge
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim

# Additional Dependencies
sudo apt install python3-pip mosquitto mosquitto-clients
sudo apt install python3-paho-mqtt

# Python Dependencies
pip3 install numpy scipy
```

### Build the mmWave Plugin

```bash
cd sim/gazebo/plugins
mkdir -p build && cd build
cmake ..
make -j4
```

## Simulation Modes

| Mode | Description | Components |
|------|-------------|------------|
| `full` | Complete simulation | Gazebo + ROS2 + MQTT + Automation |
| `gazebo` | Gazebo simulation only | Gazebo + ROS2 bridge |
| `sensor` | Sensor simulation | Gazebo + ROS2 + MQTT bridge |
| `automation` | Automation only | MQTT automation controller |

## Command Line Options

- `--gui` - Launch Gazebo with GUI (default: headless)
- `--rviz` - Launch RViz2 for visualization
- `--automation` - Launch automation controller
- `--list-modes` - Show available modes

## Usage Examples

### Development Workflow
```bash
# 1. Test sensor data flow
./olympus sensor --rviz

# 2. Test complete automation
./olympus full --automation

# 3. Debug with GUI
./olympus full --gui --rviz --automation
```

### Testing Automation
```bash
# Terminal 1: Run simulation
./olympus full --automation

# Terminal 2: Monitor MQTT
mosquitto_sub -t sensor/#

# Terminal 3: Test automation loop
python3 tests/test_automation_loop.py 30
```

## Architecture

The system implements a complete sensor-to-actuator pipeline:

```
Gazebo mmWave Sensor → ROS2 PointCloud2 → mmWave MQTT Bridge → MQTT Topics → Automation Controller → Actuator Commands
```

### Components

1. **Gazebo Simulation**: Physics-based 3D environment with mmWave sensor
2. **ROS2 Bridge**: Converts Gazebo messages to ROS2 topics
3. **mmWave MQTT Bridge**: Processes point cloud data for presence detection
4. **Automation Controller**: Implements automation logic based on sensor data

### Data Flow

1. **Sensor Data Generation**: mmWave plugin generates point cloud data in Gazebo
2. **ROS2 Integration**: Data published to `/mmwave/points` topic
3. **MQTT Bridge**: Converts point cloud to presence detection
4. **Automation Logic**: Triggers actuator commands based on presence
5. **Actuator Control**: MQTT commands sent to control devices

## Automation Features

### mmWave MQTT Bridge

**Purpose**: Converts ROS2 PointCloud2 data to MQTT presence detection

**Configuration Parameters**:
- `detection_threshold`: Minimum distance for detection (default: 0.1m)
- `max_detection_range`: Maximum detection range (default: 10.0m)
- `min_points_for_detection`: Minimum points to consider detection (default: 5)

**MQTT Topics Published**:
- `sensor/mmwave1/presence` - Detailed JSON presence data
- `sensor/mmwave1/human_present` - Simple boolean presence
- `sensor/mmwave1/status` - Periodic status updates

### Automation Controller

**Features**:
- Monitors `sensor/+/presence` topics
- Triggers `actuators/lamp_hall/on` when presence detected
- Publishes automation events with latency metrics
- Configurable automation rules with cooldown periods

## Testing and Validation

### Test Scripts

Located in `tests/` directory:

- `test_mmwave_mqtt.py` - Monitor MQTT sensor topics
- `test_automation_loop.py` - End-to-end automation testing with latency metrics

### Running Tests

```bash
# Monitor MQTT sensor data
python3 tests/test_mmwave_mqtt.py

# Test complete automation loop (30 second test)
python3 tests/test_automation_loop.py 30

# Monitor all sensor topics
mosquitto_sub -t sensor/#
```

## Tools and Utilities

Located in `tools/` directory:

### Setup Verification

Check that all components are properly organized:

```bash
python3 tools/verify_setup.py
```

### Scene Manipulation

Interactive tool for testing sensor responses:

```bash
# Start simulation first
./olympus full --gui

# In another terminal
python3 tools/manipulate_scene.py
```

**Commands**:
- `list` - Show all entities in the scene
- `move <entity> <x> <y> <z>` - Move an object to coordinates
- `spawn <model> <x> <y> <z>` - Add new objects
- `delete <entity>` - Remove objects
- `help` - Show all commands
- `quit` - Exit the tool

## Project Structure

```
olympus-sim/
├── ./olympus                    # Main launcher script
├── sim/
│   ├── gazebo/
│   │   ├── plugins/            # mmWave sensor plugin
│   │   ├── worlds/             # Gazebo world files
│   │   └── models/             # 3D models
│   └── ros2/
│       └── mmwave_mqtt_bridge.py  # ROS2 to MQTT bridge
├── automation/
│   └── automation_demo.py      # Automation controller
├── tests/
│   ├── test_mmwave_mqtt.py     # MQTT testing
│   └── test_automation_loop.py # End-to-end testing
├── tools/
│   ├── launch_olympus.py       # Main launcher
│   ├── manipulate_scene.py     # Scene manipulation
│   └── verify_setup.py         # Setup verification
└── legacy_launch/              # Old launch scripts
```

## Configuration

### mmWave Sensor Configuration

The mmWave sensor can be configured in the world file:

```xml
<plugin filename="libMmWaveSensorPlugin.so" name="olympus_sim::MmWaveSensorPlugin">
  <topic>/mmwave/points</topic>
  <update_rate>10</update_rate>
  <horizontal_fov>1.5708</horizontal_fov>  <!-- 90 degrees -->
  <vertical_fov>0.5236</vertical_fov>      <!-- 30 degrees -->
  <range_min>0.1</range_min>
  <range_max>10.0</range_max>
  <range_resolution>0.01</range_resolution>
</plugin>
```

### Environment Variables

The launcher automatically sets up required environment variables:

- `GZ_SIM_RESOURCE_PATH` - Gazebo model and world paths
- `GZ_SIM_PLUGIN_PATH` - Plugin library path
- `LD_LIBRARY_PATH` - Shared library paths
- `ROS_DOMAIN_ID` - ROS2 domain isolation

## Troubleshooting

### Common Issues

1. **Plugin Loading Errors**
   - Ensure plugin is built: `cd sim/gazebo/plugins/build && make`
   - Check plugin path in environment variables

2. **ROS2 Bridge Issues**
   - Source ROS2 environment: `source /opt/ros/jazzy/setup.bash`
   - Check ROS2 installation: `ros2 --version`

3. **MQTT Connection Issues**
   - Start mosquitto broker: `sudo systemctl start mosquitto`
   - Check broker status: `mosquitto_pub -t test -m "hello"`

4. **WSL Graphics Issues**
   - Install VcXsrv or similar X11 server
   - Set DISPLAY environment variable

### Debug Mode

Run with verbose logging:

```bash
# Enable Gazebo debug output
GZ_VERBOSE=1 ./olympus full --gui

# Monitor all processes
./olympus full --gui --rviz --automation
```

## Performance Optimization

### For WSL Users

```bash
# Enable GPU acceleration (if available)
export LIBGL_ALWAYS_INDIRECT=1

# Reduce graphics quality for better performance
export GZ_SIM_RENDER_ENGINE=ogre
```

### Memory Usage

- Minimum 4GB RAM recommended
- Close unnecessary applications during simulation
- Use headless mode for automated testing

## Contributing

1. Follow the existing code structure
2. Test changes with all simulation modes
3. Update documentation for new features
4. Ensure WSL compatibility

## License

This project is part of the Olympus simulation system.
