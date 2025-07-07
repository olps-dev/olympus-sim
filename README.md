# Olympus Simulation System (This readme is outdated)

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

# Full simulation with GUI, RViz, and web dashboard
./olympus full --gui --rviz --automation --dashboard

# Just the sensor simulation with dashboard
./olympus sensor --rviz --dashboard

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
- `--dashboard` - Launch web dashboard on http://localhost:5001
- `--list-modes` - Show available modes

## Usage Examples

### Development Workflow
```bash
# 1. Test sensor data flow with dashboard
./olympus sensor --rviz --dashboard

# 2. Test complete automation with monitoring
./olympus full --automation --dashboard

# 3. Debug with full visualization
./olympus full --gui --rviz --automation --dashboard
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
5. **Web Dashboard**: Real-time 3D visualization and metrics monitoring

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

## Web Dashboard

The web dashboard provides real-time monitoring and visualization at http://localhost:5001

### Features

1. **3D Visualization**:
   - Real-time point cloud display from mmWave sensors
   - Interactive 3D scene with OrbitControls
   - Synchronized human model positioning based on detections
   - Sensor field-of-view indicators

2. **Performance Metrics**:
   - End-to-end latency tracking
   - Sample count and statistical analysis
   - Real-time latency graphs
   - Min/Max/P99 latency metrics

3. **Live Monitoring**:
   - Active sensor status
   - Automation event stream
   - Battery level indicators
   - MQTT message flow visualization

### Accessing the Dashboard

```bash
# Start simulation with dashboard
./olympus full --automation --dashboard

# Access from browser
# - From WSL: http://localhost:5001
# - From Windows: http://<WSL-IP>:5001
```

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
├── olympus                     # Main launcher script
├── sim/
│   ├── gazebo/
│   │   ├── plugins/            # mmWave sensor plugin
│   │   ├── worlds/             # Gazebo world files
│   │   └── models/             # 3D models
│   └── ros2/
│       ├── multi_mmwave_mqtt_bridge.py  # ROS2 to MQTT bridge
│       └── config/             # RViz configurations
├── automation/
│   └── live_automation.py      # Live automation controller
├── dashboard/
│   ├── app_simple.py           # Flask web dashboard
│   ├── live_data_manager.py    # Real-time data management
│   └── static/                 # Frontend assets
├── docker/
│   ├── Dockerfile.*            # Container definitions
│   ├── docker-compose.yml      # Multi-container setup
│   └── README.md               # Docker documentation
├── metrics/
│   └── latency_battery_tracker.py  # Performance tracking
├── tests/
│   ├── test_mmwave_mqtt.py     # MQTT testing
│   └── test_automation_loop.py # End-to-end testing
├── tools/
│   ├── launch_olympus.py       # Main launcher backend
│   ├── manipulate_scene.py     # Scene manipulation
│   └── verify_setup.py         # Setup verification
└── scripts/
    ├── test_utils/             # Test utilities
    └── *.sh                    # Helper scripts
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
