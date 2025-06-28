# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Olympus Simulation System is a comprehensive multi-sensor simulation platform that integrates:
- MQTT-based hierarchical communication with Gazebo physics simulation
- ROS2 visualization with advanced mmWave radar sensor simulation
- Cross-platform support (native Linux and WSL2)
- Complete sensor-to-actuator automation pipeline

## Key Commands

### Build Commands
```bash
# Build the mmWave sensor plugin (required for simulation)
cd sim/gazebo/plugins
mkdir -p build && cd build
cmake ..
make -j4
cd ../../../../

# Install Python dependencies
pip3 install -r requirements.txt
```

### Launch Commands
```bash
# Primary launcher script with multiple modes
./olympus                          # Simple full simulation (headless)
./olympus full --gui --rviz --automation  # Full simulation with GUI
./olympus sensor --rviz           # Sensor simulation only
./olympus --list-modes            # Show available modes

# Available modes:
# - full: Complete simulation (Gazebo + ROS2 + MQTT + Automation)
# - gazebo: Gazebo simulation only
# - sensor: Sensor simulation with MQTT bridge
# - automation: Automation controller only
```

### Test Commands
```bash
# Python tests
pytest tests/

# Test automation loop (30 second test)
python3 tests/test_automation_loop.py 30

# Monitor MQTT sensor data
python3 tests/test_mmwave_mqtt.py

# Docker-based tests (if using Docker setup)
make test_all
make hardware_test
make test_integration
```

### Development Tools
```bash
# Verify setup
python3 tools/verify_setup.py

# Scene manipulation (requires running simulation)
python3 tools/manipulate_scene.py

# Monitor MQTT topics
mosquitto_sub -t sensor/#
mosquitto_sub -t actuators/#
```

## Architecture

### System Flow
```
Gazebo mmWave Sensor → ROS2 PointCloud2 → mmWave MQTT Bridge → MQTT Topics → Automation Controller → Actuator Commands
```

### Key Components

1. **Gazebo Plugin** (`sim/gazebo/plugins/`): C++ mmWave sensor simulation
   - Generates point cloud data based on scene geometry
   - Configurable FOV, range, and update rate
   - WSL-compatible rendering

2. **ROS2 Bridge** (`sim/ros2/`): 
   - `multi_mmwave_mqtt_bridge.py`: Converts ROS2 PointCloud2 to MQTT presence detection
   - Publishes to topics like `sensor/mmwave1/presence`, `sensor/mmwave1/human_present`

3. **Automation Controller** (`automation/automation_demo.py`):
   - Monitors sensor presence topics
   - Implements automation rules
   - Controls actuators via MQTT

4. **Unified Launcher** (`tools/launch_olympus.py`):
   - Manages all process lifecycles
   - Sets up environment variables automatically
   - Handles graceful shutdown

### MQTT Topic Structure
```
sensor/
├── mmwave1/
│   ├── presence       # Detailed JSON presence data
│   ├── human_present  # Boolean presence
│   └── status        # Periodic status updates
└── mmwave2/          # Second sensor (if configured)
    └── ...

actuators/
└── lamp_hall/
    └── command       # Actuator control commands
```

### Important Paths
- Plugin library: `sim/gazebo/plugins/build/libMmWaveSensorPlugin.so`
- World files: `sim/gazebo/worlds/`
- RViz config: `sim/ros2/config/olympus_rviz.rviz`
- Test data: `tests/`

## Development Workflow

1. **Making Plugin Changes**:
   - Edit C++ files in `sim/gazebo/plugins/sensor_config/`
   - Rebuild: `cd sim/gazebo/plugins/build && make -j4`
   - Test with: `./olympus sensor --gui --rviz`

2. **Testing Automation Logic**:
   - Edit `automation/automation_demo.py`
   - Test with: `./olympus full --automation`
   - Monitor: `mosquitto_sub -t sensor/# -t actuators/#`

3. **Adding New Sensors**:
   - Add sensor to world file (`sim/gazebo/worlds/mmwave_test.sdf`)
   - Update MQTT bridge to handle new topics
   - Add visualization to RViz config if needed

## Environment Setup

The launcher automatically configures:
- `GZ_SIM_RESOURCE_PATH`: Gazebo model/world paths
- `GZ_SIM_PLUGIN_PATH`: Plugin library paths
- `LD_LIBRARY_PATH`: Shared libraries
- `ROS_DOMAIN_ID`: ROS2 domain isolation
- ROS2 environment sourcing

## Common Issues

1. **Plugin not loading**: Ensure plugin is built (`make` in plugin build directory)
2. **ROS2 issues**: Check that ROS2 Jazzy is installed and sourced
3. **MQTT connection**: Ensure mosquitto is running (`sudo systemctl start mosquitto`)
4. **WSL graphics**: May need X11 server (VcXsrv) and `export DISPLAY=:0`

## Testing Strategy

- Unit tests: Individual component testing with pytest
- Integration tests: End-to-end automation loop testing
- Manual testing: Scene manipulation tool for sensor response verification
- Performance testing: Monitor latency metrics in automation events