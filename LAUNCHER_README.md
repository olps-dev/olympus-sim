# Olympus Simulation - Unified Launcher

## Quick Start

The Olympus simulation now has a single, clean entry point:

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

## Available Modes

| Mode | Description | Components |
|------|-------------|------------|
| `full` | Complete simulation | Gazebo + ROS2 + MQTT + Automation |
| `gazebo` | Gazebo simulation only | Gazebo + ROS2 bridge |
| `sensor` | Sensor simulation | Gazebo + ROS2 + MQTT bridge |
| `automation` | Automation only | MQTT automation controller |

## Options

- `--gui` - Launch Gazebo with GUI (default: headless)
- `--rviz` - Launch RViz2 for visualization
- `--automation` - Launch automation controller
- `--list-modes` - Show available modes

## Examples

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

## What Happened to Old Launch Scripts?

All previous launch scripts have been moved to `legacy_launch/` folder:
- `legacy_launch/run_olympus.py` - Original Python launcher
- `legacy_launch/run_simulation.sh` - Original shell script
- Other legacy scripts remain in `scripts/` folder

The new unified launcher (`launch_olympus.py` + `./olympus` wrapper) replaces all of these with a single, clean interface.

## Architecture

```
./olympus (bash wrapper)
    ↓
launch_olympus.py (Python launcher)
    ↓
┌─────────────────────────────────────┐
│ Gazebo Sim + mmWave Plugin          │
│ ↓                                   │
│ ROS2 Bridge (gz → ros2)             │
│ ↓                                   │
│ mmWave MQTT Bridge (ros2 → mqtt)    │
│ ↓                                   │
│ Automation Controller (mqtt logic)  │
│ ↓                                   │
│ Actuator Commands (mqtt output)     │
└─────────────────────────────────────┘
```

## Process Management

The launcher properly manages all processes:
- Clean startup sequence with proper delays
- Environment variable setup
- Graceful shutdown on Ctrl+C
- Process monitoring and error reporting
- Automatic cleanup of zombie processes

## Troubleshooting

### "ROS2 not found"
```bash
source /opt/ros/jazzy/setup.bash
./olympus
```

### "Gazebo models not found"
The launcher automatically sets `GZ_SIM_RESOURCE_PATH` - no manual setup needed.

### "MQTT broker connection failed"
Start mosquitto broker:
```bash
mosquitto -v
```

### Check what's running
```bash
ps aux | grep -E "(gz|ros2|python3.*olympus)"
```
