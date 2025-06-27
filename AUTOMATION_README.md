# Olympus Simulation - Automation Integration

This document describes the new automation features that bridge the mmWave sensor data from ROS2 to MQTT for home automation scenarios.

## Overview

The automation system implements **Step 1** and **Step 2** from the roadmap:

1. **Wire mmWave frames into the MQTT world** - ROS→MQTT bridge
2. **Close the automation loop** - Presence detection triggers actuator control

## Architecture

```
Gazebo mmWave Sensor → ROS2 PointCloud2 → mmWave MQTT Bridge → MQTT Topics → Automation Controller → Actuator Commands
```

## New Components

### 1. mmWave MQTT Bridge (`sim/ros2/mmwave_mqtt_bridge.py`)

**Purpose**: Converts ROS2 PointCloud2 data to MQTT presence detection

**Features**:
- Subscribes to `/mmwave/points` (PointCloud2)
- Analyzes point cloud for human presence detection
- Publishes to MQTT topics:
  - `sensor/mmwave1/presence` (detailed JSON)
  - `sensor/mmwave1/human_present` (simple boolean)
  - `sensor/mmwave1/status` (periodic status)

**Configuration Parameters**:
- `detection_threshold`: Minimum distance for detection (default: 0.1m)
- `max_detection_range`: Maximum detection range (default: 10.0m)
- `min_points_for_detection`: Minimum points to consider detection (default: 5)

### 2. Automation Controller (`automation_demo.py`)

**Purpose**: Demonstrates end-to-end automation logic

**Features**:
- Monitors `sensor/+/presence` topics
- Triggers `actuators/lamp_hall/on` when presence detected
- Publishes automation events with latency metrics
- Configurable automation rules

### 3. Test Scripts

- **`test_mmwave_mqtt.py`**: Monitor MQTT topics for sensor data
- **`test_automation_loop.py`**: Complete end-to-end automation testing
- **`automation_demo.py`**: Basic automation controller

## Quick Start

### 1. Run the Complete Simulation

```bash
# Terminal 1: Start the simulation (includes mmWave MQTT bridge)
./run_simulation.sh

# Terminal 2: Start automation controller
python3 automation_demo.py

# Terminal 3: Monitor the automation loop
python3 test_automation_loop.py 30  # Run for 30 seconds
```

### 2. Test Individual Components

```bash
# Test MQTT bridge only
python3 test_mmwave_mqtt.py

# Test automation logic only
python3 automation_demo.py
```

### 3. Trigger Detection

Use the scene manipulation script to move objects and trigger detection:

```bash
python3 manipulate_scene.py
```

## Expected MQTT Topics

### Sensor Topics (Published by mmWave Bridge)
- `sensor/mmwave1/presence` - Detailed presence data (JSON)
- `sensor/mmwave1/human_present` - Simple boolean (true/false)
- `sensor/mmwave1/status` - Bridge status updates

### Actuator Topics (Published by Automation Controller)
- `actuators/lamp_hall/on` - Turn lamp on (JSON command)
- `actuators/lamp_hall/off` - Turn lamp off (JSON command)
- `actuators/lamp_hall/state` - Simple state (on/off)

### Automation Topics
- `automation/events` - Automation events with latency metrics

## Success Criteria

✅ **Step 1 Complete**: ROS→MQTT bridge working
- `mosquitto_sub -t sensor/#` shows presence data
- Point cloud data converted to boolean presence detection

✅ **Step 2 Complete**: Automation loop closed
- Presence detection triggers lamp control within <300ms
- End-to-end latency measured and logged

## Testing Commands

```bash
# Monitor all sensor topics
mosquitto_sub -t sensor/#

# Monitor all actuator commands
mosquitto_sub -t actuators/#

# Monitor automation events
mosquitto_sub -t automation/#

# Test complete loop with metrics
python3 test_automation_loop.py 60
```

## Integration with Launch System

The mmWave MQTT bridge is automatically started with the main simulation:

```bash
./run_simulation.sh  # Includes mmWave bridge
```

The bridge is configured in `sim/ros2/launch/olympus_gazebo.launch.py` and will start automatically when the simulation launches.

## Next Steps (Roadmap Steps 3-7)

1. **Add second sensor node** - Duplicate sensor with different ID
2. **Latency & battery instrumentation** - Add metrics collection
3. **Network realism layer** - Add packet loss/delay simulation
4. **GitHub Action / CI** - Automated testing
5. **Minimal GUI dashboard** - Web interface for monitoring

## Troubleshooting

### No MQTT Messages
- Check if MQTT broker is running: `mosquitto -v`
- Verify bridge is connected: Check logs in simulation terminal

### No Presence Detection
- Ensure objects are within detection range (0.1m - 10.0m)
- Check point cloud data: `ros2 topic echo /mmwave/points`
- Adjust detection parameters in launch file

### High Latency
- Check system load
- Verify MQTT broker performance
- Review detection algorithm parameters
