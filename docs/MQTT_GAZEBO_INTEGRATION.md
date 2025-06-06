# Olympus Simulation - MQTT & Gazebo Integration Guide

This guide describes how to integrate the Python-based Olympus sensor simulation with Gazebo visualization using MQTT.

## Overview

The integration works by:

1. Replacing the simulated MQTT system with a real MQTT broker (Mosquitto)
2. Creating Gazebo plugins that connect to the same MQTT broker
3. Updating the sensor models in Gazebo to display data based on MQTT messages

## Dependencies

### Python Dependencies
```bash
pip install -r requirements_mqtt.txt
```

### System Dependencies
```bash
sudo apt-get update
sudo apt-get install -y mosquitto mosquitto-clients libmosquitto-dev nlohmann-json3-dev
```

### Gazebo Dependencies
You need Gazebo Garden or newer (`gz`) with development libraries:

```bash
sudo apt-get install gz-garden
sudo apt-get install libgz-cmake3-dev libgz-plugin2-dev libgz-sim7-dev libgz-rendering7-dev libgz-transport12-dev libgz-msgs9-dev
```

## Building the Gazebo Plugin

The MQTT Gazebo plugin needs to be built before it can be used:

```bash
cd sim/gazebo/plugins
mkdir build && cd build
cmake ..
make
sudo make install
```

## Components

### Modified Python Files

1. `mqtt_real.py` - Real MQTT client implementation using Paho-MQTT
2. `main_simulation_mqtt.py` - Updated simulation using real MQTT
3. Modified `olympus_nodes.py` - Updated to work with real MQTT
4. Modified `hardware_sensor_bridge.py` - Updated to work with real MQTT

### New Gazebo Files

1. `OlympusMQTTPlugin.hh` and `OlympusMQTTPlugin.cc` - Gazebo plugin for MQTT visualization
2. `sensor_model.sdf` - SDF model for MQTT-enabled sensors
3. `CMakeLists.txt` - Build configuration for the Gazebo plugin

## Running the Integrated Simulation

### Option 1: Using the Launch Script

The easiest way to run the integrated simulation is using the provided launch script:

```bash
chmod +x launch_integrated_simulation.sh
./launch_integrated_simulation.sh
```

This script will:
1. Start the Mosquitto MQTT broker
2. Launch Gazebo with the apartment world and sensor models
3. Start the Python simulation

### Option 2: Manual Start

If you prefer to start each component separately:

1. Start the MQTT broker:
   ```bash
   mosquitto -v
   ```

2. Launch Gazebo (in a new terminal):
   ```bash
   # Make sure the sensor_model is in ~/.gz/models/
   cp -r sim/gazebo/worlds/sensor_model.sdf ~/.gz/models/sensor_model/model.sdf
   gz sim -r sim/gazebo/worlds/apartment.sdf
   ```

3. Run the Python simulation (in a new terminal):
   ```bash
   python main_simulation_mqtt.py
   ```

## MQTT Topic Structure

The system uses the following MQTT topic structure:

- Temperature data: `olympus/sensor_data/<sensor_id>/bme680`
- Battery data: `olympus/sensor_data/<sensor_id>/battery`
- Display status: `olympus/sensor_data/<sensor_id>/display_status`
- Mid-tier processed data: `olympus/processed_mid_tier/<mid_tier_id>`

## Visualization Types

The Gazebo plugin supports different visualization types:

1. **Text Display**: Shows text values (e.g., temperature readings)
2. **Color Indicator**: Changes color based on values (e.g., battery level)

You can modify which sensor data is visualized by editing the `sensor_model.sdf` file.

## Troubleshooting

### MQTT Connection Issues

If you experience MQTT connection problems:

1. Check if Mosquitto is running: `ps aux | grep mosquitto`
2. Test MQTT communication: `mosquitto_sub -t olympus/# -v`

### Gazebo Plugin Issues

If the Gazebo plugin isn't working properly:

1. Check plugin loading: `export GZ_VERBOSE=1` before starting Gazebo
2. Verify plugin installation: `ls -l /usr/lib/gz-sim-7/plugins/`

### Python Simulation Issues

If the Python simulation isn't connecting to MQTT:

1. Check for Paho-MQTT installation: `pip list | grep paho-mqtt`
2. Try with explicit host/port: `python main_simulation_mqtt.py --host localhost --port 1883`

## Customization

### Adding New Sensors

1. Update `main_simulation_mqtt.py` to create additional sensor nodes
2. Add new sensor models in the Gazebo world file
3. Ensure the sensor IDs match between Python and Gazebo

### Custom Visualizations

To create new visualization types:

1. Extend the `OlympusMQTTPlugin.cc` with new visualization methods
2. Update the `sensor_model.sdf` to use the new visualization type
