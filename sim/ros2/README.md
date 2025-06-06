# Olympus Simulation - ROS2 and Gazebo Integration

This directory contains the ROS2 integration components for the Olympus simulation. The integration allows the existing MQTT-based simulation to work with ROS2 and Gazebo, providing 3D visualization capabilities.

## Components

- `mqtt_ros2_bridge.py`: Bridges between MQTT topics and ROS2 topics
- `time_sync_bridge.py`: Synchronizes simulation time across different components
- `launch_olympus_simulation.py`: ROS2 launch file for starting the integrated simulation

## Prerequisites

- ROS2 (Humble or later)
- Gazebo (Garden or later)
- MQTT Broker (Mosquitto)
- Python 3.8+

## Running the Integrated Simulation

There are several ways to run the integrated simulation:

### 1. Using the Makefile

The simplest way is to use the provided Makefile targets:

```bash
# Run the simulation with ROS2 integration (no Gazebo)
make ros2_sim

# Run the full simulation with ROS2 and Gazebo
make ros2_gazebo_sim
```

### 2. Using the Launch Script

You can also run the integration script directly:

```bash
./scripts/run_integrated_simulation.sh
```

### 3. Manual Launch

For more control, you can start each component separately:

1. Start the MQTT broker:
   ```bash
   mosquitto -v
   ```

2. Start the ROS2 bridge:
   ```bash
   source /opt/ros/humble/setup.bash
   python3 sim/ros2/mqtt_ros2_bridge.py
   ```

3. Launch Gazebo with the Olympus world:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 launch sim/ros2/launch_olympus_simulation.py
   ```

4. Run the simulation with ROS2 integration:
   ```bash
   python3 sim/python/core/main_simulation_ros2.py --ros2
   ```

## Topic Structure

### MQTT Topics
- `olympus/sensor_data/<sensor_id>/bme680`: Temperature, humidity, pressure data
- `olympus/sensor_data/<sensor_id>/battery`: Battery voltage data
- `olympus/sensor_data/<sensor_id>/display_status`: Display status data
- `olympus/processed_mid_tier/<mid_tier_id>`: Processed data from mid-tier nodes

### ROS2 Topics
- `/olympus/sensor/<sensor_id>/temperature`: Temperature data
- `/olympus/sensor/<sensor_id>/humidity`: Humidity data
- `/olympus/sensor/<sensor_id>/pressure`: Pressure data
- `/olympus/sensor/<sensor_id>/battery`: Battery state
- `/olympus/sensor/<sensor_id>/display`: Display text
- `/olympus/sensor/<sensor_id>/visualization`: Combined data for visualization
- `/olympus/mid_tier/<mid_tier_id>/data`: Mid-tier processed data
- `/olympus/simulation/time`: Current simulation time
- `/olympus/simulation/status`: Simulation status

## Customization

### Adding New Sensors

To add new sensors to the simulation:

1. Modify `main_simulation_ros2.py` to create additional sensor nodes
2. Update the Gazebo world file to include new sensor models
3. Ensure the sensor IDs match between the Python simulation and Gazebo

### Custom Visualizations

To create new visualization types:

1. Extend the Gazebo plugin with new visualization methods
2. Update the sensor model SDF file to use the new visualization type
