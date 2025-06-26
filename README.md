# Olympus Simulation System

A comprehensive multi-sensor simulation platform that integrates MQTT-based hierarchical communication with Gazebo physics simulation and ROS2 visualization. The system features advanced mmWave radar sensor simulation with WSL compatibility and real-time point cloud visualization.

## Overview

The simulation system provides:

- **Multi-Protocol Communication**: MQTT for internal coordination, ROS2 for visualization
- **Advanced mmWave Radar Simulation**: Physics-based radar point cloud generation
- **Cross-Platform Compatibility**: Native Linux and WSL2 support with automatic detection
- **Real-Time Visualization**: Gazebo 3D simulation and RViz2 sensor data visualization
- **Interactive Scene Manipulation**: Programmatic object movement and sensor testing

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
pip3 install paho-mqtt numpy scipy

# WSL Graphics Support (WSL only)
sudo apt install x11-apps mesa-utils
```

## Quick Start

### 1. Clone and Setup
```bash
git clone <repository-url> olympus-sim
cd olympus-sim
chmod +x run_simulation.sh
chmod +x manipulate_scene.py
```

### 2. Build mmWave Plugin
```bash
cd sim/gazebo/plugins
./build_mmwave_plugin.sh --build-only
cd ../../..
```

### 3. Run Simulation

**Headless Mode (Recommended for WSL):**
```bash
./run_simulation.sh
```

**GUI Mode (Native Linux):**
```bash
./run_simulation.sh --gui
```

### 4. Verify Operation
- **RViz2**: Should open showing mmWave point cloud visualization
- **Terminal Output**: Should show "Generated 190 points" messages
- **Topics**: Check with `ros2 topic list` - should see `/mmwave/points`

## Interactive Scene Manipulation

The system includes a powerful scene manipulation tool for testing sensor responses:

```bash
# In a new terminal (while simulation is running)
python3 manipulate_scene.py
```

### Available Commands:
- `list` - Show all entities in the scene
- `move <entity> <x> <y> <z>` - Move an object to new coordinates
- `test` - Run predefined test scenarios
- `quit` - Exit the manipulator

### Example Usage:
```bash
 Enter command: move box_obstacle_1 1.0 1.0 0.5
 Moved box_obstacle_1 to (1.0, 1.0, 0.5)
 Check RViz2 to see how the mmWave sensor response changes!
 Enter command: test
 Running test scenarios...
 Test 1: Moving box_obstacle_1 closer to sensor
 Test 2: Moving cylinder_obstacle to new position  
 Test 3: Moving box_obstacle_2 directly in front of sensor
 Test scenarios complete!
```

## mmWave Sensor Details

### Technical Specifications
- **Frequency**: 77-81 GHz (automotive radar band)
- **Range**: 0.05m to 50m
- **Field of View**: 90° horizontal, 30° vertical
- **Resolution**: 32×16 rays (512 total)
- **Update Rate**: 10 Hz
- **Point Cloud Output**: ~190 points per scan
- **Frame ID**: `mmwave`
- **Topic**: `/mmwave/points` (sensor_msgs/PointCloud2)

### WSL Compatibility Mode
The mmWave plugin automatically detects WSL environments and enables compatibility mode:
- **Physics-Based Simulation**: Uses geometric ray-object intersection
- **No OpenGL Dependency**: Avoids WSL graphics limitations  
- **Consistent Performance**: Maintains 190-point output regardless of environment
- **Automatic Detection**: No manual configuration required

### Sensor Configuration
Located in `sim/gazebo/worlds/mmwave_test.sdf`:
```xml
<plugin filename="libMmWaveSensorPlugin.so" name="olympus_sim::MmWaveSensorPlugin">
  <topic>/mmwave/points</topic>
  <update_rate>10</update_rate>
  <horizontal_fov>1.5708</horizontal_fov>  <!-- 90 degrees -->
  <vertical_fov>0.5236</vertical_fov>      <!-- 30 degrees -->
  <horizontal_resolution>32</horizontal_resolution>
  <vertical_resolution>16</vertical_resolution>
  <min_range>0.05</min_range>
  <max_range>50.0</max_range>
  <wsl_compat_mode>true</wsl_compat_mode>
</plugin>
```

## ROS2 Integration

### Topics Published
- `/mmwave/points` - mmWave radar point cloud (sensor_msgs/PointCloud2)
- `/tf` - Transform tree for coordinate frames
- `/tf_static` - Static transforms (world→mmwave)
- `/clock` - Simulation time synchronization

### Coordinate Frames
- `world` - Global reference frame
- `mmwave` - mmWave sensor frame (1m above origin)

### Bridge Configuration
The ROS2-Gazebo bridge automatically handles:
- Point cloud message conversion (gz.msgs.PointCloudPacked → sensor_msgs/PointCloud2)
- Transform synchronization
- Clock synchronization for simulation time

## Visualization

### RViz2 Configuration
Pre-configured visualization includes:
- **PointCloud2 Display**: mmWave radar points with intensity coloring
- **TF Display**: Coordinate frame visualization
- **Grid Display**: Reference grid for spatial orientation
- **Axes Display**: World coordinate system

### Gazebo GUI (Native Linux)
When running with `--gui` flag:
- **3D Scene View**: Interactive 3D world visualization
- **Entity Inspector**: Object property examination
- **Transform Tools**: Manual object manipulation
- **Plugin Panels**: Sensor configuration and monitoring

## Configuration

### Environment Variables
The system automatically configures:
```bash
# Gazebo paths
export GZ_SIM_RESOURCE_PATH="$OLYMPUS_SIM_ROOT/sim/gazebo/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$OLYMPUS_SIM_ROOT/sim/gazebo/plugins/build"

# WSL graphics compatibility
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export QT_X11_NO_MITSHM=1
```

### Simulation Parameters
Edit `config/simulation_config.yaml`:
```yaml
simulation:
  update_rate: 100  # Hz
  real_time_factor: 1.0
  
mmwave_sensor:
  update_rate: 10
  max_range: 50.0
  noise_stddev: 0.01
  
mqtt:
  broker_host: "localhost"
  broker_port: 1883
```

## Troubleshooting

### Common Issues

**1. Plugin Build Failures**
```bash
# Ensure all dependencies are installed
sudo apt install build-essential cmake libgz-sim8-dev

# Clean and rebuild
cd sim/gazebo/plugins
rm -rf build/
./build_mmwave_plugin.sh --build-only
```

**2. No Point Cloud Data**
```bash
# Check if plugin loaded successfully
gz topic -l | grep mmwave

# Verify ROS2 bridge is running
ros2 topic list | grep mmwave

# Check for error messages
ros2 launch olympus_gazebo.launch.py gui:=false rviz:=true
```

**3. WSL Graphics Issues**
```bash
# Test basic graphics
glxinfo | grep "direct rendering"

# For WSLg (Windows 11)
echo $DISPLAY  # Should show :0

# For X11 forwarding
export DISPLAY=$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0.0
```

**4. MQTT Connection Issues**
```bash
# Start mosquitto broker
sudo systemctl start mosquitto
sudo systemctl enable mosquitto

# Test MQTT connectivity
mosquitto_pub -h localhost -t test -m "hello"
mosquitto_sub -h localhost -t test
```

**5. RViz2 Transform Errors**
```bash
# Check transform tree
ros2 run tf2_tools view_frames

# Verify static transform publisher
ros2 topic echo /tf_static
```

### Performance Optimization

**For WSL Users:**
- Use headless mode: `./run_simulation.sh` (without --gui)
- Increase WSL memory: Add to `.wslconfig`: `memory=4GB`
- Enable WSLg for better graphics support

**For Native Linux:**
- Use GUI mode for full interactivity: `./run_simulation.sh --gui`
- Adjust simulation real-time factor in world file
- Monitor CPU/GPU usage with `htop` and `nvidia-smi`

## Monitoring and Debugging

### Real-Time Monitoring
```bash
# Monitor point cloud data rate
ros2 topic hz /mmwave/points

# View point cloud data
ros2 topic echo /mmwave/points --no-arr

# Monitor system performance
htop
```

### Debug Logging
Enable verbose logging:
```bash
# Gazebo debug output
export GZ_VERBOSE=1
./run_simulation.sh

# ROS2 debug output
ros2 launch olympus_gazebo.launch.py gui:=false rviz:=true --ros-args --log-level DEBUG
```

### Data Recording
```bash
# Record sensor data
ros2 bag record /mmwave/points /tf /tf_static

# Playback recorded data
ros2 bag play <bag_file>
```
### Development Guidelines
- Follow ROS2 coding standards
- Add unit tests for new sensors
- Update documentation for new features
- Test on both native Linux and WSL2
