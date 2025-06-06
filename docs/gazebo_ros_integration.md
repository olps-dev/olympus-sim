# Olympus Simulation - ROS2/Gazebo Integration

This guide explains how to integrate the Olympus simulation with ROS2 Jazzy and Gazebo 8.9.

## Overview

This integration uses ROS2 as a bridge between MQTT and Gazebo:
1. Python simulation publishes sensor data to MQTT
2. ROS2 MQTT bridge subscribes to MQTT topics and publishes to ROS2 topics
3. Gazebo models subscribe to ROS2 topics for visualization

## Setup Instructions

### 1. Create ROS2 Workspace

```bash
mkdir -p ~/ros2_olympus_ws/src
cd ~/ros2_olympus_ws/src
```

### 2. Create the ROS2 Package

```bash
ros2 pkg create --build-type ament_cmake olympus_gazebo --dependencies rclcpp gazebo_ros std_msgs geometry_msgs
```

### 3. Copy the ROS2 MQTT Bridge to Your ROS2 Package

```bash
cp ~/olympus-sim/ros2_mqtt_bridge.py ~/ros2_olympus_ws/src/olympus_gazebo/scripts/
chmod +x ~/ros2_olympus_ws/src/olympus_gazebo/scripts/ros2_mqtt_bridge.py
```

### 4. Create the Sensor Model for Gazebo

Create a file at `~/ros2_olympus_ws/src/olympus_gazebo/models/sensor_display/model.sdf` with this content:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="sensor_display">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
          <specular>0 0 0 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
      </collision>
    </link>
    
    <!-- Text display -->
    <plugin name="text_display" filename="libgazebo_ros_text_display.so">
      <ros>
        <namespace>/olympus</namespace>
        <remapping>~/text:=sensor/{sensor_id}/display_text</remapping>
      </ros>
      <position>0 0 0.15</position>
      <font_size>12</font_size>
    </plugin>
    
    <!-- Material color visualizer -->
    <plugin name="material_color" filename="libgazebo_ros_material_color.so">
      <ros>
        <namespace>/olympus</namespace>
        <remapping>~/color:=sensor/{sensor_id}/material_color</remapping>
      </ros>
      <link_name>link</link_name>
      <visual_name>visual</visual_name>
    </plugin>
  </model>
</sdf>
```

### 5. Create a World File with Sensor Models

Create a file at `~/ros2_olympus_ws/src/olympus_gazebo/worlds/apartment_sensors.world` with:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="apartment_sensors">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include your apartment model here -->
    <!-- <include>
      <uri>model://apartment</uri>
    </include> -->
    
    <!-- Sensor 1 -->
    <include>
      <uri>model://sensor_display</uri>
      <name>sensor_1</name>
      <pose>2 2 0.5 0 0 0</pose>
      <plugin name="sensor_id_setter" filename="libgazebo_ros_set_parameters.so">
        <ros>
          <namespace>/olympus/sensor_1</namespace>
        </ros>
        <parameters>
          <parameter name="sensor_id" type="string">sensor-1</parameter>
        </parameters>
      </plugin>
    </include>
    
    <!-- Add more sensors as needed -->
    <include>
      <uri>model://sensor_display</uri>
      <name>sensor_2</name>
      <pose>4 2 0.5 0 0 0</pose>
      <plugin name="sensor_id_setter" filename="libgazebo_ros_set_parameters.so">
        <ros>
          <namespace>/olympus/sensor_2</namespace>
        </ros>
        <parameters>
          <parameter name="sensor_id" type="string">sensor-2</parameter>
        </parameters>
      </plugin>
    </include>
    
    <!-- Add more sensors as needed -->
  </world>
</sdf>
```

### 6. Create a Launch File

Create a file at `~/ros2_olympus_ws/src/olympus_gazebo/launch/olympus_simulation.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_olympus_gazebo = get_package_share_directory('olympus_gazebo')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_olympus_gazebo, 'worlds', 'apartment_sensors.world')
        }.items()
    )
    
    # MQTT Bridge node
    mqtt_bridge = Node(
        package='olympus_gazebo',
        executable='ros2_mqtt_bridge.py',
        name='mqtt_bridge',
        output='screen',
        parameters=[
            {'mqtt_broker': 'localhost'},
            {'mqtt_port': 1883},
            {'mqtt_topic_prefix': 'olympus/'}
        ]
    )
    
    return LaunchDescription([
        gazebo,
        mqtt_bridge
    ])
```

### 7. Update CMakeLists.txt and package.xml

Edit `~/ros2_olympus_ws/src/olympus_gazebo/CMakeLists.txt` to include:

```cmake
# Install Python scripts
install(
  PROGRAMS
    scripts/ros2_mqtt_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY
    launch
    worlds
    models
  DESTINATION share/${PROJECT_NAME}
)
```

Update `~/ros2_olympus_ws/src/olympus_gazebo/package.xml` to include:

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>gazebo_ros</depend>
<exec_depend>python3-paho-mqtt</exec_depend>
```

### 8. Build the ROS2 Package

```bash
cd ~/ros2_olympus_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the Integrated Simulation

1. **Start the MQTT Broker** (if not already running):
   ```bash
   mosquitto -d
   ```

2. **Run the ROS2 Gazebo Integration**:
   ```bash
   source ~/ros2_olympus_ws/install/setup.bash
   ros2 launch olympus_gazebo olympus_simulation.launch.py
   ```

3. **Run the Python Simulation**:
   ```bash
   cd ~/olympus-sim/
   source venv/bin/activate  # Activate your Python virtual environment
   python main_simulation_mqtt.py --duration 300
   ```

Now you should see the data from the Python simulation being visualized in Gazebo through the ROS2 bridge!

## Custom Gazebo Plugins

For more advanced visualization, you might need to create custom Gazebo plugins:

1. Text display plugin (`libgazebo_ros_text_display.so`):
   - Displays text messages on models in Gazebo
   - Subscribes to ROS2 String messages

2. Material color plugin (`libgazebo_ros_material_color.so`):
   - Changes the color of a model based on data
   - Subscribes to ROS2 ColorRGBA messages

These plugins need to be implemented in C++ and built as part of your ROS2 package.

## Troubleshooting

1. If ROS2 topics aren't appearing, check:
   ```bash
   ros2 topic list
   ```

2. To monitor MQTT messages:
   ```bash
   mosquitto_sub -t 'olympus/#' -v
   ```

3. To monitor ROS2 messages:
   ```bash
   ros2 topic echo /olympus/sensor/sensor-1/temperature
   ```
