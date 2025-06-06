#!/bin/bash
# Launch script for Olympus simulation with ROS2 and Gazebo integration

# Stop on errors
set -e

# Source ROS2 environment
if [ -f /opt/ros/jazzy/setup.bash ]; then
    echo "Sourcing /opt/ros/jazzy/setup.bash"
    source /opt/ros/jazzy/setup.bash
else
    echo "Warning: /opt/ros/jazzy/setup.bash not found. Assuming ROS_DISTRO is set and environment is sourced."
fi

# Check for Mosquitto
if ! command -v mosquitto >/dev/null 2>&1; then
    echo "Mosquitto MQTT broker is not installed. Please install with:"
    echo "sudo apt install mosquitto mosquitto-clients"
    exit 1
fi

# Check for ROS2
if ! command -v ros2 >/dev/null 2>&1; then
    echo "ROS2 command not found. Attempting to source ROS2 environment..."
    if [ -d "/opt/ros/jazzy" ]; then
        source /opt/ros/jazzy/setup.bash
    else
        echo "ROS2 Jazzy installation directory not found."
        echo "Please check your ROS2 installation or source the setup file manually."
        exit 1
    fi
    
    # Check again after attempting to source
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "ROS2 command still not found after sourcing. Please check your ROS2 installation."
        exit 1
    fi
fi

echo "ROS2 environment detected."

# Check for ROS2 development tools (specifically 'ros2 pkg' command)
if ! ros2 pkg --help > /dev/null 2>&1; then
    echo "ROS2 'pkg' command (needed for 'ros2 pkg create') not found."
    echo "This command is typically provided by 'ros-<distro>-ros2pkg'."
    echo "Please try installing it by running:"
    echo "  sudo apt update"
    echo "  sudo apt install ros-jazzy-ros2pkg"
    echo "After installation, please re-run this script."
    echo "If the issue persists, you might need a more comprehensive ROS2 installation (e.g., 'ros-jazzy-desktop') or check your ROS2 environment setup."
    exit 1
fi

# Check for colcon build tool
if ! command -v colcon > /dev/null; then
    echo "colcon build tool not found."
    echo "Please install it by running:"
    echo "  sudo apt install python3-colcon-common-extensions"
    exit 1
fi
echo "ROS2 development tools and colcon found."

# Check for ROS2 launch command
if ! ros2 launch --help > /dev/null 2>&1; then
    echo "--------------------------------------------------------------------"
    echo "ERROR: ROS2 'launch' command not found."
    echo "This command is essential for running ROS2 applications."
    echo "Your ROS2 installation appears to be missing this core component."
    echo ""
    echo "Please try installing it by running:"
    echo "  sudo apt update"
    echo "  sudo apt install ros-\$ROS_DISTRO-launch"
    echo ""
    echo "If 'ros-\$ROS_DISTRO-launch' is not found or doesn't resolve the issue,"
    echo "consider installing a more comprehensive ROS2 package, such as:"
    echo "  sudo apt install ros-\$ROS_DISTRO-ros-base"
    echo "or for a full desktop experience with simulation tools:"
    echo "  sudo apt install ros-\$ROS_DISTRO-desktop"
    echo ""
    echo "After installation, please re-run this script."
    echo "--------------------------------------------------------------------"
    exit 1
fi
echo "ROS2 launch command found."

# Define workspace and package paths
ROS2_WS_PATH="$HOME/ros2_olympus_ws"
ROS2_WS_SRC_PATH="$ROS2_WS_PATH/src"
PACKAGE_NAME="olympus_gazebo"
PACKAGE_PATH="$ROS2_WS_SRC_PATH/$PACKAGE_NAME"
PACKAGE_XML_PATH="$PACKAGE_PATH/package.xml"
MQTT_BRIDGE_SCRIPT_NAME="ros2_mqtt_bridge.py"
MQTT_BRIDGE_SOURCE_PATH="$HOME/olympus-sim/$MQTT_BRIDGE_SCRIPT_NAME"
MQTT_BRIDGE_DEST_PATH="$PACKAGE_PATH/scripts/$MQTT_BRIDGE_SCRIPT_NAME"
FORCE_RECREATE_PACKAGE=true

# Function to create and populate the ROS2 package
create_and_populate_package() {
    echo "Ensuring we are in $ROS2_WS_SRC_PATH..."
    mkdir -p "$ROS2_WS_SRC_PATH"
    cd "$ROS2_WS_SRC_PATH"

    echo "Removing existing package directory $PACKAGE_PATH to ensure clean creation..."
    rm -rf "$PACKAGE_PATH"

    # Create ROS2 package with dependencies for simulation
    echo "Creating ROS2 package $PACKAGE_NAME..."
    ros2 pkg create --build-type ament_cmake "$PACKAGE_NAME" --dependencies rclcpp ros_gz_sim std_msgs geometry_msgs

    echo "Adding python3-yaml dependency to package.xml..."
    sed -i "s/<\/package>/  <exec_depend>python3-yaml<\/exec_depend>\n  <exec_depend>python3-numpy<\/exec_depend>\n<\/package>/" "$PACKAGE_PATH/package.xml"
    echo "Added python3-yaml dependency to package.xml."

    # Install PyYAML and numpy directly with pip
    echo "Installing PyYAML and numpy with pip..."
    pip install PyYAML numpy
    echo "PyYAML and numpy installed."

    echo "Creating package subdirectories..."
    mkdir -p "$PACKAGE_PATH/scripts"
    mkdir -p "$PACKAGE_PATH/launch"
    mkdir -p "$PACKAGE_PATH/worlds"
    mkdir -p "$PACKAGE_PATH/models/sensor_display"

    echo "Copying ROS2 MQTT bridge script..."
    if [ -f "$MQTT_BRIDGE_SOURCE_PATH" ]; then
        cp "$MQTT_BRIDGE_SOURCE_PATH" "$MQTT_BRIDGE_DEST_PATH"
        chmod +x "$MQTT_BRIDGE_DEST_PATH"
    else
        echo "ERROR: MQTT Bridge script $MQTT_BRIDGE_SOURCE_PATH not found!"
        exit 1
    fi

    echo "Modifying CMakeLists.txt..."
    cat >> "$PACKAGE_PATH/CMakeLists.txt" << 'EOF'

# Install Python scripts
install(
  PROGRAMS
    scripts/ros2_mqtt_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, worlds, and models
install(
  DIRECTORY
    launch
    worlds
    models
  DESTINATION share/${PROJECT_NAME}
)
EOF

    echo "Creating Gazebo model sensor_display/model.sdf..."
    cat > "$PACKAGE_PATH/models/sensor_display/model.sdf" << 'EOF'
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
  </model>
</sdf>
EOF

    echo "Creating Gazebo world apartment_sensors.world..."
    cat > "$PACKAGE_PATH/worlds/apartment_sensors.world" << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="apartment_sensors">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Sensor 1 Display (e.g., for BME680) -->
    <model name="sensor_1_display">
      <pose>2 2 0.5 0 0 0</pose> <!-- Position for first sensor display -->
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Sensor 2 Display (e.g., for PMSA003I) -->
    <model name="sensor_2_display">
      <pose>4 2 0.5 0 0 0</pose> <!-- Position for second sensor display -->
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Green</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
    <!-- Add more sensor display models as needed -->
  </world>
</sdf>
EOF

    # Fix material script tags in the world file
    sed -i 's/<n>Gazebo\/Red<\/n>/<name>Gazebo\/Red<\/name>/g' "$PACKAGE_PATH/worlds/apartment_sensors.world"
    sed -i 's/<n>Gazebo\/Green<\/n>/<name>Gazebo\/Green<\/name>/g' "$PACKAGE_PATH/worlds/apartment_sensors.world"
    echo "Fixed material script tags in world file"

    echo "Creating launch file olympus_simulation.launch.py..."
    cat > "$PACKAGE_PATH/launch/olympus_simulation.launch.py" << 'EOF'
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_olympus_gazebo_path = get_package_share_directory('olympus_gazebo')
    
    # Pre-construct the full world path string
    world_path = os.path.join(pkg_olympus_gazebo_path, 'worlds', 'apartment_sensors.world')
    gz_arguments = f"-r {world_path}"

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': gz_arguments
        }.items()
    )
    
    mqtt_bridge = Node(
        package='olympus_gazebo',
        executable='ros2_mqtt_bridge.py',
        name='mqtt_bridge',
        output='screen',
        parameters=[
            {'mqtt_broker': 'localhost'},
            {'mqtt_port': 1883},
            {'mqtt_topic_prefix': 'olympus/'}, # Ensure this matches your MQTT publisher
            {'ros_topic_prefix': 'olympus_ros'} # Prefix for ROS topics published by the bridge
        ]
    )
    
    # Construct GZ_SIM_RESOURCE_PATH more robustly
    home_dir = os.path.expanduser('~')
    gz_fuel_path = os.path.join(home_dir, '.gz', 'fuel')
    gz_sim7_models_path = '/usr/share/gz/gz-sim7/models'  # For Gazebo Harmonic/Sim 7
    ros_gz_models_path = '/opt/ros/jazzy/share/gazebo_ros/models'
    gazebo_classic_models_path = '/usr/share/gazebo/models' # Common Gazebo models

    # Get existing GZ_SIM_RESOURCE_PATH from the parent environment (where launch_ros_gazebo.sh runs)
    existing_gz_sim_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    paths_to_check_and_add = [
        gz_fuel_path,                # User's cached Fuel models
        gz_sim7_models_path,         # System Gazebo Sim 7 models
        ros_gz_models_path,          # ROS Gazebo package models
        gazebo_classic_models_path,  # Common Gazebo models (might help)
    ]

    # Start with paths we definitely want to try
    # Filter out non-existent paths from our list to be cleaner, though Gazebo might ignore them
    # For now, let's assume they are generally valid paths to try.
    # unique_ordered_paths = [p for p in paths_to_check_and_add if os.path.exists(p)]
    # Decided to include them anyway, Gazebo should ignore non-existent paths.
    unique_ordered_paths = list(paths_to_check_and_add)

    if existing_gz_sim_resource_path:
        unique_ordered_paths.extend(existing_gz_sim_resource_path.split(os.pathsep))

    # Create a unique list while preserving order as much as possible
    # (from Python 3.7+, dict.fromkeys preserves insertion order)
    final_path_list = list(dict.fromkeys(filter(None, unique_ordered_paths))) # filter(None, ...) removes empty strings
    final_gz_sim_resource_path = os.pathsep.join(final_path_list)

    # For debugging purposes, print the path that will be set
    print(f"[OLYMPUS LAUNCH] Attempting to set GZ_SIM_RESOURCE_PATH to: {final_gz_sim_resource_path}")

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=final_gz_sim_resource_path),
        gz_sim_launch,
        mqtt_bridge
    ])
EOF
    echo "Package $PACKAGE_NAME created and populated successfully."
}

# Check and create ROS2 workspace and package
if [ "$FORCE_RECREATE_PACKAGE" = true ]; then
    echo "FORCE_RECREATE_PACKAGE is true. Forcing package recreation..."
    create_and_populate_package
elif [ ! -d "$ROS2_WS_PATH" ]; then
    echo "ROS2 workspace $ROS2_WS_PATH not found. Creating..."
    mkdir -p "$ROS2_WS_SRC_PATH"
    create_and_populate_package
elif [ ! -f "$PACKAGE_XML_PATH" ]; then
    echo "ROS2 package $PACKAGE_NAME with package.xml not found in $ROS2_WS_SRC_PATH. Recreating..."
    create_and_populate_package
else
    echo "ROS2 workspace and package $PACKAGE_NAME seem to exist (FORCE_RECREATE_PACKAGE is not true)."
    # Optional: Add a check for a key file like the bridge script and re-populate if missing
    if [ ! -f "$MQTT_BRIDGE_DEST_PATH" ]; then
        echo "Key file $MQTT_BRIDGE_DEST_PATH missing. Re-populating package."
        create_and_populate_package
    else
        echo "Key files found. Assuming package is correctly set up."
    fi
fi


# Navigate to workspace and attempt to build
echo "Navigating to ROS2 workspace: $HOME/ros2_olympus_ws"
cd $HOME/ros2_olympus_ws

echo "Cleaning workspace: removing build, install, log directories..."
rm -rf build install log

echo "Current CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"

echo "Attempting to build ROS2 workspace with colcon..."
# Allow colcon to output errors directly
if ! colcon build --symlink-install --packages-up-to olympus_gazebo; then
    echo "ERROR: colcon build failed. Please check the output above for errors."
    exit 1
fi

# Check if build was successful by looking for setup.bash
if [ ! -f "$HOME/ros2_olympus_ws/install/setup.bash" ]; then
    echo "ERROR: ROS2 workspace build completed, but install/setup.bash was not found."
    echo "This usually indicates a problem with the package structure or CMakeLists.txt."
    echo "Please check the contents of $HOME/ros2_olympus_ws."
    exit 1
fi
# Additional check to ensure the specific package was built
if ! colcon list --packages-select olympus_gazebo | grep -q "olympus_gazebo"; then
    echo "--------------------------------------------------------------------"
    echo "ERROR: colcon build seemed to complete, but the 'olympus_gazebo' package was not found by 'colcon list'."
    echo "This often indicates an issue with your package.xml, CMakeLists.txt, or the directory structure within:"
    echo "  \$HOME/ros2_olympus_ws/src/olympus_gazebo"
    echo ""
    echo "Common causes:"
    echo "  - Incorrect 'build_type' in package.xml for the CMakeLists.txt contents."
    echo "  - Missing 'ament_package()' in CMakeLists.txt."
    echo "  - package.xml not found or malformed in the package directory."
    echo ""
    echo "Please review the files in your package and the colcon build output above."
    echo "--------------------------------------------------------------------"
    exit 1
fi
echo "ROS2 workspace built successfully, and 'olympus_gazebo' package is recognized by colcon."

# Start Mosquitto if not running
if ! pgrep mosquitto > /dev/null; then
    echo "Starting MQTT broker..."
    mosquitto -d
fi

# Source ROS2 workspace
source $HOME/ros2_olympus_ws/install/setup.bash

# Launch ROS2 and Gazebo in background
echo "Launching ROS2 with Gazebo..."

# Set up Gazebo model path to include standard models
echo "Setting up Gazebo model paths..."
export GAZEBO_MODEL_PATH=/opt/ros/jazzy/share/gazebo_ros/models:${GAZEBO_MODEL_PATH:-}
export GZ_SIM_RESOURCE_PATH=/usr/share/gz/gz-sim7/models:/opt/ros/jazzy/share/gazebo_ros/models:${GZ_SIM_RESOURCE_PATH:-}

# Make sure Python ROS2 environment has necessary dependencies
echo "Installing system Python dependencies..."
sudo apt-get update -y
sudo apt-get install -y python3-numpy python3-yaml

# Fix material script tags in the world file directly
echo "Fixing material script tags in world file..."
WORLD_PATH="$ROS2_WS_PATH/install/olympus_gazebo/share/olympus_gazebo/worlds/apartment_sensors.world"
if [ -f "$WORLD_PATH" ]; then
  sed -i -e 's|<n>Gazebo/Red</n>|<name>Gazebo/Red</name>|g' "$WORLD_PATH"
  sed -i -e 's|<n>Gazebo/Green</n>|<name>Gazebo/Green</name>|g' "$WORLD_PATH"
  echo "Fixed material script tags in world file"
else
  echo "World file not found at: $WORLD_PATH"
  echo "Searching for world files..."
  find "$ROS2_WS_PATH" -name "apartment_sensors.world" 2>/dev/null
fi

# Fix Python module path to ensure yaml module is found
echo "Setting up Python module paths..."

echo "Verifying standard Gazebo model paths..."
echo "Contents of /usr/share/gz/gz-sim7/models:"
ls -l /usr/share/gz/gz-sim7/models || echo "/usr/share/gz/gz-sim7/models not found or ls failed"
echo "Checking for sun model in /usr/share/gz/gz-sim7/models/sun:"
ls -l /usr/share/gz/gz-sim7/models/sun || echo "/usr/share/gz/gz-sim7/models/sun not found or ls failed"
echo "Checking for ground_plane model in /usr/share/gz/gz-sim7/models/ground_plane:"
ls -l /usr/share/gz/gz-sim7/models/ground_plane || echo "/usr/share/gz/gz-sim7/models/ground_plane not found or ls failed"
echo "Contents of /usr/share/gazebo/models:"
ls -l /usr/share/gazebo/models || echo "/usr/share/gazebo/models not found or ls failed"

export PYTHONPATH=$(python3 -c 'import sys; print(":" + ":".join(sys.path))')

source "$HOME/ros2_olympus_ws/install/setup.bash"
ros2 launch "$PACKAGE_NAME" olympus_simulation.launch.py &
ROS_PID=$!

# Give ROS and Gazebo time to start
sleep 5

# Set up Python environment
if [ -d "$HOME/olympus-sim/venv" ]; then
    echo "Activating Python virtual environment..."
    source $HOME/olympus-sim/venv/bin/activate
else
    echo "Creating Python virtual environment..."
    cd $HOME/olympus-sim
    python3 -m venv venv
    source venv/bin/activate
    pip install paho-mqtt
fi

# Run the Python simulation
echo "Starting Olympus simulation..."
cd $HOME/olympus-sim
python main_simulation_mqtt.py --duration 300

# Clean up
echo "Shutting down ROS2 and Gazebo..."
kill $ROS_PID

echo "Simulation complete"
