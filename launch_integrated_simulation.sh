#!/bin/bash
# Integrated launch script for Olympus Simulation with Gazebo
# This script will:
# 1. Start the Mosquitto MQTT broker
# 2. Launch Gazebo with the apartment.sdf world
# 3. Start the Python simulation

# Exit on error
set -e

echo "==== Olympus Integrated Simulation Launch ===="
echo "Starting services..."

# Check if Mosquitto broker is installed
if ! command -v mosquitto &> /dev/null; then
    echo "Mosquitto MQTT broker not found. Please install it first:"
    echo "sudo apt-get update && sudo apt-get install -y mosquitto mosquitto-clients"
    exit 1
fi

# Check if Gazebo is installed
if ! command -v gz &> /dev/null; then
    echo "Gazebo not found. Please install Gazebo Garden or newer."
    exit 1
fi

# Kill any existing mosquitto processes
echo "Stopping any existing MQTT brokers..."
pkill mosquitto || true
sleep 1

# Start Mosquitto broker
echo "Starting MQTT broker..."
mosquitto -v -d

# Give it a moment to start
sleep 2

# Check if broker started properly
if ! pgrep mosquitto > /dev/null; then
    echo "Failed to start Mosquitto broker."
    exit 1
fi

echo "MQTT broker running."

# Launch Gazebo in the background
echo "Launching Gazebo with apartment world..."
# Get the current directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORLD_PATH="${SCRIPT_DIR}/sim/gazebo/worlds/apartment.sdf"

# Add sensor models to the apartment world
# Create a temporary modified world file with sensor models included
TMP_WORLD="/tmp/apartment_with_sensors.sdf"
cp "${WORLD_PATH}" "${TMP_WORLD}"

# Extract the end tag position and insert sensor models before it
END_TAG_LINE=$(grep -n "</world>" "${TMP_WORLD}" | cut -d: -f1)
head -n $((END_TAG_LINE-1)) "${TMP_WORLD}" > "${TMP_WORLD}.new"

# Add sensor models for each sensor in the simulation 
cat >> "${TMP_WORLD}.new" << EOF
    <!-- Sensor 1 -->
    <include>
      <uri>model://sensor_model</uri>
      <name>sensor_1</name>
      <pose>2 2 0.5 0 0 0</pose>
    </include>
    
    <!-- Sensor 2 -->
    <include>
      <uri>model://sensor_model</uri>
      <name>sensor_2</name>
      <pose>5 2 0.5 0 0 0</pose>
    </include>
    
    <!-- Sensor 3 -->
    <include>
      <uri>model://sensor_model</uri>
      <name>sensor_3</name>
      <pose>8 2 0.5 0 0 0</pose>
    </include>
    
    <!-- Sensor 4 -->
    <include>
      <uri>model://sensor_model</uri>
      <name>sensor_4</name>
      <pose>2 6 0.5 0 0 0</pose>
    </include>
    
    <!-- Sensor 5 -->
    <include>
      <uri>model://sensor_model</uri>
      <name>sensor_5</name>
      <pose>5 6 0.5 0 0 0</pose>
    </include>
    
    <!-- Sensor 6 -->
    <include>
      <uri>model://sensor_model</uri>
      <name>sensor_6</name>
      <pose>8 6 0.5 0 0 0</pose>
    </include>
EOF

# Add closing tag
echo "</world>" >> "${TMP_WORLD}.new"
echo "</sdf>" >> "${TMP_WORLD}.new"
mv "${TMP_WORLD}.new" "${TMP_WORLD}"

# Create a model directory for Gazebo to find our sensor model
mkdir -p ~/.gz/models/sensor_model
cp "${SCRIPT_DIR}/sim/gazebo/worlds/sensor_model.sdf" ~/.gz/models/sensor_model/model.sdf

# Create a model.config file
cat > ~/.gz/models/sensor_model/model.config << EOF
<?xml version="1.0"?>
<model>
  <name>Olympus Sensor</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>
    A sensor node for the Olympus simulation with MQTT visualization capabilities.
  </description>
</model>
EOF

# Launch Gazebo with the modified world file
gz sim -r "${TMP_WORLD}" &
GAZEBO_PID=$!

# Give Gazebo some time to start
sleep 5
echo "Gazebo running with PID: ${GAZEBO_PID}"

# Start Python simulation
echo "Starting Python simulation..."
cd "${SCRIPT_DIR}"
python3 main_simulation_mqtt.py &
PYTHON_PID=$!
echo "Python simulation running with PID: ${PYTHON_PID}"

echo "==== All components started ===="
echo "Press Ctrl+C to shutdown all components..."

# Wait for user input to terminate
trap 'echo "Shutting down..."; kill $GAZEBO_PID $PYTHON_PID; pkill mosquitto; echo "Done!"; exit 0' INT TERM
wait
