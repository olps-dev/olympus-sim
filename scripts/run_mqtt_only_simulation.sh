#!/bin/bash
# Run the Olympus simulation with MQTT only (no ROS2/Gazebo dependencies)

# Set script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Check if MQTT broker is running
if ! pgrep -x "mosquitto" > /dev/null; then
    echo "Starting MQTT broker..."
    mosquitto -d
    sleep 2
fi

# Run the simulation using the wrapper script
echo "Starting Olympus simulation (MQTT-only mode)..."
cd "$PROJECT_ROOT"
python3 -m scripts.run_simulation --duration 60

echo "Simulation complete!"

