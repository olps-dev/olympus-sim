#!/usr/bin/env python3
"""
Wrapper script to run the Olympus simulation with ROS2 integration
"""

import os
import sys
import argparse
import time
import logging
import importlib.util

# Configure logging
logging.basicConfig(level=logging.INFO,
                   format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
log = logging.getLogger('olympus_sim')

# Add the project root to the Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
sys.path.insert(0, project_root)

# Check if ROS2 is available
ros2_available = False
try:
    import rclpy
    from rclpy.node import Node
    ros2_available = True
    log.info("ROS2 is available")
except ImportError:
    log.warning("ROS2 is not available. Running in MQTT-only mode.")

# Now we can import from sim.python modules using absolute imports
from sim.python.olympus_nodes import Node as OlympusNode
from sim.python.mqtt_broker_sim import MQTTBrokerSim, MQTTSimulatedClient
from sim.python.hardware_sensor_bridge import HardwareSensorBridge

def main():
    """Run the Olympus simulation with optional ROS2 integration."""
    parser = argparse.ArgumentParser(description='Olympus Simulation with ROS2 Integration')
    parser.add_argument('--duration', type=int, default=60,
                      help='Simulation duration in seconds')
    parser.add_argument('--tick', type=float, default=1.0,
                      help='Simulation tick interval in seconds')
    parser.add_argument('--ros2', action='store_true',
                      help='Enable ROS2 integration')
    parser.add_argument('--gazebo', action='store_true',
                      help='Enable Gazebo visualization (requires ROS2)')
    args = parser.parse_args()
    
    # Determine which simulation to run
    if args.ros2 and ros2_available:
        log.info("Running simulation with ROS2 integration")
        
        # Import the ROS2 version of the simulation
        # We use importlib to avoid import errors if the file has ROS2 imports
        spec = importlib.util.spec_from_file_location(
            "main_simulation_ros2", 
            os.path.join(project_root, "sim", "python", "core", "main_simulation_ros2.py")
        )
        ros2_sim = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ros2_sim)
        
        # Run the ROS2 simulation
        ros2_sim.main(duration=args.duration, tick_interval=args.tick)
    else:
        log.info("Running standard simulation (no ROS2)")
        
        # Import the standard simulation
        import sim.python.core.main_simulation as std_sim
        
        # Run the standard simulation
        std_sim.main(duration=args.duration, tick_interval=args.tick)

if __name__ == "__main__":
    main()
