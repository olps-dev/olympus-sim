#!/usr/bin/env python3
"""
Main entry point for running the Olympus simulation with or without ROS2/Gazebo
This script handles Python module imports correctly
"""

import os
import sys
import time
import argparse
import logging
import subprocess
import importlib.util
import inspect

# Configure logging
logging.basicConfig(level=logging.INFO,
                   format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
log = logging.getLogger('olympus_sim')

# Add the project root to the Python path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

# Check if MQTT broker is running
def ensure_mqtt_broker():
    """Make sure an MQTT broker is running"""
    try:
        import paho.mqtt.client as mqtt
        # Try to connect to check if broker is running
        client = mqtt.Client()
        client.connect("localhost", 1883, 1)
        client.disconnect()
        log.info("MQTT broker is already running")
    except:
        log.info("Starting MQTT broker...")
        try:
            # Try to start mosquitto
            subprocess.Popen(["mosquitto", "-d"], 
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
            time.sleep(1)
            log.info("MQTT broker started")
        except Exception as e:
            log.warning(f"Could not start MQTT broker: {e}")
            log.warning("You may need to start it manually")

# Check if ROS2 is available
def check_ros2_available():
    """Check if ROS2 is available"""
    try:
        import rclpy
        return True
    except ImportError:
        return False

def main():
    """Run the Olympus simulation with optional ROS2 integration."""
    parser = argparse.ArgumentParser(description='Olympus Simulation')
    parser.add_argument('--duration', type=int, default=60,
                      help='Simulation duration in seconds')
    parser.add_argument('--tick', type=float, default=1.0,
                      help='Simulation tick interval in seconds')
    parser.add_argument('--ros2', action='store_true',
                      help='Enable ROS2 integration if available')
    parser.add_argument('--mqtt-only', action='store_true',
                      help='Force MQTT-only mode (no ROS2)')
    parser.add_argument('--gazebo', action='store_true',
                      help='Launch Gazebo visualization (requires ROS2)')
    args = parser.parse_args()
    
    # Make sure MQTT broker is running
    ensure_mqtt_broker()
    
    # Check ROS2 availability
    ros2_available = check_ros2_available()
    
    # Import the simulation modules
    from sim.python.olympus_nodes import Node
    from sim.python.mqtt_broker_sim import MQTTBrokerSim, MQTTSimulatedClient
    from sim.python.hardware_sensor_bridge import HardwareSensorBridge
    
    # Import the main simulation module
    import sim.python.core.main_simulation as main_sim
    
    # Modify the global constants in the main_sim module to use our parameters
    if hasattr(main_sim, 'SIMULATION_DURATION_SECONDS'):
        main_sim.SIMULATION_DURATION_SECONDS = args.duration
    if hasattr(main_sim, 'SIMULATION_TICK_INTERVAL'):
        main_sim.SIMULATION_TICK_INTERVAL = args.tick
    
    # Determine which simulation to run
    if args.ros2 and ros2_available and not args.mqtt_only:
        # Check if Gazebo was requested
        gazebo_enabled = args.gazebo
        log.info("Running simulation with ROS2 integration")
        
        # Try to import the ROS2 simulation module
        try:
            # Use direct import first
            try:
                import sim.python.core.main_simulation_ros2 as ros2_sim
                # Check if the ROS2 version accepts parameters
                if len(inspect.signature(ros2_sim.main).parameters) > 0:
                    ros2_sim.main(duration=args.duration, tick_interval=args.tick, launch_gazebo=gazebo_enabled)
                else:
                    # Modify module constants instead
                    if hasattr(ros2_sim, 'SIMULATION_DURATION_SECONDS'):
                        ros2_sim.SIMULATION_DURATION_SECONDS = args.duration
                    if hasattr(ros2_sim, 'SIMULATION_TICK_INTERVAL'):
                        ros2_sim.SIMULATION_TICK_INTERVAL = args.tick
                    if hasattr(ros2_sim, 'LAUNCH_GAZEBO'):
                        ros2_sim.LAUNCH_GAZEBO = gazebo_enabled
                    ros2_sim.main()
            except ImportError:
                # If that fails, try to load it using importlib
                log.info("Using importlib to load ROS2 simulation")
                spec = importlib.util.spec_from_file_location(
                    "main_simulation_ros2", 
                    os.path.join(project_root, "sim", "python", "core", "main_simulation_ros2.py")
                )
                ros2_sim = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(ros2_sim)
                # Set module constants if available
                if hasattr(ros2_sim, 'LAUNCH_GAZEBO'):
                    ros2_sim.LAUNCH_GAZEBO = gazebo_enabled
                # Just call main without parameters and rely on the module constants
                ros2_sim.main()
        except Exception as e:
            log.error(f"Failed to run ROS2 simulation: {e}")
            log.info("Falling back to standard simulation")
            main_sim.main()
    else:
        if args.gazebo:
            log.warning("Gazebo requested but requires ROS2. Ignoring Gazebo flag.")
        if args.ros2 and not ros2_available:
            log.warning("ROS2 requested but not available. Running standard simulation.")
        elif args.mqtt_only:
            log.info("MQTT-only mode selected")
        else:
            log.info("Running standard simulation")
        
        # Run the standard simulation without parameters
        main_sim.main()

if __name__ == "__main__":
    main()
