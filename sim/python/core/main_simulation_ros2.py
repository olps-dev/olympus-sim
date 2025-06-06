#!/usr/bin/env python3
"""
Project Olympus - Main Simulation with ROS2 Integration
This version of the main simulation integrates with ROS2 and Gazebo
while maintaining compatibility with the existing MQTT-based system.
"""

import time
import random
import argparse
import threading
import os
import sys
import signal
import logging
import subprocess
from typing import Optional, List

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import Olympus components
from ..olympus_nodes import Node
from ..mqtt_broker_sim import MQTTBrokerSim, MQTTSimulatedClient
from ..hardware_sensor_bridge import HardwareSensorBridge

# Import ROS2 components if available
try:
    import rclpy
    from rclpy.node import Node as ROS2Node
    from std_msgs.msg import String, Float64
    from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ROS2 packages not found. Running in MQTT-only mode.")
    
    # Create dummy classes for ROS2 when not available
    class ROS2Node:
        """Dummy ROS2 Node class when ROS2 is not available."""
        def __init__(self, name):
            self.name = name
            print(f"[DUMMY] Created ROS2 node '{name}'")
            
        def create_publisher(self, *args, **kwargs):
            return DummyPublisher()
            
        def create_timer(self, *args, **kwargs):
            return None
            
    class DummyPublisher:
        """Dummy publisher when ROS2 is not available."""
        def publish(self, *args, **kwargs):
            pass

# Simulation Configuration
NUM_MID_TIER_NODES = 2
NUM_SENSORS_PER_MID_TIER = 3
SIMULATION_DURATION_SECONDS = 3600  # 1 hour by default
SIMULATION_TICK_INTERVAL = 1.0    # 1 second by default
LAUNCH_GAZEBO = False             # Whether to launch Gazebo

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
log = logging.getLogger('OlympusSimulation')

class OlympusROS2Bridge(ROS2Node):
    """ROS2 bridge for the Olympus simulation."""
    
    def __init__(self):
        """Initialize the ROS2 bridge."""
        if not ROS2_AVAILABLE:
            # Create a dummy bridge when ROS2 is not available
            self.sim_time_pub = DummyPublisher() if 'DummyPublisher' in globals() else None
            self.sim_status_pub = DummyPublisher() if 'DummyPublisher' in globals() else None
            log.info("Created dummy ROS2 bridge (ROS2 not available)")
            return
            
        try:
            super().__init__('olympus_simulation')
            
            # Publishers
            self.sim_time_pub = self.create_publisher(Float64, '/olympus/simulation/time', 10)
            self.sim_status_pub = self.create_publisher(String, '/olympus/simulation/status', 10)
            
            # Create a timer for publishing simulation status
            self.create_timer(1.0, self.publish_status)
            
            log.info("ROS2 bridge initialized")
        except Exception as e:
            log.error(f"Failed to initialize ROS2 bridge: {e}")
            # Create dummy publishers for compatibility
            self.sim_time_pub = DummyPublisher() if 'DummyPublisher' in globals() else None
            self.sim_status_pub = DummyPublisher() if 'DummyPublisher' in globals() else None
    
    def publish_time(self, sim_time: float):
        """Publish the current simulation time."""
        if not ROS2_AVAILABLE:
            return
            
        msg = Float64()
        msg.data = sim_time
        self.sim_time_pub.publish(msg)
    
    def publish_status(self):
        """Publish the simulation status."""
        if not ROS2_AVAILABLE:
            return
            
        msg = String()
        msg.data = "running"
        self.sim_status_pub.publish(msg)

def main(duration=None, tick_interval=None, launch_gazebo=None):
    """Main simulation function."""
    # Parse command line arguments if not provided as parameters
    if duration is None or tick_interval is None or launch_gazebo is None:
        parser = argparse.ArgumentParser(description='Olympus Simulation with ROS2 Integration')
        parser.add_argument('--duration', type=int, default=SIMULATION_DURATION_SECONDS,
                            help='Simulation duration in seconds')
        parser.add_argument('--tick', type=float, default=SIMULATION_TICK_INTERVAL,
                            help='Simulation tick interval in seconds')
        parser.add_argument('--ros2', action='store_true', default=ROS2_AVAILABLE,
                            help='Enable ROS2 integration')
        parser.add_argument('--gazebo', action='store_true', default=LAUNCH_GAZEBO,
                            help='Launch Gazebo visualization')
        args = parser.parse_args()
        duration = args.duration
        tick_interval = args.tick
        launch_gazebo = args.gazebo
    else:
        # Use provided parameters
        args = argparse.Namespace()
        args.duration = duration
        args.tick = tick_interval
        args.ros2 = ROS2_AVAILABLE
        args.gazebo = launch_gazebo
    
    # Initialize ROS2 if available and enabled
    ros2_node = None
    ros2_thread = None
    gazebo_process = None
    if args.ros2 and ROS2_AVAILABLE:
        try:
            rclpy.init()
            ros2_node = OlympusROS2Bridge()
            
            # Create a separate thread for ROS2 spinning
            def spin_ros2():
                try:
                    while rclpy.ok():
                        rclpy.spin_once(ros2_node, timeout_sec=0.1)
                        time.sleep(0.01)
                except Exception as e:
                    log.error(f"Error in ROS2 spin thread: {e}")
            
            ros2_thread = threading.Thread(target=spin_ros2)
            ros2_thread.daemon = True
            ros2_thread.start()
            log.info("ROS2 integration enabled")
            
            # Launch Gazebo Harmonic if requested
            if args.gazebo:
                try:
                    log.info("Launching Gazebo Harmonic...")
                    # Check if we have the gazebo launch file
                    gazebo_launch_file = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 
                                                    "ros2", "launch", "olympus_gazebo.launch.py")
                    
                    if os.path.exists(gazebo_launch_file):
                        # Launch Gazebo Harmonic using ros2 launch
                        cmd = ["ros2", "launch", gazebo_launch_file]
                        gazebo_process = subprocess.Popen(cmd, 
                                                        stdout=subprocess.PIPE,
                                                        stderr=subprocess.PIPE,
                                                        text=True)
                        log.info("Gazebo Harmonic launched successfully")
                        
                        # Wait a moment for Gazebo to initialize
                        time.sleep(2)
                        
                        # Check if Gazebo is running
                        if gazebo_process.poll() is not None:
                            log.error(f"Gazebo Harmonic failed to start. Return code: {gazebo_process.returncode}")
                            # Get error output
                            _, stderr = gazebo_process.communicate()
                            log.error(f"Gazebo error: {stderr}")
                            gazebo_process = None
                    else:
                        log.error(f"Gazebo launch file not found at {gazebo_launch_file}")
                except Exception as e:
                    log.error(f"Failed to launch Gazebo Harmonic: {e}")
        except Exception as e:
            log.error(f"Failed to initialize ROS2: {e}")
            args.ros2 = False
            # Create a dummy ROS2 node for compatibility
            ros2_node = OlympusROS2Bridge()
    else:
        # Create a dummy ROS2 node for compatibility
        ros2_node = OlympusROS2Bridge()
        log.info("Running without ROS2 integration")
    
    print(f"[{time.strftime('%H:%M:%S')}] Olympus Simulation Starting...")
    print("-" * 50)

    # 1. Create Network Components
    #    - Zeus Node and its Broker
    #    - Mid-Tier Nodes and their Brokers
    #    - Sensor Nodes

    # Zeus Layer
    zeus_node = Node(node_id="zeus-1", node_type="zeus")
    zeus_broker = MQTTBrokerSim(broker_id="zeus-broker-1", owner_node=zeus_node)
    zeus_node.set_local_broker(zeus_broker)

    mid_tier_nodes = []
    sensor_nodes = []

    # Mid-Tier and Sensor Layers
    for i in range(NUM_MID_TIER_NODES):
        mid_tier_id = f"mid-tier-{i+1}"
        mid_tier_node = Node(node_id=mid_tier_id, node_type="mid_tier", parent_broker=zeus_broker)
        mid_tier_broker = MQTTBrokerSim(broker_id=f"{mid_tier_id}-broker", owner_node=mid_tier_node)
        mid_tier_node.set_local_broker(mid_tier_broker)
        
        # Mid-tier subscribes to its own local broker for data from its sensors
        # And also subscribes to the Zeus broker if it expects commands (not shown here)
        mid_tier_mqtt_client_for_zeus = MQTTSimulatedClient(client_id_node=mid_tier_node, broker_sim=zeus_broker)
        # mid_tier_node.parent_broker is already zeus_broker, client is for publishing upwards

        zeus_node.add_child_node(mid_tier_node) # Zeus is parent of mid-tier
        mid_tier_nodes.append(mid_tier_node)

        for j in range(NUM_SENSORS_PER_MID_TIER):
            sensor_id = f"sensor-{i*NUM_SENSORS_PER_MID_TIER + j + 1}"
            # Sensor node's parent broker is the mid-tier broker it reports to
            sensor_node = Node(node_id=sensor_id, node_type="sensor_node", parent_broker=mid_tier_broker)
            
            # Each sensor node gets an MQTT client to publish to its mid-tier broker
            sensor_mqtt_client = MQTTSimulatedClient(client_id_node=sensor_node, broker_sim=mid_tier_broker)
            
            # Create and assign HardwareSensorBridge to the sensor node
            # The bridge uses the sensor's MQTT client to publish data
            hardware_bridge = HardwareSensorBridge(node_id=sensor_id, mqtt_client_sim=sensor_mqtt_client)
            sensor_node.set_hardware_bridge(hardware_bridge)

            mid_tier_node.add_child_node(sensor_node) # Mid-tier is parent of sensor
            sensor_nodes.append(sensor_node)

            # Mid-tier node needs to subscribe to topics from its sensors on its local broker
            # Example: mid-tier-1-broker will get sensor_data/sensor-1/bme680, sensor_data/sensor-1/battery etc.
            mid_tier_broker.subscribe(client_node=mid_tier_node, topic=f"sensor_data/{sensor_id}/bme680")
            mid_tier_broker.subscribe(client_node=mid_tier_node, topic=f"sensor_data/{sensor_id}/battery")
            mid_tier_broker.subscribe(client_node=mid_tier_node, topic=f"sensor_data/{sensor_id}/display_status")
    
    # Zeus node subscribes to data from mid-tier nodes on its broker
    for mt_node in mid_tier_nodes:
        zeus_broker.subscribe(client_node=zeus_node, topic=f"processed_mid_tier/{mt_node.node_id}")
        zeus_broker.subscribe(client_node=zeus_node, topic=f"sensor_data/{mt_node.node_id}/#") # If mid-tiers forward raw data too

    print("-" * 50)
    print(f"[{time.strftime('%H:%M:%S')}] Simulation Setup Complete. Starting main loop for {args.duration}s...")
    print("-" * 50)

    # 2. Simulation Loop
    start_time = time.time()
    current_tick = 0
    try:
        while (time.time() - start_time) < args.duration:
            current_tick_time = time.time()
            simulation_time = time.time() - start_time
            
            print(f"\n[{time.strftime('%H:%M:%S')}] ----- Simulation Tick: {current_tick + 1} -----")

            # Tick all nodes (Sensor nodes will use their bridge to publish)
            for node in sensor_nodes + mid_tier_nodes + [zeus_node]:
                node.tick()
            
            # Publish simulation time to ROS2 if enabled
            if ros2_node:
                ros2_node.publish_time(simulation_time)
            
            current_tick += 1
            # Wait for the next tick, accounting for processing time
            time_to_sleep = args.tick - (time.time() - current_tick_time)
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)
            else:
                print(f"[{time.strftime('%H:%M:%S')}] Warning: Tick processing time ({-(time_to_sleep*1000):.2f}ms) exceeded interval {args.tick*1000:.0f}ms.")

    except KeyboardInterrupt:
        print(f"\n[{time.strftime('%H:%M:%S')}] Simulation interrupted by user.")
    finally:
        print("-" * 50)
        print(f"[{time.strftime('%H:%M:%S')}] Simulation Finished after {current_tick} ticks.")
        print("-" * 50)
        
        # Clean up ROS2 resources
        if args.ros2 and ROS2_AVAILABLE:
            try:
                if ros2_thread and ros2_thread.is_alive():
                    rclpy.shutdown()
                    ros2_thread.join(timeout=1.0)
                    
                # Terminate Gazebo Harmonic if it was launched
                if gazebo_process:
                    log.info("Shutting down Gazebo Harmonic...")
                    # First try a graceful termination
                    gazebo_process.terminate()
                    try:
                        gazebo_process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        log.warning("Gazebo Harmonic did not terminate gracefully, forcing shutdown...")
                        gazebo_process.kill()
                        try:
                            gazebo_process.wait(timeout=2)
                        except subprocess.TimeoutExpired:
                            log.error("Failed to kill Gazebo Harmonic process")
                    log.info("Gazebo Harmonic shutdown complete")
            except Exception as e:
                log.error(f"Error during ROS2/Gazebo shutdown: {e}")

if __name__ == "__main__":
    main()
