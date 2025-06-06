#!/usr/bin/env python3
"""
Wrapper script to run the Olympus simulation with proper imports
"""

import os
import sys
import argparse

# Add the project root to the Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
sys.path.insert(0, project_root)

# Now we can import from sim.python modules using absolute imports
from sim.python.olympus_nodes import Node
from sim.python.mqtt_broker_sim import MQTTBrokerSim, MQTTSimulatedClient
from sim.python.hardware_sensor_bridge import HardwareSensorBridge

# Import the main simulation functions
import sim.python.core.main_simulation as main_sim

def main():
    """Run the Olympus simulation."""
    parser = argparse.ArgumentParser(description='Olympus Simulation')
    parser.add_argument('--duration', type=int, default=main_sim.SIMULATION_DURATION_SECONDS,
                      help='Simulation duration in seconds')
    parser.add_argument('--tick', type=float, default=main_sim.SIMULATION_TICK_INTERVAL,
                      help='Simulation tick interval in seconds')
    args = parser.parse_args()
    
    # Call the main simulation function with the parsed arguments
    main_sim.main(duration=args.duration, tick_interval=args.tick)

if __name__ == "__main__":
    main()
