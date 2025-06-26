#!/usr/bin/env python3
"""
Launch script for mmWave ROS2 Bridge

This script provides a simple way to launch the mmWave ROS2 bridge
with proper environment setup.
"""

import os
import sys
import subprocess
import argparse
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
log = logging.getLogger('launch_mmwave_bridge')

def main():
    """Main entry point for the launch script."""
    parser = argparse.ArgumentParser(description='Launch mmWave ROS2 Bridge')
    parser.add_argument('--mode', choices=['normal', 'debug', 'test'], 
                        default='normal',
                        help='Launch mode: normal, debug (with extra logging), or test (with test data only)')
    parser.add_argument('--duration', type=int, default=0,
                        help='Run duration in seconds (0 for indefinite)')
    args = parser.parse_args()
    
    # Get the directory of this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Check if ROS2 is sourced
    if 'ROS_DISTRO' not in os.environ:
        log.warning("ROS2 environment not detected. Attempting to source ROS2...")
        try:
            # Try common ROS2 setup paths
            ros_setup_paths = [
                '/opt/ros/humble/setup.bash',
                '/opt/ros/foxy/setup.bash',
                '/opt/ros/galactic/setup.bash',
                '/opt/ros/rolling/setup.bash'
            ]
            
            for path in ros_setup_paths:
                if os.path.exists(path):
                    log.info(f"Found ROS2 setup at {path}")
                    # We can't directly source in Python, so we'll inform the user
                    log.info(f"Please source ROS2 first with: source {path}")
                    log.info("Then run this script again")
                    return 1
            
            log.error("Could not find ROS2 setup file. Please install ROS2 or source it manually.")
            return 1
        except Exception as e:
            log.error(f"Error checking for ROS2: {e}")
            return 1
    
    # Prepare the command
    if args.mode == 'test':
        # Use the test script
        cmd = [sys.executable, os.path.join(script_dir, 'test_mmwave_bridge.py')]
        if args.duration > 0:
            cmd.extend(['--duration', str(args.duration)])
    else:
        # Use the main bridge script
        cmd = [sys.executable, os.path.join(script_dir, 'mmwave_ros2_bridge.py')]
        
        # Add debug flag if needed
        if args.mode == 'debug':
            os.environ['MMWAVE_BRIDGE_DEBUG'] = '1'
            log.info("Debug mode enabled")
    
    # Launch the bridge
    log.info(f"Launching mmWave bridge in {args.mode} mode")
    log.info(f"Command: {' '.join(cmd)}")
    
    try:
        # Execute the command
        process = subprocess.run(cmd)
        return process.returncode
    except KeyboardInterrupt:
        log.info("Launch interrupted by user")
        return 0
    except Exception as e:
        log.error(f"Error launching bridge: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
