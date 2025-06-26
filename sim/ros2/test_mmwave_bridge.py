#!/usr/bin/env python3
"""
Test script for mmWave ROS2 Bridge

This script provides a simple way to test the mmWave ROS2 bridge
without launching the full Olympus simulation.
"""

import sys
import os
import time
import argparse
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
log = logging.getLogger('test_mmwave_bridge')

def main():
    """Main entry point for the test script."""
    parser = argparse.ArgumentParser(description='Test mmWave ROS2 Bridge')
    parser.add_argument('--test-mode', choices=['ros2-only', 'with-gazebo'], 
                        default='ros2-only',
                        help='Test mode: ros2-only publishes test data, with-gazebo attempts Gazebo connection')
    parser.add_argument('--duration', type=int, default=30,
                        help='Test duration in seconds')
    args = parser.parse_args()
    
    log.info(f"Starting mmWave bridge test in {args.test_mode} mode")
    
    # Add the parent directory to the path to find the mmwave_bridge package
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if script_dir not in sys.path:
        sys.path.append(script_dir)
    
    try:
        # Import our bridge first to ensure Gazebo libs are loaded before ROS2
        from mmwave_bridge import MmWaveRos2Bridge
        log.info("Successfully imported mmWave bridge package")

        # Then, import ROS2
        import rclpy
        log.info("Successfully imported ROS2")
        
        # Initialize ROS2
        rclpy.init()
        log.info("ROS2 initialized")
        
        # Create the bridge node
        bridge_node = MmWaveRos2Bridge()
        log.info("Bridge node created")
        
        # Run for the specified duration
        log.info(f"Running test for {args.duration} seconds...")
        
        # Use a separate thread for spinning
        import threading
        spin_thread = threading.Thread(target=lambda: rclpy.spin(bridge_node))
        spin_thread.daemon = True
        spin_thread.start()
        
        # Wait for the specified duration
        start_time = time.time()
        try:
            while time.time() - start_time < args.duration:
                time.sleep(1)
                elapsed = int(time.time() - start_time)
                if elapsed % 5 == 0:  # Log every 5 seconds
                    log.info(f"Test running for {elapsed}s...")
        except KeyboardInterrupt:
            log.info("Test interrupted by user")
        
        # Clean shutdown
        log.info("Test complete, shutting down")
        bridge_node.shutdown()
        bridge_node.destroy_node()
        rclpy.shutdown()
        
    except ImportError as e:
        log.error(f"Import error: {e}")
        log.error("Please ensure ROS2 and the mmWave bridge package are properly installed")
        return 1
    except Exception as e:
        log.error(f"Error: {e}")
        import traceback
        log.error(traceback.format_exc())
        return 1
    
    log.info("Test completed successfully")
    return 0

if __name__ == "__main__":
    sys.exit(main())
