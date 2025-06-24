#!/usr/bin/env python3
"""
mmWave ROS2 Bridge Entry Point

This is the main entry point for the mmWave ROS2 bridge that connects
Gazebo mmWave sensor data to ROS2 PointCloud2 topics.

This file is a simplified entry point that uses the modular bridge implementation.
"""

import sys
import os
import logging

# Setup basic logging for this entry point
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
log = logging.getLogger('mmwave_bridge_entry')

# Log startup information
log.info("mmWave ROS2 Bridge starting")
log.info(f"Python version: {sys.version}")
log.info(f"Environment: {os.environ.get('ROS_DISTRO', 'ROS_DISTRO not set')}")

def main(args=None):
    """Main entry point for the ROS2 node."""
    # Initialize ROS2 first
    try:
        import rclpy
        log.info("Initializing ROS2...")
        rclpy.init(args=args)
        log.info("ROS2 initialized successfully.")
    except ImportError as e:
        log.error(f"Failed to import or initialize ROS2: {e}")
        log.error("Please ensure ROS2 is properly installed and sourced.")
        sys.exit(1)
    except Exception as e:
        log.error(f"An unexpected error occurred during ROS2 initialization: {e}")
        sys.exit(1)

    # Now, import and run the bridge node
    node = None
    try:
        log.info("Importing mmWave bridge package...")
        from mmwave_bridge import MmWaveRos2Bridge
        log.info("Successfully imported mmWave bridge package.")
        
        log.info("Creating MmWaveRos2Bridge node...")
        node = MmWaveRos2Bridge()
        log.info("Node created. Spinning...")
        
        rclpy.spin(node)
        
    except ImportError as e:
        log.error(f"Could not import MmWaveRos2Bridge: {e}")
        log.error("Ensure the bridge is installed and sys.path is correct.")
    except KeyboardInterrupt:
        log.info("Keyboard interrupt received, shutting down.")
    except Exception as e:
        log.error(f"An error occurred in the main node execution: {e}")
    finally:
        if node:
            log.info("Shutting down the node.")
            node.shutdown()
            node.destroy_node()
        log.info("Shutting down rclpy.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
