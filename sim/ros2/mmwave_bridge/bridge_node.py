#!/usr/bin/env python3
"""
MmWave ROS2 Bridge Node

Main ROS2 node that bridges mmWave sensor data from Gazebo to ROS2 PointCloud2 topics.
"""

import sys
import os
import time
import logging
from typing import List, Optional, Dict, Any

from .utils.logging_utils import setup_logger
from .pointcloud.pointcloud_utils import create_test_pointcloud
from .gazebo.transport import GazeboTransport
from .conversions import gz_pointcloud_to_ros2

# Setup logging
log = setup_logger('mmwave_bridge')

# Global flags
ROS2_AVAILABLE = False

# Try to import ROS2 components
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    
    ROS2_AVAILABLE = True
except ImportError as e:
    log.error(f"ROS2 import error: {e}")
    log.warning("ROS2 packages not found. Running in dummy mode.")
    
    # Import dummy classes for testing without ROS2
    from .utils.dummy_ros2 import Node


class MmWaveRos2Bridge(Node):
    """Bridge between Gazebo transport and ROS2 for mmWave sensor data."""
    
    def __init__(self):
        """Initialize the bridge node."""
        try:
            super().__init__('mmwave_ros2_bridge')
            self.get_logger().info('mmWave Gazebo to ROS2 Bridge starting...')
        except Exception as e:
            log.error(f"Error initializing ROS2 node: {e}")
            return
        
        # Parameters
        self.mmwave_gazebo_topic = '/mmwave/points'
        self.mmwave_ros2_topic = '/mmwave/pointcloud'
        
        # Create ROS2 publisher for the pointcloud
        self.get_logger().info(f"Creating ROS2 publisher for topic: {self.mmwave_ros2_topic}")
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            self.mmwave_ros2_topic,
            10
        )
        self.get_logger().info(f"ROS2 publisher created successfully for {self.mmwave_ros2_topic}")
        
        # Flag to track if we've received data
        self.received_data = False
        
        # Initialize Gazebo transport only if requested
        self.gazebo_transport = None
        try:
            self.gazebo_transport = GazeboTransport(
                self.mmwave_gazebo_topic,
                self.on_gazebo_data,
                self.get_logger()
            )
            
            if self.gazebo_transport.is_initialized():
                self.get_logger().info("Gazebo transport initialized successfully")
            else:
                self.get_logger().warn("Gazebo transport not available. Will use test data.")
                
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Gazebo transport: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            
        # Add a timer to check if we're receiving data
        self.create_timer(5.0, self.check_data_reception)
        
        self.get_logger().info("mmWave Gazebo-ROS2 Bridge initialized")
    
    def on_gazebo_data(self, gz_msg):
        """Handle incoming Gazebo mmWave sensor data, convert, and publish to ROS2."""
        try:
            self.get_logger().debug(f"Received Gazebo message of type {type(gz_msg)}")

            # Convert Gazebo message to ROS2 PointCloud2 message
            ros2_msg = gz_pointcloud_to_ros2(gz_msg)

            # Publish to ROS2
            if ros2_msg is not None:
                self.pointcloud_publisher.publish(ros2_msg)
                self.get_logger().debug(f"Published PointCloud2 with {ros2_msg.width} points")
                self.received_data = True
            else:
                self.get_logger().warning("Failed to convert Gazebo message to ROS2 PointCloud2")
        except Exception as e:
            self.get_logger().error(f"Error processing Gazebo data: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
    
    def check_data_reception(self):
        """Periodic callback to check if we're receiving data from Gazebo."""
        if not self.received_data:
            self.get_logger().warning("No mmWave data received from Gazebo yet. NOT publishing test data.")
            # Disabled test data publishing to ensure we only see real data
            # self.publish_test_data()
            
            # Check available Gazebo topics and log them
            if self.gazebo_transport:
                topics = self.gazebo_transport.check_topics()
                self.get_logger().info(f"Available Gazebo topics: {topics}")
                self.get_logger().info(f"Looking for topic: {self.mmwave_gazebo_topic}")
                
                # Check if our topic exists or if there are any mmwave-related topics
                mmwave_topics = [t for t in topics if 'mmwave' in t.lower()]
                if mmwave_topics:
                    self.get_logger().info(f"Found mmWave-related topics: {mmwave_topics}")
                else:
                    self.get_logger().warning("No mmWave-related topics found in Gazebo")
                    
                # Log environment variables that might affect plugin loading
                import os
                self.get_logger().info(f"GZ_SIM_SYSTEM_PLUGIN_PATH={os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', 'not set')}")
                self.get_logger().info(f"GZ_SIM_RESOURCE_PATH={os.environ.get('GZ_SIM_RESOURCE_PATH', 'not set')}")
                self.get_logger().info(f"LIBGL_ALWAYS_SOFTWARE={os.environ.get('LIBGL_ALWAYS_SOFTWARE', 'not set')}")
                self.get_logger().info(f"WSL_DISTRO_NAME={os.environ.get('WSL_DISTRO_NAME', 'not set')}")
                
                # Log a hint about the issue
                self.get_logger().warning("If the mmWave plugin is not publishing data, check that:\n"
                                        "1. The plugin is properly loaded in Gazebo\n"
                                        "2. The plugin is configured to use ray casting mode\n"
                                        "3. The plugin has objects in its field of view\n"
                                        "4. The plugin is publishing to the expected topic")
                
                # Try to subscribe to any mmwave topic found
                if mmwave_topics and self.mmwave_gazebo_topic not in mmwave_topics:
                    new_topic = mmwave_topics[0]
                    self.get_logger().info(f"Attempting to subscribe to alternative topic: {new_topic}")
                    self.gazebo_transport.topic = new_topic
    
    def publish_test_data(self):
        """Publish test point cloud data when no Gazebo data is available."""
        try:
            # Create test pointcloud using the utility function
            msg = create_test_pointcloud(self.get_clock().now().to_msg())
            
            # Publish the test data
            self.pointcloud_publisher.publish(msg)
            self.get_logger().debug(f"Published test pointcloud with {msg.width} points")
        except Exception as e:
            self.get_logger().error(f"Error publishing test data: {e}")
    
    def shutdown(self):
        """Clean shutdown of the bridge."""
        if self.gazebo_transport:
            self.gazebo_transport.shutdown()


def main(args=None):
    """Main entry point."""
    log.info("Starting mmWave ROS2 Bridge")
    
    if not ROS2_AVAILABLE:
        log.error("ROS2 is required but not available. Exiting.")
        return
    
    try:
        log.info("Initializing ROS2")
        rclpy.init(args=args)
        log.info("ROS2 initialized")
        
        bridge_node = MmWaveRos2Bridge()
        log.info("Node created, starting spinning")
        
        try:
            log.info("Spinning node")
            rclpy.spin(bridge_node)
            log.info("Spin completed")
        except KeyboardInterrupt:
            log.info("Keyboard interrupt received")
        except Exception as e:
            log.error(f"Error during spin: {e}")
            import traceback
            log.error(traceback.format_exc())
        finally:
            log.info("Shutting down node")
            bridge_node.shutdown()
            bridge_node.destroy_node()
            log.info("Shutting down ROS2")
            rclpy.shutdown()
            log.info("ROS2 shutdown complete")
    except Exception as e:
        log.error(f"Error in main: {e}")
        import traceback
        log.error(traceback.format_exc())


if __name__ == '__main__':
    main()
