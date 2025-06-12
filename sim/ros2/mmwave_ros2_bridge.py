#!/usr/bin/env python3
"""Gazebo-ROS2 Bridge for mmWave Sensor
Subscribes to Gazebo transport topics for mmWave sensor data and republishes to ROS2
v2: Improved stability and error handling
"""

import sys
import os
import time
import threading
import logging
import struct
import numpy as np
from typing import List, Optional, Dict, Any

# Configure logging
logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
log = logging.getLogger('mmwave_ros2_bridge')

# Try to import ROS2 components
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    import std_msgs.msg
    ROS2_AVAILABLE = True
except ImportError:
    log.warning("ROS2 packages not found. Running in dummy mode.")
    ROS2_AVAILABLE = False
    
    # Create dummy classes for ROS2 when not available
    class Node:
        """Dummy ROS2 Node class when ROS2 is not available."""
        def __init__(self, name):
            self.name = name
            log.info(f"[DUMMY] Created ROS2 node '{name}'")
            
        def create_publisher(self, *args, **kwargs):
            return DummyPublisher()
            
        def get_logger(self):
            return DummyLogger()
            
    class DummyPublisher:
        """Dummy publisher when ROS2 is not available."""
        def publish(self, *args, **kwargs):
            pass
            
    class DummyLogger:
        """Dummy logger when ROS2 is not available."""
        def info(self, msg):
            log.info(msg)
            
        def warning(self, msg):
            log.warning(msg)
            
        def error(self, msg):
            log.error(msg)

# Try to import Gazebo transport components
try:
    # Add these paths to the Python path to help find modules
    if '/usr/lib/python3/dist-packages' not in sys.path:
        sys.path.append('/usr/lib/python3/dist-packages')
    
    # Use pure Python implementation of protobuf to avoid C++ extension issues
    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
    
    # Import the needed Gazebo packages
    import gz.transport14 as gz_transport
    import gz.msgs11 as gz_msgs
    
    GAZEBO_AVAILABLE = True
    log.info("Successfully imported Gazebo packages (gz.transport14, gz.msgs11)")
except ImportError as e:
    log.warning(f"Gazebo packages import failed: {e}")
    log.warning("Please ensure python3-gz-transport14 and python3-gz-msgs11 are installed")
    GAZEBO_AVAILABLE = False

class MmWaveGazeboROS2Bridge(Node):
    """Bridge between Gazebo transport and ROS2 for mmWave sensor data."""
    
    def __init__(self):
        # Initialize the Node
        try:
            super().__init__('mmwave_gazebo_ros2_bridge')
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
        
        # Print all available ROS2 topics for debugging
        try:
            import subprocess
            self.get_logger().info("Checking available ROS2 topics...")
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
            self.get_logger().info(f"Available ROS2 topics:\n{result.stdout}")
        except Exception as e:
            self.get_logger().warn(f"Failed to list ROS2 topics: {e}")
        
        self.get_logger().info("mmWave Gazebo to ROS2 Bridge starting...")
        
        # Flag to track if we've received data
        self.received_data = False
        
        # Initialize Gazebo transport only if available
        if GAZEBO_AVAILABLE:
            try:
                # Initialize Gazebo transport - simpler approach with fewer error points
                self.get_logger().info("Initializing Gazebo transport...")
                self.gz_node = gz_transport.Node()
                
                # Define a simple callback that delegates to our processing method
                def callback(raw_msg):
                    try:
                        self.get_logger().info(f"Data received on {self.mmwave_gazebo_topic}")
                        self.on_gazebo_data(raw_msg)
                        self.received_data = True
                    except Exception as e:
                        self.get_logger().error(f"Error in Gazebo callback: {e}")
                
                # Subscribe with intentionally empty message type to receive raw data
                # This avoids protobuf descriptor pool conflicts
                self.get_logger().info(f"Subscribing to Gazebo topic: {self.mmwave_gazebo_topic}")
                success = self.gz_node.subscribe(self.mmwave_gazebo_topic, callback, '')
                
                if not success:
                    self.get_logger().error(f"Failed to subscribe to {self.mmwave_gazebo_topic}")
                    # Don't return/exit - allow the node to keep running even if subscription failed
                    # It will periodically try to publish dummy data
                else:
                    self.get_logger().info(f"Successfully subscribed to {self.mmwave_gazebo_topic}")
                    
                # List available Gazebo topics for debugging
                try:
                    topics = self.gz_node.topic_list()
                    self.get_logger().info(f"Available Gazebo topics: {topics}")
                    
                    if self.mmwave_gazebo_topic not in topics:
                        self.get_logger().warning(f"Warning: {self.mmwave_gazebo_topic} not found in Gazebo topics")
                except Exception as e:
                    self.get_logger().warning(f"Failed to list Gazebo topics: {e}")
                
                # Start a thread to spin the Gazebo transport node
                self.gz_spin_thread = threading.Thread(target=self.spin_gz_node)
                self.gz_spin_thread.daemon = True
                self.gz_spin_thread.start()
                
                self.get_logger().info("mmWave Gazebo-ROS2 Bridge initialized with raw subscription")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize Gazebo transport: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
        else:
            self.get_logger().warn("Gazebo transport not available. mmWave bridge will not function.")
            
        # Add a timer to check if we're receiving data
        self.create_timer(5.0, self.check_data_reception)
        
        self.get_logger().info("mmWave Gazebo-ROS2 Bridge initialized")
    
    def on_gazebo_data(self, data):
        """Handle incoming Gazebo mmWave sensor data and publish to ROS2."""
        try:
            if data is None:
                self.get_logger().warn("Received None data from Gazebo")
                return
                
            # Try to determine data type
            data_type = type(data).__name__
            self.get_logger().debug(f"Processing data of type {data_type}")
            
            # If it's bytes or a string, treat as raw data
            if isinstance(data, (bytes, bytearray, str)):
                data_len = len(data)
                self.get_logger().debug(f"Processing raw binary data: {data_len} bytes")
            
            # Attempt to manually parse the raw Protobuf binary data
            # We'll extract XYZ points from the binary data without using Protobuf API
            try:
                # Create a ROS2 PointCloud2 message directly from raw bytes
                # This is a simplified parsing approach that works with basic point cloud data
                ros_msg = self.create_pointcloud2_from_raw_data(data)
                
                # Only publish if we successfully parsed points
                if ros_msg.width > 0:
                    # Publish to ROS2
                    self.pointcloud_publisher.publish(ros_msg)
                    self.get_logger().info(f"Published point cloud with {ros_msg.width} points")
                    
                    # Verify topic exists after publishing
                    try:
                        topic_list = self.get_topic_names_and_types()
                        topics_str = '\n - '.join([f"{t[0]}: {t[1]}" for t in topic_list])
                        self.get_logger().info(f"Available ROS2 topics after publishing:\n - {topics_str}")
                        
                        if f"{self.mmwave_ros2_topic}" in [t[0] for t in topic_list]:
                            self.get_logger().info(f"SUCCESS: Topic {self.mmwave_ros2_topic} is available in ROS2!")
                        else:
                            self.get_logger().warning(f"Topic {self.mmwave_ros2_topic} not found in ROS2 topic list after publishing")
                    except Exception as e:
                        self.get_logger().error(f"Error checking topic list: {str(e)}")
                else:
                    self.get_logger().warn("Failed to extract points from raw data")
            except Exception as e:
                self.get_logger().error(f"Error parsing/converting raw data: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
        except Exception as e:
            self.get_logger().error(f"Unhandled error in on_raw_gazebo_data: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def create_pointcloud2_from_raw_data(self, raw_data):
        """Create a ROS2 PointCloud2 message from raw Gazebo PointCloudPacked binary data.
           This uses a simplified binary parsing approach rather than relying on Protobuf."""
        from sensor_msgs.msg import PointField
        
        # Create a new ROS2 PointCloud2 message
        ros_msg = sensor_msgs.msg.PointCloud2()
        
        # Set header
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = 'mmwave_sensor'  # Default frame
        
        # Log raw data characteristics for debugging
        self.get_logger().info(f"Raw data type: {type(raw_data)}, length: {len(raw_data)}")
        if len(raw_data) > 20:
            self.get_logger().debug(f"First 20 bytes: {raw_data[:20]}")
        
        # Here we attempt to parse the Gazebo PointCloudPacked message format
        # PointCloudPacked format typically includes:
        # - Header information
        # - Number of points
        # - Point data (XYZ coordinates, possibly with intensity)
        try:
            # Try to decode as a Gazebo PointCloudPacked message
            # First locate byte patterns that might indicate the start of point data
            
            # Look for patterns that suggest float sequences
            # We expect groups of 3 consecutive float32 values (12 bytes per point)
            points = []
            
            # Starting at several possible offsets to find the beginning of point data
            # Common offsets include 16, 24, or 32 bytes from the start
            potential_offsets = [16, 24, 32, 48, 64]
            
            for offset in potential_offsets:
                if offset + 12 > len(raw_data):  # Need at least one point (3 floats)
                    continue
                    
                # Try to interpret the data as an array of XYZ triplets
                try:
                    # First, try to find the number of points from the header
                    # This is often stored as a uint32 at specific offsets
                    num_points_candidates = []
                    for i in range(0, min(16, len(raw_data) - 4), 4):
                        num_points = struct.unpack('<I', raw_data[i:i+4])[0]
                        if 0 < num_points < 100000:  # Reasonable range check
                            num_points_candidates.append((i, num_points))
                    
                    # If we found potential point counts, try the most likely one
                    if num_points_candidates:
                        _, num_points = max(num_points_candidates, key=lambda x: x[1])
                        self.get_logger().info(f"Detected approximately {num_points} points in message")
                        
                        # Ensure we have enough bytes for all points
                        data_size = num_points * 12  # 3 floats x 4 bytes per float
                        if offset + data_size <= len(raw_data):
                            points = []
                            for i in range(num_points):
                                point_offset = offset + (i * 12)
                                x, y, z = struct.unpack('<fff', raw_data[point_offset:point_offset+12])
                                points.append([x, y, z])
                            break
                    else:
                        # If we couldn't find a point count, try a fixed stride approach
                        stride = 12  # Assuming XYZ float32 values (4 bytes each)
                        for i in range(offset, len(raw_data) - stride + 1, stride):
                            # Unpack 3 consecutive float32 values
                            try:
                                x, y, z = struct.unpack('<fff', raw_data[i:i+stride])
                                
                                # Apply sanity checks to filter out invalid values
                                if (-1000 < x < 1000) and (-1000 < y < 1000) and (-1000 < z < 1000):
                                    points.append([float(x), float(y), float(z)])
                            except struct.error:
                                pass
                        
                        # If we found reasonable points, break
                        if len(points) > 0:
                            break
                except Exception as e:
                    self.get_logger().debug(f"Error parsing at offset {offset}: {str(e)}")
            
            # If no points found, try one more basic approach
            if not points:
                self.get_logger().warning("Initial parsing approaches failed, trying basic stride parsing")
                stride = 12  # Assuming XYZ float32 values (4 bytes each)
                for i in range(0, len(raw_data) - stride + 1, stride):
                    # Unpack 3 consecutive float32 values
                    try:
                        x, y, z = struct.unpack('<fff', raw_data[i:i+stride])
                        
                        # Apply sanity checks to filter out invalid values
                        if (-1000 < x < 1000) and (-1000 < y < 1000) and (-1000 < z < 1000):
                            points.append([float(x), float(y), float(z)])
                    except struct.error:
                        pass
            
            self.get_logger().info(f"Extracted {len(points)} points from Gazebo data")
            
            # Set fields for XYZ point cloud
            ros_msg.fields = [
                self._create_point_field('x', 0, PointField.FLOAT32),
                self._create_point_field('y', 4, PointField.FLOAT32),
                self._create_point_field('z', 8, PointField.FLOAT32)
            ]
            
            # Set point cloud properties
            ros_msg.point_step = 12  # 3 floats * 4 bytes
            ros_msg.width = len(points)
            ros_msg.height = 1
            ros_msg.row_step = ros_msg.point_step * ros_msg.width
            ros_msg.is_dense = True
            
            # Pack XYZ data into byte array
            buffer = bytearray(ros_msg.width * ros_msg.point_step)
            for i, point in enumerate(points):
                offset = i * ros_msg.point_step
                # Pack x, y, z as floats
                struct.pack_into('fff', buffer, offset, point[0], point[1], point[2])
            
            ros_msg.data = bytes(buffer)
            return ros_msg
            
        except Exception as e:
            self.get_logger().error(f"Error creating PointCloud2 from raw data: {e}")
            # Return minimal valid point cloud
            ros_msg.height = 1
            ros_msg.width = 0
            ros_msg.fields = [
                self._create_point_field('x', 0, PointField.FLOAT32),
                self._create_point_field('y', 4, PointField.FLOAT32),
                self._create_point_field('z', 8, PointField.FLOAT32)
            ]
            ros_msg.is_bigendian = False
            ros_msg.point_step = 12
            ros_msg.row_step = 0
            ros_msg.data = bytes()
            return ros_msg
    
    def _create_point_field(self, name, offset, datatype):
        """Helper to create a PointField object."""
        from sensor_msgs.msg import PointField
        field = PointField()
        field.name = name
        field.offset = offset
        field.datatype = datatype
        field.count = 1
        return field
    
    def spin_gz_node(self):
        """Spin the Gazebo transport node in a separate thread."""
        try:
            self.get_logger().info("Starting Gazebo transport spin thread")
            while rclpy.ok():
                try:
                    # Process Gazebo transport messages (non-blocking)
                    self.gz_node.run_once(timeout_ms=100)
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
                except Exception as e:
                    self.get_logger().error(f"Error processing Gazebo messages: {e}")
                    time.sleep(1.0)  # Longer sleep on error
        except Exception as e:
            self.get_logger().error(f"Fatal error in Gazebo spin thread: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            
    def check_data_reception(self):
        """Periodic callback to check if we're receiving data from Gazebo."""
        if not self.received_data:
            self.get_logger().warn("No mmWave data received from Gazebo yet. Publishing test data.")
            # Publish some test data to ensure the topic is active
            self.publish_test_data()
        else:
            self.get_logger().info("mmWave data is being received from Gazebo.")
            
    def publish_test_data(self):
        """Publish test point cloud data when no Gazebo data is available."""
        try:
            # Create a small test point cloud
            from sensor_msgs.msg import PointField, PointCloud2
            import math
            import time
            
            # Create a new ROS2 PointCloud2 message
            ros_msg = PointCloud2()
            
            # Set header
            ros_msg.header.stamp = self.get_clock().now().to_msg()
            ros_msg.header.frame_id = 'world'  # Using world frame for compatibility
            
            # Generate a simple pattern of points (a small spiral)
            num_points = 100
            points = []
            
            # Use current time to animate the pattern
            t = time.time() % 10.0  # Cycle every 10 seconds
            
            for i in range(num_points):
                angle = (i / num_points) * 2.0 * math.pi + t
                radius = 0.5 + (i / num_points) * 0.5
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                z = (i / num_points) * 0.5
                points.append([float(x), float(y), float(z)])
            
            # Set fields for XYZ point cloud
            ros_msg.fields = [
                self._create_point_field('x', 0, PointField.FLOAT32),
                self._create_point_field('y', 4, PointField.FLOAT32),
                self._create_point_field('z', 8, PointField.FLOAT32)
            ]
            
            # Set point cloud properties
            ros_msg.point_step = 12  # 3 floats * 4 bytes
            ros_msg.width = len(points)
            ros_msg.height = 1
            ros_msg.row_step = ros_msg.point_step * ros_msg.width
            ros_msg.is_dense = True
            
            # Pack XYZ data into byte array
            buffer = bytearray(ros_msg.width * ros_msg.point_step)
            for i, point in enumerate(points):
                offset = i * ros_msg.point_step
                # Pack x, y, z as floats
                struct.pack_into('fff', buffer, offset, point[0], point[1], point[2])
            
            ros_msg.data = bytes(buffer)
            
            # Publish the test data
            self.pointcloud_publisher.publish(ros_msg)
            self.get_logger().info(f"Published test point cloud with {len(points)} points")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing test data: {e}")
    
    def shutdown(self):
        """Clean shutdown of the bridge."""
        self.get_logger().info("Shutting down mmWave Gazebo-ROS2 Bridge")

def main(args=None):
    # Check if ROS2 is available
    if not ROS2_AVAILABLE:
        log.error("ROS2 is not available. Cannot run the mmWave Gazebo-ROS2 bridge.")
        log.error("Please install ROS2 or fix your ROS2 environment setup.")
        return
    
    # Check if Gazebo transport is available
    if not GAZEBO_AVAILABLE:
        log.error("Gazebo transport is not available. Cannot run the mmWave Gazebo-ROS2 bridge.")
        log.error("Please install Gazebo transport Python bindings (e.g., python3-gz-transport14, python3-gz-msgs11).")
        return
    
    # Normal ROS2 operation
    try:
        rclpy.init(args=args)
        node = MmWaveGazeboROS2Bridge()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.shutdown()
            node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        log.error(f"Error in ROS2 initialization: {e}")

if __name__ == '__main__':
    main()
