#!/usr/bin/env python3
"""Gazebo-ROS2 Bridge for mmWave Sensor
Subscribes to Gazebo transport topics for mmWave sensor data and republishes to ROS2
v3: Enhanced debugging and subscription reliability
"""

import sys
import os
import time
import threading
import logging
import struct
import numpy as np
from typing import List, Optional, Dict, Any

# Setup logging with both console and file output
log = logging.getLogger('mmwave_ros2_bridge')
log.setLevel(logging.DEBUG)  # Set to debug level

# Console handler
console_handler = logging.StreamHandler()
console_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
log.addHandler(console_handler)

# File handler - log to a file for detailed debugging
log_file = os.path.expanduser('~/mmwave_bridge_debug.log')
file_handler = logging.FileHandler(log_file, mode='w')
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
log.addHandler(file_handler)

# Log startup information
log.info(f"mmWave ROS2 Bridge starting, logging to {log_file}")
log.info(f"Python version: {sys.version}")
log.info(f"Environment: {os.environ.get('ROS_DISTRO', 'ROS_DISTRO not set')}")

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
                
                # Define a more robust callback that delegates to our processing method
                def callback(raw_msg):
                    try:
                        self.get_logger().info(f"Data received on {self.mmwave_gazebo_topic} with type {type(raw_msg)}")
                        
                        # Debug log the first message received to understand its format
                        if not self.received_data:
                            if hasattr(raw_msg, '__str__'):
                                self.get_logger().info(f"First message sample: {str(raw_msg)[:200]}...")
                            if hasattr(raw_msg, 'SerializeToString'):
                                self.get_logger().info(f"Message has SerializeToString method")
                            if hasattr(raw_msg, 'data'):
                                self.get_logger().info(f"Message has data field with length: {len(raw_msg.data)}")
                            if hasattr(raw_msg, 'point_step'):
                                self.get_logger().info(f"Message point_step: {raw_msg.point_step}")
                        
                        # Process the message
                        self.on_gazebo_data(raw_msg)
                        self.received_data = True
                    except Exception as e:
                        self.get_logger().error(f"Error in Gazebo callback: {e}")
                        import traceback
                        self.get_logger().error(f"Full traceback: {traceback.format_exc()}")
                
                # First try to subscribe with proper message type
                self.get_logger().info(f"Subscribing to Gazebo topic: {self.mmwave_gazebo_topic}")
                
                # First check if the topic exists
                try:
                    topics = self.gz_node.topic_list()
                    self.get_logger().info(f"Available Gazebo topics at startup: {topics}")
                    
                    if self.mmwave_gazebo_topic in topics:
                        self.get_logger().info(f"Found {self.mmwave_gazebo_topic} in available topics!")
                    else:
                        # Check for similar topics
                        mmwave_related = [t for t in topics if 'mmwave' in t.lower()]
                        if mmwave_related:
                            self.get_logger().info(f"Found mmWave-related topics: {mmwave_related}")
                            # If we find a better topic name, use it instead
                            if len(mmwave_related) == 1:
                                self.mmwave_gazebo_topic = mmwave_related[0]
                                self.get_logger().info(f"Switching to use found topic: {self.mmwave_gazebo_topic}")
                        
                        # Check for point cloud topics
                        point_topics = [t for t in topics if 'point' in t.lower() or 'cloud' in t.lower()]
                        if point_topics:
                            self.get_logger().info(f"Found point cloud topics: {point_topics}")
                            
                        self.get_logger().warning(f"Warning: Topic {self.mmwave_gazebo_topic} not found in available topics")
                        self.get_logger().warning("Will subscribe anyway in case topic appears later")
                except Exception as e:
                    self.get_logger().warning(f"Failed to list topics: {e}")
                
                # We need to use a real message type object, not a string
                # First try with empty string message type (most compatible)
                try:
                    self.get_logger().info("Subscribing with empty message type string")
                    success = self.gz_node.subscribe(self.mmwave_gazebo_topic, callback, '')
                except Exception as e:
                    self.get_logger().error(f"Failed to subscribe with empty message type: {e}")
                    success = False
                    
                # If that didn't work, try alternative methods
                if not success:
                    try:
                        # Try to import the actual message type if possible
                        from gz.msgs11 import pointcloud_packed_pb2
                        self.get_logger().info("Using actual Protobuf message class for subscription")
                        pc_msg_class = pointcloud_pb2.PointCloudPacked
                        success = self.gz_node.subscribe(self.mmwave_gazebo_topic, callback, pc_msg_class)
                    except Exception as e:
                        self.get_logger().error(f"Failed to subscribe with message class: {e}")
                        try:
                            self.get_logger().info("Falling back to raw subscription method")
                            # Some versions support subscribe_raw
                            if hasattr(self.gz_node, 'subscribe_raw'):
                                success = self.gz_node.subscribe_raw(self.mmwave_gazebo_topic, callback)
                            else:
                                self.get_logger().error("subscribe_raw method not available")
                                success = False
                        except Exception as e:
                            self.get_logger().error(f"Failed to subscribe with raw method: {e}")
                            success = False
                
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
            self.get_logger().info(f"Processing data of type {data_type}")
            
            # Handle different message types
            if hasattr(data, 'data') and hasattr(data, 'point_step') and hasattr(data, 'width'):
                # It's a proper PointCloudPacked message
                self.get_logger().info(f"Received proper PointCloudPacked with {data.width} points")
                ros_msg = self.convert_gz_pointcloud_to_ros2(data)
            elif isinstance(data, (bytes, bytearray, str)):
                # If it's bytes or a string, treat as raw data
                data_len = len(data)
                self.get_logger().info(f"Processing raw binary data: {data_len} bytes")
                ros_msg = self.create_pointcloud2_from_raw_data(data)
            else:
                # Try to access common attributes for debugging
                self.get_logger().warning(f"Received unknown message format of type {data_type}")
                # Try to extract any fields/methods that might help us understand the format
                for attr in dir(data):
                    if not attr.startswith('_'):
                        try:
                            value = getattr(data, attr)
                            if not callable(value):
                                self.get_logger().info(f"Attribute {attr}: {value}")
                        except Exception:
                            pass
                # Fall back to raw data conversion
                if hasattr(data, 'SerializeToString'):
                    self.get_logger().info("Converting protobuf message to bytes")
                    bin_data = data.SerializeToString()
                    ros_msg = self.create_pointcloud2_from_raw_data(bin_data)
                else:
                    self.get_logger().error("Cannot process this message format")
                    return
            
            # Only publish if we successfully parsed points
            if ros_msg and ros_msg.width > 0:
                # Publish to ROS2
                self.pointcloud_publisher.publish(ros_msg)
                self.get_logger().info(f"Published point cloud with {ros_msg.width} points from Gazebo data")
                return True
            else:
                self.get_logger().warn("Failed to extract points from data")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Unhandled error in on_gazebo_data: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
            
    def convert_gz_pointcloud_to_ros2(self, gz_pc):
        """Convert a Gazebo PointCloudPacked message to ROS2 PointCloud2"""
        from sensor_msgs.msg import PointField
        
        # Create a new ROS2 PointCloud2 message
        ros_msg = sensor_msgs.msg.PointCloud2()
        
        # Set header
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = 'world'  # Use world frame to match RViz config
        
        # Copy fields from Gazebo message to ROS2 message
        ros_msg.height = gz_pc.height
        ros_msg.width = gz_pc.width
        ros_msg.point_step = gz_pc.point_step
        ros_msg.row_step = gz_pc.row_step
        ros_msg.is_dense = gz_pc.is_dense
        
        # Set up field definitions for x, y, z
        ros_msg.fields = []
        
        # Copy fields from Gazebo message
        for gz_field in gz_pc.field:
            ros_field = PointField()
            ros_field.name = gz_field.name
            ros_field.offset = gz_field.offset
            ros_field.datatype = gz_field.datatype  # May need mapping between Gazebo and ROS2 types
            ros_field.count = gz_field.count
            ros_msg.fields.append(ros_field)
            
        # Copy the actual point data
        if isinstance(gz_pc.data, bytes) or isinstance(gz_pc.data, bytearray):
            ros_msg.data = gz_pc.data
        else:
            # Handle case where data might be in a different format
            self.get_logger().warning(f"Data is not bytes but {type(gz_pc.data)}")
            ros_msg.data = bytes(gz_pc.data)  # Try to convert to bytes
            
        return ros_msg
    
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
            # The Python bindings for Gazebo transport don't need explicit spinning
            # The subscription callback will be called automatically when messages arrive
            # We just need to keep this thread alive to prevent the Python process from exiting
            while rclpy.ok():
                try:
                    # Just sleep periodically - callbacks are handled automatically by the Gazebo transport library
                    time.sleep(0.1)  # Sleep to prevent CPU hogging while keeping thread responsive
                except Exception as e:
                    self.get_logger().error(f"Error in Gazebo transport thread: {e}")
                    time.sleep(1.0)  # Longer sleep on error
        except Exception as e:
            self.get_logger().error(f"Fatal error in Gazebo spin thread: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            
    def check_data_reception(self):
        """Periodic callback to check if we're receiving data from Gazebo."""
        # Check if we've received data from Gazebo topic
        if not self.received_data:
            # Debug: check available Gazebo topics again
            if GAZEBO_AVAILABLE:
                try:
                    # Check what topics are being published in Gazebo
                    topics = self.gz_node.topic_list()
                    topic_str = "\n   - ".join(topics) if topics else "None"
                    self.get_logger().info(f"Available Gazebo topics (check_data_reception): \n   - {topic_str}")
                    
                    # Check specifically for mmWave topic
                    if self.mmwave_gazebo_topic in topics:
                        self.get_logger().info(f"FOUND: {self.mmwave_gazebo_topic} exists but no data received")
                        
                        # Try to diagnose why data isn't being received
                        try:
                            # Try to get topic info
                            if hasattr(self.gz_node, 'topic_info'):
                                topic_info = self.gz_node.topic_info(self.mmwave_gazebo_topic)
                                self.get_logger().info(f"Topic info: {topic_info}")
                                
                            # Try to resubscribe with different approach
                            self.get_logger().info(f"Attempting direct data echo from topic {self.mmwave_gazebo_topic}...")
                            import subprocess
                            echo_cmd = ["gz", "topic", "-e", "-n", "1", self.mmwave_gazebo_topic]
                            result = subprocess.run(echo_cmd, capture_output=True, text=True, timeout=1)
                            if result.stdout.strip():
                                self.get_logger().info(f"Echo result: {result.stdout[:100]}...")
                            else:
                                self.get_logger().warning(f"No data from echo: {result.stderr}")
                        except Exception as e:
                            self.get_logger().warning(f"Diagnostic error: {e}")
                        
                        # Get topic info if possible
                        try:
                            if hasattr(self.gz_node, 'topic_info'):
                                info = self.gz_node.topic_info(self.mmwave_gazebo_topic)
                                self.get_logger().info(f"Topic info: {info}")
                        except Exception as e:
                            self.get_logger().warning(f"Couldn't get topic info: {e}")
                        
                        # Attempt to resubscribe with explicit Protobuf message class
                        try:
                            def callback(raw_msg):
                                try:
                                    self.get_logger().info(f"Data received on {self.mmwave_gazebo_topic} (resubscribe)")
                                    self.on_gazebo_data(raw_msg)
                                    self.received_data = True
                                except Exception as e:
                                    self.get_logger().error(f"Error in resubscribe callback: {e}")
                                    import traceback
                                    self.get_logger().error(f"Callback traceback: {traceback.format_exc()}")
                            
                            # Try different subscription approaches in sequence
                            self.get_logger().info("Resubscribing to topic with empty message type")
                            success = self.gz_node.subscribe(self.mmwave_gazebo_topic, callback, '')
                            
                            if not success:
                                self.get_logger().warning("Resubscription with empty type failed, trying with explicit module")
                                try:
                                    from gz.msgs11 import pointcloud_packed_pb2
                                    pc_class = pointcloud_packed_pb2.PointCloudPacked
                                    success = self.gz_node.subscribe(self.mmwave_gazebo_topic, callback, pc_class)
                                except Exception as e:
                                    self.get_logger().error(f"Failed with explicit message class: {e}")
                                    success = False
                        except Exception as e:
                            self.get_logger().error(f"Error during resubscription: {e}")
                            success = False
                    else:
                        # Check for similar topics if exact name not found
                        mmwave_topics = [t for t in topics if 'mmwave' in t.lower()]
                        if mmwave_topics:
                            self.get_logger().warning(f"Exact topic not found, but found mmWave-related topics: {mmwave_topics}")
                            
                        # Look for any pointcloud topics as alternatives
                        pointcloud_topics = [t for t in topics if 'point' in t.lower() or 'cloud' in t.lower()]
                        if pointcloud_topics:
                            self.get_logger().warning(f"Found these potential point cloud topics: {pointcloud_topics}")
                        
                        self.get_logger().warning(f"{self.mmwave_gazebo_topic} not found in available topics")
                except Exception as e:
                    self.get_logger().error(f"Error checking Gazebo topics: {e}")
                    import traceback
                    self.get_logger().error(f"Topic check traceback: {traceback.format_exc()}")
            
            # Check if system is running the necessary Gazebo processes
            try:
                import subprocess
                result = subprocess.run(['pgrep', '-f', 'gz sim'], capture_output=True, text=True)
                if result.stdout.strip():
                    self.get_logger().info(f"Gazebo processes running: {result.stdout.strip()}")
                else:
                    self.get_logger().warning("No Gazebo processes detected running")
            except Exception as e:
                self.get_logger().error(f"Failed to check for Gazebo processes: {e}")
            
            # Check which Gazebo processes are running
            try:
                import subprocess
                result = subprocess.run(['pgrep', '-fa', 'gz'], capture_output=True, text=True)
                if result.stdout.strip():
                    self.get_logger().info(f"Gazebo processes: {result.stdout.strip()[:200]}...")
                else:
                    self.get_logger().warning("No Gazebo processes detected")
            except Exception as e:
                self.get_logger().warning(f"Process check error: {e}")
                
            self.get_logger().warn("No mmWave data received from Gazebo yet. Publishing test data.")
            # Publish test data as a fallback
            try:
                self.publish_test_data()
                self.get_logger().info("Published test pointcloud data successfully")
            except Exception as e:
                self.get_logger().error(f"Error publishing test data: {e}")
                import traceback
                self.get_logger().error(f"Test data error: {traceback.format_exc()}")
                
        # Reset flag to track continuous reception
        self.received_data = False
        
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
