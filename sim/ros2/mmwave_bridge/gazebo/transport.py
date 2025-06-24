#!/usr/bin/env python3
"""
Gazebo Transport Integration

Handles all Gazebo transport communication for the mmWave ROS2 bridge.
Isolates the complexity of Gazebo transport from the main bridge node.
"""

import sys
import os
import time
import threading
import logging
import struct
import numpy as np
from typing import List, Optional, Dict, Any, Callable

# Setup logging
log = logging.getLogger('mmwave_bridge.gazebo')

# Try to import Gazebo transport components first to avoid protobuf conflicts
GAZEBO_AVAILABLE = False
try:
    # Add these paths to the Python path to help find modules
    if '/usr/lib/python3/dist-packages' not in sys.path:
        sys.path.append('/usr/lib/python3/dist-packages')
    
    # Try different Gazebo transport versions
    try:
        import gz.transport14 as gz_transport
        import gz.msgs11 as gz_msgs
        from gz.msgs11 import pointcloud_packed_pb2
        log.info("Found Gazebo Fortress (14) transport and msgs")
        log.info("Using Gazebo Transport 14 with Messages 11")
        GAZEBO_AVAILABLE = True
    except ImportError:
        try:
            import gz.transport13 as gz_transport
            import gz.msgs10 as gz_msgs
            from gz.msgs10 import pointcloud_packed_pb2
            log.info("Found Gazebo Harmonic (13) transport and msgs")
            GAZEBO_AVAILABLE = True
        except ImportError:
            log.warning("Could not import Gazebo transport packages")
            GAZEBO_AVAILABLE = False
except Exception as e:
    log.error(f"Error importing Gazebo transport: {e}")
    GAZEBO_AVAILABLE = False

# Import ROS2 components if available
try:
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    # Import dummy classes from our utils
    from ..pointcloud.pointcloud_utils import PointCloud2, PointField, Header


class GazeboTransport:
    """
    Handles Gazebo transport communication for mmWave sensor data.
    
    This class encapsulates all the complexity of subscribing to Gazebo topics
    and processing the data into a format suitable for ROS2 publication.
    """
    
    def __init__(self, topic: str, callback: Callable, ros_logger=None):
        """
        Initialize the Gazebo transport handler.
        
        Args:
            topic: Gazebo topic to subscribe to
            callback: Function to call with processed data
            ros_logger: Optional ROS logger for additional logging
        """
        self.topic = topic
        self.callback = callback
        self.ros_logger = ros_logger
        self.gz_node = None
        self.gz_spin_thread = None
        self.initialized = False
        
        # Log to both standard log and ROS logger if available
        self.log_info(f"Initializing Gazebo transport for topic: {topic}")
        
        if not GAZEBO_AVAILABLE:
            self.log_warning("Gazebo transport not available. Cannot subscribe to Gazebo topics.")
            return
            
        try:
            # Initialize Gazebo transport
            self.log_info("Creating Gazebo transport node")
            self.gz_node = gz_transport.Node()
            
            # Define a robust callback wrapper
            def callback_wrapper(raw_msg):
                try:
                    self.log_debug(f"Received data on {topic}")
                    # Process the message and pass to the provided callback
                    self.callback(raw_msg)
                except Exception as e:
                    self.log_error(f"Error in Gazebo callback: {e}")
                    import traceback
                    self.log_error(f"Full traceback: {traceback.format_exc()}")
            
            # Check if the topic exists
            try:
                topics = self.gz_node.topic_list()
                self.log_info(f"Available Gazebo topics at startup: {topics}")
                
                if self.topic in topics:
                    self.log_info(f"Found {self.topic} in available topics!")
                else:
                    # Check for similar topics
                    mmwave_related = [t for t in topics if 'mmwave' in t.lower()]
                    if mmwave_related:
                        self.log_info(f"Found mmWave-related topics: {mmwave_related}")
                        # If we find a better topic name, use it instead
                        if len(mmwave_related) == 1:
                            self.topic = mmwave_related[0]
                            self.log_info(f"Switching to use found topic: {self.topic}")
            except Exception as e:
                self.log_warning(f"Failed to list topics: {e}")
            
            # Subscribe to the topic
            try:
                self.log_info(f"Subscribing to Gazebo topic: {self.topic}")
                # Use the proper message type for subscription.
                # This is more robust and avoids potential bugs in the transport's type inference.
                self.log_info(f"Subscribing to Gazebo topic: {self.topic} with explicit message type.")
                msg_type = pointcloud_packed_pb2.PointCloudPacked
                success = self.gz_node.subscribe(msg_type, self.topic, callback_wrapper)
                
                if success:
                    self.log_info(f"Successfully subscribed to {self.topic}")
                    self.initialized = True
                    

                else:
                    self.log_error(f"Failed to subscribe to {self.topic}")
            except Exception as e:
                self.log_error(f"Error subscribing to Gazebo topic: {e}")
                import traceback
                self.log_error(traceback.format_exc())
                
        except Exception as e:
            self.log_error(f"Failed to initialize Gazebo transport: {e}")
            import traceback
            self.log_error(traceback.format_exc())
    
    def is_initialized(self) -> bool:
        """Check if Gazebo transport is initialized."""
        return self.initialized
    
    def check_topics(self):
        """Check available Gazebo topics for debugging."""
        if not GAZEBO_AVAILABLE or not self.gz_node:
            self.log_warning("Gazebo transport not available. Cannot list topics.")
            return []
            
        try:
            topics = self.gz_node.topic_list()
            self.log_info(f"Available Gazebo topics: {topics}")
            
            if self.topic not in topics:
                self.log_warning(f"Warning: {self.topic} not found in Gazebo topics")
                
                # Check for similar topics
                mmwave_related = [t for t in topics if 'mmwave' in t.lower()]
                if mmwave_related:
                    self.log_info(f"Found mmWave-related topics: {mmwave_related}")
            
            return topics
        except Exception as e:
            self.log_warning(f"Failed to list Gazebo topics: {e}")
            return []
    

    
    def create_pointcloud2_from_raw_data(self, raw_data: bytes) -> Optional[PointCloud2]:
        """
        Create a ROS2 PointCloud2 message from raw Gazebo PointCloudPacked binary data.
        
        This uses a simplified binary parsing approach rather than relying on Protobuf.
        
        Args:
            raw_data: Raw binary data from Gazebo
            
        Returns:
            ROS2 PointCloud2 message or None if parsing fails
        """
        if not ROS2_AVAILABLE:
            self.log_warning("ROS2 not available. Cannot create PointCloud2 message.")
            return None
            
        try:
            # Try to parse the raw data as a PointCloudPacked message
            # This is a simplified approach that may need to be adjusted based on the
            # actual binary format of the Gazebo PointCloudPacked message
            
            # Create a new PointCloud2 message
            msg = PointCloud2()
            msg.header = Header()
            msg.header.frame_id = 'world'  # Use the world frame
            
            # For now, just pass through the raw data and set some reasonable defaults
            # In a real implementation, we would parse the binary format properly
            
            # Assume the data is a sequence of XYZ points (3 floats per point)
            point_size = 3 * 4  # 3 floats * 4 bytes per float
            num_points = len(raw_data) // point_size
            
            if num_points > 0:
                msg.height = 1
                msg.width = num_points
                msg.fields = [
                    self._create_point_field('x', 0, PointField.FLOAT32),
                    self._create_point_field('y', 4, PointField.FLOAT32),
                    self._create_point_field('z', 8, PointField.FLOAT32)
                ]
                msg.is_bigendian = False
                msg.point_step = point_size
                msg.row_step = point_size * num_points
                msg.is_dense = True
                msg.data = raw_data
                
                self.log_debug(f"Created PointCloud2 with {num_points} points")
                return msg
            else:
                self.log_warning("No points found in raw data")
                return None
                
        except Exception as e:
            self.log_error(f"Error creating PointCloud2 from raw data: {e}")
            import traceback
            self.log_error(traceback.format_exc())
            return None
    
    def _create_point_field(self, name: str, offset: int, datatype: int) -> PointField:
        """
        Create a PointField object.
        
        Args:
            name: Field name
            offset: Byte offset in point structure
            datatype: Data type enum value
            
        Returns:
            PointField object
        """
        return PointField(
            name=name,
            offset=offset,
            datatype=datatype,
            count=1
        )
    
    def shutdown(self):
        """Clean shutdown of Gazebo transport."""
        self.log_info("Shutting down Gazebo transport")
        # Nothing specific needed for cleanup
    
    # Logging helpers that log to both standard log and ROS logger if available
    def log_info(self, msg: str):
        """Log info message to both standard log and ROS logger."""
        log.info(msg)
        if self.ros_logger:
            self.ros_logger.info(msg)
    
    def log_warning(self, msg: str):
        """Log warning message to both standard log and ROS logger."""
        log.warning(msg)
        if self.ros_logger:
            self.ros_logger.warning(msg)
    
    def log_error(self, msg: str):
        """Log error message to both standard log and ROS logger."""
        log.error(msg)
        if self.ros_logger:
            self.ros_logger.error(msg)
    
    def log_debug(self, msg: str):
        """Log debug message to both standard log and ROS logger."""
        log.debug(msg)
        if self.ros_logger:
            self.ros_logger.debug(msg)