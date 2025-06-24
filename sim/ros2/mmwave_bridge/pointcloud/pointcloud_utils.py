#!/usr/bin/env python3
"""
PointCloud utilities for the mmWave ROS2 bridge.

Provides functions for creating and manipulating ROS2 PointCloud2 messages.
"""

import numpy as np
import logging
from typing import Dict, List, Any, Optional, Tuple

log = logging.getLogger('mmwave_bridge.pointcloud')

# Try to import ROS2 components
try:
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    log.warning("ROS2 packages not found. Using dummy classes.")
    ROS2_AVAILABLE = False
    
    # Create dummy classes for testing without ROS2
    class PointField:
        FLOAT32 = 7
        
        def __init__(self, name='', offset=0, datatype=0, count=1):
            self.name = name
            self.offset = offset
            self.datatype = datatype
            self.count = count
    
    class PointCloud2:
        def __init__(self):
            self.header = Header()
            self.height = 0
            self.width = 0
            self.fields = []
            self.is_bigendian = False
            self.point_step = 0
            self.row_step = 0
            self.data = bytes()
            self.is_dense = True
    
    class Header:
        def __init__(self):
            self.stamp = {}
            self.frame_id = ''


def create_test_pointcloud(timestamp: Any) -> PointCloud2:
    """
    Create a test pattern pointcloud message.
    
    Args:
        timestamp: ROS Time object for the message header
        
    Returns:
        ROS2 PointCloud2 message
    """
    # Create a simple test pattern: a spiral of points
    num_points = 100
    theta = np.linspace(0, 6*np.pi, num_points)
    
    # Spiral pattern
    radius = 0.1 * theta
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    # Add some vertical variation
    z = 0.1 * np.sin(theta)
    
    # Calculate intensity - use distance from center
    intensity = np.ones_like(theta)
    
    # Pack data for pointcloud
    cloud_data = np.zeros(num_points, dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32)
    ])
    
    cloud_data['x'] = x
    cloud_data['y'] = y
    cloud_data['z'] = z
    cloud_data['intensity'] = intensity
    
    # Create pointcloud message
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = timestamp
    msg.header.frame_id = 'world'
    
    msg.height = 1
    msg.width = num_points
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16  # 4 bytes per float * 4 fields
    msg.row_step = msg.point_step * num_points
    msg.is_dense = True
    
    # Convert numpy array to bytes
    msg.data = cloud_data.tobytes()
    
    return msg


def create_point_field(name: str, offset: int, datatype: int) -> PointField:
    """
    Create a PointField object for a PointCloud2 message.
    
    Args:
        name: The name of the field
        offset: Byte offset in the point structure
        datatype: The datatype enum
        
    Returns:
        PointField object
    """
    return PointField(
        name=name,
        offset=offset,
        datatype=datatype,
        count=1
    )
