#!/usr/bin/env python3
"""
mmWave ROS2 to MQTT Bridge for Olympus Simulation
Subscribes to /mmwave/points (PointCloud2) and publishes human presence detection to MQTT
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import paho.mqtt.client as mqtt
import json
import numpy as np
import struct
import time
from threading import Thread
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('mmwave_mqtt_bridge')

class MmWaveMQTTBridge(Node):
    def __init__(self):
        super().__init__('mmwave_mqtt_bridge')
        
        # Parameters
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('sensor_id', 'mmwave1')
        self.declare_parameter('detection_threshold', 0.1)  # Minimum distance for detection
        self.declare_parameter('max_detection_range', 10.0)  # Maximum range in meters
        self.declare_parameter('min_points_for_detection', 5)  # Minimum points to consider detection
        
        # Get parameters
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.sensor_id = self.get_parameter('sensor_id').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.max_detection_range = self.get_parameter('max_detection_range').value
        self.min_points_for_detection = self.get_parameter('min_points_for_detection').value
        
        # State tracking
        self.last_detection_time = 0
        self.detection_cooldown = 0.5  # Seconds between detections
        self.current_presence = False
        
        # Setup MQTT client - compatible with older paho-mqtt versions
        try:
            # Try new API first (paho-mqtt >= 2.0)
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        except AttributeError:
            # Fall back to old API (paho-mqtt < 2.0)
            self.mqtt_client = mqtt.Client()
        
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # ROS2 Subscriber
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/mmwave/points',
            self.pointcloud_callback,
            10
        )
        
        self.get_logger().info(f"mmWave MQTT Bridge starting...")
        self.get_logger().info(f"Sensor ID: {self.sensor_id}")
        self.get_logger().info(f"Detection threshold: {self.detection_threshold}m")
        self.get_logger().info(f"Max range: {self.max_detection_range}m")
        self.get_logger().info(f"Min points: {self.min_points_for_detection}")
        
        # Connect to MQTT in a separate thread
        self.mqtt_thread = Thread(target=self.mqtt_connect)
        self.mqtt_thread.daemon = True
        self.mqtt_thread.start()
        
        # Timer for periodic status updates
        self.create_timer(5.0, self.publish_status)
        
    def mqtt_connect(self):
        """Connect to MQTT broker"""
        try:
            self.get_logger().info(f"Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_forever()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
    
    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        """Callback for MQTT connection - compatible with both old and new paho-mqtt versions"""
        if reason_code == 0:
            self.get_logger().info("Connected to MQTT broker successfully")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker: {reason_code}")
    
    def on_mqtt_disconnect(self, client, userdata, reason_code, properties):
        """Callback for MQTT disconnection"""
        self.get_logger().warning(f"Disconnected from MQTT broker: {reason_code}")
    
    def pointcloud_callback(self, msg):
        """Process incoming PointCloud2 messages"""
        try:
            # Parse point cloud data
            points = self.parse_pointcloud2(msg)
            
            if points is None or len(points) == 0:
                self.publish_presence(False, 0, "No points detected")
                return
            
            # Analyze points for human presence
            presence_detected, num_points, analysis = self.analyze_presence(points)
            
            # Publish presence detection
            self.publish_presence(presence_detected, num_points, analysis)
            
            # Log detection events
            if presence_detected != self.current_presence:
                self.get_logger().info(f"Presence changed: {presence_detected} ({analysis})")
                self.current_presence = presence_detected
                
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")
    
    def parse_pointcloud2(self, msg):
        """Parse PointCloud2 message to extract XYZ coordinates"""
        try:
            # Get point step and field info
            point_step = msg.point_step
            
            # Find XYZ field offsets
            x_offset = y_offset = z_offset = None
            for field in msg.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
            
            if None in [x_offset, y_offset, z_offset]:
                self.get_logger().warning("Could not find XYZ fields in point cloud")
                return None
            
            # Extract points
            points = []
            data = msg.data
            
            for i in range(0, len(data), point_step):
                if i + point_step <= len(data):
                    # Extract XYZ coordinates (assuming float32)
                    x = struct.unpack('f', data[i + x_offset:i + x_offset + 4])[0]
                    y = struct.unpack('f', data[i + y_offset:i + y_offset + 4])[0]
                    z = struct.unpack('f', data[i + z_offset:i + z_offset + 4])[0]
                    
                    # Filter out invalid points
                    if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                        points.append([x, y, z])
            
            return np.array(points) if points else None
            
        except Exception as e:
            self.get_logger().error(f"Error parsing point cloud: {e}")
            return None
    
    def analyze_presence(self, points):
        """Analyze point cloud for human presence detection"""
        if points is None or len(points) == 0:
            return False, 0, "No points"
        
        # Calculate distances from sensor origin
        distances = np.sqrt(np.sum(points**2, axis=1))
        
        # Filter points within detection range
        valid_points = points[
            (distances >= self.detection_threshold) & 
            (distances <= self.max_detection_range)
        ]
        
        num_valid_points = len(valid_points)
        
        if num_valid_points < self.min_points_for_detection:
            return False, num_valid_points, f"Too few points ({num_valid_points})"
        
        # Calculate centroid of detected points
        centroid = np.mean(valid_points, axis=0)
        centroid_distance = np.sqrt(np.sum(centroid**2))
        
        # Simple presence detection based on point density and distance
        presence_detected = True
        analysis = f"{num_valid_points} points, centroid at {centroid_distance:.2f}m"
        
        return presence_detected, num_valid_points, analysis
    
    def publish_presence(self, presence, num_points, analysis):
        """Publish presence detection to MQTT"""
        try:
            # Prepare MQTT message
            mqtt_topic = f"sensor/{self.sensor_id}/presence"
            
            payload = {
                "timestamp": time.time(),
                "sensor_id": self.sensor_id,
                "human_present": presence,
                "num_points": num_points,
                "analysis": analysis,
                "detection_range": {
                    "min": self.detection_threshold,
                    "max": self.max_detection_range
                }
            }
            
            # Publish to MQTT
            self.mqtt_client.publish(mqtt_topic, json.dumps(payload), qos=1)
            
            # Also publish simple boolean for automation
            simple_topic = f"sensor/{self.sensor_id}/human_present"
            self.mqtt_client.publish(simple_topic, str(presence).lower(), qos=1)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing to MQTT: {e}")
    
    def publish_status(self):
        """Publish periodic status updates"""
        try:
            status_topic = f"sensor/{self.sensor_id}/status"
            status_payload = {
                "timestamp": time.time(),
                "sensor_id": self.sensor_id,
                "status": "online",
                "bridge_active": True,
                "current_presence": self.current_presence
            }
            
            self.mqtt_client.publish(status_topic, json.dumps(status_payload), qos=1)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down mmWave MQTT Bridge")
        try:
            self.mqtt_client.disconnect()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge = MmWaveMQTTBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    except Exception as e:
        logger.error(f"Error in main: {e}")
    finally:
        if 'bridge' in locals():
            bridge.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
