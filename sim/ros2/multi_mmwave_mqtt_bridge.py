#!/usr/bin/env python3
"""
Multi-mmWave ROS2 to MQTT Bridge for Olympus Simulation
Subscribes to multiple /mmwaveX/points topics and publishes presence detection to MQTT
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
logger = logging.getLogger('multi_mmwave_mqtt_bridge')

class MultiMmWaveMQTTBridge(Node):
    def __init__(self):
        super().__init__('multi_mmwave_mqtt_bridge')
        
        # Parameters
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('sensor_topics', ['/mmwave/points', '/mmwave2/points'])
        self.declare_parameter('sensor_ids', ['mmwave1', 'mmwave2'])
        self.declare_parameter('detection_threshold', 0.1)
        self.declare_parameter('max_detection_range', 10.0)
        self.declare_parameter('min_points_for_detection', 5)
        
        # Get parameters
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.sensor_topics = self.get_parameter('sensor_topics').value
        self.sensor_ids = self.get_parameter('sensor_ids').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.max_detection_range = self.get_parameter('max_detection_range').value
        self.min_points_for_detection = self.get_parameter('min_points_for_detection').value
        
        # Validate sensor configuration
        if len(self.sensor_topics) != len(self.sensor_ids):
            self.get_logger().error("Number of sensor topics must match number of sensor IDs")
            return
        
        # State tracking per sensor
        self.sensor_states = {}
        for sensor_id in self.sensor_ids:
            self.sensor_states[sensor_id] = {
                'last_detection_time': 0,
                'current_presence': False,
                'detection_cooldown': 0.5
            }
        
        # Setup MQTT client
        try:
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        except AttributeError:
            self.mqtt_client = mqtt.Client()
        
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # Create ROS2 Subscribers for each sensor
        self.subscribers = []
        for i, (topic, sensor_id) in enumerate(zip(self.sensor_topics, self.sensor_ids)):
            subscriber = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, sid=sensor_id: self.pointcloud_callback(msg, sid),
                10
            )
            self.subscribers.append(subscriber)
            self.get_logger().info(f"Subscribed to {topic} for sensor {sensor_id}")
        
        self.get_logger().info(f"Multi-mmWave MQTT Bridge starting...")
        self.get_logger().info(f"Sensors: {self.sensor_ids}")
        self.get_logger().info(f"Detection threshold: {self.detection_threshold}m")
        self.get_logger().info(f"Max range: {self.max_detection_range}m")
        self.get_logger().info(f"Min points: {self.min_points_for_detection}")
        
        # Connect to MQTT in a separate thread
        self.mqtt_thread = Thread(target=self.mqtt_connect)
        self.mqtt_thread.daemon = True
        self.mqtt_thread.start()
        
        # Timer for periodic status updates
        self.create_timer(10.0, self.publish_status)
        
    def mqtt_connect(self):
        """Connect to MQTT broker"""
        try:
            self.get_logger().info(f"Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_forever()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
    
    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        """Callback for MQTT connection"""
        if reason_code == 0:
            self.get_logger().info("Connected to MQTT broker successfully")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker: {reason_code}")
    
    def on_mqtt_disconnect(self, client, userdata, reason_code, properties):
        """Callback for MQTT disconnection"""
        self.get_logger().warning(f"Disconnected from MQTT broker: {reason_code}")
    
    def pointcloud_callback(self, msg, sensor_id):
        """Process point cloud data for a specific sensor"""
        try:
            current_time = time.time()
            sensor_state = self.sensor_states[sensor_id]
            
            # Check cooldown
            if current_time - sensor_state['last_detection_time'] < sensor_state['detection_cooldown']:
                return
            
            # Parse point cloud data
            points = self.parse_pointcloud(msg)
            
            if points is None or len(points) == 0:
                # No points detected
                if sensor_state['current_presence']:
                    self.publish_presence(sensor_id, False, 0, "No points detected")
                    sensor_state['current_presence'] = False
                return
            
            # Filter points by range
            valid_points = []
            for point in points:
                distance = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
                if self.detection_threshold <= distance <= self.max_detection_range:
                    valid_points.append(point)
            
            num_valid_points = len(valid_points)
            human_present = num_valid_points >= self.min_points_for_detection
            
            # Calculate centroid if points exist
            analysis = f"{num_valid_points} points"
            if valid_points:
                centroid = np.mean(valid_points, axis=0)
                centroid_distance = np.sqrt(centroid[0]**2 + centroid[1]**2 + centroid[2]**2)
                analysis += f", centroid at {centroid_distance:.2f}m"
            
            # Publish if state changed or periodic update
            if human_present != sensor_state['current_presence'] or current_time - sensor_state['last_detection_time'] > 2.0:
                self.publish_presence(sensor_id, human_present, num_valid_points, analysis)
                sensor_state['current_presence'] = human_present
                sensor_state['last_detection_time'] = current_time
                
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud for {sensor_id}: {e}")
    
    def parse_pointcloud(self, msg):
        """Parse PointCloud2 message and extract XYZ coordinates"""
        try:
            # Find field offsets
            x_offset = y_offset = z_offset = None
            for field in msg.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
            
            if x_offset is None or y_offset is None or z_offset is None:
                self.get_logger().error("Could not find x, y, z fields in point cloud")
                return None
            
            # Extract points
            points = []
            point_step = msg.point_step
            
            for i in range(msg.width * msg.height):
                base_idx = i * point_step
                
                # Extract x, y, z (assuming float32)
                x = struct.unpack('f', msg.data[base_idx + x_offset:base_idx + x_offset + 4])[0]
                y = struct.unpack('f', msg.data[base_idx + y_offset:base_idx + y_offset + 4])[0]
                z = struct.unpack('f', msg.data[base_idx + z_offset:base_idx + z_offset + 4])[0]
                
                # Skip invalid points
                if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                    continue
                
                points.append([x, y, z])
            
            return np.array(points) if points else None
            
        except Exception as e:
            self.get_logger().error(f"Error parsing point cloud: {e}")
            return None
    
    def publish_presence(self, sensor_id, human_present, num_points, analysis):
        """Publish human presence detection to MQTT"""
        try:
            topic = f"sensor/{sensor_id}/presence"
            
            message = {
                "timestamp": time.time(),
                "sensor_id": sensor_id,
                "human_present": human_present,
                "num_points": num_points,
                "analysis": analysis,
                "detection_range": {
                    "min": self.detection_threshold,
                    "max": self.max_detection_range
                }
            }
            
            payload = json.dumps(message)
            self.mqtt_client.publish(topic, payload, qos=1, retain=True)
            
            status = "DETECTED" if human_present else "CLEAR"
            self.get_logger().info(f"[{sensor_id}] {status}: {analysis}")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing presence for {sensor_id}: {e}")
    
    def publish_status(self):
        """Publish periodic status update"""
        try:
            for sensor_id in self.sensor_ids:
                topic = f"sensor/{sensor_id}/status"
                message = {
                    "timestamp": time.time(),
                    "sensor_id": sensor_id,
                    "status": "online",
                    "bridge_node": "multi_mmwave_mqtt_bridge"
                }
                payload = json.dumps(message)
                self.mqtt_client.publish(topic, payload, qos=0, retain=True)
                
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge = MultiMmWaveMQTTBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Bridge error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
