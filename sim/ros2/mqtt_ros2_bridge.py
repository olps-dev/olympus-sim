#!/usr/bin/env python3
"""
MQTT-ROS2 Bridge for Olympus Simulation
Connects the existing MQTT-based simulation with ROS2 topics
"""

import json
import time
import threading
import sys
import os
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
log = logging.getLogger('mqtt_ros2_bridge')

# Try to import ROS2 components
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float64, Bool
    from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure, BatteryState
    from geometry_msgs.msg import Pose, PoseStamped
    ROS2_AVAILABLE = True
except ImportError:
    log.warning("ROS2 packages not found. Running in MQTT-only mode.")
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

# Try to import MQTT client
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    log.warning("MQTT client not found. Please install paho-mqtt.")
    MQTT_AVAILABLE = False

class MQTTToROS2Bridge(Node):
    """Bridge between MQTT and ROS2 for the Olympus simulation."""
    
    def __init__(self):
        # Initialize the Node
        try:
            super().__init__('mqtt_ros2_bridge')
            self.get_logger().info('MQTT to ROS2 Bridge starting...')
        except Exception as e:
            log.error(f"Error initializing ROS2 node: {e}")
            return
        
        # Configuration parameters
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic_prefix', 'olympus/')
        
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.mqtt_topic_prefix = self.get_parameter('mqtt_topic_prefix').value
        
        # ROS2 publishers - created dynamically as needed
        self.ros_publishers = {}
        
        # Initialize MQTT client if available
        if MQTT_AVAILABLE:
            try:
                self.mqtt_client = mqtt.Client()
                self.mqtt_client.on_connect = self.on_mqtt_connect
                self.mqtt_client.on_message = self.on_mqtt_message
                
                # Connect to MQTT broker
                try:
                    self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
                    self.mqtt_client.loop_start()
                    self.get_logger().info(f"Connected to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
                except Exception as e:
                    self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize MQTT client: {e}")
        else:
            self.get_logger().warning('MQTT client not available. Running without MQTT.')
            raise
        
        # Create a timer for heartbeat
        self.create_timer(1.0, self.publish_heartbeat)
        
        self.get_logger().info("MQTT-ROS2 Bridge initialized")
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Handle MQTT connection."""
        if rc == 0:
            self.get_logger().info(f"Connected to MQTT broker with result code {rc}")
            
            # Subscribe to all sensor data topics
            topic = f"{self.mqtt_topic_prefix}sensor_data/#"
            client.subscribe(topic)
            self.get_logger().info(f"Subscribed to {topic}")
            
            # Subscribe to processed mid-tier data
            topic = f"{self.mqtt_topic_prefix}processed_mid_tier/#"
            client.subscribe(topic)
            self.get_logger().info(f"Subscribed to {topic}")
            
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker with result code {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages and publish to ROS2."""
        try:
            # Parse the topic to determine the message type
            topic = msg.topic
            
            # Remove prefix if present
            if topic.startswith(self.mqtt_topic_prefix):
                topic = topic[len(self.mqtt_topic_prefix):]
            
            # Parse the payload
            try:
                payload = json.loads(msg.payload.decode())
            except json.JSONDecodeError:
                # If not JSON, treat as plain text
                payload = {"text": msg.payload.decode()}
            
            # Process based on topic structure
            topic_parts = topic.split('/')
            
            if len(topic_parts) >= 3 and topic_parts[0] == "sensor_data":
                # Handle sensor data: sensor_data/sensor-id/sensor-type
                sensor_id = topic_parts[1]
                sensor_type = topic_parts[2]
                
                self.process_sensor_data(sensor_id, sensor_type, payload)
                
            elif len(topic_parts) >= 2 and topic_parts[0] == "processed_mid_tier":
                # Handle processed data: processed_mid_tier/mid-tier-id
                mid_tier_id = topic_parts[1]
                
                self.process_mid_tier_data(mid_tier_id, payload)
                
            else:
                # Generic handling for other topics
                ros_topic = f"/olympus/{topic.replace('/', '_')}"
                self.publish_generic_message(ros_topic, payload)
                
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")
    
    def process_sensor_data(self, sensor_id: str, sensor_type: str, payload: dict):
        """Process sensor data and publish to appropriate ROS2 topics."""
        base_topic = f"/olympus/sensor/{sensor_id}"
        
        if sensor_type == "bme680":
            # Temperature
            if "temperature_c" in payload:
                topic = f"{base_topic}/temperature"
                if topic not in self.ros_publishers:
                    self.ros_publishers[topic] = self.create_publisher(Temperature, topic, 10)
                
                msg = Temperature()
                msg.temperature = float(payload["temperature_c"])
                msg.variance = 0.0  # Could be set if available
                self.ros_publishers[topic].publish(msg)
            
            # Humidity
            if "humidity_rh" in payload:
                topic = f"{base_topic}/humidity"
                if topic not in self.ros_publishers:
                    self.ros_publishers[topic] = self.create_publisher(RelativeHumidity, topic, 10)
                
                msg = RelativeHumidity()
                msg.relative_humidity = float(payload["humidity_rh"]) / 100.0  # Convert to 0-1 range
                msg.variance = 0.0  # Could be set if available
                self.ros_publishers[topic].publish(msg)
            
            # Pressure
            if "pressure_hpa" in payload:
                topic = f"{base_topic}/pressure"
                if topic not in self.ros_publishers:
                    self.ros_publishers[topic] = self.create_publisher(FluidPressure, topic, 10)
                
                msg = FluidPressure()
                msg.fluid_pressure = float(payload["pressure_hpa"]) * 100.0  # Convert hPa to Pa
                msg.variance = 0.0  # Could be set if available
                self.ros_publishers[topic].publish(msg)
            
            # Also publish raw data as String for visualization
            topic = f"{base_topic}/bme680_raw"
            if topic not in self.ros_publishers:
                self.ros_publishers[topic] = self.create_publisher(String, topic, 10)
            
            msg = String()
            msg.data = json.dumps(payload)
            self.ros_publishers[topic].publish(msg)
            
        elif sensor_type == "battery":
            # Battery state
            topic = f"{base_topic}/battery"
            if topic not in self.ros_publishers:
                self.ros_publishers[topic] = self.create_publisher(BatteryState, topic, 10)
            
            msg = BatteryState()
            if "voltage_v" in payload:
                msg.voltage = float(payload["voltage_v"])
                # Calculate percentage based on voltage range (3.0V - 4.2V)
                voltage = float(payload["voltage_v"])
                percentage = (voltage - 3.0) / (4.2 - 3.0)
                percentage = max(0.0, min(1.0, percentage))
                msg.percentage = percentage * 100.0
            
            msg.present = True
            self.ros_publishers[topic].publish(msg)
            
        elif sensor_type == "display_status":
            # Display status as String
            topic = f"{base_topic}/display"
            if topic not in self.ros_publishers:
                self.ros_publishers[topic] = self.create_publisher(String, topic, 10)
            
            msg = String()
            if "message" in payload:
                msg.data = payload["message"]
            else:
                msg.data = json.dumps(payload)
            
            self.ros_publishers[topic].publish(msg)
        
        # Also publish a combined topic for Gazebo visualization
        topic = f"{base_topic}/visualization"
        if topic not in self.ros_publishers:
            self.ros_publishers[topic] = self.create_publisher(String, topic, 10)
        
        # Create a visualization-friendly message
        vis_data = {
            "sensor_id": sensor_id,
            "sensor_type": sensor_type,
            "data": payload
        }
        
        msg = String()
        msg.data = json.dumps(vis_data)
        self.ros_publishers[topic].publish(msg)
    
    def process_mid_tier_data(self, mid_tier_id: str, payload: dict):
        """Process mid-tier data and publish to ROS2."""
        topic = f"/olympus/mid_tier/{mid_tier_id}/data"
        
        if topic not in self.ros_publishers:
            self.ros_publishers[topic] = self.create_publisher(String, topic, 10)
        
        msg = String()
        msg.data = json.dumps(payload)
        self.ros_publishers[topic].publish(msg)
    
    def publish_generic_message(self, ros_topic: str, payload: dict):
        """Publish a generic message to a ROS2 topic."""
        if ros_topic not in self.ros_publishers:
            self.ros_publishers[ros_topic] = self.create_publisher(String, ros_topic, 10)
        
        msg = String()
        msg.data = json.dumps(payload)
        self.ros_publishers[ros_topic].publish(msg)
    
    def publish_heartbeat(self):
        """Publish a heartbeat message to indicate the bridge is active."""
        topic = "/olympus/bridge/heartbeat"
        
        if topic not in self.ros_publishers:
            self.ros_publishers[topic] = self.create_publisher(String, topic, 10)
        
        msg = String()
        msg.data = json.dumps({
            "timestamp": time.time(),
            "status": "active"
        })
        
        self.ros_publishers[topic].publish(msg)
    
    def shutdown(self):
        """Clean shutdown of the bridge."""
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            log.info("MQTT client disconnected")

def main(args=None):
    # Check if ROS2 is available
    if not ROS2_AVAILABLE:
        log.error("ROS2 is not available. Cannot run the MQTT-ROS2 bridge.")
        log.error("Please install ROS2 or fix your ROS2 environment setup.")
        log.info("For now, running in MQTT-only mode for debugging.")
        
        # Create a dummy bridge for debugging
        node = MQTTToROS2Bridge()
        
        try:
            # Just keep the script running for debugging
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            log.info("Shutting down")
        return
    
    # Normal ROS2 operation
    try:
        rclpy.init(args=args)
        node = MQTTToROS2Bridge()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        log.error(f"Error in ROS2 initialization: {e}")

if __name__ == '__main__':
    main()
