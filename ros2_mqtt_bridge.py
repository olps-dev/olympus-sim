#!/usr/bin/env python3
"""
ROS2 MQTT Bridge for Olympus Simulation
This node connects to the MQTT broker, subscribes to Olympus topics,
and republishes the data as ROS2 messages to control Gazebo models.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, ColorRGBA
from geometry_msgs.msg import Pose
import paho.mqtt.client as mqtt
import json
import time
from threading import Thread
import math

class OlympusMQTTBridge(Node):
    def __init__(self):
        super().__init__('olympus_mqtt_bridge')
        
        # Parameters
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic_prefix', 'olympus/')
        
        # Get parameters
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.mqtt_topic_prefix = self.get_parameter('mqtt_topic_prefix').value
        
        # Setup MQTT client
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # ROS2 Publishers - one per sensor
        self.temp_publishers = {}
        self.battery_publishers = {}
        self.material_publishers = {}
        self.text_publishers = {}
        
        self.get_logger().info(f"Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
        
        # Connect to MQTT in a separate thread
        self.mqtt_thread = Thread(target=self.mqtt_connect)
        self.mqtt_thread.daemon = True
        self.mqtt_thread.start()
        
        # Timer for checking connection status
        self.create_timer(5.0, self.check_mqtt_connection)
        
        self.get_logger().info("ROS2 MQTT Bridge initialized")
    
    def mqtt_connect(self):
        """Connect to MQTT broker in a separate thread."""
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_forever()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
    
    def check_mqtt_connection(self):
        """Periodically check MQTT connection status."""
        if not self.mqtt_client.is_connected():
            self.get_logger().warn("MQTT client disconnected, attempting to reconnect...")
            try:
                self.mqtt_client.reconnect()
                self.get_logger().info("MQTT client reconnected successfully")
            except Exception as e:
                self.get_logger().error(f"Failed to reconnect to MQTT broker: {e}")
    
    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties):
        """Callback when connected to MQTT broker."""
        if reason_code == 0:
            self.get_logger().info(f"Connected to MQTT broker with result code {reason_code}")
            # Subscribe to all sensor data topics
            mqtt_topic = f"{self.mqtt_topic_prefix}sensor_data/#"
            client.subscribe(mqtt_topic)
            self.get_logger().info(f"Subscribed to MQTT topic: {mqtt_topic}")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker with result code {reason_code}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Callback when MQTT message is received."""
        try:
            # Parse topic to extract sensor ID and data type
            # Expected format: olympus/sensor_data/sensor-X/data_type
            topic_parts = msg.topic.split('/')
            if len(topic_parts) >= 4 and topic_parts[0] == 'olympus' and topic_parts[1] == 'sensor_data':
                sensor_id = topic_parts[2]
                data_type = topic_parts[3]
                
                # Parse payload
                payload = json.loads(msg.payload)
                
                self.get_logger().debug(f"Received {data_type} data from {sensor_id}: {payload}")
                
                # Process different data types
                if data_type == 'bme680':
                    self.process_bme680_data(sensor_id, payload)
                elif data_type == 'battery':
                    self.process_battery_data(sensor_id, payload)
                elif data_type == 'display_status':
                    self.process_display_data(sensor_id, payload)
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")
    
    def process_bme680_data(self, sensor_id, payload):
        """Process BME680 sensor data and publish to ROS2."""
        ros_compatible_sensor_id = sensor_id.replace('-', '_')
        if 'temperature_c' in payload:
            # Create publisher if it doesn't exist
            if sensor_id not in self.temp_publishers:
                topic_name = f'/olympus/sensor/{ros_compatible_sensor_id}/temperature'
                self.temp_publishers[sensor_id] = self.create_publisher(Float32, topic_name, 10)
                self.get_logger().info(f"Created temperature publisher for {sensor_id} (ROS: {ros_compatible_sensor_id}) on {topic_name}")
            
            # Publish temperature
            temp_msg = Float32()
            temp_msg.data = float(payload['temperature_c'])
            self.temp_publishers[sensor_id].publish(temp_msg)
            
            # Create text publisher if it doesn't exist
            if sensor_id not in self.text_publishers:
                topic_name = f'/olympus/sensor/{ros_compatible_sensor_id}/display_text'
                self.text_publishers[sensor_id] = self.create_publisher(String, topic_name, 10)
                self.get_logger().info(f"Created text publisher for {sensor_id} (ROS: {ros_compatible_sensor_id}) on {topic_name}")
            
            # Publish temperature as text for Gazebo visual text
            text_msg = String()
            text_msg.data = f"Temp: {payload['temperature_c']:.1f}°C"
            self.text_publishers[sensor_id].publish(text_msg)
    
    def process_battery_data(self, sensor_id, payload):
        """Process battery data and publish to ROS2."""
        ros_compatible_sensor_id = sensor_id.replace('-', '_')
        if 'voltage_v' in payload:
            # Create publisher if it doesn't exist
            if sensor_id not in self.battery_publishers:
                topic_name = f'/olympus/sensor/{ros_compatible_sensor_id}/battery'
                self.battery_publishers[sensor_id] = self.create_publisher(Float32, topic_name, 10)
                self.get_logger().info(f"Created battery publisher for {sensor_id} (ROS: {ros_compatible_sensor_id}) on {topic_name}")
            
            # Publish battery voltage
            battery_msg = Float32()
            battery_msg.data = float(payload['voltage_v'])
            self.battery_publishers[sensor_id].publish(battery_msg)
            
            # Create material publisher if it doesn't exist
            if sensor_id not in self.material_publishers:
                topic_name = f'/olympus/sensor/{ros_compatible_sensor_id}/material_color'
                self.material_publishers[sensor_id] = self.create_publisher(ColorRGBA, topic_name, 10)
                self.get_logger().info(f"Created material publisher for {sensor_id} (ROS: {ros_compatible_sensor_id}) on {topic_name}")
            
            # Map battery voltage to color (3.0V=red, 4.2V=green)
            battery_voltage = float(payload['voltage_v'])
            battery_pct = min(max((battery_voltage - 3.0) / (4.2 - 3.0), 0), 1)
            
            # Create color message
            color_msg = ColorRGBA()
            if battery_pct > 0.7:
                color_msg.r = 0.0
                color_msg.g = 1.0
                color_msg.b = 0.0
            elif battery_pct > 0.3:
                color_msg.r = 1.0
                color_msg.g = 1.0
                color_msg.b = 0.0
            else:
                color_msg.r = 1.0
                color_msg.g = 0.0
                color_msg.b = 0.0
            color_msg.a = 1.0
            
            # Publish color
            self.material_publishers[sensor_id].publish(color_msg)
    
    def process_display_data(self, sensor_id, payload):
        """Process display status data and publish to ROS2."""
        ros_compatible_sensor_id = sensor_id.replace('-', '_')
        if 'message' in payload:
            # Create text publisher if it doesn't exist
            if sensor_id not in self.text_publishers:
                topic_name = f'/olympus/sensor/{ros_compatible_sensor_id}/display_text'
                self.text_publishers[sensor_id] = self.create_publisher(String, topic_name, 10)
                self.get_logger().info(f"Created text publisher for {sensor_id} (ROS: {ros_compatible_sensor_id}) on {topic_name}")
            
            # Publish display message
            text_msg = String()
            text_msg.data = payload['message']
            self.text_publishers[sensor_id].publish(text_msg)

def main(args=None):
    rclpy.init(args=args)
    
    mqtt_bridge = OlympusMQTTBridge()
    
    try:
        rclpy.spin(mqtt_bridge)
    except KeyboardInterrupt:
        mqtt_bridge.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        # Cleanup
        mqtt_bridge.mqtt_client.disconnect()
        mqtt_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
