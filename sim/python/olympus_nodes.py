import time
import random
import subprocess
import json # For constructing the ROS message string
from typing import Optional, Dict, Any, List

class Node:
    """Represents a node in the Olympus simulation (Zeus, Mid-Tier, or Sensor)."""
    def __init__(self, node_id, node_type, parent_broker=None, network_simulator=None):
        self.node_id = node_id
        self.node_type = node_type  # 'zeus', 'mid_tier', 'sensor_node'
        self.parent_broker = parent_broker  # MQTTBrokerSim instance for upstream communication
        self.network_simulator = network_simulator # Instance of NetworkSimulator
        self.children_nodes = [] # For Zeus and Mid-Tier nodes
        self.local_mqtt_broker = None # For Zeus and Mid-Tier nodes
        self.sensors = {} # For Sensor Nodes (can be used for logical sensors or if bridge not used)
        self.data_buffer = []
        self.hardware_bridge = None # For Sensor Nodes using HardwareSensorBridge
        self.last_lamp_state: Optional[bool] = None # For Zeus node to track lamp state
        self.mqtt_client = None # For real MQTT communication (replaces parent_broker for real deployments)

        print(f"[{time.strftime('%H:%M:%S')}] Node {self.node_id} ({self.node_type}) created.")

    def add_child_node(self, child_node):
        if self.node_type in ['zeus', 'mid_tier']:
            self.children_nodes.append(child_node)
            print(f"[{time.strftime('%H:%M:%S')}] Node {child_node.node_id} added as child to {self.node_id}")
        else:
            print(f"[{time.strftime('%H:%M:%S')}] Error: Node {self.node_id} of type {self.node_type} cannot have children.")

    def set_local_broker(self, broker):
        if self.node_type in ['zeus', 'mid_tier']:
            self.local_mqtt_broker = broker
            print(f"[{time.strftime('%H:%M:%S')}] Node {self.node_id} assigned local broker {broker.broker_id}")
        else:
            print(f"[{time.strftime('%H:%M:%S')}] Error: Node {self.node_id} of type {self.node_type} cannot have a local broker.")

    def add_sensor_instance(self, sensor_name, sensor_instance):
        if self.node_type == 'sensor_node':
            self.sensors[sensor_name] = sensor_instance
            print(f"[{time.strftime('%H:%M:%S')}] Node {self.node_id} added sensor {sensor_name}")
        else:
            print(f"[{time.strftime('%H:%M:%S')}] Error: Node {self.node_id} of type {self.node_type} cannot have direct sensor instances.")

    def set_hardware_bridge(self, bridge_instance):
        if self.node_type == 'sensor_node':
            self.hardware_bridge = bridge_instance
            print(f"[{time.strftime('%H:%M:%S')}] Node {self.node_id} assigned HardwareSensorBridge.")
        else:
            print(f"[{time.strftime('%H:%M:%S')}] Error: Node {self.node_id} of type {self.node_type} cannot have a HardwareSensorBridge.")

    def set_mqtt_client(self, mqtt_client):
        """Set a real MQTT client for this node."""
        self.mqtt_client = mqtt_client
        print(f"[{time.strftime('%H:%M:%S')}] Node {self.node_id} assigned MQTT client.")

    def publish_to_parent(self, topic, message, qos=0):
        # First, try real MQTT client if available
        if self.mqtt_client:
            full_topic = f"olympus/{topic}"
            self.mqtt_client.publish(full_topic, message, qos)
            print(f"[{time.strftime('%H:%M:%S')}] Node {self.node_id} published to real MQTT broker on topic '{full_topic}': {message}")
            return
            
        # Fall back to simulated MQTT if no real client is set
        if self.parent_broker:
            # Simulate network delay/conditions if network_simulator is present
            if self.network_simulator:
                self.network_simulator.simulate_transmission(self, self.parent_broker.owner_node, message)
            
            self.parent_broker.publish(self.node_id, topic, message, qos)
            print(f"[{time.strftime('%H:%M:%S')}] Node {self.node_id} published to parent broker on topic '{topic}': {message}")
        else:
            print(f"[{time.strftime('%H:%M:%S')}] Node {self.node_id} has no parent broker or MQTT client to publish to.")

    def receive_message(self, topic, message, publisher_id):
        """Called by an MQTT broker when a message is received on a subscribed topic."""
        print(f"[{time.strftime('%H:%M:%S')}] Node {self.node_id} received message on topic '{topic}' from {publisher_id}: {message}")
        self.data_buffer.append({'topic': topic, 'message': message, 'publisher': publisher_id})
        self.process_data(message)

    def process_data(self, data):
        """Placeholder for data processing logic specific to node type."""
        if self.node_type == 'sensor_node':
            # Sensor nodes typically publish pre-processed data
            print(f"[{time.strftime('%H:%M:%S')}] Sensor Node {self.node_id} processing: {data}")
            # Example: Directly publish to parent
            # self.publish_to_parent(f"data/{self.node_id}", data)
            pass

        elif self.node_type == 'mid_tier':
            print(f"[{time.strftime('%H:%M:%S')}] Mid-Tier Node {self.node_id} processing and aggregating: {data}")
            # Aggregate data, apply rules, then publish to Zeus
            # self.publish_to_parent(f"aggregated/{self.node_id}", aggregated_data)
            pass

        elif self.node_type == 'zeus':
            print(f"[{time.strftime('%H:%M:%S')}] Zeus Node {self.node_id} performing advanced analytics: {data}")
            # Perform AI, long-term storage, etc.
            pass

    def get_sensor_reading(self, sensor_name):
        if self.node_type == 'sensor_node' and sensor_name in self.sensors:
            # This assumes sensor instances have a 'read()' method
            # In the actual Olympus sim, this would interact with the HardwareSensorBridge
            try:
                return self.sensors[sensor_name].read()
            except AttributeError:
                print(f"[{time.strftime('%H:%M:%S')}] Error: Sensor {sensor_name} on {self.node_id} does not have a read() method.")
                return None
        return None

    def tick(self):
        """Simulate node activity for one time step."""
        if self.node_type == 'sensor_node':
            if self.hardware_bridge:
                # The bridge's method handles reading its internal sensors and publishing via its MQTT client
                self.hardware_bridge.read_all_sensors_and_publish()
            else:
                # Fallback or alternative sensor handling if no bridge, or for other logical sensors
                for sensor_name, sensor_instance in self.sensors.items():
                    if hasattr(sensor_instance, 'read'):
                        reading = sensor_instance.read()
                        payload = {'timestamp': time.time(), 'sensor': sensor_name, 'value': reading}
                        self.publish_to_parent(f"legacy_sensor_data/{self.node_id}/{sensor_name}", payload)

        elif self.node_type == 'mid_tier':
            if self.data_buffer:
                latest_data = self.data_buffer.pop(0)
                print(f"[{time.strftime('%H:%M:%S')}] Mid-Tier {self.node_id} processing buffered data: {latest_data}")
                # Potentially aggregate and forward to Zeus
                self.publish_to_parent(f"processed_mid_tier/{self.node_id}", latest_data['message'])
        
        elif self.node_type == 'zeus':
            if self.data_buffer:
                latest_data = self.data_buffer.pop(0)
                # print(f"[{time.strftime('%H:%M:%S')}] Zeus {self.node_id} processing buffered data: {latest_data}")
                
                message_content = latest_data.get('message')
                if isinstance(message_content, dict) and 'lamp_on' in message_content:
                    current_lamp_state = bool(message_content['lamp_on'])
                    # print(f"[{time.strftime('%H:%M:%S')}] Zeus {self.node_id} detected lamp_on: {current_lamp_state}")
                    if current_lamp_state != self.last_lamp_state:
                        self.last_lamp_state = current_lamp_state
                        ros_msg_data = 'true' if current_lamp_state else 'false'
                        ros2_command = [
                            'ros2',
                            'topic',
                            'pub',
                            '--once',
                            '/hall_lamp/cmd',
                            'std_msgs/msg/Bool',
                            f'{{data: {ros_msg_data}}}'
                        ]
                        try:
                            print(f"[{time.strftime('%H:%M:%S')}] Zeus {self.node_id} sending command to lamp: {'On' if current_lamp_state else 'Off'}")
                            # print(f"Executing: {' '.join(ros2_command)}")
                            subprocess.run(ros2_command, check=True, capture_output=True, text=True)
                            # print(f"[{time.strftime('%H:%M:%S')}] ROS 2 command successful.")
                        except subprocess.CalledProcessError as e:
                            print(f"[{time.strftime('%H:%M:%S')}] Error executing ROS 2 command: {e}")
                            print(f"[{time.strftime('%H:%M:%S')}] stdout: {e.stdout}")
                            print(f"[{time.strftime('%H:%M:%S')}] stderr: {e.stderr}")
                        except FileNotFoundError:
                            print(f"[{time.strftime('%H:%M:%S')}] Error: 'ros2' command not found. Ensure ROS 2 environment is sourced.")
                # Else, could be other data for Zeus to process
                # print(f"[{time.strftime('%H:%M:%S')}] Zeus {self.node_id} processed data: {latest_data}")

        # If this node has a local broker, it should also tick
        if self.local_mqtt_broker:
            self.local_mqtt_broker.tick() # Process its own queue if any logic is added there
