import time
import paho.mqtt.client as mqtt
from typing import Callable, Dict, Any, Optional

class MQTTRealClient:
    """Real MQTT client implementation using Paho-MQTT."""
    def __init__(self, client_id: str, broker_host: str = "localhost", broker_port: int = 1883, 
                 keep_alive: int = 60, owner_node = None):
        """
        Initialize a real MQTT client.
        
        Args:
            client_id (str): Client ID for MQTT connection
            broker_host (str): MQTT broker hostname/IP
            broker_port (int): MQTT broker port
            keep_alive (int): Keep alive timeout in seconds
            owner_node: The Node instance that owns this client (optional)
        """
        self.client_id = client_id
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.owner_node = owner_node
        
        # Initialize MQTT client
        self.client = mqtt.Client(client_id=client_id)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect
        
        # Track subscriptions and message callbacks
        self._topic_callbacks: Dict[str, Callable] = {}
        
        # Connect to broker
        try:
            self.client.connect(broker_host, broker_port, keep_alive)
            self.client.loop_start()  # Start the background thread for MQTT
            print(f"[{time.strftime('%H:%M:%S')}] MQTT Client '{self.client_id}' connected to broker at {broker_host}:{broker_port}")
        except Exception as e:
            print(f"[{time.strftime('%H:%M:%S')}] Error connecting to MQTT broker: {e}")
    
    def _on_connect(self, client, userdata, flags, rc):
        """Callback for when the client connects to the broker."""
        if rc == 0:
            print(f"[{time.strftime('%H:%M:%S')}] MQTT Client '{self.client_id}' successfully connected to broker.")
            # Resubscribe to all topics
            for topic in self._topic_callbacks:
                self.client.subscribe(topic)
        else:
            print(f"[{time.strftime('%H:%M:%S')}] MQTT Client '{self.client_id}' failed to connect with code {rc}")
    
    def _on_message(self, client, userdata, msg):
        """Callback for when a message is received from the broker."""
        try:
            topic = msg.topic
            # If owner_node is set and has a receive_message method, use that
            if self.owner_node and hasattr(self.owner_node, 'receive_message'):
                payload = None
                try:
                    payload = msg.payload.decode('utf-8')
                    # Try to parse as JSON if it looks like JSON
                    if payload.startswith('{') and payload.endswith('}'):
                        import json
                        payload = json.loads(payload)
                except Exception:
                    # If decoding fails, pass the raw payload
                    payload = msg.payload
                
                # Pass the message to the owner node
                self.owner_node.receive_message(topic, payload, "unknown")  # can't know publisher
            
            # If a specific callback is registered for this topic, call it
            if topic in self._topic_callbacks:
                self._topic_callbacks[topic](topic, msg.payload)
        except Exception as e:
            print(f"[{time.strftime('%H:%M:%S')}] Error handling MQTT message: {e}")
    
    def _on_disconnect(self, client, userdata, rc):
        """Callback for when the client disconnects from the broker."""
        if rc != 0:
            print(f"[{time.strftime('%H:%M:%S')}] MQTT Client '{self.client_id}' unexpectedly disconnected with code {rc}")
            # Try to reconnect
            try:
                self.client.reconnect()
            except Exception as e:
                print(f"[{time.strftime('%H:%M:%S')}] Error reconnecting to MQTT broker: {e}")
        else:
            print(f"[{time.strftime('%H:%M:%S')}] MQTT Client '{self.client_id}' disconnected.")
    
    def subscribe(self, topic: str, qos: int = 0, callback: Optional[Callable] = None):
        """
        Subscribe to a topic.
        
        Args:
            topic (str): Topic to subscribe to
            qos (int): Quality of Service level
            callback (Callable): Optional callback for messages on this topic
        """
        self.client.subscribe(topic, qos)
        if callback:
            self._topic_callbacks[topic] = callback
        print(f"[{time.strftime('%H:%M:%S')}] MQTT Client '{self.client_id}' subscribed to topic '{topic}'")
    
    def unsubscribe(self, topic: str):
        """
        Unsubscribe from a topic.
        
        Args:
            topic (str): Topic to unsubscribe from
        """
        self.client.unsubscribe(topic)
        if topic in self._topic_callbacks:
            del self._topic_callbacks[topic]
        print(f"[{time.strftime('%H:%M:%S')}] MQTT Client '{self.client_id}' unsubscribed from topic '{topic}'")
    
    def publish(self, topic: str, message: Any, qos: int = 0, retain: bool = False):
        """
        Publish a message to a topic.
        
        Args:
            topic (str): Topic to publish to
            message (Any): Message payload (will be converted to JSON if dict/list)
            qos (int): Quality of Service level
            retain (bool): Whether to retain the message
        """
        try:
            # Convert dict/list to JSON string
            if isinstance(message, (dict, list)):
                import json
                payload = json.dumps(message)
            else:
                payload = str(message)
            
            self.client.publish(topic, payload, qos, retain)
            print(f"[{time.strftime('%H:%M:%S')}] MQTT Client '{self.client_id}' published to topic '{topic}': {message}")
        except Exception as e:
            print(f"[{time.strftime('%H:%M:%S')}] Error publishing MQTT message: {e}")
    
    def disconnect(self):
        """Disconnect from the MQTT broker."""
        self.client.loop_stop()
        self.client.disconnect()
        print(f"[{time.strftime('%H:%M:%S')}] MQTT Client '{self.client_id}' disconnected from broker.")


# Adapter class to make integration with existing code easier
class MQTTClientAdapter:
    """
    Adapter to maintain compatibility with the existing API.
    Use this for easier transition from simulated MQTT to real MQTT.
    """
    def __init__(self, client_id_node, broker_host="localhost", broker_port=1883):
        # Extract node_id if a Node object is passed
        self.client_id = client_id_node.node_id if hasattr(client_id_node, 'node_id') else client_id_node
        self.real_client = MQTTRealClient(
            client_id=self.client_id, 
            broker_host=broker_host, 
            broker_port=broker_port,
            owner_node=client_id_node if hasattr(client_id_node, 'node_id') else None
        )
    
    def subscribe(self, topic: str, qos: int = 0):
        self.real_client.subscribe(topic, qos)
    
    def publish(self, topic: str, message: Any, qos: int = 0, retain: bool = False):
        self.real_client.publish(topic, message, qos, retain)
    
    def unsubscribe(self, topic: str):
        self.real_client.unsubscribe(topic)
    
    def disconnect(self):
        self.real_client.disconnect()
