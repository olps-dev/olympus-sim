#!/usr/bin/env python3
"""
Network Realism Layer for Olympus Simulation
Implements Step 5: Network realism layer

Provides:
- Packet loss simulation with configurable drop rates
- Latency injection with jitter
- Connection reliability simulation
- QoS-aware network behaviors
- Retry logic and backoff strategies
"""

import time
import random
import json
import logging
import threading
from typing import Dict, Optional, Callable, Any
from dataclasses import dataclass
from queue import Queue, Empty
from enum import Enum
import paho.mqtt.client as mqtt

logger = logging.getLogger('network_realism')

class NetworkCondition(Enum):
    EXCELLENT = "excellent"
    GOOD = "good" 
    FAIR = "fair"
    POOR = "poor"
    UNRELIABLE = "unreliable"

@dataclass
class NetworkConfig:
    """Network simulation configuration"""
    # Packet loss parameters
    packet_loss_rate: float = 0.0  # 0.0 to 1.0
    burst_loss_probability: float = 0.05  # Probability of burst losses
    burst_loss_duration: float = 0.5  # Seconds of sustained loss
    
    # Latency parameters
    base_latency_ms: float = 5.0  # Base network latency
    jitter_ms: float = 2.0  # Random jitter range
    congestion_multiplier: float = 1.0  # Congestion delay multiplier
    
    # Connection reliability
    connection_drop_rate: float = 0.0  # Rate of random disconnections
    reconnect_delay_base: float = 1.0  # Base reconnection delay
    reconnect_delay_max: float = 30.0  # Max reconnection delay
    
    # QoS-specific behaviors
    qos0_additional_loss: float = 0.0  # Extra loss for QoS 0 messages
    qos1_retry_delay_ms: float = 100.0  # Delay before QoS 1 retry
    
    # Simulation control
    enabled: bool = True  # Enable/disable network simulation
    condition: NetworkCondition = NetworkCondition.GOOD

    @classmethod
    def from_condition(cls, condition: NetworkCondition) -> 'NetworkConfig':
        """Create config from predefined network condition"""
        configs = {
            NetworkCondition.EXCELLENT: cls(
                packet_loss_rate=0.001,
                base_latency_ms=1.0,
                jitter_ms=0.5,
                condition=condition
            ),
            NetworkCondition.GOOD: cls(
                packet_loss_rate=0.01,
                base_latency_ms=5.0,
                jitter_ms=2.0,
                condition=condition
            ),
            NetworkCondition.FAIR: cls(
                packet_loss_rate=0.05,
                base_latency_ms=15.0,
                jitter_ms=5.0,
                burst_loss_probability=0.1,
                condition=condition
            ),
            NetworkCondition.POOR: cls(
                packet_loss_rate=0.10,
                base_latency_ms=50.0,
                jitter_ms=20.0,
                burst_loss_probability=0.2,
                burst_loss_duration=1.0,
                connection_drop_rate=0.01,
                condition=condition
            ),
            NetworkCondition.UNRELIABLE: cls(
                packet_loss_rate=0.25,
                base_latency_ms=100.0,
                jitter_ms=50.0,
                burst_loss_probability=0.3,
                burst_loss_duration=2.0,
                connection_drop_rate=0.05,
                qos0_additional_loss=0.1,
                condition=condition
            )
        }
        return configs[condition]

class NetworkSimulator:
    """Core network simulation engine"""
    
    def __init__(self, config: NetworkConfig):
        self.config = config
        self.burst_loss_until = 0.0
        self.connection_attempts = 0
        self.last_disconnect_time = 0.0
        
        # Message queuing for congestion simulation
        self.message_queue = Queue()
        self.processing_messages = False
        self.queue_thread = None
        
        # Statistics
        self.stats = {
            'messages_sent': 0,
            'messages_dropped': 0,
            'messages_delayed': 0,
            'total_latency_added_ms': 0.0,
            'disconnections': 0,
            'reconnections': 0
        }
        
        self._start_message_processor()
    
    def _start_message_processor(self):
        """Start background thread for processing queued messages"""
        if self.queue_thread is None or not self.queue_thread.is_alive():
            self.processing_messages = True
            self.queue_thread = threading.Thread(target=self._process_message_queue, daemon=True)
            self.queue_thread.start()
    
    def _process_message_queue(self):
        """Process queued messages with simulated delays"""
        while self.processing_messages:
            try:
                message_data = self.message_queue.get(timeout=1.0)
                if message_data is None:  # Shutdown signal
                    break
                
                # Apply congestion delay
                delay = self._calculate_congestion_delay()
                if delay > 0:
                    time.sleep(delay / 1000.0)  # Convert ms to seconds
                    self.stats['messages_delayed'] += 1
                    self.stats['total_latency_added_ms'] += delay
                
                # Execute the actual publish
                callback, args, kwargs = message_data
                callback(*args, **kwargs)
                
                self.message_queue.task_done()
                
            except Empty:
                continue
            except Exception as e:
                logger.error(f"Error processing message queue: {e}")
    
    def should_drop_packet(self, qos: int = 0) -> bool:
        """Determine if a packet should be dropped"""
        if not self.config.enabled:
            return False
        
        current_time = time.time()
        
        # Check for burst loss
        if current_time < self.burst_loss_until:
            return True
        
        # Check if we should start a burst loss
        if random.random() < self.config.burst_loss_probability:
            self.burst_loss_until = current_time + self.config.burst_loss_duration
            logger.debug(f"Starting burst loss for {self.config.burst_loss_duration}s")
            return True
        
        # Regular packet loss
        loss_rate = self.config.packet_loss_rate
        if qos == 0:
            loss_rate += self.config.qos0_additional_loss
        
        should_drop = random.random() < loss_rate
        if should_drop:
            self.stats['messages_dropped'] += 1
            logger.debug(f"Dropped packet (QoS {qos}, rate {loss_rate:.3f})")
        
        return should_drop
    
    def calculate_latency_delay(self) -> float:
        """Calculate network latency delay in milliseconds"""
        if not self.config.enabled:
            return 0.0
        
        # Base latency + jitter
        base_delay = self.config.base_latency_ms
        jitter = random.uniform(-self.config.jitter_ms, self.config.jitter_ms)
        
        # Congestion multiplier
        congestion_delay = self._calculate_congestion_delay()
        
        total_delay = base_delay + jitter + congestion_delay
        return max(0.0, total_delay)  # Ensure non-negative
    
    def _calculate_congestion_delay(self) -> float:
        """Calculate additional delay due to network congestion"""
        queue_size = self.message_queue.qsize()
        if queue_size == 0:
            return 0.0
        
        # Simulate queuing delay based on queue size
        base_processing_time = 2.0  # 2ms per message base processing
        congestion_delay = queue_size * base_processing_time * self.config.congestion_multiplier
        
        return congestion_delay
    
    def should_disconnect(self) -> bool:
        """Determine if connection should be dropped"""
        if not self.config.enabled:
            return False
        
        should_dc = random.random() < self.config.connection_drop_rate
        if should_dc:
            self.stats['disconnections'] += 1
            self.last_disconnect_time = time.time()
            logger.debug("Simulated connection drop")
        
        return should_dc
    
    def get_reconnect_delay(self) -> float:
        """Calculate delay before reconnection attempt"""
        if not self.config.enabled:
            return 0.0
        
        # Exponential backoff
        backoff_multiplier = min(2 ** self.connection_attempts, 16)  # Cap at 16x
        delay = self.config.reconnect_delay_base * backoff_multiplier
        delay = min(delay, self.config.reconnect_delay_max)
        
        # Add jitter to avoid thundering herd
        jitter = random.uniform(0.8, 1.2)
        final_delay = delay * jitter
        
        self.connection_attempts += 1
        logger.debug(f"Reconnect delay: {final_delay:.2f}s (attempt {self.connection_attempts})")
        
        return final_delay
    
    def on_connection_success(self):
        """Reset connection attempt counter on successful connection"""
        if self.connection_attempts > 0:
            self.stats['reconnections'] += 1
            logger.debug(f"Reconnected after {self.connection_attempts} attempts")
        self.connection_attempts = 0
    
    def simulate_publish_delay(self, topic: str, payload: Any, qos: int = 0) -> bool:
        """
        Simulate network delay for message publish
        Returns True if message should proceed, False if dropped
        """
        if not self.config.enabled:
            return True
        
        self.stats['messages_sent'] += 1
        
        # Check for packet loss
        if self.should_drop_packet(qos):
            return False
        
        # Calculate and apply latency delay
        delay_ms = self.calculate_latency_delay()
        if delay_ms > 0:
            time.sleep(delay_ms / 1000.0)
            self.stats['total_latency_added_ms'] += delay_ms
        
        return True
    
    def queue_message(self, callback: Callable, *args, **kwargs):
        """Queue a message for delayed processing"""
        if not self.config.enabled:
            callback(*args, **kwargs)
            return
        
        self.message_queue.put((callback, args, kwargs))
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get network simulation statistics"""
        total_messages = self.stats['messages_sent']
        drop_rate = (self.stats['messages_dropped'] / total_messages) if total_messages > 0 else 0.0
        avg_latency = (self.stats['total_latency_added_ms'] / total_messages) if total_messages > 0 else 0.0
        
        return {
            **self.stats,
            'queue_size': self.message_queue.qsize(),
            'drop_rate_actual': drop_rate,
            'avg_latency_added_ms': avg_latency,
            'network_condition': self.config.condition.value,
            'config': {
                'packet_loss_rate': self.config.packet_loss_rate,
                'base_latency_ms': self.config.base_latency_ms,
                'connection_drop_rate': self.config.connection_drop_rate
            }
        }
    
    def shutdown(self):
        """Shutdown the network simulator"""
        self.processing_messages = False
        if self.queue_thread and self.queue_thread.is_alive():
            self.message_queue.put(None)  # Signal shutdown
            self.queue_thread.join(timeout=5.0)

class NetworkRealisticMQTTClient:
    """MQTT client wrapper with network realism simulation"""
    
    def __init__(self, base_client: mqtt.Client, network_sim: NetworkSimulator):
        self.base_client = base_client
        self.network_sim = network_sim
        
        # Wrap callbacks to add network simulation
        self._original_on_connect = base_client.on_connect
        self._original_on_disconnect = base_client.on_disconnect
        
        # Set our callback wrappers
        base_client.on_connect = self._on_connect_wrapper
        base_client.on_disconnect = self._on_disconnect_wrapper
        
        # QoS 1 retry tracking
        self.pending_qos1_messages = {}  # msg_id -> (topic, payload, retry_count)
        self.max_qos1_retries = 3
        
        # Connection state
        self.connected = False
        self.connecting = False
    
    def _on_connect_wrapper(self, client, userdata, flags, reason_code, properties=None):
        """Wrapper for connection callback with network simulation"""
        self.connected = True
        self.connecting = False
        self.network_sim.on_connection_success()
        
        if self._original_on_connect:
            try:
                # Handle both old and new paho-mqtt callback signatures
                if properties is not None:
                    self._original_on_connect(client, userdata, flags, reason_code, properties)
                else:
                    self._original_on_connect(client, userdata, flags, reason_code)
            except Exception as e:
                logger.error(f"Error in original on_connect callback: {e}")
    
    def _on_disconnect_wrapper(self, client, userdata, reason_code, properties=None):
        """Wrapper for disconnect callback with network simulation"""
        self.connected = False
        
        if self._original_on_disconnect:
            try:
                if properties is not None:
                    self._original_on_disconnect(client, userdata, reason_code, properties)
                else:
                    # Handle old callback signature
                    if hasattr(self._original_on_disconnect, '__code__') and \
                       self._original_on_disconnect.__code__.co_argcount == 3:
                        self._original_on_disconnect(client, userdata, reason_code)
                    else:
                        self._original_on_disconnect(client, userdata, reason_code, properties)
            except Exception as e:
                logger.error(f"Error in original on_disconnect callback: {e}")
    
    def publish(self, topic: str, payload=None, qos: int = 0, retain: bool = False) -> mqtt.MQTTMessageInfo:
        """Publish with network realism simulation"""
        # Check for simulated connection drops
        if self.connected and self.network_sim.should_disconnect():
            self.base_client.disconnect()
            return mqtt.MQTTMessageInfo(mid=-1)  # Indicate failure
        
        # Simulate network conditions for the publish
        if not self.network_sim.simulate_publish_delay(topic, payload, qos):
            # Message dropped due to network conditions
            logger.debug(f"Network dropped message to {topic}")
            return mqtt.MQTTMessageInfo(mid=-1)  # Indicate failure
        
        # Proceed with actual publish
        try:
            result = self.base_client.publish(topic, payload, qos, retain)
            
            # Handle QoS 1 retry logic
            if qos == 1 and result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.pending_qos1_messages[result.mid] = (topic, payload, 0)
                # Set up retry timer (simplified - in production use proper timer)
                threading.Timer(
                    self.network_sim.config.qos1_retry_delay_ms / 1000.0,
                    self._check_qos1_retry,
                    args=[result.mid]
                ).start()
            
            return result
            
        except Exception as e:
            logger.error(f"Error publishing to {topic}: {e}")
            return mqtt.MQTTMessageInfo(mid=-1)
    
    def _check_qos1_retry(self, msg_id: int):
        """Check if QoS 1 message needs retry"""
        if msg_id in self.pending_qos1_messages:
            topic, payload, retry_count = self.pending_qos1_messages[msg_id]
            
            if retry_count < self.max_qos1_retries:
                # Retry the message
                logger.debug(f"Retrying QoS 1 message to {topic} (attempt {retry_count + 1})")
                self.pending_qos1_messages[msg_id] = (topic, payload, retry_count + 1)
                
                # Retry with network simulation
                if self.network_sim.simulate_publish_delay(topic, payload, 1):
                    try:
                        result = self.base_client.publish(topic, payload, 1, False)
                        if result.rc == mqtt.MQTT_ERR_SUCCESS:
                            # Update tracking with new message ID
                            del self.pending_qos1_messages[msg_id]
                            self.pending_qos1_messages[result.mid] = (topic, payload, retry_count + 1)
                    except Exception as e:
                        logger.error(f"Error in QoS 1 retry: {e}")
            else:
                # Max retries exceeded
                logger.warning(f"QoS 1 message to {topic} failed after {self.max_qos1_retries} retries")
                del self.pending_qos1_messages[msg_id]
    
    def connect(self, host: str, port: int = 1883, keepalive: int = 60):
        """Connect with network realism"""
        if self.connecting:
            return
        
        self.connecting = True
        
        # Apply reconnection delay if needed
        delay = self.network_sim.get_reconnect_delay()
        if delay > 0:
            logger.debug(f"Applying reconnection delay: {delay:.2f}s")
            time.sleep(delay)
        
        try:
            return self.base_client.connect(host, port, keepalive)
        except Exception as e:
            logger.error(f"Connection error: {e}")
            self.connecting = False
            raise
    
    def disconnect(self):
        """Disconnect wrapper"""
        self.connected = False
        return self.base_client.disconnect()
    
    def loop_start(self):
        """Start network loop"""
        return self.base_client.loop_start()
    
    def loop_stop(self):
        """Stop network loop"""
        return self.base_client.loop_stop()
    
    def loop_forever(self):
        """Run network loop forever"""
        return self.base_client.loop_forever()
    
    def subscribe(self, topic, qos=0):
        """Subscribe wrapper"""
        return self.base_client.subscribe(topic, qos)
    
    def __getattr__(self, name):
        """Delegate other attributes to base client"""
        return getattr(self.base_client, name)

def create_realistic_mqtt_client(network_config: NetworkConfig = None, 
                               mqtt_client_args: tuple = None,
                               mqtt_client_kwargs: dict = None) -> NetworkRealisticMQTTClient:
    """
    Factory function to create MQTT client with network realism
    
    Args:
        network_config: Network simulation configuration
        mqtt_client_args: Arguments for mqtt.Client() constructor
        mqtt_client_kwargs: Keyword arguments for mqtt.Client() constructor
    
    Returns:
        NetworkRealisticMQTTClient instance
    """
    if network_config is None:
        network_config = NetworkConfig.from_condition(NetworkCondition.GOOD)
    
    if mqtt_client_args is None:
        mqtt_client_args = ()
    if mqtt_client_kwargs is None:
        mqtt_client_kwargs = {}
    
    # Create base MQTT client with version compatibility
    try:
        base_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, *mqtt_client_args, **mqtt_client_kwargs)
    except (AttributeError, TypeError):
        # Fall back to old API
        base_client = mqtt.Client(*mqtt_client_args, **mqtt_client_kwargs)
    
    # Create network simulator
    network_sim = NetworkSimulator(network_config)
    
    # Create and return realistic client
    return NetworkRealisticMQTTClient(base_client, network_sim)