#!/usr/bin/env python3
"""
Live Data Manager for Olympus Dashboard
Connects directly to MQTT for real-time data streaming
"""

import json
import time
import threading
import logging
from typing import Dict, List, Any, Optional
from collections import defaultdict, deque
from dataclasses import dataclass, asdict
from datetime import datetime
import paho.mqtt.client as mqtt
import numpy as np

logger = logging.getLogger(__name__)

@dataclass
class SensorData:
    """Sensor status and metrics"""
    sensor_id: str
    human_present: bool = False
    battery_level_mah: float = 3000.0
    last_update: float = 0.0
    event_count: int = 0
    num_points: int = 0
    analysis: str = ""
    
@dataclass
class LatencyEvent:
    """End-to-end latency measurement"""
    timestamp: float
    sensor_id: str
    event_id: str
    latency_ms: float
    stage: str = "e2e"

@dataclass
class AutomationEvent:
    """Automation control event"""
    timestamp: float
    actuator_id: str
    action: str
    triggered_by: List[str]
    latency_ms: float = 0.0

class LiveDataManager:
    """Manages real-time data collection from MQTT"""
    
    def __init__(self, mqtt_broker='mosquitto', mqtt_port=1883, max_history=1000):
        # Configuration
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.max_history = max_history
        
        # Data stores
        self.sensors: Dict[str, SensorData] = {}
        self.latency_events = deque(maxlen=max_history)
        self.automation_events = deque(maxlen=max_history)
        self.presence_events = defaultdict(deque)  # sensor_id -> deque of timestamps
        
        # MQTT client
        self.mqtt_client = None
        self.mqtt_connected = False
        
        # Threading
        self.lock = threading.RLock()
        self.running = False
        
        # Performance tracking
        self.latency_window_seconds = 300  # 5 minutes
        
    def start(self):
        """Start data collection"""
        logger.info("Starting live data manager")
        self.running = True
        self._connect_mqtt()
        
    def stop(self):
        """Stop data collection"""
        logger.info("Stopping live data manager")
        self.running = False
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            
    def _connect_mqtt(self):
        """Connect to MQTT broker"""
        try:
            self.mqtt_client = mqtt.Client(client_id=f"dashboard_{int(time.time())}")
            self.mqtt_client.on_connect = self._on_mqtt_connect
            self.mqtt_client.on_message = self._on_mqtt_message
            self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
            
            logger.info(f"Connecting to MQTT broker: {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            
        except Exception as e:
            logger.error(f"Failed to connect to MQTT: {e}")
            self.mqtt_connected = False
            
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            logger.info("Connected to MQTT broker")
            self.mqtt_connected = True
            
            # Subscribe to all relevant topics
            topics = [
                "sensor/+/presence",
                "sensor/+/status",
                "sensor/+/human_present",
                "actuators/+/command",
                "automation/events",
                "metrics/latency/+",
                "metrics/e2e_latency"
            ]
            
            for topic in topics:
                client.subscribe(topic)
                logger.info(f"Subscribed to {topic}")
        else:
            logger.error(f"Failed to connect to MQTT: {rc}")
            self.mqtt_connected = False
            
    def _on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        logger.warning(f"Disconnected from MQTT broker (rc={rc})")
        self.mqtt_connected = False
        
    def _on_mqtt_message(self, client, userdata, msg):
        """Process incoming MQTT messages"""
        try:
            topic = msg.topic
            payload = msg.payload.decode()
            timestamp = time.time()
            
            with self.lock:
                if "presence" in topic:
                    self._process_presence(topic, payload, timestamp)
                elif "human_present" in topic:
                    self._process_human_present(topic, payload, timestamp)
                elif "actuators" in topic and "command" in topic:
                    self._process_actuator_command(topic, payload, timestamp)
                elif "automation/events" in topic:
                    self._process_automation_event(payload, timestamp)
                elif "metrics/latency" in topic or "e2e_latency" in topic:
                    self._process_latency_metric(payload, timestamp)
                    
        except Exception as e:
            logger.error(f"Error processing MQTT message on {topic}: {e}")
            
    def _process_presence(self, topic: str, payload: str, timestamp: float):
        """Process sensor presence detection"""
        try:
            data = json.loads(payload)
            sensor_id = data.get('sensor_id', topic.split('/')[1])
            
            # Update or create sensor
            if sensor_id not in self.sensors:
                self.sensors[sensor_id] = SensorData(sensor_id=sensor_id)
                
            sensor = self.sensors[sensor_id]
            sensor.human_present = data.get('human_present', False)
            sensor.battery_level_mah = data.get('battery_level_mah', sensor.battery_level_mah)
            sensor.num_points = data.get('num_points', 0)
            sensor.analysis = data.get('analysis', '')
            sensor.last_update = timestamp
            sensor.event_count += 1
            
            # Track presence events
            if sensor.human_present:
                self.presence_events[sensor_id].append(timestamp)
                # Keep only recent events
                cutoff = timestamp - self.latency_window_seconds
                while self.presence_events[sensor_id] and self.presence_events[sensor_id][0] < cutoff:
                    self.presence_events[sensor_id].popleft()
                    
            logger.debug(f"Updated sensor {sensor_id}: present={sensor.human_present}")
            
        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON in presence message: {payload}")
            
    def _process_human_present(self, topic: str, payload: str, timestamp: float):
        """Process simple human presence boolean"""
        try:
            sensor_id = topic.split('/')[1]
            human_present = payload.lower() == 'true'
            
            if sensor_id not in self.sensors:
                self.sensors[sensor_id] = SensorData(sensor_id=sensor_id)
                
            self.sensors[sensor_id].human_present = human_present
            self.sensors[sensor_id].last_update = timestamp
            
        except Exception as e:
            logger.error(f"Error processing human_present: {e}")
            
    def _process_actuator_command(self, topic: str, payload: str, timestamp: float):
        """Process actuator commands"""
        try:
            parts = topic.split('/')
            if len(parts) >= 2:
                actuator_id = parts[1]
                
                # Create automation event from command
                event = AutomationEvent(
                    timestamp=timestamp,
                    actuator_id=actuator_id,
                    action=payload,
                    triggered_by=[]
                )
                
                self.automation_events.append(event)
                logger.info(f"Actuator command: {actuator_id} -> {payload}")
                
        except Exception as e:
            logger.error(f"Error processing actuator command: {e}")
            
    def _process_automation_event(self, payload: str, timestamp: float):
        """Process automation events"""
        try:
            data = json.loads(payload)
            
            event = AutomationEvent(
                timestamp=timestamp,
                actuator_id=data.get('actuator_id', 'unknown'),
                action=data.get('action', 'unknown'),
                triggered_by=data.get('triggered_by', []),
                latency_ms=data.get('latency_ms', 0.0)
            )
            
            self.automation_events.append(event)
            
            # If this has latency info, also create a latency event
            if event.latency_ms > 0 and event.triggered_by:
                latency_event = LatencyEvent(
                    timestamp=timestamp,
                    sensor_id=event.triggered_by[0] if event.triggered_by else 'unknown',
                    event_id=f"auto_{int(timestamp*1000)}",
                    latency_ms=event.latency_ms
                )
                self.latency_events.append(latency_event)
                
        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON in automation event: {payload}")
            
    def _process_latency_metric(self, payload: str, timestamp: float):
        """Process latency metric events"""
        try:
            data = json.loads(payload)
            
            event = LatencyEvent(
                timestamp=timestamp,
                sensor_id=data.get('sensor_id', 'unknown'),
                event_id=data.get('event_id', f"lat_{int(timestamp*1000)}"),
                latency_ms=data.get('latency_ms', 0.0),
                stage=data.get('stage', 'e2e')
            )
            
            if event.latency_ms > 0:
                self.latency_events.append(event)
                logger.info(f"Latency event: {event.sensor_id} -> {event.latency_ms:.2f}ms")
                
        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON in latency metric: {payload}")
            
    def get_sensor_summary(self) -> Dict[str, Any]:
        """Get current sensor status summary"""
        with self.lock:
            active_sensors = sum(1 for s in self.sensors.values() 
                               if time.time() - s.last_update < 30)
            
            return {
                'total_sensors': len(self.sensors),
                'active_sensors': active_sensors,
                'sensors': {sid: asdict(sensor) for sid, sensor in self.sensors.items()}
            }
            
    def get_latency_stats(self, window_seconds: Optional[int] = None) -> Dict[str, Any]:
        """Calculate latency statistics for recent events"""
        with self.lock:
            if not self.latency_events:
                return {}
                
            window = window_seconds or self.latency_window_seconds
            cutoff = time.time() - window
            
            # Get recent latencies
            recent_latencies = [
                event.latency_ms 
                for event in self.latency_events 
                if event.timestamp > cutoff and event.latency_ms > 0
            ]
            
            if not recent_latencies:
                return {}
                
            return {
                'mean': float(np.mean(recent_latencies)),
                'median': float(np.median(recent_latencies)),
                'p95': float(np.percentile(recent_latencies, 95)),
                'p99': float(np.percentile(recent_latencies, 99)),
                'min': float(np.min(recent_latencies)),
                'max': float(np.max(recent_latencies)),
                'count': len(recent_latencies),
                'window_seconds': window
            }
            
    def get_latency_history(self, limit: int = 100) -> List[Dict[str, Any]]:
        """Get recent latency events for graphing"""
        with self.lock:
            recent = list(self.latency_events)[-limit:]
            return [asdict(event) for event in recent]
            
    def get_automation_events(self, limit: int = 20) -> List[Dict[str, Any]]:
        """Get recent automation events"""
        with self.lock:
            recent = list(self.automation_events)[-limit:]
            return [asdict(event) for event in recent]
            
    def get_dashboard_data(self) -> Dict[str, Any]:
        """Get complete dashboard data snapshot"""
        return {
            'timestamp': time.time(),
            'mqtt_connected': self.mqtt_connected,
            'sensors': self.get_sensor_summary(),
            'latency_stats': self.get_latency_stats(),
            'latency_history': self.get_latency_history(),
            'automation_events': self.get_automation_events(),
            'system_status': 'running' if self.running else 'stopped'
        }