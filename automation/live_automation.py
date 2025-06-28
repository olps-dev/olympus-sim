#!/usr/bin/env python3
"""
Live Automation Controller for Olympus Dashboard
Publishes real-time events to MQTT for dashboard consumption
"""

import paho.mqtt.client as mqtt
import json
import time
import logging
import os
from typing import Dict, List
from dataclasses import dataclass

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('live_automation')

@dataclass
class SensorEvent:
    sensor_id: str
    human_present: bool
    timestamp: float
    event_id: str
    battery_level_mah: float = 3000.0

class LiveAutomationController:
    """Automation controller that publishes live events for dashboard"""
    
    def __init__(self):
        self.client = mqtt.Client(client_id=f"automation_{int(time.time())}")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        
        # State tracking
        self.sensor_states: Dict[str, SensorEvent] = {}
        self.lamp_state = False
        self.last_action_time = 0
        self.action_cooldown = 3.0  # Seconds between actions
        
        # Event tracking for latency
        self.pending_events: Dict[str, float] = {}  # event_id -> detection_timestamp
        
        # Statistics
        self.stats = {
            'presence_events': 0,
            'automation_actions': 0,
            'start_time': time.time()
        }
        
    def on_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            logger.info("Connected to MQTT broker")
            
            # Subscribe to sensor presence
            client.subscribe("sensor/+/presence")
            client.subscribe("sensor/+/human_present")
            logger.info("Subscribed to sensor topics")
        else:
            logger.error(f"Failed to connect to MQTT: {rc}")
            
    def on_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        logger.warning(f"Disconnected from MQTT (rc={rc})")
        
    def on_message(self, client, userdata, msg):
        """Process incoming sensor messages"""
        try:
            topic = msg.topic
            payload = msg.payload.decode()
            
            if "presence" in topic:
                self.process_presence_message(topic, payload)
            elif "human_present" in topic:
                self.process_human_present_message(topic, payload)
                
        except Exception as e:
            logger.error(f"Error processing message on {topic}: {e}")
            
    def process_presence_message(self, topic: str, payload: str):
        """Process detailed presence messages"""
        try:
            data = json.loads(payload)
            sensor_id = data.get('sensor_id', topic.split('/')[1])
            
            event = SensorEvent(
                sensor_id=sensor_id,
                human_present=data.get('human_present', False),
                timestamp=time.time(),
                event_id=data.get('event_id', f"{sensor_id}_{int(time.time()*1000)}"),
                battery_level_mah=data.get('battery_level_mah', 3000.0)
            )
            
            self.update_sensor_state(event)
            
        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON in presence message: {payload}")
            
    def process_human_present_message(self, topic: str, payload: str):
        """Process simple boolean presence messages"""
        try:
            sensor_id = topic.split('/')[1]
            human_present = payload.lower() == 'true'
            
            event = SensorEvent(
                sensor_id=sensor_id,
                human_present=human_present,
                timestamp=time.time(),
                event_id=f"{sensor_id}_{int(time.time()*1000)}"
            )
            
            self.update_sensor_state(event)
            
        except Exception as e:
            logger.error(f"Error processing human_present: {e}")
            
    def update_sensor_state(self, event: SensorEvent):
        """Update sensor state and trigger automation"""
        # Store sensor state
        self.sensor_states[event.sensor_id] = event
        
        if event.human_present:
            self.stats['presence_events'] += 1
            self.pending_events[event.event_id] = event.timestamp
            logger.info(f"Presence detected: {event.sensor_id}")
            
        # Publish sensor status update
        self.publish_sensor_status(event)
        
        # Process automation rules
        self.process_automation_rules()
        
    def publish_sensor_status(self, event: SensorEvent):
        """Publish sensor status update for dashboard"""
        status_data = {
            'sensor_id': event.sensor_id,
            'human_present': event.human_present,
            'battery_level_mah': event.battery_level_mah,
            'timestamp': event.timestamp,
            'event_id': event.event_id
        }
        
        self.client.publish(f"sensor/{event.sensor_id}/status", 
                          json.dumps(status_data), qos=0)
        
    def process_automation_rules(self):
        """Apply automation rules based on sensor states"""
        current_time = time.time()
        
        # Cooldown check
        if current_time - self.last_action_time < self.action_cooldown:
            return
            
        # Check if any sensor detects presence
        active_sensors = []
        for sensor_id, event in self.sensor_states.items():
            # Consider recent events (last 10 seconds)
            if (event.human_present and 
                current_time - event.timestamp < 10):
                active_sensors.append(sensor_id)
                
        # Determine lamp state
        should_be_on = len(active_sensors) > 0
        
        # Take action if state should change
        if should_be_on != self.lamp_state:
            self.control_lamp(should_be_on, active_sensors)
            self.lamp_state = should_be_on
            self.last_action_time = current_time
            
    def control_lamp(self, turn_on: bool, triggered_by: List[str]):
        """Control lamp and publish automation event"""
        action = "on" if turn_on else "off"
        timestamp = time.time()
        
        # Calculate latency for this automation action
        automation_latency = 0.0
        if turn_on and triggered_by:
            # Find the most recent event from triggering sensors
            recent_timestamps = []
            for sensor_id in triggered_by:
                if sensor_id in self.sensor_states:
                    recent_timestamps.append(self.sensor_states[sensor_id].timestamp)
            
            if recent_timestamps:
                detection_time = max(recent_timestamps)
                automation_latency = (timestamp - detection_time) * 1000  # ms
                
        # Publish actuator command
        command_topic = f"actuators/lamp_hall/command"
        command_data = {
            'action': action,
            'timestamp': timestamp,
            'triggered_by': triggered_by
        }
        self.client.publish(command_topic, json.dumps(command_data), qos=1)
        
        # Publish automation event for dashboard
        automation_event = {
            'event_type': 'lamp_control',
            'actuator_id': 'lamp_hall',
            'action': action,
            'triggered_by': triggered_by,
            'automation_latency_ms': automation_latency,
            'timestamp': timestamp
        }
        self.client.publish("automation/events", json.dumps(automation_event), qos=0)
        
        # Publish latency metric if we have end-to-end data
        if automation_latency > 0:
            latency_metric = {
                'sensor_id': triggered_by[0] if triggered_by else 'unknown',
                'event_id': f"auto_{int(timestamp*1000)}",
                'latency_ms': automation_latency,
                'stage': 'e2e',
                'timestamp': timestamp
            }
            self.client.publish("metrics/e2e_latency", json.dumps(latency_metric), qos=0)
            
        # Update statistics
        self.stats['automation_actions'] += 1
        
        # Log action
        status = "ON" if turn_on else "OFF"
        sensors_str = ", ".join(triggered_by) if triggered_by else "manual"
        latency_str = f" ({automation_latency:.1f}ms)" if automation_latency > 0 else ""
        logger.info(f"AUTOMATION: lamp_hall -> {status} (triggered by: {sensors_str}){latency_str}")
        
    def run(self):
        """Run the automation controller"""
        logger.info("Starting Live Automation Controller")
        
        try:
            # Connect to MQTT
            mqtt_broker = os.getenv('MQTT_BROKER', 'localhost')
            mqtt_port = int(os.getenv('MQTT_PORT', '1883'))
            
            logger.info(f"Connecting to MQTT broker: {mqtt_broker}:{mqtt_port}")
            self.client.connect(mqtt_broker, mqtt_port, 60)
            
            # Start MQTT loop
            self.client.loop_forever()
            
        except KeyboardInterrupt:
            logger.info("Shutting down automation controller")
            self.print_final_stats()
        except Exception as e:
            logger.error(f"Automation controller error: {e}")
        finally:
            self.client.disconnect()
            
    def print_final_stats(self):
        """Print final statistics"""
        runtime = time.time() - self.stats['start_time']
        logger.info(f"Final Statistics:")
        logger.info(f"  Runtime: {runtime:.1f} seconds")
        logger.info(f"  Presence events: {self.stats['presence_events']}")
        logger.info(f"  Automation actions: {self.stats['automation_actions']}")
        logger.info(f"  Active sensors: {len(self.sensor_states)}")

if __name__ == "__main__":
    controller = LiveAutomationController()
    controller.run()