#!/usr/bin/env python3
"""
Working automation controller for Olympus simulation
"""
import paho.mqtt.client as mqtt
import json
import time
import logging
import sys
import os

# Add metrics path for imports
sys.path.append('/app/metrics')
from latency_battery_tracker import LatencyBatteryTracker

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('working_automation')

class WorkingAutomationController:
    def __init__(self):
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        # State tracking
        self.sensor_states = {}
        self.lamp_state = False
        self.last_action_time = 0
        self.action_cooldown = 2.0
        
        # Initialize metrics tracker
        self.metrics_tracker = LatencyBatteryTracker()
        
        # Statistics
        self.stats = {
            'messages_received': 0,
            'lamp_commands_sent': 0,
            'human_detections': 0,
            'start_time': time.time()
        }
        
    def on_connect(self, client, userdata, flags, rc, properties=None):
        logger.info(f"AUTOMATION: Connected with result code {rc}")
        if rc == 0:
            client.subscribe("sensor/+/presence")
            logger.info("AUTOMATION: Subscribed to sensor presence topics")
        else:
            logger.error(f"AUTOMATION: Failed to connect: {rc}")
    
    def on_message(self, client, userdata, msg):
        self.stats['messages_received'] += 1
        
        try:
            data = json.loads(msg.payload.decode())
            sensor_id = data.get('sensor_id', 'unknown')
            human_present = data.get('human_present', False)
            event_id = data.get('event_id')
            
            # Update sensor state
            self.sensor_states[sensor_id] = {
                'human_present': human_present,
                'last_update': time.time(),
                'event_id': event_id
            }
            
            if human_present:
                self.stats['human_detections'] += 1
            
            # Log automation triggered for latency tracking
            if event_id:
                self.metrics_tracker.log_automation_triggered(event_id, sensor_id)
            
            # Process automation rules
            self.process_automation()
            
        except Exception as e:
            logger.error(f"AUTOMATION: Error processing message: {e}")
    
    def process_automation(self):
        current_time = time.time()
        
        # Cooldown check
        if current_time - self.last_action_time < self.action_cooldown:
            return
        
        # Check if any sensor detects presence
        any_presence = any(
            state.get('human_present', False) 
            for state in self.sensor_states.values()
        )
        
        # Take action if state changed
        if any_presence != self.lamp_state:
            self.control_lamp(any_presence)
            self.lamp_state = any_presence
            self.last_action_time = current_time
    
    def control_lamp(self, turn_on):
        action = "on" if turn_on else "off"
        topic = f"actuators/lamp_hall/{action}"
        
        # Find triggering sensors and event IDs
        triggering_sensors = []
        event_ids = []
        
        if turn_on:
            for sensor_id, state in self.sensor_states.items():
                if state.get('human_present', False):
                    triggering_sensors.append(sensor_id)
                    if state.get('event_id'):
                        event_ids.append(state['event_id'])
        
        # Create control payload
        control_payload = {
            'timestamp': time.time(),
            'action': action,
            'triggered_by': triggering_sensors,
            'event_ids': event_ids
        }
        
        # Publish commands
        self.client.publish(topic, json.dumps(control_payload), qos=1)
        self.client.publish("actuators/lamp_hall/state", action, qos=1)
        
        # Track statistics
        self.stats['lamp_commands_sent'] += 1
        
        # Log lamp toggle events for latency tracking
        for event_id in event_ids:
            for sensor_id in triggering_sensors:
                if (sensor_id in self.sensor_states and 
                    self.sensor_states[sensor_id].get('event_id') == event_id):
                    self.metrics_tracker.log_lamp_toggled(event_id, sensor_id)
        
        # Log action
        status = "ON" if turn_on else "OFF"
        sensors_str = ", ".join(triggering_sensors) if triggering_sensors else "none"
        logger.info(f"AUTOMATION: lamp_hall -> {status} (triggered by: {sensors_str})")
    
    def run(self):
        logger.info("Working Automation Controller Starting")
        logger.info("Connecting to MQTT...")
        
        try:
            mqtt_broker = os.getenv('MQTT_BROKER', 'mosquitto')
            mqtt_port = int(os.getenv('MQTT_PORT', '1883'))
            
            result = self.client.connect(mqtt_broker, mqtt_port, 60)
            logger.info(f"Connect result: {result}")
            
            self.client.loop_forever()
            
        except KeyboardInterrupt:
            logger.info("AUTOMATION: Shutting down...")
            
            # Generate final metrics
            try:
                plots = self.metrics_tracker.generate_plots()
                if plots:
                    logger.info(f"AUTOMATION: Generated plots: {list(plots.keys())}")
                
                # Print final stats
                runtime = time.time() - self.stats['start_time']
                logger.info(f"AUTOMATION: Final Statistics:")
                logger.info(f"  Runtime: {runtime:.1f} seconds")
                logger.info(f"  Messages received: {self.stats['messages_received']}")
                logger.info(f"  Human detections: {self.stats['human_detections']}")
                logger.info(f"  Lamp commands sent: {self.stats['lamp_commands_sent']}")
                
                summary = self.metrics_tracker.get_metrics_summary()
                if summary:
                    latency = summary.get('latency', {})
                    logger.info(f"  End-to-end latency samples: {latency.get('count', 0)}")
                    if latency.get('count', 0) > 0:
                        logger.info(f"  Mean latency: {latency.get('mean_ms', 0):.1f}ms")
                        logger.info(f"  P95 latency: {latency.get('p95_ms', 0):.1f}ms")
                    
            except Exception as e:
                logger.error(f"AUTOMATION: Error generating final metrics: {e}")
                
        except Exception as e:
            logger.error(f"AUTOMATION: Error: {e}")
        finally:
            self.client.disconnect()
            logger.info("AUTOMATION: controller stopped")

if __name__ == "__main__":
    controller = WorkingAutomationController()
    controller.run()