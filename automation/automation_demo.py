#!/usr/bin/env python3
"""
Automation Demo for Olympus Simulation
Demonstrates Step 2: Close the automation loop
- Subscribes to sensor/+/presence topics
- When human_present is true, publishes actuators/lamp_hall/on
- Simulates basic home automation logic
"""

import paho.mqtt.client as mqtt
import json
import time
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('automation_demo')

class AutomationController:
    def __init__(self):
        # Create MQTT client - compatible with older paho-mqtt versions
        try:
            # Try new API first (paho-mqtt >= 2.0)
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        except AttributeError:
            # Fall back to old API (paho-mqtt < 2.0)
            self.client = mqtt.Client()
        
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        # State tracking
        self.sensor_states = {}
        self.lamp_state = False
        self.last_action_time = 0
        self.action_cooldown = 2.0  # Seconds between actions
        
        # Automation rules
        self.automation_rules = {
            'lamp_hall': {
                'trigger_sensors': ['mmwave1'],  # Sensors that can trigger this actuator
                'action_on': 'on',
                'action_off': 'off',
                'auto_off_delay': 30.0  # Auto turn off after 30 seconds
            }
        }
        
    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        """MQTT connection callback - compatible with both old and new paho-mqtt versions"""
        if reason_code == 0:
            logger.info("✅ Connected to MQTT broker")
            # Subscribe to all sensor presence topics
            client.subscribe("sensor/+/presence")
            client.subscribe("sensor/+/human_present")
            logger.info("📡 Subscribed to sensor presence topics")
        else:
            logger.error(f"❌ Failed to connect: {reason_code}")
    
    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()
        
        try:
            # Parse topic to extract sensor info
            topic_parts = topic.split('/')
            if len(topic_parts) >= 3 and topic_parts[0] == 'sensor':
                sensor_id = topic_parts[1]
                message_type = topic_parts[2]
                
                # Handle different message types
                if message_type == 'presence':
                    self.handle_presence_message(sensor_id, payload)
                elif message_type == 'human_present':
                    self.handle_simple_presence(sensor_id, payload)
                    
        except Exception as e:
            logger.error(f"Error processing message from {topic}: {e}")
    
    def handle_presence_message(self, sensor_id, payload):
        """Handle detailed presence JSON messages"""
        try:
            data = json.loads(payload)
            human_present = data.get('human_present', False)
            analysis = data.get('analysis', 'No analysis')
            
            logger.info(f"📊 {sensor_id}: {human_present} ({analysis})")
            
            # Update sensor state
            self.sensor_states[sensor_id] = {
                'human_present': human_present,
                'last_update': time.time(),
                'analysis': analysis
            }
            
            # Trigger automation
            self.process_automation_rules()
            
        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON from {sensor_id}: {payload}")
    
    def handle_simple_presence(self, sensor_id, payload):
        """Handle simple boolean presence messages"""
        human_present = payload.lower() == 'true'
        
        logger.info(f"📊 {sensor_id}: {human_present} (simple)")
        
        # Update sensor state
        if sensor_id not in self.sensor_states:
            self.sensor_states[sensor_id] = {}
        
        self.sensor_states[sensor_id].update({
            'human_present': human_present,
            'last_update': time.time()
        })
        
        # Trigger automation
        self.process_automation_rules()
    
    def process_automation_rules(self):
        """Process automation rules based on current sensor states"""
        current_time = time.time()
        
        # Avoid rapid-fire actions
        if current_time - self.last_action_time < self.action_cooldown:
            return
        
        # Check each automation rule
        for actuator_id, rule in self.automation_rules.items():
            trigger_sensors = rule['trigger_sensors']
            
            # Check if any trigger sensor detects presence
            any_presence = False
            active_sensors = []
            
            for sensor_id in trigger_sensors:
                if sensor_id in self.sensor_states:
                    sensor_state = self.sensor_states[sensor_id]
                    if sensor_state.get('human_present', False):
                        any_presence = True
                        active_sensors.append(sensor_id)
            
            # Determine desired actuator state
            desired_state = any_presence
            
            # Take action if state should change
            if desired_state != self.lamp_state:
                action = rule['action_on'] if desired_state else rule['action_off']
                self.control_actuator(actuator_id, action, active_sensors)
                self.lamp_state = desired_state
                self.last_action_time = current_time
    
    def control_actuator(self, actuator_id, action, trigger_sensors):
        """Send control command to actuator"""
        actuator_topic = f"actuators/{actuator_id}/{action}"
        
        # Create control payload
        control_payload = {
            'timestamp': time.time(),
            'action': action,
            'triggered_by': trigger_sensors,
            'automation_controller': 'olympus_demo'
        }
        
        # Publish control command
        self.client.publish(actuator_topic, json.dumps(control_payload), qos=1)
        
        # Also publish simple command
        simple_topic = f"actuators/{actuator_id}/state"
        self.client.publish(simple_topic, action, qos=1)
        
        # Log action
        status = "🟡 ON" if action == 'on' else "⚫ OFF"
        sensors_str = ", ".join(trigger_sensors) if trigger_sensors else "none"
        logger.info(f"🏠 AUTOMATION: {actuator_id} -> {status} (triggered by: {sensors_str})")
        
        # Publish automation event
        event_topic = "automation/events"
        event_payload = {
            'timestamp': time.time(),
            'event_type': 'actuator_control',
            'actuator_id': actuator_id,
            'action': action,
            'triggered_by': trigger_sensors,
            'automation_latency_ms': (time.time() - max(
                [self.sensor_states[s]['last_update'] for s in trigger_sensors] + [0]
            )) * 1000 if trigger_sensors else 0
        }
        self.client.publish(event_topic, json.dumps(event_payload), qos=1)
    
    def run(self):
        logger.info("🏠 Olympus Automation Controller Starting")
        logger.info("=" * 60)
        logger.info("This controller demonstrates end-to-end automation:")
        logger.info("  1. Monitors sensor/+/presence for human detection")
        logger.info("  2. Controls actuators/lamp_hall/on when presence detected")
        logger.info("  3. Publishes automation events and latency metrics")
        logger.info("")
        logger.info("Expected flow:")
        logger.info("  sensor/mmwave1/presence -> actuators/lamp_hall/on")
        logger.info("")
        logger.info("Press Ctrl+C to exit")
        logger.info("=" * 60)
        
        try:
            self.client.connect("localhost", 1883, 60)
            self.client.loop_forever()
        except KeyboardInterrupt:
            logger.info("\n👋 Shutting down automation controller...")
        except Exception as e:
            logger.error(f"❌ Error: {e}")
        finally:
            self.client.disconnect()
            logger.info("🏠 Automation controller stopped")

if __name__ == "__main__":
    controller = AutomationController()
    controller.run()
