#!/usr/bin/env python3
"""
Complete Automation Loop Test
Tests the full pipeline: mmWave sensor -> ROS2 -> MQTT -> Automation -> Actuator control
"""

import paho.mqtt.client as mqtt
import json
import time
import threading
import sys
from datetime import datetime
import statistics

class AutomationLoopTester:
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
        
        # Metrics tracking
        self.metrics = {
            'sensor_messages': 0,
            'presence_detections': 0,
            'actuator_commands': 0,
            'latencies': [],
            'start_time': time.time()
        }
        
        # State tracking
        self.last_sensor_time = 0
        self.last_actuator_time = 0
        self.current_presence = False
        self.current_lamp_state = False
        
        # Test configuration
        self.test_duration = 60  # seconds
        self.running = True
        
    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        """MQTT connection callback - compatible with both old and new paho-mqtt versions"""
        if reason_code == 0:
            print("✅ Connected to MQTT broker")
            # Subscribe to all relevant topics
            topics = [
                "sensor/+/+",
                "actuators/+/+",
                "automation/+"
            ]
            for topic in topics:
                client.subscribe(topic)
            print("📡 Subscribed to test topics")
        else:
            print(f"❌ Failed to connect: {reason_code}")
    
    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()
        current_time = time.time()
        
        try:
            # Handle sensor messages
            if topic.startswith('sensor/'):
                self.handle_sensor_message(topic, payload, current_time)
            
            # Handle actuator messages
            elif topic.startswith('actuators/'):
                self.handle_actuator_message(topic, payload, current_time)
            
            # Handle automation events
            elif topic.startswith('automation/'):
                self.handle_automation_event(topic, payload, current_time)
                
        except Exception as e:
            print(f"❌ Error processing {topic}: {e}")
    
    def handle_sensor_message(self, topic, payload, current_time):
        """Handle sensor messages and track metrics"""
        self.metrics['sensor_messages'] += 1
        
        # Parse presence detection
        presence_detected = False
        
        if 'presence' in topic:
            try:
                data = json.loads(payload)
                presence_detected = data.get('human_present', False)
                if presence_detected:
                    self.metrics['presence_detections'] += 1
                    self.last_sensor_time = current_time
                    print(f"🔍 SENSOR: Presence detected - {data.get('analysis', 'N/A')}")
            except json.JSONDecodeError:
                pass
        
        elif 'human_present' in topic:
            presence_detected = payload.lower() == 'true'
            if presence_detected:
                self.metrics['presence_detections'] += 1
                self.last_sensor_time = current_time
                print(f"🔍 SENSOR: Presence detected (simple)")
        
        # Track presence state changes
        if presence_detected != self.current_presence:
            self.current_presence = presence_detected
            status = "DETECTED" if presence_detected else "CLEARED"
            print(f"🚨 PRESENCE CHANGE: {status}")
    
    def handle_actuator_message(self, topic, payload, current_time):
        """Handle actuator control messages"""
        self.metrics['actuator_commands'] += 1
        
        # Calculate latency if we have a recent sensor detection
        if self.last_sensor_time > 0:
            latency = (current_time - self.last_sensor_time) * 1000  # ms
            self.metrics['latencies'].append(latency)
            print(f"⚡ ACTUATOR: Command received (latency: {latency:.1f}ms)")
        else:
            print(f"⚡ ACTUATOR: Command received")
        
        # Parse actuator state
        if 'lamp_hall' in topic:
            if 'on' in topic or payload == 'on':
                self.current_lamp_state = True
                print(f"💡 LAMP: ON")
            elif 'off' in topic or payload == 'off':
                self.current_lamp_state = False
                print(f"💡 LAMP: OFF")
        
        self.last_actuator_time = current_time
    
    def handle_automation_event(self, topic, payload, current_time):
        """Handle automation system events"""
        try:
            data = json.loads(payload)
            if data.get('event_type') == 'actuator_control':
                latency = data.get('automation_latency_ms', 0)
                actuator = data.get('actuator_id', 'unknown')
                action = data.get('action', 'unknown')
                print(f"🏠 AUTOMATION: {actuator} -> {action} (system latency: {latency:.1f}ms)")
        except json.JSONDecodeError:
            pass
    
    def print_status(self):
        """Print current status and metrics"""
        elapsed = time.time() - self.metrics['start_time']
        
        print("\n" + "="*60)
        print(f"📊 AUTOMATION LOOP TEST STATUS ({elapsed:.1f}s elapsed)")
        print("="*60)
        print(f"Sensor messages:      {self.metrics['sensor_messages']}")
        print(f"Presence detections:  {self.metrics['presence_detections']}")
        print(f"Actuator commands:    {self.metrics['actuator_commands']}")
        
        if self.metrics['latencies']:
            avg_latency = statistics.mean(self.metrics['latencies'])
            max_latency = max(self.metrics['latencies'])
            min_latency = min(self.metrics['latencies'])
            print(f"Latency (avg/min/max): {avg_latency:.1f}/{min_latency:.1f}/{max_latency:.1f}ms")
        
        print(f"Current presence:     {'🟢 YES' if self.current_presence else '🔴 NO'}")
        print(f"Current lamp state:   {'🟡 ON' if self.current_lamp_state else '⚫ OFF'}")
        print("="*60)
    
    def run_test(self):
        """Run the automation loop test"""
        print("🧪 AUTOMATION LOOP TESTER")
        print("="*60)
        print("This test monitors the complete automation pipeline:")
        print("  1. mmWave sensor data (ROS2 -> MQTT bridge)")
        print("  2. Presence detection processing")
        print("  3. Automation rule execution")
        print("  4. Actuator control commands")
        print("  5. End-to-end latency measurement")
        print("")
        print(f"Test will run for {self.test_duration} seconds...")
        print("Move objects in the Gazebo simulation to trigger detection!")
        print("="*60)
        
        try:
            # Connect to MQTT
            self.client.connect("localhost", 1883, 60)
            
            # Start MQTT loop in background
            self.client.loop_start()
            
            # Run test for specified duration
            start_time = time.time()
            last_status_time = start_time
            
            while self.running and (time.time() - start_time) < self.test_duration:
                time.sleep(1)
                
                # Print status every 10 seconds
                if time.time() - last_status_time >= 10:
                    self.print_status()
                    last_status_time = time.time()
            
            # Final status
            self.print_status()
            
            # Test results
            print("\n🏁 TEST COMPLETED")
            print("="*60)
            
            if self.metrics['presence_detections'] > 0 and self.metrics['actuator_commands'] > 0:
                print("✅ SUCCESS: Complete automation loop verified!")
                print(f"   - Detected {self.metrics['presence_detections']} presence events")
                print(f"   - Triggered {self.metrics['actuator_commands']} actuator commands")
                
                if self.metrics['latencies']:
                    avg_latency = statistics.mean(self.metrics['latencies'])
                    if avg_latency < 300:  # Target: <300ms
                        print(f"   - Average latency: {avg_latency:.1f}ms ✅")
                    else:
                        print(f"   - Average latency: {avg_latency:.1f}ms ⚠️ (target: <300ms)")
            else:
                print("⚠️  INCOMPLETE: Automation loop not fully tested")
                if self.metrics['presence_detections'] == 0:
                    print("   - No presence detections (check mmWave sensor)")
                if self.metrics['actuator_commands'] == 0:
                    print("   - No actuator commands (check automation controller)")
            
        except KeyboardInterrupt:
            print("\n\n👋 Test interrupted by user")
        except Exception as e:
            print(f"❌ Test error: {e}")
        finally:
            self.running = False
            self.client.loop_stop()
            self.client.disconnect()
            print("🧪 Test completed")

def main():
    if len(sys.argv) > 1:
        try:
            duration = int(sys.argv[1])
            tester = AutomationLoopTester()
            tester.test_duration = duration
        except ValueError:
            print("Usage: python test_automation_loop.py [duration_seconds]")
            return
    else:
        tester = AutomationLoopTester()
    
    tester.run_test()

if __name__ == "__main__":
    main()
