#!/usr/bin/env python3
"""
Test script for mmWave MQTT Bridge
Subscribes to MQTT topics to verify the bridge is working
"""

import paho.mqtt.client as mqtt
import json
import time
import sys

class MQTTTester:
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
        self.presence_count = 0
        self.last_presence = None
        
    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        """MQTT connection callback - compatible with both old and new paho-mqtt versions"""
        if reason_code == 0:
            print("✅ Connected to MQTT broker successfully")
            # Subscribe to all sensor topics
            client.subscribe("sensor/+/+")
            client.subscribe("sensor/mmwave1/presence")
            client.subscribe("sensor/mmwave1/human_present")
            client.subscribe("sensor/mmwave1/status")
            print("📡 Subscribed to sensor topics")
        else:
            print(f"❌ Failed to connect: {reason_code}")
    
    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()
        
        print(f"\n📨 Topic: {topic}")
        
        # Try to parse as JSON first
        try:
            data = json.loads(payload)
            print(f"📄 Data: {json.dumps(data, indent=2)}")
            
            # Track presence changes
            if topic.endswith('/presence') and 'human_present' in data:
                current_presence = data['human_present']
                if current_presence != self.last_presence:
                    self.presence_count += 1
                    status = "🟢 DETECTED" if current_presence else "🔴 CLEAR"
                    print(f"🚨 PRESENCE CHANGE #{self.presence_count}: {status}")
                    if 'analysis' in data:
                        print(f"   Analysis: {data['analysis']}")
                    self.last_presence = current_presence
                    
        except json.JSONDecodeError:
            # Simple string payload
            print(f"📄 Data: {payload}")
            
            # Track simple boolean presence
            if topic.endswith('/human_present'):
                current_presence = payload.lower() == 'true'
                if current_presence != self.last_presence:
                    self.presence_count += 1
                    status = "🟢 DETECTED" if current_presence else "🔴 CLEAR"
                    print(f"🚨 PRESENCE CHANGE #{self.presence_count}: {status}")
                    self.last_presence = current_presence
        
        print("-" * 50)
    
    def run(self):
        print("🔍 mmWave MQTT Bridge Tester")
        print("=" * 50)
        print("This script will monitor MQTT topics for mmWave sensor data.")
        print("Expected topics:")
        print("  - sensor/mmwave1/presence (detailed JSON)")
        print("  - sensor/mmwave1/human_present (simple boolean)")
        print("  - sensor/mmwave1/status (periodic status)")
        print("\nPress Ctrl+C to exit")
        print("=" * 50)
        
        try:
            self.client.connect("localhost", 1883, 60)
            self.client.loop_forever()
        except KeyboardInterrupt:
            print("\n\n👋 Shutting down...")
        except Exception as e:
            print(f"❌ Error: {e}")
        finally:
            self.client.disconnect()
            print(f"📊 Total presence changes detected: {self.presence_count}")

if __name__ == "__main__":
    tester = MQTTTester()
    tester.run()
