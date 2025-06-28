#!/usr/bin/env python3
"""
Simple test automation to debug connection issues
"""
import paho.mqtt.client as mqtt
import json
import time
import os
import sys

# Add paths for imports
sys.path.append('/app/network')
sys.path.append('/app/metrics')

def on_connect(client, userdata, flags, rc, properties=None):
    print(f"Connected with result code {rc}")
    if rc == 0:
        print("Successfully connected to MQTT")
        # Subscribe to test topic
        client.subscribe("sensor/+/presence")
        print("Subscribed to sensor/+/presence")
    else:
        print(f"Failed to connect with code {rc}")

def on_message(client, userdata, msg):
    print(f"Received message: {msg.topic} -> {msg.payload.decode()}")
    
    # Try to parse and process
    try:
        data = json.loads(msg.payload.decode())
        sensor_id = data.get('sensor_id', 'unknown')
        human_present = data.get('human_present', False)
        print(f"AUTOMATION TEST: {sensor_id} detected human: {human_present}")
        
        if human_present:
            # Send lamp control
            lamp_topic = "actuators/lamp_hall/on"
            control_payload = {"action": "on", "triggered_by": sensor_id, "test": True}
            result = client.publish(lamp_topic, json.dumps(control_payload))
            print(f"Sent lamp control command: {result.rc}")
    except Exception as e:
        print(f"Error processing message: {e}")

def main():
    print("Starting automation test...")
    
    # Create MQTT client
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect
    mqtt_broker = os.getenv('MQTT_BROKER', 'localhost')
    mqtt_port = int(os.getenv('MQTT_PORT', '1883'))
    
    print(f"Connecting to {mqtt_broker}:{mqtt_port}")
    
    try:
        result = client.connect(mqtt_broker, mqtt_port, 60)
        print(f"Connect result: {result}")
        
        print("Starting MQTT loop...")
        client.loop_start()
        
        # Keep running for a while
        print("Running for 30 seconds...")
        time.sleep(30)
        
        print("Stopping...")
        client.loop_stop()
        client.disconnect()
        print("Test completed successfully")
        
    except Exception as e:
        print(f"Error during test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()