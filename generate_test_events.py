#!/usr/bin/env python3
"""
Generate test automation events for dashboard demonstration
"""

import paho.mqtt.client as mqtt
import json
import time
import random

def generate_events():
    client = mqtt.Client()
    client.connect("localhost", 1883, 60)
    
    print("Generating test automation events...")
    
    for i in range(10):
        timestamp = time.time()
        
        # Generate automation event
        automation_event = {
            'event_type': 'lamp_control',
            'actuator_id': 'lamp_hall',
            'action': 'on' if i % 2 == 0 else 'off',
            'triggered_by': ['mmwave1', 'mmwave2'],
            'automation_latency_ms': random.uniform(10.0, 50.0),
            'timestamp': timestamp
        }
        client.publish("automation/events", json.dumps(automation_event), qos=0)
        print(f"Published automation event {i+1}: {automation_event['action']}")
        
        # Generate latency metric
        latency_metric = {
            'sensor_id': random.choice(['mmwave1', 'mmwave2']),
            'event_id': f"test_{int(timestamp*1000)}",
            'latency_ms': random.uniform(15.0, 75.0),
            'stage': 'e2e',
            'timestamp': timestamp
        }
        client.publish("metrics/e2e_latency", json.dumps(latency_metric), qos=0)
        print(f"Published latency metric {i+1}: {latency_metric['latency_ms']:.1f}ms")
        
        time.sleep(2)
    
    client.disconnect()
    print("Test event generation complete")

if __name__ == "__main__":
    generate_events()