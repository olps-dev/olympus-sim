#!/usr/bin/env python3
"""
Test script to validate latency logging fix
"""
import sys
import time
sys.path.append('/app/metrics')

from latency_battery_tracker import LatencyBatteryTracker

def test_latency_logging():
    print("Testing latency logging fix...")
    
    # Create tracker
    tracker = LatencyBatteryTracker()
    
    # Simulate sensor detection
    sensor_id = "mmwave1"
    event_id = tracker.log_pointcloud_received(sensor_id)
    print(f"Created event: {event_id}")
    
    # Simulate MQTT publish
    time.sleep(0.01)  # 10ms delay
    tracker.log_mqtt_published(sensor_id, event_id)
    
    # Simulate automation triggered  
    time.sleep(0.005)  # 5ms delay
    tracker.log_automation_triggered(event_id, sensor_id)
    
    # Simulate lamp toggle
    time.sleep(0.002)  # 2ms delay  
    tracker.log_lamp_toggled(event_id, sensor_id)
    
    print("Test completed - check latest CSV entry")

if __name__ == "__main__":
    test_latency_logging()