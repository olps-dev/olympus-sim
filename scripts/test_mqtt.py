#!/usr/bin/env python3
"""
Project Olympus - MQTT Test Script
Simple test to publish sensor data and verify IRIS dashboard integration.
"""

import time
import json
import random
import math

def main():
    """Test MQTT publishing with simulated sensor data."""
    try:
        import paho.mqtt.client as mqtt
        print("✅ paho-mqtt available")
    except ImportError:
        print("❌ paho-mqtt not available")
        return
    
    # Connect to MQTT
    client = mqtt.Client()
    
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("✅ Connected to MQTT broker")
        else:
            print(f"❌ MQTT connection failed: {rc}")
    
    client.on_connect = on_connect
    
    try:
        client.connect("mqtt", 1883, 60)
        client.loop_start()
        
        print("🎯 Publishing test sensor data...")
        print("📊 Check IRIS Dashboard: http://localhost:8080")
        print("🛑 Press Ctrl+C to stop")
        
        for cycle in range(100):  # Run for 100 cycles
            # Publish data for 5 sensor nodes
            for node_id in range(5):
                # BME680 sensor data
                temp = 22.0 + math.sin(time.time() / 60) * 2 + random.gauss(0, 0.3)
                iaq = 55.0 + (temp - 22.0) * 0.8 + random.gauss(0, 1.5)
                
                bme_data = {
                    'temperature': round(temp, 1),
                    'iaq': round(max(40, min(70, iaq)), 1),
                    'humidity': round(45 + random.gauss(0, 5), 1),
                    'pressure': round(1013.25 + random.gauss(0, 2), 1),
                    'timestamp': time.time(),
                    'node_id': node_id
                }
                
                # Battery data
                battery_data = {
                    'voltage': round(4.2 - random.uniform(0, 0.2), 3),
                    'timestamp': time.time(),
                    'node_id': node_id
                }
                
                # Display data
                display_data = {
                    'text': f"Node{node_id} T:{bme_data['temperature']:.1f} IAQ:{bme_data['iaq']:.0f}",
                    'timestamp': time.time(),
                    'node_id': node_id
                }
                
                # Publish to MQTT
                client.publish(f"olympus/sensors/{node_id}/bme680", json.dumps(bme_data))
                client.publish(f"olympus/sensors/{node_id}/battery", json.dumps(battery_data))
                client.publish(f"olympus/sensors/{node_id}/display", json.dumps(display_data))
                
                print(f"📤 Node{node_id}: T={bme_data['temperature']:.1f}°C, IAQ={bme_data['iaq']:.1f}, Bat={battery_data['voltage']:.2f}V")
            
            # Publish network stats every 10 cycles
            if cycle % 10 == 0:
                mesh_stats = {
                    "packet_delivery_ratio": round(0.97 + math.sin(time.time() / 100) * 0.02, 3),
                    "avg_latency_ms": round(18.0 + math.sin(time.time() / 100) * 3, 1),
                    "avg_throughput_mbps": round(26.0 + math.sin(time.time() / 100) * 2, 1),
                    "active_nodes": 5,
                    "timestamp": time.time()
                }
                client.publish("olympus/network/mesh", json.dumps(mesh_stats))
                print("📶 Published network stats")
            
            # Publish system metrics every 5 cycles
            if cycle % 5 == 0:
                system_metrics = {
                    "total_pipeline_latency_ms": round(55 + random.gauss(0, 5), 1),
                    "projected_battery_hours": round(240 + random.gauss(0, 10), 1),
                    "mesh_packet_loss": round(0.03 + random.gauss(0, 0.01), 3),
                    "actor_detections_per_minute": round(3.2 + random.gauss(0, 0.5), 1),
                    "simulation_time": cycle,
                    "active_sensor_nodes": 5,
                    "avg_battery_voltage": round(4.1 + random.gauss(0, 0.05), 2),
                    "timestamp": time.time()
                }
                client.publish("olympus/metrics", json.dumps(system_metrics))
                print("📊 Published system metrics")
            
            # Simulate detection events occasionally
            if cycle % 20 == 0 and cycle > 0:
                detection_data = {
                    "detection_count": random.choice([1, 2]),
                    "timestamp": time.time(),
                    "node_id": random.choice([0, 1, 2, 3, 4]),
                    "event_type": "motion_detected"
                }
                client.publish(f"olympus/detections/{detection_data['node_id']}", json.dumps(detection_data))
                print(f"🎯 Detection event: Node {detection_data['node_id']}")
            
            time.sleep(1)  # 1 Hz updates
            
    except KeyboardInterrupt:
        print("\n🛑 Test stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        client.loop_stop()
        client.disconnect()
        print("✅ MQTT test complete")

if __name__ == '__main__':
    main() 