#!/usr/bin/env python3
"""
Project Olympus - MQTT-Enabled Demo
Standalone demo that sends real sensor data to IRIS dashboard via MQTT.
"""

import time
import sys
import threading
import socket
import json
import logging
import random
import math
from pathlib import Path


# Standalone sensor simulation (no external dependencies)
class BME680Sim:
    """Simulates BME680 sensor with realistic data."""
    
    def __init__(self):
        self.base_temp = 22.0
        self.base_iaq = 55.0
        
    def read(self):
        # Realistic temperature variation
        drift = math.sin(time.time() / 60.0) * 1.5  # Slow drift
        noise = random.gauss(0, 0.3)  # Random noise
        temperature = self.base_temp + drift + noise
        
        # IAQ correlated with temperature
        iaq = self.base_iaq + (temperature - self.base_temp) * 0.8 + random.gauss(0, 1.5)
        iaq = max(40, min(70, iaq))  # Clamp to realistic range
        
        return {
            'temperature': round(temperature, 1),
            'iaq': round(iaq, 1),
            'humidity': round(45 + random.gauss(0, 5), 1),
            'pressure': round(1013.25 + random.gauss(0, 2), 1)
        }


class BatterySim:
    """Simulates battery with realistic discharge."""
    
    def __init__(self):
        self.start_voltage = 4.2
        self.start_time = time.time()
        
    def voltage(self):
        # Simulate slow discharge over time
        elapsed_hours = (time.time() - self.start_time) / 3600
        discharge_rate = 0.01  # V per hour (very slow for demo)
        voltage = self.start_voltage - (elapsed_hours * discharge_rate)
        
        # Add small noise
        voltage += random.gauss(0, 0.005)
        
        return max(3.0, round(voltage, 3))


class OlympusMQTTDemo:
    """MQTT-enabled demo using system packages."""
    
    def __init__(self, mqtt_host='mqtt', mqtt_port=1883):
        self.running = False
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.mqtt_client = None
        
        # Initialize sensor simulations for 5 nodes
        self.nodes = {}
        for node_id in range(5):
            self.nodes[node_id] = {
                'bme680': BME680Sim(),
                'battery': BatterySim(),
                'uptime': 0
            }
        
        # UART server
        self.uart_server = None
        self.client_socket = None
        
        # Set up logging
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        self.log = logging.getLogger('OlympusMQTT')
        
    def connect_mqtt(self):
        """Connect to MQTT broker using system-installed paho-mqtt."""
        try:
            # Import here to avoid ESP-IDF environment issues
            import paho.mqtt.client as mqtt
            
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            self.log.info(f"Connecting to MQTT broker at {self.mqtt_host}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            return True
            
        except ImportError:
            self.log.error("paho-mqtt not available. Install with: pip3 install paho-mqtt")
            return False
        except Exception as e:
            self.log.error(f"Failed to connect to MQTT: {e}")
            return False
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Handle MQTT connection."""
        if rc == 0:
            self.log.info("✅ Connected to MQTT broker")
            # Publish startup message
            client.publish("olympus/system/demo", json.dumps({
                "status": "online",
                "timestamp": time.time(),
                "nodes": list(self.nodes.keys()),
                "version": "mqtt_demo_v1.0"
            }))
        else:
            self.log.error(f"❌ MQTT connection failed with code {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Handle MQTT disconnection."""
        self.log.warning("MQTT disconnected")
    
    def publish_sensor_data(self, node_id, sensor_type, data):
        """Publish sensor data to MQTT."""
        if self.mqtt_client:
            topic = f"olympus/sensors/{node_id}/{sensor_type}"
            payload = json.dumps(data)
            result = self.mqtt_client.publish(topic, payload)
            if result.rc == 0:
                self.log.debug(f"📤 MQTT: {topic}")
            else:
                self.log.warning(f"Failed to publish to {topic}")
    
    def publish_detection_event(self, node_id, detection_count):
        """Publish detection event to MQTT."""
        if self.mqtt_client:
            topic = f"olympus/detections/{node_id}"
            payload = json.dumps({
                "detection_count": detection_count,
                "timestamp": time.time(),
                "node_id": node_id,
                "event_type": "motion_detected"
            })
            self.mqtt_client.publish(topic, payload)
            self.log.info(f"🎯 Detection: Node {node_id} detected {detection_count} objects")
    
    def publish_network_stats(self):
        """Publish simulated network statistics."""
        if self.mqtt_client:
            # Realistic mesh network simulation
            base_pdr = 0.97
            base_latency = 18.0
            base_throughput = 26.0
            
            # Add time-based variation
            time_factor = time.time() / 100
            pdr = base_pdr + math.sin(time_factor) * 0.02
            latency = base_latency + math.sin(time_factor * 1.1) * 3
            throughput = base_throughput + math.sin(time_factor * 0.9) * 2
            
            mesh_stats = {
                "packet_delivery_ratio": round(pdr, 3),
                "avg_latency_ms": round(latency, 1),
                "avg_throughput_mbps": round(throughput, 1),
                "active_nodes": len(self.nodes),
                "timestamp": time.time()
            }
            
            self.mqtt_client.publish("olympus/network/mesh", json.dumps(mesh_stats))
    
    def publish_system_metrics(self):
        """Publish system-wide KPI metrics."""
        if self.mqtt_client:
            # Calculate system metrics
            active_nodes = len(self.nodes)
            
            # Pipeline latency simulation (sensor + network + processing)
            sensor_latency = 25 + random.gauss(0, 3)
            network_latency = 18 + random.gauss(0, 2)
            processing_latency = 12 + random.gauss(0, 1)
            total_latency = sensor_latency + network_latency + processing_latency
            
            # Battery life estimation
            avg_voltage = sum(node['battery'].voltage() for node in self.nodes.values()) / active_nodes
            battery_percent = max(0, min(100, (avg_voltage - 3.0) / 1.2 * 100))
            projected_hours = battery_percent * 2.4  # ~10 days at full charge
            
            # Detection rate with realistic variation
            base_rate = 3.2
            time_of_day_factor = abs(math.sin(time.time() / 1000)) * 2  # Day/night variation
            detection_rate = base_rate + time_of_day_factor + random.gauss(0, 0.5)
            
            system_metrics = {
                "total_pipeline_latency_ms": round(total_latency, 1),
                "projected_battery_hours": round(projected_hours, 1),
                "mesh_packet_loss": round(1.0 - (0.97 + math.sin(time.time() / 100) * 0.02), 3),
                "actor_detections_per_minute": round(detection_rate, 1),
                "simulation_time": round(time.time() - self.start_time, 1),
                "active_sensor_nodes": active_nodes,
                "avg_battery_voltage": round(avg_voltage, 2),
                "timestamp": time.time()
            }
            
            self.mqtt_client.publish("olympus/metrics", json.dumps(system_metrics))
    
    def start_uart_server(self, port=3333):
        """Start UART TCP server for compatibility."""
        try:
            self.uart_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.uart_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.uart_server.bind(('0.0.0.0', port))
            self.uart_server.listen(1)
            
            self.log.info(f"🔌 UART server on port {port}")
            
            def accept_connection():
                try:
                    self.client_socket, addr = self.uart_server.accept()
                    self.log.info(f"📞 UART client: {addr}")
                except:
                    pass
            
            threading.Thread(target=accept_connection, daemon=True).start()
            return True
            
        except Exception as e:
            self.log.error(f"UART server failed: {e}")
            return False
    
    def send_uart(self, data):
        """Send data to UART client."""
        if self.client_socket:
            try:
                self.client_socket.send((data + '\n').encode('utf-8'))
            except:
                pass
    
    def simulate_node_cycle(self, node_id):
        """Simulate one sensor reading cycle for a node."""
        node = self.nodes[node_id]
        
        # Read sensors
        bme_data = node['bme680'].read()
        bme_data.update({
            'timestamp': time.time(),
            'node_id': node_id
        })
        
        battery_voltage = node['battery'].voltage()
        battery_data = {
            'voltage': battery_voltage,
            'timestamp': time.time(),
            'node_id': node_id
        }
        
        display_data = {
            'text': f"Node{node_id} T:{bme_data['temperature']:.1f} IAQ:{bme_data['iaq']:.0f}",
            'timestamp': time.time(),
            'node_id': node_id
        }
        
        # Publish to MQTT
        self.publish_sensor_data(node_id, 'bme680', bme_data)
        self.publish_sensor_data(node_id, 'battery', battery_data)
        self.publish_sensor_data(node_id, 'display', display_data)
        
        # Console output
        msg = f"Node{node_id}: T={bme_data['temperature']:.1f}°C IAQ={bme_data['iaq']:.1f} Bat={battery_voltage:.2f}V"
        self.log.info(msg)
        self.send_uart(msg)
        
        # Simulate detection events occasionally
        if node['uptime'] % 25 == 0 and node['uptime'] > 0:
            detection_count = random.choice([1, 2])
            self.publish_detection_event(node_id, detection_count)
        
        node['uptime'] += 1
    
    def run(self):
        """Run the complete MQTT demo."""
        print("🎯 Project Olympus - MQTT-Enabled Demo")
        print("=" * 50)
        print()
        print("Real sensor data flowing to:")
        print("  📊 IRIS Dashboard: http://localhost:8080")
        print("  🔧 Node-RED: http://localhost:1880")
        print("  📈 InfluxDB: http://localhost:8086")
        print("  🔌 UART: telnet localhost 3333")
        print()
        
        self.start_time = time.time()
        
        # Connect to MQTT
        if not self.connect_mqtt():
            self.log.error("MQTT connection failed - exiting")
            return False
        
        # Start UART server
        self.start_uart_server()
        
        # Boot sequence
        self.log.info("🔄 Booting 5 sensor nodes...")
        for node_id in self.nodes.keys():
            boot_msg = f"Node {node_id}: OLYMPUS ready - sensors online"
            self.log.info(boot_msg)
            self.send_uart(boot_msg)
        
        self.running = True
        
        try:
            self.log.info("📊 Starting sensor monitoring...")
            self.log.info("🛑 Press Ctrl+C to stop")
            print()
            
            cycle = 0
            while self.running:
                # Process all sensor nodes
                for node_id in self.nodes.keys():
                    self.simulate_node_cycle(node_id)
                
                # Network stats every 8 cycles
                if cycle % 8 == 0:
                    self.publish_network_stats()
                
                # System metrics every 5 cycles
                if cycle % 5 == 0:
                    self.publish_system_metrics()
                
                cycle += 1
                time.sleep(1)  # 1 Hz sensor updates
                
        except KeyboardInterrupt:
            self.log.info("\n🛑 Demo stopped")
        finally:
            self.cleanup()
        
        return True
    
    def cleanup(self):
        """Clean up resources."""
        self.log.info("🧹 Cleaning up...")
        self.running = False
        
        if self.mqtt_client:
            self.mqtt_client.publish("olympus/system/demo", json.dumps({
                "status": "offline",
                "timestamp": time.time()
            }))
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        if self.client_socket:
            self.client_socket.close()
        if self.uart_server:
            self.uart_server.close()
        
        self.log.info("✅ Cleanup complete")


def main():
    """Main entry point."""
    demo = OlympusMQTTDemo()
    success = demo.run()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main() 