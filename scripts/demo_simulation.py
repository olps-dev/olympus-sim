#!/usr/bin/env python3
"""
Project Olympus - Working Demo with MQTT Integration
Demonstrates the digital twin concept with real sensor data flowing to IRIS dashboard.
"""

import time
import sys
import threading
import socket
import json
import logging
from pathlib import Path

# Try to import MQTT, fallback to system if not available
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    print("⚠️  paho-mqtt not available in current environment")
    print("📦 Installing paho-mqtt...")
    import subprocess
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "paho-mqtt"])
        import paho.mqtt.client as mqtt
        MQTT_AVAILABLE = True
        print("✅ Successfully installed paho-mqtt")
    except Exception as e:
        print(f"❌ Failed to install paho-mqtt: {e}")
        print("🔄 Running in simulation-only mode (no MQTT)")
        MQTT_AVAILABLE = False

# Add the sim/python directory to the path
sys.path.append(str(Path(__file__).parent.parent / "sim" / "python"))

from sensor_stubs import BME680Stub, SSD1306Stub, BatteryADC

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
log = logging.getLogger('OlympusDemo')


class OlympusWorkingDemo:
    """Working demo that sends real data to MQTT for IRIS dashboard."""
    
    def __init__(self, mqtt_host='mqtt', mqtt_port=1883):
        self.running = False
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.mqtt_client = None
        self.mqtt_available = MQTT_AVAILABLE
        
        # Initialize sensor stubs for multiple nodes
        self.nodes = {}
        for node_id in range(5):  # 5 sensor nodes
            self.nodes[node_id] = {
                'bme680': BME680Stub(),
                'ssd1306': SSD1306Stub(),
                'battery': BatteryADC(),
                'uptime': 0
            }
        
        # UART server for compatibility
        self.uart_server = None
        self.client_socket = None
        
    def connect_mqtt(self):
        """Connect to MQTT broker."""
        if not self.mqtt_available:
            log.warning("MQTT not available, running in simulation-only mode")
            return True
            
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            log.info(f"Connecting to MQTT broker at {self.mqtt_host}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            return True
            
        except Exception as e:
            log.error(f"Failed to connect to MQTT: {e}")
            log.warning("Continuing in simulation-only mode")
            self.mqtt_available = False
            return True
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Handle MQTT connection."""
        if rc == 0:
            log.info("✅ Connected to MQTT broker")
            # Publish initial status
            client.publish("olympus/system/demo", json.dumps({
                "status": "online",
                "timestamp": time.time(),
                "nodes": list(self.nodes.keys())
            }))
        else:
            log.error(f"❌ MQTT connection failed with code {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Handle MQTT disconnection."""
        log.warning("MQTT disconnected")
    
    def publish_sensor_data(self, node_id, sensor_type, data):
        """Publish sensor data to MQTT."""
        if self.mqtt_client and self.mqtt_available:
            topic = f"olympus/sensors/{node_id}/{sensor_type}"
            payload = json.dumps(data)
            self.mqtt_client.publish(topic, payload)
            log.debug(f"📤 MQTT: {topic} => {payload}")
    
    def publish_detection_event(self, node_id, detection_count):
        """Publish detection event to MQTT."""
        if self.mqtt_client and self.mqtt_available:
            topic = f"olympus/detections/{node_id}"
            payload = json.dumps({
                "detection_count": detection_count,
                "timestamp": time.time(),
                "node_id": node_id
            })
            self.mqtt_client.publish(topic, payload)
            log.info(f"🎯 Detection event: Node {node_id} detected {detection_count} objects")
    
    def publish_network_stats(self):
        """Publish simulated network statistics."""
        if self.mqtt_client and self.mqtt_available:
            # Simulate realistic mesh network stats
            mesh_stats = {
                "packet_delivery_ratio": 0.96 + (time.time() % 10) * 0.004,  # 96-100%
                "avg_latency_ms": 15.0 + (time.time() % 5),  # 15-20ms
                "avg_throughput_mbps": 24.0 + (time.time() % 4),  # 24-28 Mbps
                "timestamp": time.time()
            }
            
            self.mqtt_client.publish("olympus/network/mesh", json.dumps(mesh_stats))
    
    def publish_system_metrics(self):
        """Publish system-wide metrics for KPI monitoring."""
        if self.mqtt_client and self.mqtt_available:
            # Calculate realistic system metrics
            active_nodes = len(self.nodes)
            
            # Simulated pipeline latency (sensor + network + processing)
            pipeline_latency = 45 + (time.time() % 30)  # 45-75ms
            
            # Battery life calculation (hours)
            avg_battery_voltage = sum(node['battery'].voltage() for node in self.nodes.values()) / active_nodes
            battery_percent = max(0, min(100, (avg_battery_voltage - 3.0) / 1.2 * 100))
            projected_battery_hours = battery_percent * 2.4  # Up to ~10 days
            
            # Detection rate simulation
            detection_rate = 2.5 + (time.time() % 3)  # 2.5-5.5 per minute
            
            system_metrics = {
                "total_pipeline_latency_ms": pipeline_latency,
                "projected_battery_hours": projected_battery_hours,
                "mesh_packet_loss": 1.0 - (0.96 + (time.time() % 10) * 0.004),
                "actor_detections_per_minute": detection_rate,
                "simulation_time": time.time() - self.start_time,
                "timestamp": time.time()
            }
            
            self.mqtt_client.publish("olympus/metrics", json.dumps(system_metrics))
    
    def start_uart_server(self, port=3333):
        """Start a TCP server that simulates UART output for compatibility."""
        try:
            self.uart_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.uart_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.uart_server.bind(('0.0.0.0', port))
            self.uart_server.listen(1)
            
            log.info(f"🔌 UART TCP server listening on port {port}")
            log.info("💡 Connect with: telnet localhost 3333")
            
            # Accept connection in background
            def accept_connection():
                try:
                    self.client_socket, addr = self.uart_server.accept()
                    log.info(f"📞 UART client connected from {addr}")
                except:
                    pass
            
            threading.Thread(target=accept_connection, daemon=True).start()
            return True
            
        except Exception as e:
            log.error(f"Failed to start UART server: {e}")
            return False
    
    def send_uart_data(self, data):
        """Send data to the connected UART client."""
        if self.client_socket:
            try:
                self.client_socket.send((data + '\n').encode('utf-8'))
                return True
            except:
                return False
        return False
    
    def simulate_node_cycle(self, node_id):
        """Simulate one sensor reading cycle for a node."""
        node = self.nodes[node_id]
        
        # Read BME680 sensor
        bme_data = node['bme680'].read()
        bme_data['timestamp'] = time.time()
        bme_data['node_id'] = node_id
        
        # Read battery voltage
        battery_voltage = node['battery'].voltage()
        battery_data = {
            'voltage': battery_voltage,
            'timestamp': time.time(),
            'node_id': node_id
        }
        
        # Update OLED display
        display_text = f"Node{node_id} T:{bme_data['temperature']:.1f} IAQ:{bme_data['iaq']:.0f}"
        node['ssd1306'].draw_banner(display_text)
        display_data = {
            'text': display_text,
            'timestamp': time.time(),
            'node_id': node_id
        }
        
        # Publish to MQTT
        self.publish_sensor_data(node_id, 'bme680', bme_data)
        self.publish_sensor_data(node_id, 'battery', battery_data)
        self.publish_sensor_data(node_id, 'display', display_data)
        
        # Log to console and UART
        log_msg = f"Node{node_id}: T={bme_data['temperature']:.1f}°C, IAQ={bme_data['iaq']:.1f}, Bat={battery_voltage:.2f}V"
        log.info(log_msg)
        self.send_uart_data(log_msg)
        
        # Simulate occasional detection events
        if node['uptime'] % 30 == 0 and node['uptime'] > 0:  # Every 30 seconds
            detection_count = 1 if time.time() % 2 < 1 else 2  # 1 or 2 detections
            self.publish_detection_event(node_id, detection_count)
        
        node['uptime'] += 1
    
    def run(self):
        """Run the complete working demo."""
        print("🎯 Project Olympus - Working Demo with MQTT Integration")
        print("=" * 60)
        print()
        if self.mqtt_available:
            print("This demo sends real sensor data to MQTT that you can see in:")
            print("  📊 IRIS Dashboard: http://localhost:8080")
            print("  🔧 Node-RED: http://localhost:1880")
            print("  📈 InfluxDB: http://localhost:8086")
        else:
            print("Running in simulation-only mode (no MQTT connection)")
            print("UART output available on port 3333")
        print()
        
        self.start_time = time.time()
        
        # Connect to MQTT
        if not self.connect_mqtt():
            log.error("Demo initialization failed")
            return
        
        # Start UART server for compatibility
        self.start_uart_server()
        
        # Boot sequence
        log.info("🔄 Simulating ESP32 firmware boot for all nodes...")
        for node_id in self.nodes.keys():
            boot_msg = f"Node {node_id}: OLYMPUS_MAIN: System ready - monitoring sensors"
            log.info(boot_msg)
            self.send_uart_data(boot_msg)
        
        self.running = True
        
        try:
            log.info("📊 Starting sensor monitoring loop...")
            log.info("🛑 Press Ctrl+C to stop")
            print()
            
            cycle = 0
            while self.running:
                # Process all nodes
                for node_id in self.nodes.keys():
                    self.simulate_node_cycle(node_id)
                
                # Publish network stats every 10 cycles
                if cycle % 10 == 0:
                    self.publish_network_stats()
                
                # Publish system metrics every 5 cycles
                if cycle % 5 == 0:
                    self.publish_system_metrics()
                
                cycle += 1
                time.sleep(1)  # 1 Hz updates
                
        except KeyboardInterrupt:
            log.info("\n🛑 Demo stopped by user")
        except Exception as e:
            log.error(f"\n❌ Demo error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        log.info("🧹 Cleaning up...")
        self.running = False
        
        if self.mqtt_client and self.mqtt_available:
            # Publish offline status
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
        
        log.info("✅ Cleanup complete")


def main():
    """Main entry point."""
    demo = OlympusWorkingDemo()
    demo.run()


if __name__ == '__main__':
    main() 