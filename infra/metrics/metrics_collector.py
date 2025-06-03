#!/usr/bin/env python3
"""
Project Olympus - KPI Metrics Collector
Collects and analyzes system performance metrics
"""

import time
import json
import logging
import signal
import sys
import os
from typing import Dict, Any, List
from datetime import datetime

import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
import schedule

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
log = logging.getLogger('MetricsCollector')

class OlympusMetricsCollector:
    """Collects and analyzes Olympus system metrics."""
    
    def __init__(self):
        # Configuration from environment
        self.mqtt_url = os.getenv('MQTT_URL', 'mqtt://localhost:1883')
        self.influx_url = os.getenv('INFLUX_URL', 'http://localhost:8086')
        self.influx_token = os.getenv('INFLUX_TOKEN', 'olympus_token')
        self.influx_org = os.getenv('INFLUX_ORG', 'olympus')
        self.influx_bucket = os.getenv('INFLUX_BUCKET', 'olympus')
        
        # Clients
        self.mqtt_client = None
        self.influx_client = None
        self.influx_write_api = None
        
        # Metrics storage
        self.metrics_buffer = []
        self.sensor_data = {}
        self.network_stats = {}
        self.kpi_history = []
        
        # State
        self.running = False
        self.start_time = time.time()
        
    def init_connections(self):
        """Initialize MQTT and InfluxDB connections."""
        try:
            # MQTT Client
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            
            # Extract host and port from MQTT URL
            mqtt_parts = self.mqtt_url.replace('mqtt://', '').split(':')
            mqtt_host = mqtt_parts[0]
            mqtt_port = int(mqtt_parts[1]) if len(mqtt_parts) > 1 else 1883
            
            self.mqtt_client.connect(mqtt_host, mqtt_port, 60)
            self.mqtt_client.loop_start()
            
            log.info(f"Connected to MQTT: {self.mqtt_url}")
            
            # InfluxDB Client
            self.influx_client = InfluxDBClient(
                url=self.influx_url,
                token=self.influx_token,
                org=self.influx_org
            )
            self.influx_write_api = self.influx_client.write_api(write_options=SYNCHRONOUS)
            
            log.info(f"Connected to InfluxDB: {self.influx_url}")
            
            return True
            
        except Exception as e:
            log.error(f"Failed to initialize connections: {e}")
            return False
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Handle MQTT connection."""
        if rc == 0:
            log.info("✅ Connected to MQTT broker")
            # Subscribe to all Olympus topics
            client.subscribe("olympus/+/+/+")
            client.subscribe("olympus/+/+")
            client.subscribe("olympus/+")
        else:
            log.error(f"❌ MQTT connection failed with code {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Process incoming MQTT messages."""
        try:
            topic = msg.topic
            data = json.loads(msg.payload.decode())
            timestamp = time.time()
            
            # Store message for analysis
            self.metrics_buffer.append({
                'topic': topic,
                'data': data,
                'timestamp': timestamp
            })
            
            # Process by topic type
            topic_parts = topic.split('/')
            
            if len(topic_parts) >= 4 and topic_parts[1] == 'sensors':
                # Sensor data
                node_id = topic_parts[2]
                sensor_type = topic_parts[3]
                
                if node_id not in self.sensor_data:
                    self.sensor_data[node_id] = {}
                
                self.sensor_data[node_id][sensor_type] = {
                    **data,
                    'timestamp': timestamp
                }
            
            elif len(topic_parts) >= 3 and topic_parts[1] == 'network':
                # Network statistics
                network_type = topic_parts[2]
                self.network_stats[network_type] = {
                    **data,
                    'timestamp': timestamp
                }
            
            # Write to InfluxDB
            self.write_to_influx(topic, data, timestamp)
            
        except Exception as e:
            log.error(f"Error processing MQTT message: {e}")
    
    def write_to_influx(self, topic: str, data: Dict[str, Any], timestamp: float):
        """Write metrics to InfluxDB."""
        try:
            if not self.influx_write_api:
                return
            
            topic_parts = topic.split('/')
            measurement = f"olympus_{topic_parts[1]}" if len(topic_parts) > 1 else "olympus_misc"
            
            # Create tags
            tags = {'topic': topic}
            if len(topic_parts) >= 3:
                tags['node_id'] = topic_parts[2]
            if len(topic_parts) >= 4:
                tags['sensor_type'] = topic_parts[3]
            
            # Create fields from data
            fields = {}
            for key, value in data.items():
                if isinstance(value, (int, float)):
                    fields[key] = value
                elif isinstance(value, str):
                    try:
                        # Try to convert string numbers
                        fields[key] = float(value)
                    except:
                        fields[f"{key}_str"] = value
                elif isinstance(value, bool):
                    fields[key] = int(value)
            
            if fields:  # Only write if we have numeric fields
                point = Point(measurement) \
                    .time(int(timestamp * 1e9)) \
                    .tag_dict(tags) \
                    .field_dict(fields)
                
                self.influx_write_api.write(bucket=self.influx_bucket, record=point)
        
        except Exception as e:
            log.error(f"Failed to write to InfluxDB: {e}")
    
    def calculate_kpis(self) -> Dict[str, Any]:
        """Calculate current system KPIs."""
        now = time.time()
        
        # Basic KPIs
        kpis = {
            'timestamp': now,
            'uptime_seconds': now - self.start_time,
            'active_sensor_nodes': len(self.sensor_data),
            'total_mqtt_messages': len(self.metrics_buffer),
            'message_rate_per_minute': 0,
        }
        
        # Calculate message rate (last 5 minutes)
        recent_messages = [m for m in self.metrics_buffer if now - m['timestamp'] < 300]
        kpis['message_rate_per_minute'] = len(recent_messages) / 5.0
        
        # Network KPIs
        if 'mesh' in self.network_stats:
            mesh = self.network_stats['mesh']
            kpis.update({
                'mesh_packet_delivery_ratio': mesh.get('packet_delivery_ratio', 0.95),
                'mesh_avg_latency_ms': mesh.get('avg_latency_ms', 15.0),
                'mesh_throughput_mbps': mesh.get('avg_throughput_mbps', 25.0),
            })
        
        # Sensor KPIs
        sensor_count = 0
        total_battery = 0
        battery_count = 0
        
        for node_data in self.sensor_data.values():
            sensor_count += len(node_data)
            
            if 'battery' in node_data:
                battery = node_data['battery']
                if 'voltage' in battery:
                    total_battery += battery['voltage']
                    battery_count += 1
        
        kpis['total_sensors'] = sensor_count
        if battery_count > 0:
            kpis['avg_battery_voltage'] = total_battery / battery_count
            # Simple battery life estimation (hours)
            avg_voltage = kpis['avg_battery_voltage']
            # Linear approximation: 4.2V = 100%, 3.0V = 0%
            battery_percent = max(0, min(100, (avg_voltage - 3.0) / 1.2 * 100))
            # Estimate remaining hours based on percent and usage pattern
            kpis['estimated_battery_hours'] = battery_percent * 2.4  # Max ~10 days at full
        
        # Performance KPIs
        kpis.update({
            'pipeline_latency_ms': 50 + kpis.get('mesh_avg_latency_ms', 15),  # Simplified
            'system_health_score': min(100, 
                kpis.get('mesh_packet_delivery_ratio', 0.95) * 100 * 
                (1 if kpis['message_rate_per_minute'] > 0.1 else 0.5)
            )
        })
        
        return kpis
    
    def log_kpis(self):
        """Calculate and log current KPIs."""
        kpis = self.calculate_kpis()
        
        log.info("📊 Current KPIs:")
        log.info(f"   Uptime: {kpis['uptime_seconds']:.0f}s")
        log.info(f"   Active Nodes: {kpis['active_sensor_nodes']}")
        log.info(f"   Message Rate: {kpis['message_rate_per_minute']:.1f}/min")
        log.info(f"   Pipeline Latency: {kpis.get('pipeline_latency_ms', 'N/A')}ms")
        
        if 'mesh_packet_delivery_ratio' in kpis:
            log.info(f"   Mesh PDR: {kpis['mesh_packet_delivery_ratio']:.1%}")
        
        if 'avg_battery_voltage' in kpis:
            log.info(f"   Avg Battery: {kpis['avg_battery_voltage']:.2f}V")
        
        # Store in history
        self.kpi_history.append(kpis)
        
        # Keep only last 1000 entries
        if len(self.kpi_history) > 1000:
            self.kpi_history = self.kpi_history[-1000:]
        
        # Write KPIs to InfluxDB
        self.write_kpis_to_influx(kpis)
    
    def write_kpis_to_influx(self, kpis: Dict[str, Any]):
        """Write KPI metrics to InfluxDB."""
        try:
            if not self.influx_write_api:
                return
            
            point = Point("olympus_kpis") \
                .time(int(kpis['timestamp'] * 1e9))
            
            # Add all numeric KPIs as fields
            for key, value in kpis.items():
                if isinstance(value, (int, float)) and key != 'timestamp':
                    point = point.field(key, value)
            
            self.influx_write_api.write(bucket=self.influx_bucket, record=point)
            
        except Exception as e:
            log.error(f"Failed to write KPIs to InfluxDB: {e}")
    
    def cleanup_old_data(self):
        """Clean up old data from memory."""
        now = time.time()
        cutoff = now - 3600  # Keep last hour
        
        # Clean metrics buffer
        self.metrics_buffer = [m for m in self.metrics_buffer if m['timestamp'] > cutoff]
        
        # Clean sensor data
        for node_id in list(self.sensor_data.keys()):
            for sensor_type in list(self.sensor_data[node_id].keys()):
                if self.sensor_data[node_id][sensor_type]['timestamp'] < cutoff:
                    del self.sensor_data[node_id][sensor_type]
            
            # Remove empty nodes
            if not self.sensor_data[node_id]:
                del self.sensor_data[node_id]
        
        log.info("🧹 Cleaned up old metrics data")
    
    def schedule_tasks(self):
        """Schedule periodic tasks."""
        schedule.every(30).seconds.do(self.log_kpis)
        schedule.every(15).minutes.do(self.cleanup_old_data)
    
    def run(self):
        """Main run loop."""
        log.info("📊 Starting Olympus Metrics Collector...")
        
        if not self.init_connections():
            log.error("Failed to initialize connections")
            return False
        
        self.schedule_tasks()
        self.running = True
        
        try:
            while self.running:
                schedule.run_pending()
                time.sleep(1)
                
        except KeyboardInterrupt:
            log.info("Metrics collector interrupted")
        finally:
            self.cleanup()
        
        return True
    
    def cleanup(self):
        """Clean up resources."""
        log.info("Cleaning up metrics collector...")
        self.running = False
        
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        if self.influx_client:
            self.influx_client.close()

def main():
    """Main entry point."""
    collector = OlympusMetricsCollector()
    
    # Set up signal handlers
    def signal_handler(signum, frame):
        log.info(f"Received signal {signum}, shutting down...")
        collector.running = False
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Run collector
    success = collector.run()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main() 