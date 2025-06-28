#!/usr/bin/env python3
"""
Olympus Metrics Collector for Docker Environment
Dedicated service for metrics collection and analysis
"""

import os
import sys
import time
import signal
import logging
import threading
import json
from pathlib import Path
from datetime import datetime, timedelta

# Add paths
sys.path.append('/app/metrics')
sys.path.append('/app/network')

import paho.mqtt.client as mqtt
from latency_battery_tracker import LatencyBatteryTracker

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('metrics_collector')

class MetricsCollector:
    """Dedicated metrics collection service"""
    
    def __init__(self):
        self.running = False
        self.mqtt_client = None
        self.tracker = LatencyBatteryTracker('/app/metrics')
        
        # Configuration
        self.mqtt_broker = os.getenv('MQTT_BROKER', 'mosquitto')
        self.mqtt_port = int(os.getenv('MQTT_PORT', '1883'))
        self.collection_interval = int(os.getenv('COLLECTION_INTERVAL', '5'))
        self.retention_days = int(os.getenv('METRICS_RETENTION_DAYS', '7'))
        
        # Data storage
        self.data_dir = Path('/app/data')
        self.data_dir.mkdir(exist_ok=True)
        
        # Health tracking
        self.health_file = self.data_dir / 'health.txt'
        
        # Setup signal handlers
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        logger.info(f"Received signal {signum}, shutting down...")
        self.stop()
        sys.exit(0)
    
    def connect_mqtt(self):
        """Connect to MQTT broker"""
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            logger.info(f"Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to MQTT: {e}")
            return False
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            logger.info("Connected to MQTT broker")
            
            # Subscribe to metrics-relevant topics
            topics = [
                "sensor/+/presence",
                "automation/events",
                "actuators/+/+",
                "network/stats"
            ]
            
            for topic in topics:
                client.subscribe(topic)
                logger.info(f"Subscribed to {topic}")
        else:
            logger.error(f"Failed to connect to MQTT: {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        logger.warning(f"Disconnected from MQTT broker: {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Process MQTT messages for metrics"""
        try:
            topic = msg.topic
            payload = msg.payload.decode()
            
            # Process different message types
            if "presence" in topic:
                self.process_presence_metrics(topic, payload)
            elif "automation/events" in topic:
                self.process_automation_metrics(topic, payload)
            elif "network/stats" in topic:
                self.process_network_metrics(topic, payload)
                
        except Exception as e:
            logger.error(f"Error processing MQTT message: {e}")
    
    def process_presence_metrics(self, topic, payload):
        """Process sensor presence messages for metrics"""
        try:
            data = json.loads(payload)
            sensor_id = data.get('sensor_id')
            event_id = data.get('event_id')
            
            if sensor_id and event_id:
                # Initialize sensor battery tracking
                self.tracker.init_sensor_battery(sensor_id)
                
                # Log the MQTT publish event for latency tracking
                self.tracker.log_mqtt_published(sensor_id, event_id)
                
        except (json.JSONDecodeError, KeyError) as e:
            logger.debug(f"Could not parse presence message: {e}")
    
    def process_automation_metrics(self, topic, payload):
        """Process automation events for metrics"""
        try:
            data = json.loads(payload)
            event_ids = data.get('event_ids', [])
            
            # Log lamp toggle events for end-to-end latency
            for event_id in event_ids:
                if 'triggered_by' in data:
                    for sensor_id in data['triggered_by']:
                        self.tracker.log_lamp_toggled(event_id, sensor_id)
                        
        except (json.JSONDecodeError, KeyError) as e:
            logger.debug(f"Could not parse automation event: {e}")
    
    def process_network_metrics(self, topic, payload):
        """Process network statistics"""
        try:
            data = json.loads(payload)
            
            # Store network metrics
            metrics_file = self.data_dir / 'network_metrics.jsonl'
            with open(metrics_file, 'a') as f:
                json.dump({
                    'timestamp': time.time(),
                    'data': data
                }, f)
                f.write('\n')
                
        except (json.JSONDecodeError, IOError) as e:
            logger.debug(f"Could not process network metrics: {e}")
    
    def generate_periodic_reports(self):
        """Generate periodic metrics reports"""
        while self.running:
            try:
                # Generate plots
                plots = self.tracker.generate_plots()
                if plots:
                    logger.info(f"Generated metrics plots: {list(plots.keys())}")
                
                # Update health file
                self.update_health_status()
                
                # Clean old data
                self.cleanup_old_data()
                
            except Exception as e:
                logger.error(f"Error generating reports: {e}")
            
            # Wait for next interval
            time.sleep(self.collection_interval * 60)  # Convert to minutes
    
    def update_health_status(self):
        """Update health status file"""
        try:
            health_data = {
                'timestamp': time.time(),
                'status': 'healthy',
                'metrics_summary': self.tracker.get_metrics_summary()
            }
            
            with open(self.health_file, 'w') as f:
                json.dump(health_data, f, indent=2)
                
        except Exception as e:
            logger.error(f"Error updating health status: {e}")
    
    def cleanup_old_data(self):
        """Clean up old metrics data"""
        try:
            cutoff_time = datetime.now() - timedelta(days=self.retention_days)
            
            # Clean up old CSV files (simplified - would need more sophisticated cleanup)
            for file_path in self.data_dir.glob('*.csv'):
                if file_path.stat().st_mtime < cutoff_time.timestamp():
                    file_path.unlink()
                    logger.info(f"Cleaned up old file: {file_path}")
                    
        except Exception as e:
            logger.error(f"Error cleaning up old data: {e}")
    
    def start(self):
        """Start metrics collection"""
        logger.info("Starting Olympus metrics collector")
        
        if not self.connect_mqtt():
            logger.error("Failed to connect to MQTT")
            return False
        
        self.running = True
        
        # Start periodic report generation
        report_thread = threading.Thread(target=self.generate_periodic_reports, daemon=True)
        report_thread.start()
        
        logger.info("Metrics collector started")
        
        # Keep running
        try:
            while self.running:
                time.sleep(10)
                
                # Periodic health update
                self.update_health_status()
                
        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
        
        return True
    
    def stop(self):
        """Stop metrics collection"""
        logger.info("Stopping metrics collector...")
        self.running = False
        
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        logger.info("Metrics collector stopped")

def main():
    """Main entry point"""
    collector = MetricsCollector()
    
    try:
        success = collector.start()
        if not success:
            logger.error("Failed to start metrics collector")
            sys.exit(1)
    except Exception as e:
        logger.error(f"Metrics collector error: {e}")
        sys.exit(1)
    finally:
        collector.stop()

if __name__ == "__main__":
    main()