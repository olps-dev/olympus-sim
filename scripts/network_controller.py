#!/usr/bin/env python3
"""
Olympus Network Controller for Docker Environment
Manages network conditions and publishes statistics
"""

import os
import sys
import time
import signal
import logging
import threading
import json
import random
from datetime import datetime

# Add paths
sys.path.append('/app/network')

import paho.mqtt.client as mqtt
from config import NetworkConfigManager

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('network_controller')

class NetworkController:
    """Network conditions controller and statistics publisher"""
    
    def __init__(self):
        self.running = False
        self.mqtt_client = None
        self.config_manager = NetworkConfigManager('/app/config/network_config.json')
        
        # Configuration
        self.mqtt_broker = os.getenv('MQTT_BROKER', 'mosquitto')
        self.mqtt_port = int(os.getenv('MQTT_PORT', '1883'))
        self.default_condition = os.getenv('DEFAULT_CONDITION', 'good')
        self.enable_dynamic = os.getenv('ENABLE_DYNAMIC_CONDITIONS', 'false').lower() == 'true'
        
        # Statistics
        self.stats = {
            'start_time': time.time(),
            'condition_changes': 0,
            'uptime_seconds': 0
        }
        
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
            
            # Subscribe to network control topics
            topics = [
                "network/control/condition",
                "network/control/scenario"
            ]
            
            for topic in topics:
                client.subscribe(topic)
                logger.info(f"Subscribed to {topic}")
        else:
            logger.error(f"Failed to connect to MQTT: {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Process MQTT control messages"""
        try:
            topic = msg.topic
            payload = msg.payload.decode()
            
            if "condition" in topic:
                self.handle_condition_change(payload)
            elif "scenario" in topic:
                self.handle_scenario_change(payload)
                
        except Exception as e:
            logger.error(f"Error processing control message: {e}")
    
    def handle_condition_change(self, condition):
        """Handle network condition change request"""
        try:
            # Update configuration
            old_config = self.config_manager.get_statistics_summary()
            self.config_manager.set_scenario(condition)
            
            self.stats['condition_changes'] += 1
            logger.info(f"Network condition changed to: {condition}")
            
            # Publish update
            self.publish_network_status()
            
        except Exception as e:
            logger.error(f"Error changing network condition: {e}")
    
    def handle_scenario_change(self, scenario):
        """Handle network scenario change request"""
        try:
            self.config_manager.set_scenario(scenario)
            self.stats['condition_changes'] += 1
            logger.info(f"Network scenario changed to: {scenario}")
            
            self.publish_network_status()
            
        except Exception as e:
            logger.error(f"Error changing network scenario: {e}")
    
    def publish_network_status(self):
        """Publish current network status and statistics"""
        try:
            config_summary = self.config_manager.get_statistics_summary()
            
            status = {
                'timestamp': time.time(),
                'controller_stats': {
                    **self.stats,
                    'uptime_seconds': time.time() - self.stats['start_time']
                },
                'network_config': config_summary,
                'dynamic_conditions_enabled': self.enable_dynamic
            }
            
            self.mqtt_client.publish("network/stats", json.dumps(status), qos=1)
            
        except Exception as e:
            logger.error(f"Error publishing network status: {e}")
    
    def apply_dynamic_conditions(self):
        """Apply dynamic network conditions based on time or other factors"""
        if not self.enable_dynamic:
            return
        
        try:
            # Simple time-based condition changes (for demonstration)
            current_hour = datetime.now().hour
            
            # Simulate different network conditions throughout the day
            if 9 <= current_hour <= 17:  # Business hours - more traffic
                target_condition = 'fair'
            elif 22 <= current_hour or current_hour <= 6:  # Night - better conditions
                target_condition = 'excellent'
            else:  # Other times
                target_condition = 'good'
            
            # Add some randomness
            if random.random() < 0.1:  # 10% chance of condition change
                conditions = ['excellent', 'good', 'fair', 'poor']
                target_condition = random.choice(conditions)
            
            current_config = self.config_manager.get_statistics_summary()
            if current_config.get('default_condition') != target_condition:
                logger.info(f"Dynamic condition change: {target_condition}")
                self.config_manager.set_scenario(target_condition)
                self.stats['condition_changes'] += 1
                self.publish_network_status()
                
        except Exception as e:
            logger.error(f"Error applying dynamic conditions: {e}")
    
    def periodic_tasks(self):
        """Run periodic network controller tasks"""
        while self.running:
            try:
                # Publish regular status updates
                self.publish_network_status()
                
                # Apply dynamic conditions if enabled
                self.apply_dynamic_conditions()
                
                # Log current status
                uptime = time.time() - self.stats['start_time']
                logger.debug(f"Network controller uptime: {uptime:.0f}s, "
                           f"condition changes: {self.stats['condition_changes']}")
                
            except Exception as e:
                logger.error(f"Error in periodic tasks: {e}")
            
            time.sleep(30)  # Run every 30 seconds
    
    def start(self):
        """Start network controller"""
        logger.info("Starting Olympus network controller")
        
        # Initialize default configuration
        try:
            self.config_manager.set_scenario(self.default_condition)
            logger.info(f"Initialized with condition: {self.default_condition}")
        except Exception as e:
            logger.error(f"Failed to initialize configuration: {e}")
            return False
        
        if not self.connect_mqtt():
            logger.error("Failed to connect to MQTT")
            return False
        
        self.running = True
        
        # Start periodic tasks
        tasks_thread = threading.Thread(target=self.periodic_tasks, daemon=True)
        tasks_thread.start()
        
        logger.info(f"Network controller started (dynamic conditions: {self.enable_dynamic})")
        
        # Initial status publish
        self.publish_network_status()
        
        # Keep running
        try:
            while self.running:
                time.sleep(10)
                
        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
        
        return True
    
    def stop(self):
        """Stop network controller"""
        logger.info("Stopping network controller...")
        self.running = False
        
        if self.mqtt_client:
            # Publish final status
            try:
                self.publish_network_status()
            except:
                pass
            
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        logger.info("Network controller stopped")

def main():
    """Main entry point"""
    controller = NetworkController()
    
    try:
        success = controller.start()
        if not success:
            logger.error("Failed to start network controller")
            sys.exit(1)
    except Exception as e:
        logger.error(f"Network controller error: {e}")
        sys.exit(1)
    finally:
        controller.stop()

if __name__ == "__main__":
    main()