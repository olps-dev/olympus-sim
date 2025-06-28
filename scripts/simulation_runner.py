#!/usr/bin/env python3
"""
Olympus Simulation Runner for Docker Environment
Orchestrates the complete simulation within container
"""

import os
import sys
import time
import signal
import logging
import threading
import subprocess
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=getattr(logging, os.getenv('LOG_LEVEL', 'INFO')),
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('simulation_runner')

class SimulationRunner:
    """Manages the complete simulation environment"""
    
    def __init__(self):
        self.processes = []
        self.running = False
        self.mqtt_broker = os.getenv('MQTT_BROKER', 'localhost')
        self.mqtt_port = int(os.getenv('MQTT_PORT', '1883'))
        self.simulation_mode = os.getenv('SIMULATION_MODE', 'headless')
        
        # Setup signal handlers
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        logger.info(f"Received signal {signum}, shutting down...")
        self.stop()
        sys.exit(0)
    
    def wait_for_mqtt(self, timeout=60):
        """Wait for MQTT broker to be available"""
        logger.info(f"Waiting for MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                import paho.mqtt.client as mqtt
                client = mqtt.Client()
                result = client.connect(self.mqtt_broker, self.mqtt_port, 10)
                if result == 0:
                    client.disconnect()
                    logger.info("MQTT broker is ready")
                    return True
            except Exception as e:
                logger.debug(f"MQTT connection attempt failed: {e}")
            
            time.sleep(2)
        
        logger.error("MQTT broker not available within timeout")
        return False
    
    def start_sensor_bridge(self):
        """Start the sensor bridge component"""
        logger.info("Starting sensor bridge...")
        
        try:
            # Use synthetic bridge for containerized environment
            bridge_script = self.create_container_bridge()
            
            proc = subprocess.Popen([
                sys.executable, bridge_script
            ], env=self.get_process_env())
            
            self.processes.append(('sensor_bridge', proc))
            logger.info("Sensor bridge started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start sensor bridge: {e}")
            return False
    
    def start_automation(self):
        """Start the live automation controller"""
        logger.info("Starting live automation controller...")
        
        try:
            proc = subprocess.Popen([
                sys.executable, '/app/automation/live_automation.py'
            ], env=self.get_process_env())
            
            self.processes.append(('automation', proc))
            logger.info("Live automation controller started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start automation: {e}")
            return False
    
    def create_container_bridge(self):
        """Create optimized sensor bridge for container environment"""
        bridge_path = Path("container_sensor_bridge.py")
        
        bridge_code = f'''#!/usr/bin/env python3
"""Container-optimized sensor bridge for Olympus simulation"""
import sys
import os
import time
import random
import json
import math
import signal

# Add paths
sys.path.append('/app/network')
sys.path.append('/app/metrics')

from network_realism import create_realistic_mqtt_client
from latency_battery_tracker import LatencyBatteryTracker

class ContainerSensorBridge:
    def __init__(self):
        self.running = True
        self.client = create_realistic_mqtt_client()
        self.client.on_connect = self.on_connect
        
        self.tracker = LatencyBatteryTracker()
        self.sensor_ids = ['mmwave1', 'mmwave2']
        
        for sensor_id in self.sensor_ids:
            self.tracker.init_sensor_battery(sensor_id)
        
        self.message_count = 0
        
        # Setup signal handlers
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def signal_handler(self, signum, frame):
        print(f"Bridge received signal {{signum}}, shutting down...")
        self.running = False
        
    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        print(f"Container bridge connected to MQTT: {{reason_code}}")
        
    def run(self):
        print("Starting container sensor bridge...")
        self.client.connect("{self.mqtt_broker}", {self.mqtt_port}, 60)
        self.client.loop_start()
        
        try:
            start_time = time.time()
            
            while self.running:
                for sensor_id in self.sensor_ids:
                    if not self.running:
                        break
                        
                    # Generate realistic sensor data
                    elapsed = time.time() - start_time
                    
                    # Create varied presence patterns
                    base_wave = math.sin(elapsed * 0.1) * 0.5 + 0.5
                    noise = random.uniform(-0.2, 0.2)
                    presence_probability = max(0, min(1, base_wave + noise))
                    
                    human_present = random.random() < presence_probability
                    
                    # Log pointcloud received
                    event_id = self.tracker.log_pointcloud_received(sensor_id)
                    
                    # Create message
                    message = {{
                        "timestamp": time.time(),
                        "sensor_id": sensor_id,
                        "human_present": human_present,
                        "num_points": random.randint(5, 50) if human_present else random.randint(0, 3),
                        "analysis": f"container simulation - {{elapsed:.1f}}s",
                        "event_id": event_id,
                        "battery_level_mah": self.tracker.get_battery_level(sensor_id)
                    }}
                    
                    # Publish
                    try:
                        topic = f"sensor/{{sensor_id}}/presence"
                        payload = json.dumps(message)
                        result = self.client.publish(topic, payload, qos=1)
                        
                        if result.rc == 0:
                            self.tracker.log_mqtt_published(sensor_id, event_id)
                        
                        self.message_count += 1
                        
                        if self.message_count % 20 == 0:
                            print(f"Bridge: {{self.message_count}} messages sent")
                        
                    except Exception as e:
                        print(f"ERROR: Publish failed for {{sensor_id}}: {{e}}")
                
                time.sleep(1.0)  # 1Hz rate for container stability
            
        except Exception as e:
            print(f"Bridge error: {{e}}")
        finally:
            self.client.disconnect()
            print("Container sensor bridge stopped")

if __name__ == "__main__":
    bridge = ContainerSensorBridge()
    bridge.run()
'''
        
        with open(bridge_path, 'w') as f:
            f.write(bridge_code)
        
        return str(bridge_path)
    
    def get_process_env(self):
        """Get environment variables for subprocesses"""
        env = os.environ.copy()
        env.update({
            'PYTHONPATH': '/app',
            'MQTT_BROKER': self.mqtt_broker,
            'MQTT_PORT': str(self.mqtt_port)
        })
        return env
    
    def monitor_processes(self):
        """Monitor running processes"""
        while self.running:
            for name, proc in self.processes:
                if proc.poll() is not None:
                    logger.warning(f"Process {name} exited with code {proc.returncode}")
                    # In production, might want to restart processes
            
            time.sleep(10)
    
    def start(self):
        """Start the complete simulation"""
        logger.info("Starting Olympus simulation in container mode")
        
        # Wait for dependencies
        if not self.wait_for_mqtt():
            logger.error("Failed to connect to MQTT broker")
            return False
        
        self.running = True
        
        # Start components
        if not self.start_sensor_bridge():
            return False
        
        time.sleep(2)  # Allow sensor bridge to initialize
        
        if not self.start_automation():
            return False
        
        # Start monitoring
        monitor_thread = threading.Thread(target=self.monitor_processes, daemon=True)
        monitor_thread.start()
        
        logger.info("Simulation started successfully")
        
        # Keep running
        try:
            while self.running:
                time.sleep(5)
                
                # Health check
                active_processes = sum(1 for _, proc in self.processes if proc.poll() is None)
                logger.debug(f"Active processes: {active_processes}/{len(self.processes)}")
                
        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
        
        return True
    
    def stop(self):
        """Stop all simulation components"""
        logger.info("Stopping simulation...")
        self.running = False
        
        for name, proc in self.processes:
            try:
                if proc.poll() is None:
                    logger.info(f"Terminating {name}")
                    proc.terminate()
                    
                    # Wait for graceful shutdown
                    try:
                        proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        logger.warning(f"Force killing {name}")
                        proc.kill()
                        
            except Exception as e:
                logger.error(f"Error stopping {name}: {e}")
        
        # Clean up temporary files
        temp_files = ['container_sensor_bridge.py']
        for file_path in temp_files:
            try:
                if os.path.exists(file_path):
                    os.remove(file_path)
            except Exception as e:
                logger.error(f"Error cleaning up {file_path}: {e}")
        
        logger.info("Simulation stopped")

def main():
    """Main entry point"""
    runner = SimulationRunner()
    
    try:
        success = runner.start()
        if not success:
            logger.error("Failed to start simulation")
            sys.exit(1)
    except Exception as e:
        logger.error(f"Simulation error: {e}")
        sys.exit(1)
    finally:
        runner.stop()

if __name__ == "__main__":
    main()