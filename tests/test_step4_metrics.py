#!/usr/bin/env python3
"""
Test Script for Step 4: Latency & Battery Instrumentation
Runs a one-minute headless simulation and generates metrics plots

Success test: One-minute headless run auto-spits latency_hist.png and battery_curve.png
"""

import subprocess
import time
import sys
import os
import signal
import logging
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('step4_test')

class Step4MetricsTest:
    def __init__(self):
        self.processes = []
        self.test_duration = 60  # 1 minute
        self.metrics_dir = Path("metrics")
        self.metrics_dir.mkdir(exist_ok=True)
        
    def cleanup_processes(self):
        """Clean up any running processes"""
        for proc in self.processes:
            try:
                if proc.poll() is None:  # Process is still running
                    proc.terminate()
                    proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
            except Exception as e:
                logger.error(f"Error cleaning up process: {e}")
        self.processes.clear()
    
    def signal_handler(self, signum, frame):
        """Handle interrupt signals"""
        logger.info("\n🛑 Received interrupt signal, cleaning up...")
        self.cleanup_processes()
        sys.exit(0)
    
    def check_dependencies(self):
        """Check if required dependencies are available"""
        logger.info("📋 Checking dependencies...")
        
        try:
            import matplotlib.pyplot as plt
            import pandas as pd
            import numpy as np
            logger.info("✅ Python dependencies available")
        except ImportError as e:
            logger.error(f"❌ Missing Python dependency: {e}")
            logger.info("Install with: pip install matplotlib pandas numpy")
            return False
        
        # Check if mosquitto is available
        try:
            result = subprocess.run(['which', 'mosquitto'], capture_output=True, text=True)
            if result.returncode == 0:
                logger.info("✅ Mosquitto MQTT broker available")
            else:
                logger.warning("⚠️  Mosquitto not found - will try to start anyway")
        except Exception as e:
            logger.warning(f"⚠️  Could not check mosquitto: {e}")
        
        return True
    
    def start_mqtt_broker(self):
        """Start MQTT broker"""
        try:
            logger.info("🔌 Starting MQTT broker...")
            proc = subprocess.Popen(['mosquitto', '-v'], 
                                  stdout=subprocess.PIPE, 
                                  stderr=subprocess.PIPE)
            self.processes.append(proc)
            time.sleep(2)  # Give broker time to start
            
            if proc.poll() is None:
                logger.info("✅ MQTT broker started")
                return True
            else:
                logger.error("❌ MQTT broker failed to start")
                return False
        except FileNotFoundError:
            logger.error("❌ Mosquitto not found. Install with: sudo apt install mosquitto")
            return False
        except Exception as e:
            logger.error(f"❌ Error starting MQTT broker: {e}")
            return False
    
    def start_sensor_bridge(self):
        """Start the mmWave MQTT bridge (simulated data)"""
        try:
            logger.info("📡 Starting sensor bridge...")
            
            # Create a simple test version that generates synthetic data
            test_bridge_script = self.create_test_bridge()
            
            proc = subprocess.Popen([sys.executable, test_bridge_script],
                                  stdout=subprocess.PIPE, 
                                  stderr=subprocess.PIPE)
            self.processes.append(proc)
            time.sleep(3)  # Give bridge time to start
            
            if proc.poll() is None:
                logger.info("✅ Sensor bridge started")
                return True
            else:
                stdout, stderr = proc.communicate()
                logger.error(f"❌ Sensor bridge failed: {stderr.decode()}")
                return False
        except Exception as e:
            logger.error(f"❌ Error starting sensor bridge: {e}")
            return False
    
    def create_test_bridge(self):
        """Create a test bridge that generates synthetic sensor data"""
        test_bridge_path = Path("test_synthetic_bridge.py")
        
        test_bridge_code = '''#!/usr/bin/env python3
"""Synthetic mmWave bridge for testing Step 4 metrics"""
import paho.mqtt.client as mqtt
import json
import time
import random
import sys
import os

# Add metrics to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'metrics'))
from latency_battery_tracker import LatencyBatteryTracker

class SyntheticBridge:
    def __init__(self):
        try:
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        except AttributeError:
            self.client = mqtt.Client()
        
        self.client.on_connect = self.on_connect
        self.tracker = LatencyBatteryTracker()
        self.sensor_ids = ['mmwave1', 'mmwave2']
        
        for sensor_id in self.sensor_ids:
            self.tracker.init_sensor_battery(sensor_id)
    
    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        print(f"Connected to MQTT broker: {reason_code}")
    
    def run(self):
        print("🧪 Starting synthetic sensor bridge for testing...")
        self.client.connect("localhost", 1883, 60)
        self.client.loop_start()
        
        try:
            for i in range(120):  # Run for 2 minutes with 1Hz
                for sensor_id in self.sensor_ids:
                    # Simulate presence detection
                    human_present = random.random() < 0.3  # 30% chance
                    
                    # Log pointcloud received
                    event_id = self.tracker.log_pointcloud_received(sensor_id)
                    
                    # Create presence message
                    message = {
                        "timestamp": time.time(),
                        "sensor_id": sensor_id,
                        "human_present": human_present,
                        "num_points": random.randint(0, 50) if human_present else 0,
                        "analysis": f"synthetic test data",
                        "event_id": event_id,
                        "battery_level_mah": self.tracker.get_battery_level(sensor_id)
                    }
                    
                    # Publish
                    topic = f"sensor/{sensor_id}/presence"
                    payload = json.dumps(message)
                    self.client.publish(topic, payload, qos=1)
                    
                    # Log MQTT published
                    self.tracker.log_mqtt_published(sensor_id, event_id)
                    
                    print(f"[{sensor_id}] {'DETECTED' if human_present else 'CLEAR'} "
                          f"(Battery: {message['battery_level_mah']:.1f}mAh)")
                
                time.sleep(0.5)  # 2Hz publishing rate
                
        except KeyboardInterrupt:
            pass
        finally:
            self.client.disconnect()
            print("🧪 Synthetic bridge stopped")

if __name__ == "__main__":
    bridge = SyntheticBridge()
    bridge.run()
'''
        
        with open(test_bridge_path, 'w') as f:
            f.write(test_bridge_code)
        
        return str(test_bridge_path)
    
    def start_automation(self):
        """Start the automation controller"""
        try:
            logger.info("🏠 Starting automation controller...")
            
            proc = subprocess.Popen([sys.executable, 'automation/automation_demo.py'],
                                  stdout=subprocess.PIPE, 
                                  stderr=subprocess.PIPE)
            self.processes.append(proc)
            time.sleep(3)  # Give automation time to start
            
            if proc.poll() is None:
                logger.info("✅ Automation controller started")
                return True
            else:
                stdout, stderr = proc.communicate()
                logger.error(f"❌ Automation controller failed: {stderr.decode()}")
                return False
        except Exception as e:
            logger.error(f"❌ Error starting automation: {e}")
            return False
    
    def monitor_test(self):
        """Monitor the test and check for expected outputs"""
        logger.info(f"⏱️  Running test for {self.test_duration} seconds...")
        logger.info("📊 Monitoring metrics generation...")
        
        start_time = time.time()
        
        try:
            while time.time() - start_time < self.test_duration:
                # Check if processes are still running
                for i, proc in enumerate(self.processes):
                    if proc.poll() is not None:
                        logger.warning(f"⚠️  Process {i} has stopped")
                
                # Check for metrics files
                latency_csv = self.metrics_dir / "latency_metrics.csv"
                battery_csv = self.metrics_dir / "battery_metrics.csv"
                
                if latency_csv.exists() and battery_csv.exists():
                    logger.info("📈 Metrics files being generated...")
                
                time.sleep(5)  # Check every 5 seconds
                
        except KeyboardInterrupt:
            logger.info("Test interrupted by user")
    
    def check_results(self):
        """Check if the expected plots were generated"""
        logger.info("🔍 Checking test results...")
        
        expected_files = [
            self.metrics_dir / "latency_hist.png",
            self.metrics_dir / "battery_curve.png",
            self.metrics_dir / "latency_metrics.csv",
            self.metrics_dir / "battery_metrics.csv"
        ]
        
        success = True
        
        for file_path in expected_files:
            if file_path.exists():
                logger.info(f"✅ {file_path.name} generated successfully")
                if file_path.suffix == '.csv':
                    # Check if CSV has data
                    try:
                        with open(file_path, 'r') as f:
                            lines = f.readlines()
                            if len(lines) > 1:  # Header + at least one data row
                                logger.info(f"   📊 {len(lines)-1} data rows")
                            else:
                                logger.warning(f"   ⚠️  {file_path.name} is empty")
                    except Exception as e:
                        logger.error(f"   ❌ Error reading {file_path.name}: {e}")
            else:
                logger.error(f"❌ {file_path.name} not found")
                success = False
        
        return success
    
    def run_test(self):
        """Run the complete Step 4 test"""
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        logger.info("🚀 Starting Step 4 Metrics Test")
        logger.info("=" * 60)
        logger.info("Testing: Latency & battery instrumentation")
        logger.info("Expected: latency_hist.png and battery_curve.png generation")
        logger.info("=" * 60)
        
        try:
            # Check dependencies
            if not self.check_dependencies():
                return False
            
            # Start components
            if not self.start_mqtt_broker():
                return False
            
            if not self.start_sensor_bridge():
                return False
            
            if not self.start_automation():
                return False
            
            # Monitor test
            self.monitor_test()
            
            # Generate final plots by sending interrupt to automation
            logger.info("📊 Generating final metrics plots...")
            for proc in self.processes:
                if 'automation_demo.py' in ' '.join(proc.args):
                    proc.send_signal(signal.SIGINT)
                    proc.wait(timeout=10)
            
            # Wait a bit for file I/O
            time.sleep(2)
            
            # Check results
            success = self.check_results()
            
            if success:
                logger.info("🎉 Step 4 test PASSED!")
                logger.info("✅ Latency and battery instrumentation working correctly")
                logger.info(f"📁 Metrics saved to: {self.metrics_dir.absolute()}")
            else:
                logger.error("❌ Step 4 test FAILED!")
                logger.error("Some expected files were not generated")
            
            return success
            
        except Exception as e:
            logger.error(f"❌ Test error: {e}")
            return False
        finally:
            self.cleanup_processes()
            # Clean up test files
            test_bridge = Path("test_synthetic_bridge.py")
            if test_bridge.exists():
                test_bridge.unlink()

def main():
    test = Step4MetricsTest()
    success = test.run_test()
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()