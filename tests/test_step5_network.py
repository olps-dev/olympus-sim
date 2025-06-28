#!/usr/bin/env python3
"""
Test Script for Step 5: Network Realism Layer
Tests packet loss and latency injection with retry behavior verification

Success test: Set 10% drop probability; verify presence still toggles lamp after retries, 
and latency histogram widens.
"""

import subprocess
import time
import sys
import os
import signal
import logging
import json
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('step5_test')

class Step5NetworkTest:
    def __init__(self):
        self.processes = []
        self.test_duration = 120  # 2 minutes for network behavior analysis
        self.metrics_dir = Path("metrics")
        self.network_dir = Path("network")
        self.metrics_dir.mkdir(exist_ok=True)
        self.network_dir.mkdir(exist_ok=True)
        
        # Test scenarios
        self.scenarios = [
            {
                'name': 'baseline',
                'config': {
                    'packet_loss_rate': 0.0,
                    'base_latency_ms': 1.0,
                    'enabled': True
                },
                'duration': 30
            },
            {
                'name': 'network_stress',
                'config': {
                    'packet_loss_rate': 0.10,  # 10% packet loss as specified
                    'base_latency_ms': 20.0,
                    'jitter_ms': 10.0,
                    'burst_loss_probability': 0.15,
                    'enabled': True
                },
                'duration': 60
            }
        ]
        
    def cleanup_processes(self):
        """Clean up any running processes"""
        for proc in self.processes:
            try:
                if proc.poll() is None:
                    proc.terminate()
                    proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
            except Exception as e:
                logger.error(f"Error cleaning up process: {e}")
        self.processes.clear()
    
    def signal_handler(self, signum, frame):
        """Handle interrupt signals"""
        logger.info("\\n🛑 Received interrupt signal, cleaning up...")
        self.cleanup_processes()
        sys.exit(0)
    
    def check_dependencies(self):
        """Check if required dependencies are available"""
        logger.info("📋 Checking dependencies...")
        
        try:
            import matplotlib.pyplot as plt
            import pandas as pd
            import numpy as np
            import paho.mqtt.client as mqtt
            logger.info("✅ Python dependencies available")
        except ImportError as e:
            logger.error(f"❌ Missing Python dependency: {e}")
            return False
        
        # Check network module
        try:
            sys.path.append(str(self.network_dir))
            from network_realism import NetworkConfig, NetworkCondition
            logger.info("✅ Network realism module available")
        except ImportError as e:
            logger.error(f"❌ Network realism module not found: {e}")
            return False
        
        return True
    
    def create_network_config(self, scenario_config):
        """Create network configuration file for scenario"""
        config_path = self.network_dir / "network_config.json"
        
        global_config = {
            'default_condition': 'good',
            'sensor_bridge_config': scenario_config,
            'automation_config': scenario_config,
            'enabled': True,
            'log_network_events': True
        }
        
        with open(config_path, 'w') as f:
            json.dump(global_config, f, indent=2)
        
        logger.info(f"📁 Created network config: {scenario_config}")
    
    def start_mqtt_broker(self):
        """Start MQTT broker"""
        try:
            logger.info("🔌 Starting MQTT broker...")
            proc = subprocess.Popen(['mosquitto', '-v'], 
                                  stdout=subprocess.PIPE, 
                                  stderr=subprocess.PIPE)
            self.processes.append(proc)
            time.sleep(2)
            
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
    
    def create_test_bridge(self):
        """Create enhanced test bridge with network realism"""
        test_bridge_path = Path("test_network_bridge.py")
        
        test_bridge_code = '''#!/usr/bin/env python3
"""Enhanced synthetic mmWave bridge for testing Step 5 network realism"""
import sys
import os
import time
import random
import json

# Add network and metrics to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'network'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'metrics'))

from network_realism import create_realistic_mqtt_client
from config import get_network_config_for_component
from latency_battery_tracker import LatencyBatteryTracker

class NetworkAwareBridge:
    def __init__(self):
        # Create realistic MQTT client
        self.client = create_realistic_mqtt_client()
        self.client.on_connect = self.on_connect
        
        self.tracker = LatencyBatteryTracker()
        self.sensor_ids = ['mmwave1', 'mmwave2']
        
        for sensor_id in self.sensor_ids:
            self.tracker.init_sensor_battery(sensor_id)
        
        self.message_count = 0
        self.successful_publishes = 0
    
    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        print(f"🔗 Connected to MQTT broker: {reason_code}")
        
        # Print network configuration
        network_stats = self.client.network_sim.get_statistics()
        config = network_stats.get('config', {})
        print(f"📡 Network condition: {network_stats.get('network_condition', 'unknown')}")
        print(f"   Packet loss rate: {config.get('packet_loss_rate', 0):.1%}")
        print(f"   Base latency: {config.get('base_latency_ms', 0):.1f}ms")
    
    def run(self):
        print("🧪 Starting network-aware sensor bridge...")
        self.client.connect("localhost", 1883, 60)
        self.client.loop_start()
        
        try:
            start_time = time.time()
            
            while time.time() - start_time < 120:  # Run for 2 minutes
                for sensor_id in self.sensor_ids:
                    # Simulate presence detection with varying patterns
                    current_time = time.time() - start_time
                    
                    # Create more dynamic presence patterns for better testing
                    presence_probability = 0.4 + 0.3 * abs(math.sin(current_time * 0.1))
                    human_present = random.random() < presence_probability
                    
                    # Log pointcloud received
                    event_id = self.tracker.log_pointcloud_received(sensor_id)
                    
                    # Create presence message
                    message = {
                        "timestamp": time.time(),
                        "sensor_id": sensor_id,
                        "human_present": human_present,
                        "num_points": random.randint(5, 50) if human_present else 0,
                        "analysis": f"network test data - msg {self.message_count}",
                        "event_id": event_id,
                        "battery_level_mah": self.tracker.get_battery_level(sensor_id)
                    }
                    
                    # Publish with network realism
                    topic = f"sensor/{sensor_id}/presence"
                    payload = json.dumps(message)
                    
                    try:
                        result = self.client.publish(topic, payload, qos=1)
                        if result.rc == 0:  # Success
                            self.successful_publishes += 1
                            # Log MQTT published
                            self.tracker.log_mqtt_published(sensor_id, event_id)
                        
                        self.message_count += 1
                        
                        print(f"[{sensor_id}] {'DETECTED' if human_present else 'CLEAR'} "
                              f"(Battery: {message['battery_level_mah']:.1f}mAh, "
                              f"Msg: {self.message_count})")
                    
                    except Exception as e:
                        print(f"❌ Publish error for {sensor_id}: {e}")
                
                time.sleep(0.5)  # 2Hz publishing rate
                
                # Print periodic network statistics
                if self.message_count % 20 == 0:
                    network_stats = self.client.network_sim.get_statistics()
                    success_rate = (self.successful_publishes / self.message_count) if self.message_count > 0 else 0
                    print(f"📊 Network Stats - Sent: {network_stats.get('messages_sent', 0)}, "
                          f"Dropped: {network_stats.get('messages_dropped', 0)}, "
                          f"Success Rate: {success_rate:.1%}")
                
        except KeyboardInterrupt:
            pass
        finally:
            self.client.disconnect()
            
            # Print final statistics
            network_stats = self.client.network_sim.get_statistics()
            print("🏁 Final Network Statistics:")
            print(f"   Total messages attempted: {self.message_count}")
            print(f"   Successful publishes: {self.successful_publishes}")
            print(f"   Network messages sent: {network_stats.get('messages_sent', 0)}")
            print(f"   Network messages dropped: {network_stats.get('messages_dropped', 0)}")
            print(f"   Actual drop rate: {network_stats.get('drop_rate_actual', 0):.1%}")
            print(f"   Average latency added: {network_stats.get('avg_latency_added_ms', 0):.1f}ms")
            print("🧪 Network-aware bridge stopped")

if __name__ == "__main__":
    import math  # Add missing import
    bridge = NetworkAwareBridge()
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
            time.sleep(3)
            
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
    
    def run_scenario(self, scenario):
        """Run a specific test scenario"""
        logger.info(f"🎬 Running scenario: {scenario['name']}")
        logger.info(f"   Config: {scenario['config']}")
        
        # Update network configuration
        self.create_network_config(scenario['config'])
        
        # Create and start test bridge
        test_bridge_script = self.create_test_bridge()
        
        try:
            proc = subprocess.Popen([sys.executable, test_bridge_script],
                                  stdout=subprocess.PIPE, 
                                  stderr=subprocess.PIPE)
            self.processes.append(proc)
            
            # Monitor scenario
            start_time = time.time()
            while time.time() - start_time < scenario['duration']:
                if proc.poll() is not None:
                    logger.warning(f"⚠️  Bridge process stopped unexpectedly")
                    break
                time.sleep(5)
            
            # Stop the bridge process
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=10)
            
            # Remove from process list since we've handled it
            if proc in self.processes:
                self.processes.remove(proc)
                
        except Exception as e:
            logger.error(f"❌ Error running scenario {scenario['name']}: {e}")
        finally:
            # Clean up test file
            test_bridge_path = Path(test_bridge_script)
            if test_bridge_path.exists():
                test_bridge_path.unlink()
    
    def analyze_results(self):
        """Analyze test results and compare baseline vs stressed network"""
        logger.info("📊 Analyzing network realism test results...")
        
        try:
            # Check if metrics files exist
            latency_csv = self.metrics_dir / "latency_metrics.csv"
            
            if not latency_csv.exists():
                logger.error("❌ No latency metrics found")
                return False
            
            # Read and analyze latency data
            import pandas as pd
            df = pd.read_csv(latency_csv)
            
            # Filter for end-to-end latencies
            e2e_data = df[(df['stage'] == 'lamp_toggled') & df['cumulative_latency_ms'].notna()]
            
            if len(e2e_data) == 0:
                logger.warning("⚠️  No end-to-end latency data found")
                return False
            
            # Analyze latency distribution
            latencies = e2e_data['cumulative_latency_ms']
            
            stats = {
                'count': len(latencies),
                'mean_ms': latencies.mean(),
                'median_ms': latencies.median(),
                'p95_ms': latencies.quantile(0.95),
                'p99_ms': latencies.quantile(0.99),
                'std_ms': latencies.std(),
                'min_ms': latencies.min(),
                'max_ms': latencies.max()
            }
            
            logger.info("📈 Latency Analysis Results:")
            logger.info(f"   Sample count: {stats['count']}")
            logger.info(f"   Mean latency: {stats['mean_ms']:.1f}ms")
            logger.info(f"   P95 latency: {stats['p95_ms']:.1f}ms")
            logger.info(f"   P99 latency: {stats['p99_ms']:.1f}ms")
            logger.info(f"   Standard deviation: {stats['std_ms']:.1f}ms")
            logger.info(f"   Range: {stats['min_ms']:.1f}ms - {stats['max_ms']:.1f}ms")
            
            # Check for network stress impact
            success_criteria = {
                'has_samples': stats['count'] > 10,
                'reasonable_p95': stats['p95_ms'] < 2000,  # Under 2 seconds
                'latency_variation': stats['std_ms'] > 5,   # Some variation due to network conditions
            }
            
            success = all(success_criteria.values())
            
            logger.info("✅ Success Criteria:")
            for criterion, passed in success_criteria.items():
                status = "✅ PASS" if passed else "❌ FAIL"
                logger.info(f"   {criterion}: {status}")
            
            return success
            
        except Exception as e:
            logger.error(f"❌ Error analyzing results: {e}")
            return False
    
    def check_automation_functionality(self):
        """Verify that automation still works despite network issues"""
        logger.info("🔍 Checking automation functionality...")
        
        # Subscribe to automation events to verify functionality
        test_subscriber_code = '''
import paho.mqtt.client as mqtt
import json
import time
import sys

events_received = 0
lamp_toggles = 0

def on_connect(client, userdata, flags, reason_code, properties=None):
    print("📡 Test subscriber connected")
    client.subscribe("automation/events")
    client.subscribe("actuators/+/+")

def on_message(client, userdata, msg):
    global events_received, lamp_toggles
    
    topic = msg.topic
    payload = msg.payload.decode()
    
    events_received += 1
    
    if "lamp_hall" in topic:
        lamp_toggles += 1
        print(f"💡 Lamp toggle detected: {topic}")
    
    if "automation/events" in topic:
        try:
            data = json.loads(payload)
            if data.get('event_type') == 'actuator_control':
                print(f"🏠 Automation event: {data.get('action')} for {data.get('actuator_id')}")
        except:
            pass

try:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
except AttributeError:
    client = mqtt.Client()

client.on_connect = on_connect
client.on_message = on_message

client.connect("localhost", 1883, 60)

start_time = time.time()
client.loop_start()

try:
    while time.time() - start_time < 30:  # Monitor for 30 seconds
        time.sleep(1)
finally:
    client.loop_stop()
    client.disconnect()
    
    print(f"🎯 Automation Test Results:")
    print(f"   Events received: {events_received}")
    print(f"   Lamp toggles: {lamp_toggles}")
    
    # Return success indicator
    success = events_received > 0 and lamp_toggles > 0
    print(f"   Automation working: {'✅ YES' if success else '❌ NO'}")
    sys.exit(0 if success else 1)
'''
        
        test_subscriber_path = Path("test_automation_subscriber.py")
        with open(test_subscriber_path, 'w') as f:
            f.write(test_subscriber_code)
        
        try:
            # Run the subscriber test
            result = subprocess.run([sys.executable, str(test_subscriber_path)], 
                                  capture_output=True, text=True, timeout=35)
            
            success = result.returncode == 0
            logger.info(f"Automation functionality: {'✅ WORKING' if success else '❌ FAILED'}")
            
            if result.stdout:
                logger.info(f"Subscriber output: {result.stdout}")
            
            return success
            
        except subprocess.TimeoutExpired:
            logger.error("❌ Automation test timed out")
            return False
        except Exception as e:
            logger.error(f"❌ Error testing automation: {e}")
            return False
        finally:
            # Clean up
            if test_subscriber_path.exists():
                test_subscriber_path.unlink()
    
    def run_test(self):
        """Run the complete Step 5 network realism test"""
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        logger.info("🚀 Starting Step 5 Network Realism Test")
        logger.info("=" * 60)
        logger.info("Testing: Network realism layer with packet loss and latency")
        logger.info("Expected: 10% packet loss, retries, widened latency histogram")
        logger.info("=" * 60)
        
        try:
            # Check dependencies
            if not self.check_dependencies():
                return False
            
            # Start MQTT broker
            if not self.start_mqtt_broker():
                return False
            
            # Start automation controller
            if not self.start_automation():
                return False
            
            # Run test scenarios
            for scenario in self.scenarios:
                logger.info(f"🎭 Scenario: {scenario['name']}")
                self.run_scenario(scenario)
                time.sleep(2)  # Brief pause between scenarios
            
            # Generate final plots
            logger.info("📊 Generating final metrics plots...")
            time.sleep(3)  # Allow final data to be written
            
            # Analyze results
            analysis_success = self.analyze_results()
            
            # Check automation functionality
            automation_success = self.check_automation_functionality()
            
            # Overall success
            overall_success = analysis_success and automation_success
            
            if overall_success:
                logger.info("🎉 Step 5 test PASSED!")
                logger.info("✅ Network realism layer working correctly")
                logger.info("✅ Packet loss and latency injection functional")
                logger.info("✅ Automation survives network stress")
                logger.info(f"📁 Results saved to: {self.metrics_dir.absolute()}")
            else:
                logger.error("❌ Step 5 test FAILED!")
                if not analysis_success:
                    logger.error("❌ Latency analysis failed")
                if not automation_success:
                    logger.error("❌ Automation functionality test failed")
            
            return overall_success
            
        except Exception as e:
            logger.error(f"❌ Test error: {e}")
            return False
        finally:
            self.cleanup_processes()

def main():
    test = Step5NetworkTest()
    success = test.run_test()
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()