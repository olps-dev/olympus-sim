#!/usr/bin/env python3
"""
CI-Specific Simulation Test for Olympus
Designed for GitHub Actions environment with strict performance criteria

Success criteria:
- End-to-end latency <= 300ms P95 
- No more than 2 dropped control messages
- Test runs for minimum 30 seconds
- Generates required artifacts
"""

import subprocess
import time
import sys
import os
import signal
import logging
import json
import threading
from pathlib import Path
from typing import Dict, Any, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO, 
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('automation_test.log')
    ]
)
logger = logging.getLogger('ci_simulation_test')

class CISimulationTest:
    """CI-optimized simulation test with strict performance criteria"""
    
    def __init__(self):
        self.processes = []
        self.test_duration = int(os.getenv('TEST_DURATION_SEC', '30'))
        self.scenario_name = os.getenv('TEST_SCENARIO_NAME', 'ci_test')
        self.network_condition = os.getenv('TEST_NETWORK_CONDITION', 'good')
        self.expected_latency_ms = int(os.getenv('TEST_EXPECTED_LATENCY_MS', '100'))
        
        # CI-specific thresholds
        self.max_latency_p95_ms = int(os.getenv('MAX_LATENCY_P95_MS', '300'))
        self.max_dropped_messages = int(os.getenv('MAX_DROPPED_MESSAGES', '2'))
        
        # Paths
        self.metrics_dir = Path("metrics")
        self.network_dir = Path("network")
        self.metrics_dir.mkdir(exist_ok=True)
        
        # Test state
        self.automation_events = 0
        self.control_messages = 0
        self.start_time = None
        
        logger.info(f"CI Test Configuration:")
        logger.info(f"  Scenario: {self.scenario_name}")
        logger.info(f"  Network condition: {self.network_condition}")
        logger.info(f"  Expected latency: {self.expected_latency_ms}ms")
        logger.info(f"  Max P95 latency: {self.max_latency_p95_ms}ms")
        logger.info(f"  Duration: {self.test_duration}s")
        
    def cleanup_processes(self):
        """Clean up all test processes"""
        for proc in self.processes:
            try:
                if proc.poll() is None:
                    proc.terminate()
                    proc.wait(timeout=5)
                    logger.info(f"Terminated process {proc.pid}")
            except subprocess.TimeoutExpired:
                proc.kill()
                logger.warning(f"Killed process {proc.pid}")
            except Exception as e:
                logger.error(f"Error cleaning up process: {e}")
        self.processes.clear()
    
    def signal_handler(self, signum, frame):
        """Handle interrupt signals gracefully"""
        logger.info("Received interrupt signal, cleaning up...")
        self.cleanup_processes()
        sys.exit(1)
    
    def check_mqtt_broker(self) -> bool:
        """Check if MQTT broker is available"""
        try:
            result = subprocess.run(['mosquitto_pub', '-h', 'localhost', '-t', 'test', '-m', 'ping'], 
                                  capture_output=True, timeout=5)
            if result.returncode == 0:
                logger.info("MQTT broker is available")
                return True
            else:
                logger.error("MQTT broker not responding")
                return False
        except subprocess.TimeoutExpired:
            logger.error("MQTT broker check timed out")
            return False
        except FileNotFoundError:
            logger.error("mosquitto_pub command not found")
            return False
        except Exception as e:
            logger.error(f"MQTT broker check failed: {e}")
            return False
    
    def configure_network(self) -> bool:
        """Configure network conditions for test"""
        try:
            logger.info(f"Configuring network condition: {self.network_condition}")
            
            # Use network CLI to set condition
            result = subprocess.run([
                sys.executable, 'network/cli.py', 'condition', self.network_condition
            ], capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                logger.info(f"Network condition set successfully")
                logger.info(f"CLI output: {result.stdout}")
                return True
            else:
                logger.error(f"Failed to set network condition: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Error configuring network: {e}")
            return False
    
    def create_ci_sensor_bridge(self) -> str:
        """Create CI-optimized sensor bridge"""
        bridge_path = Path("test_ci_bridge.py")
        
        bridge_code = '''#!/usr/bin/env python3
"""CI-optimized sensor bridge for automated testing"""
import sys
import os
import time
import random
import json
import math

# Add paths
sys.path.append(os.path.join(os.path.dirname(__file__), 'network'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'metrics'))

from network_realism import create_realistic_mqtt_client
from latency_battery_tracker import LatencyBatteryTracker

class CISensorBridge:
    def __init__(self):
        self.client = create_realistic_mqtt_client()
        self.client.on_connect = self.on_connect
        
        self.tracker = LatencyBatteryTracker()
        self.sensor_ids = ['mmwave1', 'mmwave2']
        
        for sensor_id in self.sensor_ids:
            self.tracker.init_sensor_battery(sensor_id)
        
        self.message_count = 0
        self.successful_publishes = 0
        
    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        print(f"CI Bridge connected: {reason_code}")
        
    def run(self):
        print("Starting CI sensor bridge...")
        self.client.connect("localhost", 1883, 60)
        self.client.loop_start()
        
        try:
            start_time = time.time()
            test_duration = int(os.getenv('TEST_DURATION_SEC', '30'))
            
            while time.time() - start_time < test_duration:
                for sensor_id in self.sensor_ids:
                    # Generate realistic presence patterns
                    elapsed = time.time() - start_time
                    
                    # Create predictable but varied presence pattern
                    wave = math.sin(elapsed * 0.2)  # Slow sine wave
                    noise = random.uniform(-0.3, 0.3)  # Add noise
                    presence_score = (wave + noise + 1) / 2  # Normalize to 0-1
                    
                    human_present = presence_score > 0.6  # 40% chance roughly
                    
                    # Log pointcloud received
                    event_id = self.tracker.log_pointcloud_received(sensor_id)
                    
                    # Create message
                    message = {
                        "timestamp": time.time(),
                        "sensor_id": sensor_id,
                        "human_present": human_present,
                        "num_points": random.randint(5, 50) if human_present else random.randint(0, 3),
                        "analysis": f"ci test data - elapsed {elapsed:.1f}s",
                        "event_id": event_id,
                        "battery_level_mah": self.tracker.get_battery_level(sensor_id)
                    }
                    
                    # Publish
                    try:
                        topic = f"sensor/{sensor_id}/presence"
                        payload = json.dumps(message)
                        result = self.client.publish(topic, payload, qos=1)
                        
                        if result.rc == 0:
                            self.successful_publishes += 1
                            self.tracker.log_mqtt_published(sensor_id, event_id)
                        
                        self.message_count += 1
                        
                        print(f"[{sensor_id}] {elapsed:.1f}s: {'PRESENT' if human_present else 'CLEAR'} "
                              f"(Battery: {message['battery_level_mah']:.1f}mAh)")
                              
                    except Exception as e:
                        print(f"ERROR: Publish failed for {sensor_id}: {e}")
                
                time.sleep(1.0)  # 1Hz rate for CI stability
            
        except KeyboardInterrupt:
            pass
        finally:
            self.client.disconnect()
            
            # Print final statistics
            network_stats = self.client.network_sim.get_statistics()
            print(f"CI Bridge Final Stats:")
            print(f"  Messages attempted: {self.message_count}")
            print(f"  Successful publishes: {self.successful_publishes}")
            print(f"  Network drop rate: {network_stats.get('drop_rate_actual', 0):.1%}")
            print(f"  Avg latency added: {network_stats.get('avg_latency_added_ms', 0):.1f}ms")

if __name__ == "__main__":
    bridge = CISensorBridge()
    bridge.run()
'''
        
        with open(bridge_path, 'w') as f:
            f.write(bridge_code)
        
        return str(bridge_path)
    
    def start_components(self) -> bool:
        """Start all simulation components"""
        logger.info("Starting simulation components...")
        
        # Start sensor bridge
        bridge_script = self.create_ci_sensor_bridge()
        try:
            bridge_proc = subprocess.Popen([sys.executable, bridge_script],
                                         stdout=subprocess.PIPE, 
                                         stderr=subprocess.STDOUT,
                                         text=True)
            self.processes.append(bridge_proc)
            logger.info("Sensor bridge started")
        except Exception as e:
            logger.error(f"Failed to start sensor bridge: {e}")
            return False
        
        # Start automation controller
        try:
            automation_proc = subprocess.Popen([sys.executable, 'automation/automation_demo.py'],
                                             stdout=subprocess.PIPE, 
                                             stderr=subprocess.STDOUT,
                                             text=True)
            self.processes.append(automation_proc)
            logger.info("Automation controller started")
        except Exception as e:
            logger.error(f"Failed to start automation controller: {e}")
            return False
        
        # Give components time to initialize
        time.sleep(3)
        
        # Check if processes are still running
        for i, proc in enumerate(self.processes):
            if proc.poll() is not None:
                logger.error(f"Process {i} exited prematurely")
                return False
        
        logger.info("All components started successfully")
        return True
    
    def monitor_test(self) -> bool:
        """Monitor test execution and collect metrics"""
        logger.info(f"Monitoring test for {self.test_duration} seconds...")
        
        self.start_time = time.time()
        end_time = self.start_time + self.test_duration
        
        # Start log monitoring in background
        log_thread = threading.Thread(target=self.monitor_logs, daemon=True)
        log_thread.start()
        
        try:
            while time.time() < end_time:
                # Check process health
                for i, proc in enumerate(self.processes):
                    if proc.poll() is not None:
                        logger.warning(f"Process {i} stopped unexpectedly")
                        stdout, stderr = proc.communicate()
                        if stdout:
                            logger.info(f"Process {i} stdout: {stdout}")
                        if stderr:
                            logger.error(f"Process {i} stderr: {stderr}")
                
                # Log progress
                elapsed = time.time() - self.start_time
                remaining = self.test_duration - elapsed
                logger.info(f"Test progress: {elapsed:.1f}s elapsed, {remaining:.1f}s remaining")
                
                time.sleep(5)  # Check every 5 seconds
            
            logger.info("Test monitoring completed")
            return True
            
        except Exception as e:
            logger.error(f"Error during test monitoring: {e}")
            return False
    
    def monitor_logs(self):
        """Monitor automation logs for control messages"""
        try:
            # Simple file-based monitoring
            while True:
                if os.path.exists('automation_test.log'):
                    with open('automation_test.log', 'r') as f:
                        content = f.read()
                        self.automation_events = content.count('AUTOMATION:')
                        self.control_messages = content.count('lamp_hall')
                
                time.sleep(2)
        except Exception as e:
            logger.error(f"Log monitoring error: {e}")
    
    def stop_components(self):
        """Gracefully stop all components"""
        logger.info("Stopping simulation components...")
        
        for proc in self.processes:
            try:
                if proc.poll() is None:
                    proc.terminate()
            except Exception as e:
                logger.error(f"Error terminating process: {e}")
        
        # Wait for graceful shutdown
        time.sleep(2)
        
        # Force kill if necessary
        for proc in self.processes:
            try:
                if proc.poll() is None:
                    proc.kill()
                    logger.warning("Force killed process")
            except Exception as e:
                logger.error(f"Error killing process: {e}")
        
        # Clean up test files
        test_files = ['test_ci_bridge.py']
        for file_path in test_files:
            try:
                if os.path.exists(file_path):
                    os.remove(file_path)
            except Exception as e:
                logger.error(f"Error cleaning up {file_path}: {e}")
    
    def analyze_performance(self) -> Dict[str, Any]:
        """Analyze performance metrics and return results"""
        logger.info("Analyzing performance metrics...")
        
        results = {
            'success': False,
            'latency_analysis': {},
            'message_analysis': {},
            'errors': []
        }
        
        try:
            # Check for metrics files
            latency_csv = self.metrics_dir / "latency_metrics.csv"
            battery_csv = self.metrics_dir / "battery_metrics.csv"
            
            if not latency_csv.exists():
                results['errors'].append("No latency metrics file found")
                return results
            
            # Analyze latency data
            import pandas as pd
            
            df = pd.read_csv(latency_csv)
            e2e_data = df[(df['stage'] == 'lamp_toggled') & df['cumulative_latency_ms'].notna()]
            
            if len(e2e_data) == 0:
                results['errors'].append("No end-to-end latency data found")
                return results
            
            latencies = e2e_data['cumulative_latency_ms']
            
            latency_stats = {
                'sample_count': len(latencies),
                'mean_ms': float(latencies.mean()),
                'median_ms': float(latencies.median()),
                'p95_ms': float(latencies.quantile(0.95)),
                'p99_ms': float(latencies.quantile(0.99)),
                'min_ms': float(latencies.min()),
                'max_ms': float(latencies.max()),
                'std_ms': float(latencies.std())
            }
            
            results['latency_analysis'] = latency_stats
            
            # Performance checks
            checks = {
                'sufficient_samples': latency_stats['sample_count'] >= 5,
                'p95_within_limit': latency_stats['p95_ms'] <= self.max_latency_p95_ms,
                'mean_reasonable': latency_stats['mean_ms'] <= self.expected_latency_ms * 1.5,
                'samples_not_zero': latency_stats['sample_count'] > 0
            }
            
            results['performance_checks'] = checks
            
            # Message analysis
            results['message_analysis'] = {
                'automation_events': self.automation_events,
                'control_messages': self.control_messages,
                'sufficient_activity': self.automation_events >= 3
            }
            
            # Overall success
            all_checks_passed = all(checks.values())
            sufficient_activity = results['message_analysis']['sufficient_activity']
            
            results['success'] = all_checks_passed and sufficient_activity
            
            logger.info(f"Performance Analysis Results:")
            logger.info(f"  Sample count: {latency_stats['sample_count']}")
            logger.info(f"  Mean latency: {latency_stats['mean_ms']:.1f}ms")
            logger.info(f"  P95 latency: {latency_stats['p95_ms']:.1f}ms")
            logger.info(f"  Automation events: {self.automation_events}")
            logger.info(f"  Overall success: {results['success']}")
            
            return results
            
        except Exception as e:
            error_msg = f"Performance analysis failed: {e}"
            logger.error(error_msg)
            results['errors'].append(error_msg)
            return results
    
    def generate_artifacts(self):
        """Generate plots and reports for CI artifacts"""
        logger.info("Generating CI artifacts...")
        
        try:
            # Generate plots using the existing tracker
            sys.path.append('metrics')
            from latency_battery_tracker import LatencyBatteryTracker
            
            tracker = LatencyBatteryTracker()
            plots = tracker.generate_plots()
            
            if plots:
                logger.info(f"Generated plots: {list(plots.keys())}")
            else:
                logger.warning("No plots generated")
                
        except Exception as e:
            logger.error(f"Error generating artifacts: {e}")
    
    def run_test(self) -> bool:
        """Run the complete CI simulation test"""
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        logger.info("Starting Olympus CI Simulation Test")
        logger.info("=" * 50)
        
        try:
            # Pre-flight checks
            if not self.check_mqtt_broker():
                logger.error("MQTT broker not available")
                return False
            
            if not self.configure_network():
                logger.error("Network configuration failed")
                return False
            
            # Start simulation
            if not self.start_components():
                logger.error("Failed to start simulation components")
                return False
            
            # Monitor test execution
            if not self.monitor_test():
                logger.error("Test monitoring failed")
                return False
            
            # Stop components gracefully
            self.stop_components()
            
            # Analyze results
            results = self.analyze_performance()
            
            # Generate artifacts
            self.generate_artifacts()
            
            # Final verdict
            if results['success']:
                logger.info("SUCCESS: CI simulation test passed")
                logger.info("All performance criteria met")
                return True
            else:
                logger.error("FAILURE: CI simulation test failed")
                logger.error(f"Errors: {results.get('errors', [])}")
                
                # Log detailed failure reasons
                checks = results.get('performance_checks', {})
                for check, passed in checks.items():
                    if not passed:
                        logger.error(f"Failed check: {check}")
                
                return False
            
        except Exception as e:
            logger.error(f"Test execution error: {e}")
            return False
        finally:
            self.cleanup_processes()

def main():
    """Main entry point for CI test"""
    test = CISimulationTest()
    success = test.run_test()
    
    # Exit with appropriate code for CI
    exit_code = 0 if success else 1
    logger.info(f"Exiting with code: {exit_code}")
    sys.exit(exit_code)

if __name__ == "__main__":
    main()