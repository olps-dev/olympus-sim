#!/usr/bin/env python3
"""
Latency and Battery Metrics Tracker for Olympus Simulation
Implements Step 4: Latency & battery instrumentation

Tracks:
- End-to-end latency from PointCloud → MQTT → Lamp toggle
- Battery consumption per sensor frame and Wi-Fi transmission
- Generates histogram plots and battery curves
"""

import time
import csv
import json
import logging
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional

logger = logging.getLogger('latency_battery_tracker')

class LatencyBatteryTracker:
    def __init__(self, output_dir: str = "metrics"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # CSV files for data logging
        self.latency_csv = self.output_dir / "latency_metrics.csv"
        self.battery_csv = self.output_dir / "battery_metrics.csv"
        
        # Initialize CSV files with headers
        self._init_csv_files()
        
        # Battery model parameters (per sensor)
        self.battery_config = {
            'initial_capacity_mah': 3000,  # Initial battery capacity
            'frame_cost_mah': 0.1,         # mAh consumed per mmWave frame
            'wifi_tx_cost_mah': 0.05,      # mAh consumed per MQTT publish
            'base_consumption_mah_per_sec': 0.5  # Baseline consumption
        }
        
        # Battery state tracking (per sensor)
        self.battery_states = {}
        
        # Active tracking sessions (event_id -> {stage: timestamp})
        self.active_sessions = {}
        
        # Start time for battery tracking
        self.start_time = time.time()
        
        logger.info(f"Metrics tracker initialized - output: {self.output_dir}")
    
    def _init_csv_files(self):
        """Initialize CSV files with appropriate headers"""
        # Latency metrics CSV
        latency_headers = [
            'timestamp', 'event_id', 'sensor_id', 'stage', 
            'stage_timestamp', 'cumulative_latency_ms', 'stage_latency_ms'
        ]
        
        if not self.latency_csv.exists():
            with open(self.latency_csv, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(latency_headers)
        
        # Battery metrics CSV
        battery_headers = [
            'timestamp', 'sensor_id', 'event_type', 'battery_level_mah', 
            'consumption_mah', 'runtime_seconds'
        ]
        
        if not self.battery_csv.exists():
            with open(self.battery_csv, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(battery_headers)
    
    def init_sensor_battery(self, sensor_id: str) -> None:
        """Initialize battery state for a sensor"""
        if sensor_id not in self.battery_states:
            self.battery_states[sensor_id] = {
                'capacity_mah': self.battery_config['initial_capacity_mah'],
                'last_update': time.time()
            }
            logger.info(f"Initialized battery for {sensor_id}: {self.battery_config['initial_capacity_mah']} mAh")
    
    def log_pointcloud_received(self, sensor_id: str, event_id: str = None) -> str:
        """Log when a PointCloud2 message is received"""
        if event_id is None:
            event_id = f"{sensor_id}_{int(time.time() * 1000)}"
        
        timestamp = time.time()
        
        # Start new tracking session
        self.active_sessions[event_id] = {
            'pointcloud_received': timestamp,
            'sensor_id': sensor_id
        }
        
        # Log to CSV
        self._log_latency_event(event_id, sensor_id, 'pointcloud_received', timestamp)
        
        # Update battery (frame processing cost)
        self._consume_battery(sensor_id, 'frame_processing', self.battery_config['frame_cost_mah'])
        
        return event_id
    
    def log_mqtt_published(self, sensor_id: str, event_id: str) -> None:
        """Log when MQTT message is published"""
        if event_id not in self.active_sessions:
            logger.debug(f"No active session for event_id: {event_id}, creating new session")
            # Create a minimal session for this event
            timestamp = time.time()
            self.active_sessions[event_id] = {
                'pointcloud_received': timestamp - 0.01,  # Assume 10ms processing time
                'sensor_id': sensor_id
            }
        
        timestamp = time.time()
        session = self.active_sessions[event_id]
        session['mqtt_published'] = timestamp
        
        # Calculate stage latency
        stage_latency = (timestamp - session['pointcloud_received']) * 1000
        
        # Log to CSV
        self._log_latency_event(event_id, sensor_id, 'mqtt_published', timestamp, stage_latency)
        
        # Update battery (Wi-Fi transmission cost)
        self._consume_battery(sensor_id, 'wifi_transmission', self.battery_config['wifi_tx_cost_mah'])
    
    def log_automation_triggered(self, event_id: str, sensor_id: str) -> None:
        """Log when automation rule is triggered"""
        if event_id not in self.active_sessions:
            # Create session if it doesn't exist (for external triggers)
            self.active_sessions[event_id] = {'sensor_id': sensor_id}
        
        timestamp = time.time()
        session = self.active_sessions[event_id]
        session['automation_triggered'] = timestamp
        
        # Calculate stage latency if we have previous stage
        stage_latency = None
        if 'mqtt_published' in session:
            stage_latency = (timestamp - session['mqtt_published']) * 1000
        
        # Log to CSV
        self._log_latency_event(event_id, sensor_id, 'automation_triggered', timestamp, stage_latency)
    
    def log_lamp_toggled(self, event_id: str, sensor_id: str) -> None:
        """Log when lamp is actually toggled - completes the latency chain"""
        if event_id not in self.active_sessions:
            logger.warning(f"No active session for event_id: {event_id}")
            return
        
        timestamp = time.time()
        session = self.active_sessions[event_id]
        session['lamp_toggled'] = timestamp
        
        # Calculate stage latency
        stage_latency = None
        if 'automation_triggered' in session:
            stage_latency = (timestamp - session['automation_triggered']) * 1000
        
        # Calculate end-to-end latency
        e2e_latency = None
        if 'pointcloud_received' in session:
            e2e_latency = (timestamp - session['pointcloud_received']) * 1000
        
        # Log to CSV (cumulative_latency_ms, stage_latency_ms)  
        self._log_latency_event(event_id, sensor_id, 'lamp_toggled', timestamp, e2e_latency, stage_latency)
        
        # Clean up session
        del self.active_sessions[event_id]
        
        logger.info(f"End-to-end latency for {sensor_id}: {e2e_latency:.1f}ms")
    
    def _log_latency_event(self, event_id: str, sensor_id: str, stage: str, 
                          stage_timestamp: float, cumulative_latency_ms: float = None, 
                          stage_latency_ms: float = None):
        """Log latency event to CSV"""
        try:
            with open(self.latency_csv, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    time.time(), event_id, sensor_id, stage, 
                    stage_timestamp, cumulative_latency_ms, stage_latency_ms
                ])
        except Exception as e:
            logger.error(f"Error logging latency event: {e}")
    
    def _consume_battery(self, sensor_id: str, event_type: str, consumption_mah: float):
        """Update battery state and log consumption"""
        self.init_sensor_battery(sensor_id)
        
        current_time = time.time()
        battery_state = self.battery_states[sensor_id]
        
        # Add baseline consumption since last update
        time_delta = current_time - battery_state['last_update']
        baseline_consumption = self.battery_config['base_consumption_mah_per_sec'] * time_delta
        
        # Total consumption
        total_consumption = consumption_mah + baseline_consumption
        
        # Update battery level
        battery_state['capacity_mah'] -= total_consumption
        battery_state['last_update'] = current_time
        
        # Calculate runtime
        runtime = current_time - self.start_time
        
        # Log to CSV
        try:
            with open(self.battery_csv, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    current_time, sensor_id, event_type, 
                    battery_state['capacity_mah'], total_consumption, runtime
                ])
        except Exception as e:
            logger.error(f"Error logging battery consumption: {e}")
        
        # Log warning if battery is low
        if battery_state['capacity_mah'] < 300:  # 10% of initial capacity
            logger.warning(f"Low battery for {sensor_id}: {battery_state['capacity_mah']:.1f} mAh remaining")
    
    def get_battery_level(self, sensor_id: str) -> float:
        """Get current battery level for a sensor"""
        self.init_sensor_battery(sensor_id)
        # Update for baseline consumption
        self._consume_battery(sensor_id, 'baseline_check', 0)
        return self.battery_states[sensor_id]['capacity_mah']
    
    def generate_plots(self, duration_minutes: int = 1) -> Dict[str, str]:
        """Generate latency histogram and battery curve plots"""
        plots_generated = {}
        
        try:
            # Generate latency histogram
            latency_plot = self._generate_latency_histogram()
            if latency_plot:
                plots_generated['latency_histogram'] = latency_plot
            
            # Generate battery curve
            battery_plot = self._generate_battery_curve()
            if battery_plot:
                plots_generated['battery_curve'] = battery_plot
            
            logger.info(f"Generated plots: {list(plots_generated.keys())}")
            
        except Exception as e:
            logger.error(f"Error generating plots: {e}")
        
        return plots_generated
    
    def _generate_latency_histogram(self) -> Optional[str]:
        """Generate latency histogram from CSV data"""
        try:
            # Read latency data
            df = pd.read_csv(self.latency_csv)
            
            # Filter for end-to-end latencies (lamp_toggled events with cumulative_latency_ms)
            e2e_data = df[(df['stage'] == 'lamp_toggled') & df['cumulative_latency_ms'].notna()]
            
            if len(e2e_data) == 0:
                logger.warning("No end-to-end latency data available for histogram")
                return None
            
            # Create histogram
            plt.figure(figsize=(10, 6))
            latencies = e2e_data['cumulative_latency_ms']
            
            plt.hist(latencies, bins=20, alpha=0.7, color='skyblue', edgecolor='black')
            plt.axvline(latencies.mean(), color='red', linestyle='--', label=f'Mean: {latencies.mean():.1f}ms')
            plt.axvline(latencies.quantile(0.95), color='orange', linestyle='--', label=f'P95: {latencies.quantile(0.95):.1f}ms')
            
            plt.xlabel('End-to-End Latency (ms)')
            plt.ylabel('Frequency')
            plt.title('Olympus Sensor-to-Actuator Latency Distribution')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            # Add statistics text
            stats_text = f"""Statistics:
Mean: {latencies.mean():.1f}ms
Median: {latencies.median():.1f}ms
P95: {latencies.quantile(0.95):.1f}ms
Min: {latencies.min():.1f}ms
Max: {latencies.max():.1f}ms
Count: {len(latencies)}"""
            
            plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
            
            plot_path = self.output_dir / "latency_hist.png"
            plt.savefig(plot_path, dpi=150, bbox_inches='tight')
            plt.close()
            
            logger.info(f"Latency histogram saved: {plot_path}")
            return str(plot_path)
            
        except Exception as e:
            logger.error(f"Error generating latency histogram: {e}")
            return None
    
    def _generate_battery_curve(self) -> Optional[str]:
        """Generate battery level curve from CSV data"""
        try:
            # Read battery data
            df = pd.read_csv(self.battery_csv)
            
            if len(df) == 0:
                logger.warning("No battery data available for curve")
                return None
            
            # Create battery curve plot
            plt.figure(figsize=(12, 8))
            
            # Plot each sensor separately
            sensors = df['sensor_id'].unique()
            colors = plt.cm.Set1(np.linspace(0, 1, len(sensors)))
            
            for sensor, color in zip(sensors, colors):
                sensor_data = df[df['sensor_id'] == sensor].sort_values('runtime_seconds')
                plt.plot(sensor_data['runtime_seconds'] / 60, sensor_data['battery_level_mah'], 
                        label=sensor, color=color, linewidth=2, marker='o', markersize=3)
            
            plt.xlabel('Runtime (minutes)')
            plt.ylabel('Battery Level (mAh)')
            plt.title('Olympus Sensor Battery Consumption Over Time')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            # Add horizontal line for low battery threshold
            plt.axhline(300, color='red', linestyle='--', alpha=0.7, label='Low Battery (10%)')
            
            plot_path = self.output_dir / "battery_curve.png"
            plt.savefig(plot_path, dpi=150, bbox_inches='tight')
            plt.close()
            
            logger.info(f"Battery curve saved: {plot_path}")
            return str(plot_path)
            
        except Exception as e:
            logger.error(f"Error generating battery curve: {e}")
            return None
    
    def get_metrics_summary(self) -> Dict:
        """Get summary of current metrics"""
        try:
            # Latency summary
            latency_df = pd.read_csv(self.latency_csv)
            e2e_latencies = latency_df[(latency_df['stage'] == 'lamp_toggled') & 
                                     latency_df['cumulative_latency_ms'].notna()]['cumulative_latency_ms']
            
            # Battery summary
            battery_df = pd.read_csv(self.battery_csv)
            
            summary = {
                'latency': {
                    'count': len(e2e_latencies),
                    'mean_ms': e2e_latencies.mean() if len(e2e_latencies) > 0 else 0,
                    'p95_ms': e2e_latencies.quantile(0.95) if len(e2e_latencies) > 0 else 0,
                    'max_ms': e2e_latencies.max() if len(e2e_latencies) > 0 else 0
                },
                'battery': {
                    'sensors': {sensor: self.get_battery_level(sensor) 
                              for sensor in self.battery_states.keys()},
                    'runtime_minutes': (time.time() - self.start_time) / 60
                }
            }
            
            return summary
            
        except Exception as e:
            logger.error(f"Error generating metrics summary: {e}")
            return {}