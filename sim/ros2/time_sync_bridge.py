#!/usr/bin/env python3
"""
Project Olympus - ROS 2 Time Synchronization Bridge
Synchronizes simulation time across Gazebo, ns-3, QEMU, and backend services
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState, SetModelState

import json
import time
import threading
import asyncio
import logging
import signal
import sys
from typing import Dict, Any, List, Optional
from dataclasses import dataclass, asdict

import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
log = logging.getLogger('TimeSyncBridge')

@dataclass
class ActorState:
    """State of a human actor in the simulation."""
    name: str
    position: tuple  # (x, y, z)
    velocity: tuple  # (vx, vy, vz)
    timestamp: float
    detected_by_nodes: List[int]

@dataclass
class SimulationMetrics:
    """Comprehensive simulation metrics for KPI analysis."""
    timestamp: float
    simulation_time: float
    real_time_factor: float
    
    # Network metrics
    mesh_packet_loss: float
    mesh_latency_ms: float
    mesh_throughput_mbps: float
    
    # Sensor metrics
    sensor_readings_per_sec: float
    sensor_error_rate: float
    
    # Power metrics
    avg_power_consumption_mw: float
    projected_battery_hours: float
    
    # Latency metrics (end-to-end)
    sensor_to_mqtt_latency_ms: float
    mqtt_to_dashboard_latency_ms: float
    total_pipeline_latency_ms: float
    
    # Actor detection metrics
    actor_detections_per_minute: float
    false_positive_rate: float
    false_negative_rate: float

class OlympusTimeSyncBridge(Node):
    """ROS 2 bridge for time synchronization and KPI collection."""
    
    def __init__(self):
        super().__init__('olympus_time_sync_bridge')
        
        # Configuration
        self.simulation_rate = 10.0  # 10 Hz
        self.real_time_factor = 1.0
        self.start_time = time.time()
        self.simulation_time = 0.0
        
        # State tracking
        self.actor_states: Dict[str, ActorState] = {}
        self.sensor_nodes: Dict[int, Dict[str, Any]] = {}
        self.network_stats: Dict[str, Any] = {}
        self.metrics_history: List[SimulationMetrics] = []
        
        # External connections
        self.mqtt_client = None
        self.influx_client = None
        self.influx_write_api = None
        
        # ROS 2 subscriptions and services
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        
        self.get_model_state_client = self.create_client(
            GetModelState,
            '/gazebo/get_model_state'
        )
        
        # Publishers for simulation control
        self.time_sync_pub = self.create_publisher(
            Float64,
            '/olympus/simulation_time',
            10
        )
        
        self.metrics_pub = self.create_publisher(
            String,
            '/olympus/metrics',
            10
        )
        
        self.actor_poses_pub = self.create_publisher(
            PoseArray,
            '/olympus/actor_poses',
            10
        )
        
        # Timer for main sync loop
        self.sync_timer = self.create_timer(
            1.0 / self.simulation_rate,
            self.sync_loop_callback
        )
        
        # Initialize external connections
        self.init_mqtt_connection()
        self.init_influx_connection()
        
        log.info("Olympus Time Sync Bridge initialized")
    
    def init_mqtt_connection(self):
        """Initialize MQTT connection for sensor data collection."""
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            
            # Connect to MQTT broker
            self.mqtt_client.connect("mqtt", 1883, 60)
            self.mqtt_client.loop_start()
            
            log.info("MQTT connection initialized")
            
        except Exception as e:
            log.error(f"Failed to initialize MQTT: {e}")
    
    def init_influx_connection(self):
        """Initialize InfluxDB connection for metrics storage."""
        try:
            self.influx_client = InfluxDBClient(
                url="http://influxdb:8086",
                token="olympus_token",
                org="olympus"
            )
            self.influx_write_api = self.influx_client.write_api(write_options=SYNCHRONOUS)
            
            log.info("InfluxDB connection initialized")
            
        except Exception as e:
            log.error(f"Failed to initialize InfluxDB: {e}")
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Handle MQTT connection."""
        if rc == 0:
            log.info("Connected to MQTT broker")
            # Subscribe to sensor topics
            client.subscribe("olympus/sensors/+/+")
            client.subscribe("olympus/network/+")
        else:
            log.error(f"MQTT connection failed with code {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages from sensors."""
        try:
            topic_parts = msg.topic.split('/')
            
            if len(topic_parts) >= 4 and topic_parts[1] == 'sensors':
                # Sensor data message: olympus/sensors/node_id/sensor_type
                node_id = int(topic_parts[2])
                sensor_type = topic_parts[3]
                
                # Parse sensor data
                data = json.loads(msg.payload.decode())
                data['timestamp'] = time.time()
                data['simulation_time'] = self.simulation_time
                
                # Store sensor reading
                if node_id not in self.sensor_nodes:
                    self.sensor_nodes[node_id] = {}
                
                self.sensor_nodes[node_id][sensor_type] = data
                
                # Update metrics
                self.update_sensor_metrics(node_id, sensor_type, data)
                
            elif len(topic_parts) >= 3 and topic_parts[1] == 'network':
                # Network statistics message
                network_type = topic_parts[2]
                data = json.loads(msg.payload.decode())
                self.network_stats[network_type] = data
                
        except Exception as e:
            log.error(f"Error processing MQTT message: {e}")
    
    def model_states_callback(self, msg):
        """Handle Gazebo model states for actor tracking."""
        try:
            current_time = time.time()
            
            # Extract actor positions
            for i, name in enumerate(msg.name):
                if name.startswith('person_'):
                    pose = msg.pose[i]
                    twist = msg.twist[i]
                    
                    position = (pose.position.x, pose.position.y, pose.position.z)
                    velocity = (twist.linear.x, twist.linear.y, twist.linear.z)
                    
                    # Check which sensor nodes can detect this actor
                    detected_nodes = self.calculate_actor_detection(position)
                    
                    # Update actor state
                    self.actor_states[name] = ActorState(
                        name=name,
                        position=position,
                        velocity=velocity,
                        timestamp=current_time,
                        detected_by_nodes=detected_nodes
                    )
            
            # Publish actor poses for visualization
            self.publish_actor_poses()
            
        except Exception as e:
            log.error(f"Error processing model states: {e}")
    
    def calculate_actor_detection(self, position: tuple) -> List[int]:
        """Calculate which sensor nodes should detect an actor at given position."""
        detected_nodes = []
        
        # Sensor node positions (from Gazebo world)
        sensor_positions = {
            0: (2.0, 2.0, 1.5),    # Living room
            1: (8.0, 2.0, 1.5),    # Bedroom 1
            2: (8.0, 6.0, 1.5),    # Bedroom 2
            3: (2.0, 6.0, 1.5),    # Kitchen
            4: (5.0, 4.0, 1.5),    # Hallway
        }
        
        # Detection range for mmWave radar (5 meter range)
        detection_range = 5.0
        
        for node_id, sensor_pos in sensor_positions.items():
            distance = ((position[0] - sensor_pos[0])**2 + 
                       (position[1] - sensor_pos[1])**2)**0.5
            
            if distance <= detection_range:
                detected_nodes.append(node_id)
        
        return detected_nodes
    
    def publish_actor_poses(self):
        """Publish actor poses for visualization and analysis."""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "world"
        
        for actor_state in self.actor_states.values():
            pose = Pose()
            pose.position.x = actor_state.position[0]
            pose.position.y = actor_state.position[1]
            pose.position.z = actor_state.position[2]
            pose_array.poses.append(pose)
        
        self.actor_poses_pub.publish(pose_array)
    
    def update_sensor_metrics(self, node_id: int, sensor_type: str, data: Dict[str, Any]):
        """Update sensor-specific metrics."""
        # Simulate realistic sensor behavior based on actor proximity
        detection_bonus = 0
        for actor_state in self.actor_states.values():
            if node_id in actor_state.detected_by_nodes:
                detection_bonus += 1
        
        # Adjust sensor readings based on detections (simulate mmWave interference)
        if sensor_type == 'bme680' and detection_bonus > 0:
            # Simulate slight temperature increase from human presence
            if 'temperature' in data:
                data['temperature'] += detection_bonus * 0.2
        
        # Send detection triggers to sensor nodes
        if detection_bonus > 0:
            self.trigger_sensor_detection(node_id, detection_bonus)
    
    def trigger_sensor_detection(self, node_id: int, detection_count: int):
        """Trigger sensor detection event via MQTT."""
        detection_msg = {
            'type': 'detection',
            'node_id': node_id,
            'detection_count': detection_count,
            'timestamp': time.time(),
            'simulation_time': self.simulation_time
        }
        
        if self.mqtt_client:
            topic = f"olympus/detections/{node_id}"
            self.mqtt_client.publish(topic, json.dumps(detection_msg))
    
    def collect_simulation_metrics(self) -> SimulationMetrics:
        """Collect comprehensive simulation metrics."""
        current_time = time.time()
        elapsed_real_time = current_time - self.start_time
        
        # Calculate real-time factor
        if elapsed_real_time > 0:
            self.real_time_factor = self.simulation_time / elapsed_real_time
        
        # Network metrics (from ns-3 mesh controller)
        mesh_stats = self.network_stats.get('mesh', {})
        mesh_packet_loss = mesh_stats.get('avg_packet_loss', 0.05)
        mesh_latency_ms = mesh_stats.get('avg_latency_ms', 10.0)
        mesh_throughput_mbps = mesh_stats.get('avg_throughput_mbps', 20.0)
        
        # Sensor metrics
        total_sensors = len(self.sensor_nodes) * 3  # 3 sensors per node
        sensor_readings_per_sec = total_sensors * 1.0  # 1 Hz per sensor
        sensor_error_rate = 0.001  # Simulated 0.1% error rate
        
        # Power metrics (realistic ESP32 consumption)
        active_power_mw = 160  # Active WiFi transmission
        sleep_power_mw = 0.01  # Deep sleep
        duty_cycle = 0.1  # 10% active time
        avg_power_mw = active_power_mw * duty_cycle + sleep_power_mw * (1 - duty_cycle)
        
        # Battery capacity: 3000mAh @ 3.7V = 11.1Wh = 11100mWh
        battery_capacity_mwh = 11100
        projected_battery_hours = battery_capacity_mwh / avg_power_mw
        
        # Latency metrics (simulated pipeline)
        sensor_to_mqtt_latency_ms = 50 + mesh_latency_ms  # Sensor processing + mesh
        mqtt_to_dashboard_latency_ms = 20  # Backend processing
        total_pipeline_latency_ms = sensor_to_mqtt_latency_ms + mqtt_to_dashboard_latency_ms
        
        # Actor detection metrics
        total_detections = sum(len(actor.detected_by_nodes) for actor in self.actor_states.values())
        actor_detections_per_minute = total_detections * 6  # Convert to per-minute rate
        false_positive_rate = 0.02  # Simulated 2% false positives
        false_negative_rate = 0.05  # Simulated 5% false negatives
        
        return SimulationMetrics(
            timestamp=current_time,
            simulation_time=self.simulation_time,
            real_time_factor=self.real_time_factor,
            mesh_packet_loss=mesh_packet_loss,
            mesh_latency_ms=mesh_latency_ms,
            mesh_throughput_mbps=mesh_throughput_mbps,
            sensor_readings_per_sec=sensor_readings_per_sec,
            sensor_error_rate=sensor_error_rate,
            avg_power_consumption_mw=avg_power_mw,
            projected_battery_hours=projected_battery_hours,
            sensor_to_mqtt_latency_ms=sensor_to_mqtt_latency_ms,
            mqtt_to_dashboard_latency_ms=mqtt_to_dashboard_latency_ms,
            total_pipeline_latency_ms=total_pipeline_latency_ms,
            actor_detections_per_minute=actor_detections_per_minute,
            false_positive_rate=false_positive_rate,
            false_negative_rate=false_negative_rate
        )
    
    def store_metrics_to_influx(self, metrics: SimulationMetrics):
        """Store metrics to InfluxDB for analysis."""
        if not self.influx_write_api:
            return
        
        try:
            # Create InfluxDB points
            points = [
                Point("simulation_time").time(int(metrics.timestamp * 1e9)).field("value", metrics.simulation_time),
                Point("real_time_factor").time(int(metrics.timestamp * 1e9)).field("value", metrics.real_time_factor),
                Point("mesh_packet_loss").time(int(metrics.timestamp * 1e9)).field("value", metrics.mesh_packet_loss),
                Point("mesh_latency_ms").time(int(metrics.timestamp * 1e9)).field("value", metrics.mesh_latency_ms),
                Point("projected_battery_hours").time(int(metrics.timestamp * 1e9)).field("value", metrics.projected_battery_hours),
                Point("total_pipeline_latency_ms").time(int(metrics.timestamp * 1e9)).field("value", metrics.total_pipeline_latency_ms),
                Point("actor_detections_per_minute").time(int(metrics.timestamp * 1e9)).field("value", metrics.actor_detections_per_minute),
            ]
            
            self.influx_write_api.write(bucket="olympus", record=points)
            
        except Exception as e:
            log.error(f"Failed to store metrics to InfluxDB: {e}")
    
    def sync_loop_callback(self):
        """Main synchronization loop callback."""
        # Update simulation time
        self.simulation_time += 1.0 / self.simulation_rate
        
        # Publish synchronized time
        time_msg = Float64()
        time_msg.data = self.simulation_time
        self.time_sync_pub.publish(time_msg)
        
        # Collect and publish metrics every second
        if int(self.simulation_time) % 1 == 0:
            metrics = self.collect_simulation_metrics()
            self.metrics_history.append(metrics)
            
            # Keep only last 1000 metrics (about 16 minutes)
            if len(self.metrics_history) > 1000:
                self.metrics_history = self.metrics_history[-1000:]
            
            # Publish metrics
            metrics_msg = String()
            metrics_msg.data = json.dumps(asdict(metrics))
            self.metrics_pub.publish(metrics_msg)
            
            # Store to InfluxDB
            self.store_metrics_to_influx(metrics)
            
            # Log key metrics
            log.info(f"Sim time: {self.simulation_time:.1f}s, "
                    f"RTF: {metrics.real_time_factor:.2f}, "
                    f"Pipeline latency: {metrics.total_pipeline_latency_ms:.1f}ms, "
                    f"Battery life: {metrics.projected_battery_hours:.1f}h, "
                    f"Detections/min: {metrics.actor_detections_per_minute:.1f}")
            
            # Check CI/KPI thresholds
            self.check_kpi_thresholds(metrics)
    
    def check_kpi_thresholds(self, metrics: SimulationMetrics):
        """Check KPI thresholds for CI/CD pipeline validation."""
        failures = []
        
        # P95 latency threshold: <300ms
        if metrics.total_pipeline_latency_ms > 300:
            failures.append(f"Pipeline latency {metrics.total_pipeline_latency_ms:.1f}ms > 300ms threshold")
        
        # Battery life threshold: >9 days (216 hours)
        if metrics.projected_battery_hours < 216:
            failures.append(f"Battery life {metrics.projected_battery_hours:.1f}h < 216h (9 days) threshold")
        
        # Mesh packet delivery ratio: >95%
        if metrics.mesh_packet_loss > 0.05:
            failures.append(f"Mesh packet loss {metrics.mesh_packet_loss:.1%} > 5% threshold")
        
        # Real-time factor: >0.8 (simulation should run at least 80% of real-time)
        if metrics.real_time_factor < 0.8:
            failures.append(f"Real-time factor {metrics.real_time_factor:.2f} < 0.8 threshold")
        
        if failures:
            log.warning("KPI threshold violations:")
            for failure in failures:
                log.warning(f"  - {failure}")
        
        # In CI mode, exit with failure if thresholds are violated
        ci_mode = self.get_parameter_or('ci_mode', False)
        if ci_mode and failures:
            log.error("CI mode: KPI thresholds violated, exiting with failure")
            self.cleanup()
            sys.exit(1)
    
    def get_simulation_summary(self) -> Dict[str, Any]:
        """Get simulation summary for reporting."""
        if not self.metrics_history:
            return {}
        
        latest_metrics = self.metrics_history[-1]
        
        # Calculate averages over the simulation
        avg_latency = sum(m.total_pipeline_latency_ms for m in self.metrics_history) / len(self.metrics_history)
        avg_battery_life = sum(m.projected_battery_hours for m in self.metrics_history) / len(self.metrics_history)
        avg_rtf = sum(m.real_time_factor for m in self.metrics_history) / len(self.metrics_history)
        
        return {
            'simulation_duration_s': self.simulation_time,
            'real_duration_s': time.time() - self.start_time,
            'avg_real_time_factor': avg_rtf,
            'avg_pipeline_latency_ms': avg_latency,
            'avg_battery_life_hours': avg_battery_life,
            'total_actor_detections': sum(len(actor.detected_by_nodes) for actor in self.actor_states.values()),
            'active_sensor_nodes': len(self.sensor_nodes),
            'kpi_pass': avg_latency < 300 and avg_battery_life > 216,
            'final_metrics': asdict(latest_metrics)
        }
    
    def cleanup(self):
        """Clean up resources."""
        log.info("Cleaning up Time Sync Bridge...")
        
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        if self.influx_client:
            self.influx_client.close()


def main():
    """Main entry point."""
    rclpy.init()
    
    bridge = OlympusTimeSyncBridge()
    
    # Set up signal handlers
    def signal_handler(signum, frame):
        log.info(f"Received signal {signum}, shutting down...")
        bridge.cleanup()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        log.info("🕒 Starting Olympus Time Sync Bridge")
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        log.info("Bridge interrupted by user")
    except Exception as e:
        log.error(f"Bridge error: {e}")
    finally:
        bridge.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 