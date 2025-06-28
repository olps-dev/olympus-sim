#!/usr/bin/env python3
"""
Olympus Simulation Dashboard Backend
Real-time monitoring dashboard for IoT automation system

Features:
- Live latency metrics streaming
- Battery level monitoring  
- Network condition visualization
- Automation event tracking
- Performance analytics
"""

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import json
import time
import threading
import pandas as pd
import numpy as np
from pathlib import Path
import logging
import sys
import os
from datetime import datetime, timedelta
from typing import Dict, List, Any, Optional

# Add project paths
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'metrics'))
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'network'))

try:
    import paho.mqtt.client as mqtt
    from latency_battery_tracker import LatencyBatteryTracker
    from config import get_network_config_manager
except ImportError as e:
    print(f"Warning: Could not import dependencies: {e}")
    # Create mock function for network config
    def get_network_config_manager():
        class MockManager:
            def get_statistics_summary(self):
                return {'enabled': False, 'default_condition': 'unknown', 'environment_based': False}
        return MockManager()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('olympus_dashboard')

app = Flask(__name__)
app.config['SECRET_KEY'] = 'olympus_dashboard_secret_2024'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

class DashboardDataManager:
    """Manages real-time data collection and processing for dashboard"""
    
    def __init__(self):
        self.metrics_dir = Path("/app/metrics")
        self.data_buffer = {
            'latency_history': [],
            'battery_levels': {},
            'sensor_status': {},
            'automation_events': [],
            'network_stats': {},
            'system_status': 'starting'
        }
        
        # MQTT client for live data
        self.mqtt_client = None
        self.mqtt_connected = False
        
        # Data update thread
        self.update_thread = None
        self.running = False
        
        # Performance metrics
        self.performance_window = timedelta(minutes=5)  # 5-minute rolling window
        
    def start(self):
        """Start data collection and processing"""
        logger.info("Starting dashboard data manager")
        self.running = True
        
        # Connect to MQTT for live updates
        self.connect_mqtt()
        
        # Start background data update thread
        logger.info("Dashboard: Creating update thread")
        self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
        logger.info("Dashboard: Starting update thread")
        self.update_thread.start()
        logger.info("Dashboard: Update thread started")
        
        self.data_buffer['system_status'] = 'running'
        
    def stop(self):
        """Stop data collection"""
        logger.info("Stopping dashboard data manager")
        self.running = False
        
        if self.mqtt_client:
            self.mqtt_client.disconnect()
        
        self.data_buffer['system_status'] = 'stopped'
        
    def connect_mqtt(self):
        """Connect to MQTT broker for live data feeds"""
        try:
            self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv311)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            mqtt_broker = os.getenv('MQTT_BROKER', 'localhost')
            mqtt_port = int(os.getenv('MQTT_PORT', '1883'))
            logger.info(f"Attempting to connect to MQTT broker: {mqtt_broker}:{mqtt_port}")
            
            result = self.mqtt_client.connect(mqtt_broker, mqtt_port, 60)
            logger.info(f"MQTT connect result: {result}")
            self.mqtt_client.loop_start()
            logger.info("MQTT client loop started")
            
        except Exception as e:
            logger.error(f"Failed to connect to MQTT: {e}")
            self.mqtt_connected = False
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            logger.info("Connected to MQTT broker")
            self.mqtt_connected = True
            
            # Subscribe to relevant topics
            topics = [
                "sensor/+/presence",
                "sensor/+/status", 
                "actuators/+/+",
                "automation/events"
            ]
            
            for topic in topics:
                client.subscribe(topic)
                logger.info(f"Subscribed to {topic}")
                
        else:
            logger.error(f"Failed to connect to MQTT: {rc}")
            self.mqtt_connected = False
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        logger.warning("Disconnected from MQTT broker")
        self.mqtt_connected = False
    
    def on_mqtt_message(self, client, userdata, msg):
        """Process incoming MQTT messages"""
        try:
            topic = msg.topic
            payload = msg.payload.decode()
            timestamp = time.time()
            
            # Parse different message types
            if "presence" in topic:
                self.process_presence_message(topic, payload, timestamp)
            elif "status" in topic:
                self.process_status_message(topic, payload, timestamp)
            elif "actuators" in topic:
                self.process_actuator_message(topic, payload, timestamp)
            elif "automation/events" in topic:
                self.process_automation_event(topic, payload, timestamp)
                
        except Exception as e:
            logger.error(f"Error processing MQTT message: {e}")
    
    def process_presence_message(self, topic, payload, timestamp):
        """Process sensor presence messages"""
        try:
            data = json.loads(payload)
            sensor_id = data.get('sensor_id', 'unknown')
            
            # Update sensor status
            self.data_buffer['sensor_status'][sensor_id] = {
                'human_present': data.get('human_present', False),
                'battery_level_mah': data.get('battery_level_mah', 0),
                'last_update': timestamp,
                'analysis': data.get('analysis', ''),
                'event_id': data.get('event_id')
            }
            
            # Update battery levels
            self.data_buffer['battery_levels'][sensor_id] = {
                'level_mah': data.get('battery_level_mah', 0),
                'timestamp': timestamp
            }
            
            # Emit real-time update
            socketio.emit('sensor_update', {
                'sensor_id': sensor_id,
                'data': self.data_buffer['sensor_status'][sensor_id]
            })
            
        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON in presence message: {payload}")
    
    def process_automation_event(self, topic, payload, timestamp):
        """Process automation events"""
        try:
            data = json.loads(payload)
            
            # Add to events buffer
            event = {
                'timestamp': timestamp,
                'event_type': data.get('event_type'),
                'actuator_id': data.get('actuator_id'),
                'action': data.get('action'),
                'triggered_by': data.get('triggered_by', []),
                'latency_ms': data.get('automation_latency_ms', 0)
            }
            
            self.data_buffer['automation_events'].append(event)
            
            # Keep only recent events (last 100)
            if len(self.data_buffer['automation_events']) > 100:
                self.data_buffer['automation_events'] = self.data_buffer['automation_events'][-100:]
            
            # Emit real-time update
            socketio.emit('automation_event', event)
            
        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON in automation event: {payload}")
    
    def process_actuator_message(self, topic, payload, timestamp):
        """Process actuator control messages"""
        try:
            # Extract actuator info from topic
            topic_parts = topic.split('/')
            if len(topic_parts) >= 3:
                actuator_id = topic_parts[1]
                action = topic_parts[2]
                
                # Emit actuator update
                socketio.emit('actuator_update', {
                    'actuator_id': actuator_id,
                    'action': action,
                    'timestamp': timestamp
                })
                
        except Exception as e:
            logger.error(f"Error processing actuator message: {e}")
    
    def update_loop(self):
        """Background thread for periodic data updates"""
        logger.info("Dashboard: Starting update loop thread")
        update_count = 0
        while self.running:
            try:
                update_count += 1
                if update_count % 5 == 0:  # Log every 5th update (every 10 seconds)
                    logger.info(f"Dashboard: Update loop iteration {update_count}")
                
                # Update latency metrics from CSV
                self.update_latency_metrics()
                
                # Update network statistics
                self.update_network_stats()
                
                # Emit periodic dashboard update
                socketio.emit('dashboard_update', self.get_dashboard_summary())
                
                time.sleep(2)  # Update every 2 seconds
                
            except Exception as e:
                logger.error(f"Error in update loop: {e}")
                time.sleep(5)  # Wait longer on error
    
    def update_latency_metrics(self):
        """Update latency metrics from CSV files"""
        try:
            latency_csv = self.metrics_dir / "latency_metrics.csv"
            
            if latency_csv.exists():
                df = pd.read_csv(latency_csv)
                logger.info(f"Dashboard: Read {len(df)} rows from CSV")
                
                # Get recent end-to-end latencies
                e2e_data = df[(df['stage'] == 'lamp_toggled') & 
                             df['cumulative_latency_ms'].notna()]
                
                logger.info(f"Dashboard: Found {len(e2e_data)} end-to-end latency events")
                
                if len(e2e_data) > 0:
                    # Add recent latencies to buffer
                    recent_threshold = time.time() - self.performance_window.total_seconds()
                    recent_data = e2e_data[e2e_data['timestamp'] > recent_threshold]
                    
                    logger.info(f"Dashboard: Found {len(recent_data)} recent events (threshold: {recent_threshold})")
                    
                    for _, row in recent_data.iterrows():
                        latency_point = {
                            'timestamp': row['timestamp'],
                            'latency_ms': row['cumulative_latency_ms'],
                            'sensor_id': row['sensor_id']
                        }
                        
                        # Add if not already in buffer
                        if latency_point not in self.data_buffer['latency_history']:
                            self.data_buffer['latency_history'].append(latency_point)
                    
                    # Keep only recent data
                    recent_threshold = time.time() - self.performance_window.total_seconds()
                    self.data_buffer['latency_history'] = [
                        point for point in self.data_buffer['latency_history']
                        if point['timestamp'] > recent_threshold
                    ]
                    
        except Exception as e:
            logger.error(f"Error updating latency metrics: {e}")
    
    def update_network_stats(self):
        """Update network statistics"""
        try:
            manager = get_network_config_manager()
            config = manager.get_statistics_summary()
            
            self.data_buffer['network_stats'] = {
                'enabled': config.get('enabled', False),
                'condition': config.get('default_condition', 'unknown'),
                'environment_based': config.get('environment_based', False),
                'mqtt_connected': self.mqtt_connected,
                'last_update': time.time()
            }
            
        except Exception as e:
            logger.error(f"Error updating network stats: {e}")
    
    def get_dashboard_summary(self) -> Dict[str, Any]:
        """Get complete dashboard data summary"""
        try:
            # Calculate latency statistics - try direct CSV read if buffer is empty
            latency_stats = {}
            if self.data_buffer['latency_history']:
                latencies = [p['latency_ms'] for p in self.data_buffer['latency_history']]
                latency_stats = {
                    'mean': np.mean(latencies),
                    'p95': np.percentile(latencies, 95),
                    'p99': np.percentile(latencies, 99),
                    'min': np.min(latencies),
                    'max': np.max(latencies),
                    'count': len(latencies)
                }
            else:
                # Fallback: read directly from CSV
                try:
                    latency_csv = self.metrics_dir / "latency_metrics.csv"
                    if latency_csv.exists():
                        df = pd.read_csv(latency_csv)
                        e2e_data = df[(df['stage'] == 'lamp_toggled') & df['cumulative_latency_ms'].notna()]
                        
                        if len(e2e_data) > 0:
                            # Get recent data (last 5 minutes)
                            recent_threshold = time.time() - 300
                            recent_data = e2e_data[e2e_data['timestamp'] > recent_threshold]
                            
                            if len(recent_data) > 0:
                                latencies = recent_data['cumulative_latency_ms'].values
                                latency_stats = {
                                    'mean': float(np.mean(latencies)),
                                    'p95': float(np.percentile(latencies, 95)),
                                    'p99': float(np.percentile(latencies, 99)),
                                    'min': float(np.min(latencies)),
                                    'max': float(np.max(latencies)),
                                    'count': int(len(latencies))
                                }
                except Exception as e:
                    logger.error(f"Error reading CSV for latency stats: {e}")
            
            # Battery summary
            battery_summary = {}
            for sensor_id, battery_data in self.data_buffer['battery_levels'].items():
                battery_summary[sensor_id] = {
                    'level_mah': battery_data['level_mah'],
                    'percentage': (battery_data['level_mah'] / 3000.0) * 100,  # Assume 3000mAh max
                    'status': 'low' if battery_data['level_mah'] < 300 else 'normal'
                }
            
            # Recent automation events
            recent_events = self.data_buffer['automation_events'][-10:]  # Last 10 events
            
            return {
                'timestamp': time.time(),
                'system_status': self.data_buffer['system_status'],
                'latency_stats': latency_stats,
                'battery_summary': battery_summary,
                'sensor_status': self.data_buffer['sensor_status'],
                'recent_events': recent_events,
                'network_stats': self.data_buffer['network_stats'],
                'performance_window_minutes': self.performance_window.total_seconds() / 60
            }
            
        except Exception as e:
            logger.error(f"Error generating dashboard summary: {e}")
            return {'error': str(e)}

# Global data manager instance
data_manager = DashboardDataManager()

@app.route('/')
def dashboard():
    """Main dashboard page"""
    return render_template('dashboard.html')

@app.route('/api/status')
def api_status():
    """API endpoint for system status"""
    # Get basic summary from data manager
    summary = data_manager.get_dashboard_summary()
    
    # Use direct metrics reader for real data
    try:
        from direct_metrics import get_real_latency_stats, get_recent_automation_events
        
        # Get real latency statistics
        latency_stats = get_real_latency_stats()
        
        if latency_stats:
            summary['latency_stats'] = latency_stats
            logger.info(f"Dashboard: Using real latency data - {latency_stats['count']} events")
        else:
            summary['latency_stats'] = {}
            logger.warning("Dashboard: No real latency data available")
            
        # Get recent automation events
        recent_events = get_recent_automation_events()
        if recent_events:
            summary['recent_events'] = recent_events
            logger.info(f"Dashboard: Added {len(recent_events)} recent automation events")
            
    except Exception as e:
        logger.error(f"Error getting real metrics: {e}")
        summary['latency_stats'] = {}
    
    return jsonify(summary)

@app.route('/api/latency/history')
def api_latency_history():
    """API endpoint for latency history"""
    return jsonify({
        'latency_history': data_manager.data_buffer['latency_history']
    })

@app.route('/api/metrics/live')
def api_metrics_live():
    """Direct CSV reading endpoint for real metrics"""
    try:
        import pandas as pd
        import numpy as np
        from pathlib import Path
        
        latency_csv = Path("/app/metrics/latency_metrics.csv")
        
        if not latency_csv.exists():
            return jsonify({'error': 'CSV not found', 'latency_stats': {}})
            
        df = pd.read_csv(latency_csv)
        
        # Get lamp_toggled events with valid latency data
        lamp_events = df[
            (df['stage'] == 'lamp_toggled') & 
            df['cumulative_latency_ms'].notna() &
            (df['cumulative_latency_ms'] > 0)
        ]
        
        if len(lamp_events) == 0:
            return jsonify({'error': 'No lamp events', 'latency_stats': {}})
            
        # Use recent events (last 10 minutes) or latest 20 events
        recent_threshold = time.time() - 600
        recent_events = lamp_events[lamp_events['timestamp'] > recent_threshold]
        
        if len(recent_events) >= 3:
            events_to_use = recent_events
            data_source = "recent"
        else:
            events_to_use = lamp_events.tail(20)
            data_source = "historical"
            
        latencies = events_to_use['cumulative_latency_ms'].values
        
        if len(latencies) == 0:
            return jsonify({'error': 'No valid latencies', 'latency_stats': {}})
            
        stats = {
            'mean': float(np.mean(latencies)),
            'p95': float(np.percentile(latencies, 95)),
            'p99': float(np.percentile(latencies, 99)),
            'min': float(np.min(latencies)),
            'max': float(np.max(latencies)),
            'count': int(len(latencies)),
            'data_source': data_source,
            'total_events': int(len(lamp_events))
        }
        
        # Get recent events for display
        recent_automation_events = []
        for _, row in events_to_use.tail(10).iterrows():
            recent_automation_events.append({
                'timestamp': row['timestamp'],
                'sensor_id': row['sensor_id'],
                'event_id': row['event_id'],
                'latency_ms': row['cumulative_latency_ms'],
                'action': 'on'
            })
            
        return jsonify({
            'latency_stats': stats,
            'recent_events': recent_automation_events,
            'csv_info': {
                'total_rows': len(df),
                'lamp_events': len(lamp_events)
            }
        })
        
    except Exception as e:
        return jsonify({'error': str(e), 'latency_stats': {}})

@app.route('/api/latency/live')
def api_latency_live():
    """API endpoint for live latency data directly from CSV"""
    try:
        import pandas as pd
        import numpy as np
        from pathlib import Path
        import time
        
        # Read CSV directly
        latency_csv = Path("/app/metrics/latency_metrics.csv")
        if not latency_csv.exists():
            return jsonify({'error': 'CSV file not found', 'latency_stats': {}, 'recent_events': []})
        
        df = pd.read_csv(latency_csv)
        
        # Get end-to-end latencies
        e2e_data = df[(df['stage'] == 'lamp_toggled') & df['cumulative_latency_ms'].notna()]
        
        # Get recent data (last 5 minutes)
        recent_threshold = time.time() - 300
        recent_data = e2e_data[e2e_data['timestamp'] > recent_threshold]
        
        latency_stats = {}
        recent_events = []
        
        if len(recent_data) > 0:
            latencies = recent_data['cumulative_latency_ms'].values
            latency_stats = {
                'mean': float(np.mean(latencies)),
                'p95': float(np.percentile(latencies, 95)),
                'p99': float(np.percentile(latencies, 99)),
                'min': float(np.min(latencies)),
                'max': float(np.max(latencies)),
                'count': int(len(latencies))
            }
            
            # Convert recent events to list
            for _, row in recent_data.tail(10).iterrows():
                recent_events.append({
                    'timestamp': row['timestamp'],
                    'sensor_id': row['sensor_id'],
                    'latency_ms': row['cumulative_latency_ms']
                })
        
        return jsonify({
            'latency_stats': latency_stats,
            'recent_events': recent_events,
            'total_events': int(len(e2e_data)),
            'csv_rows': int(len(df))
        })
        
    except Exception as e:
        return jsonify({'error': str(e), 'latency_stats': {}, 'recent_events': []})

@app.route('/api/latency/direct')
def api_latency_direct():
    """Direct CSV read endpoint"""
    try:
        import pandas as pd
        import numpy as np
        from pathlib import Path
        import time as time_module
        
        # Read CSV directly
        latency_csv = Path("/app/metrics/latency_metrics.csv")
        if not latency_csv.exists():
            return jsonify({'error': 'CSV not found'})
        
        df = pd.read_csv(latency_csv)
        e2e_data = df[(df['stage'] == 'lamp_toggled') & df['cumulative_latency_ms'].notna()]
        
        if len(e2e_data) == 0:
            return jsonify({'error': 'No lamp_toggled events'})
        
        # Get recent data (last 5 minutes)
        recent_threshold = time_module.time() - 300
        recent_data = e2e_data[e2e_data['timestamp'] > recent_threshold]
        
        if len(recent_data) == 0:
            return jsonify({'error': f'No recent events. Latest: {e2e_data["timestamp"].max()}, Current: {time_module.time()}'})
        
        latencies = recent_data['cumulative_latency_ms'].values
        return jsonify({
            'success': True,
            'latency_stats': {
                'mean': float(np.mean(latencies)),
                'p95': float(np.percentile(latencies, 95)),
                'count': int(len(latencies)),
                'min': float(np.min(latencies)),
                'max': float(np.max(latencies))
            },
            'csv_rows': int(len(df)),
            'total_events': int(len(e2e_data)),
            'recent_events': int(len(recent_data))
        })
    except Exception as e:
        return jsonify({'error': str(e)})

@app.route('/api/sensors')
def api_sensors():
    """API endpoint for sensor status"""
    return jsonify({
        'sensors': data_manager.data_buffer['sensor_status'],
        'battery_levels': data_manager.data_buffer['battery_levels']
    })

@app.route('/api/automation/events')
def api_automation_events():
    """API endpoint for automation events"""
    return jsonify({
        'events': data_manager.data_buffer['automation_events']
    })

@app.route('/api/network')
def api_network():
    """API endpoint for network statistics"""
    return jsonify(data_manager.data_buffer['network_stats'])

@socketio.on('connect')
def on_connect():
    """WebSocket connection handler"""
    logger.info(f"Client connected: {request.sid}")
    emit('connected', {'status': 'connected'})
    
    # Send initial data
    emit('dashboard_update', data_manager.get_dashboard_summary())

@socketio.on('disconnect')
def on_disconnect():
    """WebSocket disconnection handler"""
    logger.info(f"Client disconnected: {request.sid}")

@socketio.on('request_update')
def on_request_update():
    """Manual update request from client"""
    emit('dashboard_update', data_manager.get_dashboard_summary())

def create_app():
    """Application factory"""
    # Start data manager
    data_manager.start()
    
    return app

if __name__ == '__main__':
    logger.info("Starting Olympus Dashboard")
    
    try:
        # Start data collection
        data_manager.start()
        
        # Run Flask app with SocketIO
        socketio.run(app, 
                    host='0.0.0.0', 
                    port=5000, 
                    debug=False,
                    allow_unsafe_werkzeug=True)
                    
    except KeyboardInterrupt:
        logger.info("Shutting down dashboard")
        data_manager.stop()
    except Exception as e:
        logger.error(f"Dashboard error: {e}")
        data_manager.stop()
        sys.exit(1)