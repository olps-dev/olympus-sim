#!/usr/bin/env python3

import asyncio
import json
import logging
import time
from typing import Dict, List, Any, Optional
import socketio
import requests
from flask import Flask, request
from flask_socketio import SocketIO, emit

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class OlympusBackendBridge:
    def __init__(self):
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'olympus-bridge-secret'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", logger=True, engineio_logger=True)
        
        # Simulation state
        self.simulation_state = {
            'nodes': [],
            'sensors': [],
            'rooms': [],
            'automationRules': [],
            'isRunning': False
        }
        
        # Connect to existing Three.js simulation server
        self.threejs_server_url = 'http://localhost:5555'
        
        self.setup_socket_handlers()
        
    def setup_socket_handlers(self):
        @self.socketio.on('connect')
        def handle_connect():
            logger.info(f'React dashboard connected: {request.sid}')
            emit('simulation_state', self.simulation_state)
            
        @self.socketio.on('disconnect')
        def handle_disconnect():
            logger.info(f'React dashboard disconnected: {request.sid}')
            
        @self.socketio.on('get_simulation_state')
        def handle_get_simulation_state():
            logger.info('Sending current simulation state')
            emit('simulation_state', self.simulation_state)
            
        @self.socketio.on('create_node')
        def handle_create_node(data):
            logger.info(f'Creating node: {data}')
            node = self.create_node(data)
            if node:
                self.simulation_state['nodes'].append(node)
                self.socketio.emit('simulation_state', self.simulation_state)
                
        @self.socketio.on('create_sensor')
        def handle_create_sensor(data):
            logger.info(f'Creating sensor: {data}')
            sensor = self.create_sensor(data)
            if sensor:
                self.simulation_state['sensors'].append(sensor)
                self.socketio.emit('simulation_state', self.simulation_state)
                
        @self.socketio.on('create_room')
        def handle_create_room(data):
            logger.info(f'Creating room: {data}')
            room = self.create_room(data)
            if room:
                self.simulation_state['rooms'].append(room)
                self.socketio.emit('simulation_state', self.simulation_state)
                
        @self.socketio.on('start_simulation')
        def handle_start_simulation():
            logger.info('Starting simulation')
            self.simulation_state['isRunning'] = True
            self.socketio.emit('simulation_state', self.simulation_state)
            self.start_simulation_backend()
            
        @self.socketio.on('stop_simulation')
        def handle_stop_simulation():
            logger.info('Stopping simulation')
            self.simulation_state['isRunning'] = False
            self.socketio.emit('simulation_state', self.simulation_state)
            self.stop_simulation_backend()
            
        @self.socketio.on('update_node')
        def handle_update_node(data):
            logger.info(f'Updating node: {data}')
            self.update_node(data['nodeId'], data['updates'])
            self.socketio.emit('simulation_state', self.simulation_state)
            
        @self.socketio.on('update_sensor')
        def handle_update_sensor(data):
            logger.info(f'Updating sensor: {data}')
            self.update_sensor(data['sensorId'], data['updates'])
            self.socketio.emit('simulation_state', self.simulation_state)
            
        @self.socketio.on('delete_node')
        def handle_delete_node(data):
            logger.info(f'Deleting node: {data}')
            self.delete_node(data['nodeId'])
            self.socketio.emit('simulation_state', self.simulation_state)
            
        @self.socketio.on('delete_sensor')
        def handle_delete_sensor(data):
            logger.info(f'Deleting sensor: {data}')
            self.delete_sensor(data['sensorId'])
            self.socketio.emit('simulation_state', self.simulation_state)
            
        @self.socketio.on('save_node_script')
        def handle_save_node_script(data):
            logger.info(f'Saving script for node: {data["nodeId"]}')
            self.save_node_script(data['nodeId'], data['script'])
            self.socketio.emit('simulation_state', self.simulation_state)
            
        @self.socketio.on('execute_node_script')
        def handle_execute_node_script(data):
            logger.info(f'Executing script for node: {data["nodeId"]}')
            result = self.execute_node_script(data['nodeId'])
            emit('script_execution_result', {
                'nodeId': data['nodeId'],
                'success': result['success'],
                'output': result['output'],
                'error': result.get('error')
            })
            
        @self.socketio.on('stop_node_script')
        def handle_stop_node_script(data):
            logger.info(f'Stopping script for node: {data["nodeId"]}')
            self.stop_node_script(data['nodeId'])
            self.socketio.emit('simulation_state', self.simulation_state)
    
    def create_node(self, config: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Create a new IoT node"""
        node_id = f"{config['type']}-{int(time.time() * 1000)}"
        
        # Default capabilities based on node type
        capabilities = {
            'zeus': {
                'processing': 'high',
                'storage': 'high', 
                'networking': 'ethernet',
                'power': 'mains',
                'ai': True,
                'gpu': 'RTX 4080',
                'ram': '64GB'
            },
            'mid-tier': {
                'processing': 'medium',
                'storage': 'medium',
                'networking': 'wifi', 
                'power': 'mains',
                'ai': False,
                'ram': '8GB'
            },
            'low-power': {
                'processing': 'low',
                'storage': 'low',
                'networking': 'mesh',
                'power': 'battery',
                'ai': False,
                'ram': '512MB'
            }
        }
        
        node = {
            'id': node_id,
            'type': config['type'],
            'name': f"{config['type'].title().replace('-', ' ')} Node",
            'position': config.get('position', {'x': 0, 'y': 1, 'z': 0}),
            'rotation': {'x': 0, 'y': 0, 'z': 0},
            'sensors': [],
            'status': 'online',
            'capabilities': capabilities.get(config['type'], capabilities['low-power']),
            'room': config.get('room', 'default'),
            'script': '',
            'scriptStatus': 'stopped',
            'lastModified': time.time(),
            'openai_api_key': config.get('openai_api_key', ''),  # API key for LLM features
            'voice_enabled': config.get('voice_enabled', False)  # Voice activation support
        }
        
        logger.info(f'Created node: {node}')
        return node
        
    def create_sensor(self, config: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Create a new sensor"""
        sensor_id = f"{config['type']}-{int(time.time() * 1000)}"
        
        # Find parent node
        parent_node = None
        for node in self.simulation_state['nodes']:
            if node['id'] == config['nodeId']:
                parent_node = node
                break
                
        if not parent_node:
            logger.error(f"Parent node {config['nodeId']} not found")
            return None
            
        # Default specifications based on sensor type
        default_specs = {
            'mmwave': {'range': 8, 'fov': 60, 'updateRate': 10, 'accuracy': '±2cm', 'powerConsumption': 3.5},
            'temperature': {'updateRate': 1, 'accuracy': '±0.5°C', 'powerConsumption': 0.1},
            'audio': {'range': 5, 'updateRate': 44100, 'accuracy': '16-bit', 'powerConsumption': 1.8},
            'camera': {'range': 10, 'fov': 90, 'updateRate': 30, 'accuracy': '1080p', 'powerConsumption': 5.2},
            'pir': {'range': 6, 'fov': 110, 'updateRate': 5, 'accuracy': 'Motion detection', 'powerConsumption': 0.05},
            'pressure': {'updateRate': 2, 'accuracy': '±0.1 kPa', 'powerConsumption': 0.3},
            'light': {'updateRate': 2, 'accuracy': '±5 lux', 'powerConsumption': 0.02},
            'humidity': {'updateRate': 1, 'accuracy': '±3% RH', 'powerConsumption': 0.15}
        }
        
        specs = default_specs.get(config['type'], default_specs['temperature'])
        specs.update(config.get('specifications', {}))
        
        sensor = {
            'id': sensor_id,
            'type': config['type'],
            'name': f"{config['type'].title()} Sensor",
            'position': {
                'x': parent_node['position']['x'] + (time.time() % 2 - 1),  # Small random offset
                'y': parent_node['position']['y'] + 0.5,
                'z': parent_node['position']['z'] + (time.time() % 2 - 1)
            },
            'rotation': {'x': 0, 'y': 0, 'z': 0},
            'specifications': specs,
            'status': 'active',
            'nodeId': config['nodeId'],
            'room': parent_node['room']
        }
        
        # Add sensor to parent node
        parent_node['sensors'].append(sensor_id)
        
        logger.info(f'Created sensor: {sensor}')
        return sensor
        
    def create_room(self, config: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Create a new room"""
        room_id = f"room-{int(time.time() * 1000)}"
        
        room = {
            'id': room_id,
            'name': config['name'],
            'dimensions': config.get('dimensions', {'width': 5, 'height': 3, 'depth': 4}),
            'position': config.get('position', {'x': 0, 'y': 0, 'z': 0}),
            'nodes': [],
            'sensors': [],
            'automationRules': []
        }
        
        logger.info(f'Created room: {room}')
        return room
        
    def update_node(self, node_id: str, updates: Dict[str, Any]):
        """Update an existing node"""
        for i, node in enumerate(self.simulation_state['nodes']):
            if node['id'] == node_id:
                self.simulation_state['nodes'][i].update(updates)
                logger.info(f'Updated node {node_id}: {updates}')
                break
                
    def update_sensor(self, sensor_id: str, updates: Dict[str, Any]):
        """Update an existing sensor"""
        for i, sensor in enumerate(self.simulation_state['sensors']):
            if sensor['id'] == sensor_id:
                self.simulation_state['sensors'][i].update(updates)
                logger.info(f'Updated sensor {sensor_id}: {updates}')
                break
                
    def delete_node(self, node_id: str):
        """Delete a node and its sensors"""
        # Remove sensors belonging to this node
        self.simulation_state['sensors'] = [
            s for s in self.simulation_state['sensors'] 
            if s['nodeId'] != node_id
        ]
        
        # Remove the node
        self.simulation_state['nodes'] = [
            n for n in self.simulation_state['nodes'] 
            if n['id'] != node_id
        ]
        
        logger.info(f'Deleted node {node_id} and its sensors')
        
    def delete_sensor(self, sensor_id: str):
        """Delete a sensor"""
        # Remove sensor from node's sensor list
        for node in self.simulation_state['nodes']:
            if sensor_id in node['sensors']:
                node['sensors'].remove(sensor_id)
                
        # Remove the sensor
        self.simulation_state['sensors'] = [
            s for s in self.simulation_state['sensors'] 
            if s['id'] != sensor_id
        ]
        
        logger.info(f'Deleted sensor {sensor_id}')
        
    def save_node_script(self, node_id: str, script: str):
        """Save a Python script to a node"""
        for i, node in enumerate(self.simulation_state['nodes']):
            if node['id'] == node_id:
                self.simulation_state['nodes'][i]['script'] = script
                self.simulation_state['nodes'][i]['scriptStatus'] = 'stopped'
                self.simulation_state['nodes'][i]['lastModified'] = time.time()
                logger.info(f'Saved script for node {node_id}')
                break
                
    def execute_node_script(self, node_id: str) -> Dict[str, Any]:
        """Execute the Python script on a node"""
        import subprocess
        import tempfile
        import os
        import sys
        
        node = None
        for n in self.simulation_state['nodes']:
            if n['id'] == node_id:
                node = n
                break
                
        if not node or 'script' not in node:
            return {'success': False, 'error': 'Node or script not found'}
            
        try:
            # Update node status
            for i, n in enumerate(self.simulation_state['nodes']):
                if n['id'] == node_id:
                    self.simulation_state['nodes'][i]['scriptStatus'] = 'running'
                    break
            
            # Create mock environment for the script
            script_env = self.create_script_environment(node)
            
            # Prepare the script with extended environment for AI/LLM features
            api_key = script_env.get('openai_api_key', 'demo-key-not-set')
            full_script = f"""
import sys
import json
import time
import os
import requests
import paho.mqtt.client as mqtt

# AI/LLM Libraries (install with: pip install openai)
try:
    import openai
    OPENAI_AVAILABLE = True
    openai.api_key = "{api_key}"
except ImportError:
    OPENAI_AVAILABLE = False
    print("Warning: OpenAI library not available. Install with: pip install openai")

# Mock sensor data and node info
sensor_data = {json.dumps(script_env['sensor_data'])}
node_info = {json.dumps(script_env['node_info'])}

# MQTT client setup (for real integration)
mqtt_client = mqtt.Client()
ros2_available = False

def mqtt_publish(topic, data):
    print(f"MQTT Publish -> Topic: {{topic}}, Data: {{data}}")
    # In real implementation, this would publish to actual MQTT broker
    try:
        mqtt_client.connect("localhost", 1883, 60)
        mqtt_client.publish(topic, json.dumps(data))
        mqtt_client.disconnect()
    except:
        pass  # Fallback for demo mode

def ros2_publish(topic, msg_type, data):
    print(f"ROS2 Publish -> Topic: {{topic}}, Type: {{msg_type}}, Data: {{data}}")
    # In real implementation, this would use rclpy to publish
    if ros2_available:
        # TODO: Implement actual ROS2 publishing
        pass

def llm_query(prompt, system_message="You are a helpful IoT assistant."):
    \"\"\"Query OpenAI LLM with sensor context\"\"\"
    if not OPENAI_AVAILABLE:
        return "Error: OpenAI library not available"
    
    try:
        # Add sensor context to the prompt
        context = f"Current sensors: {{sensor_data}}, Node info: {{node_info}}"
        full_prompt = f"Context: {{context}}\\n\\nUser request: {{prompt}}"
        
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {{"role": "system", "content": system_message}},
                {{"role": "user", "content": full_prompt}}
            ],
            max_tokens=200,
            temperature=0.7
        )
        return response.choices[0].message.content.strip()
    except Exception as e:
        return f"LLM Error: {{str(e)}}"

def check_internet():
    \"\"\"Check if internet is available for API calls\"\"\"
    try:
        response = requests.get("https://httpbin.org/ip", timeout=5)
        return response.status_code == 200
    except:
        return False

# Internet connectivity check
INTERNET_AVAILABLE = check_internet()
print(f"Internet available: {{INTERNET_AVAILABLE}}")
print(f"OpenAI available: {{OPENAI_AVAILABLE}}")

# User script starts here:
{node['script']}
"""
            
            # Execute script in temporary file
            with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
                f.write(full_script)
                script_path = f.name
                
            # Run script and capture output (extended timeout for LLM API calls)
            result = subprocess.run(
                [sys.executable, script_path],
                capture_output=True,
                text=True,
                timeout=300  # 5 minute timeout for LLM/API calls
            )
            
            # Clean up
            os.unlink(script_path)
            
            # Update node status
            for i, n in enumerate(self.simulation_state['nodes']):
                if n['id'] == node_id:
                    if result.returncode == 0:
                        self.simulation_state['nodes'][i]['scriptStatus'] = 'running'
                    else:
                        self.simulation_state['nodes'][i]['scriptStatus'] = 'error'
                    break
            
            return {
                'success': result.returncode == 0,
                'output': result.stdout,
                'error': result.stderr if result.returncode != 0 else None
            }
            
        except subprocess.TimeoutExpired:
            # Update node status
            for i, n in enumerate(self.simulation_state['nodes']):
                if n['id'] == node_id:
                    self.simulation_state['nodes'][i]['scriptStatus'] = 'error'
                    break
            return {'success': False, 'error': 'Script execution timed out'}
        except Exception as e:
            # Update node status
            for i, n in enumerate(self.simulation_state['nodes']):
                if n['id'] == node_id:
                    self.simulation_state['nodes'][i]['scriptStatus'] = 'error'
                    break
            return {'success': False, 'error': str(e)}
            
    def stop_node_script(self, node_id: str):
        """Stop the script execution on a node"""
        for i, node in enumerate(self.simulation_state['nodes']):
            if node['id'] == node_id:
                self.simulation_state['nodes'][i]['scriptStatus'] = 'stopped'
                logger.info(f'Stopped script for node {node_id}')
                break
                
    def create_script_environment(self, node: Dict[str, Any]) -> Dict[str, Any]:
        """Create the execution environment for a node script"""
        # Gather sensor data for this node
        sensor_data = {}
        for sensor in self.simulation_state['sensors']:
            if sensor.get('nodeId') == node['id']:
                # Generate current sensor reading
                sensor_data[sensor['id']] = self.generate_sensor_data(sensor)
                
        # Create node info
        node_info = {
            'id': node['id'],
            'type': node['type'],
            'capabilities': node['capabilities'],
            'status': node['status'],
            'sensors': node['sensors']
        }
        
        # API key management (in production, store securely)
        # For demo purposes, use environment variable or node-specific key
        import os
        openai_api_key = node.get('openai_api_key') or os.getenv('OPENAI_API_KEY', 'demo-key-not-set')
        
        return {
            'sensor_data': sensor_data,
            'node_info': node_info,
            'openai_api_key': openai_api_key
        }
        
    def start_simulation_backend(self):
        """Start the backend simulation processes"""
        try:
            # Try to start existing Three.js simulation if available
            response = requests.post(f'{self.threejs_server_url}/start', timeout=5)
            if response.status_code == 200:
                logger.info('Started Three.js simulation backend')
            else:
                logger.warning('Could not start Three.js simulation - running in standalone mode')
        except requests.exceptions.RequestException as e:
            logger.warning(f'Three.js simulation not available: {e} - running in standalone mode')
            
        # Start simulated sensor data generation
        self.start_sensor_data_simulation()
        
    def stop_simulation_backend(self):
        """Stop the backend simulation processes"""
        try:
            response = requests.post(f'{self.threejs_server_url}/stop', timeout=5)
            if response.status_code == 200:
                logger.info('Stopped Three.js simulation backend')
        except requests.exceptions.RequestException as e:
            logger.warning(f'Could not stop Three.js simulation: {e}')
            
    def start_sensor_data_simulation(self):
        """Generate simulated sensor data"""
        def generate_data():
            import threading
            import time
            
            def data_loop():
                while self.simulation_state['isRunning']:
                    for sensor in self.simulation_state['sensors']:
                        if sensor['status'] == 'active':
                            # Generate realistic sensor data based on type
                            data = self.generate_sensor_data(sensor)
                            self.socketio.emit('sensor_data', {
                                'sensorId': sensor['id'],
                                'timestamp': time.time(),
                                'data': data
                            })
                            
                    # Generate system metrics
                    metrics = {
                        'timestamp': time.time(),
                        'nodes': {node['id']: {
                            'cpu': 45 + (time.time() % 20),
                            'memory': 60 + (time.time() % 15),
                            'network': 30 + (time.time() % 25),
                            'storage': 25,
                            'temperature': 45 + (time.time() % 10)
                        } for node in self.simulation_state['nodes']},
                        'sensors': {sensor['id']: {
                            'lastUpdate': time.time(),
                            'dataRate': sensor['specifications']['updateRate'],
                            'errorRate': 0.01
                        } for sensor in self.simulation_state['sensors']},
                        'automation': {
                            'rulesExecuted': len(self.simulation_state['automationRules']),
                            'averageResponseTime': 2.3,
                            'errors': 0
                        }
                    }
                    
                    self.socketio.emit('system_metrics', metrics)
                    time.sleep(1)  # Update every second
                    
            # Run in background thread
            thread = threading.Thread(target=data_loop)
            thread.daemon = True
            thread.start()
            
        generate_data()
        
    def generate_sensor_data(self, sensor: Dict[str, Any]) -> Dict[str, Any]:
        """Generate realistic sensor data based on sensor type"""
        import random
        import math
        
        sensor_type = sensor['type']
        current_time = time.time()
        
        if sensor_type == 'mmwave':
            return {
                'presence': random.choice([True, False]),
                'distance': random.uniform(0.5, sensor['specifications']['range']),
                'angle': random.uniform(-sensor['specifications']['fov']/2, sensor['specifications']['fov']/2),
                'velocity': random.uniform(-2, 2)
            }
        elif sensor_type == 'temperature':
            base_temp = 22.0  # 22°C base
            return {
                'temperature': base_temp + random.uniform(-2, 2) + math.sin(current_time / 3600) * 3
            }
        elif sensor_type == 'audio':
            # Enhanced audio data for voice activation features
            speech_detected = random.choice([True, False])
            base_audio = {
                'volume': random.uniform(30, 80),  # dB
                'frequency': random.uniform(100, 8000),  # Hz
                'speechDetected': speech_detected
            }
            
            # Add voice activation features if speech is detected
            if speech_detected:
                # Simulate voice transcription and commands
                voice_commands = [
                    "turn on the lights",
                    "what's the temperature", 
                    "set timer for 5 minutes",
                    "play music",
                    "lock the doors",
                    "good morning",
                    "turn off all lights",
                    "what time is it"
                ]
                base_audio.update({
                    'transcription': random.choice(voice_commands),
                    'confidence': random.uniform(0.8, 0.99),
                    'wake_word_detected': random.choice([True, False]),
                    'speaker_identified': random.choice(['user1', 'user2', 'unknown']),
                    'language': 'en-US',
                    'voice_command': True
                })
            else:
                base_audio.update({
                    'transcription': '',
                    'confidence': 0.0,
                    'wake_word_detected': False,
                    'speaker_identified': 'unknown',
                    'language': 'en-US', 
                    'voice_command': False
                })
                
            return base_audio
        elif sensor_type == 'camera':
            return {
                'motionDetected': random.choice([True, False]),
                'objectsDetected': random.randint(0, 3),
                'lightLevel': random.uniform(50, 1000)  # lux
            }
        elif sensor_type == 'pir':
            return {
                'motionDetected': random.choice([True, False]),
                'lastMotion': current_time - random.uniform(0, 300)
            }
        elif sensor_type == 'pressure':
            return {
                'pressure': 1013.25 + random.uniform(-5, 5)  # hPa
            }
        elif sensor_type == 'light':
            return {
                'illuminance': random.uniform(10, 1000),  # lux
                'colorTemperature': random.uniform(2700, 6500)  # K
            }
        elif sensor_type == 'humidity':
            return {
                'humidity': 45 + random.uniform(-10, 15),  # % RH
                'dewPoint': random.uniform(10, 25)  # °C
            }
        else:
            return {'value': random.uniform(0, 100)}

    def run(self, host='0.0.0.0', port=3001, debug=False):
        """Run the bridge server"""
        logger.info(f'Starting Olympus Backend Bridge on {host}:{port}')
        self.socketio.run(self.app, host=host, port=port, debug=debug, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    bridge = OlympusBackendBridge()
    bridge.run(debug=True)