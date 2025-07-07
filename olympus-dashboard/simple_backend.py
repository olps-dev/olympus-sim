#!/usr/bin/env python3

import json
import logging
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
import urllib.parse

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class OlympusSimpleBackend:
    def __init__(self):
        self.simulation_state = {
            'nodes': [],
            'sensors': [],
            'rooms': [],
            'automationRules': [],
            'isRunning': False
        }
        
    def create_node(self, config):
        """Create a new IoT node"""
        node_id = f"{config['type']}-{int(time.time() * 1000)}"
        
        capabilities = {
            'zeus': {'processing': 'high', 'storage': 'high', 'networking': 'ethernet', 'power': 'mains', 'ai': True},
            'mid-tier': {'processing': 'medium', 'storage': 'medium', 'networking': 'wifi', 'power': 'mains', 'ai': False},
            'low-power': {'processing': 'low', 'storage': 'low', 'networking': 'mesh', 'power': 'battery', 'ai': False}
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
            'room': config.get('room', 'default')
        }
        
        self.simulation_state['nodes'].append(node)
        logger.info(f'Created node: {node_id}')
        return node
        
    def create_sensor(self, config):
        """Create a new sensor"""
        sensor_id = f"{config['type']}-{int(time.time() * 1000)}"
        
        # Find parent node
        parent_node = None
        for node in self.simulation_state['nodes']:
            if node['id'] == config['nodeId']:
                parent_node = node
                break
                
        if not parent_node:
            return None
            
        sensor = {
            'id': sensor_id,
            'type': config['type'],
            'name': f"{config['type'].title()} Sensor",
            'position': {
                'x': parent_node['position']['x'] + 0.5,
                'y': parent_node['position']['y'] + 0.5,
                'z': parent_node['position']['z'] + 0.5
            },
            'rotation': {'x': 0, 'y': 0, 'z': 0},
            'specifications': config.get('specifications', {'updateRate': 10, 'accuracy': 'Standard'}),
            'status': 'active',
            'nodeId': config['nodeId'],
            'room': parent_node['room']
        }
        
        self.simulation_state['sensors'].append(sensor)
        parent_node['sensors'].append(sensor_id)
        logger.info(f'Created sensor: {sensor_id}')
        return sensor
        
    def create_room(self, config):
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
        
        self.simulation_state['rooms'].append(room)
        logger.info(f'Created room: {room_id}')
        return room

class OlympusHTTPHandler(BaseHTTPRequestHandler):
    def __init__(self, backend, *args, **kwargs):
        self.backend = backend
        super().__init__(*args, **kwargs)
        
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
        
    def do_GET(self):
        if self.path == '/simulation_state':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(self.backend.simulation_state).encode())
        elif self.path == '/health':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({'status': 'healthy'}).encode())
        else:
            self.send_response(404)
            self.end_headers()
            
    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        
        try:
            data = json.loads(post_data.decode())
        except json.JSONDecodeError:
            self.send_response(400)
            self.end_headers()
            return
            
        response_data = {'success': False}
        
        if self.path == '/create_node':
            result = self.backend.create_node(data)
            if result:
                response_data = {'success': True, 'node': result, 'state': self.backend.simulation_state}
        elif self.path == '/create_sensor':
            result = self.backend.create_sensor(data)
            if result:
                response_data = {'success': True, 'sensor': result, 'state': self.backend.simulation_state}
        elif self.path == '/create_room':
            result = self.backend.create_room(data)
            if result:
                response_data = {'success': True, 'room': result, 'state': self.backend.simulation_state}
        elif self.path == '/start_simulation':
            self.backend.simulation_state['isRunning'] = True
            response_data = {'success': True, 'state': self.backend.simulation_state}
        elif self.path == '/stop_simulation':
            self.backend.simulation_state['isRunning'] = False
            response_data = {'success': True, 'state': self.backend.simulation_state}
            
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(response_data).encode())

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""
    
def create_handler(backend):
    def handler(*args, **kwargs):
        return OlympusHTTPHandler(backend, *args, **kwargs)
    return handler

if __name__ == '__main__':
    backend = OlympusSimpleBackend()
    handler = create_handler(backend)
    
    server = ThreadedHTTPServer(('0.0.0.0', 3001), handler)
    logger.info('Starting Olympus Simple Backend on port 3001')
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info('Shutting down server...')
        server.shutdown()
        server.server_close()