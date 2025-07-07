#!/usr/bin/env python3
"""
Three.js Simulation MQTT Bridge
Receives sensor data from browser-based Three.js simulation
and publishes to MQTT topics for automation integration
"""

import json
import time
import logging
import asyncio
import websockets
import paho.mqtt.client as mqtt
from flask import Flask, request, jsonify, render_template_string
from flask_cors import CORS
from threading import Thread
import signal
import sys

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('threejs_mqtt_bridge')

class ThreeJSMqttBridge:
    def __init__(self, mqtt_broker='localhost', mqtt_port=1883, http_port=5555, ws_port=9001):
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.http_port = http_port
        self.ws_port = ws_port
        
        # MQTT client
        self.mqtt_client = mqtt.Client(client_id=f"threejs_bridge_{int(time.time())}")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # Flask app for HTTP interface
        self.app = Flask(__name__)
        CORS(self.app)
        self.setup_routes()
        
        # WebSocket server for real-time communication
        self.ws_clients = set()
        
        # Statistics
        self.stats = {
            'messages_received': 0,
            'mqtt_published': 0,
            'start_time': time.time()
        }
        
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template_string('''
<!DOCTYPE html>
<html>
<head>
    <title>Olympus Three.js Simulation</title>
    <style>
        body { margin: 0; font-family: Arial, sans-serif; background: #111; color: #fff; }
        #container { width: 100vw; height: 100vh; position: relative; }
        #info { position: absolute; top: 10px; left: 10px; z-index: 100; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 5px; }
        #controls { position: absolute; top: 10px; right: 10px; z-index: 100; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 5px; }
        button { margin: 5px; padding: 8px 12px; }
        .sensor-info { margin: 5px 0; font-size: 12px; }
    </style>
</head>
<body>
    <div id="info">
        <h3>Olympus Simulation</h3>
        <div>Status: <span id="status">Initializing...</span></div>
        <div>Sensors: <span id="sensor-count">0</span></div>
        <div>MQTT: <span id="mqtt-status">Disconnected</span></div>
        <div id="sensor-data"></div>
    </div>
    <div id="controls">
        <button onclick="addSensor()">Add Sensor</button>
        <button onclick="toggleSimulation()">Start/Stop</button>
        <button onclick="resetScene()">Reset Scene</button>
        <div>
            <label>Update Rate: <input type="range" id="updateRate" min="1" max="30" value="10" onchange="updateRate(this.value)"> <span id="rateValue">10</span> Hz</label>
        </div>
    </div>
    <div id="container"></div>

    <script type="module">
        import { OlympusSimulation } from './static/simulation_core.js';
        
        let simulation;
        let isRunning = false;
        
        function init() {
            const container = document.getElementById('container');
            simulation = new OlympusSimulation(container, {
                mqttBridgeUrl: window.location.origin,
                debug: true,
                sensorUpdateRate: 10
            });
            
            // Add default sensors
            simulation.addSensor('mmwave1', 
                new THREE.Vector3(-5, 1, 0), 
                new THREE.Euler(0, 0, 0)
            );
            
            simulation.addSensor('mmwave2', 
                new THREE.Vector3(5, 1, 0), 
                new THREE.Euler(0, Math.PI, 0)
            );
            
            // Listen for sensor data events
            window.addEventListener('olympus-sensor-data', (event) => {
                updateSensorDisplay(event.detail.sensorId, event.detail.data);
            });
            
            document.getElementById('status').textContent = 'Running';
            isRunning = true;
        }
        
        function updateSensorDisplay(sensorId, data) {
            const sensorData = document.getElementById('sensor-data');
            const sensorDiv = document.getElementById(`sensor-${sensorId}`) || document.createElement('div');
            sensorDiv.id = `sensor-${sensorId}`;
            sensorDiv.className = 'sensor-info';
            sensorDiv.innerHTML = `
                <strong>${sensorId}:</strong> ${data.num_points} points, 
                ${data.human_present ? 'Human detected' : 'No detection'}
                ${data.centroid_position ? ` at ${data.centroid_position.distance.toFixed(2)}m` : ''}
            `;
            
            if (!document.getElementById(`sensor-${sensorId}`)) {
                sensorData.appendChild(sensorDiv);
            }
            
            document.getElementById('sensor-count').textContent = simulation.sensors.size;
        }
        
        window.addSensor = function() {
            const sensorCount = simulation.sensors.size;
            const angle = (sensorCount * Math.PI * 2) / 8;
            const radius = 6;
            simulation.addSensor(
                `mmwave${sensorCount + 1}`,
                new THREE.Vector3(Math.cos(angle) * radius, 1, Math.sin(angle) * radius),
                new THREE.Euler(0, -angle, 0)
            );
        }
        
        window.toggleSimulation = function() {
            // Implementation depends on simulation control methods
            isRunning = !isRunning;
            document.getElementById('status').textContent = isRunning ? 'Running' : 'Paused';
        }
        
        window.resetScene = function() {
            if (simulation) {
                simulation.destroy();
            }
            init();
        }
        
        window.updateRate = function(value) {
            document.getElementById('rateValue').textContent = value;
            if (simulation) {
                simulation.options.sensorUpdateRate = parseInt(value);
            }
        }
        
        // Initialize when page loads
        init();
    </script>
</body>
</html>
            ''')
        
        @self.app.route('/sensor/<sensor_id>/presence', methods=['POST'])
        def sensor_presence(sensor_id):
            try:
                data = request.get_json()
                if not data:
                    return jsonify({'error': 'No data provided'}), 400
                
                # Publish to MQTT
                topic = f"sensor/{sensor_id}/presence"
                self.mqtt_client.publish(topic, json.dumps(data), qos=0)
                
                # Also publish human_present topic
                human_topic = f"sensor/{sensor_id}/human_present"
                self.mqtt_client.publish(human_topic, json.dumps({
                    'human_present': data.get('human_present', False),
                    'timestamp': data.get('timestamp', time.time())
                }), qos=0)
                
                # Publish status topic
                status_topic = f"sensor/{sensor_id}/status"
                self.mqtt_client.publish(status_topic, json.dumps({
                    'sensor_id': sensor_id,
                    'status': 'online',
                    'bridge_node': 'threejs_mqtt_bridge',
                    'timestamp': time.time()
                }), qos=0)
                
                self.stats['messages_received'] += 1
                self.stats['mqtt_published'] += 3  # 3 topics published
                
                logger.info(f"[{sensor_id}] Published sensor data: {data.get('num_points', 0)} points")
                
                return jsonify({'status': 'success', 'sensor_id': sensor_id})
                
            except Exception as e:
                logger.error(f"Error processing sensor data: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/static/simulation_core.js')
        def serve_simulation_core():
            # Serve the simulation core JS file
            try:
                with open('/home/aliza/olympus-sim/sim/threejs/engine/simulation_core.js', 'r') as f:
                    content = f.read()
                return content, 200, {'Content-Type': 'application/javascript'}
            except FileNotFoundError:
                return "// Simulation core not found", 404, {'Content-Type': 'application/javascript'}
        
        @self.app.route('/status')
        def status():
            runtime = time.time() - self.stats['start_time']
            return jsonify({
                'status': 'running',
                'runtime_seconds': runtime,
                'messages_received': self.stats['messages_received'],
                'mqtt_published': self.stats['mqtt_published'],
                'mqtt_connected': self.mqtt_client.is_connected(),
                'active_sensors': []  # TODO: Track active sensors
            })
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            logger.info(f"Connected to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
        else:
            logger.error(f"Failed to connect to MQTT broker: {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        logger.warning(f"Disconnected from MQTT broker (rc={rc})")
    
    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections for real-time communication"""
        self.ws_clients.add(websocket)
        logger.info(f"WebSocket client connected. Total clients: {len(self.ws_clients)}")
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.handle_websocket_message(data, websocket)
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({'error': 'Invalid JSON'}))
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.ws_clients.remove(websocket)
            logger.info(f"WebSocket client disconnected. Total clients: {len(self.ws_clients)}")
    
    async def handle_websocket_message(self, data, websocket):
        """Handle incoming WebSocket messages"""
        message_type = data.get('type')
        
        if message_type == 'sensor_data':
            # Forward sensor data to MQTT
            sensor_id = data.get('sensor_id')
            sensor_data = data.get('data')
            
            if sensor_id and sensor_data:
                topic = f"sensor/{sensor_id}/presence"
                self.mqtt_client.publish(topic, json.dumps(sensor_data), qos=0)
                
                await websocket.send(json.dumps({
                    'type': 'ack',
                    'sensor_id': sensor_id,
                    'status': 'published'
                }))
        
        elif message_type == 'ping':
            await websocket.send(json.dumps({'type': 'pong'}))
    
    async def broadcast_to_websockets(self, message):
        """Broadcast message to all connected WebSocket clients"""
        if self.ws_clients:
            disconnected = set()
            for client in self.ws_clients:
                try:
                    await client.send(json.dumps(message))
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)
            
            # Remove disconnected clients
            self.ws_clients -= disconnected
    
    def start_websocket_server(self):
        """Start WebSocket server in a separate thread"""
        async def run_server():
            logger.info(f"Starting WebSocket server on port {self.ws_port}")
            await websockets.serve(self.websocket_handler, "localhost", self.ws_port)
        
        def run_in_thread():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(run_server())
            loop.run_forever()
        
        ws_thread = Thread(target=run_in_thread, daemon=True)
        ws_thread.start()
    
    def run(self):
        logger.info("Starting Three.js MQTT Bridge")
        logger.info(f"HTTP server on port {self.http_port}")
        logger.info(f"WebSocket server on port {self.ws_port}")
        logger.info(f"MQTT broker: {self.mqtt_broker}:{self.mqtt_port}")
        
        # Connect to MQTT
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            logger.error(f"Failed to connect to MQTT: {e}")
        
        # Start WebSocket server
        self.start_websocket_server()
        
        # Start Flask app
        self.app.run(host='0.0.0.0', port=self.http_port, debug=False)
    
    def stop(self):
        logger.info("Stopping Three.js MQTT Bridge")
        if self.mqtt_client.is_connected():
            self.mqtt_client.disconnect()
            self.mqtt_client.loop_stop()

def main():
    bridge = ThreeJSMqttBridge()
    
    # Handle graceful shutdown
    def signal_handler(sig, frame):
        logger.info("Received shutdown signal")
        bridge.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        bridge.run()
    except KeyboardInterrupt:
        bridge.stop()

if __name__ == "__main__":
    main()