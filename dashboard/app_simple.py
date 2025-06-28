#!/usr/bin/env python3
"""
Simplified Olympus Dashboard - Real-time MQTT data only
"""

from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit
import os
import time
import threading
import logging
from live_data_manager import LiveDataManager

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('olympus_dashboard')

app = Flask(__name__)
app.config['SECRET_KEY'] = 'olympus_live_2024'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global data manager
data_manager = LiveDataManager(
    mqtt_broker=os.getenv('MQTT_BROKER', 'mosquitto'),
    mqtt_port=int(os.getenv('MQTT_PORT', '1883'))
)

def broadcast_updates():
    """Background thread to broadcast updates via WebSocket"""
    while True:
        try:
            # Get fresh data
            dashboard_data = data_manager.get_dashboard_data()
            
            # Broadcast to all connected clients
            socketio.emit('dashboard_update', dashboard_data)
            
            time.sleep(1)  # Update every second
            
        except Exception as e:
            logger.error(f"Error in broadcast thread: {e}")
            time.sleep(5)

@app.route('/')
def dashboard():
    """Main dashboard page"""
    return '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Olympus Live Dashboard</title>
    <script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>
    <script type="importmap">
        {
            "imports": {
                "three": "https://unpkg.com/three@0.156.1/build/three.module.js",
                "three/addons/": "https://unpkg.com/three@0.156.1/examples/jsm/"
            }
        }
    </script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: Arial, sans-serif; 
            background: linear-gradient(135deg, #0a0a0a 0%, #1a1a2e 100%); 
            color: #e0e0e0; 
            height: 100vh; 
            overflow: hidden; 
        }
        .container { 
            display: grid; 
            grid-template-areas: "header header" "main sidebar"; 
            grid-template-columns: 1fr 400px; 
            grid-template-rows: 60px 1fr; 
            height: 100vh; 
            gap: 2px; 
            background: #000; 
        }
        .header { 
            grid-area: header; 
            background: #1a1a2e; 
            display: flex; 
            align-items: center; 
            justify-content: space-between; 
            padding: 0 20px; 
            border-bottom: 2px solid #00bcd4; 
        }
        .header h1 { color: #00bcd4; font-size: 1.4rem; }
        .status { 
            padding: 5px 15px; 
            border-radius: 12px; 
            background: rgba(76,175,80,0.2); 
            border: 1px solid #4caf50; 
            color: #4caf50; 
            font-size: 0.85rem; 
        }
        .main { 
            grid-area: main; 
            background: #0a0a0a; 
            position: relative; 
        }
        .sidebar { 
            grid-area: sidebar; 
            background: #1a1a2e; 
            overflow-y: auto; 
            padding: 20px; 
        }
        .panel { 
            background: rgba(255,255,255,0.05); 
            border-radius: 8px; 
            margin-bottom: 20px; 
            border: 1px solid rgba(255,255,255,0.1); 
        }
        .panel-header { 
            background: #16213e; 
            padding: 15px; 
            border-bottom: 1px solid rgba(255,255,255,0.1); 
            font-weight: 600; 
            color: #00bcd4; 
        }
        .panel-content { padding: 15px; }
        .metric-row { 
            display: flex; 
            justify-content: space-between; 
            align-items: center; 
            padding: 10px 0; 
            border-bottom: 1px solid rgba(255,255,255,0.05); 
        }
        .metric-row:last-child { border-bottom: none; }
        .metric-label { color: #999; font-size: 0.9rem; }
        .metric-value { color: #fff; font-weight: 600; }
        .sensor-card { 
            background: rgba(255,255,255,0.03); 
            border-radius: 6px; 
            padding: 15px; 
            margin: 10px 0; 
            border: 1px solid rgba(255,255,255,0.1); 
        }
        .sensor-name { color: #00bcd4; font-weight: 600; margin-bottom: 10px; }
        .sensor-points { color: #4caf50; font-size: 1.2rem; font-weight: bold; }
        .event-item { 
            background: rgba(255,255,255,0.05); 
            border-radius: 6px; 
            padding: 12px; 
            margin: 8px 0; 
            border-left: 3px solid #00bcd4; 
            font-size: 0.9rem; 
        }
        .event-time { color: #999; font-size: 0.8rem; }
        #three-container { 
            width: 100%; 
            height: 100%; 
            background: #1a1a1a; 
            border: 2px solid #333; 
            border-radius: 8px; 
            position: relative; 
            overflow: hidden; 
        }
        .loading { 
            position: absolute; 
            top: 50%; 
            left: 50%; 
            transform: translate(-50%, -50%); 
            text-align: center; 
            color: #00bcd4; 
            font-size: 1.1rem; 
        }
        .success { color: #4caf50; }
        .error { color: #f44336; }
        canvas { display: block; width: 100% !important; height: 100% !important; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🎯 Olympus Live Dashboard</h1>
            <div class="status" id="connection-status">🔴 Connecting...</div>
        </div>
        <div class="main">
            <div id="three-container">
                <div class="loading">
                    <div>⚙️ Initializing 3D Engine...</div>
                    <div id="status-text" style="margin-top: 10px; font-size: 0.9rem;">Loading Three.js...</div>
                </div>
            </div>
        </div>
        <div class="sidebar">
            <div class="panel">
                <div class="panel-header">📊 Performance Metrics</div>
                <div class="panel-content">
                    <div class="metric-row">
                        <div class="metric-label">Average Latency</div>
                        <div class="metric-value" id="avg-latency">-- ms</div>
                    </div>
                    <div class="metric-row">
                        <div class="metric-label">P95 Latency</div>
                        <div class="metric-value" id="p95-latency">-- ms</div>
                    </div>
                    <div class="metric-row">
                        <div class="metric-label">Sample Count</div>
                        <div class="metric-value" id="sample-count">0</div>
                    </div>
                    <div class="metric-row">
                        <div class="metric-label">Data Source</div>
                        <div class="metric-value success">Real Gazebo</div>
                    </div>
                </div>
            </div>
            <div class="panel">
                <div class="panel-header">📡 mmWave Sensors</div>
                <div class="panel-content">
                    <div class="sensor-card">
                        <div class="sensor-name">mmWave Sensor 1</div>
                        <div>Points: <span class="sensor-points" id="sensor1-points">0</span></div>
                        <div style="color: #999; font-size: 0.8rem;">Position: (-5, 2.5, 0)</div>
                    </div>
                    <div class="sensor-card">
                        <div class="sensor-name">mmWave Sensor 2</div>
                        <div>Points: <span class="sensor-points" id="sensor2-points">0</span></div>
                        <div style="color: #999; font-size: 0.8rem;">Position: (5, 2.5, 0)</div>
                    </div>
                </div>
            </div>
            <div class="panel">
                <div class="panel-header">🤖 Live Automation Events</div>
                <div class="panel-content">
                    <div id="automation-events">
                        <div style="text-align: center; color: #999; padding: 20px;">Waiting for events...</div>
                    </div>
                </div>
            </div>
        </div>
    </div>
    <script type="module">
        import * as THREE from 'three';
        import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
        
        console.log('🚀 Starting Olympus Dashboard...');
        let scene, camera, renderer, controls, sensors = {}, human, pointClouds = {}, socket;
        
        function init3D() {
            console.log('🎨 Initializing 3D scene...');
            const container = document.getElementById('three-container');
            const loadingDiv = container.querySelector('.loading');
            
            try {
                if (typeof THREE === 'undefined') throw new Error('Three.js failed to load');
                console.log('✅ Three.js loaded, version:', THREE.REVISION);
                updateStatus('Three.js loaded successfully');
                
                scene = new THREE.Scene();
                scene.background = new THREE.Color(0x2c2c2c);
                camera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 100);
                camera.position.set(10, 8, 10);
                camera.lookAt(0, 0, 0);
                
                renderer = new THREE.WebGLRenderer({ antialias: true });
                renderer.setSize(container.clientWidth, container.clientHeight);
                renderer.setPixelRatio(window.devicePixelRatio);
                renderer.shadowMap.enabled = true;
                
                loadingDiv.style.display = 'none';
                container.appendChild(renderer.domElement);
                
                controls = new OrbitControls(camera, renderer.domElement);
                controls.enableDamping = true;
                controls.dampingFactor = 0.05;
                
                const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
                scene.add(ambientLight);
                const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
                directionalLight.position.set(10, 10, 5);
                directionalLight.castShadow = true;
                scene.add(directionalLight);
                
                createEnvironment();
                animate();
                console.log('✅ 3D scene initialized successfully');
                updateStatus('3D environment ready');
                
            } catch (error) {
                console.error('❌ 3D initialization failed:', error);
                updateStatus(`ERROR: ${error.message}`);
                loadingDiv.innerHTML = `<div style="color: #f44336;">❌ 3D Error: ${error.message}</div>`;
            }
        }
        
        function createEnvironment() {
            const floorGeometry = new THREE.PlaneGeometry(20, 20);
            const floorMaterial = new THREE.MeshLambertMaterial({ color: 0x444444 });
            const floor = new THREE.Mesh(floorGeometry, floorMaterial);
            floor.rotation.x = -Math.PI / 2;
            floor.receiveShadow = true;
            scene.add(floor);
            
            const gridHelper = new THREE.GridHelper(20, 40, 0x666666, 0x333333);
            scene.add(gridHelper);
            
            createSensor('mmwave1', -5, 2.5, 0, 0x00ff00);
            createSensor('mmwave2', 5, 2.5, 0, 0x0080ff);
            
            const humanGroup = new THREE.Group();
            const bodyGeometry = new THREE.CylinderGeometry(0.3, 0.3, 1.6);
            const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0xff6b6b });
            const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
            body.position.y = 0.8;
            body.castShadow = true;
            humanGroup.add(body);
            
            const headGeometry = new THREE.SphereGeometry(0.2);
            const head = new THREE.Mesh(headGeometry, bodyMaterial);
            head.position.y = 1.8;
            head.castShadow = true;
            humanGroup.add(head);
            
            humanGroup.position.set(0, 0, 0);
            scene.add(humanGroup);
            human = humanGroup;
            console.log('🏠 Environment created');
        }
        
        function createSensor(id, x, y, z, color) {
            const group = new THREE.Group();
            const geometry = new THREE.CylinderGeometry(0.1, 0.1, 0.3);
            const material = new THREE.MeshPhongMaterial({ color: color });
            const mesh = new THREE.Mesh(geometry, material);
            mesh.castShadow = true;
            group.add(mesh);
            
            const fovGeometry = new THREE.ConeGeometry(3, 6, 8, 1, true);
            const fovMaterial = new THREE.MeshBasicMaterial({ 
                color: color, transparent: true, opacity: 0.1, side: THREE.DoubleSide
            });
            const fov = new THREE.Mesh(fovGeometry, fovMaterial);
            fov.rotation.x = Math.PI / 2;
            fov.position.z = id === 'mmwave2' ? -3 : 3;
            group.add(fov);
            
            group.position.set(x, y, z);
            if (id === 'mmwave2') group.rotation.y = Math.PI;
            scene.add(group);
            sensors[id] = { group, mesh, fov };
            
            const pointGeometry = new THREE.BufferGeometry();
            const positions = new Float32Array(1000 * 3);
            const colors = new Float32Array(1000 * 3);
            pointGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
            pointGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
            
            const pointMaterial = new THREE.PointsMaterial({ 
                size: 0.05, vertexColors: true, transparent: true, opacity: 0.8
            });
            
            const points = new THREE.Points(pointGeometry, pointMaterial);
            points.visible = false;
            scene.add(points);
            pointClouds[id] = points;
        }
        
        function animate() {
            requestAnimationFrame(animate);
            if (controls) controls.update();
            Object.values(sensors).forEach(sensor => {
                if (sensor.mesh) sensor.mesh.rotation.y += 0.01;
            });
            if (renderer && scene && camera) renderer.render(scene, camera);
        }
        
        function updateStatus(message) {
            const statusEl = document.getElementById('status-text');
            if (statusEl) statusEl.textContent = message;
            console.log('📝 Status:', message);
        }
        
        function updateElement(id, value) {
            const el = document.getElementById(id);
            if (el) el.textContent = value;
        }
        
        function updatePointCloud(sensorId, numPoints) {
            const pointCloud = pointClouds[sensorId];
            const sensor = sensors[sensorId];
            if (!pointCloud || !sensor) return;
            
            const positions = pointCloud.geometry.attributes.position.array;
            const colors = pointCloud.geometry.attributes.color.array;
            
            if (numPoints > 0) {
                const sensorPos = sensor.group.position;
                for (let i = 0; i < Math.min(numPoints, 333); i++) {
                    const idx = i * 3;
                    const angle = (Math.random() - 0.5) * Math.PI / 3;
                    const distance = 1 + Math.random() * 5;
                    const height = Math.random() * 2;
                    
                    positions[idx] = sensorPos.x + Math.sin(angle) * distance;
                    positions[idx + 1] = height;
                    positions[idx + 2] = sensorPos.z + Math.cos(angle) * distance * (sensorId === 'mmwave2' ? -1 : 1);
                    
                    const intensity = 1 - (distance / 6);
                    colors[idx] = sensorId === 'mmwave1' ? intensity : 0;
                    colors[idx + 1] = sensorId === 'mmwave1' ? intensity : intensity;
                    colors[idx + 2] = sensorId === 'mmwave2' ? intensity : 0;
                }
                pointCloud.geometry.setDrawRange(0, Math.min(numPoints, 333));
                pointCloud.visible = true;
            } else {
                pointCloud.visible = false;
            }
            pointCloud.geometry.attributes.position.needsUpdate = true;
            pointCloud.geometry.attributes.color.needsUpdate = true;
        }
        
        function initWebSocket() {
            console.log('🔌 Connecting to WebSocket...');
            socket = io();
            
            socket.on('connect', () => {
                console.log('✅ WebSocket connected');
                document.getElementById('connection-status').innerHTML = '🟢 Connected';
                document.getElementById('connection-status').className = 'status success';
            });
            
            socket.on('disconnect', () => {
                console.log('❌ WebSocket disconnected');
                document.getElementById('connection-status').innerHTML = '🔴 Disconnected';
                document.getElementById('connection-status').className = 'status error';
            });
            
            socket.on('dashboard_update', (data) => {
                console.log('📊 Dashboard update received:', data);
                
                if (data.sensors && data.sensors.sensors) {
                    Object.entries(data.sensors.sensors).forEach(([sensorId, info]) => {
                        const points = info.num_points || 0;
                        updateElement(`${sensorId.replace('mmwave', 'sensor')}-points`, points);
                        updatePointCloud(sensorId, points);
                    });
                }
                
                if (data.latency_stats) {
                    updateElement('avg-latency', 
                        data.latency_stats.mean ? `${data.latency_stats.mean.toFixed(1)} ms` : '-- ms');
                    updateElement('p95-latency', 
                        data.latency_stats.p95 ? `${data.latency_stats.p95.toFixed(1)} ms` : '-- ms');
                    updateElement('sample-count', data.latency_stats.count || 0);
                }
                
                if (data.automation_events && data.automation_events.length > 0) {
                    const eventsEl = document.getElementById('automation-events');
                    eventsEl.innerHTML = '';
                    data.automation_events.slice().reverse().slice(0, 5).forEach(event => {
                        const div = document.createElement('div');
                        div.className = 'event-item';
                        const time = new Date(event.timestamp * 1000).toLocaleTimeString();
                        div.innerHTML = `
                            <div><strong>${event.actuator_id} → ${event.action}</strong></div>
                            <div class="event-time">${time}</div>
                        `;
                        eventsEl.appendChild(div);
                    });
                }
            });
        }
        
        document.addEventListener('DOMContentLoaded', () => {
            console.log('🌟 DOM loaded, starting dashboard...');
            init3D();
            initWebSocket();
        });
        
        window.addEventListener('resize', () => {
            if (camera && renderer) {
                const container = document.getElementById('three-container');
                camera.aspect = container.clientWidth / container.clientHeight;
                camera.updateProjectionMatrix();
                renderer.setSize(container.clientWidth, container.clientHeight);
            }
        });
    </script>
</body>
</html>'''

@app.route('/api/status')
def api_status():
    """Current system status"""
    return jsonify(data_manager.get_dashboard_data())

@app.route('/api/sensors')
def api_sensors():
    """Sensor status"""
    return jsonify(data_manager.get_sensor_summary())

@app.route('/api/latency/stats')
def api_latency_stats():
    """Latency statistics"""
    return jsonify(data_manager.get_latency_stats())

@app.route('/api/latency/history')
def api_latency_history():
    """Latency history for graphing"""
    return jsonify({
        'history': data_manager.get_latency_history(limit=100)
    })

@app.route('/api/automation/events')
def api_automation_events():
    """Recent automation events"""
    return jsonify({
        'events': data_manager.get_automation_events(limit=50)
    })

@socketio.on('connect')
def on_connect():
    """Client connected"""
    logger.info("Client connected")
    emit('connected', {'status': 'ok'})
    
    # Send initial data
    emit('dashboard_update', data_manager.get_dashboard_data())

@socketio.on('disconnect')
def on_disconnect():
    """Client disconnected"""
    logger.info("Client disconnected")

if __name__ == '__main__':
    logger.info("Starting Olympus Live Dashboard")
    
    # Start data manager
    data_manager.start()
    
    # Start broadcast thread
    broadcast_thread = threading.Thread(target=broadcast_updates, daemon=True)
    broadcast_thread.start()
    
    try:
        # Run Flask app
        socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        logger.info("Shutting down")
        data_manager.stop()
    except Exception as e:
        logger.error(f"Dashboard error: {e}")
        data_manager.stop()