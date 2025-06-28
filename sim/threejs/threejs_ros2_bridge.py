#!/usr/bin/env python3
"""
Three.js to ROS2 Bridge for Olympus Simulation
Receives sensor data from browser-based Three.js simulation
and publishes as ROS2 PointCloud2 messages for scalable smart home systems
"""

import json
import time
import logging
import numpy as np
import os
from flask import Flask, request, jsonify, render_template_string
from threading import Thread
import signal
import sys
import struct

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# Simple point cloud creation without sensor_msgs_py dependency
def create_point_cloud2(header, fields, points):
    """Create PointCloud2 message manually"""
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = fields
    cloud.is_bigendian = False
    cloud.point_step = sum(field.count * 4 for field in fields)  # 4 bytes per float32
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    
    # Pack point data
    data = []
    for point in points:
        for value in point:
            data.extend(struct.pack('f', float(value)))
    
    cloud.data = bytes(data)
    return cloud

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('threejs_ros2_bridge')

class ThreeJSROS2Bridge(Node):
    def __init__(self, http_port=5555):
        super().__init__('threejs_ros2_bridge')
        
        self.http_port = http_port
        
        # ROS2 publishers for each sensor
        self.sensor_publishers = {}
        
        # QoS profile for sensor data
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Flask app for HTTP interface
        self.app = Flask(__name__)
        # Add CORS headers manually
        @self.app.after_request
        def after_request(response):
            response.headers.add('Access-Control-Allow-Origin', '*')
            response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
            response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
            return response
        
        self.setup_routes()
        
        # Statistics and data storage for point cloud viewer
        self.stats = {
            'messages_received': 0,
            'ros2_published': 0,
            'start_time': time.time(),
            'active_sensors': set()
        }
        self.latest_pointclouds = {}
        
        # Frame ID counter for point cloud headers
        self.frame_counter = 0
        
        logger.info("Three.js ROS2 Bridge initialized")
        
    def create_sensor_publisher(self, sensor_id):
        """Create ROS2 publisher for a sensor if it doesn't exist"""
        if sensor_id not in self.sensor_publishers:
            topic_name = f'/mmwave/{sensor_id}/points'
            publisher = self.create_publisher(
                PointCloud2, 
                topic_name, 
                self.qos_profile
            )
            self.sensor_publishers[sensor_id] = publisher
            self.stats['active_sensors'].add(sensor_id)
            logger.info(f"Created ROS2 publisher for {sensor_id} on topic {topic_name}")
            
        return self.sensor_publishers[sensor_id]
    
    def threejs_to_pointcloud2(self, sensor_id, point_cloud_data, metadata):
        """Convert Three.js point cloud data to ROS2 PointCloud2 message"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = f"{sensor_id}_frame"
        
        # Define point cloud fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='range', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='rcs', offset=24, datatype=PointField.FLOAT32, count=1)
        ]
        
        # Convert point data to structured array
        if point_cloud_data and len(point_cloud_data) > 0:
            points = []
            for point in point_cloud_data:
                points.append([
                    float(point.get('x', 0)),
                    float(point.get('y', 0)), 
                    float(point.get('z', 0)),
                    float(point.get('intensity', 0)),
                    float(point.get('range', 0)),
                    float(point.get('velocity', 0)),
                    float(point.get('rcs', 1.0))
                ])
            
            points_array = np.array(points, dtype=np.float32)
        else:
            # Empty point cloud
            points_array = np.empty((0, 7), dtype=np.float32)
        
        # Create PointCloud2 message
        point_cloud_msg = create_point_cloud2(header, fields, points_array)
        
        # Add metadata as custom fields if needed
        if metadata:
            point_cloud_msg.width = len(point_cloud_data) if point_cloud_data else 0
            point_cloud_msg.height = 1
            point_cloud_msg.is_dense = True
        
        return point_cloud_msg
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template_string('''
<!DOCTYPE html>
<html>
<head>
    <title>Olympus Three.js ROS2 Simulation</title>
    <style>
        body { margin: 0; font-family: Arial, sans-serif; background: #111; color: #fff; }
        #container { width: 100vw; height: 100vh; position: relative; }
        #info { position: absolute; top: 10px; left: 10px; z-index: 100; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 5px; }
        #controls { position: absolute; top: 10px; right: 10px; z-index: 100; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 5px; }
        button { margin: 5px; padding: 8px 12px; }
        .sensor-info { margin: 5px 0; font-size: 12px; }
        .ros2-status { color: #4caf50; font-weight: bold; }
    </style>
</head>
<body>
    <div id="info">
        <h3>Olympus ROS2 Simulation</h3>
        <div>Status: <span id="status">Initializing...</span></div>
        <div>Sensors: <span id="sensor-count">0</span></div>
        <div class="ros2-status">ROS2: <span id="ros2-status">Active</span></div>
        <div id="sensor-data"></div>
    </div>
    <div id="controls">
        <button onclick="addSensor()">Add Sensor</button>
        <button onclick="toggleSimulation()">Start/Stop</button>
        <button onclick="resetScene()">Reset Scene</button>
        <button onclick="toggleROS2Mode()">Toggle ROS2 Mode</button>
        <button onclick="testDataSend()" style="background: #ff6600;">🔧 Test Data Send</button>
        <button onclick="debugSimulation()" style="background: #6600ff;">🐛 Debug Sim</button>
        <button onclick="manualSensorRun()" style="background: #00ff66;">▶️ Manual Run</button>
        <div>
            <label>Update Rate: <input type="range" id="updateRate" min="1" max="10" value="2" onchange="updateRate(this.value)"> <span id="rateValue">2</span> Hz</label>
        </div>
    </div>
    <div id="container"></div>

    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script>
        console.log('🚀 Starting Three.js ROS2 Simulation...');
        
        let scene, camera, renderer, controls, sensors = new Map(), animationId;
        let isRunning = false, ros2Mode = true;
        
        function init() {
            console.log('🎨 Initializing Three.js scene...');
            document.getElementById('status').textContent = 'Loading Three.js...';
            
            // Check if THREE is available
            if (typeof THREE === 'undefined') {
                console.error('❌ THREE.js library not loaded');
                document.getElementById('status').textContent = 'ERROR: THREE.js not loaded';
                setTimeout(init, 1000); // Retry in 1 second
                return;
            }
            
            console.log('✅ THREE.js available, version:', THREE.REVISION);
            const container = document.getElementById('container');
            
            try {
                // Initialize Three.js scene
                scene = new THREE.Scene();
                scene.background = new THREE.Color(0x222222);
                
                camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
                camera.position.set(10, 8, 10);
                camera.lookAt(0, 0, 0);
                
                renderer = new THREE.WebGLRenderer({ antialias: true });
                renderer.setSize(container.clientWidth, container.clientHeight);
                renderer.setClearColor(0x222222);
                renderer.shadowMap.enabled = true;
                container.appendChild(renderer.domElement);
                
                // Disable right-click context menu
                renderer.domElement.addEventListener('contextmenu', (e) => e.preventDefault());
                
                // Manual camera controls and object interaction
                setupCameraControls();
                setupObjectInteraction();
                
                setupLighting();
                setupScene();
                addDefaultSensors();
                startSimulation();
                
                document.getElementById('status').textContent = 'Running (ROS2 Mode)';
                isRunning = true;
                console.log('[DEBUG] Simulation started, isRunning=', isRunning);
                console.log('[DEBUG] Scene objects:', window.sceneObjects ? window.sceneObjects.length : 'undefined');
                
                console.log('✅ Three.js scene initialized successfully');
                
            } catch (error) {
                console.error('❌ Three.js initialization failed:', error);
                document.getElementById('status').textContent = 'ERROR: ' + error.message;
            }
        }
        
        function setupLighting() {
            const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
            scene.add(ambientLight);
            
            const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
            directionalLight.position.set(10, 10, 5);
            directionalLight.castShadow = true;
            scene.add(directionalLight);
        }
        
        function setupScene() {
            // Ground plane
            const groundGeometry = new THREE.PlaneGeometry(20, 20);
            const groundMaterial = new THREE.MeshLambertMaterial({ color: 0x666666 });
            const ground = new THREE.Mesh(groundGeometry, groundMaterial);
            ground.rotation.x = -Math.PI / 2;
            ground.receiveShadow = true;
            scene.add(ground);
            
            // Grid helper
            const gridHelper = new THREE.GridHelper(20, 40, 0x444444, 0x222222);
            scene.add(gridHelper);
            
            // Sample objects for detection
            createSampleObjects();
        }
        
        function createSampleObjects() {
            // Create object storage
            window.sceneObjects = [];
            
            // Human figure (draggable)
            const humanGroup = new THREE.Group();
            const humanGeometry = new THREE.CylinderGeometry(0.3, 0.3, 1.6, 8);
            const humanMaterial = new THREE.MeshLambertMaterial({ color: 0x4444ff });
            const humanBody = new THREE.Mesh(humanGeometry, humanMaterial);
            humanBody.position.y = 0.8;
            humanBody.castShadow = true;
            humanGroup.add(humanBody);
            
            // Head
            const headGeometry = new THREE.SphereGeometry(0.2, 16, 16);
            const head = new THREE.Mesh(headGeometry, humanMaterial);
            head.position.y = 1.8;
            head.castShadow = true;
            humanGroup.add(head);
            
            // Label for human
            const humanLabel = createLabel('HUMAN (Drag me!)');
            humanLabel.position.y = 2.5;
            humanGroup.add(humanLabel);
            
            humanGroup.position.set(-3, 0, 4);  // Position human in front of mmwave1 sensor (-5,1,0) facing +Z
            humanGroup.userData = { type: 'human', rcs: 1.0, moveable: true, name: 'Human' };
            scene.add(humanGroup);
            window.sceneObjects.push(humanGroup);
            
            // Table (draggable)
            const tableGroup = new THREE.Group();
            const tableGeometry = new THREE.BoxGeometry(1.5, 0.1, 0.8);
            const tableMaterial = new THREE.MeshLambertMaterial({ color: 0x8B4513 });
            const table = new THREE.Mesh(tableGeometry, tableMaterial);
            table.position.y = 0.05;
            table.castShadow = true;
            tableGroup.add(table);
            
            // Table legs
            for (let i = 0; i < 4; i++) {
                const legGeometry = new THREE.CylinderGeometry(0.05, 0.05, 0.8);
                const leg = new THREE.Mesh(legGeometry, tableMaterial);
                const x = (i % 2) * 1.4 - 0.7;
                const z = Math.floor(i / 2) * 0.6 - 0.3;
                leg.position.set(x, -0.4, z);
                leg.castShadow = true;
                tableGroup.add(leg);
            }
            
            const tableLabel = createLabel('TABLE');
            tableLabel.position.y = 1.2;
            tableGroup.add(tableLabel);
            
            tableGroup.position.set(3, 0.8, -3);  // Position table in front of mmwave2 sensor (5,1,0) facing -Z
            tableGroup.userData = { type: 'furniture', rcs: 0.5, moveable: true, name: 'Table' };
            scene.add(tableGroup);
            window.sceneObjects.push(tableGroup);
            
            // Metal object (draggable)
            const metalGroup = new THREE.Group();
            const metalGeometry = new THREE.SphereGeometry(0.3, 16, 16);
            const metalMaterial = new THREE.MeshLambertMaterial({ color: 0x999999 });
            const metalObject = new THREE.Mesh(metalGeometry, metalMaterial);
            metalObject.castShadow = true;
            metalGroup.add(metalObject);
            
            const metalLabel = createLabel('METAL SPHERE');
            metalLabel.position.y = 0.8;
            metalGroup.add(metalLabel);
            
            metalGroup.position.set(-4, 0.3, 2);  // Position metal sphere within mmwave1's FOV
            metalGroup.userData = { type: 'object', rcs: 2.0, moveable: true, name: 'Metal Sphere' };
            scene.add(metalGroup);
            window.sceneObjects.push(metalGroup);
        }
        
        function createLabel(text) {
            const canvas = document.createElement('canvas');
            const context = canvas.getContext('2d');
            canvas.width = 256;
            canvas.height = 64;
            
            context.fillStyle = 'rgba(0, 0, 0, 0.8)';
            context.fillRect(0, 0, canvas.width, canvas.height);
            
            context.fillStyle = 'white';
            context.font = '16px Arial';
            context.textAlign = 'center';
            context.fillText(text, canvas.width / 2, canvas.height / 2 + 6);
            
            const texture = new THREE.CanvasTexture(canvas);
            const material = new THREE.SpriteMaterial({ map: texture });
            const sprite = new THREE.Sprite(material);
            sprite.scale.set(2, 0.5, 1);
            
            return sprite;
        }
        
        function addDefaultSensors() {
            addSensorToScene('mmwave1', new THREE.Vector3(-5, 1, 0), new THREE.Euler(0, 0, 0));
            addSensorToScene('mmwave2', new THREE.Vector3(5, 1, 0), new THREE.Euler(0, Math.PI, 0));
        }
        
        function addSensorToScene(id, position, rotation) {
            const sensorGroup = new THREE.Group();
            
            // Sensor body (more visible)
            const sensorGeometry = new THREE.BoxGeometry(0.2, 0.2, 0.1);
            const sensorMaterial = new THREE.MeshLambertMaterial({ color: 0xff0000 });
            const sensorMesh = new THREE.Mesh(sensorGeometry, sensorMaterial);
            sensorGroup.add(sensorMesh);
            
            // FOV visualization - subtle wireframe 
            const fovAngle = 22.5; // Half FOV in degrees (45° total)
            const range = 10;
            const fovGeometry = new THREE.ConeGeometry(
                range * Math.tan(fovAngle * Math.PI / 180), 
                range, 
                16, 1, true
            );
            const fovMaterial = new THREE.MeshBasicMaterial({
                color: 0x00ff44,
                transparent: true,
                opacity: 0.08,
                wireframe: true
            });
            const fovCone = new THREE.Mesh(fovGeometry, fovMaterial);
            fovCone.position.z = range / 2;
            fovCone.rotation.x = Math.PI / 2;
            sensorGroup.add(fovCone);
            
            // Sensor label
            const sensorLabel = createLabel(id.toUpperCase() + ' SENSOR');
            sensorLabel.position.y = 1;
            sensorGroup.add(sensorLabel);
            
            sensorGroup.position.copy(position);
            sensorGroup.rotation.copy(rotation);
            scene.add(sensorGroup);
            
            sensors.set(id, {
                id: id,
                group: sensorGroup,
                lastUpdate: 0,
                config: { horizontalFov: Math.PI/2, verticalFov: Math.PI/6, minRange: 0.1, maxRange: 10.0 }
            });
            
            updateSensorCount();
            console.log('Added sensor:', id);
        }
        
        // Camera controls - right mouse button only
        function setupCameraControls() {
            let isMouseDown = false;
            let mouseX = 0, mouseY = 0;
            let cameraDistance = 15;
            let cameraAngleX = 0;
            let cameraAngleY = Math.PI / 4;
            
            // Right mouse button for camera orbit
            renderer.domElement.addEventListener('mousedown', (event) => {
                if (event.button === 2) { // Right mouse button
                    isMouseDown = true;
                    mouseX = event.clientX;
                    mouseY = event.clientY;
                    event.preventDefault();
                }
            });
            
            renderer.domElement.addEventListener('mouseup', () => {
                isMouseDown = false;
            });
            
            renderer.domElement.addEventListener('mousemove', (event) => {
                if (isMouseDown && event.buttons === 2) { // Right mouse button held
                    const deltaX = event.clientX - mouseX;
                    const deltaY = event.clientY - mouseY;
                    
                    cameraAngleX += deltaX * 0.01;
                    cameraAngleY = Math.max(0.1, Math.min(Math.PI - 0.1, cameraAngleY - deltaY * 0.01));
                    
                    updateCameraPosition();
                    
                    mouseX = event.clientX;
                    mouseY = event.clientY;
                    event.preventDefault();
                }
            });
            
            // Zoom with mouse wheel
            renderer.domElement.addEventListener('wheel', (event) => {
                cameraDistance += event.deltaY * 0.01;
                cameraDistance = Math.max(5, Math.min(50, cameraDistance));
                updateCameraPosition();
                event.preventDefault();
            });
            
            function updateCameraPosition() {
                camera.position.x = Math.sin(cameraAngleY) * Math.cos(cameraAngleX) * cameraDistance;
                camera.position.y = Math.cos(cameraAngleY) * cameraDistance;
                camera.position.z = Math.sin(cameraAngleY) * Math.sin(cameraAngleX) * cameraDistance;
                camera.lookAt(0, 0, 0);
            }
            
            updateCameraPosition();
        }
        
        // Object interaction - separate from camera controls
        function setupObjectInteraction() {
            const raycaster = new THREE.Raycaster();
            const mouse = new THREE.Vector2();
            let selectedObject = null;
            let isDragging = false;
            
            // Handle object selection and dragging with left mouse button only
            renderer.domElement.addEventListener('mousedown', (event) => {
                // Only handle left mouse button for object interaction
                if (event.button !== 0) return;
                
                mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
                mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
                
                raycaster.setFromCamera(mouse, camera);
                const intersects = raycaster.intersectObjects(window.sceneObjects || [], true);
                
                if (intersects.length > 0) {
                    // Find the top-level object (group)
                    let object = intersects[0].object;
                    while (object.parent && object.parent !== scene) {
                        object = object.parent;
                    }
                    
                    if (object.userData && object.userData.moveable) {
                        selectedObject = object;
                        isDragging = true;
                        document.body.style.cursor = 'grabbing';
                        console.log('Selected for dragging:', object.userData.name || object.userData.type);
                        event.stopPropagation();
                        event.preventDefault();
                    }
                }
            });
            
            renderer.domElement.addEventListener('mousemove', (event) => {
                if (isDragging && selectedObject) {
                    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
                    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
                    
                    raycaster.setFromCamera(mouse, camera);
                    
                    // Project onto ground plane (y = 0)
                    const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
                    const intersectionPoint = new THREE.Vector3();
                    raycaster.ray.intersectPlane(groundPlane, intersectionPoint);
                    
                    if (intersectionPoint) {
                        selectedObject.position.x = intersectionPoint.x;
                        selectedObject.position.z = intersectionPoint.z;
                        // Keep original Y position for object type
                        if (selectedObject.userData.type === 'human') {
                            selectedObject.position.y = 0;
                        } else if (selectedObject.userData.type === 'furniture') {
                            selectedObject.position.y = 0.8;
                        } else {
                            selectedObject.position.y = 0.3;
                        }
                    }
                    event.stopPropagation();
                }
            });
            
            renderer.domElement.addEventListener('mouseup', () => {
                if (isDragging) {
                    isDragging = false;
                    selectedObject = null;
                    document.body.style.cursor = 'default';
                }
            });
            
            // Hover effects
            renderer.domElement.addEventListener('mousemove', (event) => {
                if (!isDragging) {
                    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
                    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
                    
                    raycaster.setFromCamera(mouse, camera);
                    const intersects = raycaster.intersectObjects(window.sceneObjects || [], true);
                    
                    if (intersects.length > 0) {
                        let object = intersects[0].object;
                        while (object.parent && object.parent !== scene) {
                            object = object.parent;
                        }
                        if (object.userData && object.userData.moveable) {
                            document.body.style.cursor = 'grab';
                            return;
                        }
                    }
                    document.body.style.cursor = 'default';
                }
            });
        }
        
        function updateSensorCount() {
            document.getElementById('sensor-count').textContent = sensors.size;
        }
        
        function simulateSensors() {
            console.log(`[DEBUG] simulateSensors called, isRunning=${isRunning}, sensors=${sensors.size}`);
            sensors.forEach((sensor, sensorId) => {
                const pointCloud = simulatePointCloud(sensor);
                console.log(`[DEBUG] ${sensorId}: Generated ${pointCloud.length} points`);
                publishSensorData(sensorId, pointCloud);
                updateSensorDisplay(sensorId, pointCloud);
            });
        }
        
        // Helper functions for realistic ray casting
        function getObjectBounds(object) {
            const box = new THREE.Box3().setFromObject(object);
            const size = box.getSize(new THREE.Vector3());
            return {
                radius: Math.max(size.x, size.y, size.z) / 2,
                center: box.getCenter(new THREE.Vector3())
            };
        }
        
        function calculateReflection(objectType, surfaceNormal, rayDirection) {
            // mmWave reflection based on material and incident angle
            const incidentAngle = Math.abs(surfaceNormal.dot(rayDirection));
            
            // Material reflection coefficients for 60GHz mmWave
            const materialCoeff = {
                'human': 0.8,      // High water content
                'furniture': 0.3,  // Wood/fabric mix
                'object': 0.9      // Metal objects
            };
            
            const baseReflection = materialCoeff[objectType] || 0.5;
            
            // Fresnel-like reflection (simplified)
            const angleReflection = 0.2 + 0.8 * Math.pow(incidentAngle, 0.5);
            
            return baseReflection * angleReflection * (0.8 + 0.4 * Math.random());
        }
        
        function simulatePointCloud(sensor) {
            if (!isRunning) {
                console.log(`[${sensor.id}] Simulation not running, returning empty points`);
                return [];
            }
            
            const points = [];
            
            // Ensure sensor transform is up to date
            sensor.group.updateMatrixWorld(true);
            const sensorPos = sensor.group.position;
            const sensorRotation = sensor.group.rotation;
            
            console.log(`[${sensor.id}] Starting simulation - isRunning=${isRunning}, sceneObjects=${window.sceneObjects ? window.sceneObjects.length : 'undefined'}`);
            console.log(`[${sensor.id}] Sensor position:`, sensorPos);
            console.log(`[${sensor.id}] Sensor rotation:`, sensorRotation);
            
            // Check each moveable object
            (window.sceneObjects || []).forEach((object, index) => {
                console.log(`[${sensor.id}] Checking object ${index}: ${object.userData ? object.userData.name : 'no userData'} at position:`, object.position);
                
                if (object.userData && object.userData.moveable) {
                    // Ensure object matrix is updated
                    object.updateMatrixWorld(true);
                    const distance = sensorPos.distanceTo(object.position);
                    console.log(`[${sensor.id}] Object ${object.userData.name}: distance=${distance.toFixed(2)}m, range=${sensor.config.minRange}-${sensor.config.maxRange}`);
                    
                    if (distance <= sensor.config.maxRange && distance >= sensor.config.minRange) {
                        // Check if object is within sensor FOV
                        const directionToObject = object.position.clone().sub(sensorPos).normalize();
                        const sensorForward = new THREE.Vector3(0, 0, 1);
                        sensorForward.applyEuler(sensorRotation);
                        
                        const angle = sensorForward.angleTo(directionToObject);
                        const fovHalf = sensor.config.horizontalFov / 2;
                        console.log(`[${sensor.id}] Object ${object.userData.name}: angle=${(angle*180/Math.PI).toFixed(1)}°, FOV_half=${(fovHalf*180/Math.PI).toFixed(1)}°, inFOV=${angle <= fovHalf}`);
                        
                        if (angle <= sensor.config.horizontalFov / 2) {
                            // Realistic mmWave ray casting simulation
                            console.log(`[${sensor.id}] DETECTED ${object.userData.name}! Starting ray casting...`);
                            
                            // Cast multiple rays in a pattern around the object
                            const raysPerObject = 64; // Higher resolution
                            const objectBounds = getObjectBounds(object);
                            
                            for (let i = 0; i < raysPerObject; i++) {
                                // Create ray pattern around object surface
                                const phi = (i / raysPerObject) * 2 * Math.PI;
                                const theta = Math.random() * Math.PI / 6; // Small vertical spread
                                
                                // Ray target on object surface
                                const targetOffset = new THREE.Vector3(
                                    Math.cos(phi) * Math.sin(theta) * objectBounds.radius,
                                    Math.cos(theta) * objectBounds.radius * 0.5,
                                    Math.sin(phi) * Math.sin(theta) * objectBounds.radius
                                );
                                
                                const rayTarget = object.position.clone().add(targetOffset);
                                const rayDirection = rayTarget.clone().sub(sensorPos).normalize();
                                
                                // Simulate ray casting with realistic physics
                                const raycaster = new THREE.Raycaster(sensorPos, rayDirection);
                                // Filter out sprites/labels and only raycast against mesh objects
                                const meshChildren = [];
                                object.traverse((child) => {
                                    if (child.isMesh && child.geometry && child.material) {
                                        meshChildren.push(child);
                                    }
                                });
                                const intersects = raycaster.intersectObjects(meshChildren, false);
                                
                                if (intersects.length > 0) {
                                    const hit = intersects[0];
                                    const hitDistance = hit.distance;
                                    
                                    // mmWave reflection properties
                                    const reflectionStrength = calculateReflection(object.userData.type, hit.face.normal, rayDirection);
                                    
                                    if (reflectionStrength > 0.1) { // Minimum reflection threshold
                                        // Use world coordinates for point cloud viewer correlation
                                        const worldPoint = hit.point.clone();
                                        
                                        // Add realistic noise based on distance and material
                                        const noiseLevel = 0.02 + (hitDistance * 0.005);
                                        worldPoint.x += (Math.random() - 0.5) * noiseLevel;
                                        worldPoint.y += (Math.random() - 0.5) * noiseLevel;
                                        worldPoint.z += (Math.random() - 0.5) * noiseLevel;
                                        
                                        points.push({
                                            x: worldPoint.x,
                                            y: worldPoint.y,
                                            z: worldPoint.z,
                                            intensity: reflectionStrength,
                                            range: distance,
                                            velocity: 0,
                                            rcs: object.userData.rcs
                                        });
                                    }
                                }
                            }
                        }
                    }
                }
            });
            
            console.log(`[${sensor.id}] Simulation complete: Generated ${points.length} total points`);
            return points;
        }
        
        function publishSensorData(sensorId, pointCloud) {
            const sensorData = {
                sensor_id: sensorId,
                timestamp: Date.now() / 1000,
                point_cloud: pointCloud,
                metadata: {
                    human_present: pointCloud.length > 5,
                    num_points: pointCloud.length,
                    centroid: calculateCentroid(pointCloud)
                }
            };
            
            // Send to ROS2 bridge
            console.log(`[DEBUG] Sending ${pointCloud.length} points for ${sensorId} to /sensor/${sensorId}/pointcloud`);
            fetch(`/sensor/${sensorId}/pointcloud`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(sensorData)
            }).then(response => {
                if (response.ok) {
                    console.log(`✅ Successfully sent ${pointCloud.length} points for ${sensorId}`);
                } else {
                    console.error(`❌ Failed to send data for ${sensorId}:`, response.status);
                }
                return response.json();
            }).then(data => {
                console.log(`[DEBUG] Server response for ${sensorId}:`, data);
            }).catch(err => {
                console.warn(`Bridge not available: ${err.message}`);
            });
        }
        
        function calculateCentroid(pointCloud) {
            if (pointCloud.length === 0) return null;
            
            const centroid = pointCloud.reduce((sum, point) => ({
                x: sum.x + point.x,
                y: sum.y + point.y,
                z: sum.z + point.z
            }), { x: 0, y: 0, z: 0 });
            
            centroid.x /= pointCloud.length;
            centroid.y /= pointCloud.length;
            centroid.z /= pointCloud.length;
            centroid.distance = Math.sqrt(centroid.x * centroid.x + centroid.y * centroid.y + centroid.z * centroid.z);
            
            return centroid;
        }
        
        function updateSensorDisplay(sensorId, pointCloud) {
            const sensorData = document.getElementById('sensor-data');
            const sensorDiv = document.getElementById(`sensor-${sensorId}`) || document.createElement('div');
            sensorDiv.id = `sensor-${sensorId}`;
            sensorDiv.className = 'sensor-info';
            sensorDiv.innerHTML = `
                <strong>${sensorId}:</strong> ${pointCloud.length} points → ROS2,
                ${pointCloud.length > 5 ? 'Objects detected' : 'No detection'}
            `;
            
            if (!document.getElementById(`sensor-${sensorId}`)) {
                sensorData.appendChild(sensorDiv);
            }
        }
        
        function startSimulation() {
            animate();
            // Update sensors based on current rate
            updateSimulationRate();
        }
        
        function updateSimulationRate() {
            if (window.simulationInterval) {
                clearInterval(window.simulationInterval);
            }
            const rate = parseInt(document.getElementById('rateValue').textContent) || 10;
            const intervalMs = 1000 / rate;
            window.simulationInterval = setInterval(simulateSensors, intervalMs);
            console.log(`Simulation rate set to ${rate} Hz (${intervalMs}ms interval)`);
        }
        
        function animate() {
            animationId = requestAnimationFrame(animate);
            if (controls) controls.update();
            if (renderer && scene && camera) renderer.render(scene, camera);
        }
        
        // Global functions for UI controls
        window.addSensor = function() {
            const sensorCount = sensors.size;
            const angle = (sensorCount * Math.PI * 2) / 8;
            const radius = 6;
            const position = new THREE.Vector3(Math.cos(angle) * radius, 1, Math.sin(angle) * radius);
            const rotation = new THREE.Euler(0, -angle, 0);
            const newSensorId = `mmwave${sensorCount + 1}`;
            addSensorToScene(newSensorId, position, rotation);
            console.log(`Added new sensor: ${newSensorId} at angle ${angle.toFixed(2)} rad`);
        }
        
        window.toggleSimulation = function() {
            isRunning = !isRunning;
            const status = isRunning ? 'Running' : 'PAUSED';
            const mode = ros2Mode ? 'ROS2 Mode' : 'Direct MQTT Mode';
            document.getElementById('status').textContent = `${status} (${mode})`;
            
            if (isRunning) {
                updateSimulationRate();
            } else {
                if (window.simulationInterval) {
                    clearInterval(window.simulationInterval);
                }
            }
            console.log(`Simulation ${status.toLowerCase()}`);
        }
        
        window.resetScene = function() {
            // Remove all sensors except defaults
            sensors.forEach((sensor, id) => {
                if (id !== 'mmwave1' && id !== 'mmwave2') {
                    scene.remove(sensor.group);
                    sensors.delete(id);
                }
            });
            
            // Reset object positions to be within sensor FOV
            if (window.sceneObjects && window.sceneObjects.length > 0) {
                window.sceneObjects[0].position.set(-3, 0, 4);    // Human (mmwave1 FOV)
                window.sceneObjects[1].position.set(3, 0.8, -3);  // Table (mmwave2 FOV) 
                window.sceneObjects[2].position.set(-4, 0.3, 2);  // Metal sphere (mmwave1 FOV)
                console.log('Scene objects reset to default positions');
            } else {
                console.log('No scene objects found to reset');
            }
            
            updateSensorCount();
            console.log('Scene reset to default positions');
        }
        
        window.toggleROS2Mode = function() {
            ros2Mode = !ros2Mode;
            const mode = ros2Mode ? 'ROS2 Mode' : 'Direct MQTT Mode';
            const status = isRunning ? 'Running' : 'PAUSED';
            document.getElementById('ros2-status').textContent = ros2Mode ? 'Active' : 'Disabled';
            document.getElementById('status').textContent = `${status} (${mode})`;
            console.log(`Switched to ${mode}`);
        }
        
        window.updateRate = function(value) {
            document.getElementById('rateValue').textContent = value;
            if (isRunning) {
                updateSimulationRate();
            }
            console.log(`Update rate changed to: ${value} Hz`);
        }
        
        window.testDataSend = function() {
            console.log('🔧 Testing data send...');
            const testData = {
                sensor_id: 'test_frontend',
                timestamp: Date.now() / 1000,
                point_cloud: [
                    {x: 1.0, y: 0.5, z: 2.0, intensity: 0.8},
                    {x: 1.1, y: 0.6, z: 2.1, intensity: 0.7},
                    {x: 0.9, y: 0.4, z: 1.9, intensity: 0.9}
                ],
                metadata: {
                    human_present: true,
                    num_points: 3,
                    centroid: {x: 1.0, y: 0.5, z: 2.0}
                }
            };
            
            fetch('/sensor/test_frontend/pointcloud', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(testData)
            }).then(response => response.json())
            .then(data => {
                console.log('✅ Test data sent successfully:', data);
                alert('Test data sent! Check point cloud viewer.');
            }).catch(err => {
                console.error('❌ Test failed:', err);
                alert('Test failed: ' + err.message);
            });
        }
        
        window.debugSimulation = function() {
            console.log('🐛 SIMULATION DEBUG INFO:');
            console.log('- isRunning:', isRunning);
            console.log('- sensors.size:', sensors.size);
            console.log('- window.sceneObjects:', window.sceneObjects);
            console.log('- window.sceneObjects.length:', window.sceneObjects ? window.sceneObjects.length : 'undefined');
            console.log('- window.simulationInterval:', window.simulationInterval);
            console.log('- Sensors:', Array.from(sensors.keys()));
            
            // Debug sensors
            sensors.forEach((sensor, id) => {
                console.log(`- Sensor ${id}:`, {
                    position: sensor.group.position,
                    rotation: sensor.group.rotation,
                    config: sensor.config
                });
            });
            
            if (window.sceneObjects) {
                window.sceneObjects.forEach((obj, i) => {
                    try {
                        console.log(`- Object ${i}:`, {
                            name: obj.userData ? obj.userData.name : 'no name',
                            type: obj.userData ? obj.userData.type : 'no type',
                            position: {x: obj.position.x.toFixed(2), y: obj.position.y.toFixed(2), z: obj.position.z.toFixed(2)},
                            moveable: obj.userData ? obj.userData.moveable : false,
                            rcs: obj.userData ? obj.userData.rcs : 'undefined'
                        });
                    } catch (e) {
                        console.log(`- Object ${i}: Error reading object data:`, e.message);
                    }
                });
            }
            
            alert('Debug info printed to console (F12)');
        }
        
        window.manualSensorRun = function() {
            console.log('▶️ Manual sensor simulation run...');
            if (!isRunning) {
                console.log('⚠️ Simulation is not running! Click Start/Stop first.');
                alert('Simulation is not running! Click Start/Stop first.');
                return;
            }
            
            simulateSensors();
            alert('Manual sensor run completed. Check console and point cloud viewer.');
        }
        
        // Handle window resize
        window.addEventListener('resize', () => {
            if (camera && renderer) {
                const container = document.getElementById('container');
                camera.aspect = container.clientWidth / container.clientHeight;
                camera.updateProjectionMatrix();
                renderer.setSize(container.clientWidth, container.clientHeight);
            }
        });
        
        // Initialize when page loads
        window.addEventListener('load', () => {
            setTimeout(init, 500); // Give Three.js time to load
        });
    </script>
</body>
</html>
            ''')
        
        @self.app.route('/pointcloud-viewer')
        def pointcloud_viewer():
            """Serve the dedicated point cloud visualization window"""
            try:
                viewer_path = os.path.join(os.path.dirname(__file__), 'pointcloud_viewer.html')
                with open(viewer_path, 'r') as f:
                    return f.read()
            except FileNotFoundError:
                return "Point cloud viewer not found", 404

        @self.app.route('/api/pointclouds')
        def get_pointclouds():
            """API endpoint for real-time point cloud data"""
            logger.info(f"Point cloud data requested. Current data: {list(self.latest_pointclouds.keys())}")
            return jsonify(self.latest_pointclouds)

        @self.app.route('/api/debug')
        def debug_status():
            """Debug endpoint to check system status"""
            return jsonify({
                'sensors_with_data': list(self.latest_pointclouds.keys()),
                'total_sensors': len(self.latest_pointclouds),
                'stats': self.stats,
                'latest_data_sample': {k: len(v.get('points', [])) for k, v in self.latest_pointclouds.items()}
            })

        @self.app.route('/sensor/<sensor_id>/pointcloud', methods=['POST'])
        def sensor_pointcloud(sensor_id):
            try:
                data = request.get_json()
                if not data:
                    return jsonify({'error': 'No data provided'}), 400
                
                point_cloud_data = data.get('point_cloud', [])
                metadata = data.get('metadata', {})
                
                # Store data for point cloud viewer
                self.latest_pointclouds[sensor_id] = {
                    'points': point_cloud_data,
                    'metadata': metadata,
                    'timestamp': time.time(),
                    'range': metadata.get('range', '--')
                }
                
                # Create and publish ROS2 PointCloud2 message
                publisher = self.create_sensor_publisher(sensor_id)
                pointcloud_msg = self.threejs_to_pointcloud2(sensor_id, point_cloud_data, metadata)
                publisher.publish(pointcloud_msg)
                
                self.stats['messages_received'] += 1
                self.stats['ros2_published'] += 1
                
                logger.info(f"[{sensor_id}] Published PointCloud2: {len(point_cloud_data)} points to ROS2")
                
                return jsonify({
                    'status': 'success', 
                    'sensor_id': sensor_id,
                    'points_published': len(point_cloud_data),
                    'ros2_topic': f'/mmwave/{sensor_id}/points'
                })
                
            except Exception as e:
                logger.error(f"Error processing point cloud data: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/static/simulation_core.js')
        def serve_simulation_core():
            # Serve the modified simulation core JS file
            try:
                with open('/home/aliza/olympus-sim/sim/threejs/engine/simulation_core.js', 'r') as f:
                    content = f.read()
                
                # Modify the content to use ROS2 endpoint
                modified_content = content.replace(
                    '/sensor/${sensorId}/presence',
                    '/sensor/${sensorId}/pointcloud'
                )
                
                return modified_content, 200, {'Content-Type': 'application/javascript'}
            except FileNotFoundError:
                return "// Simulation core not found", 404, {'Content-Type': 'application/javascript'}
        
        @self.app.route('/status')
        def status():
            runtime = time.time() - self.stats['start_time']
            return jsonify({
                'status': 'running',
                'mode': 'ros2_bridge',
                'runtime_seconds': runtime,
                'messages_received': self.stats['messages_received'],
                'ros2_published': self.stats['ros2_published'],
                'active_sensors': list(self.stats['active_sensors']),
                'ros2_topics': [f'/mmwave/{sensor}/points' for sensor in self.stats['active_sensors']]
            })
    
    def publish_static_transforms(self):
        """Publish static transforms for sensor frames"""
        # This would normally be done by a separate transform publisher
        # For now, we'll rely on the existing static transform publishers in the launcher
        pass
    
    def run_flask(self):
        """Run Flask app in a separate thread"""
        def run_app():
            self.app.run(host='0.0.0.0', port=self.http_port, debug=False)
        
        flask_thread = Thread(target=run_app, daemon=True)
        flask_thread.start()
        logger.info(f"Flask HTTP server running on port {self.http_port}")
    
    def run(self):
        logger.info("Starting Three.js ROS2 Bridge")
        logger.info(f"HTTP server on port {self.http_port}")
        logger.info("Publishing ROS2 PointCloud2 messages to /mmwave/*/points topics")
        
        # Start Flask app
        self.run_flask()
        
        # Start ROS2 spinning
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            logger.info("Shutting down ROS2 bridge")
        finally:
            self.destroy_node()

def main():
    # Initialize ROS2
    rclpy.init()
    
    bridge = ThreeJSROS2Bridge()
    
    # Handle graceful shutdown
    def signal_handler(sig, frame):
        logger.info("Received shutdown signal")
        bridge.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        bridge.run()
    except KeyboardInterrupt:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()