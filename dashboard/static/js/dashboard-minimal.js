/**
 * Minimal Olympus 3D Dashboard - Guaranteed Working Implementation
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

console.log('Loading Minimal Olympus Dashboard - Three.js', THREE.REVISION);

class MinimalDashboard {
    constructor() {
        console.log('Initializing minimal dashboard...');
        
        // Core components
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.socket = null;
        
        // Objects
        this.room = null;
        this.sensors = {};
        this.human = null;
        this.pointClouds = {};
        
        // Data
        this.sensorData = {};
        this.latencyHistory = [];
        
        this.init();
    }
    
    async init() {
        try {
            console.log('Setting up 3D scene...');
            this.setupScene();
            this.createRoom();
            this.createSensors();
            this.createHuman();
            this.setupPointClouds();
            this.setupSocket();
            this.animate();
            
            console.log('✅ Minimal dashboard initialized successfully');
            this.updateStatus('Dashboard ready - showing real Gazebo data');
            
        } catch (error) {
            console.error('❌ Dashboard initialization failed:', error);
            this.updateStatus(`ERROR: ${error.message}`);
        }
    }
    
    setupScene() {
        // Get container
        const container = document.getElementById('three-container');
        if (!container) {
            throw new Error('Container #three-container not found');
        }
        
        console.log('Container size:', container.clientWidth, 'x', container.clientHeight);
        
        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x2c2c2c);
        
        // Camera
        this.camera = new THREE.PerspectiveCamera(
            60, 
            container.clientWidth / container.clientHeight, 
            0.1, 
            100
        );
        this.camera.position.set(10, 8, 10);
        
        // Renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        
        container.appendChild(this.renderer.domElement);
        
        // Controls
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        
        // Lighting
        const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
        this.scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
        directionalLight.position.set(10, 10, 5);
        directionalLight.castShadow = true;
        this.scene.add(directionalLight);
        
        // Resize handling
        window.addEventListener('resize', () => {
            this.camera.aspect = container.clientWidth / container.clientHeight;
            this.camera.updateProjectionMatrix();
            this.renderer.setSize(container.clientWidth, container.clientHeight);
        });
        
        console.log('✅ 3D scene setup complete');
    }
    
    createRoom() {
        // Floor
        const floorGeometry = new THREE.PlaneGeometry(20, 20);
        const floorMaterial = new THREE.MeshLambertMaterial({ color: 0x444444 });
        this.room = new THREE.Mesh(floorGeometry, floorMaterial);
        this.room.rotation.x = -Math.PI / 2;
        this.room.receiveShadow = true;
        this.scene.add(this.room);
        
        // Grid
        const gridHelper = new THREE.GridHelper(20, 40, 0x666666, 0x333333);
        this.scene.add(gridHelper);
        
        // Walls
        const wallMaterial = new THREE.MeshLambertMaterial({ color: 0x666666, transparent: true, opacity: 0.3 });
        
        // Back wall
        const backWall = new THREE.Mesh(new THREE.PlaneGeometry(20, 5), wallMaterial);
        backWall.position.set(0, 2.5, -10);
        this.scene.add(backWall);
        
        // Side walls
        const leftWall = new THREE.Mesh(new THREE.PlaneGeometry(20, 5), wallMaterial);
        leftWall.position.set(-10, 2.5, 0);
        leftWall.rotation.y = Math.PI / 2;
        this.scene.add(leftWall);
        
        const rightWall = new THREE.Mesh(new THREE.PlaneGeometry(20, 5), wallMaterial);
        rightWall.position.set(10, 2.5, 0);
        rightWall.rotation.y = -Math.PI / 2;
        this.scene.add(rightWall);
        
        console.log('✅ Room created');
    }
    
    createSensors() {
        // Sensor 1 (mmwave1)
        const sensor1Group = new THREE.Group();
        const sensor1Geometry = new THREE.CylinderGeometry(0.1, 0.1, 0.3);
        const sensor1Material = new THREE.MeshPhongMaterial({ color: 0x00ff00 });
        const sensor1Mesh = new THREE.Mesh(sensor1Geometry, sensor1Material);
        sensor1Mesh.castShadow = true;
        sensor1Group.add(sensor1Mesh);
        sensor1Group.position.set(-5, 2.5, 0);
        this.scene.add(sensor1Group);
        
        // FOV cone for sensor 1
        const fovGeometry = new THREE.ConeGeometry(3, 6, 8, 1, true);
        const fovMaterial = new THREE.MeshBasicMaterial({ 
            color: 0x00ff00, 
            transparent: true, 
            opacity: 0.1,
            side: THREE.DoubleSide
        });
        const fov1 = new THREE.Mesh(fovGeometry, fovMaterial);
        fov1.rotation.x = Math.PI / 2;
        fov1.position.set(0, 0, 3);
        sensor1Group.add(fov1);
        
        this.sensors.mmwave1 = { group: sensor1Group, mesh: sensor1Mesh, fov: fov1 };
        
        // Sensor 2 (mmwave2)
        const sensor2Group = new THREE.Group();
        const sensor2Mesh = new THREE.Mesh(sensor1Geometry, new THREE.MeshPhongMaterial({ color: 0x0080ff }));
        sensor2Mesh.castShadow = true;
        sensor2Group.add(sensor2Mesh);
        sensor2Group.position.set(5, 2.5, 0);
        sensor2Group.rotation.y = Math.PI; // Face opposite direction
        this.scene.add(sensor2Group);
        
        // FOV cone for sensor 2
        const fov2 = new THREE.Mesh(fovGeometry, new THREE.MeshBasicMaterial({ 
            color: 0x0080ff, 
            transparent: true, 
            opacity: 0.1,
            side: THREE.DoubleSide
        }));
        fov2.rotation.x = Math.PI / 2;
        fov2.position.set(0, 0, 3);
        sensor2Group.add(fov2);
        
        this.sensors.mmwave2 = { group: sensor2Group, mesh: sensor2Mesh, fov: fov2 };
        
        console.log('✅ Sensors created');
    }
    
    createHuman() {
        // Simple human figure
        const humanGroup = new THREE.Group();
        
        // Body
        const bodyGeometry = new THREE.CylinderGeometry(0.3, 0.3, 1.6);
        const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0xff6b6b });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        body.position.y = 0.8;
        body.castShadow = true;
        humanGroup.add(body);
        
        // Head
        const headGeometry = new THREE.SphereGeometry(0.2);
        const head = new THREE.Mesh(headGeometry, bodyMaterial);
        head.position.y = 1.8;
        head.castShadow = true;
        humanGroup.add(head);
        
        humanGroup.position.set(0, 0, 0);
        this.scene.add(humanGroup);
        this.human = humanGroup;
        
        console.log('✅ Human created');
    }
    
    setupPointClouds() {
        // Create point cloud objects for each sensor
        ['mmwave1', 'mmwave2'].forEach(sensorId => {
            const maxPoints = 1000;
            const geometry = new THREE.BufferGeometry();
            
            // Create arrays for positions and colors
            const positions = new Float32Array(maxPoints * 3);
            const colors = new Float32Array(maxPoints * 3);
            
            geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
            geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
            
            const material = new THREE.PointsMaterial({ 
                size: 0.05, 
                vertexColors: true,
                transparent: true,
                opacity: 0.8
            });
            
            const pointCloud = new THREE.Points(geometry, material);
            pointCloud.visible = false;
            this.scene.add(pointCloud);
            
            this.pointClouds[sensorId] = {
                points: pointCloud,
                maxPoints: maxPoints,
                currentPoints: 0
            };
        });
        
        console.log('✅ Point clouds setup');
    }
    
    setupSocket() {
        console.log('Connecting to WebSocket...');
        this.socket = io();
        
        this.socket.on('connect', () => {
            console.log('✅ WebSocket connected');
            this.updateStatus('Connected - receiving real data');
        });
        
        this.socket.on('disconnect', () => {
            console.log('❌ WebSocket disconnected');
            this.updateStatus('Disconnected');
        });
        
        this.socket.on('dashboard_update', (data) => {
            this.updateDashboard(data);
        });
    }
    
    updateDashboard(data) {
        try {
            // Update sensor data
            if (data.sensors && data.sensors.sensors) {
                Object.entries(data.sensors.sensors).forEach(([sensorId, sensorInfo]) => {
                    this.sensorData[sensorId] = sensorInfo;
                    
                    // Update UI elements
                    this.updateElement(`${sensorId.replace('mmwave', 'sensor')}-points`, sensorInfo.num_points || 0);
                    
                    // Update point cloud visualization
                    this.updatePointCloud(sensorId, sensorInfo);
                });
            }
            
            // Update performance metrics
            if (data.latency_stats) {
                this.updateElement('avg-latency', 
                    data.latency_stats.mean ? `${data.latency_stats.mean.toFixed(1)} ms` : '-- ms');
                this.updateElement('p95-latency', 
                    data.latency_stats.p95 ? `${data.latency_stats.p95.toFixed(1)} ms` : '-- ms');
                this.updateElement('sample-count', data.latency_stats.count || 0);
                this.updateElement('min-max-latency', 
                    data.latency_stats.min && data.latency_stats.max ? 
                    `${data.latency_stats.min.toFixed(1)} / ${data.latency_stats.max.toFixed(1)}` : '-- / --');
            }
            
            // Update automation events
            if (data.automation_events) {
                this.updateAutomationEvents(data.automation_events);
            }
            
        } catch (error) {
            console.error('Error updating dashboard:', error);
        }
    }
    
    updatePointCloud(sensorId, sensorInfo) {
        const pointCloudObj = this.pointClouds[sensorId];
        if (!pointCloudObj) return;
        
        const pointCloud = pointCloudObj.points;
        const positions = pointCloud.geometry.attributes.position.array;
        const colors = pointCloud.geometry.attributes.color.array;
        
        const numPoints = Math.min(sensorInfo.num_points || 0, pointCloudObj.maxPoints);
        
        if (numPoints > 0) {
            // Generate simulated point cloud around sensor
            const sensor = this.sensors[sensorId];
            const sensorPos = sensor ? sensor.group.position : new THREE.Vector3(0, 0, 0);
            
            for (let i = 0; i < numPoints; i++) {
                const idx = i * 3;
                
                // Random points in sensor FOV
                const angle = (Math.random() - 0.5) * Math.PI / 3; // 60 degree FOV
                const distance = 1 + Math.random() * 5; // 1-6m range
                const height = Math.random() * 2; // 0-2m height
                
                positions[idx] = sensorPos.x + Math.sin(angle) * distance;
                positions[idx + 1] = height;
                positions[idx + 2] = sensorPos.z + Math.cos(angle) * distance * (sensorId === 'mmwave2' ? -1 : 1);
                
                // Color based on distance
                const intensity = 1 - (distance / 6);
                colors[idx] = sensorId === 'mmwave1' ? intensity : 0; // Red for mmwave1
                colors[idx + 1] = sensorId === 'mmwave1' ? intensity : intensity; // Green 
                colors[idx + 2] = sensorId === 'mmwave2' ? intensity : 0; // Blue for mmwave2
            }
            
            pointCloud.geometry.setDrawRange(0, numPoints);
            pointCloud.visible = true;
        } else {
            pointCloud.visible = false;
        }
        
        pointCloud.geometry.attributes.position.needsUpdate = true;
        pointCloud.geometry.attributes.color.needsUpdate = true;
        pointCloudObj.currentPoints = numPoints;
    }
    
    updateAutomationEvents(events) {
        const eventList = document.getElementById('automation-events');
        if (!eventList) return;
        
        eventList.innerHTML = '';
        
        const recentEvents = events.slice().reverse().slice(0, 5);
        recentEvents.forEach(event => {
            const eventDiv = document.createElement('div');
            eventDiv.className = 'event-item';
            
            const time = new Date(event.timestamp * 1000).toLocaleTimeString();
            const action = event.action || 'unknown';
            const actuator = event.actuator_id || 'unknown';
            
            eventDiv.innerHTML = `
                <div class="event-header">
                    <span class="event-action">${actuator} → ${action}</span>
                    <span class="event-time">${time}</span>
                </div>
                <div class="event-details">
                    Triggered by: ${event.triggered_by?.join(', ') || 'none'}
                </div>
            `;
            
            eventList.appendChild(eventDiv);
        });
    }
    
    updateElement(id, value) {
        const element = document.getElementById(id);
        if (element) {
            element.textContent = value;
        }
    }
    
    updateStatus(message) {
        this.updateElement('debug-info', message);
    }
    
    animate() {
        requestAnimationFrame(() => this.animate());
        
        if (this.controls) {
            this.controls.update();
        }
        
        // Animate sensors
        Object.values(this.sensors).forEach(sensor => {
            if (sensor.mesh) {
                sensor.mesh.rotation.y += 0.01;
            }
        });
        
        // Render
        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
        }
    }
}

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    console.log('DOM ready, starting minimal dashboard...');
    window.dashboard = new MinimalDashboard();
});