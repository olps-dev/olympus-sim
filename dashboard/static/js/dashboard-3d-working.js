/**
 * Olympus 3D Dashboard - Working Implementation
 * Using proper Three.js ES modules
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

console.log('Starting Olympus 3D Dashboard with Three.js', THREE.REVISION);

class Olympus3DDashboard {
    constructor() {
        console.log('Initializing Olympus 3D Dashboard...');
        
        // Three.js scene components
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        
        // Sensor FOV renderer
        this.sensorScene = null;
        this.sensorCamera = null;
        this.sensorRenderer = null;
        
        // 3D objects
        this.sensors = {};
        this.human = null;
        this.pointClouds = {};
        this.lamp = null;
        this.sensorFOVObjects = {};
        
        // Data management
        this.socket = null;
        this.latencyChart = null;
        this.sensorData = {};
        this.latencyHistory = [];
        this.selectedSensor = 'mmwave1';
        
        // Animation
        this.animationId = null;
        this.isInitialized = false;
        
        this.init();
    }
    
    init() {
        try {
            this.updateDebugInfo('Initializing 3D scenes...');
            this.setupMainScene();
            this.setupSensorView();
            this.createEnvironment();
            this.setupControls();
            this.setupWebSocket();
            this.setupCharts();
            this.animate();
            
            this.isInitialized = true;
            this.updateDebugInfo('3D Dashboard initialized successfully');
            console.log('Olympus 3D Dashboard initialized successfully');
            
        } catch (error) {
            console.error('Failed to initialize dashboard:', error);
            this.showError(`Initialization failed: ${error.message}`);
        }
    }
    
    updateDebugInfo(message) {
        const debugElement = document.getElementById('debug-info');
        if (debugElement) {
            debugElement.textContent = message;
        }
        console.log(message);
    }
    
    showError(message) {
        const debugElement = document.getElementById('debug-info');
        if (debugElement) {
            debugElement.textContent = `ERROR: ${message}`;
            debugElement.style.color = '#f44336';
        }
        console.error(message);
    }
    
    setupMainScene() {
        const container = document.getElementById('three-container');
        if (!container) {
            throw new Error('Three.js container not found');
        }
        
        console.log('Container dimensions:', container.clientWidth, 'x', container.clientHeight);
        
        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x2a2a2a);
        this.scene.fog = new THREE.Fog(0x2a2a2a, 10, 50);
        
        // Camera
        const aspect = container.clientWidth / container.clientHeight;
        this.camera = new THREE.PerspectiveCamera(75, aspect, 0.1, 1000);
        this.camera.position.set(8, 6, 8);
        this.camera.lookAt(0, 0, 0);
        
        // Renderer
        this.renderer = new THREE.WebGLRenderer({ 
            antialias: true,
            alpha: false
        });
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.outputColorSpace = THREE.SRGBColorSpace;
        
        container.appendChild(this.renderer.domElement);
        
        this.setupLighting();
        
        // Handle window resize
        window.addEventListener('resize', () => this.onWindowResize());
        
        this.updateDebugInfo('Main 3D scene created');
    }
    
    setupSensorView() {
        const container = document.getElementById('sensor-fov-container');
        if (!container) {
            console.warn('Sensor FOV container not found');
            return;
        }
        
        // Sensor FOV Scene
        this.sensorScene = new THREE.Scene();
        this.sensorScene.background = new THREE.Color(0x1a1a1a);
        
        // Top-down camera for sensor FOV
        this.sensorCamera = new THREE.OrthographicCamera(-6, 6, 6, -6, 0.1, 100);
        this.sensorCamera.position.set(0, 10, 0);
        this.sensorCamera.lookAt(0, 0, 0);
        
        // Sensor FOV Renderer
        this.sensorRenderer = new THREE.WebGLRenderer({ antialias: true });
        this.sensorRenderer.setSize(container.clientWidth, container.clientHeight);
        this.sensorRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        
        container.appendChild(this.sensorRenderer.domElement);
        
        this.createSensorFOV();
        
        this.updateDebugInfo('Sensor FOV view created');
    }
    
    setupLighting() {
        // Ambient light
        const ambientLight = new THREE.AmbientLight(0x404040, 0.4);
        this.scene.add(ambientLight);
        
        // Main directional light
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 5);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        directionalLight.shadow.camera.near = 0.5;
        directionalLight.shadow.camera.far = 50;
        directionalLight.shadow.camera.left = -10;
        directionalLight.shadow.camera.right = 10;
        directionalLight.shadow.camera.top = 10;
        directionalLight.shadow.camera.bottom = -10;
        this.scene.add(directionalLight);
        
        // Accent lights
        const light1 = new THREE.PointLight(0x00bcd4, 0.3, 10);
        light1.position.set(-4, 3, 0);
        this.scene.add(light1);
        
        const light2 = new THREE.PointLight(0x4fc3f7, 0.3, 10);
        light2.position.set(4, 3, 0);
        this.scene.add(light2);
    }
    
    createEnvironment() {
        this.updateDebugInfo('Creating 3D environment...');
        
        // Room floor
        const floorGeometry = new THREE.PlaneGeometry(12, 12);
        const floorMaterial = new THREE.MeshLambertMaterial({
            color: 0x555555,
            transparent: false
        });
        const floor = new THREE.Mesh(floorGeometry, floorMaterial);
        floor.rotation.x = -Math.PI / 2;
        floor.receiveShadow = true;
        this.scene.add(floor);
        
        // Grid helper
        const gridHelper = new THREE.GridHelper(12, 24, 0x666666, 0x333333);
        this.scene.add(gridHelper);
        
        // Axis helper
        const axesHelper = new THREE.AxesHelper(2);
        this.scene.add(axesHelper);
        
        // Create walls
        this.createWalls();
        
        // Create sensors
        this.createSensor('mmwave1', -4, 2.5, 0, 0x00bcd4);
        this.createSensor('mmwave2', 4, 2.5, 0, 0x4fc3f7);
        
        // Create human figure
        this.createHuman();
        
        // Create lamp
        this.createLamp();
        
        this.updateDebugInfo('3D environment created');
    }
    
    createWalls() {
        const wallMaterial = new THREE.LineBasicMaterial({ 
            color: 0x666666,
            transparent: true,
            opacity: 0.5
        });
        
        // Create room outline points
        const points = [
            new THREE.Vector3(-6, 0, -6),
            new THREE.Vector3(6, 0, -6),
            new THREE.Vector3(6, 0, 6),
            new THREE.Vector3(-6, 0, 6),
            new THREE.Vector3(-6, 0, -6),
            new THREE.Vector3(-6, 3, -6),
            new THREE.Vector3(6, 3, -6),
            new THREE.Vector3(6, 3, 6),
            new THREE.Vector3(-6, 3, 6),
            new THREE.Vector3(-6, 3, -6)
        ];
        
        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const line = new THREE.Line(geometry, wallMaterial);
        this.scene.add(line);
        
        // Vertical lines for corners
        const verticals = [
            [new THREE.Vector3(6, 0, -6), new THREE.Vector3(6, 3, -6)],
            [new THREE.Vector3(6, 0, 6), new THREE.Vector3(6, 3, 6)],
            [new THREE.Vector3(-6, 0, 6), new THREE.Vector3(-6, 3, 6)]
        ];
        
        verticals.forEach(points => {
            const geo = new THREE.BufferGeometry().setFromPoints(points);
            const line = new THREE.Line(geo, wallMaterial);
            this.scene.add(line);
        });
    }
    
    createSensor(id, x, y, z, color) {
        const group = new THREE.Group();
        
        // Sensor body
        const sensorGeometry = new THREE.CylinderGeometry(0.1, 0.15, 0.3, 8);
        const sensorMaterial = new THREE.MeshPhongMaterial({ 
            color: color,
            shininess: 30
        });
        const sensorMesh = new THREE.Mesh(sensorGeometry, sensorMaterial);
        sensorMesh.castShadow = true;
        
        // Sensor base
        const baseGeometry = new THREE.CylinderGeometry(0.2, 0.2, 0.05, 8);
        const baseMaterial = new THREE.MeshPhongMaterial({ color: 0x333333 });
        const base = new THREE.Mesh(baseGeometry, baseMaterial);
        base.position.y = -0.175;
        base.castShadow = true;
        
        // Detection cone
        const coneGeometry = new THREE.ConeGeometry(3, 6, 16, 1, true);
        const coneMaterial = new THREE.MeshBasicMaterial({
            color: color,
            wireframe: true,
            transparent: true,
            opacity: 0.15
        });
        const cone = new THREE.Mesh(coneGeometry, coneMaterial);
        cone.rotation.x = Math.PI;
        cone.position.y = -3;
        
        // LED indicator
        const ledGeometry = new THREE.SphereGeometry(0.02, 8, 8);
        const ledMaterial = new THREE.MeshBasicMaterial({ 
            color: 0xff0000,
            transparent: true,
            opacity: 0.8
        });
        const led = new THREE.Mesh(ledGeometry, ledMaterial);
        led.position.set(0, 0.1, 0.12);
        
        group.add(sensorMesh);
        group.add(base);
        group.add(cone);
        group.add(led);
        group.position.set(x, y, z);
        
        this.scene.add(group);
        this.sensors[id] = {
            group: group,
            mesh: sensorMesh,
            cone: cone,
            led: led,
            active: false
        };
        
        // Create point cloud
        this.createPointCloud(id, color);
    }
    
    createPointCloud(sensorId, color) {
        const maxPoints = 100;
        const geometry = new THREE.BufferGeometry();
        
        const positions = new Float32Array(maxPoints * 3);
        const colors = new Float32Array(maxPoints * 3);
        
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        
        const material = new THREE.PointsMaterial({
            size: 0.05,
            vertexColors: true,
            transparent: true,
            opacity: 0.8,
            sizeAttenuation: true
        });
        
        const pointCloud = new THREE.Points(geometry, material);
        pointCloud.visible = false;
        this.scene.add(pointCloud);
        
        this.pointClouds[sensorId] = {
            points: pointCloud,
            maxPoints: maxPoints,
            color: new THREE.Color(color)
        };
    }
    
    createHuman() {
        const group = new THREE.Group();
        
        // Human body
        const bodyGeometry = new THREE.CapsuleGeometry(0.25, 1.4, 4, 8);
        const bodyMaterial = new THREE.MeshPhongMaterial({ 
            color: 0xff6b6b,
            shininess: 10
        });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        body.position.y = 0.85;
        body.castShadow = true;
        
        // Head
        const headGeometry = new THREE.SphereGeometry(0.15, 16, 16);
        const head = new THREE.Mesh(headGeometry, bodyMaterial);
        head.position.y = 1.8;
        head.castShadow = true;
        
        group.add(body);
        group.add(head);
        group.position.set(0, 0, 0);
        
        this.scene.add(group);
        this.human = group;
    }
    
    createLamp() {
        const group = new THREE.Group();
        
        // Lamp post
        const postGeometry = new THREE.CylinderGeometry(0.05, 0.05, 3, 8);
        const postMaterial = new THREE.MeshPhongMaterial({ color: 0x444444 });
        const post = new THREE.Mesh(postGeometry, postMaterial);
        post.position.y = 1.5;
        post.castShadow = true;
        
        // Lamp shade
        const shadeGeometry = new THREE.CylinderGeometry(0.3, 0.2, 0.4, 8);
        const shadeMaterial = new THREE.MeshPhongMaterial({ color: 0x666666 });
        const shade = new THREE.Mesh(shadeGeometry, shadeMaterial);
        shade.position.y = 3.0;
        
        // Light bulb
        const bulbGeometry = new THREE.SphereGeometry(0.1, 16, 16);
        const bulbMaterial = new THREE.MeshBasicMaterial({ 
            color: 0xffeb3b,
            transparent: true,
            opacity: 0.3
        });
        const bulb = new THREE.Mesh(bulbGeometry, bulbMaterial);
        bulb.position.y = 3.0;
        
        group.add(post);
        group.add(shade);
        group.add(bulb);
        group.position.set(-2, 0, -2);
        
        this.scene.add(group);
        this.lamp = { 
            group: group, 
            bulb: bulb, 
            material: bulbMaterial,
            isOn: false 
        };
    }
    
    createSensorFOV() {
        if (!this.sensorScene) return;
        
        // Floor grid
        const gridHelper = new THREE.GridHelper(12, 24, 0x444444, 0x222222);
        this.sensorScene.add(gridHelper);
        
        // Sensor FOV markers
        this.createSensorFOVMarkers();
    }
    
    createSensorFOVMarkers() {
        // Sensor 1 FOV
        const sensor1FOV = this.createFOVCone(-4, 0, 0x00bcd4);
        this.sensorScene.add(sensor1FOV);
        
        // Sensor 2 FOV
        const sensor2FOV = this.createFOVCone(4, 0, 0x4fc3f7);
        this.sensorScene.add(sensor2FOV);
        
        // Human marker
        const humanGeometry = new THREE.CircleGeometry(0.3, 16);
        const humanMaterial = new THREE.MeshBasicMaterial({ 
            color: 0xff6b6b,
            transparent: true,
            opacity: 0.8
        });
        const humanMarker = new THREE.Mesh(humanGeometry, humanMaterial);
        humanMarker.rotation.x = -Math.PI / 2;
        humanMarker.position.set(0, 0.01, 0);
        this.sensorScene.add(humanMarker);
        
        this.sensorFOVObjects = {
            sensor1: sensor1FOV,
            sensor2: sensor2FOV,
            human: humanMarker
        };
    }
    
    createFOVCone(x, z, color) {
        const group = new THREE.Group();
        
        // Sensor marker
        const sensorGeometry = new THREE.CircleGeometry(0.2, 8);
        const sensorMaterial = new THREE.MeshBasicMaterial({ color: color });
        const sensor = new THREE.Mesh(sensorGeometry, sensorMaterial);
        sensor.rotation.x = -Math.PI / 2;
        sensor.position.set(x, 0.02, z);
        
        // FOV cone shape
        const angle = Math.PI / 3; // 60 degrees
        const range = 6;
        const segments = 20;
        
        const fovGeometry = new THREE.BufferGeometry();
        const vertices = [];
        const indices = [];
        
        // Center point
        vertices.push(x, 0.01, z);
        
        // Arc points
        for (let i = 0; i <= segments; i++) {
            const theta = -angle/2 + (angle * i / segments);
            const px = x + Math.sin(theta) * range;
            const pz = z + Math.cos(theta) * range;
            vertices.push(px, 0.01, pz);
            
            if (i < segments) {
                indices.push(0, i + 1, i + 2);
            }
        }
        
        fovGeometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
        fovGeometry.setIndex(indices);
        
        const fovMaterial = new THREE.MeshBasicMaterial({
            color: color,
            transparent: true,
            opacity: 0.2,
            side: THREE.DoubleSide
        });
        
        const fov = new THREE.Mesh(fovGeometry, fovMaterial);
        
        group.add(sensor);
        group.add(fov);
        
        return group;
    }
    
    setupControls() {
        // OrbitControls for main view
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.minDistance = 3;
        this.controls.maxDistance = 20;
        this.controls.maxPolarAngle = Math.PI / 2 + 0.3;
        
        // UI Controls
        this.setupUIControls();
    }
    
    setupUIControls() {
        // Human position controls
        const humanXSlider = document.getElementById('human-x');
        const humanZSlider = document.getElementById('human-z');
        const sensor1YSlider = document.getElementById('sensor1-y');
        const sensor2YSlider = document.getElementById('sensor2-y');
        
        if (humanXSlider) {
            humanXSlider.addEventListener('input', (e) => {
                const value = parseFloat(e.target.value);
                if (this.human) {
                    this.human.position.x = value;
                }
                if (this.sensorFOVObjects && this.sensorFOVObjects.human) {
                    this.sensorFOVObjects.human.position.x = value;
                }
                document.getElementById('human-x-value').textContent = value.toFixed(1);
                this.updateDetection();
            });
        }
        
        if (humanZSlider) {
            humanZSlider.addEventListener('input', (e) => {
                const value = parseFloat(e.target.value);
                if (this.human) {
                    this.human.position.z = value;
                }
                if (this.sensorFOVObjects && this.sensorFOVObjects.human) {
                    this.sensorFOVObjects.human.position.z = value;
                }
                document.getElementById('human-z-value').textContent = value.toFixed(1);
                this.updateDetection();
            });
        }
        
        if (sensor1YSlider) {
            sensor1YSlider.addEventListener('input', (e) => {
                const value = parseFloat(e.target.value);
                if (this.sensors['mmwave1']) {
                    this.sensors['mmwave1'].group.position.y = value;
                }
                document.getElementById('sensor1-y-value').textContent = value.toFixed(1);
                this.updateDetection();
            });
        }
        
        if (sensor2YSlider) {
            sensor2YSlider.addEventListener('input', (e) => {
                const value = parseFloat(e.target.value);
                if (this.sensors['mmwave2']) {
                    this.sensors['mmwave2'].group.position.y = value;
                }
                document.getElementById('sensor2-y-value').textContent = value.toFixed(1);
                this.updateDetection();
            });
        }
        
        // Buttons
        const resetBtn = document.getElementById('reset-scene');
        const randomBtn = document.getElementById('randomize-scene');
        
        if (resetBtn) {
            resetBtn.addEventListener('click', () => this.resetScene());
        }
        
        if (randomBtn) {
            randomBtn.addEventListener('click', () => this.randomizeScene());
        }
    }
    
    setupWebSocket() {
        this.socket = io();
        
        this.socket.on('connect', () => {
            console.log('WebSocket connected');
            this.updateStatusIndicator('ws-status', true);
            this.updateStatusIndicator('simulation-status', true);
        });
        
        this.socket.on('disconnect', () => {
            console.log('WebSocket disconnected');
            this.updateStatusIndicator('ws-status', false);
        });
        
        this.socket.on('dashboard_update', (data) => {
            this.handleDashboardUpdate(data);
        });
        
        this.socket.on('sensor_update', (data) => {
            this.handleSensorUpdate(data);
        });
        
        this.socket.on('automation_event', (data) => {
            this.handleAutomationEvent(data);
        });
    }
    
    setupCharts() {
        const ctx = document.getElementById('latency-chart');
        if (!ctx) {
            console.warn('Latency chart canvas not found');
            return;
        }
        
        this.latencyChart = new Chart(ctx.getContext('2d'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Latency (ms)',
                    data: [],
                    borderColor: '#00bcd4',
                    backgroundColor: 'rgba(0, 188, 212, 0.1)',
                    borderWidth: 2,
                    tension: 0.4,
                    pointRadius: 2,
                    pointHoverRadius: 4
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: { display: false }
                },
                scales: {
                    x: { display: false },
                    y: {
                        beginAtZero: true,
                        grid: { color: 'rgba(255, 255, 255, 0.1)' },
                        ticks: {
                            color: '#999',
                            callback: function(value) {
                                return value + 'ms';
                            }
                        }
                    }
                },
                animation: { duration: 0 }
            }
        });
    }
    
    handleDashboardUpdate(data) {
        // Update MQTT status
        this.updateStatusIndicator('mqtt-status', data.mqtt_connected);
        
        // Update performance metrics
        if (data.latency_stats && data.latency_stats.count > 0) {
            this.updateElement('avg-latency', data.latency_stats.mean.toFixed(1) + ' ms');
            this.updateElement('p95-latency', data.latency_stats.p95.toFixed(1) + ' ms');
            this.updateElement('sample-count', data.latency_stats.count);
            this.updateElement('min-max-latency', 
                data.latency_stats.min.toFixed(1) + ' / ' + data.latency_stats.max.toFixed(1));
        }
        
        // Update sensors
        if (data.sensors && data.sensors.sensors) {
            this.updateSensorDisplay(data.sensors.sensors);
        }
        
        // Update latency chart
        if (data.latency_history && data.latency_history.length > 0) {
            this.updateLatencyChart(data.latency_history);
        }
        
        // Update automation events
        if (data.automation_events && data.automation_events.length > 0) {
            this.updateAutomationEvents(data.automation_events);
        }
    }
    
    handleSensorUpdate(data) {
        const sensorId = data.sensor_id;
        const sensorData = data.data;
        
        this.sensorData[sensorId] = sensorData;
        this.updateSensorVisualization(sensorId, sensorData);
        this.updatePointCloudSimulation(sensorId, sensorData);
    }
    
    handleAutomationEvent(event) {
        console.log('Automation event:', event);
        
        if (event.actuator_id === 'lamp_hall') {
            this.updateLampState(event.action === 'on');
        }
        
        if (event.latency_ms > 0) {
            this.addLatencyPoint(event.latency_ms);
        }
    }
    
    updateElement(id, value) {
        const element = document.getElementById(id);
        if (element) {
            element.textContent = value;
        }
    }
    
    updateSensorDisplay(sensors) {
        const sensorList = document.getElementById('sensor-list');
        if (!sensorList) return;
        
        sensorList.innerHTML = '';
        
        Object.entries(sensors).forEach(([sensorId, sensor]) => {
            const sensorCard = document.createElement('div');
            sensorCard.className = `sensor-card ${sensor.human_present ? 'active' : ''}`;
            sensorCard.innerHTML = `
                <div class="sensor-header">
                    <div class="sensor-name">${sensorId}</div>
                    <div class="sensor-status ${sensor.human_present ? 'active' : ''}">
                        ${sensor.human_present ? 'DETECTING' : 'IDLE'}
                    </div>
                </div>
                <div class="sensor-details">
                    <div>Battery: ${sensor.battery_level_mah.toFixed(0)} mAh</div>
                    <div>Points: ${sensor.num_points || 0}</div>
                </div>
            `;
            
            sensorCard.addEventListener('click', () => {
                this.selectSensor(sensorId);
            });
            
            sensorList.appendChild(sensorCard);
        });
    }
    
    selectSensor(sensorId) {
        this.selectedSensor = sensorId;
        this.updateElement('selected-sensor', `Select sensor: ${sensorId}`);
    }
    
    updateSensorVisualization(sensorId, sensorData) {
        if (!this.sensors[sensorId]) return;
        
        const sensor = this.sensors[sensorId];
        const isActive = sensorData.human_present;
        
        // Update LED color
        if (isActive) {
            sensor.led.material.color.setHex(0x00ff00); // Green
            sensor.cone.material.opacity = 0.3;
        } else {
            sensor.led.material.color.setHex(0xff0000); // Red
            sensor.cone.material.opacity = 0.1;
        }
        
        sensor.active = isActive;
    }
    
    updatePointCloudSimulation(sensorId, sensorData) {
        if (!this.pointClouds[sensorId] || !this.human) return;
        
        const pointCloudObj = this.pointClouds[sensorId];
        const pointCloud = pointCloudObj.points;
        const positions = pointCloud.geometry.attributes.position.array;
        const colors = pointCloud.geometry.attributes.color.array;
        
        if (sensorData.human_present && sensorData.num_points > 0) {
            const numPoints = Math.min(sensorData.num_points, pointCloudObj.maxPoints);
            const humanPos = this.human.position;
            
            this.updateElement(`${sensorId.replace('mmwave', 'sensor')}-points`, numPoints);
            
            // Generate points around human
            for (let i = 0; i < numPoints; i++) {
                const idx = i * 3;
                
                const spread = 0.4;
                positions[idx] = humanPos.x + (Math.random() - 0.5) * spread;
                positions[idx + 1] = humanPos.y + Math.random() * 1.8 + 0.2;
                positions[idx + 2] = humanPos.z + (Math.random() - 0.5) * spread;
                
                const color = pointCloudObj.color;
                colors[idx] = color.r;
                colors[idx + 1] = color.g;
                colors[idx + 2] = color.b;
            }
            
            // Clear remaining points
            for (let i = numPoints; i < pointCloudObj.maxPoints; i++) {
                const idx = i * 3;
                positions[idx] = positions[idx + 1] = positions[idx + 2] = 0;
                colors[idx] = colors[idx + 1] = colors[idx + 2] = 0;
            }
            
            pointCloud.geometry.setDrawRange(0, numPoints);
            pointCloud.visible = true;
        } else {
            pointCloud.visible = false;
            this.updateElement(`${sensorId.replace('mmwave', 'sensor')}-points`, '0');
        }
        
        pointCloud.geometry.attributes.position.needsUpdate = true;
        pointCloud.geometry.attributes.color.needsUpdate = true;
    }
    
    updateLatencyChart(latencyHistory) {
        if (!this.latencyChart) return;
        
        const recent = latencyHistory.slice(-20);
        const labels = recent.map((_, i) => i.toString());
        const data = recent.map(point => point.latency_ms);
        
        this.latencyChart.data.labels = labels;
        this.latencyChart.data.datasets[0].data = data;
        this.latencyChart.update('none');
    }
    
    addLatencyPoint(latencyMs) {
        this.latencyHistory.push({ latency_ms: latencyMs, timestamp: Date.now() });
        
        if (this.latencyHistory.length > 50) {
            this.latencyHistory = this.latencyHistory.slice(-20);
        }
        
        this.updateLatencyChart(this.latencyHistory);
    }
    
    updateAutomationEvents(events) {
        const eventList = document.getElementById('automation-events');
        if (!eventList) return;
        
        eventList.innerHTML = '';
        
        const recentEvents = events.slice().reverse().slice(0, 8);
        
        recentEvents.forEach(event => {
            const eventItem = document.createElement('div');
            eventItem.className = 'event-item';
            
            const eventTime = new Date(event.timestamp * 1000).toLocaleTimeString();
            const triggeredBy = event.triggered_by.join(', ') || 'Manual';
            
            eventItem.innerHTML = `
                <div class="event-header">
                    <div class="event-action">${event.actuator_id} → ${event.action.toUpperCase()}</div>
                    <div class="event-time">${eventTime}</div>
                </div>
                <div class="event-details">
                    Triggered by: ${triggeredBy}
                    ${event.latency_ms > 0 ? ` (${event.latency_ms.toFixed(1)}ms)` : ''}
                </div>
            `;
            
            eventList.appendChild(eventItem);
        });
    }
    
    updateLampState(isOn) {
        if (!this.lamp) return;
        
        this.lamp.isOn = isOn;
        
        if (isOn) {
            this.lamp.material.opacity = 0.9;
            this.lamp.material.color.setHex(0xffeb3b);
        } else {
            this.lamp.material.opacity = 0.3;
            this.lamp.material.color.setHex(0x666666);
        }
    }
    
    updateDetection() {
        // Simulate detection based on human position
        if (!this.human || !this.sensors) return;
        
        Object.entries(this.sensors).forEach(([sensorId, sensor]) => {
            const humanPos = this.human.position;
            const sensorPos = sensor.group.position;
            
            const distance = humanPos.distanceTo(sensorPos);
            const isInRange = distance < 6;
            
            // Simple FOV check
            const toHuman = humanPos.clone().sub(sensorPos).normalize();
            const forward = new THREE.Vector3(0, 0, 1);
            const angle = Math.acos(Math.max(-1, Math.min(1, toHuman.dot(forward))));
            const isInFOV = angle < Math.PI / 6;
            
            const detected = isInRange && isInFOV;
            
            // Update visualization
            if (detected) {
                sensor.led.material.color.setHex(0x00ff00);
                sensor.cone.material.opacity = 0.3;
            } else {
                sensor.led.material.color.setHex(0xff0000);
                sensor.cone.material.opacity = 0.1;
            }
        });
    }
    
    updateStatusIndicator(id, connected) {
        const indicator = document.getElementById(id);
        if (!indicator) return;
        
        if (connected) {
            indicator.classList.add('connected');
            indicator.classList.remove('disconnected');
        } else {
            indicator.classList.add('disconnected');
            indicator.classList.remove('connected');
        }
    }
    
    resetScene() {
        if (!this.human || !this.sensors) return;
        
        // Reset positions
        this.human.position.set(0, 0, 0);
        this.sensors['mmwave1'].group.position.set(-4, 2.5, 0);
        this.sensors['mmwave2'].group.position.set(4, 2.5, 0);
        
        // Reset UI
        this.updateElement('human-x-value', '0.0');
        this.updateElement('human-z-value', '0.0');
        this.updateElement('sensor1-y-value', '2.5');
        this.updateElement('sensor2-y-value', '2.5');
        
        const sliders = ['human-x', 'human-z', 'sensor1-y', 'sensor2-y'];
        const values = [0, 0, 2.5, 2.5];
        sliders.forEach((id, i) => {
            const element = document.getElementById(id);
            if (element) element.value = values[i];
        });
        
        if (this.sensorFOVObjects) {
            this.sensorFOVObjects.human.position.set(0, 0.01, 0);
        }
        
        this.updateDetection();
    }
    
    randomizeScene() {
        if (!this.human || !this.sensors) return;
        
        const humanX = (Math.random() - 0.5) * 8;
        const humanZ = (Math.random() - 0.5) * 8;
        const sensor1Y = 1 + Math.random() * 3;
        const sensor2Y = 1 + Math.random() * 3;
        
        this.human.position.set(humanX, 0, humanZ);
        this.sensors['mmwave1'].group.position.y = sensor1Y;
        this.sensors['mmwave2'].group.position.y = sensor2Y;
        
        // Update UI
        this.updateElement('human-x-value', humanX.toFixed(1));
        this.updateElement('human-z-value', humanZ.toFixed(1));
        this.updateElement('sensor1-y-value', sensor1Y.toFixed(1));
        this.updateElement('sensor2-y-value', sensor2Y.toFixed(1));
        
        const updates = [
            { id: 'human-x', value: humanX },
            { id: 'human-z', value: humanZ },
            { id: 'sensor1-y', value: sensor1Y },
            { id: 'sensor2-y', value: sensor2Y }
        ];
        
        updates.forEach(update => {
            const element = document.getElementById(update.id);
            if (element) element.value = update.value;
        });
        
        if (this.sensorFOVObjects) {
            this.sensorFOVObjects.human.position.set(humanX, 0.01, humanZ);
        }
        
        this.updateDetection();
    }
    
    setupEventListeners() {
        // Human position controls
        const humanXSlider = document.getElementById('human-x');
        const humanZSlider = document.getElementById('human-z');
        const humanXValue = document.getElementById('human-x-value');
        const humanZValue = document.getElementById('human-z-value');
        
        if (humanXSlider) {
            humanXSlider.addEventListener('input', (e) => {
                const value = parseFloat(e.target.value);
                if (humanXValue) humanXValue.textContent = value.toFixed(1);
                if (this.human) {
                    this.human.position.x = value;
                    if (this.sensorFOVObjects && this.sensorFOVObjects.human) {
                        this.sensorFOVObjects.human.position.x = value;
                    }
                    this.updateDetection();
                }
            });
        }
        
        if (humanZSlider) {
            humanZSlider.addEventListener('input', (e) => {
                const value = parseFloat(e.target.value);
                if (humanZValue) humanZValue.textContent = value.toFixed(1);
                if (this.human) {
                    this.human.position.z = value;
                    if (this.sensorFOVObjects && this.sensorFOVObjects.human) {
                        this.sensorFOVObjects.human.position.z = value;
                    }
                    this.updateDetection();
                }
            });
        }
        
        // Sensor position controls
        const sensor1YSlider = document.getElementById('sensor1-y');
        const sensor2YSlider = document.getElementById('sensor2-y');
        const sensor1YValue = document.getElementById('sensor1-y-value');
        const sensor2YValue = document.getElementById('sensor2-y-value');
        
        if (sensor1YSlider) {
            sensor1YSlider.addEventListener('input', (e) => {
                const value = parseFloat(e.target.value);
                if (sensor1YValue) sensor1YValue.textContent = value.toFixed(1);
                if (this.sensors && this.sensors['mmwave1']) {
                    this.sensors['mmwave1'].group.position.y = value;
                    this.updateDetection();
                }
            });
        }
        
        if (sensor2YSlider) {
            sensor2YSlider.addEventListener('input', (e) => {
                const value = parseFloat(e.target.value);
                if (sensor2YValue) sensor2YValue.textContent = value.toFixed(1);
                if (this.sensors && this.sensors['mmwave2']) {
                    this.sensors['mmwave2'].group.position.y = value;
                    this.updateDetection();
                }
            });
        }
        
        // Reset and randomize buttons
        const resetButton = document.getElementById('reset-scene');
        const randomizeButton = document.getElementById('randomize-scene');
        
        if (resetButton) {
            resetButton.addEventListener('click', () => this.resetScene());
        }
        
        if (randomizeButton) {
            randomizeButton.addEventListener('click', () => this.randomizeScene());
        }
    }

    onWindowResize() {
        if (!this.camera || !this.renderer) return;
        
        const container = document.getElementById('three-container');
        if (!container) return;
        
        this.camera.aspect = container.clientWidth / container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        
        if (this.sensorRenderer) {
            const sensorContainer = document.getElementById('sensor-fov-container');
            if (sensorContainer) {
                this.sensorRenderer.setSize(sensorContainer.clientWidth, sensorContainer.clientHeight);
            }
        }
    }
    
    animate() {
        if (!this.isInitialized) return;
        
        this.animationId = requestAnimationFrame(() => this.animate());
        
        // Update controls
        if (this.controls) {
            this.controls.update();
        }
        
        // Animate sensors
        Object.values(this.sensors).forEach(sensor => {
            if (sensor.mesh) {
                sensor.mesh.rotation.y += 0.005;
            }
        });
        
        // Animate point clouds
        Object.values(this.pointClouds).forEach(pointCloudObj => {
            if (pointCloudObj.points.material) {
                pointCloudObj.points.material.opacity = 0.6 + 0.2 * Math.sin(Date.now() * 0.003);
            }
        });
        
        // Render scenes
        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
        }
        
        if (this.sensorRenderer && this.sensorScene && this.sensorCamera) {
            this.sensorRenderer.render(this.sensorScene, this.sensorCamera);
        }
    }
    
    destroy() {
        if (this.animationId) {
            cancelAnimationFrame(this.animationId);
        }
        if (this.socket) {
            this.socket.disconnect();
        }
        if (this.renderer) {
            this.renderer.dispose();
        }
        if (this.sensorRenderer) {
            this.sensorRenderer.dispose();
        }
    }
}

// Initialize dashboard when page loads
let dashboard;
document.addEventListener('DOMContentLoaded', () => {
    console.log('DOM loaded, initializing dashboard...');
    dashboard = new Olympus3DDashboard();
});

// Cleanup on page unload
window.addEventListener('beforeunload', () => {
    if (dashboard) {
        dashboard.destroy();
    }
});