/**
 * Olympus 3D Dashboard - Advanced IoT Simulation Visualization
 * Features: 3D scene, point cloud visualization, real-time data
 */

class Olympus3DDashboard {
    constructor() {
        // Three.js scene setup
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        
        // 3D objects
        this.room = null;
        this.sensors = {};
        this.human = null;
        this.pointClouds = {};
        this.sensorBeams = {};
        
        // Data management
        this.socket = null;
        this.latencyChart = null;
        this.sensorData = {};
        this.latencyHistory = [];
        
        // Animation
        this.animationId = null;
        
        this.init();
    }
    
    init() {
        this.setupScene();
        this.createEnvironment();
        this.setupControls();
        this.setupWebSocket();
        this.setupCharts();
        this.setupEventListeners();
        this.animate();
        
        console.log('Olympus 3D Dashboard initialized');
    }
    
    setupScene() {
        const container = document.getElementById('three-container');
        
        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0a0a0a);
        this.scene.fog = new THREE.Fog(0x0a0a0a, 10, 50);
        
        // Camera
        this.camera = new THREE.PerspectiveCamera(
            75, 
            container.clientWidth / container.clientHeight, 
            0.1, 
            1000
        );
        this.camera.position.set(8, 6, 8);
        this.camera.lookAt(0, 0, 0);
        
        // Renderer
        this.renderer = new THREE.WebGLRenderer({ 
            antialias: true,
            alpha: true
        });
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.outputEncoding = THREE.sRGBEncoding;
        
        container.appendChild(this.renderer.domElement);
        
        // Lighting
        const ambientLight = new THREE.AmbientLight(0x404040, 0.3);
        this.scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 5);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        this.scene.add(directionalLight);
        
        // Point lights for atmosphere
        const pointLight1 = new THREE.PointLight(0x00bcd4, 0.5, 10);
        pointLight1.position.set(-3, 3, -3);
        this.scene.add(pointLight1);
        
        const pointLight2 = new THREE.PointLight(0x4fc3f7, 0.5, 10);
        pointLight2.position.set(3, 3, 3);
        this.scene.add(pointLight2);
        
        // Handle window resize
        window.addEventListener('resize', () => this.onWindowResize());
    }
    
    createEnvironment() {
        // Room floor
        const floorGeometry = new THREE.PlaneGeometry(12, 12);
        const floorMaterial = new THREE.MeshLambertMaterial({
            color: 0x2a2a2a,
            transparent: true,
            opacity: 0.8
        });
        const floor = new THREE.Mesh(floorGeometry, floorMaterial);
        floor.rotation.x = -Math.PI / 2;
        floor.receiveShadow = true;
        this.scene.add(floor);
        
        // Room walls (wireframe)
        const wallMaterial = new THREE.LineBasicMaterial({ 
            color: 0x333333,
            transparent: true,
            opacity: 0.3
        });
        
        // Create room outline
        const roomGeometry = new THREE.EdgesGeometry(new THREE.BoxGeometry(12, 6, 12));
        const roomEdges = new THREE.LineSegments(roomGeometry, wallMaterial);
        roomEdges.position.y = 3;
        this.scene.add(roomEdges);
        
        // Grid helper
        const gridHelper = new THREE.GridHelper(12, 24, 0x444444, 0x222222);
        this.scene.add(gridHelper);
        
        // Create sensors
        this.createSensor('mmwave1', -4, 2.5, 0, 0x00bcd4);
        this.createSensor('mmwave2', 4, 2.5, 0, 0x4fc3f7);
        
        // Create human figure
        this.createHuman();
        
        // Create lamp
        this.createLamp();
    }
    
    createSensor(id, x, y, z, color) {
        const group = new THREE.Group();
        
        // Sensor body
        const sensorGeometry = new THREE.CylinderGeometry(0.1, 0.15, 0.3, 8);
        const sensorMaterial = new THREE.MeshPhongMaterial({ 
            color: color,
            transparent: true,
            opacity: 0.9
        });
        const sensorMesh = new THREE.Mesh(sensorGeometry, sensorMaterial);
        sensorMesh.castShadow = true;
        
        // Sensor base
        const baseGeometry = new THREE.CylinderGeometry(0.2, 0.2, 0.05, 8);
        const baseMaterial = new THREE.MeshPhongMaterial({ color: 0x333333 });
        const base = new THREE.Mesh(baseGeometry, baseMaterial);
        base.position.y = -0.175;
        
        // Detection cone (wireframe)
        const coneGeometry = new THREE.ConeGeometry(3, 6, 16, 1, true);
        const coneMaterial = new THREE.MeshBasicMaterial({
            color: color,
            wireframe: true,
            transparent: true,
            opacity: 0.1
        });
        const cone = new THREE.Mesh(coneGeometry, coneMaterial);
        cone.rotation.x = Math.PI;
        cone.position.y = -3;
        
        group.add(sensorMesh);
        group.add(base);
        group.add(cone);
        group.position.set(x, y, z);
        
        this.scene.add(group);
        this.sensors[id] = {
            group: group,
            mesh: sensorMesh,
            cone: cone,
            active: false,
            pointCloud: null
        };
        
        // Create point cloud for this sensor
        this.createPointCloud(id, color);
    }
    
    createPointCloud(sensorId, color) {
        const pointGeometry = new THREE.BufferGeometry();
        const pointMaterial = new THREE.PointsMaterial({
            color: color,
            size: 0.1,
            transparent: true,
            opacity: 0.8,
            sizeAttenuation: true
        });
        
        // Initialize with empty positions
        const positions = new Float32Array(300); // Max 100 points
        pointGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        
        const pointCloud = new THREE.Points(pointGeometry, pointMaterial);
        this.scene.add(pointCloud);
        
        this.pointClouds[sensorId] = pointCloud;
    }
    
    createHuman() {
        const group = new THREE.Group();
        
        // Human body (simple cylinder)
        const bodyGeometry = new THREE.CylinderGeometry(0.3, 0.3, 1.7, 8);
        const bodyMaterial = new THREE.MeshPhongMaterial({ 
            color: 0xff6b6b,
            transparent: true,
            opacity: 0.8
        });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        body.position.y = 0.85;
        body.castShadow = true;
        
        // Head
        const headGeometry = new THREE.SphereGeometry(0.2, 16, 16);
        const head = new THREE.Mesh(headGeometry, bodyMaterial);
        head.position.y = 1.9;
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
        
        // Lamp head
        const lampGeometry = new THREE.SphereGeometry(0.3, 16, 16);
        const lampMaterial = new THREE.MeshPhongMaterial({ 
            color: 0xffeb3b,
            transparent: true,
            opacity: 0.3
        });
        const lamp = new THREE.Mesh(lampGeometry, lampMaterial);
        lamp.position.y = 3.2;
        
        group.add(post);
        group.add(lamp);
        group.position.set(-2, 0, -2);
        
        this.scene.add(group);
        this.lamp = { group: group, lamp: lamp, isOn: false };
    }
    
    setupControls() {
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.minDistance = 3;
        this.controls.maxDistance = 20;
        this.controls.maxPolarAngle = Math.PI / 2 + 0.3;
        
        // Control sliders
        const humanXSlider = document.getElementById('human-x');
        const humanZSlider = document.getElementById('human-z');
        const sensor1YSlider = document.getElementById('sensor1-y');
        const sensor2YSlider = document.getElementById('sensor2-y');
        
        humanXSlider.addEventListener('input', (e) => {
            const value = parseFloat(e.target.value);
            this.human.position.x = value;
            document.getElementById('human-x-value').textContent = value.toFixed(1);
            this.updateDetection();
        });
        
        humanZSlider.addEventListener('input', (e) => {
            const value = parseFloat(e.target.value);
            this.human.position.z = value;
            document.getElementById('human-z-value').textContent = value.toFixed(1);
            this.updateDetection();
        });
        
        sensor1YSlider.addEventListener('input', (e) => {
            const value = parseFloat(e.target.value);
            this.sensors['mmwave1'].group.position.y = value;
            document.getElementById('sensor1-y-value').textContent = value.toFixed(1);
            this.updateDetection();
        });
        
        sensor2YSlider.addEventListener('input', (e) => {
            const value = parseFloat(e.target.value);
            this.sensors['mmwave2'].group.position.y = value;
            document.getElementById('sensor2-y-value').textContent = value.toFixed(1);
            this.updateDetection();
        });
        
        // Buttons
        document.getElementById('reset-scene').addEventListener('click', () => {
            this.resetScene();
        });
        
        document.getElementById('randomize-scene').addEventListener('click', () => {
            this.randomizeScene();
        });
    }
    
    setupWebSocket() {
        this.socket = io();
        
        this.socket.on('connect', () => {
            console.log('WebSocket connected');
            this.updateStatusIndicator('ws-status', true);
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
        const ctx = document.getElementById('latency-chart').getContext('2d');
        this.latencyChart = new Chart(ctx, {
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
                    legend: {
                        display: false
                    }
                },
                scales: {
                    x: {
                        display: false
                    },
                    y: {
                        beginAtZero: true,
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#999',
                            callback: function(value) {
                                return value + 'ms';
                            }
                        }
                    }
                },
                animation: {
                    duration: 0
                }
            }
        });
    }
    
    setupEventListeners() {
        // Additional event listeners can be added here
    }
    
    handleDashboardUpdate(data) {
        // Update MQTT status
        this.updateStatusIndicator('mqtt-status', data.mqtt_connected);
        
        // Update performance metrics
        if (data.latency_stats && data.latency_stats.count > 0) {
            document.getElementById('avg-latency').textContent = data.latency_stats.mean.toFixed(1) + ' ms';
            document.getElementById('p95-latency').textContent = data.latency_stats.p95.toFixed(1) + ' ms';
            document.getElementById('sample-count').textContent = data.latency_stats.count;
            document.getElementById('min-max-latency').textContent = 
                data.latency_stats.min.toFixed(1) + ' / ' + data.latency_stats.max.toFixed(1);
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
        
        // Update 3D visualization
        if (this.sensors[sensorId]) {
            this.updateSensorVisualization(sensorId, sensorData);
        }
        
        // Simulate point cloud based on presence
        this.updatePointCloudSimulation(sensorId, sensorData);
    }
    
    handleAutomationEvent(event) {
        console.log('Automation event:', event);
        
        // Update lamp visualization
        if (event.actuator_id === 'lamp_hall') {
            this.updateLampState(event.action === 'on');
        }
        
        // Add latency point if available
        if (event.latency_ms > 0) {
            this.addLatencyPoint(event.latency_ms);
        }
    }
    
    updateSensorDisplay(sensors) {
        const sensorList = document.getElementById('sensor-list');
        sensorList.innerHTML = '';
        
        Object.entries(sensors).forEach(([sensorId, sensor]) => {
            const sensorCard = document.createElement('div');
            sensorCard.className = `sensor-card ${sensor.human_present ? 'active' : ''}`;
            sensorCard.innerHTML = `
                <div class="sensor-header">
                    <div class="sensor-name">${sensorId}</div>
                    <div class="sensor-status ${sensor.human_present ? 'active' : ''}">
                        ${sensor.human_present ? 'ACTIVE' : 'IDLE'}
                    </div>
                </div>
                <div class="sensor-details">
                    <div>Battery: ${sensor.battery_level_mah.toFixed(0)} mAh</div>
                    <div>Events: ${sensor.event_count || 0}</div>
                </div>
            `;
            sensorList.appendChild(sensorCard);
            
            // Update 3D sensor
            if (this.sensors[sensorId]) {
                this.updateSensorVisualization(sensorId, sensor);
            }
        });
    }
    
    updateSensorVisualization(sensorId, sensorData) {
        if (!this.sensors[sensorId]) return;
        
        const sensor = this.sensors[sensorId];
        const isActive = sensorData.human_present;
        
        // Update sensor material opacity/color
        if (isActive) {
            sensor.mesh.material.opacity = 1.0;
            sensor.cone.material.opacity = 0.2;
        } else {
            sensor.mesh.material.opacity = 0.7;
            sensor.cone.material.opacity = 0.05;
        }
        
        sensor.active = isActive;
    }
    
    updatePointCloudSimulation(sensorId, sensorData) {
        if (!this.pointClouds[sensorId]) return;
        
        const pointCloud = this.pointClouds[sensorId];
        const positions = pointCloud.geometry.attributes.position.array;
        
        if (sensorData.human_present && sensorData.num_points) {
            // Generate simulated point cloud around human position
            const humanPos = this.human.position;
            const sensorPos = this.sensors[sensorId].group.position;
            const numPoints = Math.min(sensorData.num_points, 100);
            
            // Update point count display
            document.getElementById(`${sensorId.replace('mmwave', 'sensor')}-points`).textContent = numPoints;
            
            for (let i = 0; i < numPoints; i++) {
                const idx = i * 3;
                
                // Add some randomness around human position
                positions[idx] = humanPos.x + (Math.random() - 0.5) * 0.6;
                positions[idx + 1] = humanPos.y + Math.random() * 1.7 + 0.2;
                positions[idx + 2] = humanPos.z + (Math.random() - 0.5) * 0.6;
            }
            
            // Clear remaining points
            for (let i = numPoints * 3; i < positions.length; i++) {
                positions[i] = 0;
            }
            
            pointCloud.geometry.setDrawRange(0, numPoints);
        } else {
            // No detection - clear points
            pointCloud.geometry.setDrawRange(0, 0);
            document.getElementById(`${sensorId.replace('mmwave', 'sensor')}-points`).textContent = '0';
        }
        
        pointCloud.geometry.attributes.position.needsUpdate = true;
    }
    
    updateLatencyChart(latencyHistory) {
        const recent = latencyHistory.slice(-20); // Last 20 points
        const labels = recent.map((_, i) => i.toString());
        const data = recent.map(point => point.latency_ms);
        
        this.latencyChart.data.labels = labels;
        this.latencyChart.data.datasets[0].data = data;
        this.latencyChart.update('none');
    }
    
    addLatencyPoint(latencyMs) {
        this.latencyHistory.push({ latency_ms: latencyMs, timestamp: Date.now() });
        
        // Keep only recent points
        if (this.latencyHistory.length > 50) {
            this.latencyHistory = this.latencyHistory.slice(-20);
        }
        
        this.updateLatencyChart(this.latencyHistory);
    }
    
    updateAutomationEvents(events) {
        const eventList = document.getElementById('automation-events');
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
            this.lamp.lamp.material.opacity = 0.8;
            this.lamp.lamp.material.emissive.setHex(0x444400);
        } else {
            this.lamp.lamp.material.opacity = 0.3;
            this.lamp.lamp.material.emissive.setHex(0x000000);
        }
    }
    
    updateDetection() {
        // Simulate detection based on human position relative to sensors
        Object.entries(this.sensors).forEach(([sensorId, sensor]) => {
            const humanPos = this.human.position;
            const sensorPos = sensor.group.position;
            
            const distance = humanPos.distanceTo(sensorPos);
            const isInRange = distance < 6; // 6m detection range
            
            // Simple angle check (assume 60 degree FOV)
            const angle = Math.atan2(humanPos.z - sensorPos.z, humanPos.x - sensorPos.x);
            const sensorAngle = Math.atan2(0, sensorId === 'mmwave1' ? 1 : -1); // Facing towards center
            const angleDiff = Math.abs(angle - sensorAngle);
            const isInFOV = angleDiff < Math.PI / 3; // 60 degree FOV
            
            const detected = isInRange && isInFOV;
            
            // Update visualization
            if (detected) {
                sensor.mesh.material.opacity = 1.0;
                sensor.cone.material.opacity = 0.2;
            } else {
                sensor.mesh.material.opacity = 0.7;
                sensor.cone.material.opacity = 0.05;
            }
        });
    }
    
    updateStatusIndicator(id, connected) {
        const indicator = document.getElementById(id);
        if (connected) {
            indicator.classList.add('connected');
            indicator.classList.remove('disconnected');
        } else {
            indicator.classList.add('disconnected');
            indicator.classList.remove('connected');
        }
    }
    
    resetScene() {
        // Reset all positions to defaults
        this.human.position.set(0, 0, 0);
        this.sensors['mmwave1'].group.position.set(-4, 2.5, 0);
        this.sensors['mmwave2'].group.position.set(4, 2.5, 0);
        
        // Reset sliders
        document.getElementById('human-x').value = 0;
        document.getElementById('human-z').value = 0;
        document.getElementById('sensor1-y').value = 2.5;
        document.getElementById('sensor2-y').value = 2.5;
        
        // Update value displays
        document.getElementById('human-x-value').textContent = '0.0';
        document.getElementById('human-z-value').textContent = '0.0';
        document.getElementById('sensor1-y-value').textContent = '2.5';
        document.getElementById('sensor2-y-value').textContent = '2.5';
        
        this.updateDetection();
    }
    
    randomizeScene() {
        // Randomize positions
        const humanX = (Math.random() - 0.5) * 8;
        const humanZ = (Math.random() - 0.5) * 8;
        const sensor1Y = 1 + Math.random() * 3;
        const sensor2Y = 1 + Math.random() * 3;
        
        this.human.position.set(humanX, 0, humanZ);
        this.sensors['mmwave1'].group.position.y = sensor1Y;
        this.sensors['mmwave2'].group.position.y = sensor2Y;
        
        // Update sliders
        document.getElementById('human-x').value = humanX;
        document.getElementById('human-z').value = humanZ;
        document.getElementById('sensor1-y').value = sensor1Y;
        document.getElementById('sensor2-y').value = sensor2Y;
        
        // Update value displays
        document.getElementById('human-x-value').textContent = humanX.toFixed(1);
        document.getElementById('human-z-value').textContent = humanZ.toFixed(1);
        document.getElementById('sensor1-y-value').textContent = sensor1Y.toFixed(1);
        document.getElementById('sensor2-y-value').textContent = sensor2Y.toFixed(1);
        
        this.updateDetection();
    }
    
    onWindowResize() {
        const container = document.getElementById('three-container');
        this.camera.aspect = container.clientWidth / container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(container.clientWidth, container.clientHeight);
    }
    
    animate() {
        this.animationId = requestAnimationFrame(() => this.animate());
        
        this.controls.update();
        
        // Rotate sensors slightly for visual interest
        Object.values(this.sensors).forEach(sensor => {
            sensor.mesh.rotation.y += 0.005;
        });
        
        // Animate point clouds
        Object.values(this.pointClouds).forEach(pointCloud => {
            if (pointCloud.material) {
                pointCloud.material.opacity = 0.6 + 0.2 * Math.sin(Date.now() * 0.003);
            }
        });
        
        this.renderer.render(this.scene, this.camera);
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
    }
}

// Initialize dashboard when page loads
let dashboard;
document.addEventListener('DOMContentLoaded', () => {
    dashboard = new Olympus3DDashboard();
});

// Cleanup on page unload
window.addEventListener('beforeunload', () => {
    if (dashboard) {
        dashboard.destroy();
    }
});