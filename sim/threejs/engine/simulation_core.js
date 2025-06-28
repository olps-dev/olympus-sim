/**
 * Olympus Three.js Simulation Core
 * Main simulation engine providing mmWave sensor simulation
 * without Gazebo dependencies
 */

import * as THREE from 'https://unpkg.com/three@0.156.1/build/three.module.js';
import { OrbitControls } from 'https://unpkg.com/three@0.156.1/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'https://unpkg.com/three@0.156.1/examples/jsm/loaders/GLTFLoader.js';

export class OlympusSimulation {
    constructor(container, options = {}) {
        this.container = container;
        this.options = {
            enablePhysics: options.enablePhysics || false,
            bridgeUrl: options.bridgeUrl || options.mqttBridgeUrl || 'http://localhost:5555',
            ros2Mode: options.ros2Mode !== false, // Default to ROS2 mode
            sensorUpdateRate: options.sensorUpdateRate || 10, // Hz
            debug: options.debug || false,
            ...options
        };
        
        // Core Three.js components
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.controls = null;
        this.clock = new THREE.Clock();
        
        // Simulation components
        this.sensors = new Map();
        this.sceneObjects = [];
        this.animationId = null;
        this.lastSensorUpdate = 0;
        
        // MQTT connection for sensor data
        this.mqttClient = null;
        
        // Raycaster for mmWave simulation
        this.raycaster = new THREE.Raycaster();
        
        this.init();
    }
    
    init() {
        this.setupRenderer();
        this.setupCamera();
        this.setupControls();
        this.setupLighting();
        this.setupScene();
        this.setupMqttConnection();
        this.startSimulation();
        
        // Handle window resize
        window.addEventListener('resize', () => this.onWindowResize());
    }
    
    setupRenderer() {
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.setClearColor(0x222222);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.container.appendChild(this.renderer.domElement);
    }
    
    setupCamera() {
        this.camera.position.set(10, 8, 10);
        this.camera.lookAt(0, 0, 0);
    }
    
    setupControls() {
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.maxPolarAngle = Math.PI / 2;
    }
    
    setupLighting() {
        // Ambient light
        const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
        this.scene.add(ambientLight);
        
        // Directional light (sun)
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 5);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        this.scene.add(directionalLight);
    }
    
    setupScene() {
        // Ground plane
        const groundGeometry = new THREE.PlaneGeometry(20, 20);
        const groundMaterial = new THREE.MeshLambertMaterial({ 
            color: 0x666666,
            transparent: true,
            opacity: 0.8
        });
        const ground = new THREE.Mesh(groundGeometry, groundMaterial);
        ground.rotation.x = -Math.PI / 2;
        ground.receiveShadow = true;
        ground.userData = { 
            type: 'ground',
            material: 'concrete',
            rcs: 0.1 
        };
        this.scene.add(ground);
        this.sceneObjects.push(ground);
        
        // Room walls
        this.createRoom();
        
        // Sample objects
        this.createSampleObjects();
        
        // Add coordinate axes for debugging
        if (this.options.debug) {
            const axesHelper = new THREE.AxesHelper(5);
            this.scene.add(axesHelper);
        }
    }
    
    createRoom() {
        // Wall material
        const wallMaterial = new THREE.MeshLambertMaterial({ 
            color: 0xcccccc,
            transparent: true,
            opacity: 0.9
        });
        
        // Wall dimensions
        const wallHeight = 3;
        const roomWidth = 10;
        const roomDepth = 10;
        const wallThickness = 0.2;
        
        // Create walls
        const walls = [
            // Front wall
            { 
                geometry: new THREE.BoxGeometry(roomWidth, wallHeight, wallThickness),
                position: [0, wallHeight/2, roomDepth/2],
                name: 'wall_front'
            },
            // Back wall
            { 
                geometry: new THREE.BoxGeometry(roomWidth, wallHeight, wallThickness),
                position: [0, wallHeight/2, -roomDepth/2],
                name: 'wall_back'
            },
            // Left wall
            { 
                geometry: new THREE.BoxGeometry(wallThickness, wallHeight, roomDepth),
                position: [-roomWidth/2, wallHeight/2, 0],
                name: 'wall_left'
            },
            // Right wall
            { 
                geometry: new THREE.BoxGeometry(wallThickness, wallHeight, roomDepth),
                position: [roomWidth/2, wallHeight/2, 0],
                name: 'wall_right'
            }
        ];
        
        walls.forEach(wallData => {
            const wall = new THREE.Mesh(wallData.geometry, wallMaterial);
            wall.position.set(...wallData.position);
            wall.castShadow = true;
            wall.receiveShadow = true;
            wall.userData = {
                type: 'wall',
                material: 'drywall',
                rcs: 0.3,
                name: wallData.name
            };
            this.scene.add(wall);
            this.sceneObjects.push(wall);
        });
    }
    
    createSampleObjects() {
        // Human figure (cylinder)
        const humanGeometry = new THREE.CylinderGeometry(0.3, 0.3, 1.8, 8);
        const humanMaterial = new THREE.MeshLambertMaterial({ color: 0x4444ff });
        const human = new THREE.Mesh(humanGeometry, humanMaterial);
        human.position.set(2, 0.9, 0);
        human.castShadow = true;
        human.userData = {
            type: 'human',
            material: 'organic',
            rcs: 1.0,
            moveable: true
        };
        this.scene.add(human);
        this.sceneObjects.push(human);
        
        // Table (box)
        const tableGeometry = new THREE.BoxGeometry(1.5, 0.1, 0.8);
        const tableMaterial = new THREE.MeshLambertMaterial({ color: 0x8B4513 });
        const table = new THREE.Mesh(tableGeometry, tableMaterial);
        table.position.set(-2, 0.8, 2);
        table.castShadow = true;
        table.receiveShadow = true;
        table.userData = {
            type: 'furniture',
            material: 'wood',
            rcs: 0.5,
            moveable: true
        };
        this.scene.add(table);
        this.sceneObjects.push(table);
        
        // Metal object (sphere)
        const metalGeometry = new THREE.SphereGeometry(0.3, 16, 16);
        const metalMaterial = new THREE.MeshLambertMaterial({ color: 0x777777 });
        const metalObject = new THREE.Mesh(metalGeometry, metalMaterial);
        metalObject.position.set(1, 0.3, -2);
        metalObject.castShadow = true;
        metalObject.userData = {
            type: 'object',
            material: 'metal',
            rcs: 2.0,
            moveable: true
        };
        this.scene.add(metalObject);
        this.sceneObjects.push(metalObject);
    }
    
    addSensor(id, position, rotation, config = {}) {
        const sensorConfig = {
            horizontalFov: config.horizontalFov || Math.PI / 2, // 90 degrees
            verticalFov: config.verticalFov || Math.PI / 6,     // 30 degrees
            minRange: config.minRange || 0.1,
            maxRange: config.maxRange || 10.0,
            horizontalResolution: config.horizontalResolution || 32,
            verticalResolution: config.verticalResolution || 16,
            updateRate: config.updateRate || 10,
            ...config
        };
        
        // Create sensor visualization
        const sensorGroup = new THREE.Group();
        
        // Sensor body
        const sensorGeometry = new THREE.BoxGeometry(0.1, 0.1, 0.05);
        const sensorMaterial = new THREE.MeshLambertMaterial({ color: 0xff0000 });
        const sensorMesh = new THREE.Mesh(sensorGeometry, sensorMaterial);
        sensorGroup.add(sensorMesh);
        
        // FOV visualization cone
        const coneGeometry = new THREE.ConeGeometry(
            Math.tan(sensorConfig.horizontalFov / 2) * sensorConfig.maxRange,
            sensorConfig.maxRange,
            8,
            1,
            true
        );
        const coneMaterial = new THREE.MeshBasicMaterial({
            color: 0xff0000,
            transparent: true,
            opacity: 0.1,
            side: THREE.DoubleSide
        });
        const cone = new THREE.Mesh(coneGeometry, coneMaterial);
        cone.position.z = sensorConfig.maxRange / 2;
        cone.rotation.x = Math.PI / 2;
        sensorGroup.add(cone);
        
        // Position and orient sensor
        sensorGroup.position.copy(position);
        sensorGroup.rotation.copy(rotation);
        
        this.scene.add(sensorGroup);
        
        // Store sensor data
        this.sensors.set(id, {
            id,
            group: sensorGroup,
            config: sensorConfig,
            lastUpdate: 0,
            pointCloud: []
        });
        
        if (this.options.debug) {
            console.log(`Added sensor ${id} at position`, position);
        }
    }
    
    setupMqttConnection() {
        // Note: This connects to either ROS2 or MQTT bridge via HTTP
        if (this.options.debug) {
            const mode = this.options.ros2Mode ? 'ROS2' : 'MQTT';
            console.log(`${mode} bridge connecting to:`, this.options.bridgeUrl);
        }
    }
    
    publishSensorData(sensorId, pointCloud, metadata) {
        // Format data for bridge (ROS2 or MQTT)
        const sensorData = {
            timestamp: Date.now() / 1000,
            sensor_id: sensorId,
            human_present: pointCloud.length > 5,
            num_points: pointCloud.length,
            analysis: `${pointCloud.length} points detected`,
            event_id: `${sensorId}_${Date.now()}`,
            battery_level_mah: 3000 - (Date.now() % 100000) / 100,
            detection_range: {
                min: this.sensors.get(sensorId).config.minRange,
                max: this.sensors.get(sensorId).config.maxRange
            },
            point_cloud_sample: pointCloud.slice(0, 100),
            centroid_position: metadata.centroid
        };

        // Determine endpoint based on mode
        const isROS2Mode = this.options.ros2Mode !== false;
        const endpoint = isROS2Mode ? 'pointcloud' : 'presence';
        
        // Prepare payload based on mode
        const payload = isROS2Mode ? {
            sensor_id: sensorId,
            timestamp: sensorData.timestamp,
            point_cloud: pointCloud,
            metadata: {
                human_present: sensorData.human_present,
                num_points: sensorData.num_points,
                centroid: metadata.centroid,
                range_stats: metadata.range,
                detection_range: sensorData.detection_range
            }
        } : sensorData;
        
        // Send to bridge via HTTP POST
        if (this.options.bridgeUrl && this.options.bridgeUrl.startsWith('http')) {
            fetch(`${this.options.bridgeUrl}/sensor/${sensorId}/${endpoint}`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(payload)
            }).then(response => {
                if (this.options.debug && response.ok) {
                    const mode = isROS2Mode ? 'ROS2' : 'MQTT';
                    console.log(`✅ Published ${pointCloud.length} points to ${mode} via ${endpoint}`);
                }
            }).catch(err => {
                if (this.options.debug) {
                    console.warn(`Bridge not available (${endpoint}):`, err.message);
                }
            });
        }
        
        // Also emit custom event for local dashboard integration
        const event = new CustomEvent('olympus-sensor-data', {
            detail: { sensorId, data: sensorData }
        });
        window.dispatchEvent(event);
    }
    
    startSimulation() {
        this.animate();
    }
    
    animate() {
        this.animationId = requestAnimationFrame(() => this.animate());
        
        const deltaTime = this.clock.getDelta();
        const currentTime = this.clock.getElapsedTime();
        
        // Update controls
        this.controls.update();
        
        // Update sensors at specified rate
        if (currentTime - this.lastSensorUpdate >= 1.0 / this.options.sensorUpdateRate) {
            this.updateSensors();
            this.lastSensorUpdate = currentTime;
        }
        
        // Render the scene
        this.renderer.render(this.scene, this.camera);
    }
    
    updateSensors() {
        this.sensors.forEach((sensor, sensorId) => {
            const pointCloud = this.simulateMmWave(sensor);
            const metadata = this.calculateMetadata(pointCloud);
            
            // Store point cloud for visualization
            sensor.pointCloud = pointCloud;
            
            // Publish sensor data
            this.publishSensorData(sensorId, pointCloud, metadata);
        });
    }
    
    simulateMmWave(sensor) {
        const pointCloud = [];
        const sensorPos = new THREE.Vector3();
        sensor.group.getWorldPosition(sensorPos);
        
        const sensorDir = new THREE.Vector3(0, 0, 1);
        sensor.group.getWorldDirection(sensorDir);
        
        // Generate rays within FOV
        const horizontalSteps = sensor.config.horizontalResolution;
        const verticalSteps = sensor.config.verticalResolution;
        
        for (let h = 0; h < horizontalSteps; h++) {
            for (let v = 0; v < verticalSteps; v++) {
                const horizontalAngle = (h / (horizontalSteps - 1) - 0.5) * sensor.config.horizontalFov;
                const verticalAngle = (v / (verticalSteps - 1) - 0.5) * sensor.config.verticalFov;
                
                // Calculate ray direction
                const rayDir = new THREE.Vector3(
                    Math.sin(horizontalAngle),
                    Math.sin(verticalAngle),
                    Math.cos(horizontalAngle) * Math.cos(verticalAngle)
                );
                
                // Transform ray direction to world space
                rayDir.applyQuaternion(sensor.group.quaternion);
                
                // Cast ray
                this.raycaster.set(sensorPos, rayDir);
                const intersections = this.raycaster.intersectObjects(this.sceneObjects);
                
                if (intersections.length > 0) {
                    const intersection = intersections[0];
                    const distance = intersection.distance;
                    
                    // Check if within sensor range
                    if (distance >= sensor.config.minRange && distance <= sensor.config.maxRange) {
                        // Convert to sensor coordinate system
                        const localPoint = intersection.point.clone().sub(sensorPos);
                        const sensorInverse = sensor.group.quaternion.clone().invert();
                        localPoint.applyQuaternion(sensorInverse);
                        
                        // Add noise
                        const noise = 0.01; // 1cm noise
                        localPoint.x += (Math.random() - 0.5) * noise;
                        localPoint.y += (Math.random() - 0.5) * noise;
                        localPoint.z += (Math.random() - 0.5) * noise;
                        
                        // Calculate RCS and intensity
                        const objectData = intersection.object.userData;
                        const rcs = objectData.rcs || 1.0;
                        const intensity = rcs / (distance * distance);
                        
                        pointCloud.push({
                            x: localPoint.x,
                            y: localPoint.y,
                            z: localPoint.z,
                            intensity: Math.min(1.0, intensity * 100),
                            range: distance,
                            velocity: 0, // TODO: Add Doppler simulation
                            rcs: rcs
                        });
                    }
                }
            }
        }
        
        return pointCloud;
    }
    
    calculateMetadata(pointCloud) {
        if (pointCloud.length === 0) {
            return { centroid: null, range: { min: 0, max: 0 } };
        }
        
        // Calculate centroid
        const centroid = pointCloud.reduce(
            (sum, point) => ({
                x: sum.x + point.x,
                y: sum.y + point.y,
                z: sum.z + point.z
            }),
            { x: 0, y: 0, z: 0 }
        );
        
        centroid.x /= pointCloud.length;
        centroid.y /= pointCloud.length;
        centroid.z /= pointCloud.length;
        centroid.distance = Math.sqrt(centroid.x * centroid.x + centroid.y * centroid.y + centroid.z * centroid.z);
        
        // Calculate range statistics
        const ranges = pointCloud.map(p => p.range);
        const rangeStats = {
            min: Math.min(...ranges),
            max: Math.max(...ranges),
            mean: ranges.reduce((sum, r) => sum + r, 0) / ranges.length
        };
        
        return { centroid, range: rangeStats };
    }
    
    onWindowResize() {
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    }
    
    destroy() {
        if (this.animationId) {
            cancelAnimationFrame(this.animationId);
        }
        
        if (this.mqttClient) {
            this.mqttClient.disconnect();
        }
        
        this.renderer.dispose();
        window.removeEventListener('resize', this.onWindowResize);
    }
}