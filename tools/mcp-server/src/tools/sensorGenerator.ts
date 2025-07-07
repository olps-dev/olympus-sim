import { CallToolResult } from '@modelcontextprotocol/sdk/types.js';

export interface SensorConfig {
  sensorType: string;
  room: string;
  position?: { x: number; y: number; z: number };
  specifications?: Record<string, any>;
}

export class SensorGenerator {
  async generate(args: any): Promise<CallToolResult> {
    const config: SensorConfig = args as SensorConfig;
    
    try {
      const sensorCode = this.generateSensorCode(config);
      const rosConfig = this.generateROS2Config(config);
      const mqttConfig = this.generateMQTTConfig(config);
      
      return {
        content: [
          {
            type: 'text',
            text: `# Generated ${config.sensorType.toUpperCase()} Sensor for ${config.room}

## Three.js Sensor Object
\`\`\`javascript
${sensorCode}
\`\`\`

## ROS2 Configuration
\`\`\`javascript
${rosConfig}
\`\`\`

## MQTT Configuration
\`\`\`javascript
${mqttConfig}
\`\`\`

## Integration Instructions
1. Add the sensor object code to your Three.js scene initialization
2. Include the ROS2 configuration in your sensor management system
3. Set up MQTT topics as specified above
4. Update sensor count and UI accordingly`
          }
        ]
      };
    } catch (error) {
      throw new Error(`Failed to generate sensor: ${error}`);
    }
  }

  private generateSensorCode(config: SensorConfig): string {
    const specs = this.getSensorSpecifications(config.sensorType, config.specifications);
    const position = config.position || { x: 0, y: 1, z: 0 };
    
    switch (config.sensorType) {
      case 'mmwave':
        return this.generateMMWaveSensor(config, specs, position);
      case 'temperature':
        return this.generateTemperatureSensor(config, specs, position);
      case 'audio':
        return this.generateAudioSensor(config, specs, position);
      case 'camera':
        return this.generateCameraSensor(config, specs, position);
      case 'pir':
        return this.generatePIRSensor(config, specs, position);
      case 'pressure':
        return this.generatePressureSensor(config, specs, position);
      case 'light':
        return this.generateLightSensor(config, specs, position);
      case 'humidity':
        return this.generateHumiditySensor(config, specs, position);
      default:
        throw new Error(`Unknown sensor type: ${config.sensorType}`);
    }
  }

  private generateMMWaveSensor(config: SensorConfig, specs: any, position: any): string {
    return `// mmWave Radar Sensor for ${config.room}
function create${config.sensorType}Sensor_${config.room}() {
    const sensorGroup = new THREE.Group();
    
    // Sensor housing
    const housingGeometry = new THREE.BoxGeometry(0.3, 0.2, 0.1);
    const housingMaterial = new THREE.MeshLambertMaterial({ color: 0x333333 });
    const housing = new THREE.Mesh(housingGeometry, housingMaterial);
    sensorGroup.add(housing);
    
    // FOV visualization
    const fovAngle = ${specs.fov * 180 / Math.PI} * Math.PI / 180; // ${specs.fov}° converted to radians
    const range = ${specs.range};
    const fovGeometry = new THREE.ConeGeometry(
        range * Math.tan(fovAngle / 2), 
        range, 
        16, 1, true
    );
    const fovMaterial = new THREE.MeshBasicMaterial({
        color: 0x00ff44,
        transparent: true,
        opacity: 0.1,
        wireframe: true
    });
    const fovCone = new THREE.Mesh(fovGeometry, fovMaterial);
    fovCone.position.z = range / 2;
    fovCone.rotation.x = -Math.PI / 2;
    sensorGroup.add(fovCone);
    
    // Sensor label
    const label = createLabel('${config.sensorType.toUpperCase()} (${config.room})');
    label.position.y = 0.5;
    sensorGroup.add(label);
    
    // Position and configure
    sensorGroup.position.set(${position.x}, ${position.y}, ${position.z});
    sensorGroup.userData = {
        type: 'sensor',
        sensorType: '${config.sensorType}',
        room: '${config.room}',
        moveable: true,
        name: '${config.sensorType}_${config.room}',
        specs: {
            range: ${specs.range},
            fov: ${specs.fov},
            updateRate: ${specs.updateRate},
            accuracy: ${specs.accuracy}
        }
    };
    
    return sensorGroup;
}`;
  }

  private generateTemperatureSensor(config: SensorConfig, specs: any, position: any): string {
    return `// Temperature Sensor for ${config.room}
function create${config.sensorType}Sensor_${config.room}() {
    const sensorGroup = new THREE.Group();
    
    // Sensor housing - small cylindrical sensor
    const housingGeometry = new THREE.CylinderGeometry(0.05, 0.05, 0.1);
    const housingMaterial = new THREE.MeshLambertMaterial({ color: 0x0066cc });
    const housing = new THREE.Mesh(housingGeometry, housingMaterial);
    sensorGroup.add(housing);
    
    // Coverage area visualization (thermal diffusion sphere)
    const coverageGeometry = new THREE.SphereGeometry(${specs.range});
    const coverageMaterial = new THREE.MeshBasicMaterial({
        color: 0xff6600,
        transparent: true,
        opacity: 0.05,
        wireframe: true
    });
    const coverage = new THREE.Mesh(coverageGeometry, coverageMaterial);
    sensorGroup.add(coverage);
    
    // Sensor label
    const label = createLabel('TEMP (${config.room})');
    label.position.y = 0.3;
    sensorGroup.add(label);
    
    // Position and configure
    sensorGroup.position.set(${position.x}, ${position.y}, ${position.z});
    sensorGroup.userData = {
        type: 'sensor',
        sensorType: '${config.sensorType}',
        room: '${config.room}',
        moveable: true,
        name: '${config.sensorType}_${config.room}',
        specs: {
            range: ${specs.range},
            accuracy: ${specs.accuracy},
            updateRate: ${specs.updateRate},
            minTemp: ${specs.minTemp},
            maxTemp: ${specs.maxTemp}
        }
    };
    
    return sensorGroup;
}`;
  }

  private generateAudioSensor(config: SensorConfig, specs: any, position: any): string {
    return `// Audio/Microphone Sensor for ${config.room}
function create${config.sensorType}Sensor_${config.room}() {
    const sensorGroup = new THREE.Group();
    
    // Microphone housing
    const housingGeometry = new THREE.CylinderGeometry(0.03, 0.04, 0.08);
    const housingMaterial = new THREE.MeshLambertMaterial({ color: 0x222222 });
    const housing = new THREE.Mesh(housingGeometry, housingMaterial);
    sensorGroup.add(housing);
    
    // Audio pickup pattern (cardioid)
    const patternGeometry = new THREE.SphereGeometry(${specs.range}, 16, 8, 0, Math.PI * 2, 0, Math.PI * 0.75);
    const patternMaterial = new THREE.MeshBasicMaterial({
        color: 0x9900ff,
        transparent: true,
        opacity: 0.08,
        wireframe: true
    });
    const pattern = new THREE.Mesh(patternGeometry, patternMaterial);
    sensorGroup.add(pattern);
    
    // Sensor label
    const label = createLabel('AUDIO (${config.room})');
    label.position.y = 0.4;
    sensorGroup.add(label);
    
    // Position and configure
    sensorGroup.position.set(${position.x}, ${position.y}, ${position.z});
    sensorGroup.userData = {
        type: 'sensor',
        sensorType: '${config.sensorType}',
        room: '${config.room}',
        moveable: true,
        name: '${config.sensorType}_${config.room}',
        specs: {
            range: ${specs.range},
            sensitivity: ${specs.sensitivity},
            updateRate: ${specs.updateRate},
            frequencyRange: [${specs.minFreq}, ${specs.maxFreq}]
        }
    };
    
    return sensorGroup;
}`;
  }

  private generateCameraSensor(config: SensorConfig, specs: any, position: any): string {
    return `// Camera Sensor for ${config.room}
function create${config.sensorType}Sensor_${config.room}() {
    const sensorGroup = new THREE.Group();
    
    // Camera housing
    const housingGeometry = new THREE.BoxGeometry(0.08, 0.06, 0.12);
    const housingMaterial = new THREE.MeshLambertMaterial({ color: 0x1a1a1a });
    const housing = new THREE.Mesh(housingGeometry, housingMaterial);
    sensorGroup.add(housing);
    
    // Lens
    const lensGeometry = new THREE.CylinderGeometry(0.02, 0.02, 0.02);
    const lensMaterial = new THREE.MeshLambertMaterial({ color: 0x000000 });
    const lens = new THREE.Mesh(lensGeometry, lensMaterial);
    lens.position.z = 0.07;
    lens.rotation.x = Math.PI / 2;
    sensorGroup.add(lens);
    
    // Field of view frustum
    const fovAngle = ${specs.fov} * Math.PI / 180;
    const range = ${specs.range};
    const fovGeometry = new THREE.ConeGeometry(
        range * Math.tan(fovAngle / 2), 
        range, 
        8, 1, true
    );
    const fovMaterial = new THREE.MeshBasicMaterial({
        color: 0x00ffff,
        transparent: true,
        opacity: 0.1,
        wireframe: true
    });
    const fovCone = new THREE.Mesh(fovGeometry, fovMaterial);
    fovCone.position.z = range / 2;
    fovCone.rotation.x = -Math.PI / 2;
    sensorGroup.add(fovCone);
    
    // Sensor label
    const label = createLabel('CAMERA (${config.room})');
    label.position.y = 0.4;
    sensorGroup.add(label);
    
    // Position and configure
    sensorGroup.position.set(${position.x}, ${position.y}, ${position.z});
    sensorGroup.userData = {
        type: 'sensor',
        sensorType: '${config.sensorType}',
        room: '${config.room}',
        moveable: true,
        name: '${config.sensorType}_${config.room}',
        specs: {
            range: ${specs.range},
            fov: ${specs.fov},
            resolution: '${specs.resolution}',
            updateRate: ${specs.updateRate}
        }
    };
    
    return sensorGroup;
}`;
  }

  private generatePIRSensor(config: SensorConfig, specs: any, position: any): string {
    return `// PIR Motion Sensor for ${config.room}
function create${config.sensorType}Sensor_${config.room}() {
    const sensorGroup = new THREE.Group();
    
    // PIR sensor housing (dome shape)
    const housingGeometry = new THREE.SphereGeometry(0.06, 8, 4, 0, Math.PI * 2, 0, Math.PI / 2);
    const housingMaterial = new THREE.MeshLambertMaterial({ color: 0xffffff });
    const housing = new THREE.Mesh(housingGeometry, housingMaterial);
    sensorGroup.add(housing);
    
    // Detection cone
    const detectionAngle = ${specs.fov} * Math.PI / 180;
    const range = ${specs.range};
    const detectionGeometry = new THREE.ConeGeometry(
        range * Math.tan(detectionAngle / 2), 
        range, 
        8, 1, true
    );
    const detectionMaterial = new THREE.MeshBasicMaterial({
        color: 0xff0000,
        transparent: true,
        opacity: 0.08,
        wireframe: true
    });
    const detectionCone = new THREE.Mesh(detectionGeometry, detectionMaterial);
    detectionCone.position.z = range / 2;
    detectionCone.rotation.x = -Math.PI / 2;
    sensorGroup.add(detectionCone);
    
    // Sensor label
    const label = createLabel('PIR (${config.room})');
    label.position.y = 0.3;
    sensorGroup.add(label);
    
    // Position and configure
    sensorGroup.position.set(${position.x}, ${position.y}, ${position.z});
    sensorGroup.userData = {
        type: 'sensor',
        sensorType: '${config.sensorType}',
        room: '${config.room}',
        moveable: true,
        name: '${config.sensorType}_${config.room}',
        specs: {
            range: ${specs.range},
            fov: ${specs.fov},
            sensitivity: ${specs.sensitivity},
            updateRate: ${specs.updateRate}
        }
    };
    
    return sensorGroup;
}`;
  }

  private generatePressureSensor(config: SensorConfig, specs: any, position: any): string {
    return `// Pressure Sensor for ${config.room}
function create${config.sensorType}Sensor_${config.room}() {
    const sensorGroup = new THREE.Group();
    
    // Pressure sensor (flat circular)
    const housingGeometry = new THREE.CylinderGeometry(0.04, 0.04, 0.01);
    const housingMaterial = new THREE.MeshLambertMaterial({ color: 0x666666 });
    const housing = new THREE.Mesh(housingGeometry, housingMaterial);
    sensorGroup.add(housing);
    
    // Sensor label
    const label = createLabel('PRESSURE (${config.room})');
    label.position.y = 0.2;
    sensorGroup.add(label);
    
    // Position and configure
    sensorGroup.position.set(${position.x}, ${position.y}, ${position.z});
    sensorGroup.userData = {
        type: 'sensor',
        sensorType: '${config.sensorType}',
        room: '${config.room}',
        moveable: true,
        name: '${config.sensorType}_${config.room}',
        specs: {
            accuracy: ${specs.accuracy},
            updateRate: ${specs.updateRate},
            range: [${specs.minPressure}, ${specs.maxPressure}]
        }
    };
    
    return sensorGroup;
}`;
  }

  private generateLightSensor(config: SensorConfig, specs: any, position: any): string {
    return `// Light/Luminosity Sensor for ${config.room}
function create${config.sensorType}Sensor_${config.room}() {
    const sensorGroup = new THREE.Group();
    
    // Light sensor housing
    const housingGeometry = new THREE.BoxGeometry(0.06, 0.06, 0.03);
    const housingMaterial = new THREE.MeshLambertMaterial({ color: 0xffff00 });
    const housing = new THREE.Mesh(housingGeometry, housingMaterial);
    sensorGroup.add(housing);
    
    // Photodiode window
    const windowGeometry = new THREE.CircleGeometry(0.02);
    const windowMaterial = new THREE.MeshBasicMaterial({ color: 0x000033 });
    const window = new THREE.Mesh(windowGeometry, windowMaterial);
    window.position.z = 0.016;
    sensorGroup.add(window);
    
    // Sensor label
    const label = createLabel('LIGHT (${config.room})');
    label.position.y = 0.2;
    sensorGroup.add(label);
    
    // Position and configure
    sensorGroup.position.set(${position.x}, ${position.y}, ${position.z});
    sensorGroup.userData = {
        type: 'sensor',
        sensorType: '${config.sensorType}',
        room: '${config.room}',
        moveable: true,
        name: '${config.sensorType}_${config.room}',
        specs: {
            accuracy: ${specs.accuracy},
            updateRate: ${specs.updateRate},
            range: [${specs.minLux}, ${specs.maxLux}],
            spectralRange: '${specs.spectralRange}'
        }
    };
    
    return sensorGroup;
}`;
  }

  private generateHumiditySensor(config: SensorConfig, specs: any, position: any): string {
    return `// Humidity Sensor for ${config.room}
function create${config.sensorType}Sensor_${config.room}() {
    const sensorGroup = new THREE.Group();
    
    // Humidity sensor housing
    const housingGeometry = new THREE.BoxGeometry(0.08, 0.05, 0.03);
    const housingMaterial = new THREE.MeshLambertMaterial({ color: 0x00ccff });
    const housing = new THREE.Mesh(housingGeometry, housingMaterial);
    sensorGroup.add(housing);
    
    // Sensor grille
    const grilleGeometry = new THREE.PlaneGeometry(0.06, 0.03);
    const grilleMaterial = new THREE.MeshBasicMaterial({ 
        color: 0x333333,
        wireframe: true 
    });
    const grille = new THREE.Mesh(grilleGeometry, grilleMaterial);
    grille.position.z = 0.016;
    sensorGroup.add(grille);
    
    // Sensor label
    const label = createLabel('HUMIDITY (${config.room})');
    label.position.y = 0.2;
    sensorGroup.add(label);
    
    // Position and configure
    sensorGroup.position.set(${position.x}, ${position.y}, ${position.z});
    sensorGroup.userData = {
        type: 'sensor',
        sensorType: '${config.sensorType}',
        room: '${config.room}',
        moveable: true,
        name: '${config.sensorType}_${config.room}',
        specs: {
            accuracy: ${specs.accuracy},
            updateRate: ${specs.updateRate},
            range: [${specs.minHumidity}, ${specs.maxHumidity}]
        }
    };
    
    return sensorGroup;
}`;
  }

  private getSensorSpecifications(sensorType: string, userSpecs?: Record<string, any>): any {
    const defaults: Record<string, any> = {
      mmwave: {
        range: 10,
        fov: 45,
        updateRate: 10,
        accuracy: 0.1
      },
      temperature: {
        range: 5,
        accuracy: 0.1,
        updateRate: 1,
        minTemp: -40,
        maxTemp: 85
      },
      audio: {
        range: 8,
        sensitivity: -40,
        updateRate: 100,
        minFreq: 20,
        maxFreq: 20000
      },
      camera: {
        range: 15,
        fov: 90,
        resolution: '1920x1080',
        updateRate: 30
      },
      pir: {
        range: 8,
        fov: 110,
        sensitivity: 0.5,
        updateRate: 1
      },
      pressure: {
        accuracy: 0.1,
        updateRate: 1,
        minPressure: 300,
        maxPressure: 1100
      },
      light: {
        accuracy: 1,
        updateRate: 1,
        minLux: 0,
        maxLux: 100000,
        spectralRange: '380-750nm'
      },
      humidity: {
        accuracy: 1,
        updateRate: 1,
        minHumidity: 0,
        maxHumidity: 100
      }
    };

    return { ...defaults[sensorType], ...userSpecs };
  }

  private generateROS2Config(config: SensorConfig): string {
    return `// ROS2 Configuration for ${config.sensorType} sensor
const ${config.sensorType}Config = {
    topic: '/olympus/${config.room}/${config.sensorType}',
    messageType: '${this.getROS2MessageType(config.sensorType)}',
    qos: {
        reliability: 'reliable',
        durability: 'volatile',
        depth: 10
    },
    publisher: {
        nodeId: '${config.sensorType}_${config.room}_publisher',
        updateRate: ${this.getSensorSpecifications(config.sensorType, config.specifications).updateRate}
    }
};`;
  }

  private generateMQTTConfig(config: SensorConfig): string {
    return `// MQTT Configuration for ${config.sensorType} sensor
const mqttConfig = {
    topics: {
        data: 'olympus/${config.room}/${config.sensorType}/data',
        status: 'olympus/${config.room}/${config.sensorType}/status',
        command: 'olympus/${config.room}/${config.sensorType}/command'
    },
    qos: 1,
    retain: false,
    messageFormat: {
        timestamp: 'ISO8601',
        sensorId: '${config.sensorType}_${config.room}',
        room: '${config.room}',
        type: '${config.sensorType}',
        data: '${this.getMQTTDataFormat(config.sensorType)}',
        metadata: {
            specs: 'sensor_specifications',
            position: { x: 0, y: 0, z: 0 },
            health: 'operational'
        }
    }
};`;
  }

  private getROS2MessageType(sensorType: string): string {
    const messageTypes: Record<string, string> = {
      mmwave: 'sensor_msgs/PointCloud2',
      temperature: 'sensor_msgs/Temperature',
      audio: 'audio_msgs/AudioData',
      camera: 'sensor_msgs/Image',
      pir: 'std_msgs/Bool',
      pressure: 'sensor_msgs/FluidPressure',
      light: 'sensor_msgs/Illuminance',
      humidity: 'sensor_msgs/RelativeHumidity'
    };
    return messageTypes[sensorType] || 'std_msgs/String';
  }

  private getMQTTDataFormat(sensorType: string): string {
    const dataFormats: Record<string, string> = {
      mmwave: '{ points: [{ x, y, z, intensity }], human_present: boolean }',
      temperature: '{ celsius: number, fahrenheit: number }',
      audio: '{ db_level: number, frequency_analysis: object }',
      camera: '{ image_data: base64, motion_detected: boolean }',
      pir: '{ motion_detected: boolean, last_trigger: timestamp }',
      pressure: '{ hpa: number, altitude: number }',
      light: '{ lux: number, spectrum: object }',
      humidity: '{ percent: number, dew_point: number }'
    };
    return dataFormats[sensorType] || '{ value: any }';
  }
}