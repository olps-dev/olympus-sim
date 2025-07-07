// Olympus Simulation Type Definitions

export interface Position3D {
  x: number;
  y: number;
  z: number;
}

export interface Rotation3D {
  x: number;
  y: number;
  z: number;
}

export interface NodeType {
  id: string;
  type: 'zeus' | 'mid-tier' | 'low-power';
  name: string;
  position: Position3D;
  rotation: Rotation3D;
  sensors: string[];
  status: 'online' | 'offline' | 'error';
  capabilities: NodeCapabilities;
  room: string;
}

export interface NodeCapabilities {
  processing: 'high' | 'medium' | 'low';
  storage: 'high' | 'medium' | 'low';
  networking: 'mesh' | 'wifi' | 'ethernet';
  power: 'mains' | 'battery' | 'solar';
  ai: boolean;
  gpu?: string;
  ram?: string;
}

export interface Sensor {
  id: string;
  type: SensorType;
  name: string;
  position: Position3D;
  rotation: Rotation3D;
  specifications: SensorSpecs;
  status: 'active' | 'inactive' | 'error';
  nodeId: string;
  room: string;
}

export type SensorType = 
  | 'mmwave' 
  | 'temperature' 
  | 'audio' 
  | 'camera' 
  | 'pir' 
  | 'pressure' 
  | 'light' 
  | 'humidity';

export interface SensorSpecs {
  range?: number;
  fov?: number;
  updateRate: number;
  accuracy: string;
  powerConsumption: number;
  [key: string]: any;
}

export interface Room {
  id: string;
  name: string;
  dimensions: {
    width: number;
    height: number;
    depth: number;
  };
  position: Position3D;
  nodes: string[];
  sensors: string[];
  automationRules: AutomationRule[];
}

export interface AutomationRule {
  id: string;
  name: string;
  description: string;
  trigger: AutomationTrigger;
  actions: AutomationAction[];
  conditions?: AutomationCondition[];
  enabled: boolean;
}

export interface AutomationTrigger {
  sensorId: string;
  event: 'motion' | 'presence' | 'temperature' | 'audio' | 'threshold';
  value?: any;
  operator?: '>' | '<' | '=' | '>=' | '<=';
}

export interface AutomationAction {
  type: 'light' | 'hvac' | 'security' | 'notification' | 'scene';
  target: string;
  parameters: Record<string, any>;
}

export interface AutomationCondition {
  sensorId: string;
  property: string;
  operator: '>' | '<' | '=' | '>=' | '<=';
  value: any;
}

export interface SensorData {
  sensorId: string;
  timestamp: number;
  data: Record<string, any>;
  processed: boolean;
}

export interface SystemMetrics {
  timestamp: number;
  nodes: {
    [nodeId: string]: {
      cpu: number;
      memory: number;
      network: number;
      storage: number;
      temperature: number;
    };
  };
  sensors: {
    [sensorId: string]: {
      lastUpdate: number;
      dataRate: number;
      errorRate: number;
    };
  };
  automation: {
    rulesExecuted: number;
    averageResponseTime: number;
    errors: number;
  };
}

export interface SimulationConfig {
  physics: {
    gravity: number;
    timeStep: number;
    enabled: boolean;
  };
  rendering: {
    shadows: boolean;
    reflections: boolean;
    ambientLight: number;
    directionalLight: number;
  };
  networking: {
    latency: number;
    bandwidth: number;
    packetLoss: number;
  };
  realTime: boolean;
  speedMultiplier: number;
}

export interface ScenarioData {
  id: string;
  name: string;
  description: string;
  duration: number;
  participants: string[];
  events: ScenarioEvent[];
  objectives: string[];
  success: boolean;
  metrics: Record<string, any>;
}

export interface ScenarioEvent {
  timestamp: number;
  type: 'sensor_trigger' | 'automation_action' | 'user_action' | 'system_event';
  description: string;
  data: Record<string, any>;
}

export interface OlympusState {
  nodes: NodeType[];
  sensors: Sensor[];
  rooms: Room[];
  automationRules: AutomationRule[];
  systemMetrics: SystemMetrics;
  simulationConfig: SimulationConfig;
  currentScenario?: ScenarioData;
  isSimulationRunning: boolean;
  selectedNode?: string;
  selectedSensor?: string;
  selectedRoom?: string;
}