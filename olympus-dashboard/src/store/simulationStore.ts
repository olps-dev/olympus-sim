import { create } from 'zustand';
import { immer } from 'zustand/middleware/immer';

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface SensorData {
  id: string;
  type: 'mmwave' | 'camera' | 'temperature' | 'audio' | 'pir' | 'pressure' | 'light' | 'humidity';
  position: Vector3;
  rotation?: Vector3;
  range?: number;
  fov?: number;
  status: 'active' | 'inactive' | 'error';
  lastReading?: any;
  nodeId?: string;
}

export interface NodeData {
  id: string;
  type: 'zeus' | 'mid_tier' | 'low_power';
  position: Vector3;
  rotation?: Vector3;
  status: 'online' | 'offline' | 'error';
  sensors: string[]; // sensor IDs
  capabilities: {
    processing_power: number;
    memory: number;
    network_bandwidth: number;
  };
  script?: string;
  scriptStatus?: 'stopped' | 'running' | 'error';
  lastModified?: string;
  openai_api_key?: string; // OpenAI API key for LLM features
  voice_enabled?: boolean; // Voice activation support
}

export interface RoomData {
  id: string;
  name: string;
  position: Vector3;
  rotation?: Vector3;
  dimensions: Vector3;
  color: string;
}

export interface AutomationRule {
  id: string;
  name: string;
  trigger: {
    sensorId: string;
    condition: string;
    value: any;
  };
  action: {
    type: string;
    target: string;
    params: any;
  };
  enabled: boolean;
}

interface SimulationState {
  // Entities
  sensors: Record<string, SensorData>;
  nodes: Record<string, NodeData>;
  rooms: Record<string, RoomData>;
  automationRules: Record<string, AutomationRule>;
  
  // UI State
  selectedEntityId: string | null;
  selectedEntityType: 'sensor' | 'node' | 'room' | null;
  viewMode: '3d' | 'heatmap' | 'network';
  isPaused: boolean;
  currentTime: number;
  toolMode: 'select' | 'move' | 'rotate' | 'scale';
  
  // Actions
  addSensor: (sensor: SensorData) => void;
  updateSensor: (id: string, updates: Partial<SensorData>) => void;
  removeSensor: (id: string) => void;
  
  addNode: (node: NodeData) => void;
  updateNode: (id: string, updates: Partial<NodeData>) => void;
  removeNode: (id: string) => void;
  
  addRoom: (room: RoomData) => void;
  updateRoom: (id: string, updates: Partial<RoomData>) => void;
  removeRoom: (id: string) => void;
  
  setSelectedEntity: (id: string | null, type: 'sensor' | 'node' | 'room' | null) => void;
  setViewMode: (mode: '3d' | 'heatmap' | 'network') => void;
  setPaused: (paused: boolean) => void;
  setCurrentTime: (time: number) => void;
  setToolMode: (mode: 'select' | 'move' | 'rotate' | 'scale') => void;
  
  // Helpers
  attachSensorToNode: (sensorId: string, nodeId: string) => void;
  detachSensorFromNode: (sensorId: string) => void;
}

export const useSimulationStore = create<SimulationState>()(
  immer((set) => ({
    // Initial state
    sensors: {},
    nodes: {},
    rooms: {},
    automationRules: {},
    
    selectedEntityId: null,
    selectedEntityType: null,
    viewMode: '3d',
    isPaused: false,
    currentTime: 0,
    toolMode: 'select',
    
    // Sensor actions
    addSensor: (sensor) =>
      set((state) => {
        state.sensors[sensor.id] = sensor;
      }),
    
    updateSensor: (id, updates) =>
      set((state) => {
        if (state.sensors[id]) {
          Object.assign(state.sensors[id], updates);
        }
      }),
    
    removeSensor: (id) =>
      set((state) => {
        delete state.sensors[id];
        // Remove from any nodes
        Object.values(state.nodes).forEach((node) => {
          const index = node.sensors.indexOf(id);
          if (index > -1) {
            node.sensors.splice(index, 1);
          }
        });
      }),
    
    // Node actions
    addNode: (node) =>
      set((state) => {
        state.nodes[node.id] = node;
      }),
    
    updateNode: (id, updates) =>
      set((state) => {
        if (state.nodes[id]) {
          Object.assign(state.nodes[id], updates);
        }
      }),
    
    removeNode: (id) =>
      set((state) => {
        // Detach all sensors from this node
        const node = state.nodes[id];
        if (node) {
          node.sensors.forEach((sensorId) => {
            if (state.sensors[sensorId]) {
              state.sensors[sensorId].nodeId = undefined;
            }
          });
        }
        delete state.nodes[id];
      }),
    
    // Room actions
    addRoom: (room) =>
      set((state) => {
        state.rooms[room.id] = room;
      }),
    
    updateRoom: (id, updates) =>
      set((state) => {
        if (state.rooms[id]) {
          Object.assign(state.rooms[id], updates);
        }
      }),
    
    removeRoom: (id) =>
      set((state) => {
        delete state.rooms[id];
      }),
    
    // UI actions
    setSelectedEntity: (id, type) =>
      set((state) => {
        state.selectedEntityId = id;
        state.selectedEntityType = type;
      }),
    
    setViewMode: (mode) =>
      set((state) => {
        state.viewMode = mode;
      }),
    
    setPaused: (paused) =>
      set((state) => {
        state.isPaused = paused;
      }),
    
    setCurrentTime: (time) =>
      set((state) => {
        state.currentTime = time;
      }),

    setToolMode: (mode) =>
      set((state) => {
        state.toolMode = mode;
      }),
    
    // Helper actions
    attachSensorToNode: (sensorId, nodeId) =>
      set((state) => {
        const sensor = state.sensors[sensorId];
        const node = state.nodes[nodeId];
        
        if (sensor && node) {
          // Remove from previous node if any
          if (sensor.nodeId && state.nodes[sensor.nodeId]) {
            const prevNode = state.nodes[sensor.nodeId];
            const index = prevNode.sensors.indexOf(sensorId);
            if (index > -1) {
              prevNode.sensors.splice(index, 1);
            }
          }
          
          // Attach to new node
          sensor.nodeId = nodeId;
          if (!node.sensors.includes(sensorId)) {
            node.sensors.push(sensorId);
          }
        }
      }),
    
    detachSensorFromNode: (sensorId) =>
      set((state) => {
        const sensor = state.sensors[sensorId];
        if (sensor && sensor.nodeId) {
          const node = state.nodes[sensor.nodeId];
          if (node) {
            const index = node.sensors.indexOf(sensorId);
            if (index > -1) {
              node.sensors.splice(index, 1);
            }
          }
          sensor.nodeId = undefined;
        }
      }),
  }))
);