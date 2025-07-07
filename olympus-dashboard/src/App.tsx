import React, { useEffect } from 'react';
import { Header } from './components/Layout/Header';
import { StatusBar } from './components/Layout/StatusBar';
import { ResizableLayout } from './components/Layout/ResizableLayout';
import { Viewport3D } from './components/Viewport/Viewport3D';
import { SensorDataView } from './components/SensorView/SensorDataView';
import { PropertiesPanel } from './components/Properties/PropertiesPanel';
import { useSimulationStore } from './store/simulationStore';
import { websocketService } from './services/websocket';

function App() {
  const { addSensor, addNode, addRoom } = useSimulationStore();

  // Initialize WebSocket connection and demo data
  useEffect(() => {
    // Try to connect to backend (optional)
    websocketService.connect().catch(() => {
      console.log('Running in demo mode - backend not available');
    });

    return () => websocketService.disconnect();
  }, []);

  // Initialize with demo data
  useEffect(() => {
    // Add a demo room
    addRoom({
      id: 'room_living',
      name: 'Living Room',
      position: { x: 0, y: 0, z: 0 },
      dimensions: { x: 12, y: 3, z: 10 },
      color: '#00d4ff',
    });

    // Add demo nodes in logical positions
    addNode({
      id: 'zeus_hub',
      type: 'zeus',
      position: { x: 0, y: 0.5, z: 0 }, // Center of room
      status: 'online',
      sensors: [],
      capabilities: {
        processing_power: 85,
        memory: 32,
        network_bandwidth: 1000,
      },
    });

    addNode({
      id: 'mid_tier_kitchen',
      type: 'mid_tier',
      position: { x: -4, y: 0.5, z: -3 }, // Kitchen area
      status: 'online',
      sensors: [],
      capabilities: {
        processing_power: 60,
        memory: 16,
        network_bandwidth: 100,
      },
    });

    addNode({
      id: 'low_power_entrance',
      type: 'low_power',
      position: { x: 4, y: 0.5, z: 3 }, // Near entrance
      status: 'online',
      sensors: [],
      capabilities: {
        processing_power: 20,
        memory: 4,
        network_bandwidth: 10,
      },
    });

    // Add demo sensors in logical positions with proper orientations
    addSensor({
      id: 'mmwave_entrance',
      type: 'mmwave',
      position: { x: 4.5, y: 2.5, z: 3.5 }, // Wall-mounted at entrance
      rotation: { x: 0, y: -Math.PI / 4, z: 0 }, // Pointing toward entrance area
      status: 'active',
      fov: 60,
      range: 8,
      nodeId: 'low_power_entrance',
    });

    addSensor({
      id: 'camera_overview',
      type: 'camera',
      position: { x: -5, y: 2.8, z: -4 }, // Corner ceiling mount
      rotation: { x: -Math.PI / 6, y: Math.PI / 4, z: 0 }, // Angled down toward room center
      status: 'active',
      fov: 90,
      range: 15,
      nodeId: 'zeus_hub',
    });

    addSensor({
      id: 'temp_center',
      type: 'temperature',
      position: { x: 1, y: 1.5, z: 1 }, // Room center at human height
      status: 'active',
      nodeId: 'zeus_hub',
    });

    addSensor({
      id: 'pir_kitchen',
      type: 'pir',
      position: { x: -4.5, y: 2.2, z: -2 }, // Kitchen corner
      rotation: { x: 0, y: Math.PI / 3, z: 0 }, // Pointing toward kitchen work area
      status: 'active',
      range: 5,
      nodeId: 'mid_tier_kitchen',
    });

    addSensor({
      id: 'light_sensor',
      type: 'light',
      position: { x: 2, y: 2.8, z: -2 }, // Ceiling mounted
      status: 'active',
      nodeId: 'zeus_hub',
    });

    addSensor({
      id: 'audio_center',
      type: 'audio',
      position: { x: -1, y: 2.5, z: 2 }, // Central ceiling position
      status: 'active',
      nodeId: 'zeus_hub',
    });
  }, [addSensor, addNode, addRoom]);

  return (
    <div className="w-screen h-screen overflow-hidden bg-cyber-dark text-white flex flex-col">
      <Header />
      
      <ResizableLayout>
        <Viewport3D />
        <SensorDataView />
        <PropertiesPanel />
      </ResizableLayout>
      
      <StatusBar />
    </div>
  );
}

export default App;