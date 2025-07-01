import React, { useEffect, useRef, useState } from 'react';
import * as Select from '@radix-ui/react-select';
import { ChevronDownIcon } from '@radix-ui/react-icons';
import { useSimulationStore } from '../../store/simulationStore';

export const HeatmapView: React.FC = () => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const { sensors, rooms } = useSimulationStore();
  const [heatmapType, setHeatmapType] = useState<'activity' | 'temperature' | 'coverage'>('activity');

  // Generate activity data based on actual sensors
  const generateActivityData = () => {
    const activityPoints: Array<{x: number, z: number, intensity: number}> = [];
    
    // Use active sensors as activity centers
    Object.values(sensors).forEach(sensor => {
      if (sensor.status === 'active' && ['mmwave', 'camera', 'pir', 'audio'].includes(sensor.type)) {
        // Base intensity on sensor type
        let baseIntensity = 0.5;
        if (sensor.type === 'mmwave') baseIntensity = 0.9;
        if (sensor.type === 'camera') baseIntensity = 0.8;
        if (sensor.type === 'pir') baseIntensity = 0.7;
        if (sensor.type === 'audio') baseIntensity = 0.6;
        
        // Create activity cluster around each active sensor
        const numPoints = Math.floor(baseIntensity * 20);
        for (let i = 0; i < numPoints; i++) {
          const spread = sensor.range ? sensor.range * 0.3 : 2;
          activityPoints.push({
            x: sensor.position.x + (Math.random() - 0.5) * spread,
            z: sensor.position.z + (Math.random() - 0.5) * spread,
            intensity: baseIntensity * (0.6 + Math.random() * 0.4)
          });
        }
      }
    });
    
    return activityPoints;
  };

  const generateTemperatureData = () => {
    const tempPoints: Array<{x: number, z: number, temperature: number}> = [];
    
    // Use actual temperature sensors
    const temperatureSensors = Object.values(sensors).filter(s => s.type === 'temperature');
    
    if (temperatureSensors.length === 0) {
      // If no temperature sensors, create a basic ambient temperature map
      const baseTemp = 21;
      for (let x = -6; x <= 6; x += 2) {
        for (let z = -6; z <= 6; z += 2) {
          tempPoints.push({
            x,
            z,
            temperature: baseTemp + (Math.random() - 0.5) * 2
          });
        }
      }
    } else {
      // Create temperature fields around each temperature sensor
      temperatureSensors.forEach(sensor => {
        const baseTemp = sensor.lastReading?.temperature || 22;
        const spread = sensor.range || 3;
        
        // Create temperature gradient around sensor
        for (let i = 0; i < 15; i++) {
          const distance = Math.random() * spread;
          const angle = Math.random() * Math.PI * 2;
          
          tempPoints.push({
            x: sensor.position.x + Math.cos(angle) * distance,
            z: sensor.position.z + Math.sin(angle) * distance,
            temperature: baseTemp + (Math.random() - 0.5) * 1 // Less variation near actual sensors
          });
        }
      });
    }
    
    return tempPoints;
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas
    ctx.fillStyle = '#0a0a0f';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Calculate bounds
    let minX = Infinity, maxX = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;

    Object.values(rooms).forEach(room => {
      minX = Math.min(minX, room.position.x - room.dimensions.x / 2);
      maxX = Math.max(maxX, room.position.x + room.dimensions.x / 2);
      minZ = Math.min(minZ, room.position.z - room.dimensions.z / 2);
      maxZ = Math.max(maxZ, room.position.z + room.dimensions.z / 2);
    });

    const padding = 20;
    const scaleX = (canvas.width - padding * 2) / (maxX - minX);
    const scaleZ = (canvas.height - padding * 2) / (maxZ - minZ);
    const scale = Math.min(scaleX, scaleZ);

    // Helper function to convert world coords to canvas coords
    const toCanvas = (x: number, z: number) => ({
      x: padding + (x - minX) * scale,
      y: padding + (z - minZ) * scale
    });

    // Draw rooms
    ctx.strokeStyle = '#00d4ff30';
    ctx.lineWidth = 1;
    Object.values(rooms).forEach(room => {
      const topLeft = toCanvas(
        room.position.x - room.dimensions.x / 2,
        room.position.z - room.dimensions.z / 2
      );
      ctx.strokeRect(
        topLeft.x,
        topLeft.y,
        room.dimensions.x * scale,
        room.dimensions.z * scale
      );
    });

    // Draw heatmap based on type
    if (heatmapType === 'activity') {
      const activityData = generateActivityData();
      activityData.forEach(point => {
        const pos = toCanvas(point.x, point.z);
        const intensity = point.intensity;
        
        const gradient = ctx.createRadialGradient(
          pos.x, pos.y, 0,
          pos.x, pos.y, 30 * intensity
        );
        
        const alpha = intensity * 0.6;
        gradient.addColorStop(0, `rgba(255, ${50 * (1 - intensity)}, 0, ${alpha})`);
        gradient.addColorStop(0.5, `rgba(255, ${100 * (1 - intensity)}, 0, ${alpha * 0.5})`);
        gradient.addColorStop(1, 'transparent');
        
        ctx.fillStyle = gradient;
        ctx.fillRect(pos.x - 30, pos.y - 30, 60, 60);
      });
    } else if (heatmapType === 'temperature') {
      const tempData = generateTemperatureData();
      tempData.forEach(point => {
        const pos = toCanvas(point.x, point.z);
        const temp = point.temperature;
        
        const gradient = ctx.createRadialGradient(
          pos.x, pos.y, 0,
          pos.x, pos.y, 25
        );
        
        // Color based on temperature (blue = cold, red = hot)
        const normalized = (temp - 18) / 8; // 18-26°C range
        const red = Math.max(0, Math.min(255, normalized * 255));
        const blue = Math.max(0, Math.min(255, (1 - normalized) * 255));
        
        gradient.addColorStop(0, `rgba(${red}, 100, ${blue}, 0.5)`);
        gradient.addColorStop(1, 'transparent');
        
        ctx.fillStyle = gradient;
        ctx.fillRect(pos.x - 25, pos.y - 25, 50, 50);
      });
    } else {
      // Coverage view - show sensor ranges
      Object.values(sensors).forEach(sensor => {
        if (sensor.status !== 'active') return;
        const pos = toCanvas(sensor.position.x, sensor.position.z);
        const range = sensor.range ? sensor.range * scale : 40;
        
        ctx.strokeStyle = '#00d4ff40';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, range, 0, Math.PI * 2);
        ctx.stroke();
      });
    }

    // Draw sensor positions
    Object.values(sensors).forEach(sensor => {
      const pos = toCanvas(sensor.position.x, sensor.position.z);
      
      ctx.fillStyle = sensor.status === 'active' ? '#00ff88' : '#ff0000';
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, 3, 0, Math.PI * 2);
      ctx.fill();
    });

  }, [sensors, rooms, heatmapType]);

  return (
    <div className="w-full h-full p-4 overflow-hidden">
      {/* Heatmap type selector */}
      <div className="mb-4 flex items-center gap-2">
        <span className="text-sm text-gray-400">View:</span>
        <Select.Root value={heatmapType} onValueChange={(value) => setHeatmapType(value as any)}>
          <Select.Trigger className="flex items-center gap-2 px-3 py-1 bg-cyber-darker border border-panel-border rounded text-sm text-white">
            <Select.Value />
            <ChevronDownIcon />
          </Select.Trigger>
          <Select.Portal>
            <Select.Content className="bg-cyber-darker border border-panel-border rounded shadow-xl">
              <Select.Viewport>
                <Select.Item value="activity" className="px-3 py-2 text-sm text-white hover:bg-cyber-blue/20 cursor-pointer">
                  <Select.ItemText>Activity Heatmap</Select.ItemText>
                </Select.Item>
                <Select.Item value="temperature" className="px-3 py-2 text-sm text-white hover:bg-cyber-blue/20 cursor-pointer">
                  <Select.ItemText>Temperature Map</Select.ItemText>
                </Select.Item>
                <Select.Item value="coverage" className="px-3 py-2 text-sm text-white hover:bg-cyber-blue/20 cursor-pointer">
                  <Select.ItemText>Sensor Coverage</Select.ItemText>
                </Select.Item>
              </Select.Viewport>
            </Select.Content>
          </Select.Portal>
        </Select.Root>
      </div>

      <canvas
        ref={canvasRef}
        width={600}
        height={350}
        className="w-full flex-1 bg-cyber-darker rounded"
      />
      
      <div className="mt-2 flex gap-4 text-xs text-gray-400">
        {heatmapType === 'activity' && (
          <>
            <div className="flex items-center gap-2">
              <div className="w-3 h-3 bg-gradient-to-r from-red-500 to-transparent rounded-full"></div>
              <span>High Activity</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-3 h-3 bg-gradient-to-r from-yellow-500 to-transparent rounded-full"></div>
              <span>Low Activity</span>
            </div>
          </>
        )}
        {heatmapType === 'temperature' && (
          <>
            <div className="flex items-center gap-2">
              <div className="w-3 h-3 bg-red-500 rounded-full"></div>
              <span>Warm (24°C+)</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-3 h-3 bg-blue-500 rounded-full"></div>
              <span>Cool (20°C)</span>
            </div>
          </>
        )}
        {heatmapType === 'coverage' && (
          <div className="flex items-center gap-2">
            <div className="w-3 h-3 border border-cyber-blue rounded-full"></div>
            <span>Sensor Range</span>
          </div>
        )}
      </div>
    </div>
  );
};