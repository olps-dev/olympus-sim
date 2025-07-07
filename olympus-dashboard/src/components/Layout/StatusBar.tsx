import React from 'react';
import { useSimulationStore } from '../../store/simulationStore';

export const StatusBar: React.FC = () => {
  const { sensors, nodes, currentTime } = useSimulationStore();
  
  const activeSensors = Object.values(sensors).filter(s => s.status === 'active').length;
  const onlineNodes = Object.values(nodes).filter(n => n.status === 'online').length;
  
  const formatTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  return (
    <footer className="h-6 bg-cyber-darker border-t border-panel-border flex items-center justify-between px-4 text-xs text-gray-400">
      <div className="flex items-center gap-4">
        <span>Sensors: {activeSensors}/{Object.keys(sensors).length}</span>
        <span>Nodes: {onlineNodes}/{Object.keys(nodes).length}</span>
      </div>
      
      <div className="flex items-center gap-4">
        <span>Time: {formatTime(currentTime)}</span>
        <span>FPS: 60</span>
        <span>Memory: 124MB</span>
      </div>
    </footer>
  );
};