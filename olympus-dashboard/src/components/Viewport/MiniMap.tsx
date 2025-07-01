import React from 'react';
import { useSimulationStore } from '../../store/simulationStore';

export const MiniMap: React.FC = () => {
  const { rooms, sensors, nodes } = useSimulationStore();

  const scale = 0.02;
  const width = 200;
  const height = 150;

  return (
    <div className="absolute top-4 right-4 w-[200px] h-[150px] bg-panel-bg border border-panel-border rounded-lg overflow-hidden">
      <svg width={width} height={height} className="w-full h-full">
        <g transform={`translate(${width/2}, ${height/2})`}>
          {/* Grid */}
          <pattern id="grid" width="10" height="10" patternUnits="userSpaceOnUse">
            <path d="M 10 0 L 0 0 0 10" fill="none" stroke="#00d4ff" strokeWidth="0.5" opacity="0.2"/>
          </pattern>
          <rect x={-width/2} y={-height/2} width={width} height={height} fill="url(#grid)" />

          {/* Rooms */}
          {Object.values(rooms).map((room) => (
            <rect
              key={room.id}
              x={room.position.x * scale - (room.dimensions.x * scale) / 2}
              y={-room.position.z * scale - (room.dimensions.z * scale) / 2}
              width={room.dimensions.x * scale}
              height={room.dimensions.z * scale}
              fill="none"
              stroke={room.color}
              strokeWidth="1"
              opacity="0.5"
            />
          ))}

          {/* Nodes */}
          {Object.values(nodes).map((node) => (
            <circle
              key={node.id}
              cx={node.position.x * scale}
              cy={-node.position.z * scale}
              r="3"
              fill={
                node.type === 'zeus' ? '#00ff88' :
                node.type === 'mid_tier' ? '#00d4ff' :
                '#ffaa00'
              }
              opacity={node.status === 'online' ? 1 : 0.3}
            />
          ))}

          {/* Sensors */}
          {Object.values(sensors).map((sensor) => (
            <circle
              key={sensor.id}
              cx={sensor.position.x * scale}
              cy={-sensor.position.z * scale}
              r="2"
              fill="#ff006e"
              opacity={sensor.status === 'active' ? 1 : 0.3}
            />
          ))}
        </g>
      </svg>
      
      <div className="absolute top-1 left-1 text-xs text-gray-400">MiniMap</div>
    </div>
  );
};