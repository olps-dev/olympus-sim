import React from 'react';
import { Line } from '@react-three/drei';
import { useSimulationStore } from '../../../store/simulationStore';

export const ConnectionLines: React.FC = () => {
  const { sensors, nodes } = useSimulationStore();

  const connections = React.useMemo(() => {
    const lines: Array<{ from: [number, number, number]; to: [number, number, number]; color: string }> = [];

    Object.values(sensors).forEach((sensor) => {
      if (sensor.nodeId && nodes[sensor.nodeId]) {
        const node = nodes[sensor.nodeId];
        lines.push({
          from: [sensor.position.x, sensor.position.y, sensor.position.z],
          to: [node.position.x, node.position.y + 0.5, node.position.z],
          color: sensor.status === 'active' ? '#00d4ff' : '#404040',
        });
      }
    });

    return lines;
  }, [sensors, nodes]);

  return (
    <>
      {connections.map((connection, index) => (
        <Line
          key={index}
          points={[connection.from, connection.to]}
          color={connection.color}
          lineWidth={1}
          opacity={0.5}
          transparent
          dashed
          dashScale={5}
          dashSize={0.1}
          gapSize={0.1}
        />
      ))}
    </>
  );
};