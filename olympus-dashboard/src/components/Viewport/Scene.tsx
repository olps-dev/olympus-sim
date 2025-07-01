import React from 'react';
import { useSimulationStore } from '../../store/simulationStore';
import { Room } from './entities/Room';
import { Node } from './entities/Node';
import { Sensor } from './entities/Sensor';
import { ConnectionLines } from './entities/ConnectionLines';
import { Furniture } from './entities/Furniture';

export const Scene: React.FC = () => {
  const { 
    rooms, 
    nodes, 
    sensors
  } = useSimulationStore();


  return (
    <>
      {/* Render Rooms */}
      {Object.values(rooms).map((room) => (
        <Room key={room.id} data={room} />
      ))}

      {/* Render Nodes */}
      {Object.values(nodes).map((node) => (
        <Node key={node.id} data={node} />
      ))}

      {/* Render Sensors */}
      {Object.values(sensors).map((sensor) => (
        <Sensor key={sensor.id} data={sensor} />
      ))}

      {/* Render Connections */}
      <ConnectionLines />

      {/* Add Furniture for context - temporarily disabled */}
      {/* <Furniture type="sofa" position={[2, 0, 2]} rotation={[0, Math.PI / 2, 0]} />
      <Furniture type="table" position={[1, 0, 2.5]} />
      <Furniture type="kitchen_counter" position={[-4, 0, -3]} rotation={[0, Math.PI / 2, 0]} /> */}

      {/* Transform Controls temporarily disabled due to bugs */}
      {/* TODO: Fix transform controls to properly attach to 3D meshes */}
    </>
  );
};