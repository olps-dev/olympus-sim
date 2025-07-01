import React from 'react';
import { Box, Sphere, Cylinder, Text } from '@react-three/drei';
import { useSimulationStore, NodeData } from '../../../store/simulationStore';

interface NodeProps {
  data: NodeData;
}

export const Node: React.FC<NodeProps> = ({ data }) => {
  const { selectedEntityId, setSelectedEntity } = useSimulationStore();
  const isSelected = selectedEntityId === data.id;


  const handleClick = (e: any) => {
    e.stopPropagation();
    setSelectedEntity(data.id, 'node');
  };

  const getNodeGeometry = () => {
    const props = {
      castShadow: true,
      receiveShadow: true,
      onClick: handleClick,
      onPointerOver: (e: any) => {
        e.stopPropagation();
        document.body.style.cursor = 'pointer';
      },
      onPointerOut: (e: any) => {
        e.stopPropagation();
        document.body.style.cursor = 'default';
      },
    };

    switch (data.type) {
      case 'zeus':
        return (
          <Box args={[1.5, 1.5, 1.5]} {...props}>
            <meshStandardMaterial 
              color={isSelected ? '#ffaa00' : '#00ff88'} 
              emissive={data.status === 'online' ? '#00ff88' : '#000000'}
              emissiveIntensity={data.status === 'online' ? 0.3 : 0}
              metalness={0.9}
              roughness={0.1}
            />
          </Box>
        );
      case 'mid_tier':
        return (
          <Cylinder args={[0.8, 0.8, 1.2, 8]} {...props}>
            <meshStandardMaterial 
              color={isSelected ? '#ffaa00' : '#00d4ff'} 
              emissive={data.status === 'online' ? '#00d4ff' : '#000000'}
              emissiveIntensity={data.status === 'online' ? 0.3 : 0}
              metalness={0.9}
              roughness={0.1}
            />
          </Cylinder>
        );
      case 'low_power':
        return (
          <Sphere args={[0.6, 16, 16]} {...props}>
            <meshStandardMaterial 
              color={isSelected ? '#ffaa00' : '#ff8800'} 
              emissive={data.status === 'online' ? '#ff8800' : '#000000'}
              emissiveIntensity={data.status === 'online' ? 0.3 : 0}
              metalness={0.9}
              roughness={0.1}
            />
          </Sphere>
        );
    }
  };

  return (
    <group 
      position={[data.position.x, data.position.y, data.position.z]}
      rotation={data.rotation ? [data.rotation.x, data.rotation.y, data.rotation.z] : [0, 0, 0]}
    >
      {getNodeGeometry()}
      

      {/* Status indicator */}
      <Sphere 
        args={[0.1, 8, 8]} 
        position={[0, data.type === 'zeus' ? 1 : 0.8, 0]}
      >
        <meshBasicMaterial 
          color={
            data.status === 'online' ? '#00ff88' :
            data.status === 'offline' ? '#ff0000' :
            '#ffaa00'
          } 
        />
      </Sphere>

      {/* Label */}
      <Text
        position={[0, -1, 0]}
        fontSize={0.3}
        color={isSelected ? '#ffaa00' : '#ffffff'}
        anchorX="center"
        anchorY="middle"
      >
        {data.id}
      </Text>
    </group>
  );
};