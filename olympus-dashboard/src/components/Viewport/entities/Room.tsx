import React, { useRef } from 'react';
import { Box, Text, Plane } from '@react-three/drei';
import { Mesh } from 'three';
import * as THREE from 'three';
import { useSimulationStore, RoomData } from '../../../store/simulationStore';

interface RoomProps {
  data: RoomData;
}

export const Room: React.FC<RoomProps> = ({ data }) => {
  const meshRef = useRef<Mesh>(null);


  const getFloorTexture = () => {
    // Different floor textures based on room name
    if (data.name.toLowerCase().includes('kitchen')) {
      return '#f5f5f0'; // Light tile color
    } else if (data.name.toLowerCase().includes('bedroom')) {
      return '#8b7355'; // Wood color
    } else {
      return '#e8e8e8'; // Default light floor
    }
  };

  return (
    <group 
      position={[data.position.x, data.position.y, data.position.z]}
      rotation={data.rotation ? [data.rotation.x, data.rotation.y, data.rotation.z] : [0, 0, 0]}
    >
      {/* Floor */}
      <mesh
        ref={meshRef}
        rotation={[-Math.PI / 2, 0, 0]}
        position={[0, 0.01, 0]}
        receiveShadow
      >
        <planeGeometry args={[data.dimensions.x, data.dimensions.z]} />
        <meshStandardMaterial 
          color={getFloorTexture()} 
          opacity={0.9}
          transparent
        />
      </mesh>

      {/* Solid Walls - No Ceiling */}
      {/* Back Wall */}
      <mesh position={[0, data.dimensions.y / 2, -data.dimensions.z / 2]}>
        <boxGeometry args={[data.dimensions.x, data.dimensions.y, 0.2]} />
        <meshStandardMaterial color="#f5f5f5" />
      </mesh>

      {/* Front Wall with Door Opening */}
      <mesh position={[-data.dimensions.x / 4, data.dimensions.y / 2, data.dimensions.z / 2]}>
        <boxGeometry args={[data.dimensions.x / 2 - 1, data.dimensions.y, 0.2]} />
        <meshStandardMaterial color="#f5f5f5" />
      </mesh>
      <mesh position={[data.dimensions.x / 4, data.dimensions.y / 2, data.dimensions.z / 2]}>
        <boxGeometry args={[data.dimensions.x / 2 - 1, data.dimensions.y, 0.2]} />
        <meshStandardMaterial color="#f5f5f5" />
      </mesh>
      {/* Door Frame Top */}
      <mesh position={[0, data.dimensions.y - 0.4, data.dimensions.z / 2]}>
        <boxGeometry args={[2, 0.8, 0.2]} />
        <meshStandardMaterial color="#f5f5f5" />
      </mesh>

      {/* Left Wall */}
      <mesh position={[-data.dimensions.x / 2, data.dimensions.y / 2, 0]}>
        <boxGeometry args={[0.2, data.dimensions.y, data.dimensions.z]} />
        <meshStandardMaterial color="#f5f5f5" />
      </mesh>

      {/* Right Wall */}
      <mesh position={[data.dimensions.x / 2, data.dimensions.y / 2, 0]}>
        <boxGeometry args={[0.2, data.dimensions.y, data.dimensions.z]} />
        <meshStandardMaterial color="#f5f5f5" />
      </mesh>

      {/* Room label */}
      <Text
        position={[0, data.dimensions.y + 0.3, 0]}
        fontSize={0.4}
        color="#00d4ff"
        anchorX="center"
        anchorY="middle"
      >
        {data.name}
      </Text>

      {/* Floor pattern for visual interest */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.02, 0]}>
        <planeGeometry args={[data.dimensions.x * 0.9, data.dimensions.z * 0.9, 8, 8]} />
        <meshBasicMaterial 
          color={data.color} 
          wireframe 
          opacity={0.1}
          transparent
        />
      </mesh>
    </group>
  );
};