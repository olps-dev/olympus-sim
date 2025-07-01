import React from 'react';
import { 
  Cone, 
  Box, 
  Sphere, 
  Cylinder, 
  Octahedron,
  Tetrahedron,
  Ring,
  Dodecahedron
} from '@react-three/drei';
import { useSimulationStore, SensorData } from '../../../store/simulationStore';
import { SensorFOV } from './SensorFOV';

interface SensorProps {
  data: SensorData;
}

export const Sensor: React.FC<SensorProps> = ({ data }) => {
  const { selectedEntityId, setSelectedEntity } = useSimulationStore();
  const isSelected = selectedEntityId === data.id;

  const handleClick = (e: any) => {
    e.stopPropagation();
    setSelectedEntity(data.id, 'sensor');
  };

  const getSensorGeometry = () => {
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

    const material = (
      <meshStandardMaterial 
        color={isSelected ? '#ffaa00' : getSensorColor()} 
        emissive={data.status === 'active' ? getSensorColor() : '#000000'}
        emissiveIntensity={data.status === 'active' ? 0.1 : 0}
        metalness={0.6}
        roughness={0.4}
        opacity={data.status === 'inactive' ? 0.5 : 1}
        transparent
      />
    );

    switch (data.type) {
      case 'mmwave':
        return (
          <Cone args={[0.3, 0.6, 8]} {...props} rotation={[Math.PI / 2, 0, 0]}>
            {material}
          </Cone>
        );
      case 'camera':
        return (
          <Box args={[0.4, 0.3, 0.5]} {...props} rotation={[0, 0, 0]}>
            {material}
          </Box>
        );
      case 'temperature':
        return (
          <Cylinder args={[0.2, 0.2, 0.4, 16]} {...props}>
            {material}
          </Cylinder>
        );
      case 'audio':
        return (
          <Sphere args={[0.25, 16, 16]} {...props}>
            {material}
          </Sphere>
        );
      case 'pir':
        return (
          <Octahedron args={[0.3]} {...props}>
            {material}
          </Octahedron>
        );
      case 'light':
        return (
          <Tetrahedron args={[0.3]} {...props}>
            {material}
          </Tetrahedron>
        );
      case 'pressure':
        return (
          <Ring args={[0.2, 0.3, 16]} {...props} rotation={[Math.PI / 2, 0, 0]}>
            {material}
          </Ring>
        );
      case 'humidity':
        return (
          <Dodecahedron args={[0.25]} {...props}>
            {material}
          </Dodecahedron>
        );
      default:
        return (
          <Box args={[0.3, 0.3, 0.3]} {...props}>
            {material}
          </Box>
        );
    }
  };

  const getSensorColor = () => {
    const colors: Record<string, string> = {
      mmwave: '#ff006e',
      camera: '#bd00ff',
      temperature: '#ff0000',
      audio: '#4b0082',
      pir: '#00ffff',
      light: '#ffff00',
      pressure: '#8b4513',
      humidity: '#4682b4',
    };
    return colors[data.type] || '#ffffff';
  };

  return (
    <group 
      position={[data.position.x, data.position.y, data.position.z]}
      rotation={data.rotation ? [data.rotation.x, data.rotation.y, data.rotation.z] : [0, 0, 0]}
    >
      {getSensorGeometry()}

      {/* FOV visualization for applicable sensors */}
      {(data.type === 'mmwave' || data.type === 'camera') && data.fov && (
        <SensorFOV 
          fov={data.fov} 
          range={data.range || 10} 
          color={getSensorColor()}
          active={data.status === 'active'}
        />
      )}

      {/* Subtle activity indicator - reduced intensity */}
      {data.status === 'active' && (
        <pointLight
          position={[0, 0, 0]}
          color={getSensorColor()}
          intensity={0.2}
          distance={2}
          decay={2}
        />
      )}

      {/* Status LED indicator */}
      <mesh position={[0, 0.3, 0]}>
        <sphereGeometry args={[0.05, 8, 8]} />
        <meshBasicMaterial 
          color={
            data.status === 'active' ? '#00ff00' :
            data.status === 'inactive' ? '#ffaa00' :
            '#ff0000'
          }
        />
      </mesh>
    </group>
  );
};