import React from 'react';
import { Box, Cylinder } from '@react-three/drei';

interface FurnitureProps {
  type: 'sofa' | 'table' | 'kitchen_counter' | 'door' | 'window';
  position: [number, number, number];
  rotation?: [number, number, number];
  scale?: [number, number, number];
}

export const Furniture: React.FC<FurnitureProps> = ({ 
  type, 
  position, 
  rotation = [0, 0, 0], 
  scale = [1, 1, 1] 
}) => {
  const renderFurniture = () => {
    switch (type) {
      case 'sofa':
        return (
          <group>
            {/* Sofa base */}
            <Box args={[2, 0.4, 0.8]} position={[0, 0.2, 0]}>
              <meshLambertMaterial color="#4a4a4a" />
            </Box>
            {/* Sofa back */}
            <Box args={[2, 0.6, 0.2]} position={[0, 0.5, -0.3]}>
              <meshLambertMaterial color="#4a4a4a" />
            </Box>
            {/* Sofa arms */}
            <Box args={[0.2, 0.6, 0.8]} position={[-0.9, 0.5, 0]}>
              <meshLambertMaterial color="#4a4a4a" />
            </Box>
            <Box args={[0.2, 0.6, 0.8]} position={[0.9, 0.5, 0]}>
              <meshLambertMaterial color="#4a4a4a" />
            </Box>
          </group>
        );
      
      case 'table':
        return (
          <group>
            {/* Table top */}
            <Box args={[1.2, 0.1, 0.8]} position={[0, 0.75, 0]}>
              <meshLambertMaterial color="#8b7355" />
            </Box>
            {/* Table legs */}
            {[[-0.5, -0.5], [0.5, -0.5], [-0.5, 0.3], [0.5, 0.3]].map((pos, i) => (
              <Cylinder key={i} args={[0.03, 0.03, 0.7]} position={[pos[0], 0.35, pos[1]]}>
                <meshLambertMaterial color="#6b5b3f" />
              </Cylinder>
            ))}
          </group>
        );
      
      case 'kitchen_counter':
        return (
          <group>
            {/* Counter base */}
            <Box args={[3, 0.9, 0.6]} position={[0, 0.45, 0]}>
              <meshLambertMaterial color="#f5f5f0" />
            </Box>
            {/* Counter top */}
            <Box args={[3.1, 0.05, 0.65]} position={[0, 0.92, 0]}>
              <meshLambertMaterial color="#2c2c2c" />
            </Box>
          </group>
        );
      
      case 'door':
        return (
          <Box args={[0.05, 2, 0.8]} position={[0, 1, 0]}>
            <meshLambertMaterial color="#8b7355" />
          </Box>
        );
      
      case 'window':
        return (
          <Box args={[0.05, 1, 1.5]} position={[0, 1.5, 0]}>
            <meshLambertMaterial color="#87ceeb" opacity={0.6} transparent />
          </Box>
        );
      
      default:
        return null;
    }
  };

  return (
    <group 
      position={position} 
      rotation={rotation} 
      scale={scale}
    >
      {renderFurniture()}
    </group>
  );
};