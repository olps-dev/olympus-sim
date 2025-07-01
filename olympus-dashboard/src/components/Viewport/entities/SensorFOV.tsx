import React, { useMemo } from 'react';
import * as THREE from 'three';

interface SensorFOVProps {
  fov: number;
  range: number;
  color: string;
  active: boolean;
}

export const SensorFOV: React.FC<SensorFOVProps> = ({ fov, range, color, active }) => {
  const geometry = useMemo(() => {
    // Ensure minimum range to prevent invalid geometry
    const safeRange = Math.max(range, 0.1);
    const geometry = new THREE.ConeGeometry(
      safeRange * Math.tan((fov * Math.PI) / 360),
      safeRange,
      32,
      1,
      true
    );
    geometry.translate(0, -safeRange / 2, 0);
    geometry.rotateX(Math.PI);
    return geometry;
  }, [fov, range]);

  // Don't render if range is too small or inactive
  if (!active || range < 0.1) return null;

  return (
    <mesh geometry={geometry}>
      <meshBasicMaterial
        color={color}
        transparent
        opacity={0.05}
        side={THREE.DoubleSide}
      />
      <lineSegments>
        <edgesGeometry args={[geometry]} />
        <lineBasicMaterial color={color} opacity={0.2} transparent />
      </lineSegments>
    </mesh>
  );
};