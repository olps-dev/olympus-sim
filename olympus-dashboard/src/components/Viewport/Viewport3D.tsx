import React, { Suspense, useRef } from 'react';
import { Canvas } from '@react-three/fiber';
import { 
  OrbitControls, 
  Grid, 
  PerspectiveCamera,
  Environment,
  Stats
} from '@react-three/drei';
import { Scene } from './Scene';
import { Toolbar } from './Toolbar';
import { ViewCube } from './ViewCube';
import { ToolIndicator } from '../Layout/ToolIndicator';
import { useKeyboardShortcuts } from '../../hooks/useKeyboardShortcuts';

export const Viewport3D: React.FC = () => {
  const controlsRef = useRef<any>(null);
  useKeyboardShortcuts();

  return (
    <div className="relative w-full h-full bg-cyber-darker">
      <Canvas
        shadows
        camera={{ position: [10, 8, 10], fov: 60 }}
        gl={{ antialias: true, alpha: true }}
      >
        <color attach="background" args={['#0f0f23']} />
        <fog attach="fog" args={['#0f0f23', 40, 100]} />
        
        {/* Improved lighting setup */}
        <ambientLight intensity={0.3} color="#ffffff" />
        
        {/* Main directional light (sun/room lighting) */}
        <directionalLight
          position={[15, 20, 10]}
          intensity={0.8}
          color="#fff8dc"
          castShadow
          shadow-mapSize={[4096, 4096]}
          shadow-camera-far={50}
          shadow-camera-left={-20}
          shadow-camera-right={20}
          shadow-camera-top={20}
          shadow-camera-bottom={-20}
        />
        
        {/* Warm room lighting */}
        <pointLight
          position={[0, 4, 0]}
          intensity={0.4}
          color="#ffee88"
          distance={15}
          decay={2}
        />
        
        {/* Kitchen area lighting */}
        <pointLight
          position={[-4, 3, -3]}
          intensity={0.3}
          color="#ffffff"
          distance={8}
          decay={2}
        />
        
        {/* Entrance area lighting */}
        <pointLight
          position={[4, 3, 3]}
          intensity={0.25}
          color="#ffffff"
          distance={6}
          decay={2}
        />
        
        <Suspense fallback={null}>
          <Scene />
          <gridHelper args={[20, 20, "#00d4ff", "#00d4ff"]} />
        </Suspense>
        
        <OrbitControls
          ref={controlsRef}
          target={[0, 0, 0]}
          enableDamping
          dampingFactor={0.05}
          rotateSpeed={0.5}
          panSpeed={0.5}
          makeDefault
        />
        
        <PerspectiveCamera makeDefault position={[10, 8, 10]} />
        
        {/* Development helpers */}
        <Stats className="!left-auto !right-0" />
      </Canvas>
      
      {/* Floating UI Elements */}
      <Toolbar />
      <ViewCube controlsRef={controlsRef} />
      <ToolIndicator />
    </div>
  );
};