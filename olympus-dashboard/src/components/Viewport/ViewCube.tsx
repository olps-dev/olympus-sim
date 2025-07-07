import React from 'react';

interface ViewCubeProps {
  controlsRef: React.RefObject<any>;
}

export const ViewCube: React.FC<ViewCubeProps> = ({ controlsRef }) => {
  const setView = (position: [number, number, number]) => {
    if (controlsRef.current) {
      controlsRef.current.setLookAt(
        position[0] * 20, position[1] * 20, position[2] * 20,
        0, 0, 0,
        true
      );
    }
  };

  return (
    <div className="absolute bottom-4 left-4 w-20 h-20">
      <div className="relative w-full h-full">
        {/* Cube faces */}
        <button
          onClick={() => setView([0, 1, 0])}
          className="absolute top-0 left-1/2 -translate-x-1/2 px-2 py-1 text-xs bg-cyber-blue/20 hover:bg-cyber-blue/40 text-cyan-400 rounded transition-colors"
        >
          TOP
        </button>
        <button
          onClick={() => setView([0, -1, 0])}
          className="absolute bottom-0 left-1/2 -translate-x-1/2 px-2 py-1 text-xs bg-cyber-blue/20 hover:bg-cyber-blue/40 text-cyan-400 rounded transition-colors"
        >
          BTM
        </button>
        <button
          onClick={() => setView([-1, 0, 0])}
          className="absolute left-0 top-1/2 -translate-y-1/2 px-1 py-1 text-xs bg-cyber-blue/20 hover:bg-cyber-blue/40 text-cyan-400 rounded transition-colors"
        >
          L
        </button>
        <button
          onClick={() => setView([1, 0, 0])}
          className="absolute right-0 top-1/2 -translate-y-1/2 px-1 py-1 text-xs bg-cyber-blue/20 hover:bg-cyber-blue/40 text-cyan-400 rounded transition-colors"
        >
          R
        </button>
        <button
          onClick={() => setView([0, 0, 1])}
          className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 px-1 py-1 text-xs bg-cyber-blue/20 hover:bg-cyber-blue/40 text-cyan-400 rounded transition-colors"
        >
          F
        </button>
      </div>
    </div>
  );
};