import React from 'react';
import { PlayIcon, PauseIcon, StopIcon } from '@radix-ui/react-icons';
import { useSimulationStore } from '../../store/simulationStore';

export const Header: React.FC = () => {
  const { isPaused, setPaused } = useSimulationStore();

  return (
    <header className="h-12 bg-cyber-darker border-b border-panel-border flex items-center justify-between px-4">
      <div className="flex items-center gap-4">
        <h1 className="text-xl font-bold text-cyber-blue glow-text">OLYMPUS</h1>
        <span className="text-sm text-gray-400">Digital Twin Simulator</span>
      </div>

      <div className="flex items-center gap-2">
        <button
          onClick={() => setPaused(!isPaused)}
          className="p-2 rounded bg-cyber-blue/20 hover:bg-cyber-blue/30 transition-colors"
        >
          {isPaused ? (
            <PlayIcon className="w-4 h-4 text-cyber-blue" />
          ) : (
            <PauseIcon className="w-4 h-4 text-cyber-blue" />
          )}
        </button>
        <button className="p-2 rounded bg-red-500/20 hover:bg-red-500/30 transition-colors">
          <StopIcon className="w-4 h-4 text-red-400" />
        </button>
      </div>

      <div className="flex items-center gap-2">
        <div className="flex items-center gap-2 text-sm">
          <div className="w-2 h-2 rounded-full bg-cyber-green animate-pulse"></div>
          <span className="text-gray-400">Connected</span>
        </div>
      </div>
    </header>
  );
};