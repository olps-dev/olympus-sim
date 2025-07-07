import React, { useRef, useEffect } from 'react';
import { PlayIcon, PauseIcon, TriangleLeftIcon, TriangleRightIcon } from '@radix-ui/react-icons';
import * as Slider from '@radix-ui/react-slider';
import { useSimulationStore } from '../../store/simulationStore';

export const Timeline: React.FC = () => {
  const { currentTime, setCurrentTime, isPaused, setPaused } = useSimulationStore();
  const animationRef = useRef<number>();

  useEffect(() => {
    if (!isPaused) {
      const animate = () => {
        setCurrentTime(currentTime + 1);
        animationRef.current = requestAnimationFrame(animate);
      };
      animationRef.current = requestAnimationFrame(animate);
    }

    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, [isPaused, currentTime, setCurrentTime]);

  const formatTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  const handleSeek = (value: number[]) => {
    setCurrentTime(value[0]);
  };

  return (
    <div className="h-full p-4 flex flex-col">
      <div className="flex-1 overflow-auto">
        <h3 className="text-sm font-medium text-white mb-4">Simulation Timeline</h3>
        
        {/* Timeline events visualization would go here */}
        <div className="space-y-2">
          <div className="flex items-center gap-2">
            <div className="w-2 h-2 bg-cyber-blue rounded-full"></div>
            <span className="text-xs text-gray-400">System initialized</span>
          </div>
          <div className="flex items-center gap-2">
            <div className="w-2 h-2 bg-cyber-green rounded-full"></div>
            <span className="text-xs text-gray-400">All nodes online</span>
          </div>
          <div className="flex items-center gap-2">
            <div className="w-2 h-2 bg-cyber-pink rounded-full"></div>
            <span className="text-xs text-gray-400">Sensors activated</span>
          </div>
        </div>
      </div>

      <div className="border-t border-panel-border pt-4 space-y-4">
        {/* Time display */}
        <div className="flex justify-between items-center">
          <span className="text-2xl font-mono text-white">{formatTime(currentTime)}</span>
          <span className="text-sm text-gray-400">/ {formatTime(600)}</span>
        </div>

        {/* Timeline scrubber */}
        <Slider.Root
          value={[currentTime]}
          onValueChange={handleSeek}
          max={600}
          className="relative flex items-center select-none touch-none w-full h-5"
        >
          <Slider.Track className="bg-cyber-darker relative grow rounded-full h-2">
            <Slider.Range className="absolute bg-cyber-blue rounded-full h-full" />
          </Slider.Track>
          <Slider.Thumb className="block w-5 h-5 bg-white rounded-full hover:bg-gray-200 focus:outline-none focus:ring-2 focus:ring-cyber-blue" />
        </Slider.Root>

        {/* Playback controls */}
        <div className="flex justify-center items-center gap-2">
          <button
            onClick={() => setCurrentTime(Math.max(0, currentTime - 10))}
            className="p-2 rounded hover:bg-cyber-blue/20 transition-colors"
          >
            <TriangleLeftIcon className="w-4 h-4 text-gray-400" />
          </button>
          
          <button
            onClick={() => setPaused(!isPaused)}
            className="p-3 rounded-full bg-cyber-blue/20 hover:bg-cyber-blue/30 transition-colors"
          >
            {isPaused ? (
              <PlayIcon className="w-5 h-5 text-cyber-blue" />
            ) : (
              <PauseIcon className="w-5 h-5 text-cyber-blue" />
            )}
          </button>
          
          <button
            onClick={() => setCurrentTime(Math.min(600, currentTime + 10))}
            className="p-2 rounded hover:bg-cyber-blue/20 transition-colors"
          >
            <TriangleRightIcon className="w-4 h-4 text-gray-400" />
          </button>
        </div>

        {/* Speed controls */}
        <div className="flex justify-center gap-2">
          {[0.5, 1, 2, 4].map((speed) => (
            <button
              key={speed}
              className="px-2 py-1 text-xs rounded bg-cyber-darker hover:bg-cyber-blue/20 transition-colors"
            >
              {speed}x
            </button>
          ))}
        </div>
      </div>
    </div>
  );
};