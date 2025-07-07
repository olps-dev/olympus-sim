import React, { useState } from 'react';
import { PlusIcon, TrashIcon } from '@radix-ui/react-icons';
import { useSimulationStore } from '../../store/simulationStore';

export const AutomationRules: React.FC = () => {
  const { automationRules, sensors } = useSimulationStore();
  const [isCreating, setIsCreating] = useState(false);

  const mockRules = [
    {
      id: 'rule1',
      name: 'Motion Light Control',
      trigger: { sensorId: 'sensor_mmwave1', condition: 'presence', value: true },
      action: { type: 'control', target: 'light_living_room', params: { state: 'on' } },
      enabled: true,
    },
    {
      id: 'rule2',
      name: 'Night Mode',
      trigger: { sensorId: 'sensor_light1', condition: 'brightness', value: 20 },
      action: { type: 'scene', target: 'night_scene', params: { dim: 0.2 } },
      enabled: true,
    },
  ];

  return (
    <div className="h-full p-4 flex flex-col">
      <div className="flex justify-between items-center mb-4">
        <h3 className="text-sm font-medium text-white">Automation Rules</h3>
        <button
          onClick={() => setIsCreating(true)}
          className="p-2 rounded bg-cyber-blue/20 hover:bg-cyber-blue/30 transition-colors"
        >
          <PlusIcon className="w-4 h-4 text-cyber-blue" />
        </button>
      </div>

      <div className="flex-1 overflow-auto space-y-3">
        {mockRules.map((rule) => (
          <div
            key={rule.id}
            className="bg-cyber-darker border border-panel-border rounded-lg p-4"
          >
            <div className="flex justify-between items-start mb-3">
              <div className="flex-1">
                <h4 className="text-sm font-medium text-white">{rule.name}</h4>
                <div className="flex items-center gap-2 mt-1">
                  <div
                    className={`w-2 h-2 rounded-full ${
                      rule.enabled ? 'bg-cyber-green' : 'bg-gray-600'
                    }`}
                  />
                  <span className="text-xs text-gray-400">
                    {rule.enabled ? 'Enabled' : 'Disabled'}
                  </span>
                </div>
              </div>
              <button className="p-1 rounded hover:bg-red-500/20 transition-colors">
                <TrashIcon className="w-4 h-4 text-gray-400 hover:text-red-400" />
              </button>
            </div>

            <div className="space-y-2">
              <div className="text-xs">
                <span className="text-gray-400">WHEN:</span>
                <span className="text-white ml-2">
                  {rule.trigger.sensorId} {rule.trigger.condition} = {rule.trigger.value?.toString()}
                </span>
              </div>
              <div className="text-xs">
                <span className="text-gray-400">THEN:</span>
                <span className="text-white ml-2">
                  {rule.action.type} {rule.action.target}
                </span>
              </div>
            </div>

            {/* Visual flow representation */}
            <div className="mt-3 flex items-center gap-2">
              <div className="w-8 h-6 bg-cyber-pink/20 border border-cyber-pink/40 rounded text-xs flex items-center justify-center text-cyber-pink">
                S
              </div>
              <div className="flex-1 h-px bg-cyber-blue"></div>
              <div className="w-8 h-6 bg-cyber-blue/20 border border-cyber-blue/40 rounded text-xs flex items-center justify-center text-cyber-blue">
                A
              </div>
            </div>
          </div>
        ))}

        {mockRules.length === 0 && (
          <div className="text-center text-gray-400 py-8">
            No automation rules configured
          </div>
        )}
      </div>

      {isCreating && (
        <div className="border-t border-panel-border pt-4 mt-4">
          <div className="bg-cyber-darker border border-panel-border rounded-lg p-4">
            <h4 className="text-sm font-medium text-white mb-3">New Rule</h4>
            <div className="space-y-3">
              <input
                type="text"
                placeholder="Rule name"
                className="w-full px-3 py-2 bg-cyber-darker border border-panel-border rounded text-sm text-white"
              />
              <div className="grid grid-cols-2 gap-2">
                <button
                  onClick={() => setIsCreating(false)}
                  className="px-3 py-2 text-sm bg-gray-600 hover:bg-gray-500 rounded transition-colors"
                >
                  Cancel
                </button>
                <button className="px-3 py-2 text-sm bg-cyber-blue hover:bg-cyber-blue/80 rounded transition-colors">
                  Create
                </button>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};