import React from 'react';
import { useSimulationStore } from '../../store/simulationStore';

export const NetworkView: React.FC = () => {
  const { nodes, sensors } = useSimulationStore();

  const getNodeIcon = (type: string) => {
    switch (type) {
      case 'zeus':
        return '⬢'; // Hexagon
      case 'mid_tier':
        return '⬡'; // Octagon
      case 'low_power':
        return '●'; // Circle
      default:
        return '■';
    }
  };

  const getNodeColor = (type: string, status: string) => {
    if (status !== 'online') return 'text-gray-600';
    switch (type) {
      case 'zeus':
        return 'text-cyber-green';
      case 'mid_tier':
        return 'text-cyber-blue';
      case 'low_power':
        return 'text-cyber-yellow';
      default:
        return 'text-white';
    }
  };

  return (
    <div className="h-full overflow-auto p-4">
      <div className="space-y-4">
        {Object.values(nodes).map((node) => {
          const attachedSensors = node.sensors
            .map(sensorId => sensors[sensorId])
            .filter(Boolean);

          return (
            <div
              key={node.id}
              className="bg-cyber-darker border border-panel-border rounded-lg p-4"
            >
              <div className="flex items-center justify-between mb-3">
                <div className="flex items-center gap-3">
                  <span
                    className={`text-2xl ${getNodeColor(node.type, node.status)}`}
                  >
                    {getNodeIcon(node.type)}
                  </span>
                  <div>
                    <h3 className="text-sm font-medium text-white">{node.id}</h3>
                    <p className="text-xs text-gray-400">
                      {node.type.replace('_', ' ')} • {node.status}
                    </p>
                  </div>
                </div>
                <div className="text-right text-xs text-gray-400">
                  <div>CPU: {node.capabilities.processing_power}%</div>
                  <div>MEM: {node.capabilities.memory}GB</div>
                  <div>NET: {node.capabilities.network_bandwidth}Mbps</div>
                </div>
              </div>

              {attachedSensors.length > 0 && (
                <div className="mt-3 pt-3 border-t border-panel-border">
                  <div className="text-xs text-gray-400 mb-2">
                    Connected Sensors ({attachedSensors.length})
                  </div>
                  <div className="flex flex-wrap gap-2">
                    {attachedSensors.map((sensor) => (
                      <div
                        key={sensor.id}
                        className={`px-2 py-1 rounded text-xs ${
                          sensor.status === 'active'
                            ? 'bg-cyber-blue/20 text-cyber-blue'
                            : 'bg-gray-700 text-gray-400'
                        }`}
                      >
                        {sensor.type}
                      </div>
                    ))}
                  </div>
                </div>
              )}

              {/* Connection strength visualization */}
              <div className="mt-3 flex gap-1">
                {[1, 2, 3, 4, 5].map((i) => (
                  <div
                    key={i}
                    className={`h-2 flex-1 rounded ${
                      i <= (node.status === 'online' ? 4 : 1)
                        ? 'bg-cyber-green'
                        : 'bg-gray-700'
                    }`}
                  />
                ))}
              </div>
            </div>
          );
        })}

        {Object.keys(nodes).length === 0 && (
          <div className="text-center text-gray-400 py-8">
            No nodes in the network
          </div>
        )}
      </div>
    </div>
  );
};