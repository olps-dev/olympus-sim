import React from 'react';
import { EyeOpenIcon, EyeNoneIcon } from '@radix-ui/react-icons';
import { useSimulationStore } from '../../store/simulationStore';

export const SceneOutliner: React.FC = () => {
  const { 
    sensors, 
    nodes, 
    rooms, 
    selectedEntityId, 
    selectedEntityType,
    setSelectedEntity,
    updateSensor,
    updateNode,
    removeSensor,
    removeNode,
    removeRoom
  } = useSimulationStore();

  const getEntityIcon = (type: string) => {
    switch (type) {
      case 'mmwave': return '📡';
      case 'camera': return '📷';
      case 'temperature': return '🌡️';
      case 'audio': return '🎤';
      case 'pir': return '👁️';
      case 'light': return '💡';
      case 'pressure': return '⚖️';
      case 'humidity': return '💧';
      case 'zeus': return '🏛️';
      case 'mid_tier': return '🏢';
      case 'low_power': return '🔋';
      case 'room': return '🏠';
      default: return '📦';
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'active':
      case 'online':
        return 'text-cyber-green';
      case 'inactive':
      case 'offline':
        return 'text-gray-500';
      case 'error':
        return 'text-red-400';
      default:
        return 'text-gray-400';
    }
  };

  const EntityItem: React.FC<{
    id: string;
    type: string;
    name: string;
    status: string;
    entityType: 'sensor' | 'node' | 'room';
    children?: string[];
  }> = ({ id, type, name, status, entityType, children = [] }) => {
    const isSelected = selectedEntityId === id && selectedEntityType === entityType;

    return (
      <div className="mb-1">
        <div
          className={`flex items-center gap-2 p-2 rounded cursor-pointer hover:bg-cyber-blue/10 ${
            isSelected ? 'bg-cyber-blue/20 border border-cyber-blue/40' : ''
          }`}
          onClick={() => setSelectedEntity(id, entityType)}
        >
          <span className="text-lg">{getEntityIcon(type)}</span>
          <span className="flex-1 text-sm truncate">{name}</span>
          <span className={`text-xs ${getStatusColor(status)}`}>●</span>
          <button
            onClick={(e) => {
              e.stopPropagation();
              // Toggle visibility (placeholder - would need to implement in store)
            }}
            className="p-1 hover:bg-gray-600 rounded"
          >
            <EyeOpenIcon className="w-3 h-3 text-gray-400" />
          </button>
        </div>
        
        {/* Show connected sensors for nodes */}
        {children.length > 0 && (
          <div className="ml-6 pl-2 border-l border-gray-600">
            {children.map((childId) => {
              const sensor = sensors[childId];
              if (!sensor) return null;
              return (
                <EntityItem
                  key={childId}
                  id={sensor.id}
                  type={sensor.type}
                  name={sensor.id}
                  status={sensor.status}
                  entityType="sensor"
                />
              );
            })}
          </div>
        )}
      </div>
    );
  };

  return (
    <div className="h-full overflow-auto p-4">
      <div className="space-y-4">
        {/* Rooms Section */}
        <div>
          <h3 className="text-xs font-medium text-gray-400 mb-2 uppercase tracking-wide">
            Rooms ({Object.keys(rooms).length})
          </h3>
          {Object.values(rooms).map((room) => (
            <EntityItem
              key={room.id}
              id={room.id}
              type="room"
              name={room.name}
              status="active"
              entityType="room"
            />
          ))}
        </div>

        {/* Nodes Section */}
        <div>
          <h3 className="text-xs font-medium text-gray-400 mb-2 uppercase tracking-wide">
            Nodes ({Object.keys(nodes).length})
          </h3>
          {Object.values(nodes).map((node) => (
            <EntityItem
              key={node.id}
              id={node.id}
              type={node.type}
              name={node.id}
              status={node.status}
              entityType="node"
              children={node.sensors}
            />
          ))}
        </div>

        {/* Unconnected Sensors Section */}
        <div>
          <h3 className="text-xs font-medium text-gray-400 mb-2 uppercase tracking-wide">
            Unconnected Sensors
          </h3>
          {Object.values(sensors)
            .filter(sensor => !sensor.nodeId)
            .map((sensor) => (
              <EntityItem
                key={sensor.id}
                id={sensor.id}
                type={sensor.type}
                name={sensor.id}
                status={sensor.status}
                entityType="sensor"
              />
            ))}
        </div>

        {/* Scene Statistics */}
        <div className="border-t border-panel-border pt-4 mt-4">
          <h3 className="text-xs font-medium text-gray-400 mb-2 uppercase tracking-wide">
            Scene Statistics
          </h3>
          <div className="text-xs text-gray-400 space-y-1">
            <div>Total Entities: {Object.keys(sensors).length + Object.keys(nodes).length + Object.keys(rooms).length}</div>
            <div>Active Sensors: {Object.values(sensors).filter(s => s.status === 'active').length}</div>
            <div>Online Nodes: {Object.values(nodes).filter(n => n.status === 'online').length}</div>
          </div>
        </div>
      </div>
    </div>
  );
};