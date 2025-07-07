import React from 'react';
import * as Slider from '@radix-ui/react-slider';
import * as Select from '@radix-ui/react-select';
import { ChevronDownIcon } from '@radix-ui/react-icons';
import { useSimulationStore } from '../../store/simulationStore';

export const Inspector: React.FC = () => {
  const { 
    selectedEntityId, 
    selectedEntityType, 
    sensors, 
    nodes, 
    rooms,
    updateSensor,
    updateNode,
    updateRoom,
  } = useSimulationStore();

  if (!selectedEntityId || !selectedEntityType) {
    return (
      <div className="p-4 text-center text-gray-400">
        Select an entity to view properties
      </div>
    );
  }

  const renderSensorProperties = () => {
    const sensor = sensors[selectedEntityId];
    if (!sensor) return null;

    return (
      <div className="space-y-4">
        <div>
          <h3 className="text-lg font-medium text-white mb-2">{sensor.id}</h3>
          <p className="text-sm text-gray-400">{sensor.type} sensor</p>
        </div>

        <div className="space-y-3">
          <div>
            <label className="text-xs text-gray-400">Status</label>
            <Select.Root
              value={sensor.status}
              onValueChange={(value) => updateSensor(sensor.id, { status: value as any })}
            >
              <Select.Trigger className="w-full px-3 py-2 bg-cyber-darker border border-panel-border rounded text-sm text-white flex items-center justify-between">
                <Select.Value />
                <ChevronDownIcon />
              </Select.Trigger>
              <Select.Portal>
                <Select.Content className="bg-cyber-darker border border-panel-border rounded">
                  <Select.Viewport>
                    {['active', 'inactive', 'error'].map((status) => (
                      <Select.Item
                        key={status}
                        value={status}
                        className="px-3 py-2 text-sm text-white hover:bg-cyber-blue/20 cursor-pointer"
                      >
                        <Select.ItemText>{status}</Select.ItemText>
                      </Select.Item>
                    ))}
                  </Select.Viewport>
                </Select.Content>
              </Select.Portal>
            </Select.Root>
          </div>

          <div>
            <label className="text-xs text-gray-400">Position</label>
            <div className="grid grid-cols-3 gap-2">
              {['x', 'y', 'z'].map((axis) => (
                <input
                  key={axis}
                  type="number"
                  value={sensor.position[axis as keyof typeof sensor.position]}
                  onChange={(e) => updateSensor(sensor.id, {
                    position: { ...sensor.position, [axis]: parseFloat(e.target.value) }
                  })}
                  className="px-2 py-1 bg-cyber-darker border border-panel-border rounded text-sm text-white"
                  step="0.1"
                />
              ))}
            </div>
          </div>

          {sensor.fov && (
            <div>
              <label className="text-xs text-gray-400 flex justify-between">
                <span>Field of View</span>
                <span>{sensor.fov}°</span>
              </label>
              <Slider.Root
                value={[sensor.fov]}
                onValueChange={([value]) => updateSensor(sensor.id, { fov: value })}
                className="relative flex items-center select-none touch-none w-full h-5"
              >
                <Slider.Track className="bg-cyber-darker relative grow rounded-full h-1">
                  <Slider.Range className="absolute bg-cyber-blue rounded-full h-full" />
                </Slider.Track>
                <Slider.Thumb className="block w-4 h-4 bg-white rounded-full hover:bg-gray-200 focus:outline-none focus:ring-2 focus:ring-cyber-blue" />
              </Slider.Root>
            </div>
          )}

          {sensor.range && (
            <div>
              <label className="text-xs text-gray-400 flex justify-between">
                <span>Range</span>
                <span>{sensor.range}m</span>
              </label>
              <Slider.Root
                value={[sensor.range]}
                onValueChange={([value]) => updateSensor(sensor.id, { range: value })}
                max={20}
                className="relative flex items-center select-none touch-none w-full h-5"
              >
                <Slider.Track className="bg-cyber-darker relative grow rounded-full h-1">
                  <Slider.Range className="absolute bg-cyber-blue rounded-full h-full" />
                </Slider.Track>
                <Slider.Thumb className="block w-4 h-4 bg-white rounded-full hover:bg-gray-200 focus:outline-none focus:ring-2 focus:ring-cyber-blue" />
              </Slider.Root>
            </div>
          )}
        </div>
      </div>
    );
  };

  const renderNodeProperties = () => {
    const node = nodes[selectedEntityId];
    if (!node) return null;

    return (
      <div className="space-y-4">
        <div>
          <h3 className="text-lg font-medium text-white mb-2">{node.id}</h3>
          <p className="text-sm text-gray-400">{node.type.replace('_', ' ')} node</p>
        </div>

        <div className="space-y-3">
          <div>
            <label className="text-xs text-gray-400">Status</label>
            <Select.Root
              value={node.status}
              onValueChange={(value) => updateNode(node.id, { status: value as any })}
            >
              <Select.Trigger className="w-full px-3 py-2 bg-cyber-darker border border-panel-border rounded text-sm text-white flex items-center justify-between">
                <Select.Value />
                <ChevronDownIcon />
              </Select.Trigger>
              <Select.Portal>
                <Select.Content className="bg-cyber-darker border border-panel-border rounded">
                  <Select.Viewport>
                    {['online', 'offline', 'error'].map((status) => (
                      <Select.Item
                        key={status}
                        value={status}
                        className="px-3 py-2 text-sm text-white hover:bg-cyber-blue/20 cursor-pointer"
                      >
                        <Select.ItemText>{status}</Select.ItemText>
                      </Select.Item>
                    ))}
                  </Select.Viewport>
                </Select.Content>
              </Select.Portal>
            </Select.Root>
          </div>

          <div>
            <label className="text-xs text-gray-400">Capabilities</label>
            <div className="space-y-2 mt-1">
              <div className="flex justify-between text-sm">
                <span>CPU</span>
                <span>{node.capabilities.processing_power}%</span>
              </div>
              <div className="flex justify-between text-sm">
                <span>Memory</span>
                <span>{node.capabilities.memory}GB</span>
              </div>
              <div className="flex justify-between text-sm">
                <span>Network</span>
                <span>{node.capabilities.network_bandwidth}Mbps</span>
              </div>
            </div>
          </div>

          <div>
            <label className="text-xs text-gray-400">Connected Sensors</label>
            <div className="mt-1 text-sm text-white">
              {node.sensors.length} sensor{node.sensors.length !== 1 ? 's' : ''}
            </div>
          </div>
        </div>
      </div>
    );
  };

  const renderRoomProperties = () => {
    const room = rooms[selectedEntityId];
    if (!room) return null;

    return (
      <div className="space-y-4">
        <div>
          <h3 className="text-lg font-medium text-white mb-2">{room.name}</h3>
          <p className="text-sm text-gray-400">Room</p>
        </div>

        <div className="space-y-3">
          <div>
            <label className="text-xs text-gray-400">Name</label>
            <input
              type="text"
              value={room.name}
              onChange={(e) => updateRoom(room.id, { name: e.target.value })}
              className="w-full px-3 py-2 bg-cyber-darker border border-panel-border rounded text-sm text-white"
            />
          </div>

          <div>
            <label className="text-xs text-gray-400">Dimensions</label>
            <div className="grid grid-cols-3 gap-2">
              {['x', 'y', 'z'].map((axis) => (
                <input
                  key={axis}
                  type="number"
                  value={room.dimensions[axis as keyof typeof room.dimensions]}
                  onChange={(e) => updateRoom(room.id, {
                    dimensions: { ...room.dimensions, [axis]: parseFloat(e.target.value) }
                  })}
                  className="px-2 py-1 bg-cyber-darker border border-panel-border rounded text-sm text-white"
                  step="0.5"
                />
              ))}
            </div>
          </div>

          <div>
            <label className="text-xs text-gray-400">Color</label>
            <input
              type="color"
              value={room.color}
              onChange={(e) => updateRoom(room.id, { color: e.target.value })}
              className="w-full h-8 bg-cyber-darker border border-panel-border rounded cursor-pointer"
            />
          </div>
        </div>
      </div>
    );
  };

  return (
    <div className="h-full overflow-auto p-4">
      {selectedEntityType === 'sensor' && renderSensorProperties()}
      {selectedEntityType === 'node' && renderNodeProperties()}
      {selectedEntityType === 'room' && renderRoomProperties()}
    </div>
  );
};