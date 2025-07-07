import React, { useState } from 'react';
import { 
  CursorArrowIcon, 
  MoveIcon, 
  RotateCounterClockwiseIcon,
  PlusIcon,
  TrashIcon,
  SizeIcon
} from '@radix-ui/react-icons';
import * as DropdownMenu from '@radix-ui/react-dropdown-menu';
import { useSimulationStore } from '../../store/simulationStore';

export const Toolbar: React.FC = () => {
  const { 
    toolMode, 
    setToolMode, 
    addSensor, 
    addNode, 
    addRoom,
    selectedEntityId,
    selectedEntityType,
    removeSensor,
    removeNode,
    removeRoom
  } = useSimulationStore();

  const handleAddSensor = (type: string) => {
    const id = `sensor_${Date.now()}`;
    addSensor({
      id,
      type: type as any,
      position: { x: 0, y: 1, z: 0 },
      status: 'active',
      fov: type === 'camera' || type === 'mmwave' ? 60 : undefined,
      range: type === 'mmwave' ? 10 : undefined,
    });
  };

  const handleAddNode = (type: string) => {
    const id = `node_${Date.now()}`;
    addNode({
      id,
      type: type as any,
      position: { x: 0, y: 0.5, z: 0 },
      status: 'online',
      sensors: [],
      capabilities: {
        processing_power: type === 'zeus' ? 100 : type === 'mid_tier' ? 50 : 20,
        memory: type === 'zeus' ? 32 : type === 'mid_tier' ? 16 : 4,
        network_bandwidth: type === 'zeus' ? 1000 : type === 'mid_tier' ? 100 : 10,
      },
    });
  };

  const tools = [
    { id: 'select', icon: CursorArrowIcon, label: 'Select (Q)' },
    { id: 'move', icon: MoveIcon, label: 'Move (W)' },
    { id: 'rotate', icon: RotateCounterClockwiseIcon, label: 'Rotate (E)' },
    { id: 'scale', icon: SizeIcon, label: 'Scale (R)' },
  ];

  return (
    <div className="absolute top-4 left-4 flex items-center gap-2 p-2 bg-panel-bg rounded-lg border border-panel-border">
      {tools.map((tool) => (
        <button
          key={tool.id}
          onClick={() => setToolMode(tool.id as any)}
          className={`p-2 rounded transition-colors ${
            toolMode === tool.id
              ? 'bg-cyber-blue/30 text-cyber-blue'
              : 'hover:bg-cyber-blue/10 text-gray-400'
          }`}
          title={tool.label}
        >
          <tool.icon className="w-4 h-4" />
        </button>
      ))}

      <div className="w-px h-6 bg-panel-border mx-1" />

      <DropdownMenu.Root>
        <DropdownMenu.Trigger asChild>
          <button
            className="p-2 rounded hover:bg-cyber-blue/10 text-gray-400 transition-colors"
            title="Add Entity"
          >
            <PlusIcon className="w-4 h-4" />
          </button>
        </DropdownMenu.Trigger>

        <DropdownMenu.Portal>
          <DropdownMenu.Content
            className="min-w-[220px] bg-panel-bg rounded-lg border border-panel-border p-1 shadow-xl"
            sideOffset={5}
          >
            <DropdownMenu.Label className="px-2 py-1 text-xs text-gray-400">
              Sensors
            </DropdownMenu.Label>
            {['mmwave', 'camera', 'temperature', 'audio', 'pir', 'light', 'pressure', 'humidity'].map((type) => (
              <DropdownMenu.Item
                key={type}
                className="px-2 py-1 text-sm text-gray-300 rounded hover:bg-cyber-blue/20 hover:text-white cursor-pointer outline-none"
                onSelect={() => handleAddSensor(type)}
              >
                Add {type.charAt(0).toUpperCase() + type.slice(1)} Sensor
              </DropdownMenu.Item>
            ))}

            <DropdownMenu.Separator className="h-px bg-panel-border my-1" />

            <DropdownMenu.Label className="px-2 py-1 text-xs text-gray-400">
              Nodes
            </DropdownMenu.Label>
            {['zeus', 'mid_tier', 'low_power'].map((type) => (
              <DropdownMenu.Item
                key={type}
                className="px-2 py-1 text-sm text-gray-300 rounded hover:bg-cyber-blue/20 hover:text-white cursor-pointer outline-none"
                onSelect={() => handleAddNode(type)}
              >
                Add {type.replace('_', ' ').split(' ').map(w => w.charAt(0).toUpperCase() + w.slice(1)).join(' ')} Node
              </DropdownMenu.Item>
            ))}

            <DropdownMenu.Separator className="h-px bg-panel-border my-1" />

            <DropdownMenu.Item
              className="px-2 py-1 text-sm text-gray-300 rounded hover:bg-cyber-blue/20 hover:text-white cursor-pointer outline-none"
              onSelect={() => {
                const id = `room_${Date.now()}`;
                addRoom({
                  id,
                  name: 'New Room',
                  position: { x: 0, y: 0, z: 0 },
                  dimensions: { x: 10, y: 3, z: 10 },
                  color: '#00d4ff',
                });
              }}
            >
              Add Room
            </DropdownMenu.Item>
          </DropdownMenu.Content>
        </DropdownMenu.Portal>
      </DropdownMenu.Root>

      <button
        onClick={() => {
          if (selectedEntityId && selectedEntityType) {
            switch (selectedEntityType) {
              case 'sensor':
                removeSensor(selectedEntityId);
                break;
              case 'node':
                removeNode(selectedEntityId);
                break;
              case 'room':
                removeRoom(selectedEntityId);
                break;
            }
          }
        }}
        disabled={!selectedEntityId}
        className={`p-2 rounded transition-colors ${
          selectedEntityId 
            ? 'hover:bg-red-500/20 text-gray-400 hover:text-red-400' 
            : 'text-gray-600 cursor-not-allowed'
        }`}
        title={selectedEntityId ? "Delete Selected (Delete)" : "No selection"}
      >
        <TrashIcon className="w-4 h-4" />
      </button>
    </div>
  );
};