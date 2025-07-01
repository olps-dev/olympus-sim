import React from 'react';
import { useSimulationStore } from '../../store/simulationStore';

export const ToolIndicator: React.FC = () => {
  const { toolMode, selectedEntityId, selectedEntityType } = useSimulationStore();

  const getToolDescription = () => {
    switch (toolMode) {
      case 'select':
        return 'Click objects to select them';
      case 'move':
        return selectedEntityId ? 'Drag the gizmo to move the selected object' : 'Select an object first';
      case 'rotate':
        return selectedEntityId ? 'Drag the gizmo to rotate the selected object' : 'Select an object first';
      case 'scale':
        return selectedEntityId ? 'Drag the gizmo to scale the selected object' : 'Select an object first';
      default:
        return '';
    }
  };

  const getKeyboardHints = () => {
    return 'Q: Select | W: Move | E: Rotate | R: Scale | Del: Delete';
  };

  return (
    <div className="absolute bottom-4 left-1/2 -translate-x-1/2 bg-panel-bg border border-panel-border rounded-lg px-4 py-2 text-sm">
      <div className="flex items-center gap-4">
        <div className="flex items-center gap-2">
          <span className="text-cyber-blue font-medium">
            {toolMode.toUpperCase()}
          </span>
          {selectedEntityId && (
            <span className="text-gray-400">
              • {selectedEntityType}: {selectedEntityId}
            </span>
          )}
        </div>
        <div className="text-gray-400">
          {getToolDescription()}
        </div>
      </div>
      <div className="text-xs text-gray-500 mt-1 text-center">
        {getKeyboardHints()}
      </div>
    </div>
  );
};