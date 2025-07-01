import { useEffect } from 'react';
import { useSimulationStore } from '../store/simulationStore';

export const useKeyboardShortcuts = () => {
  const { setToolMode, selectedEntityId, removeSensor, removeNode, removeRoom, selectedEntityType } = useSimulationStore();

  useEffect(() => {
    const handleKeyPress = (event: KeyboardEvent) => {
      // Don't trigger shortcuts if user is typing in an input
      if (event.target instanceof HTMLInputElement || event.target instanceof HTMLTextAreaElement) {
        return;
      }

      switch (event.key.toLowerCase()) {
        case 'q':
          setToolMode('select');
          break;
        case 'w':
          setToolMode('move');
          break;
        case 'e':
          setToolMode('rotate');
          break;
        case 'r':
          setToolMode('scale');
          break;
        case 'delete':
        case 'backspace':
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
          break;
        case 'escape':
          // Deselect current selection
          useSimulationStore.getState().setSelectedEntity(null, null);
          break;
      }
    };

    window.addEventListener('keydown', handleKeyPress);
    
    return () => {
      window.removeEventListener('keydown', handleKeyPress);
    };
  }, [setToolMode, selectedEntityId, selectedEntityType, removeSensor, removeNode, removeRoom]);
};