import React, { useState, useRef, useCallback, useEffect } from 'react';
import { clsx } from 'clsx';

interface ResizablePanelProps {
  children: React.ReactNode;
  minWidth: number;
  defaultWidth: number;
  className?: string;
}

interface DividerProps {
  onResize: (delta: number) => void;
}

const Divider: React.FC<DividerProps> = ({ onResize }) => {
  const [isDragging, setIsDragging] = useState(false);
  const startX = useRef(0);

  const handleMouseDown = (e: React.MouseEvent) => {
    setIsDragging(true);
    startX.current = e.clientX;
    document.body.style.cursor = 'col-resize';
  };

  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      if (!isDragging) return;
      
      const delta = e.clientX - startX.current;
      startX.current = e.clientX;
      onResize(delta);
    };

    const handleMouseUp = () => {
      setIsDragging(false);
      document.body.style.cursor = '';
    };

    if (isDragging) {
      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);
    }

    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isDragging, onResize]);

  return (
    <div
      className={clsx(
        'w-1 cursor-col-resize flex-shrink-0 relative group',
        'hover:bg-cyber-blue/20 transition-colors',
        isDragging && 'bg-cyber-blue/30'
      )}
      onMouseDown={handleMouseDown}
    >
      <div className="absolute inset-y-0 -left-1 -right-1" />
    </div>
  );
};

export const ResizableLayout: React.FC<{ children: React.ReactNode[] }> = ({ children }) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const [panelWidths, setPanelWidths] = useState([50, 25, 25]); // percentages

  const handleResize = useCallback((dividerIndex: number, delta: number) => {
    if (!containerRef.current) return;
    
    const containerWidth = containerRef.current.offsetWidth;
    const deltaPercent = (delta / containerWidth) * 100;
    
    setPanelWidths(prev => {
      const newWidths = [...prev];
      
      if (dividerIndex === 0) {
        // Resizing between panel 0 and 1
        const newPanel0 = Math.max(20, Math.min(70, newWidths[0] + deltaPercent));
        const diff = newPanel0 - newWidths[0];
        newWidths[0] = newPanel0;
        newWidths[1] = newWidths[1] - diff;
      } else if (dividerIndex === 1) {
        // Resizing between panel 1 and 2
        const newPanel1 = Math.max(15, Math.min(50, newWidths[1] + deltaPercent));
        const diff = newPanel1 - newWidths[1];
        newWidths[1] = newPanel1;
        newWidths[2] = newWidths[2] - diff;
      }
      
      return newWidths;
    });
  }, []);

  return (
    <div ref={containerRef} className="flex flex-1 h-full overflow-hidden">
      <div 
        className="flex flex-col overflow-hidden"
        style={{ width: `${panelWidths[0]}%` }}
      >
        {children[0]}
      </div>
      
      <Divider onResize={(delta) => handleResize(0, delta)} />
      
      <div 
        className="flex flex-col overflow-hidden"
        style={{ width: `${panelWidths[1]}%` }}
      >
        {children[1]}
      </div>
      
      <Divider onResize={(delta) => handleResize(1, delta)} />
      
      <div 
        className="flex flex-col overflow-hidden"
        style={{ width: `${panelWidths[2]}%` }}
      >
        {children[2]}
      </div>
    </div>
  );
};