import React, { useState, useEffect } from 'react';
import { useSimulationStore } from '../../store/simulationStore';

interface ActivityEvent {
  id: string;
  timestamp: Date;
  type: 'detection' | 'status' | 'error' | 'automation';
  entityId: string;
  entityType: 'sensor' | 'node';
  message: string;
  severity: 'info' | 'warning' | 'error';
}

export const ActivityFeed: React.FC = () => {
  const [events, setEvents] = useState<ActivityEvent[]>([]);
  const { sensors, nodes } = useSimulationStore();

  // Simulate activity events
  useEffect(() => {
    const interval = setInterval(() => {
      const activeSensors = Object.values(sensors).filter(s => s.status === 'active');
      if (activeSensors.length === 0) return;

      const randomSensor = activeSensors[Math.floor(Math.random() * activeSensors.length)];
      
      const eventTypes = [
        {
          type: 'detection' as const,
          message: `Motion detected in ${randomSensor.type} sensor`,
          severity: 'info' as const,
        },
        {
          type: 'status' as const,
          message: `${randomSensor.type} sensor reading: ${Math.random() * 100 | 0}%`,
          severity: 'info' as const,
        },
      ];

      const event = eventTypes[Math.floor(Math.random() * eventTypes.length)];

      setEvents(prev => [{
        id: Date.now().toString(),
        timestamp: new Date(),
        entityId: randomSensor.id,
        entityType: 'sensor' as const,
        ...event,
      }, ...prev].slice(0, 100)); // Keep last 100 events
    }, 2000);

    return () => clearInterval(interval);
  }, [sensors]);

  const getSeverityColor = (severity: string) => {
    switch (severity) {
      case 'error':
        return 'text-red-400 border-red-400/30 bg-red-400/10';
      case 'warning':
        return 'text-cyber-yellow border-cyber-yellow/30 bg-cyber-yellow/10';
      default:
        return 'text-cyber-blue border-cyber-blue/30 bg-cyber-blue/10';
    }
  };

  const getIcon = (type: string) => {
    switch (type) {
      case 'detection':
        return '◉';
      case 'status':
        return '◎';
      case 'error':
        return '⚠';
      case 'automation':
        return '⚡';
      default:
        return '•';
    }
  };

  return (
    <div className="h-full overflow-auto p-4">
      <div className="space-y-2">
        {events.map((event) => (
          <div
            key={event.id}
            className={`flex items-start gap-3 p-3 rounded-lg border ${getSeverityColor(
              event.severity
            )} transition-all duration-300 animate-in slide-in-from-top`}
          >
            <span className="text-lg">{getIcon(event.type)}</span>
            <div className="flex-1 min-w-0">
              <p className="text-sm">{event.message}</p>
              <p className="text-xs opacity-60 mt-1">
                {event.entityId} • {event.timestamp.toLocaleTimeString()}
              </p>
            </div>
          </div>
        ))}

        {events.length === 0 && (
          <div className="text-center text-gray-400 py-8">
            No activity events yet
          </div>
        )}
      </div>
    </div>
  );
};