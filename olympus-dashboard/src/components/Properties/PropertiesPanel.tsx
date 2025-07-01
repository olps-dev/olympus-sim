import React, { useState } from 'react';
import * as Tabs from '@radix-ui/react-tabs';
import { Inspector } from './Inspector';
import { Timeline } from './Timeline';
import { AutomationRules } from './AutomationRules';

export const PropertiesPanel: React.FC = () => {
  const [activeTab, setActiveTab] = useState('inspector');

  return (
    <div className="flex flex-col h-full bg-panel-bg border border-panel-border rounded-lg m-2">
      <Tabs.Root value={activeTab} onValueChange={setActiveTab} className="flex flex-col h-full">
        <Tabs.List className="flex border-b border-panel-border">
          <Tabs.Trigger
            value="inspector"
            className="flex-1 px-4 py-2 text-sm font-medium text-gray-400 hover:text-white transition-colors data-[state=active]:text-cyber-blue data-[state=active]:border-b-2 data-[state=active]:border-cyber-blue"
          >
            Inspector
          </Tabs.Trigger>
          <Tabs.Trigger
            value="timeline"
            className="flex-1 px-4 py-2 text-sm font-medium text-gray-400 hover:text-white transition-colors data-[state=active]:text-cyber-blue data-[state=active]:border-b-2 data-[state=active]:border-cyber-blue"
          >
            Timeline
          </Tabs.Trigger>
          <Tabs.Trigger
            value="rules"
            className="flex-1 px-4 py-2 text-sm font-medium text-gray-400 hover:text-white transition-colors data-[state=active]:text-cyber-blue data-[state=active]:border-b-2 data-[state=active]:border-cyber-blue"
          >
            Rules
          </Tabs.Trigger>
        </Tabs.List>

        <div className="flex-1 overflow-hidden">
          <Tabs.Content value="inspector" className="h-full">
            <Inspector />
          </Tabs.Content>
          <Tabs.Content value="timeline" className="h-full">
            <Timeline />
          </Tabs.Content>
          <Tabs.Content value="rules" className="h-full">
            <AutomationRules />
          </Tabs.Content>
        </div>
      </Tabs.Root>
    </div>
  );
};