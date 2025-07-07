import React, { useState } from 'react';
import * as Tabs from '@radix-ui/react-tabs';
import { NodeProgramming } from './NodeProgramming';
import { NetworkView } from './NetworkView';
import { ActivityFeed } from './ActivityFeed';
import { SceneOutliner } from './SceneOutliner';

export const SensorDataView: React.FC = () => {
  const [activeTab, setActiveTab] = useState('outliner');

  return (
    <div className="flex flex-col h-full bg-panel-bg border border-panel-border rounded-lg m-2">
      <Tabs.Root value={activeTab} onValueChange={setActiveTab} className="flex flex-col h-full">
        <Tabs.List className="flex border-b border-panel-border">
          <Tabs.Trigger
            value="outliner"
            className="flex-1 px-4 py-2 text-sm font-medium text-gray-400 hover:text-white transition-colors data-[state=active]:text-cyber-blue data-[state=active]:border-b-2 data-[state=active]:border-cyber-blue"
          >
            Outliner
          </Tabs.Trigger>
          <Tabs.Trigger
            value="programming"
            className="flex-1 px-4 py-2 text-sm font-medium text-gray-400 hover:text-white transition-colors data-[state=active]:text-cyber-blue data-[state=active]:border-b-2 data-[state=active]:border-cyber-blue"
          >
            Programming
          </Tabs.Trigger>
          <Tabs.Trigger
            value="network"
            className="flex-1 px-4 py-2 text-sm font-medium text-gray-400 hover:text-white transition-colors data-[state=active]:text-cyber-blue data-[state=active]:border-b-2 data-[state=active]:border-cyber-blue"
          >
            Network
          </Tabs.Trigger>
          <Tabs.Trigger
            value="activity"
            className="flex-1 px-4 py-2 text-sm font-medium text-gray-400 hover:text-white transition-colors data-[state=active]:text-cyber-blue data-[state=active]:border-b-2 data-[state=active]:border-cyber-blue"
          >
            Activity
          </Tabs.Trigger>
        </Tabs.List>

        <div className="flex-1 overflow-hidden">
          <Tabs.Content value="outliner" className="h-full">
            <SceneOutliner />
          </Tabs.Content>
          <Tabs.Content value="programming" className="h-full">
            <NodeProgramming />
          </Tabs.Content>
          <Tabs.Content value="network" className="h-full">
            <NetworkView />
          </Tabs.Content>
          <Tabs.Content value="activity" className="h-full">
            <ActivityFeed />
          </Tabs.Content>
        </div>
      </Tabs.Root>
    </div>
  );
};