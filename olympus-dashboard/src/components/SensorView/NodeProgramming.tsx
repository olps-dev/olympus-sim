import React, { useState, useEffect } from 'react';
import Editor from '@monaco-editor/react';
import * as Select from '@radix-ui/react-select';
import { ChevronDownIcon, PlayIcon, StopIcon, FileIcon, DownloadIcon } from '@radix-ui/react-icons';
import { useSimulationStore } from '../../store/simulationStore';
import { websocketService } from '../../services/websocket';

const DEFAULT_PYTHON_SCRIPT = `# Advanced Node Programming Script with AI/LLM Support
# This script runs on the selected node and has access to:
# - sensor_data: Real-time sensor readings (including voice transcriptions)
# - mqtt_publish(topic, data): Publish to MQTT
# - ros2_publish(topic, msg_type, data): Publish to ROS2  
# - llm_query(prompt, system_msg): Query OpenAI GPT models
# - node_info: Current node capabilities and status
# - OPENAI_AVAILABLE: Boolean indicating if OpenAI is ready
# - INTERNET_AVAILABLE: Boolean indicating internet connectivity

import time
import json

def main():
    """Main execution function"""
    print(f"Script running on node: {node_info['id']}")
    print(f"Node type: {node_info['type']}")
    print(f"Available sensors: {list(sensor_data.keys())}")
    print(f"OpenAI available: {OPENAI_AVAILABLE}")
    print(f"Internet available: {INTERNET_AVAILABLE}")
    
    # Example: React to sensor data
    if 'mmwave_entrance' in sensor_data:
        presence = sensor_data['mmwave_entrance'].get('presence', False)
        if presence:
            # Publish automation command
            mqtt_publish('actuators/lamp_hall/command', {'action': 'on'})
            print("Motion detected - turning on lights")
    
    # Example: Voice command processing with LLM
    for sensor_id, data in sensor_data.items():
        if 'audio' in sensor_id and data.get('voice_command', False):
            transcription = data.get('transcription', '')
            confidence = data.get('confidence', 0)
            
            if transcription and confidence > 0.8:
                print(f"Voice command: {transcription}")
                
                # Use LLM to process command (if available)
                if OPENAI_AVAILABLE:
                    response = llm_query(
                        transcription, 
                        "You are a smart home assistant. Respond helpfully and concisely."
                    )
                    print(f"AI Response: {response}")
                    mqtt_publish('ai/response', {'query': transcription, 'response': response})
    
    # Example: Periodic status update with AI analysis
    status = {
        'timestamp': time.time(),
        'node_id': node_info['id'],
        'sensor_count': len(sensor_data),
        'capabilities': node_info['capabilities']
    }
    mqtt_publish(f"nodes/{node_info['id']}/status", status)

if __name__ == "__main__":
    main()
`;

const SCRIPT_TEMPLATES = {
  'automation': {
    name: 'Home Automation',
    code: `# Home Automation Script
def main():
    # Check motion sensors
    for sensor_id, data in sensor_data.items():
        if 'mmwave' in sensor_id or 'pir' in sensor_id:
            if data.get('presence', False):
                mqtt_publish('actuators/lights/on', {'zone': data.get('zone', 'default')})
    
    # Check temperature sensors
    for sensor_id, data in sensor_data.items():
        if 'temperature' in sensor_id:
            temp = data.get('temperature', 20)
            if temp > 25:
                mqtt_publish('actuators/hvac/cool', {'target': 22})
            elif temp < 18:
                mqtt_publish('actuators/hvac/heat', {'target': 20})
`
  },
  'ai_assistant': {
    name: 'AI Assistant (LLM)',
    code: `# AI Assistant with OpenAI Integration
def main():
    print(f"AI Assistant running on {node_info['id']}")
    
    # Check for voice commands
    for sensor_id, data in sensor_data.items():
        if 'audio' in sensor_id and data.get('voice_command', False):
            transcription = data.get('transcription', '')
            confidence = data.get('confidence', 0)
            
            if confidence > 0.8 and transcription:
                print(f"Voice command detected: {transcription}")
                
                # Use LLM to process the command with full context
                system_msg = f"""You are a smart home assistant controlling IoT devices. 
                Current node: {node_info['type']} with capabilities: {node_info['capabilities']}
                Available sensors: {list(sensor_data.keys())}
                
                Respond with specific MQTT commands in JSON format for:
                - lights: actuators/lights/command
                - hvac: actuators/hvac/command  
                - security: actuators/security/command
                
                Keep responses concise and actionable."""
                
                response = llm_query(transcription, system_msg)
                print(f"AI Response: {response}")
                
                # Publish AI response as both text and commands
                mqtt_publish('ai/response', {
                    'query': transcription,
                    'response': response,
                    'node_id': node_info['id'],
                    'confidence': confidence
                })
                
                # Simple command parsing (in real app, use LLM to generate structured commands)
                if 'light' in transcription.lower():
                    action = 'on' if 'on' in transcription.lower() else 'off'
                    mqtt_publish('actuators/lights/command', {'action': action})
                    
    # Periodic status check with AI analysis
    if len(sensor_data) > 0:
        status_prompt = "Analyze the current sensor readings and suggest any optimizations"
        analysis = llm_query(status_prompt)
        print(f"AI Analysis: {analysis}")
`
  },
  'voice_activation': {
    name: 'Voice Activation',
    code: `# Voice Activation and Command Processing
def main():
    print(f"Voice activation system on {node_info['id']}")
    
    for sensor_id, data in sensor_data.items():
        if 'audio' in sensor_id:
            volume = data.get('volume', 0)
            speech_detected = data.get('speechDetected', False)
            wake_word = data.get('wake_word_detected', False)
            transcription = data.get('transcription', '')
            confidence = data.get('confidence', 0)
            
            print(f"Audio: vol={volume}dB, speech={speech_detected}, wake={wake_word}")
            
            if wake_word and speech_detected and confidence > 0.7:
                print(f"Processing voice command: {transcription}")
                
                # Voice command routing
                command_lower = transcription.lower()
                
                if 'temperature' in command_lower or 'temp' in command_lower:
                    # Get temperature from sensors
                    temps = []
                    for sid, sdata in sensor_data.items():
                        if 'temperature' in sid:
                            temps.append(sdata.get('temperature', 0))
                    
                    if temps:
                        avg_temp = sum(temps) / len(temps)
                        response = f"Current temperature is {avg_temp:.1f} degrees Celsius"
                    else:
                        response = "No temperature sensors available"
                    
                    mqtt_publish('voice/response', {
                        'query': transcription,
                        'response': response,
                        'type': 'temperature_query'
                    })
                
                elif 'light' in command_lower:
                    if 'on' in command_lower or 'turn on' in command_lower:
                        mqtt_publish('actuators/lights/command', {'action': 'on', 'source': 'voice'})
                        response = "Turning on the lights"
                    elif 'off' in command_lower:
                        mqtt_publish('actuators/lights/command', {'action': 'off', 'source': 'voice'})
                        response = "Turning off the lights"
                    else:
                        response = "Light command not understood"
                    
                    mqtt_publish('voice/response', {'response': response})
                
                elif 'time' in command_lower:
                    import datetime
                    current_time = datetime.datetime.now().strftime("%I:%M %p")
                    response = f"The current time is {current_time}"
                    mqtt_publish('voice/response', {'response': response})
                
                else:
                    # Fallback to LLM if available
                    if OPENAI_AVAILABLE:
                        response = llm_query(transcription, "Respond as a helpful smart home assistant.")
                        mqtt_publish('voice/response', {'response': response, 'source': 'ai'})
                    else:
                        mqtt_publish('voice/response', {'response': 'Command not recognized'})
`
  },
  'security': {
    name: 'Security Monitoring',
    code: `# Security Monitoring Script
import time

def main():
    # Check all motion sensors
    motion_detected = False
    for sensor_id, data in sensor_data.items():
        if ('mmwave' in sensor_id or 'pir' in sensor_id or 'camera' in sensor_id):
            if data.get('presence', False) or data.get('motion', False):
                motion_detected = True
                alert = {
                    'timestamp': time.time(),
                    'sensor': sensor_id,
                    'type': 'motion_detection',
                    'location': data.get('location', 'unknown')
                }
                mqtt_publish('security/alerts', alert)
    
    # Audio monitoring
    for sensor_id, data in sensor_data.items():
        if 'audio' in sensor_id:
            noise_level = data.get('noise_level', 0)
            if noise_level > 80:  # High noise threshold
                alert = {
                    'timestamp': time.time(),
                    'sensor': sensor_id,
                    'type': 'noise_alert',
                    'level': noise_level
                }
                mqtt_publish('security/alerts', alert)
`
  },
  'analytics': {
    name: 'Data Analytics',
    code: `# Data Analytics Script
import time
import statistics

# Store historical data (in real implementation, use database)
historical_data = {}

def main():
    current_time = time.time()
    
    # Collect and analyze sensor data
    for sensor_id, data in sensor_data.items():
        if sensor_id not in historical_data:
            historical_data[sensor_id] = []
        
        # Store reading with timestamp
        reading = {
            'timestamp': current_time,
            'data': data
        }
        historical_data[sensor_id].append(reading)
        
        # Keep only last 100 readings
        if len(historical_data[sensor_id]) > 100:
            historical_data[sensor_id] = historical_data[sensor_id][-100:]
    
    # Generate analytics
    analytics = {}
    for sensor_id, readings in historical_data.items():
        if len(readings) > 5:
            # Calculate trends
            if 'temperature' in readings[-1]['data']:
                temps = [r['data']['temperature'] for r in readings[-10:] if 'temperature' in r['data']]
                if temps:
                    analytics[sensor_id] = {
                        'avg_temp': statistics.mean(temps),
                        'temp_trend': 'rising' if temps[-1] > temps[0] else 'falling',
                        'reading_count': len(readings)
                    }
    
    # Publish analytics
    if analytics:
        mqtt_publish('analytics/summary', {
            'timestamp': current_time,
            'node_id': node_info['id'],
            'analytics': analytics
        })
`
  }
};

export const NodeProgramming: React.FC = () => {
  const { selectedEntityId, selectedEntityType, nodes, updateNode } = useSimulationStore();
  const [script, setScript] = useState(DEFAULT_PYTHON_SCRIPT);
  const [scriptStatus, setScriptStatus] = useState<'stopped' | 'running' | 'error'>('stopped');
  const [logs, setLogs] = useState<string[]>([]);
  const [selectedTemplate, setSelectedTemplate] = useState<string>('');

  const selectedNode = selectedEntityId && selectedEntityType === 'node' ? nodes[selectedEntityId] : null;

  const addLog = React.useCallback((message: string) => {
    const timestamp = new Date().toLocaleTimeString();
    setLogs(prev => [...prev, `[${timestamp}] ${message}`].slice(-50)); // Keep last 50 logs
  }, []);

  useEffect(() => {
    // Load script for selected node
    if (selectedNode && selectedNode.script) {
      setScript(selectedNode.script);
      setScriptStatus(selectedNode.scriptStatus || 'stopped');
    } else {
      setScript(DEFAULT_PYTHON_SCRIPT);
      setScriptStatus('stopped');
    }
  }, [selectedNode]);

  useEffect(() => {
    // Set up websocket listeners for script execution results
    const handleScriptResult = (result: any) => {
      if (result.nodeId === selectedEntityId) {
        if (result.success) {
          addLog('Script executed successfully');
          if (result.output) {
            addLog(result.output);
          }
          setScriptStatus('running');
        } else {
          addLog(`Script execution failed: ${result.error}`);
          setScriptStatus('error');
        }
      }
    };

    websocketService.on('script_execution_result', handleScriptResult);

    return () => {
      // Cleanup listener when component unmounts
      // Note: socket.io doesn't have an easy way to remove specific listeners
      // In a real app, you'd want to implement proper cleanup
    };
  }, [selectedEntityId, addLog]);

  const handleSaveScript = () => {
    if (selectedNode) {
      // Save locally first
      updateNode(selectedNode.id, { 
        script, 
        scriptStatus,
        lastModified: new Date().toISOString() 
      });
      
      // Save to backend
      websocketService.saveNodeScript(selectedNode.id, script);
      addLog('Script saved successfully');
    }
  };

  const handleRunScript = async () => {
    if (!selectedNode) return;
    
    setScriptStatus('running');
    addLog(`Starting script execution on ${selectedNode.id}...`);
    
    try {
      // Save script first, then execute
      websocketService.saveNodeScript(selectedNode.id, script);
      websocketService.executeNodeScript(selectedNode.id);
      
      updateNode(selectedNode.id, { scriptStatus: 'running' });
    } catch (error) {
      setScriptStatus('error');
      addLog(`Error: ${error}`);
      updateNode(selectedNode.id, { scriptStatus: 'error' });
    }
  };

  const handleStopScript = () => {
    if (!selectedNode) return;
    
    setScriptStatus('stopped');
    addLog(`Stopped script execution on ${selectedNode.id}`);
    
    websocketService.stopNodeScript(selectedNode.id);
    updateNode(selectedNode.id, { scriptStatus: 'stopped' });
  };

  const handleTemplateSelect = (templateId: string) => {
    if (templateId === 'blank') {
      setScript(DEFAULT_PYTHON_SCRIPT);
      setSelectedTemplate(templateId);
      addLog('Loaded blank script template');
    } else if (templateId && SCRIPT_TEMPLATES[templateId as keyof typeof SCRIPT_TEMPLATES]) {
      setScript(SCRIPT_TEMPLATES[templateId as keyof typeof SCRIPT_TEMPLATES].code);
      setSelectedTemplate(templateId);
      addLog(`Loaded template: ${SCRIPT_TEMPLATES[templateId as keyof typeof SCRIPT_TEMPLATES].name}`);
    }
  };

  if (!selectedNode) {
    return (
      <div className="w-full h-full p-4 flex items-center justify-center text-gray-400">
        <div className="text-center">
          <FileIcon className="w-12 h-12 mx-auto mb-4 opacity-50" />
          <p className="text-lg font-medium">No Node Selected</p>
          <p className="text-sm mt-2">Select a node in the 3D viewport to program it</p>
        </div>
      </div>
    );
  }

  return (
    <div className="w-full h-full p-4 flex flex-col overflow-hidden">
      {/* Header with controls */}
      <div className="mb-4 space-y-3">
        <div className="flex items-center justify-between">
          <div>
            <h3 className="text-lg font-semibold text-white">Node Programming</h3>
            <p className="text-sm text-gray-400">
              {selectedNode.id} ({selectedNode.type})
            </p>
          </div>
          <div className="flex items-center gap-2">
            <button
              onClick={handleSaveScript}
              className="flex items-center gap-2 px-3 py-1 bg-cyber-blue/20 hover:bg-cyber-blue/30 border border-cyber-blue rounded text-sm transition-colors"
            >
              <DownloadIcon className="w-4 h-4" />
              Save
            </button>
            {scriptStatus === 'running' ? (
              <button
                onClick={handleStopScript}
                className="flex items-center gap-2 px-3 py-1 bg-red-500/20 hover:bg-red-500/30 border border-red-500 rounded text-sm transition-colors"
              >
                <StopIcon className="w-4 h-4" />
                Stop
              </button>
            ) : (
              <button
                onClick={handleRunScript}
                className="flex items-center gap-2 px-3 py-1 bg-green-500/20 hover:bg-green-500/30 border border-green-500 rounded text-sm transition-colors"
              >
                <PlayIcon className="w-4 h-4" />
                Run
              </button>
            )}
          </div>
        </div>

        {/* Template selector */}
        <div className="flex items-center gap-2">
          <span className="text-sm text-gray-400">Template:</span>
          <Select.Root value={selectedTemplate} onValueChange={handleTemplateSelect}>
            <Select.Trigger className="flex items-center gap-2 px-3 py-1 bg-cyber-darker border border-panel-border rounded text-sm text-white">
              <Select.Value placeholder="Choose template..." />
              <ChevronDownIcon />
            </Select.Trigger>
            <Select.Portal>
              <Select.Content className="bg-cyber-darker border border-panel-border rounded shadow-xl">
                <Select.Viewport>
                  <Select.Item value="blank" className="px-3 py-2 text-sm text-white hover:bg-cyber-blue/20 cursor-pointer">
                    <Select.ItemText>Blank Script</Select.ItemText>
                  </Select.Item>
                  {Object.entries(SCRIPT_TEMPLATES).map(([id, template]) => (
                    <Select.Item key={id} value={id} className="px-3 py-2 text-sm text-white hover:bg-cyber-blue/20 cursor-pointer">
                      <Select.ItemText>{template.name}</Select.ItemText>
                    </Select.Item>
                  ))}
                </Select.Viewport>
              </Select.Content>
            </Select.Portal>
          </Select.Root>
        </div>

        {/* Status indicator */}
        <div className="flex items-center gap-2">
          <span className="text-sm text-gray-400">Status:</span>
          <span className={`text-sm px-2 py-1 rounded ${
            scriptStatus === 'running' ? 'bg-green-500/20 text-green-400' :
            scriptStatus === 'error' ? 'bg-red-500/20 text-red-400' :
            'bg-gray-500/20 text-gray-400'
          }`}>
            {scriptStatus.charAt(0).toUpperCase() + scriptStatus.slice(1)}
          </span>
        </div>
      </div>

      {/* Code editor */}
      <div className="flex-1 flex flex-col min-h-0">
        <div className="flex-1 border border-panel-border rounded overflow-hidden">
          <Editor
            height="100%"
            defaultLanguage="python"
            value={script}
            onChange={(value) => setScript(value || '')}
            theme="vs-dark"
            options={{
              minimap: { enabled: false },
              scrollBeyondLastLine: false,
              fontSize: 14,
              wordWrap: 'on',
              automaticLayout: true,
            }}
          />
        </div>

        {/* Logs panel */}
        <div className="mt-3 h-32 border border-panel-border rounded bg-cyber-darker">
          <div className="p-2 border-b border-panel-border">
            <span className="text-sm font-medium text-gray-400">Execution Logs</span>
          </div>
          <div className="p-2 h-24 overflow-y-auto text-xs font-mono">
            {logs.length === 0 ? (
              <div className="text-gray-500">No logs yet...</div>
            ) : (
              logs.map((log, index) => (
                <div key={index} className="text-gray-300 whitespace-pre-wrap">
                  {log}
                </div>
              ))
            )}
          </div>
        </div>
      </div>
    </div>
  );
};