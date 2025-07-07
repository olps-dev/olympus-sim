#!/usr/bin/env node

import { Server } from '@modelcontextprotocol/sdk/server/index.js';
import { StdioServerTransport } from '@modelcontextprotocol/sdk/server/stdio.js';
import {
  CallToolRequestSchema,
  ListToolsRequestSchema,
  Tool,
} from '@modelcontextprotocol/sdk/types.js';

import { SensorGenerator } from './tools/sensorGenerator.js';
import { NodeGenerator } from './tools/nodeGenerator.js';
import { LayoutOptimizer } from './tools/layoutOptimizer.js';
import { ScenarioGenerator } from './tools/scenarioGenerator.js';

class OlympusDevServer {
  private server: Server;
  private sensorGenerator: SensorGenerator;
  private nodeGenerator: NodeGenerator;
  private layoutOptimizer: LayoutOptimizer;
  private scenarioGenerator: ScenarioGenerator;

  constructor() {
    this.server = new Server(
      {
        name: 'olympus-dev-tools',
        version: '1.0.0',
      },
      {
        capabilities: {
          tools: {},
        },
      }
    );

    // Initialize tool generators
    this.sensorGenerator = new SensorGenerator();
    this.nodeGenerator = new NodeGenerator();
    this.layoutOptimizer = new LayoutOptimizer();
    this.scenarioGenerator = new ScenarioGenerator();

    this.setupToolHandlers();
    this.setupErrorHandling();
  }

  private setupToolHandlers(): void {
    this.server.setRequestHandler(ListToolsRequestSchema, async () => {
      return {
        tools: [
          {
            name: 'olympus_generate_sensor',
            description: 'Generate a sensor configuration for the Olympus simulation',
            inputSchema: {
              type: 'object',
              properties: {
                sensorType: {
                  type: 'string',
                  enum: ['mmwave', 'temperature', 'audio', 'camera', 'pir', 'pressure', 'light', 'humidity'],
                  description: 'Type of sensor to generate'
                },
                room: {
                  type: 'string',
                  description: 'Room where sensor will be placed (e.g., kitchen, bedroom, living_room)'
                },
                position: {
                  type: 'object',
                  properties: {
                    x: { type: 'number' },
                    y: { type: 'number' },
                    z: { type: 'number' }
                  },
                  description: '3D position for sensor placement'
                },
                specifications: {
                  type: 'object',
                  description: 'Sensor-specific specifications (range, accuracy, update_rate, etc.)',
                  additionalProperties: true
                }
              },
              required: ['sensorType', 'room']
            }
          },
          {
            name: 'olympus_create_node',
            description: 'Create a node (Zeus/Mid-tier/Low-power) for the Olympus architecture',
            inputSchema: {
              type: 'object',
              properties: {
                nodeType: {
                  type: 'string',
                  enum: ['zeus', 'mid_tier', 'low_power'],
                  description: 'Type of node to create'
                },
                location: {
                  type: 'string',
                  description: 'Location/room for this node'
                },
                sensors: {
                  type: 'array',
                  items: { type: 'string' },
                  description: 'List of sensor types to attach to this node'
                },
                capabilities: {
                  type: 'object',
                  description: 'Node capabilities (processing_power, memory, network, etc.)',
                  additionalProperties: true
                }
              },
              required: ['nodeType', 'location']
            }
          },
          {
            name: 'olympus_analyze_layout',
            description: 'Analyze current sensor layout for optimization opportunities',
            inputSchema: {
              type: 'object',
              properties: {
                analysisType: {
                  type: 'string',
                  enum: ['coverage', 'latency', 'power', 'redundancy'],
                  description: 'Type of analysis to perform'
                },
                roomDimensions: {
                  type: 'object',
                  properties: {
                    width: { type: 'number' },
                    height: { type: 'number' },
                    depth: { type: 'number' }
                  },
                  description: 'Room dimensions for analysis'
                },
                currentSensors: {
                  type: 'array',
                  items: {
                    type: 'object',
                    properties: {
                      type: { type: 'string' },
                      position: {
                        type: 'object',
                        properties: {
                          x: { type: 'number' },
                          y: { type: 'number' },
                          z: { type: 'number' }
                        }
                      }
                    }
                  },
                  description: 'Current sensor configuration'
                }
              },
              required: ['analysisType']
            }
          },
          {
            name: 'olympus_create_scenario',
            description: 'Generate test scenarios for Olympus automation',
            inputSchema: {
              type: 'object',
              properties: {
                scenarioType: {
                  type: 'string',
                  enum: ['daily_routine', 'emergency', 'automation_test', 'stress_test', 'security_test'],
                  description: 'Type of scenario to generate'
                },
                duration: {
                  type: 'number',
                  description: 'Scenario duration in minutes'
                },
                participants: {
                  type: 'array',
                  items: { type: 'string' },
                  description: 'People involved in scenario (e.g., adult, child, elderly)'
                },
                objectives: {
                  type: 'array',
                  items: { type: 'string' },
                  description: 'What the scenario should test or achieve'
                }
              },
              required: ['scenarioType']
            }
          }
        ] as Tool[]
      };
    });

    this.server.setRequestHandler(CallToolRequestSchema, async (request) => {
      try {
        switch (request.params.name) {
          case 'olympus_generate_sensor':
            return await this.sensorGenerator.generate(request.params.arguments);

          case 'olympus_create_node':
            return await this.nodeGenerator.create(request.params.arguments);

          case 'olympus_analyze_layout':
            return await this.layoutOptimizer.analyze(request.params.arguments);

          case 'olympus_create_scenario':
            return await this.scenarioGenerator.create(request.params.arguments);

          default:
            throw new Error(`Unknown tool: ${request.params.name}`);
        }
      } catch (error) {
        return {
          content: [
            {
              type: 'text',
              text: `Error: ${error instanceof Error ? error.message : String(error)}`
            }
          ],
          isError: true
        };
      }
    });
  }

  private setupErrorHandling(): void {
    this.server.onerror = (error) => {
      console.error('[MCP Error]', error);
    };

    process.on('SIGINT', async () => {
      await this.server.close();
      process.exit(0);
    });
  }

  async run(): Promise<void> {
    const transport = new StdioServerTransport();
    await this.server.connect(transport);
    console.error('Olympus Development MCP Server running on stdio');
  }
}

// Start the server
const server = new OlympusDevServer();
server.run().catch(console.error);