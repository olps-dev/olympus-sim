# Olympus Development MCP Server

This MCP (Model Context Protocol) server provides specialized tools for developing and optimizing the Olympus IoT simulation system. It integrates with Claude Code to accelerate development workflows.

## Features

### 🚀 Core Tools

1. **olympus_generate_sensor** - Generate complete sensor configurations
   - Supports 8 sensor types: mmWave, temperature, audio, camera, PIR, pressure, light, humidity
   - Creates Three.js visualization code, ROS2 config, and MQTT settings
   - Generates realistic sensor specifications and housing models

2. **olympus_create_node** - Create IoT node architectures
   - Zeus nodes: Central AI servers with full processing capabilities
   - Mid-tier nodes: Room-level processing with local AI and failover
   - Low-power nodes: ESP32-based edge sensors with power management

3. **olympus_analyze_layout** - Optimize sensor placement and system design
   - Coverage analysis: Dead zones, redundancy, optimal placement
   - Latency analysis: Network topology, bottlenecks, optimization
   - Power analysis: Consumption, battery life, efficiency recommendations
   - Redundancy analysis: Single points of failure, backup strategies

4. **olympus_create_scenario** - Generate comprehensive test scenarios
   - Daily routine simulations with realistic user behavior patterns
   - Emergency response testing (fire, medical, security)
   - Automation validation with systematic rule testing
   - Stress testing for performance limits and recovery
   - Security validation with attack simulations

## Installation & Setup

### 1. Build the Server
```bash
cd /home/aliza/olympus-sim/tools/mcp-server
npm install
npm run build
```

### 2. Configure Claude Code
Add this MCP server to your Claude Code configuration:

```json
{
  "mcpServers": {
    "olympus-dev-tools": {
      "command": "node",
      "args": ["/home/aliza/olympus-sim/tools/mcp-server/dist/server.js"],
      "env": {
        "NODE_ENV": "production"
      }
    }
  }
}
```

### 3. Test the Server
```bash
# Test directly (development mode)
npm run dev

# Test with Claude Code
# The server will automatically connect when Claude Code starts
```

## Usage Examples

### Generate a mmWave Sensor
```javascript
// Use the olympus_generate_sensor tool
{
  "sensorType": "mmwave",
  "room": "living_room",
  "position": { "x": 2.5, "y": 2.0, "z": 1.0 },
  "specifications": {
    "range": 8,
    "fov": 60,
    "updateRate": 10
  }
}
```

### Create a Zeus Server Node
```javascript
// Use the olympus_create_node tool
{
  "nodeType": "zeus",
  "location": "server_room",
  "sensors": ["temperature", "humidity"],
  "capabilities": {
    "gpu": "NVIDIA RTX 4080",
    "ram": "64GB"
  }
}
```

### Analyze Room Coverage
```javascript
// Use the olympus_analyze_layout tool
{
  "analysisType": "coverage",
  "roomDimensions": { "width": 5, "height": 3, "depth": 4 },
  "currentSensors": [
    {
      "type": "mmwave",
      "position": { "x": 2.5, "y": 2.0, "z": 2.0 }
    }
  ]
}
```

### Generate Daily Routine Test
```javascript
// Use the olympus_create_scenario tool
{
  "scenarioType": "daily_routine",
  "duration": 120,
  "participants": ["adult", "child"],
  "objectives": ["Test automation flow", "Validate energy savings"]
}
```

## Output Examples

### Sensor Generation Output
- **Three.js Code**: Complete sensor visualization with FOV cones, housing, labels
- **ROS2 Config**: Topic configuration, message types, QoS settings
- **MQTT Config**: Topic structure, data formats, message schemas

### Node Generation Output
- **Node Classes**: Complete JavaScript classes for Zeus/Mid-tier/Low-power nodes
- **Deployment Config**: Docker configuration, resource requirements, networking
- **Network Config**: Communication protocols, security, routing topology

### Layout Analysis Output
- **Analysis Reports**: Detailed coverage, latency, power, redundancy analysis
- **Recommendations**: Prioritized improvement suggestions with impact metrics
- **Visualization Code**: Three.js code for rendering analysis results

### Scenario Generation Output
- **Scenario Definitions**: Detailed test scenarios with timing and triggers
- **Automation Code**: Complete test runner classes with validation
- **Test Plans**: Comprehensive execution plans with success criteria

## Architecture Integration

This MCP server integrates seamlessly with the Olympus simulation architecture:

```
Claude Code + MCP Server
       ↓
Three.js Simulation ← → ROS2 Bridge ← → MQTT Bridge
       ↓                    ↓              ↓
   Browser UI          Sensor Data    IoT Network
```

### Benefits for Development

1. **Rapid Prototyping**: Generate complete sensor/node configurations instantly
2. **System Optimization**: AI-powered analysis of layouts and performance
3. **Test Automation**: Comprehensive scenario generation for validation
4. **Best Practices**: Built-in IoT architecture patterns and security measures
5. **Documentation**: Auto-generated implementation guides and test plans

## Development Workflow

1. **Design Phase**: Use layout analysis to optimize sensor placement
2. **Implementation Phase**: Generate sensors and nodes with proper configurations
3. **Testing Phase**: Create comprehensive test scenarios for validation
4. **Optimization Phase**: Analyze performance and apply recommendations

## Technical Specifications

- **Language**: TypeScript/JavaScript
- **Protocol**: Model Context Protocol (MCP)
- **Integration**: Claude Code CLI
- **Output Formats**: Three.js, ROS2, MQTT, JavaScript
- **Analysis Types**: Coverage, Latency, Power, Redundancy, Security

## Support & Extension

The server is designed for extensibility. New tools can be added by:

1. Creating new tool classes in `src/tools/`
2. Registering tools in `src/server.ts`
3. Adding input schemas for validation
4. Implementing the tool interface

For Olympus-specific development, this MCP server provides the fastest path from concept to implementation with AI-powered optimization and comprehensive testing capabilities.