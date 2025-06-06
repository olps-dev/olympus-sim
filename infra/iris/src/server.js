#!/usr/bin/env node
/**
 * Project Olympus - IRIS Web Dashboard Server
 * Real-time IoT monitoring dashboard with MQTT and InfluxDB integration
 */

const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const mqtt = require('mqtt');
const { InfluxDB } = require('@influxdata/influxdb-client');
const cors = require('cors');
const helmet = require('helmet');
const path = require('path');

// Configuration
const PORT = process.env.PORT || 8080;
const MQTT_URL = process.env.MQTT_URL || 'mqtt://localhost:1883';
const INFLUX_URL = process.env.INFLUX_URL || 'http://localhost:8086';
const INFLUX_TOKEN = process.env.INFLUX_TOKEN || 'olympus_token';
const INFLUX_ORG = process.env.INFLUX_ORG || 'olympus';
const INFLUX_BUCKET = process.env.INFLUX_BUCKET || 'olympus';

// Initialize Express app
const app = express();
const server = http.createServer(app);
const io = socketIo(server, {
  cors: {
    origin: "*",
    methods: ["GET", "POST"]
  }
});

// Middleware
app.use(helmet({
  contentSecurityPolicy: {
    directives: {
      defaultSrc: ["'self'"],
      styleSrc: ["'self'", "'unsafe-inline'", "https://cdnjs.cloudflare.com"],
      scriptSrc: ["'self'", "'unsafe-inline'", "https://cdnjs.cloudflare.com"],
      connectSrc: ["'self'", "ws:", "wss:"]
    }
  }
}));
app.use(cors());
app.use(express.json());
app.use(express.static(path.join(__dirname, '../public')));

// MQTT Client
const mqttClient = mqtt.connect(MQTT_URL);
let isConnected = false;

// InfluxDB Client
const influxDB = new InfluxDB({ url: INFLUX_URL, token: INFLUX_TOKEN });
const queryApi = influxDB.getQueryApi(INFLUX_ORG);

// Data storage for real-time updates
let sensorData = {};
let networkStats = {};
let systemMetrics = {};

// MQTT Event Handlers
mqttClient.on('connect', () => {
  console.log('✅ Connected to MQTT broker');
  isConnected = true;
  
  // Subscribe to all Olympus topics
  mqttClient.subscribe('olympus/+/+/+');
  mqttClient.subscribe('olympus/+/+');
  mqttClient.subscribe('olympus/+');
  
  io.emit('mqtt_status', { connected: true });
});

mqttClient.on('message', (topic, message) => {
  try {
    const data = JSON.parse(message.toString());
    const topicParts = topic.split('/');
    
    console.log(`📨 MQTT: ${topic} => ${message.toString().substring(0, 100)}...`);
    
    // Process sensor data
    if (topicParts[1] === 'sensors' && topicParts.length >= 4) {
      const nodeId = topicParts[2];
      const sensorType = topicParts[3];
      
      if (!sensorData[nodeId]) {
        sensorData[nodeId] = {};
      }
      
      sensorData[nodeId][sensorType] = {
        ...data,
        timestamp: Date.now(),
        topic: topic
      };
      
      // Emit to connected clients
      io.emit('sensor_update', {
        nodeId: nodeId,
        sensorType: sensorType,
        data: sensorData[nodeId][sensorType]
      });
    }
    
    // Process network statistics
    else if (topicParts[1] === 'network') {
      const networkType = topicParts[2];
      networkStats[networkType] = {
        ...data,
        timestamp: Date.now()
      };
      
      io.emit('network_update', {
        type: networkType,
        data: networkStats[networkType]
      });
    }
    
    // Process detection events
    else if (topicParts[1] === 'detections') {
      const nodeId = topicParts[2];
      
      io.emit('detection_event', {
        nodeId: nodeId,
        data: data,
        timestamp: Date.now()
      });
    }
    
    // Process system metrics
    else if (topicParts[1] === 'metrics') {
      systemMetrics = {
        ...data,
        timestamp: Date.now()
      };
      
      io.emit('metrics_update', systemMetrics);
    }
    
  } catch (error) {
    console.error(`❌ Error processing MQTT message: ${error.message}`);
  }
});

mqttClient.on('error', (error) => {
  console.error('❌ MQTT connection error:', error);
  isConnected = false;
  io.emit('mqtt_status', { connected: false, error: error.message });
});

// Socket.IO Event Handlers
io.on('connection', (socket) => {
  console.log('🔌 Client connected to IRIS dashboard');
  
  // Send current status
  socket.emit('mqtt_status', { connected: isConnected });
  socket.emit('sensor_data', sensorData);
  socket.emit('network_stats', networkStats);
  socket.emit('system_metrics', systemMetrics);
  
  // Handle control commands
  socket.on('control_command', (command) => {
    console.log('🎮 Control command received:', command);
    
    // Publish control command to MQTT
    const topic = `olympus/commands/${command.nodeId}/${command.type}`;
    mqttClient.publish(topic, JSON.stringify(command.payload));
  });
  
  socket.on('disconnect', () => {
    console.log('🔌 Client disconnected from IRIS dashboard');
  });
});

// REST API Endpoints

// Health check
app.get('/api/health', (req, res) => {
  res.json({
    status: 'ok',
    timestamp: Date.now(),
    mqtt_connected: isConnected,
    uptime: process.uptime()
  });
});

// Get current sensor data
app.get('/api/sensors', (req, res) => {
  res.json(sensorData);
});

// Get network statistics
app.get('/api/network', (req, res) => {
  res.json(networkStats);
});

// Get system metrics
app.get('/api/metrics', (req, res) => {
  res.json(systemMetrics);
});

// Query historical data from InfluxDB
app.get('/api/history/:measurement', async (req, res) => {
  try {
    const { measurement } = req.params;
    const { timeRange = '1h' } = req.query;
    
    const query = `
      from(bucket: "${INFLUX_BUCKET}")
        |> range(start: -${timeRange})
        |> filter(fn: (r) => r._measurement == "${measurement}")
        |> aggregateWindow(every: 1m, fn: mean)
    `;
    
    const data = [];
    
    await queryApi.queryRows(query, {
      next(row, tableMeta) {
        const o = tableMeta.toObject(row);
        data.push(o);
      },
      error(error) {
        console.error('❌ InfluxDB query error:', error);
        res.status(500).json({ error: error.message });
      },
      complete() {
        res.json(data);
      }
    });
    
  } catch (error) {
    console.error('❌ InfluxDB query error:', error);
    res.status(500).json({ error: error.message });
  }
});

// Send control command
app.post('/api/control', (req, res) => {
  const { nodeId, type, payload } = req.body;
  
  if (!nodeId || !type || !payload) {
    return res.status(400).json({ error: 'Missing required fields' });
  }
  
  const topic = `olympus/commands/${nodeId}/${type}`;
  mqttClient.publish(topic, JSON.stringify(payload));
  
  res.json({ success: true, topic: topic, payload: payload });
});

// Serve the main dashboard
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, '../public/index.html'));
});

// Start server
server.listen(PORT, () => {
  console.log('🚀 IRIS Dashboard Server Started');
  console.log(`📊 Dashboard: http://localhost:${PORT}`);
  console.log(`🔗 MQTT: ${MQTT_URL}`);
  console.log(`📈 InfluxDB: ${INFLUX_URL}`);
  console.log('═══════════════════════════════════');
});

// Graceful shutdown
process.on('SIGINT', () => {
  console.log('\n🛑 Shutting down IRIS dashboard...');
  mqttClient.end();
  server.close(() => {
    console.log('✅ IRIS dashboard stopped');
    process.exit(0);
  });
});

module.exports = app; 