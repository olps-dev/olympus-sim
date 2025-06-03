/**
 * Project Olympus - IRIS Dashboard Client (Performance Optimized)
 */

class OlympusDashboard {
    constructor() {
        this.socket = null;
        this.charts = {};
        this.sensorData = {};
        this.networkStats = {};
        this.systemMetrics = {};
        this.events = [];
        this.startTime = Date.now();
        
        // Performance optimization settings
        this.updateThrottle = 2000; // 2 second throttle for UI updates
        this.lastUpdate = 0;
        this.maxDataPoints = 20; // Limit chart data points
        this.maxEvents = 20; // Limit stored events
        
        // Chart data storage with size limits
        this.chartData = {
            temperature: { labels: [], datasets: [] },
            battery: { labels: [], datasets: [] },
            latency: { labels: [], datasets: [] },
            detections: { labels: [], datasets: [] }
        };
        
        // Update queues to batch DOM operations
        this.pendingUpdates = {
            sensors: false,
            network: false,
            events: false,
            kpis: false
        };
        
        this.init();
    }
    
    init() {
        console.log('Initializing IRIS Dashboard...');
        this.connectSocket();
        this.initializeCharts();
        this.startTimeUpdates();
        this.bindEvents();
        
        // Throttled update loop
        setInterval(() => this.processUpdates(), this.updateThrottle);
        
        this.showLoading(true);
    }
    
    connectSocket() {
        try {
            this.socket = io();
            
            this.socket.on('connect', () => {
                console.log('Connected to IRIS server');
                this.updateConnectionStatus(true);
                this.showLoading(false);
            });
            
            this.socket.on('disconnect', () => {
                console.log('Disconnected from IRIS server');
                this.updateConnectionStatus(false);
            });
            
            this.socket.on('mqtt_status', (data) => {
                this.updateConnectionStatus(data.connected);
            });
            
            this.socket.on('sensor_update', (data) => {
                this.handleSensorUpdate(data);
            });
            
            this.socket.on('network_update', (data) => {
                this.handleNetworkUpdate(data);
            });
            
            this.socket.on('metrics_update', (data) => {
                this.systemMetrics = data;
                this.pendingUpdates.kpis = true;
            });
            
            this.socket.on('detection_event', (data) => {
                this.handleDetectionEvent(data);
            });
            
        } catch (error) {
            console.error('Failed to connect socket:', error);
            this.updateConnectionStatus(false);
        }
    }
    
    handleSensorUpdate(data) {
        const { nodeId, sensorType, data: sensorInfo } = data;
        
        if (!this.sensorData[nodeId]) {
            this.sensorData[nodeId] = {};
        }
        
        this.sensorData[nodeId][sensorType] = sensorInfo;
        this.updateNodeStatus(nodeId, true);
        
        // Queue UI update instead of immediate update
        this.pendingUpdates.sensors = true;
        
        // Update charts with throttling
        this.updateChartData(nodeId, sensorType, sensorInfo);
    }
    
    handleNetworkUpdate(data) {
        this.networkStats[data.type] = data.data;
        this.pendingUpdates.network = true;
    }
    
    handleDetectionEvent(data) {
        // Limit stored events
        this.events.unshift({...data, id: Date.now()});
        if (this.events.length > this.maxEvents) {
            this.events = this.events.slice(0, this.maxEvents);
        }
        
        this.pendingUpdates.events = true;
        this.flashNode(data.nodeId);
    }
    
    // Process all pending updates in batches
    processUpdates() {
        const now = Date.now();
        if (now - this.lastUpdate < this.updateThrottle) return;
        
        if (this.pendingUpdates.sensors) {
            this.updateSensorDisplay();
            this.pendingUpdates.sensors = false;
        }
        
        if (this.pendingUpdates.network) {
            this.updateNetworkDisplay();
            this.pendingUpdates.network = false;
        }
        
        if (this.pendingUpdates.events) {
            this.updateEventsDisplay();
            this.pendingUpdates.events = false;
        }
        
        if (this.pendingUpdates.kpis) {
            this.updateKPIDisplay();
            this.pendingUpdates.kpis = false;
        }
        
        this.lastUpdate = now;
    }
    
    updateSensorDisplay() {
        const container = document.getElementById('sensorsGrid');
        if (!container) return;
        
        // Only update if container is visible
        if (container.offsetParent === null) return;
        
        // Use DocumentFragment for efficient DOM updates
        const fragment = document.createDocumentFragment();
        
        Object.keys(this.sensorData).forEach(nodeId => {
            const nodeData = this.sensorData[nodeId];
            const card = this.createSensorCard(nodeId, nodeData);
            fragment.appendChild(card);
        });
        
        // Single DOM update
        container.innerHTML = '';
        container.appendChild(fragment);
    }
    
    createSensorCard(nodeId, nodeData) {
        const card = document.createElement('div');
        card.className = 'sensor-card';
        
        const hasRecentData = Object.values(nodeData).some(sensor => 
            Date.now() - sensor.timestamp < 30000
        );
        
        card.innerHTML = `
            <div class="sensor-header">
                <div class="sensor-title">Node ${nodeId}</div>
                <div class="sensor-status ${hasRecentData ? 'online' : 'offline'}">
                    ${hasRecentData ? 'Online' : 'Offline'}
                </div>
            </div>
            <div class="sensor-data">
                ${this.renderSensorData(nodeData)}
            </div>
        `;
        
        return card;
    }
    
    renderSensorData(nodeData) {
        let html = '';
        
        Object.keys(nodeData).forEach(sensorType => {
            const sensor = nodeData[sensorType];
            const age = Math.round((Date.now() - sensor.timestamp) / 1000);
            
            if (sensorType === 'bme680') {
                html += `
                    <div class="sensor-item">
                        <span class="sensor-label">Temperature</span>
                        <span class="sensor-value">${sensor.temperature?.toFixed(1) || '--'}°C</span>
                    </div>
                    <div class="sensor-item">
                        <span class="sensor-label">IAQ</span>
                        <span class="sensor-value">${sensor.iaq?.toFixed(1) || '--'}</span>
                    </div>
                `;
            } else if (sensorType === 'battery') {
                html += `
                    <div class="sensor-item">
                        <span class="sensor-label">Battery</span>
                        <span class="sensor-value">${sensor.voltage?.toFixed(2) || '--'}V</span>
                    </div>
                `;
            }
            
            html += `
                <div class="sensor-item">
                    <span class="sensor-label">Updated</span>
                    <span class="sensor-value">${age}s ago</span>
                </div>
            `;
        });
        
        return html || '<div class="sensor-item"><span class="sensor-label">No data</span></div>';
    }
    
    updateNetworkDisplay() {
        const meshStats = this.networkStats.mesh || {};
        
        this.updateElementText('packetLoss', `${(meshStats.avg_packet_loss * 100 || 0).toFixed(1)}%`);
        this.updateElementText('avgLatency', `${(meshStats.avg_latency_ms || 0).toFixed(1)}ms`);
        this.updateElementText('throughput', `${(meshStats.avg_throughput_mbps || 0).toFixed(1)} Mbps`);
    }
    
    updateKPIDisplay() {
        if (!this.systemMetrics) return;
        
        // Pipeline Latency
        const latency = this.systemMetrics.total_pipeline_latency_ms || 0;
        this.updateKPI('pipelineLatency', `${latency.toFixed(0)}ms`, 
            latency > 300 ? 'error' : latency > 200 ? 'warning' : 'success');
        
        // Battery Life
        const batteryHours = this.systemMetrics.projected_battery_hours || 0;
        const batteryDays = batteryHours / 24;
        this.updateKPI('batteryLife', `${batteryDays.toFixed(1)} days`,
            batteryDays < 9 ? 'error' : batteryDays < 15 ? 'warning' : 'success');
        
        // Mesh PDR
        const pdr = (1 - (this.systemMetrics.mesh_packet_loss || 0)) * 100;
        this.updateKPI('meshPDR', `${pdr.toFixed(1)}%`,
            pdr < 95 ? 'error' : pdr < 98 ? 'warning' : 'success');
        
        // Detection Rate
        const detectionRate = this.systemMetrics.actor_detections_per_minute || 0;
        this.updateElementText('detectionRate', `${detectionRate.toFixed(1)}/min`);
        
        // Simulation time
        const simTime = this.systemMetrics.simulation_time || 0;
        this.updateElementText('simulationTime', `Sim: ${simTime.toFixed(0)}s`);
    }
    
    updateKPI(elementId, value, className) {
        const element = document.getElementById(elementId);
        if (element) {
            element.textContent = value;
            element.className = `kpi-value ${className}`;
        }
    }
    
    updateElementText(elementId, text) {
        const element = document.getElementById(elementId);
        if (element) element.textContent = text;
    }
    
    updateEventsDisplay() {
        const container = document.getElementById('eventsList');
        if (!container) return;
        
        const fragment = document.createDocumentFragment();
        
        this.events.slice(0, 10).forEach(event => {
            const eventElement = document.createElement('div');
            eventElement.className = 'event-item';
            
            const time = new Date(event.timestamp).toLocaleTimeString();
            const description = this.formatEventDescription(event);
            
            eventElement.innerHTML = `
                <div class="event-time">${time}</div>
                <div class="event-description">${description}</div>
                <div class="event-node">Node ${event.nodeId}</div>
            `;
            
            fragment.appendChild(eventElement);
        });
        
        container.innerHTML = '';
        container.appendChild(fragment);
    }
    
    formatEventDescription(event) {
        if (event.data && event.data.detection_count) {
            return `Motion: ${event.data.detection_count} detection${event.data.detection_count > 1 ? 's' : ''}`;
        }
        return 'Motion detected';
    }
    
    updateChartData(nodeId, sensorType, sensorInfo) {
        const now = new Date();
        const timeLabel = now.toLocaleTimeString();
        
        // Throttle chart updates
        if (now.getSeconds() % 5 !== 0) return;
        
        if (sensorType === 'bme680' && sensorInfo.temperature) {
            this.addChartPoint('temperature', timeLabel, {
                label: `Node ${nodeId}`,
                value: sensorInfo.temperature
            });
        }
        
        if (sensorType === 'battery' && sensorInfo.voltage) {
            this.addChartPoint('battery', timeLabel, {
                label: `Node ${nodeId}`,
                value: sensorInfo.voltage
            });
        }
    }
    
    addChartPoint(chartType, timeLabel, dataPoint) {
        const chartData = this.chartData[chartType];
        
        // Limit data points for performance
        if (!chartData.labels.includes(timeLabel)) {
            chartData.labels.push(timeLabel);
            
            if (chartData.labels.length > this.maxDataPoints) {
                chartData.labels = chartData.labels.slice(-this.maxDataPoints);
                // Also trim datasets
                chartData.datasets.forEach(dataset => {
                    if (dataset.data.length > this.maxDataPoints) {
                        dataset.data = dataset.data.slice(-this.maxDataPoints);
                    }
                });
            }
        }
        
        let dataset = chartData.datasets.find(ds => ds.label === dataPoint.label);
        if (!dataset) {
            dataset = {
                label: dataPoint.label,
                data: [],
                borderColor: this.getChartColor(chartData.datasets.length),
                backgroundColor: this.getChartColor(chartData.datasets.length, 0.1),
                fill: false,
                tension: 0.4
            };
            chartData.datasets.push(dataset);
        }
        
        dataset.data.push(dataPoint.value);
        
        // Update chart with animation disabled for performance
        if (this.charts[chartType]) {
            this.charts[chartType].update('none');
        }
    }
    
    getChartColor(index, alpha = 1) {
        const colors = [
            `rgba(55, 63, 255, ${alpha})`,
            `rgba(0, 212, 170, ${alpha})`,
            `rgba(16, 185, 129, ${alpha})`,
            `rgba(245, 158, 11, ${alpha})`,
            `rgba(239, 68, 68, ${alpha})`
        ];
        return colors[index % colors.length];
    }
    
    updateConnectionStatus(connected) {
        const indicator = document.getElementById('statusIndicator');
        const text = document.getElementById('statusText');
        
        if (indicator) {
            indicator.classList.toggle('connected', connected);
        }
        if (text) {
            text.textContent = connected ? 'Connected' : 'Disconnected';
        }
    }
    
    showLoading(show) {
        const overlay = document.getElementById('loadingOverlay');
        if (overlay) {
            overlay.classList.toggle('hidden', !show);
        }
    }
    
    updateNodeStatus(nodeId, online) {
        const statusElement = document.getElementById(`nodeStatus${nodeId}`);
        if (statusElement) {
            statusElement.className = `node-status ${online ? '' : 'offline'}`;
        }
    }
    
    flashNode(nodeId) {
        const node = document.querySelector(`[data-node="${nodeId}"]`);
        if (node) {
            node.style.animation = 'pulse 1s ease-in-out 2';
            setTimeout(() => {
                node.style.animation = '';
            }, 2000);
        }
    }
    
    initializeCharts() {
        // Simplified chart initialization for performance
        const chartConfigs = {
            temperature: this.createChartConfig('Temperature (°C)', 'rgb(55, 63, 255)'),
            battery: this.createChartConfig('Voltage (V)', 'rgb(0, 212, 170)'),
            latency: this.createChartConfig('Latency (ms)', 'rgb(245, 158, 11)'),
            detections: this.createChartConfig('Detections/min', 'rgb(16, 185, 129)')
        };
        
        Object.keys(chartConfigs).forEach(chartType => {
            const canvas = document.getElementById(`${chartType}Chart`);
            if (canvas) {
                this.charts[chartType] = new Chart(canvas, {
                    type: chartType === 'detections' ? 'bar' : 'line',
                    data: this.chartData[chartType],
                    options: chartConfigs[chartType]
                });
            }
        });
    }
    
    createChartConfig(yAxisLabel, color) {
        return {
            responsive: true,
            maintainAspectRatio: false,
            animation: false, // Disable animations for performance
            plugins: {
                legend: {
                    labels: { color: '#cbd5e1' }
                }
            },
            scales: {
                x: {
                    ticks: { color: '#64748b', maxTicksLimit: 6 },
                    grid: { color: 'rgba(71, 85, 105, 0.3)' }
                },
                y: {
                    ticks: { color: '#64748b' },
                    grid: { color: 'rgba(71, 85, 105, 0.3)' },
                    title: {
                        display: true,
                        text: yAxisLabel,
                        color: '#cbd5e1'
                    }
                }
            },
            elements: {
                point: { radius: 2, hoverRadius: 4 }
            }
        };
    }
    
    startTimeUpdates() {
        setInterval(() => {
            const now = new Date();
            this.updateElementText('systemTime', now.toLocaleTimeString());
            
            const uptime = Math.floor((Date.now() - this.startTime) / 1000);
            this.updateElementText('uptimeDisplay', `Uptime: ${this.formatUptime(uptime)}`);
        }, 1000);
    }
    
    formatUptime(seconds) {
        const hours = Math.floor(seconds / 3600);
        const minutes = Math.floor((seconds % 3600) / 60);
        const secs = seconds % 60;
        
        if (hours > 0) {
            return `${hours}h ${minutes}m ${secs}s`;
        } else if (minutes > 0) {
            return `${minutes}m ${secs}s`;
        } else {
            return `${secs}s`;
        }
    }
    
    bindEvents() {
        // Handle window resize for charts
        window.addEventListener('resize', () => {
            Object.values(this.charts).forEach(chart => {
                chart.resize();
            });
        });
    }
}

// Initialize dashboard when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    console.log('Starting IRIS Dashboard...');
    window.olympusDashboard = new OlympusDashboard();
});

window.OlympusDashboard = OlympusDashboard; 