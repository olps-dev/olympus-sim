#!/usr/bin/env python3
"""
Web-based visualization for mmWave sensor data
This will work when RViz2 GUI doesn't display in WSL
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import json
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import os
from pathlib import Path

class MmWaveWebViz(Node):
    def __init__(self):
        super().__init__('mmwave_web_viz')
        
        # Subscribe to mmWave point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/mmwave/points',
            self.pointcloud_callback,
            10
        )
        
        self.latest_data = {
            'timestamp': 0,
            'points': [],
            'point_count': 0,
            'status': 'waiting for data...'
        }
        
        self.get_logger().info('mmWave Web Visualizer started')
        self.get_logger().info('Subscribing to /mmwave/points')
    
    def pointcloud_callback(self, msg):
        """Process incoming point cloud data"""
        try:
            # Convert point cloud to list of points
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append({
                    'x': float(point[0]),
                    'y': float(point[1]), 
                    'z': float(point[2])
                })
            
            # Update latest data
            self.latest_data = {
                'timestamp': time.time(),
                'points': points,
                'point_count': len(points),
                'status': f'Active - {len(points)} points detected'
            }
            
            self.get_logger().info(f'Received {len(points)} points')
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')
            self.latest_data['status'] = f'Error: {e}'

class WebHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, viz_node=None, **kwargs):
        self.viz_node = viz_node
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        if self.path == '/data':
            # Serve JSON data
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            data = self.viz_node.latest_data if self.viz_node else {'status': 'No data'}
            self.wfile.write(json.dumps(data).encode())
            
        elif self.path == '/' or self.path == '/index.html':
            # Serve main page
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            
            html = self.get_html_page()
            self.wfile.write(html.encode())
        else:
            super().do_GET()
    
    def get_html_page(self):
        return """
<!DOCTYPE html>
<html>
<head>
    <title>mmWave Sensor Visualization</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #1a1a1a; color: white; }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { text-align: center; margin-bottom: 30px; }
        .status { padding: 10px; background: #333; border-radius: 5px; margin-bottom: 20px; }
        .stats { display: flex; gap: 20px; margin-bottom: 20px; }
        .stat-box { flex: 1; padding: 15px; background: #2a2a2a; border-radius: 5px; text-align: center; }
        .visualization { width: 100%; height: 400px; background: #000; border: 1px solid #555; position: relative; overflow: hidden; }
        .point { position: absolute; width: 3px; height: 3px; background: #00ff00; border-radius: 50%; }
        .controls { margin-top: 20px; text-align: center; }
        button { padding: 10px 20px; margin: 5px; background: #007acc; color: white; border: none; border-radius: 5px; cursor: pointer; }
        button:hover { background: #005a99; }
        .active { color: #00ff00; }
        .error { color: #ff4444; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>mmWave Sensor Visualization</h1>
            <p>Real-time physics-based raycast data from Olympus simulation</p>
        </div>
        
        <div class="status" id="status">
            Status: Connecting...
        </div>
        
        <div class="stats">
            <div class="stat-box">
                <h3>Point Count</h3>
                <div id="pointCount">0</div>
            </div>
            <div class="stat-box">
                <h3>Update Rate</h3>
                <div id="updateRate">0 Hz</div>
            </div>
            <div class="stat-box">
                <h3>Last Update</h3>
                <div id="lastUpdate">Never</div>
            </div>
        </div>
        
        <div class="visualization" id="visualization">
            <div style="position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); color: #666;">
                Waiting for sensor data...
            </div>
        </div>
        
        <div class="controls">
            <button onclick="toggleUpdates()">Pause/Resume</button>
            <button onclick="clearVisualization()">Clear</button>
            <button onclick="resetView()">Reset View</button>
        </div>
    </div>

    <script>
        let isUpdating = true;
        let lastUpdateTime = 0;
        let updateCount = 0;
        let startTime = Date.now();

        function updateVisualization() {
            if (!isUpdating) return;
            
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    // Update status
                    const statusEl = document.getElementById('status');
                    statusEl.textContent = 'Status: ' + data.status;
                    statusEl.className = 'status ' + (data.point_count > 0 ? 'active' : '');
                    
                    // Update stats
                    document.getElementById('pointCount').textContent = data.point_count || 0;
                    
                    if (data.timestamp > lastUpdateTime) {
                        updateCount++;
                        lastUpdateTime = data.timestamp;
                        
                        // Calculate update rate
                        const elapsed = (Date.now() - startTime) / 1000;
                        const rate = elapsed > 0 ? (updateCount / elapsed).toFixed(1) : 0;
                        document.getElementById('updateRate').textContent = rate + ' Hz';
                        
                        // Update timestamp
                        const date = new Date(data.timestamp * 1000);
                        document.getElementById('lastUpdate').textContent = date.toLocaleTimeString();
                        
                        // Update visualization
                        renderPoints(data.points || []);
                    }
                })
                .catch(error => {
                    console.error('Error fetching data:', error);
                    document.getElementById('status').textContent = 'Status: Connection error';
                    document.getElementById('status').className = 'status error';
                });
        }

        function renderPoints(points) {
            const viz = document.getElementById('visualization');
            
            // Clear existing points
            viz.querySelectorAll('.point').forEach(p => p.remove());
            
            if (points.length === 0) {
                viz.innerHTML = '<div style="position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); color: #666;">No points detected</div>';
                return;
            }
            
            // Find bounds
            let minX = Math.min(...points.map(p => p.x));
            let maxX = Math.max(...points.map(p => p.x));
            let minY = Math.min(...points.map(p => p.y));
            let maxY = Math.max(...points.map(p => p.y));
            
            const width = viz.clientWidth;
            const height = viz.clientHeight;
            const margin = 20;
            
            // Scale points to fit visualization
            points.forEach(point => {
                const x = margin + ((point.x - minX) / (maxX - minX || 1)) * (width - 2 * margin);
                const y = margin + ((point.y - minY) / (maxY - minY || 1)) * (height - 2 * margin);
                
                const pointEl = document.createElement('div');
                pointEl.className = 'point';
                pointEl.style.left = x + 'px';
                pointEl.style.top = y + 'px';
                pointEl.title = `(${point.x.toFixed(2)}, ${point.y.toFixed(2)}, ${point.z.toFixed(2)})`;
                
                viz.appendChild(pointEl);
            });
        }

        function toggleUpdates() {
            isUpdating = !isUpdating;
            if (isUpdating) {
                startTime = Date.now();
                updateCount = 0;
            }
        }

        function clearVisualization() {
            document.getElementById('visualization').innerHTML = '<div style="position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); color: #666;">Cleared</div>';
        }

        function resetView() {
            updateCount = 0;
            startTime = Date.now();
            lastUpdateTime = 0;
        }

        // Start updating
        setInterval(updateVisualization, 100); // 10 Hz update rate
        updateVisualization(); // Initial call
    </script>
</body>
</html>
        """

def main():
    rclpy.init()
    
    # Create ROS2 node
    viz_node = MmWaveWebViz()
    
    # Create web server
    def create_handler(*args, **kwargs):
        return WebHandler(*args, viz_node=viz_node, **kwargs)
    
    server = HTTPServer(('localhost', 8080), create_handler)
    
    # Start web server in separate thread
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    
    print("Web visualization started at http://localhost:8080")
    print("Visualizing mmWave sensor data from /mmwave/points")
    print("Press Ctrl+C to stop")
    
    try:
        rclpy.spin(viz_node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        server.shutdown()
        viz_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
