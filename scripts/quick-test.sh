#!/bin/bash
# Quick Test Script for Olympus System
# Fast local testing without Docker

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log() { echo -e "${GREEN}[$(date +'%H:%M:%S')] $1${NC}"; }
warn() { echo -e "${YELLOW}[$(date +'%H:%M:%S')] $1${NC}"; }
error() { echo -e "${RED}[$(date +'%H:%M:%S')] $1${NC}"; }

# Check if running in WSL or Linux
check_environment() {
    log "Checking environment..."
    
    if command -v mosquitto >/dev/null 2>&1; then
        log "Mosquitto found"
    else
        error "Mosquitto not installed. Install with:"
        echo "  Ubuntu/Debian: sudo apt install mosquitto mosquitto-clients"
        echo "  macOS: brew install mosquitto"
        exit 1
    fi
    
    if python3 -c "import pandas, matplotlib, paho.mqtt.client" 2>/dev/null; then
        log "Python dependencies available"
    else
        error "Missing Python dependencies. Install with:"
        echo "  pip install -r requirements.txt"
        echo "  pip install -r requirements-step4.txt"
        echo "  pip install -r dashboard/requirements.txt"
        exit 1
    fi
}

# Start MQTT broker
start_mqtt() {
    log "Starting MQTT broker..."
    
    # Kill any existing mosquitto processes
    pkill mosquitto 2>/dev/null || true
    sleep 1
    
    # Start mosquitto in background
    mosquitto -d -p 1883
    sleep 2
    
    # Test connection
    if mosquitto_pub -h localhost -t test -m "ping" 2>/dev/null; then
        log "MQTT broker ready"
    else
        error "MQTT broker failed to start"
        exit 1
    fi
}

# Start simulation components
start_simulation() {
    log "Starting simulation components..."
    
    # Set environment
    export PYTHONPATH="$(pwd):$PYTHONPATH"
    export MQTT_BROKER=localhost
    export MQTT_PORT=1883
    
    # Start automation in background
    log "Starting automation controller..."
    python3 automation/automation_demo.py > automation.log 2>&1 &
    AUTOMATION_PID=$!
    sleep 2
    
    # Start sensor bridge
    log "Starting sensor bridge..."
    python3 -c "
import sys, os, time, random, json, math
sys.path.append('network')
sys.path.append('metrics')

from network_realism import create_realistic_mqtt_client
from latency_battery_tracker import LatencyBatteryTracker

class QuickTestBridge:
    def __init__(self):
        self.client = create_realistic_mqtt_client()
        self.tracker = LatencyBatteryTracker()
        self.sensors = ['mmwave1', 'mmwave2']
        for s in self.sensors:
            self.tracker.init_sensor_battery(s)
        self.running = True
        
    def run(self):
        print('Quick test bridge starting...')
        self.client.connect('localhost', 1883, 60)
        self.client.loop_start()
        
        try:
            start = time.time()
            count = 0
            while self.running and time.time() - start < 60:  # Run for 1 minute
                for sensor_id in self.sensors:
                    # Create presence pattern
                    elapsed = time.time() - start
                    presence = random.random() < (0.3 + 0.2 * math.sin(elapsed * 0.1))
                    
                    event_id = self.tracker.log_pointcloud_received(sensor_id)
                    
                    message = {
                        'timestamp': time.time(),
                        'sensor_id': sensor_id,
                        'human_present': presence,
                        'num_points': random.randint(5, 50) if presence else 0,
                        'analysis': f'quick test - {count}',
                        'event_id': event_id,
                        'battery_level_mah': self.tracker.get_battery_level(sensor_id)
                    }
                    
                    topic = f'sensor/{sensor_id}/presence'
                    self.client.publish(topic, json.dumps(message), qos=1)
                    self.tracker.log_mqtt_published(sensor_id, event_id)
                    
                    count += 1
                    
                time.sleep(0.5)  # 2Hz
                
        except KeyboardInterrupt:
            pass
        finally:
            self.client.disconnect()
            print(f'Bridge sent {count} messages')

bridge = QuickTestBridge()
bridge.run()
" > sensor_bridge.log 2>&1 &
    BRIDGE_PID=$!
    
    log "Simulation components started"
    log "PIDs: Automation=$AUTOMATION_PID, Bridge=$BRIDGE_PID"
}

# Start dashboard
start_dashboard() {
    log "Starting dashboard..."
    
    cd dashboard
    python3 app.py > ../dashboard.log 2>&1 &
    DASHBOARD_PID=$!
    cd ..
    
    sleep 3
    
    # Test dashboard
    if curl -s http://localhost:5000/api/status >/dev/null 2>&1; then
        log "Dashboard ready at http://localhost:5000"
    else
        warn "Dashboard may still be starting..."
    fi
}

# Monitor system
monitor_test() {
    log "Monitoring test for 30 seconds..."
    
    local start_time=$(date +%s)
    local end_time=$((start_time + 30))
    
    while [ $(date +%s) -lt $end_time ]; do
        local remaining=$((end_time - $(date +%s)))
        
        # Check for sensor data
        if timeout 2 mosquitto_sub -h localhost -t 'sensor/+/presence' -C 1 >/dev/null 2>&1; then
            log "Sensor data flowing (${remaining}s remaining)"
        else
            warn "No sensor data (${remaining}s remaining)"
        fi
        
        sleep 5
    done
}

# Show results
show_results() {
    log "Test results:"
    echo
    
    # Check metrics files
    if [ -f "metrics/latency_metrics.csv" ]; then
        local line_count=$(wc -l < metrics/latency_metrics.csv)
        log "Latency metrics: $line_count lines"
    else
        warn "No latency metrics file found"
    fi
    
    if [ -f "metrics/battery_metrics.csv" ]; then
        local line_count=$(wc -l < metrics/battery_metrics.csv)
        log "Battery metrics: $line_count lines"
    else
        warn "No battery metrics file found"
    fi
    
    # Check for plots
    if ls metrics/*.png >/dev/null 2>&1; then
        log "Generated plots:"
        ls -la metrics/*.png
    else
        warn "No plots generated yet"
    fi
    
    # Show recent MQTT activity
    echo
    log "Recent MQTT activity (5 seconds):"
    timeout 5 mosquitto_sub -h localhost -t '#' -v | head -10 || warn "No MQTT activity"
}

# Cleanup
cleanup() {
    log "Cleaning up..."
    
    # Kill processes
    [ ! -z "$AUTOMATION_PID" ] && kill $AUTOMATION_PID 2>/dev/null || true
    [ ! -z "$BRIDGE_PID" ] && kill $BRIDGE_PID 2>/dev/null || true
    [ ! -z "$DASHBOARD_PID" ] && kill $DASHBOARD_PID 2>/dev/null || true
    
    # Kill mosquitto
    pkill mosquitto 2>/dev/null || true
    
    # Clean up log files
    rm -f automation.log sensor_bridge.log dashboard.log
    
    log "Cleanup completed"
}

# Main execution
main() {
    echo
    log "OLYMPUS QUICK TEST"
    log "=================="
    echo
    
    # Set trap for cleanup
    trap cleanup EXIT
    
    # Run test sequence
    check_environment
    start_mqtt
    start_simulation
    start_dashboard
    monitor_test
    show_results
    
    echo
    log "=== QUICK TEST COMPLETED ==="
    echo
    log "Next steps:"
    echo "  1. Check dashboard: http://localhost:5000"
    echo "  2. View metrics files in metrics/ directory"
    echo "  3. Run full Docker system: ./start-olympus.sh"
    echo
    warn "Press Enter to stop all services, or Ctrl+C to exit immediately"
    read -r
}

# Handle command line args
case "${1:-run}" in
    "run"|"")
        main
        ;;
    "clean")
        cleanup
        exit 0
        ;;
    *)
        echo "Usage: $0 [run|clean]"
        echo "  run   - Run quick test (default)"
        echo "  clean - Clean up processes only"
        exit 1
        ;;
esac