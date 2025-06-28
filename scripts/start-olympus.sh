#!/bin/bash
# Olympus System Startup Script
# Complete system test and deployment

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log() {
    echo -e "${GREEN}[$(date +'%H:%M:%S')] $1${NC}"
}

warn() {
    echo -e "${YELLOW}[$(date +'%H:%M:%S')] WARNING: $1${NC}"
}

error() {
    echo -e "${RED}[$(date +'%H:%M:%S')] ERROR: $1${NC}"
}

info() {
    echo -e "${BLUE}[$(date +'%H:%M:%S')] INFO: $1${NC}"
}

# Function to check if Docker is running
check_docker() {
    if ! docker info >/dev/null 2>&1; then
        error "Docker is not running. Please start Docker first."
        exit 1
    fi
    log "Docker is running"
}

# Function to check if Docker Compose is available
check_docker_compose() {
    if ! command -v docker-compose >/dev/null 2>&1; then
        error "docker-compose not found. Please install Docker Compose."
        exit 1
    fi
    log "Docker Compose is available"
}

# Function to clean up previous runs
cleanup() {
    log "Cleaning up previous containers..."
    docker-compose down --remove-orphans >/dev/null 2>&1 || true
    docker system prune -f --volumes >/dev/null 2>&1 || true
}

# Function to build and start services
start_services() {
    log "Building and starting Olympus services..."
    
    # Build images
    info "Building Docker images (this may take a few minutes)..."
    docker-compose build --no-cache
    
    # Start services
    info "Starting services..."
    docker-compose up -d
    
    log "All services started"
}

# Function to wait for services to be ready
wait_for_services() {
    log "Waiting for services to be ready..."
    
    local max_attempts=60
    local attempt=0
    
    while [ $attempt -lt $max_attempts ]; do
        attempt=$((attempt + 1))
        
        # Check MQTT broker
        if docker-compose exec mosquitto mosquitto_pub -h localhost -t health -m test >/dev/null 2>&1; then
            break
        fi
        
        if [ $attempt -eq $max_attempts ]; then
            error "Services failed to start within timeout"
            return 1
        fi
        
        sleep 2
        info "Waiting for services... ($attempt/$max_attempts)"
    done
    
    log "Services are ready"
}

# Function to show service status
show_status() {
    echo
    log "=== OLYMPUS SYSTEM STATUS ==="
    docker-compose ps
    echo
    
    log "=== SERVICE HEALTH ==="
    for service in mosquitto simulation dashboard metrics-collector network-controller; do
        health=$(docker inspect "olympus-$service" --format='{{.State.Health.Status}}' 2>/dev/null || echo "unknown")
        case $health in
            "healthy") echo -e "  $service: ${GREEN}HEALTHY${NC}" ;;
            "unhealthy") echo -e "  $service: ${RED}UNHEALTHY${NC}" ;;
            "starting") echo -e "  $service: ${YELLOW}STARTING${NC}" ;;
            *) echo -e "  $service: ${YELLOW}UNKNOWN${NC}" ;;
        esac
    done
    echo
}

# Function to show access information
show_access_info() {
    log "=== ACCESS INFORMATION ==="
    echo
    echo "  Dashboard:     http://localhost:5000"
    echo "  MQTT Broker:   localhost:1883"
    echo "  WebSocket:     localhost:9001"
    echo
    echo "=== MQTT TOPICS TO MONITOR ==="
    echo "  sensor/+/presence     - Sensor data"
    echo "  automation/events     - Automation events"
    echo "  actuators/+/+         - Actuator commands"
    echo "  network/stats         - Network statistics"
    echo
    echo "=== EXAMPLE MQTT COMMANDS ==="
    echo "  # Monitor all topics:"
    echo "  mosquitto_sub -h localhost -t '#' -v"
    echo
    echo "  # Monitor sensor data:"
    echo "  mosquitto_sub -h localhost -t 'sensor/+/presence'"
    echo
    echo "  # Change network conditions:"
    echo "  mosquitto_pub -h localhost -t 'network/control/condition' -m 'poor'"
    echo
}

# Function to run system test
run_system_test() {
    log "Running system integration test..."
    
    # Wait a bit for data to flow
    sleep 10
    
    # Check if MQTT topics are active
    info "Checking MQTT data flow..."
    if timeout 10 mosquitto_sub -h localhost -t 'sensor/+/presence' -C 1 >/dev/null 2>&1; then
        log "Sensor data is flowing"
    else
        warn "No sensor data detected yet"
    fi
    
    # Check dashboard API
    info "Testing dashboard API..."
    if curl -s http://localhost:5000/api/status >/dev/null 2>&1; then
        log "Dashboard API is responding"
    else
        warn "Dashboard API not responding yet"
    fi
    
    # Check for automation events
    info "Checking for automation events..."
    if timeout 15 mosquitto_sub -h localhost -t 'automation/events' -C 1 >/dev/null 2>&1; then
        log "Automation events are being generated"
    else
        warn "No automation events detected yet (this is normal during startup)"
    fi
}

# Function to show logs
show_logs() {
    if [ "$1" = "logs" ]; then
        log "Showing system logs (Ctrl+C to exit)..."
        docker-compose logs -f
    fi
}

# Function to monitor system
monitor_system() {
    if [ "$1" = "monitor" ]; then
        log "Monitoring system (Ctrl+C to exit)..."
        while true; do
            clear
            show_status
            echo
            log "Recent MQTT activity:"
            timeout 5 mosquitto_sub -h localhost -t '#' -v | head -10 || true
            sleep 5
        done
    fi
}

# Main execution
main() {
    echo
    log "OLYMPUS IoT SIMULATION SYSTEM"
    log "=============================="
    echo
    
    # Check prerequisites
    check_docker
    check_docker_compose
    
    # Handle command line arguments
    case "${1:-start}" in
        "clean")
            cleanup
            log "Cleanup completed"
            exit 0
            ;;
        "stop")
            log "Stopping Olympus system..."
            docker-compose down
            log "System stopped"
            exit 0
            ;;
        "status")
            show_status
            exit 0
            ;;
        "logs")
            show_logs logs
            exit 0
            ;;
        "monitor")
            monitor_system monitor
            exit 0
            ;;
        "start"|"")
            # Continue with startup
            ;;
        *)
            echo "Usage: $0 [start|stop|clean|status|logs|monitor]"
            echo
            echo "Commands:"
            echo "  start    - Start the complete system (default)"
            echo "  stop     - Stop all services"
            echo "  clean    - Clean up containers and volumes"
            echo "  status   - Show service status"
            echo "  logs     - Show live logs"
            echo "  monitor  - Live system monitoring"
            exit 1
            ;;
    esac
    
    # Clean up any previous runs
    cleanup
    
    # Start services
    start_services
    
    # Wait for services to be ready
    wait_for_services
    
    # Show status
    show_status
    
    # Run integration test
    run_system_test
    
    # Show access information
    show_access_info
    
    log "=== OLYMPUS SYSTEM READY ==="
    echo
    info "The system is now running. Key points:"
    echo "  1. Open http://localhost:5000 for the dashboard"
    echo "  2. Watch real-time sensor data and automation"
    echo "  3. Monitor MQTT topics for data flow"
    echo "  4. Check logs with: ./start-olympus.sh logs"
    echo "  5. Stop with: ./start-olympus.sh stop"
    echo
    warn "Note: It may take 1-2 minutes for all data to appear in the dashboard"
    echo
}

# Run main function
main "$@"