#!/bin/bash
# Docker entrypoint script for Olympus containers
# Provides flexible startup options and environment setup

set -e

# Default values
COMPONENT=${COMPONENT:-simulation}
MODE=${MODE:-production}
WAIT_FOR_DEPS=${WAIT_FOR_DEPS:-true}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log() {
    echo -e "${GREEN}[$(date +'%Y-%m-%d %H:%M:%S')] $1${NC}"
}

warn() {
    echo -e "${YELLOW}[$(date +'%Y-%m-%d %H:%M:%S')] WARNING: $1${NC}"
}

error() {
    echo -e "${RED}[$(date +'%Y-%m-%d %H:%M:%S')] ERROR: $1${NC}"
}

# Wait for a service to be available
wait_for_service() {
    local host=$1
    local port=$2
    local service_name=$3
    local timeout=${4:-60}
    
    log "Waiting for $service_name at $host:$port..."
    
    for i in $(seq 1 $timeout); do
        if nc -z "$host" "$port" 2>/dev/null; then
            log "$service_name is ready!"
            return 0
        fi
        sleep 1
    done
    
    error "$service_name not available after ${timeout}s timeout"
    return 1
}

# Wait for MQTT broker
wait_for_mqtt() {
    local broker=${MQTT_BROKER:-mosquitto}
    local port=${MQTT_PORT:-1883}
    
    if [ "$WAIT_FOR_DEPS" = "true" ]; then
        wait_for_service "$broker" "$port" "MQTT Broker" 60
    fi
}

# Setup Python environment
setup_python_env() {
    log "Setting up Python environment..."
    export PYTHONPATH="/app:$PYTHONPATH"
    export PYTHONUNBUFFERED=1
    
    # Create necessary directories
    mkdir -p /app/logs /app/metrics /app/data
    
    # Set permissions
    if [ "$(whoami)" = "root" ]; then
        chown -R olympus:olympus /app/logs /app/metrics /app/data 2>/dev/null || true
    fi
}

# Initialize configuration
init_config() {
    log "Initializing configuration for component: $COMPONENT"
    
    case $COMPONENT in
        simulation)
            # Initialize network configuration
            python3 -c "
import sys, os
sys.path.append('/app/network')
from config import NetworkConfigManager
manager = NetworkConfigManager('/app/config/network_config.json')
manager.set_scenario('${NETWORK_CONDITION:-good}')
print('Network configuration initialized')
" || warn "Could not initialize network configuration"
            ;;
        dashboard)
            # Create dashboard data directories
            mkdir -p /app/data/uploads /app/static/cache
            ;;
        metrics)
            # Initialize metrics directories
            mkdir -p /app/data /app/output
            ;;
        network)
            # Initialize network controller configuration
            mkdir -p /app/config
            ;;
    esac
}

# Health check function
health_check() {
    case $COMPONENT in
        simulation)
            python3 -c "import sys; print('Simulation healthy'); sys.exit(0)"
            ;;
        dashboard)
            curl -f http://localhost:5000/api/status >/dev/null 2>&1
            ;;
        metrics)
            [ -f /app/data/health.txt ]
            ;;
        network)
            python3 -c "from network.config import get_network_config_manager; print('Network healthy')"
            ;;
        *)
            echo "Unknown component: $COMPONENT"
            exit 1
            ;;
    esac
}

# Start the appropriate component
start_component() {
    log "Starting $COMPONENT in $MODE mode..."
    
    case $COMPONENT in
        simulation)
            exec python3 /app/simulation_runner.py
            ;;
        dashboard)
            if [ "$MODE" = "development" ]; then
                exec python3 /app/app.py --debug
            else
                exec python3 /app/app.py
            fi
            ;;
        metrics)
            exec python3 /app/metrics_collector.py
            ;;
        network)
            exec python3 /app/network_controller.py
            ;;
        *)
            error "Unknown component: $COMPONENT"
            exit 1
            ;;
    esac
}

# Signal handlers
cleanup() {
    log "Received shutdown signal, cleaning up..."
    
    # Kill any background processes
    jobs -p | xargs -r kill
    
    # Component-specific cleanup
    case $COMPONENT in
        simulation)
            # Clean up temporary files
            rm -f /app/container_sensor_bridge.py
            ;;
        dashboard)
            # Clean up temporary uploads
            rm -rf /app/data/uploads/tmp*
            ;;
    esac
    
    log "Cleanup completed"
    exit 0
}

# Set up signal handlers
trap cleanup SIGTERM SIGINT

# Main execution
main() {
    log "Starting Olympus $COMPONENT container..."
    log "Mode: $MODE"
    log "Wait for dependencies: $WAIT_FOR_DEPS"
    
    # Setup environment
    setup_python_env
    
    # Wait for dependencies if needed
    if [ "$WAIT_FOR_DEPS" = "true" ]; then
        wait_for_mqtt
    fi
    
    # Initialize configuration
    init_config
    
    # Health check (optional)
    if [ "${SKIP_HEALTH_CHECK:-false}" != "true" ]; then
        if ! health_check 2>/dev/null; then
            warn "Initial health check failed, but continuing..."
        fi
    fi
    
    # Start the component
    start_component
}

# Run main function if script is executed directly
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    main "$@"
fi