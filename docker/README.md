# Olympus IoT Simulation - Docker Deployment Guide

This guide covers the complete Docker containerization of the Olympus IoT simulation system.

## Overview

The Olympus system is containerized into several microservices:

- **mosquitto**: MQTT broker for message routing
- **simulation**: Main simulation engine with sensor bridge and automation
- **dashboard**: Real-time web dashboard for monitoring
- **metrics-collector**: Dedicated metrics collection and analysis
- **network-controller**: Network conditions management

## Quick Start

### Production Deployment

```bash
# Start the complete system
docker-compose up -d

# View logs
docker-compose logs -f

# Access dashboard
open http://localhost:5000
```

### Development Mode

```bash
# Start with development tools
docker-compose -f docker-compose.yml -f docker-compose.dev.yml up -d

# Access additional tools
open http://localhost:5000   # Main dashboard
open http://localhost:3000   # Grafana (admin/dev)
open http://localhost:4000   # MQTT Explorer
```

## Service Details

### MQTT Broker (mosquitto)
- **Ports**: 1883 (MQTT), 9001 (WebSocket)
- **Configuration**: `infra/mosquitto.conf`
- **Health Check**: Automatic MQTT ping test

### Simulation Engine (simulation)
- **Purpose**: Runs sensor bridge and automation logic
- **Environment Variables**:
  - `NETWORK_CONDITION`: Network simulation condition (excellent/good/fair/poor)
  - `SIMULATION_MODE`: Mode (headless/development)
  - `LOG_LEVEL`: Logging level (DEBUG/INFO/WARNING/ERROR)

### Dashboard (dashboard)
- **Port**: 5000
- **Purpose**: Real-time monitoring web interface
- **Features**:
  - Live latency charts
  - Battery level monitoring
  - Network status visualization
  - Automation event tracking

### Metrics Collector (metrics-collector)
- **Purpose**: Dedicated metrics collection and analysis
- **Environment Variables**:
  - `COLLECTION_INTERVAL`: Collection interval in minutes (default: 5)
  - `METRICS_RETENTION_DAYS`: How long to keep metrics (default: 7)

### Network Controller (network-controller)
- **Purpose**: Manages network simulation conditions
- **Environment Variables**:
  - `DEFAULT_CONDITION`: Initial network condition
  - `ENABLE_DYNAMIC_CONDITIONS`: Enable time-based condition changes

## Configuration

### Network Conditions

Control network simulation via environment variables or MQTT:

```bash
# Set via environment
NETWORK_CONDITION=poor docker-compose up simulation

# Set via MQTT
mosquitto_pub -t network/control/condition -m "excellent"
mosquitto_pub -t network/control/scenario -m "realistic"
```

Available conditions:
- `excellent`: < 0.1% loss, ~1ms latency
- `good`: ~1% loss, ~5ms latency  
- `fair`: ~5% loss, ~15ms latency
- `poor`: ~10% loss, ~50ms latency
- `unreliable`: ~25% loss, ~100ms latency

### Volume Mounts

Data is persisted in Docker volumes:

```bash
# View volumes
docker volume ls | grep olympus

# Backup metrics data
docker run --rm -v olympus_metrics_data:/data -v $(pwd):/backup alpine tar czf /backup/metrics-backup.tar.gz /data

# Restore metrics data
docker run --rm -v olympus_metrics_data:/data -v $(pwd):/backup alpine tar xzf /backup/metrics-backup.tar.gz -C /
```

## Monitoring and Debugging

### Health Checks

All services include health checks:

```bash
# Check service health
docker-compose ps

# View health check logs
docker inspect olympus-simulation --format='{{json .State.Health}}'
```

### Log Access

```bash
# All logs
docker-compose logs

# Specific service
docker-compose logs -f simulation

# Follow logs with timestamps
docker-compose logs -f -t dashboard
```

### MQTT Monitoring

```bash
# Subscribe to all topics
mosquitto_sub -h localhost -t '#' -v

# Monitor specific data streams
mosquitto_sub -h localhost -t 'sensor/+/presence'
mosquitto_sub -h localhost -t 'automation/events'
mosquitto_sub -h localhost -t 'network/stats'
```

## Performance and Scaling

### Resource Limits

Add resource limits in production:

```yaml
services:
  simulation:
    deploy:
      resources:
        limits:
          memory: 512M
          cpus: '1.0'
        reservations:
          memory: 256M
          cpus: '0.5'
```

### Horizontal Scaling

Scale sensor simulation:

```bash
# Scale to multiple simulation instances
docker-compose up --scale simulation=3

# Use different sensor IDs per instance
SENSOR_ID_PREFIX=zone1 docker-compose up simulation
SENSOR_ID_PREFIX=zone2 docker-compose up simulation
```

## Production Deployment

### Security Considerations

1. **Change Default Passwords**:
   ```bash
   # Update Grafana password
   GF_SECURITY_ADMIN_PASSWORD=secure_password docker-compose up grafana
   ```

2. **Network Security**:
   ```yaml
   networks:
     olympus-network:
       driver: bridge
       internal: true  # Isolate from external networks
   ```

3. **SSL/TLS Termination**:
   ```yaml
   dashboard:
     environment:
       - FLASK_ENV=production
       - USE_SSL=true
   ```

### Environment-Specific Configurations

Create environment-specific compose files:

```bash
# Production
docker-compose -f docker-compose.yml -f docker-compose.prod.yml up -d

# Staging  
docker-compose -f docker-compose.yml -f docker-compose.staging.yml up -d

# Development
docker-compose -f docker-compose.yml -f docker-compose.dev.yml up -d
```

## Troubleshooting

### Common Issues

1. **MQTT Connection Failures**:
   ```bash
   # Check MQTT broker status
   docker-compose logs mosquitto
   
   # Test connectivity
   docker-compose exec simulation mosquitto_pub -h mosquitto -t test -m "hello"
   ```

2. **Dashboard Not Loading**:
   ```bash
   # Check dashboard logs
   docker-compose logs dashboard
   
   # Verify port binding
   docker-compose ps dashboard
   ```

3. **Metrics Not Generating**:
   ```bash
   # Check metrics collector
   docker-compose logs metrics-collector
   
   # Verify file permissions
   docker-compose exec metrics-collector ls -la /app/metrics/
   ```

### Performance Issues

1. **High Latency**:
   - Check Docker host resources
   - Reduce logging verbosity
   - Increase health check intervals

2. **Memory Usage**:
   ```bash
   # Monitor container resources
   docker stats
   
   # Clean up old metrics
   docker-compose exec metrics-collector python3 -c "from scripts.metrics_collector import MetricsCollector; MetricsCollector().cleanup_old_data()"
   ```

## Integration with CI/CD

### GitHub Actions Integration

```yaml
# .github/workflows/docker-test.yml
- name: Test Docker deployment
  run: |
    docker-compose up -d
    sleep 30
    docker-compose exec -T simulation python3 tests/test_ci_simulation.py
    docker-compose down
```

### Automated Testing

```bash
# Run integration tests
docker-compose -f docker-compose.test.yml up --abort-on-container-exit

# Performance testing
docker-compose up -d
sleep 60
curl http://localhost:5000/api/status | jq '.latency_stats.p95' | test $(cat) -lt 300
```

## Backup and Recovery

### Data Backup

```bash
#!/bin/bash
# backup-olympus.sh
DATE=$(date +%Y%m%d_%H%M%S)
BACKUP_DIR="backups/$DATE"

mkdir -p "$BACKUP_DIR"

# Backup volumes
docker run --rm -v olympus_metrics_data:/data -v $(pwd)/$BACKUP_DIR:/backup alpine tar czf /backup/metrics.tar.gz /data
docker run --rm -v olympus_mosquitto_data:/data -v $(pwd)/$BACKUP_DIR:/backup alpine tar czf /backup/mosquitto.tar.gz /data

# Backup configurations
cp docker-compose.yml "$BACKUP_DIR/"
cp -r infra/ "$BACKUP_DIR/"

echo "Backup completed: $BACKUP_DIR"
```

### Recovery

```bash
#!/bin/bash
# restore-olympus.sh
BACKUP_DIR=$1

if [ -z "$BACKUP_DIR" ]; then
    echo "Usage: $0 <backup_directory>"
    exit 1
fi

# Stop services
docker-compose down

# Restore volumes
docker run --rm -v olympus_metrics_data:/data -v $(pwd)/$BACKUP_DIR:/backup alpine tar xzf /backup/metrics.tar.gz -C /
docker run --rm -v olympus_mosquitto_data:/data -v $(pwd)/$BACKUP_DIR:/backup alpine tar xzf /backup/mosquitto.tar.gz -C /

# Restart services
docker-compose up -d

echo "Recovery completed from: $BACKUP_DIR"
```

## Support

For issues and questions:

1. Check the logs: `docker-compose logs`
2. Verify health checks: `docker-compose ps`  
3. Test MQTT connectivity: `mosquitto_sub -h localhost -t '#'`
4. Access dashboard diagnostics: `http://localhost:5000/api/status`

The containerized Olympus system provides a robust, scalable platform for IoT simulation with professional monitoring and management capabilities.