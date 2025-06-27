# Test Scripts

This directory contains test and monitoring scripts for the Olympus simulation.

## Files

- `test_automation_loop.py` - End-to-end automation testing with latency metrics
- `test_mmwave_mqtt.py` - Monitor MQTT sensor topics for presence detection

## Usage

### Test Full Automation Loop
```bash
# Run for 30 seconds with detailed metrics
python3 tests/test_automation_loop.py 30
```

This script monitors:
- Sensor presence detection
- Actuator commands
- Automation events
- Latency measurements

### Monitor MQTT Sensor Data
```bash
python3 tests/test_mmwave_mqtt.py
```

This script subscribes to and displays:
- `sensor/mmwave1/presence` - Detailed presence data
- `sensor/mmwave1/human_present` - Boolean presence status
- `sensor/mmwave1/status` - Sensor health status

## Testing Workflow

1. Start simulation: `./olympus full --automation`
2. Run tests: `python3 tests/test_automation_loop.py 30`
3. Monitor MQTT: `mosquitto_sub -t sensor/#`
