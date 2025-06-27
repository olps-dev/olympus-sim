# Automation Components

This directory contains the automation logic for the Olympus simulation.

## Files

- `automation_demo.py` - Main automation controller that subscribes to MQTT presence topics and controls actuators

## Usage

The automation controller is automatically launched when using:
```bash
./olympus full --automation
```

Or run standalone:
```bash
python3 automation/automation_demo.py
```

## Features

- Subscribes to `sensor/mmwave1/human_present` MQTT topic
- Controls `actuators/lamp_hall/on` based on presence detection
- Publishes automation events with latency metrics
- Configurable cooldown periods to prevent rapid toggling
