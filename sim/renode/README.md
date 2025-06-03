# Renode Simulation Guide

This directory contains the Renode configuration for Project Olympus ESP32 simulation.

## Quick Start

```bash
make sim_phase1        # builds firmware & starts Renode head-less
renode-monitor tcp:1234  # optional interactive monitor
```

## Files

- `esp32_wroom32.repl` - Platform description for ESP32-WROOM-32 with virtual peripherals
- `run.resc` - Renode script to load and start the simulation

## Virtual Peripherals

- **BME680** - Environmental sensor (I2C @ 0x76)
- **SSD1306** - OLED display (SPI)
- **ADC** - Battery voltage monitor (initial voltage: 3.7V)
- **UART0** - Serial communication (TCP port 3333)
- **NIC** - Network interface (tap0)

## Monitoring

The firmware UART output is available on TCP port 3333. You can connect using:

```bash
telnet localhost 3333
```

Or use any serial terminal that supports TCP connections. 