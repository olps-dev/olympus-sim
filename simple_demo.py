#!/usr/bin/env python3
"""
Project Olympus - Simple Demo
Shows the sensor output directly without TCP server complexity.
"""

import time
import sys
from pathlib import Path

# Add the sim/python directory to the path
sys.path.append(str(Path(__file__).parent / "sim" / "python"))

from sim.python.sensor_stubs import BME680Stub, SSD1306Stub, BatteryADC


def main():
    print("🎯 Project Olympus - Digital Twin Simulation")
    print("=" * 50)
    print()
    
    # Initialize sensor stubs
    bme680 = BME680Stub()
    ssd1306 = SSD1306Stub()
    battery = BatteryADC()
    
    # Simulate firmware boot sequence
    print("🔄 Simulating ESP32 firmware boot...")
    boot_messages = [
        "I (312) OLYMPUS_MAIN: Project Olympus - Digital Twin Node Starting",
        "I (318) OLYMPUS_MAIN: ESP-IDF Version: v5.3.3",
        "I (324) SENSOR_IFACE: Sensor interface initialized (simulation mode)",
        "I (329) OLYMPUS_MAIN: Sensor interface initialized successfully",
        "I (336) OLYMPUS_MAIN: Sensor monitoring task started",
        "I (342) OLYMPUS_MAIN: System ready - monitoring sensors every 1 second",
        "I (350) SENSOR_IFACE: Sensor monitoring task started",
        "SENSOR_OK"
    ]
    
    for message in boot_messages:
        print(f"📤 {message}")
        time.sleep(0.1)
    
    print()
    print("📊 Starting sensor monitoring loop...")
    print("🛑 Press Ctrl+C to stop")
    print()
    
    try:
        uptime = 0
        while True:
            # Read BME680 sensor
            bme_data = bme680.read()
            print(f"📤 BME680: Temperature={bme_data['temperature']:.1f}°C, IAQ={bme_data['iaq']:.1f}")
            
            # Read battery voltage
            voltage = battery.voltage()
            print(f"📤 Battery: {voltage:.2f}V")
            
            # Update OLED display
            display_text = f"T:{bme_data['temperature']:.1f} IAQ:{bme_data['iaq']:.0f}"
            ssd1306.draw_banner(display_text)
            
            # System heartbeat every 10 seconds
            if uptime % 10 == 0:
                print(f"📤 I (xxx) OLYMPUS_MAIN: System heartbeat - uptime: {uptime} seconds")
            
            time.sleep(1)
            uptime += 1
            
    except KeyboardInterrupt:
        print("\n🛑 Simulation stopped by user")
        print("🧹 Cleanup complete")


if __name__ == "__main__":
    main() 