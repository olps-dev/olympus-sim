#!/usr/bin/env python3
"""
Sensor Bridge for Project Olympus
Connects to Renode UART TCP server and responds to sensor commands.
"""

import socket
import json
import time
import threading
import sys
from sensor_stubs import BME680Stub, SSD1306Stub, BatteryADC


class SensorBridge:
    def __init__(self, host='localhost', port=3333):
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        
        # Initialize sensor stubs
        self.bme680 = BME680Stub()
        self.ssd1306 = SSD1306Stub()
        self.battery = BatteryADC()
        
    def connect(self):
        """Connect to the UART TCP server."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            print(f"Connected to UART server at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the server."""
        self.running = False
        if self.socket:
            self.socket.close()
            self.socket = None
    
    def handle_command(self, command):
        """Handle incoming commands from firmware."""
        command = command.strip()
        
        if command == "BME680?":
            # Return BME680 sensor data
            data = self.bme680.read()
            response = json.dumps(data)
            return response
            
        elif command.startswith("OLED?"):
            # Handle OLED display command
            text = command[5:]  # Remove "OLED?" prefix
            self.ssd1306.draw_banner(text)
            return '{"status": "ok"}'
            
        elif command == "ADC?":
            # Return battery voltage
            voltage = self.battery.voltage()
            response = json.dumps({"voltage": voltage})
            return response
            
        else:
            print(f"Unknown command: {command}")
            return '{"error": "unknown command"}'
    
    def run(self):
        """Main bridge loop."""
        if not self.connect():
            return
        
        self.running = True
        buffer = ""
        
        try:
            while self.running:
                try:
                    # Receive data from firmware
                    data = self.socket.recv(1024).decode('utf-8', errors='ignore')
                    if not data:
                        break
                    
                    buffer += data
                    
                    # Process complete lines (commands end with \n)
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip():
                            response = self.handle_command(line)
                            if response:
                                # Send response back to firmware
                                self.socket.send((response + '\n').encode('utf-8'))
                
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"Error in bridge loop: {e}")
                    break
                    
        except KeyboardInterrupt:
            print("\nBridge interrupted by user")
        finally:
            self.disconnect()
            print("Bridge disconnected")


def main():
    """Main entry point."""
    bridge = SensorBridge()
    
    print("Starting Sensor Bridge for Project Olympus")
    print("Waiting for connection from Renode simulation...")
    print("Press Ctrl+C to stop")
    
    try:
        bridge.run()
    except KeyboardInterrupt:
        print("\nShutting down bridge...")
    finally:
        bridge.disconnect()


if __name__ == "__main__":
    main() 