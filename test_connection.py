#!/usr/bin/env python3
"""
Simple connection test for Project Olympus simulation
"""

import socket
import time
import threading

def test_connection():
    """Test connection to the simulation"""
    print("Testing connection to localhost:3333...")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(60)  # Very long timeout
        print("📡 Connecting...")
        sock.connect(('localhost', 3333))
        
        print("✅ Connected successfully!")
        print("📡 Connection established, waiting for simulation data...")
        
        # Wait for data
        received_data = b""
        start_time = time.time()
        sensor_ok_found = False
        bme680_found = False
        battery_found = False
        last_data_time = time.time()
        
        while time.time() - start_time < 60:  # Wait up to 60 seconds
            try:
                sock.settimeout(1.0)  # 1 second timeout for recv
                data = sock.recv(1024)
                if data:
                    received_data += data
                    last_data_time = time.time()
                    text = data.decode('utf-8', errors='ignore').strip()
                    for line in text.split('\n'):
                        if line.strip():
                            print(f"📤 {line}")
                    
                    if b"SENSOR_OK" in received_data and not sensor_ok_found:
                        print("✅ Found SENSOR_OK signal!")
                        sensor_ok_found = True
                    
                    if b"BME680:" in received_data and not bme680_found:
                        print("✅ Found BME680 data!")
                        bme680_found = True
                    
                    if b"Battery:" in received_data and not battery_found:
                        print("✅ Found Battery data!")
                        battery_found = True
                        
                    if sensor_ok_found and bme680_found and battery_found:
                        print("✅ All expected data found! Test successful!")
                        time.sleep(2)  # Wait a bit more to see additional data
                        break
                        
                else:
                    # Connection closed by server
                    print("🔌 Connection closed by simulation")
                    break
                    
            except socket.timeout:
                # No data received in the last second
                elapsed = time.time() - last_data_time
                if elapsed > 10:
                    print(f"⏰ No data for {elapsed:.1f} seconds")
                    last_data_time = time.time()  # Reset timer
                continue
        
        print(f"\n📊 Total data received: {len(received_data)} bytes")
        print(f"📊 SENSOR_OK: {'✅' if sensor_ok_found else '❌'}")
        print(f"📊 BME680 data: {'✅' if bme680_found else '❌'}")
        print(f"📊 Battery data: {'✅' if battery_found else '❌'}")
        
        sock.close()
        
    except Exception as e:
        print(f"❌ Connection failed: {e}")

if __name__ == "__main__":
    test_connection() 