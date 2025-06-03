import socket
import time
import pytest
import subprocess
import threading
import sys
from pathlib import Path

# Add the scripts directory to the path
sys.path.append(str(Path(__file__).parent.parent / "scripts"))

from demo_simulation import FirmwareSimulator


def test_demo_simulation():
    """Test that the demo simulation works correctly."""
    
    # Start the simulator in a separate thread
    simulator = FirmwareSimulator()
    
    def run_simulator():
        simulator.run()
    
    # Start simulator thread
    sim_thread = threading.Thread(target=run_simulator, daemon=True)
    sim_thread.start()
    
    # Give the simulator time to start
    time.sleep(2)
    
    # Connect to the UART TCP server
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(15)  # 15 second timeout
    
    try:
        # Connect to the UART TCP server
        sock.connect(('localhost', 3333))
        
        # Read data from the socket
        start_time = time.time()
        received_data = b""
        sensor_ok_found = False
        bme680_data_found = False
        battery_data_found = False
        oled_data_found = False
        
        while time.time() - start_time < 15:
            try:
                data = sock.recv(1024)
                if data:
                    received_data += data
                    received_str = received_data.decode('utf-8', errors='ignore')
                    
                    # Check for various expected outputs
                    if b"SENSOR_OK" in received_data:
                        sensor_ok_found = True
                        print("✓ Found SENSOR_OK signal")
                    
                    if b"BME680:" in received_data:
                        bme680_data_found = True
                        print("✓ Found BME680 sensor data")
                    
                    if b"Battery:" in received_data:
                        battery_data_found = True
                        print("✓ Found Battery voltage data")
                    
                    if b"[OLED]" in received_data:
                        oled_data_found = True
                        print("✓ Found OLED display data")
                    
                    # If we found all expected data, test passes
                    if sensor_ok_found and bme680_data_found and battery_data_found and oled_data_found:
                        print("✓ All sensor data types detected")
                        return
                        
                else:
                    time.sleep(0.1)
            except socket.timeout:
                continue
            except Exception as e:
                pytest.fail(f"Error reading from socket: {e}")
        
        # Report what we found
        missing = []
        if not sensor_ok_found:
            missing.append("SENSOR_OK")
        if not bme680_data_found:
            missing.append("BME680 data")
        if not battery_data_found:
            missing.append("Battery data")
        if not oled_data_found:
            missing.append("OLED data")
            
        if missing:
            received_str = received_data.decode('utf-8', errors='ignore')
            pytest.fail(f"Missing expected data: {', '.join(missing)}. Received: {received_str}")
        
    except ConnectionRefusedError:
        pytest.fail("Could not connect to demo simulation on port 3333. Is the simulation running?")
    except Exception as e:
        pytest.fail(f"Unexpected error: {e}")
    finally:
        sock.close()
        simulator.cleanup()


def test_sensor_stubs():
    """Test that the sensor stubs generate realistic data."""
    
    # Import sensor stubs
    sys.path.append(str(Path(__file__).parent.parent / "sim" / "python"))
    from sensor_stubs import BME680Stub, SSD1306Stub, BatteryADC
    
    # Test BME680
    bme680 = BME680Stub()
    data = bme680.read()
    
    assert "temperature" in data
    assert "iaq" in data
    assert 15 <= data["temperature"] <= 30  # Reasonable temperature range
    assert 40 <= data["iaq"] <= 70  # Reasonable IAQ range
    print(f"✓ BME680 data: {data}")
    
    # Test Battery ADC
    battery = BatteryADC()
    voltage = battery.voltage()
    
    assert 3.5 <= voltage <= 4.5  # Reasonable battery voltage range
    print(f"✓ Battery voltage: {voltage}V")
    
    # Test SSD1306 (just ensure it doesn't crash)
    ssd1306 = SSD1306Stub()
    ssd1306.draw_banner("Test Message")
    print("✓ SSD1306 display test passed")


if __name__ == "__main__":
    # Allow running the test directly
    test_sensor_stubs()
    print("Sensor stub tests passed!")
    
    # Note: The demo simulation test requires manual setup
    print("To test demo simulation, run: make demo") 