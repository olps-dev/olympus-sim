import socket
import time
import pytest


def test_sensor_initialization():
    """Test that the firmware boots and sensors initialize properly."""
    
    # Connect to UART0 via TCP
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)  # 10 second timeout
    
    try:
        # Connect to the UART TCP server
        sock.connect(('localhost', 3333))
        
        # Read data from the socket
        start_time = time.time()
        received_data = b""
        
        while time.time() - start_time < 10:  # 10 second timeout
            try:
                data = sock.recv(1024)
                if data:
                    received_data += data
                    # Check if we received the expected signal
                    if b"SENSOR_OK" in received_data:
                        print("✓ Received SENSOR_OK signal")
                        return  # Test passed
                else:
                    time.sleep(0.1)  # Small delay if no data
            except socket.timeout:
                continue
            except Exception as e:
                pytest.fail(f"Error reading from socket: {e}")
        
        # If we get here, we didn't receive SENSOR_OK within timeout
        received_str = received_data.decode('utf-8', errors='ignore')
        pytest.fail(f"Did not receive SENSOR_OK within 10 seconds. Received: {received_str}")
        
    except ConnectionRefusedError:
        pytest.fail("Could not connect to UART TCP server on port 3333. Is the simulation running?")
    except Exception as e:
        pytest.fail(f"Unexpected error: {e}")
    finally:
        sock.close()


def test_sensor_data_output():
    """Test that sensor data is being output continuously."""
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(15)  # 15 second timeout for this test
    
    try:
        sock.connect(('localhost', 3333))
        
        start_time = time.time()
        received_data = b""
        sensor_ok_found = False
        bme680_data_found = False
        battery_data_found = False
        
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
                    
                    # If we found all expected data, test passes
                    if sensor_ok_found and bme680_data_found and battery_data_found:
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
            
        if missing:
            received_str = received_data.decode('utf-8', errors='ignore')
            pytest.fail(f"Missing expected data: {', '.join(missing)}. Received: {received_str}")
        
    except ConnectionRefusedError:
        pytest.fail("Could not connect to UART TCP server on port 3333. Is the simulation running?")
    except Exception as e:
        pytest.fail(f"Unexpected error: {e}")
    finally:
        sock.close()


if __name__ == "__main__":
    # Allow running the test directly
    test_sensor_initialization()
    test_sensor_data_output()
    print("All tests passed!") 