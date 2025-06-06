import time
import random
import math
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Callable
import socket
import json
import struct

# This will be resolved if mqtt_broker_sim.py is in the same directory or PYTHONPATH is set up.
# from .mqtt_broker_sim import MQTTSimulatedClient 

# Constants
BME680_I2C_ADDR = 0x77 # Example I2C Address for BME680
SSD1306_I2C_ADDR = 0x3C # Example I2C Address for SSD1306

# --- Basic Sensor Simulation Models ---
class BME680RegisterMap:
    """Simplified BME680 model for simulation purposes."""
    def __init__(self):
        self.temperature = 25.0  # deg C
        self.humidity = 50.0  # %RH
        self.pressure = 1012.0  # hPa
        self.gas_resistance = 50000.0  # Ohms
        self.chip_id = 0x61 # Default BME680 chip ID

    def read_temp_c(self):
        # Simulate slight variations
        self.temperature += random.uniform(-0.1, 0.1)
        return round(self.temperature, 2)

    def read_humidity_rh(self):
        self.humidity += random.uniform(-0.5, 0.5)
        self.humidity = max(0, min(100, self.humidity))
        return round(self.humidity, 2)

    def read_pressure_hpa(self):
        self.pressure += random.uniform(-0.2, 0.2)
        return round(self.pressure, 2)

    def read_gas_resistance_ohm(self):
        # Simulate gas sensor changes
        self.gas_resistance += random.uniform(-100, 100)
        self.gas_resistance = max(10000, min(1000000, self.gas_resistance))
        return round(self.gas_resistance)

class SSD1306Controller:
    """Simplified SSD1306 OLED display model."""
    def __init__(self, width=128, height=64):
        self.width = width
        self.height = height
        self.buffer = [[0] * width for _ in range(height)]
        # print(f"[{time.strftime('%H:%M:%S')}] SSD1306 Initialized ({width}x{height})")

    def display_text(self, line: int, text: str):
        # This is a stub. In a real sim, this would update the buffer.
        # For MQTT, we might send the text or a status update.
        # print(f"[{time.strftime('%H:%M:%S')}] [SSD1306] Line {line}: {text}")
        pass

    def clear_display(self):
        self.buffer = [[0] * self.width for _ in range(self.height)]
        # print(f"[{time.strftime('%H:%M:%S')}] [SSD1306] Display Cleared")
        pass

class ESP32ADCController:
    """Simplified ESP32 ADC model, e.g., for battery voltage."""
    def __init__(self):
        self.battery_voltage = 4.2  # Volts, fully charged
        self.min_voltage = 3.0
        self.discharge_rate = 0.0001 # Small voltage drop per read

    def read_battery_voltage(self):
        self.battery_voltage -= self.discharge_rate
        if self.battery_voltage < self.min_voltage:
            self.battery_voltage = self.min_voltage
        # Simulate ADC noise/precision
        return round(self.battery_voltage + random.uniform(-0.01, 0.01), 3)

# --- Hardware Sensor Bridge (MQTT Refactored) ---
class HardwareSensorBridge:
    def __init__(self, mqtt_client_sim, node_id: str):
        """
        Initializes the hardware bridge with an MQTT client for communication.
        Args:
            mqtt_client_sim: An instance of MQTTSimulatedClient or MQTTClientAdapter.
            node_id: The ID of the sensor node, used for MQTT topic construction.
        """
        if mqtt_client_sim is None:
            raise ValueError("MQTT client cannot be None")
        self.mqtt_client = mqtt_client_sim
        self.node_id = node_id
        
        # Initialize peripheral models
        self.bme680 = BME680RegisterMap()
        self.ssd1306 = SSD1306Controller()
        self.adc = ESP32ADCController()

        # Radar connection attributes
        self.radar_host = 'localhost'
        self.radar_port = 7654
        self.radar_socket: Optional[socket.socket] = None
        self.radar_buffer = b''
        self.human_present = False
        self._connect_to_radar_server()
        
        # Check if we're using a real MQTT client (MQTTClientAdapter) or a simulated one
        self.using_real_mqtt = hasattr(mqtt_client_sim, 'real_client')
        
        print(f"[{time.strftime('%H:%M:%S')}] HardwareSensorBridge for node '{self.node_id}' initialized with {'real' if self.using_real_mqtt else 'simulated'} MQTT client.")

    def read_all_sensors_and_publish(self):
        """Reads data from all simulated sensors and publishes it via MQTT."""
        timestamp = time.time()

        # BME680 readings
        temp = self.bme680.read_temp_c()
        humidity = self.bme680.read_humidity_rh()
        pressure = self.bme680.read_pressure_hpa()
        gas = self.bme680.read_gas_resistance_ohm()

        bme_payload = {
            'timestamp': timestamp,
            'temperature_c': temp,
            'humidity_rh': humidity,
            'pressure_hpa': pressure,
            'gas_resistance_ohm': gas
        }
        topic_bme = f"sensor_data/{self.node_id}/bme680"
        self._publish_to_mqtt(topic_bme, bme_payload)
        # print(f"[{time.strftime('%H:%M:%S')}] [{self.node_id}] Published to {topic_bme}: {bme_payload}")

        # Battery ADC reading
        battery_voltage = self.adc.read_battery_voltage()
        battery_payload = {'timestamp': timestamp, 'voltage_v': battery_voltage}
        topic_battery = f"sensor_data/{self.node_id}/battery"
        self._publish_to_mqtt(topic_battery, battery_payload)
        # print(f"[{time.strftime('%H:%M:%S')}] [{self.node_id}] Published to {topic_battery}: {battery_payload}")
        
        # SSD1306 (Example: publish a status or some data that would be displayed)
        # For simulation, we might not publish the entire display buffer.
        # Instead, publish a representative status or key metric.
        display_status_payload = {
            'timestamp': timestamp, 
            'status': "OK", 
            'message': f"Temp: {temp:.1f}C, Batt: {battery_voltage:.2f}V"
        }
        topic_display = f"sensor_data/{self.node_id}/display_status"
        self._publish_to_mqtt(topic_display, display_status_payload)
        # print(f"[{time.strftime('%H:%M:%S')}] [{self.node_id}] Published to {topic_display}: {display_status_payload}")
        
    def _publish_to_mqtt(self, topic, payload):
        """Helper method to publish to MQTT with proper topic prefixing for real clients."""
        if self.using_real_mqtt:
            # Real MQTT client expects topics to be prefixed with 'olympus/'
            full_topic = f"olympus/{topic}"
            self.mqtt_client.publish(full_topic, payload)
        else:
            # Simulated MQTT client uses topics as-is
            self.mqtt_client.publish(topic, payload)

    def _connect_to_radar_server(self):
        try:
            if self.radar_socket:
                self.radar_socket.close()
            self.radar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.radar_socket.settimeout(1.0) # Timeout for connection attempt
            self.radar_socket.connect((self.radar_host, self.radar_port))
            self.radar_socket.setblocking(False) # Non-blocking for reads
            print(f"[{time.strftime('%H:%M:%S')}] [{self.node_id}] Connected to Radar Server at {self.radar_host}:{self.radar_port}")
        except socket.error as e:
            # print(f"[{time.strftime('%H:%M:%S')}] [{self.node_id}] Error connecting to Radar Server: {e}. Will retry on next cycle.")
            self.radar_socket = None # Ensure it's None if connection failed

    def _read_radar_data(self):
        if not self.radar_socket:
            # Attempt to reconnect if socket is not valid
            # print(f"[{time.strftime('%H:%M:%S')}] [{self.node_id}] No radar socket, attempting to reconnect...")
            self._connect_to_radar_server()
            if not self.radar_socket:
                self.human_present = False # Assume no human if no radar connection
                return

        try:
            # Read data in a non-blocking way
            while True:
                chunk = self.radar_socket.recv(4096)
                if not chunk:
                    # Connection closed by server
                    print(f"[{time.strftime('%H:%M:%S')}] [{self.node_id}] Radar server closed connection.")
                    self.radar_socket.close()
                    self.radar_socket = None
                    self.human_present = False
                    return
                self.radar_buffer += chunk
                
                # Process buffer for complete messages (length-prefixed)
                while len(self.radar_buffer) >= 2:
                    payload_len = struct.unpack('<H', self.radar_buffer[:2])[0]
                    if len(self.radar_buffer) >= 2 + payload_len:
                        payload_bytes = self.radar_buffer[2:2+payload_len]
                        self.radar_buffer = self.radar_buffer[2+payload_len:]
                        try:
                            frame_str = payload_bytes.decode('utf-8').strip()
                            if frame_str:
                                frame_json = json.loads(frame_str)
                                # print(f"[{time.strftime('%H:%M:%S')}] [{self.node_id}] Received radar frame: {frame_json}")
                                if 'points' in frame_json and len(frame_json['points']) > 0:
                                    self.human_present = True
                                else:
                                    self.human_present = False
                            else:
                                self.human_present = False # Empty payload considered no detection
                        except json.JSONDecodeError as e:
                            print(f"[{time.strftime('%H:%M:%S')}] [{self.node_id}] Error decoding radar JSON: {e} on payload: {payload_bytes}")
                            self.human_present = False
                        except Exception as e:
                            print(f"[{time.strftime('%H:%M:%S')}] [{self.node_id}] Unexpected error processing radar frame: {e}")
                            self.human_present = False
                    else:
                        break # Not enough data for full payload
        except BlockingIOError:
            # No data available to read, which is normal for non-blocking sockets
            pass # self.human_present remains its previous state until new data comes
        except socket.error as e:
            print(f"[{time.strftime('%H:%M:%S')}] [{self.node_id}] Radar socket error: {e}")
            if self.radar_socket:
                self.radar_socket.close()
            self.radar_socket = None
            self.human_present = False
        except Exception as e:
            print(f"[{time.strftime('%H:%M:%S')}] [{self.node_id}] Unexpected error in _read_radar_data: {e}")
            if self.radar_socket:
                self.radar_socket.close()
            self.radar_socket = None
            self.human_present = False

    def read_all_sensors_and_publish(self):
        """Reads data from all simulated sensors and publishes it via MQTT."""
        self._read_radar_data() # Update human_present status
        timestamp = time.time()

        # BME680 readings
        temp = self.bme680.read_temp_c()
        humidity = self.bme680.read_humidity_rh()
        pressure = self.bme680.read_pressure_hpa()
        gas = self.bme680.read_gas_resistance_ohm()

        bme_payload = {
            'timestamp': timestamp,
            'temperature_c': temp,
            'humidity_rh': humidity,
            'pressure_hpa': pressure,
            'gas_resistance_ohm': gas
        }
        topic_bme = f"sensor_data/{self.node_id}/bme680"
        self._publish_to_mqtt(topic_bme, bme_payload)

        # Battery ADC reading
        battery_voltage = self.adc.read_battery_voltage()
        battery_payload = {'timestamp': timestamp, 'voltage_v': battery_voltage}
        topic_battery = f"sensor_data/{self.node_id}/battery"
        self._publish_to_mqtt(topic_battery, battery_payload)
        
        # SSD1306 (Example: publish a status or some data that would be displayed)
        display_status_payload = {
            'timestamp': timestamp, 
            'status': "OK", 
            'message': f"Temp: {temp:.1f}C, Batt: {battery_voltage:.2f}V",
            'lamp_on': self.human_present # Add lamp status based on radar
        }
        topic_display = f"sensor_data/{self.node_id}/display_status"
        self._publish_to_mqtt(topic_display, display_status_payload)
        # print(f"[{time.strftime('%H:%M:%S')}] [{self.node_id}] Published to {topic_display}: {display_status_payload}")

    # --- I2C/SPI Methods (Stubs for potential future firmware-level simulation) ---
    # These methods would typically be called by a simulated firmware interacting with the bridge.
    # For the current MQTT-based data publishing, they are not directly used by read_all_sensors_and_publish(),
    # but are kept as stubs to represent the hardware interface layer.

    def i2c_master_write_to_device(self, i2c_port, device_address, write_buffer, write_size, ticks_to_wait):
        # print(f"[{time.strftime('%H:%M:%S')}] [HW BRIDGE {self.node_id}] I2C Write to {hex(device_address)}: {bytes(write_buffer[:write_size])}")
        # Simulate success for now
        return True

    def i2c_master_read_from_device(self, i2c_port, device_address, read_buffer, read_size, ticks_to_wait):
        # print(f"[{time.strftime('%H:%M:%S')}] [HW BRIDGE {self.node_id}] I2C Read from {hex(device_address)} (requesting {read_size} bytes)")
        if device_address == BME680_I2C_ADDR:
            # Example: Return Chip ID if register for chip ID is read (highly simplified)
            if read_size > 0: read_buffer[0] = self.bme680.chip_id 
            return True
        # Simulate success for now, actual data would depend on emulated device state
        return True

    def spi_master_transmit_receive(self, spi_host, spi_transaction):
        # print(f"[{time.strftime('%H:%M:%S')}] [HW BRIDGE {self.node_id}] SPI Transaction on host {spi_host}")
        # Simulate success for now
        return True

# Conceptual standalone test code for HardwareSensorBridge (MQTT version)
if __name__ == '__main__':
    # This test requires MQTTSimulatedClient, which is in mqtt_broker_sim.py
    # To run this standalone, ensure mqtt_broker_sim.py is in PYTHONPATH or same directory.
    try:
        from mqtt_broker_sim import MQTTBrokerSim, MQTTSimulatedClient
    except ImportError:
        print("Error: MQTTSimulatedClient not found. Ensure mqtt_broker_sim.py is accessible.")
        print("This test might not run correctly without it.")
        # Define dummy classes if import fails, so the rest of the test can be parsed
        class MQTTSimulatedClient:
            def __init__(self, client_id_node, broker_sim):
                self.client_id = client_id_node.node_id if hasattr(client_id_node, 'node_id') else client_id_node
                self.broker_sim = broker_sim
                print(f"[{time.strftime('%H:%M:%S')}] (Dummy) MQTT Client '{self.client_id}' created for test.")
            def publish(self, topic, message, qos=0, retain=False):
                print(f"[{time.strftime('%H:%M:%S')}] (Dummy Client:{self.client_id}) Publishing to topic '{topic}': {message}")
                if self.broker_sim:
                    self.broker_sim.receive_publish(self.client_id, topic, message, qos, retain)
        class MQTTBrokerSim:
            def __init__(self, broker_id, owner_node=None): # Added owner_node for compatibility with main_sim
                self.broker_id = broker_id
                self.owner_node = owner_node # Not used in this dummy version
                print(f"[{time.strftime('%H:%M:%S')}] (Dummy) MQTT Broker '{self.broker_id}' created.")
            def receive_publish(self, client_id, topic, message, qos, retain):
                print(f"[{time.strftime('%H:%M:%S')}] (Dummy Broker) Received from {client_id} on topic '{topic}': {message}")

    print("\nRunning HardwareSensorBridge Standalone Test (MQTT Version)")

    @dataclass
    class TestNode:
        node_id: str

    # Setup a dummy broker and client for the bridge to use
    test_broker_instance = MQTTBrokerSim(broker_id="test_bridge_broker") # Assuming MQTTBrokerSim can be instantiated
    sensor_node_for_test = TestNode(node_id="standalone_sensor_123")
    mqtt_client_for_bridge = MQTTSimulatedClient(client_id_node=sensor_node_for_test, broker_sim=test_broker_instance)
    
    # Instantiate the bridge
    bridge = HardwareSensorBridge(mqtt_client_sim=mqtt_client_for_bridge, 
                                  node_id_for_topic=sensor_node_for_test.node_id)

    print("\nSimulating sensor reads and publishes for 3 ticks...")
    for i in range(3):
        print(f"--- Tick {i+1} ---")
        bridge.read_all_sensors_and_publish()
    print("\nHardwareSensorBridge test finished.")

    # Clean up radar socket if it was created by the bridge
    if hasattr(bridge, 'radar_socket') and bridge.radar_socket:
        print("Closing radar socket for standalone test.")
        bridge.radar_socket.close() # Simulate time passing between reads
    
    print("\nHardwareSensorBridge Standalone Test Finished.")