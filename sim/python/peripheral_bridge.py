#!/usr/bin/env python3
"""
Project Olympus - Peripheral Bridge
Connects QEMU ESP32 to realistic sensor simulations with hardware-accurate timing
"""

import asyncio
import socket
import json
import time
import threading
import logging
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
import struct

from sensor_stubs import BME680Stub, SSD1306Stub, BatteryADC

# Set up logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger('PeripheralBridge')

@dataclass
class I2CTransaction:
    """Represents a single I2C transaction with timing."""
    address: int
    register: int
    data: bytes
    is_read: bool
    timestamp: float
    duration: float  # Realistic I2C timing

@dataclass
class SPITransaction:
    """Represents a single SPI transaction with timing."""
    chip_select: int
    data: bytes
    is_read: bool
    timestamp: float
    duration: float  # Realistic SPI timing

class PeripheralBridge:
    """Hardware-accurate bridge between QEMU and sensor simulations."""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.uart_port = config.get('uart_port', 3333)
        self.node_id = config.get('node_id', 0)
        self.running = False
        
        # Initialize sensor stubs
        self.bme680 = BME680Stub()
        self.ssd1306 = SSD1306Stub() 
        self.battery_adc = BatteryADC()
        
        # Hardware timing parameters (realistic I2C/SPI speeds)
        self.i2c_speed_hz = 100000  # 100kHz standard I2C
        self.spi_speed_hz = 10000000  # 10MHz SPI
        
        # Transaction logging for KPI collection
        self.i2c_transactions: List[I2CTransaction] = []
        self.spi_transactions: List[SPITransaction] = []
        
        # UART socket
        self.uart_socket = None
        self.client_socket = None
        
        # Protocol state
        self.command_buffer = b""
        
    async def start_uart_server(self):
        """Start UART TCP server to communicate with QEMU."""
        try:
            self.uart_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.uart_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.uart_socket.bind(('0.0.0.0', self.uart_port))
            self.uart_socket.listen(1)
            
            log.info(f"Peripheral bridge listening on port {self.uart_port}")
            
            # Wait for QEMU connection
            self.client_socket, addr = self.uart_socket.accept()
            log.info(f"QEMU ESP32 connected from {addr}")
            
            return True
            
        except Exception as e:
            log.error(f"Failed to start UART server: {e}")
            return False
    
    def calculate_i2c_timing(self, data_bytes: int) -> float:
        """Calculate realistic I2C transaction timing."""
        # I2C overhead: Start + Address + ACK + Register + ACK + Data + ACK + Stop
        # At 100kHz: ~10μs per bit, ~90μs per byte including overhead
        overhead_bits = 16  # Start, stop, ACKs
        total_bits = (data_bytes * 8) + overhead_bits
        return total_bits / self.i2c_speed_hz
    
    def calculate_spi_timing(self, data_bytes: int) -> float:
        """Calculate realistic SPI transaction timing."""
        # SPI is faster but includes CS setup/hold time
        cs_overhead = 0.000001  # 1μs CS setup/hold
        data_time = (data_bytes * 8) / self.spi_speed_hz
        return data_time + cs_overhead
    
    def simulate_i2c_transaction(self, address: int, register: int, 
                                data: bytes, is_read: bool) -> Optional[bytes]:
        """Simulate I2C transaction with realistic timing and error conditions."""
        start_time = time.time()
        
        # Simulate timing delay
        timing = self.calculate_i2c_timing(len(data) if data else 1)
        time.sleep(timing)  # Realistic I2C delay
        
        # Log transaction for KPI analysis
        transaction = I2CTransaction(
            address=address,
            register=register,
            data=data,
            is_read=is_read,
            timestamp=start_time,
            duration=timing
        )
        self.i2c_transactions.append(transaction)
        
        # Simulate 1% chance of NACK (realistic I2C errors)
        import random
        if random.random() < 0.01:
            log.warning(f"I2C NACK simulation: addr=0x{address:02X}, reg=0x{register:02X}")
            return None
        
        # Route to appropriate sensor
        try:
            if address == 0x77:  # BME680
                return self._handle_bme680_i2c(register, data, is_read)
            elif address == 0x3C:  # SSD1306
                return self._handle_ssd1306_i2c(register, data, is_read)
            else:
                log.warning(f"Unknown I2C device: 0x{address:02X}")
                return None
                
        except Exception as e:
            log.error(f"I2C transaction error: {e}")
            return None
    
    def _handle_bme680_i2c(self, register: int, data: bytes, is_read: bool) -> Optional[bytes]:
        """Handle BME680 I2C transactions with register-level accuracy."""
        if is_read:
            if register == 0x22:  # Temperature MSB
                sensor_data = self.bme680.read()
                # Convert to raw ADC value (realistic 20-bit ADC)
                temp_raw = int((sensor_data['temperature'] + 25) * 1000)  # Simplified conversion
                return struct.pack('>I', temp_raw)[:3]  # 3 bytes MSB first
            elif register == 0x2A:  # Gas resistance MSB
                sensor_data = self.bme680.read()
                iaq_raw = int(sensor_data['iaq'] * 100)
                return struct.pack('>H', iaq_raw)
            elif register == 0xD0:  # Chip ID register
                return b'\x61'  # BME680 chip ID
            else:
                log.debug(f"BME680 read from unknown register 0x{register:02X}")
                return b'\x00'
        else:
            # Write operation (configuration registers)
            log.debug(f"BME680 write to register 0x{register:02X}: {data.hex()}")
            return None
    
    def _handle_ssd1306_i2c(self, register: int, data: bytes, is_read: bool) -> Optional[bytes]:
        """Handle SSD1306 I2C transactions."""
        if is_read:
            # SSD1306 rarely read from
            return b'\x00'
        else:
            # Write operation (display commands/data)
            if data:
                # Interpret as display command or pixel data
                if data[0] & 0x80:  # Command
                    log.debug(f"SSD1306 command: 0x{data[0]:02X}")
                else:  # Data
                    # Extract text if possible
                    try:
                        text = data.decode('ascii', errors='ignore')
                        if text.strip():
                            self.ssd1306.draw_banner(text.strip())
                    except:
                        pass
            return None
    
    def simulate_adc_reading(self, channel: int) -> int:
        """Simulate ADC reading with realistic noise and timing."""
        # Simulate 12-bit ADC conversion time (~13μs)
        time.sleep(0.000013)
        
        if channel == 0:  # Battery voltage channel
            voltage = self.battery_adc.voltage()
            # Convert to 12-bit ADC value (0-4095)
            # Assuming 3.3V reference, battery through voltage divider
            adc_raw = int((voltage / 4.2) * 4095)
            
            # Add realistic ADC noise (±2 LSB)
            import random
            noise = random.randint(-2, 2)
            return max(0, min(4095, adc_raw + noise))
        
        return 0
    
    async def process_uart_data(self, data: bytes):
        """Process incoming UART data from QEMU ESP32."""
        self.command_buffer += data
        
        # Look for complete commands (newline terminated)
        while b'\n' in self.command_buffer:
            line, self.command_buffer = self.command_buffer.split(b'\n', 1)
            command = line.decode('utf-8', errors='ignore').strip()
            
            if command:
                await self.handle_command(command)
    
    async def handle_command(self, command: str):
        """Handle peripheral access commands from ESP32 firmware."""
        try:
            # Parse JSON command format
            if command.startswith('{'):
                cmd_data = json.loads(command)
                response = await self.execute_command(cmd_data)
                if response:
                    response_json = json.dumps(response)
                    self.send_uart_response(response_json)
            else:
                # Legacy text commands for backward compatibility
                await self.handle_legacy_command(command)
                
        except Exception as e:
            log.error(f"Command handling error: {e}")
            error_response = {"error": str(e)}
            self.send_uart_response(json.dumps(error_response))
    
    async def execute_command(self, cmd_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Execute structured peripheral commands."""
        cmd_type = cmd_data.get('type')
        
        if cmd_type == 'i2c_read':
            address = cmd_data.get('address')
            register = cmd_data.get('register')
            length = cmd_data.get('length', 1)
            
            result = self.simulate_i2c_transaction(address, register, b'', True)
            if result:
                return {
                    'type': 'i2c_response',
                    'data': result.hex(),
                    'length': len(result)
                }
            else:
                return {'type': 'i2c_response', 'error': 'NACK'}
        
        elif cmd_type == 'i2c_write':
            address = cmd_data.get('address')
            register = cmd_data.get('register')
            data = bytes.fromhex(cmd_data.get('data', ''))
            
            result = self.simulate_i2c_transaction(address, register, data, False)
            return {'type': 'i2c_response', 'success': result is not None}
        
        elif cmd_type == 'adc_read':
            channel = cmd_data.get('channel', 0)
            value = self.simulate_adc_reading(channel)
            return {
                'type': 'adc_response',
                'channel': channel,
                'value': value
            }
        
        elif cmd_type == 'get_status':
            return {
                'type': 'status_response',
                'node_id': self.node_id,
                'uptime': time.time(),
                'sensors': {
                    'bme680': {'status': 'ok'},
                    'ssd1306': {'status': 'ok'},
                    'battery_adc': {'status': 'ok'}
                }
            }
        
        return None
    
    async def handle_legacy_command(self, command: str):
        """Handle legacy text-based commands for backward compatibility."""
        if command == "BME680?":
            sensor_data = self.bme680.read()
            response = f"BME680: Temperature={sensor_data['temperature']:.1f}°C, IAQ={sensor_data['iaq']:.1f}"
            self.send_uart_response(response)
        
        elif command == "ADC?":
            voltage = self.battery_adc.voltage()
            response = f"Battery: {voltage:.2f}V"
            self.send_uart_response(response)
        
        elif command.startswith("OLED?"):
            text = command[5:]  # Remove "OLED?" prefix
            self.ssd1306.draw_banner(text)
            response = f"OLED: {text}"
            self.send_uart_response(response)
    
    def send_uart_response(self, response: str):
        """Send response back to ESP32 via UART."""
        if self.client_socket:
            try:
                self.client_socket.send((response + '\n').encode('utf-8'))
            except Exception as e:
                log.error(f"Failed to send UART response: {e}")
    
    def get_kpi_data(self) -> Dict[str, Any]:
        """Collect KPI data for analysis."""
        now = time.time()
        
        # Calculate I2C performance metrics
        i2c_latencies = [t.duration for t in self.i2c_transactions]
        spi_latencies = [t.duration for t in self.spi_transactions]
        
        return {
            'timestamp': now,
            'node_id': self.node_id,
            'i2c_stats': {
                'transaction_count': len(self.i2c_transactions),
                'avg_latency': sum(i2c_latencies) / len(i2c_latencies) if i2c_latencies else 0,
                'max_latency': max(i2c_latencies) if i2c_latencies else 0,
                'error_rate': 0.01  # Simulated 1% NACK rate
            },
            'spi_stats': {
                'transaction_count': len(self.spi_transactions),
                'avg_latency': sum(spi_latencies) / len(spi_latencies) if spi_latencies else 0,
                'max_latency': max(spi_latencies) if spi_latencies else 0
            },
            'sensor_readings': {
                'bme680': self.bme680.read(),
                'battery': {'voltage': self.battery_adc.voltage()}
            }
        }
    
    async def main_loop(self):
        """Main peripheral bridge loop."""
        if not await self.start_uart_server():
            return
        
        log.info("Peripheral bridge started successfully")
        
        self.running = True
        
        try:
            while self.running:
                if self.client_socket:
                    try:
                        # Set socket to non-blocking
                        self.client_socket.settimeout(0.1)
                        data = self.client_socket.recv(1024)
                        if data:
                            await self.process_uart_data(data)
                        else:
                            # Connection closed
                            break
                    except socket.timeout:
                        continue
                    except Exception as e:
                        log.error(f"UART communication error: {e}")
                        break
                else:
                    await asyncio.sleep(0.1)
                    
        except KeyboardInterrupt:
            log.info("Peripheral bridge interrupted")
        finally:
            self.cleanup()
    
    def run(self):
        """Run the peripheral bridge."""
        asyncio.run(self.main_loop())
    
    def stop(self):
        """Stop the peripheral bridge."""
        self.running = False
    
    def cleanup(self):
        """Clean up resources."""
        if self.client_socket:
            self.client_socket.close()
        if self.uart_socket:
            self.uart_socket.close()
        log.info("Peripheral bridge cleaned up")


if __name__ == '__main__':
    config = {
        'uart_port': 3333,
        'node_id': 0,
        'sensors': {}
    }
    
    bridge = PeripheralBridge(config)
    bridge.run() 