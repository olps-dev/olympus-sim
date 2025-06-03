#!/usr/bin/env python3
"""
Project Olympus - QEMU ESP32 Runner
Provides proper Xtensa instruction-level simulation with peripheral bridging
"""

import subprocess
import threading
import time
import socket
import json
import logging
import signal
import sys
from pathlib import Path
from typing import Dict, Any, Optional

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
log = logging.getLogger('QEMURunner')

class ESP32QEMURunner:
    """Manages QEMU ESP32 instance with peripheral bridge integration."""
    
    def __init__(self, firmware_path: str, config: Dict[str, Any]):
        self.firmware_path = Path(firmware_path)
        self.config = config
        self.qemu_process: Optional[subprocess.Popen] = None
        self.peripheral_bridge = None
        self.running = False
        
        # QEMU machine configuration for ESP32-WROOM-32
        self.qemu_args = [
            'qemu-system-xtensa',
            '-machine', 'esp32',
            '-cpu', 'esp32',
            '-smp', '2',  # Dual core
            '-m', '4M',   # 4MB RAM
            '-serial', f'tcp::{config.get("uart_port", 3333)},server,nowait',
            '-netdev', f'tap,id=net0,ifname=tap-esp32-{config.get("node_id", 0)},script=no,downscript=no',
            '-device', 'open_eth,netdev=net0',
            '-monitor', f'tcp::{config.get("monitor_port", 4444)},server,nowait',
            '-kernel', str(self.firmware_path),
            '-s',  # GDB server on port 1234
            '-nographic'
        ]
        
        if config.get('debug', False):
            self.qemu_args.extend(['-d', 'cpu,guest_errors'])
        
        # Set up signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def setup_tap_interface(self) -> bool:
        """Set up TAP interface for network simulation."""
        tap_name = f"tap-esp32-{self.config.get('node_id', 0)}"
        
        try:
            # Create TAP interface
            subprocess.run(['sudo', 'ip', 'tuntap', 'add', 'dev', tap_name, 'mode', 'tap'], check=True)
            subprocess.run(['sudo', 'ip', 'link', 'set', tap_name, 'up'], check=True)
            
            # Assign IP address (192.168.100.x subnet for mesh)
            node_ip = f"192.168.100.{10 + self.config.get('node_id', 0)}"
            subprocess.run(['sudo', 'ip', 'addr', 'add', f'{node_ip}/24', 'dev', tap_name], check=True)
            
            log.info(f"TAP interface {tap_name} created with IP {node_ip}")
            return True
            
        except subprocess.CalledProcessError as e:
            log.error(f"Failed to setup TAP interface: {e}")
            return False
    
    def start_peripheral_bridge(self):
        """Start the peripheral bridge for sensor simulation."""
        from ..python.peripheral_bridge import PeripheralBridge
        
        bridge_config = {
            'uart_port': self.config.get('uart_port', 3333),
            'sensor_config': self.config.get('sensors', {}),
            'node_id': self.config.get('node_id', 0)
        }
        
        self.peripheral_bridge = PeripheralBridge(bridge_config)
        bridge_thread = threading.Thread(target=self.peripheral_bridge.run, daemon=True)
        bridge_thread.start()
        log.info("Peripheral bridge started")
    
    def start_qemu(self) -> bool:
        """Start the QEMU ESP32 instance."""
        if not self.firmware_path.exists():
            log.error(f"Firmware not found: {self.firmware_path}")
            return False
        
        log.info(f"Starting QEMU ESP32 with firmware: {self.firmware_path}")
        log.info(f"UART available on port {self.config.get('uart_port', 3333)}")
        log.info(f"Monitor available on port {self.config.get('monitor_port', 4444)}")
        
        try:
            self.qemu_process = subprocess.Popen(
                self.qemu_args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            
            # Monitor QEMU output
            def monitor_output():
                for line in iter(self.qemu_process.stdout.readline, ''):
                    if line.strip():
                        log.info(f"QEMU: {line.strip()}")
                        
            output_thread = threading.Thread(target=monitor_output, daemon=True)
            output_thread.start()
            
            # Wait a moment for QEMU to start
            time.sleep(2)
            
            if self.qemu_process.poll() is None:
                log.info("QEMU ESP32 started successfully")
                return True
            else:
                log.error("QEMU failed to start")
                return False
                
        except Exception as e:
            log.error(f"Failed to start QEMU: {e}")
            return False
    
    def wait_for_boot(self, timeout: int = 30) -> bool:
        """Wait for ESP32 to complete boot sequence."""
        log.info("Waiting for ESP32 boot sequence...")
        
        start_time = time.time()
        boot_complete = False
        
        try:
            # Connect to UART to monitor boot messages
            uart_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            uart_sock.settimeout(5)
            uart_sock.connect(('localhost', self.config.get('uart_port', 3333)))
            
            uart_data = b""
            while time.time() - start_time < timeout and not boot_complete:
                try:
                    data = uart_sock.recv(1024)
                    if data:
                        uart_data += data
                        text = uart_data.decode('utf-8', errors='ignore')
                        
                        # Look for boot completion signals
                        if 'SENSOR_OK' in text or 'System ready' in text:
                            boot_complete = True
                            log.info("ESP32 boot completed successfully")
                            break
                            
                except socket.timeout:
                    continue
            
            uart_sock.close()
            
        except Exception as e:
            log.warning(f"Could not monitor UART during boot: {e}")
            # Fallback: just wait for a reasonable time
            time.sleep(10)
            boot_complete = True
        
        return boot_complete
    
    def run(self) -> bool:
        """Run the complete ESP32 simulation."""
        log.info("🚀 Starting Project Olympus ESP32 Node Simulation")
        
        # Setup networking
        if not self.setup_tap_interface():
            log.error("Failed to setup networking")
            return False
        
        # Start peripheral bridge
        self.start_peripheral_bridge()
        
        # Start QEMU
        if not self.start_qemu():
            log.error("Failed to start QEMU")
            return False
        
        # Wait for boot
        if not self.wait_for_boot():
            log.warning("Boot sequence may not have completed properly")
        
        self.running = True
        
        try:
            # Keep running until interrupted
            while self.running and self.qemu_process.poll() is None:
                time.sleep(1)
                
        except KeyboardInterrupt:
            log.info("Simulation interrupted by user")
        finally:
            self.cleanup()
        
        return True
    
    def cleanup(self):
        """Clean up QEMU process and resources."""
        log.info("Cleaning up ESP32 simulation...")
        
        self.running = False
        
        if self.qemu_process:
            try:
                self.qemu_process.terminate()
                self.qemu_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.qemu_process.kill()
                self.qemu_process.wait()
            log.info("QEMU process terminated")
        
        if self.peripheral_bridge:
            self.peripheral_bridge.stop()
        
        # Clean up TAP interface
        tap_name = f"tap-esp32-{self.config.get('node_id', 0)}"
        try:
            subprocess.run(['sudo', 'ip', 'link', 'delete', tap_name], 
                         stderr=subprocess.DEVNULL)
        except:
            pass
    
    def _signal_handler(self, signum, frame):
        """Handle system signals."""
        log.info(f"Received signal {signum}, shutting down...")
        self.running = False


def main():
    """Main entry point for QEMU runner."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Project Olympus ESP32 QEMU Runner')
    parser.add_argument('firmware', help='Path to ESP32 firmware ELF file')
    parser.add_argument('--node-id', type=int, default=0, help='Node ID for networking')
    parser.add_argument('--uart-port', type=int, default=3333, help='UART TCP port')
    parser.add_argument('--monitor-port', type=int, default=4444, help='QEMU monitor port')
    parser.add_argument('--debug', action='store_true', help='Enable debug output')
    parser.add_argument('--config', help='JSON configuration file')
    
    args = parser.parse_args()
    
    # Load configuration
    config = {
        'node_id': args.node_id,
        'uart_port': args.uart_port,
        'monitor_port': args.monitor_port,
        'debug': args.debug,
        'sensors': {
            'bme680': {'enabled': True, 'i2c_addr': 0x77},
            'ssd1306': {'enabled': True, 'i2c_addr': 0x3C},
            'battery_adc': {'enabled': True, 'adc_channel': 0}
        }
    }
    
    if args.config:
        import json
        with open(args.config, 'r') as f:
            config.update(json.load(f))
    
    # Create and run ESP32 simulation
    runner = ESP32QEMURunner(args.firmware, config)
    success = runner.run()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main() 