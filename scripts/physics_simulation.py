#!/usr/bin/env python3
"""
Project Olympus - Physics-Based Digital Twin Simulation
Real environmental physics simulation with correlated sensor responses.
"""

import time
import sys
import threading
import socket
import json
import logging
import math
import random
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional
import numpy as np

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
log = logging.getLogger('PhysicsSimulation')


@dataclass
class EnvironmentalState:
    """Current environmental conditions affecting all sensors"""
    time_of_day: float  # Hours since midnight
    outdoor_temp: float  # Celsius
    outdoor_humidity: float  # %
    wind_speed: float  # m/s
    air_pressure: float  # hPa
    pollution_level: float  # IAQ units
    human_occupancy: int  # Number of people
    hvac_running: bool
    window_open: bool


@dataclass
class RoomPhysics:
    """Physics model for apartment room"""
    volume: float = 150.0  # m³
    thermal_mass: float = 50000.0  # J/K
    air_exchange_rate: float = 0.5  # ACH
    base_temp: float = 22.0  # Celsius
    base_co2: float = 400.0  # ppm
    base_iaq: float = 50.0  # IAQ score
    heat_sources: List[float] = None  # Watts from devices/humans
    
    def __post_init__(self):
        if self.heat_sources is None:
            self.heat_sources = []


class PhysicsBasedSensor:
    """Base class for physics-accurate sensor simulation"""
    
    def __init__(self, sensor_id: str, position: tuple, room: RoomPhysics):
        self.sensor_id = sensor_id
        self.position = position  # (x, y, z) in meters
        self.room = room
        self.last_reading_time = time.time()
        self.measurement_noise = 0.1  # Sensor noise factor
        
    def get_local_conditions(self, env_state: EnvironmentalState) -> dict:
        """Calculate local environmental conditions based on position and physics"""
        # Base conditions affected by position
        distance_from_hvac = math.sqrt(self.position[0]**2 + self.position[1]**2)
        distance_from_window = abs(self.position[0] - 5.0)  # Window at x=5m
        
        # Temperature varies by position
        temp_variation = 0
        if env_state.hvac_running:
            temp_variation -= 0.5 * math.exp(-distance_from_hvac / 3.0)
        if env_state.window_open:
            window_effect = (env_state.outdoor_temp - self.room.base_temp) * 0.1
            temp_variation += window_effect * math.exp(-distance_from_window / 2.0)
        
        # Human heat sources
        for i in range(env_state.human_occupancy):
            human_pos = (2.0 + i * 1.5, 2.0, 1.0)  # Assumed positions
            distance = math.sqrt(sum((a-b)**2 for a, b in zip(self.position, human_pos)))
            temp_variation += 100.0 / (4 * math.pi * distance**2) * 0.01  # Heat dissipation
        
        return {
            'temperature': self.room.base_temp + temp_variation,
            'humidity': env_state.outdoor_humidity,
            'pressure': env_state.air_pressure,
        }


class BME680PhysicsSensor(PhysicsBasedSensor):
    """Physics-accurate BME680 environmental sensor"""
    
    def __init__(self, sensor_id: str, position: tuple, room: RoomPhysics):
        super().__init__(sensor_id, position, room)
        self.iaq_history = [50.0] * 10  # IAQ has momentum
        self.gas_baseline = 100000  # Gas resistance baseline
        
    def read(self, env_state: EnvironmentalState) -> dict:
        """Simulate realistic BME680 readings based on physics"""
        now = time.time()
        dt = now - self.last_reading_time
        self.last_reading_time = now
        
        local_conditions = self.get_local_conditions(env_state)
        
        # Temperature with thermal lag
        thermal_time_constant = 30.0  # seconds
        temp_response = 1 - math.exp(-dt / thermal_time_constant)
        temperature = local_conditions['temperature'] + random.gauss(0, 0.1)
        
        # Humidity with slower response
        humidity_time_constant = 60.0
        humidity_response = 1 - math.exp(-dt / humidity_time_constant)
        humidity = local_conditions['humidity'] + random.gauss(0, 0.5)
        humidity = max(20, min(80, humidity))
        
        # Pressure (relatively stable)
        pressure = local_conditions['pressure'] + random.gauss(0, 0.5)
        
        # Gas resistance affected by air quality
        base_resistance = self.gas_baseline
        pollution_factor = env_state.pollution_level / 100.0
        human_factor = env_state.human_occupancy * 0.1
        resistance = base_resistance * (1 - pollution_factor - human_factor)
        resistance += random.gauss(0, base_resistance * 0.02)
        
        # IAQ calculation (Bosch algorithm approximation)
        # Lower gas resistance = worse air quality = higher IAQ score
        if resistance > 0:
            raw_iaq = 500 - (resistance / base_resistance) * 400
        else:
            raw_iaq = 500
        
        # IAQ momentum (air quality changes slowly)
        self.iaq_history.append(raw_iaq)
        self.iaq_history = self.iaq_history[-10:]
        iaq = sum(self.iaq_history) / len(self.iaq_history)
        
        return {
            'temperature': round(temperature, 1),
            'humidity': round(humidity, 1),
            'pressure': round(pressure, 1),
            'gas_resistance': int(resistance),
            'iaq': round(max(0, min(500, iaq)), 1),
            'iaq_accuracy': 3,  # Stabilized
            'timestamp': now
        }


class BatteryPhysicsModel:
    """Physics-based battery discharge simulation"""
    
    def __init__(self, initial_voltage=3.7, capacity_mah=2500):
        self.nominal_voltage = initial_voltage
        self.capacity_mah = capacity_mah
        self.charge_consumed = 0.0  # mAh consumed
        self.temperature_effect = 0.0
        
    def get_voltage(self, current_ma: float, temperature_c: float, dt_seconds: float) -> float:
        """Calculate battery voltage based on physics"""
        # Discharge based on current draw
        self.charge_consumed += (current_ma * dt_seconds) / 3600.0
        
        # Discharge curve (LiPo approximation)
        discharge_percent = self.charge_consumed / self.capacity_mah
        
        if discharge_percent >= 1.0:
            return 2.8  # Cutoff voltage
        
        # Non-linear discharge curve
        if discharge_percent < 0.1:
            voltage_drop = discharge_percent * 0.1
        elif discharge_percent < 0.9:
            voltage_drop = 0.01 + (discharge_percent - 0.1) * 0.4
        else:
            voltage_drop = 0.33 + (discharge_percent - 0.9) * 3.0
        
        # Temperature effect
        temp_factor = 1.0 + (temperature_c - 25.0) * 0.001
        
        # Internal resistance effect
        ir_drop = current_ma * 0.1 / 1000.0  # 100mΩ internal resistance
        
        voltage = (self.nominal_voltage - voltage_drop) * temp_factor - ir_drop
        return max(2.8, voltage)


class NodeSimulation:
    """Complete ESP32 node with realistic sensor physics and power consumption"""
    
    def __init__(self, node_id: int, position: tuple, room: RoomPhysics):
        self.node_id = node_id
        self.position = position
        self.room = room
        
        # Hardware simulation
        self.bme680 = BME680PhysicsSensor(f"bme680_{node_id}", position, room)
        self.battery = BatteryPhysicsModel()
        
        # Power consumption model (ESP32 + sensors)
        self.base_current_ma = 80.0  # Active mode
        self.sleep_current_ma = 0.01  # Deep sleep
        self.sensor_read_current_ma = 150.0  # During measurement
        
        # Operational state
        self.is_sleeping = False
        self.last_sensor_read = 0
        self.sensor_interval = 10.0  # Read every 10 seconds
        
        # Motion detection (PIR simulation)
        self.motion_detected = False
        self.last_motion_time = 0
        
    def detect_motion(self, env_state: EnvironmentalState) -> bool:
        """Simulate PIR motion detection based on human occupancy"""
        if env_state.human_occupancy == 0:
            return False
        
        # Motion detection probability based on distance to humans
        detection_prob = 0.0
        for i in range(env_state.human_occupancy):
            human_pos = (2.0 + i * 1.5, 2.0, 1.0)
            distance = math.sqrt(sum((a-b)**2 for a, b in zip(self.position, human_pos)))
            
            # PIR range ~5m, decreasing sensitivity with distance
            if distance < 5.0:
                detection_prob += (5.0 - distance) / 5.0 * 0.1
        
        return random.random() < detection_prob
    
    def simulate_step(self, env_state: EnvironmentalState, dt: float) -> Dict:
        """Simulate one time step of the node operation"""
        now = time.time()
        data = {}
        
        # Determine current consumption
        current_ma = self.base_current_ma
        
        # Sensor reading cycle
        if now - self.last_sensor_read >= self.sensor_interval:
            self.last_sensor_read = now
            current_ma = self.sensor_read_current_ma
            
            # Read environmental sensors
            sensor_data = self.bme680.read(env_state)
            data['bme680'] = sensor_data
        
        # Motion detection
        motion = self.detect_motion(env_state)
        if motion and now - self.last_motion_time > 5.0:  # Debounce
            self.motion_detected = True
            self.last_motion_time = now
            data['motion'] = {
                'detected': True,
                'detection_count': 1,
                'timestamp': now
            }
        
        # Battery simulation
        temperature = data.get('bme680', {}).get('temperature', 25.0)
        voltage = self.battery.get_voltage(current_ma, temperature, dt)
        
        data['battery'] = {
            'voltage': round(voltage, 3),
            'current_ma': round(current_ma, 1),
            'charge_consumed_mah': round(self.battery.charge_consumed, 1),
            'estimated_hours_remaining': round((self.battery.capacity_mah - self.battery.charge_consumed) / current_ma, 1),
            'timestamp': now
        }
        
        return data


class ApartmentEnvironment:
    """Simulates the apartment environment with realistic day/night cycles"""
    
    def __init__(self):
        self.start_time = time.time()
        self.time_scale = 360  # 1 hour = 10 seconds real time
        
        # Base weather patterns
        self.base_outdoor_temp = 18.0
        self.weather_pattern = random.choice(['sunny', 'cloudy', 'rainy'])
        
        # Human activity patterns
        self.occupancy_schedule = self.generate_occupancy_schedule()
        
    def generate_occupancy_schedule(self) -> List[int]:
        """Generate realistic occupancy pattern for 24 hours"""
        schedule = []
        for hour in range(24):
            if 0 <= hour < 7:  # Night
                occupancy = 2 if random.random() < 0.9 else 1
            elif 7 <= hour < 9:  # Morning rush
                occupancy = random.choice([1, 2, 3])
            elif 9 <= hour < 17:  # Work hours
                occupancy = 0 if random.random() < 0.8 else 1
            elif 17 <= hour < 22:  # Evening
                occupancy = random.choice([1, 2, 3, 4])
            else:  # Late evening
                occupancy = 2 if random.random() < 0.7 else 1
            
            schedule.append(occupancy)
        return schedule
    
    def get_current_state(self) -> EnvironmentalState:
        """Calculate current environmental conditions"""
        sim_time = (time.time() - self.start_time) * self.time_scale
        time_of_day = (sim_time / 3600.0) % 24.0
        hour = int(time_of_day)
        
        # Outdoor temperature with day/night cycle
        temp_variation = 5.0 * math.sin((time_of_day - 6) * math.pi / 12.0)
        outdoor_temp = self.base_outdoor_temp + temp_variation
        
        # Weather effects
        if self.weather_pattern == 'rainy':
            outdoor_temp -= 3.0
            humidity = 75.0 + random.gauss(0, 5)
        elif self.weather_pattern == 'sunny':
            outdoor_temp += 2.0
            humidity = 45.0 + random.gauss(0, 5)
        else:  # cloudy
            humidity = 60.0 + random.gauss(0, 5)
        
        # Air pressure (stable with small variations)
        pressure = 1013.25 + random.gauss(0, 2)
        
        # Human occupancy from schedule
        occupancy = self.occupancy_schedule[hour]
        
        # HVAC system (simple thermostat)
        hvac_running = outdoor_temp > 25.0 or outdoor_temp < 18.0
        
        # Window opening probability (based on weather and time)
        window_prob = 0.3 if 8 <= hour <= 18 and self.weather_pattern == 'sunny' else 0.1
        window_open = random.random() < window_prob
        
        # Pollution (varies with traffic patterns and occupancy)
        base_pollution = 40.0
        traffic_factor = 20.0 * (0.5 + 0.5 * math.sin((hour - 8) * math.pi / 12.0))
        occupancy_factor = occupancy * 5.0
        pollution = base_pollution + traffic_factor + occupancy_factor + random.gauss(0, 5)
        
        return EnvironmentalState(
            time_of_day=time_of_day,
            outdoor_temp=outdoor_temp,
            outdoor_humidity=max(20, min(90, humidity)),
            wind_speed=random.uniform(0, 5),
            air_pressure=pressure,
            pollution_level=max(0, min(200, pollution)),
            human_occupancy=occupancy,
            hvac_running=hvac_running,
            window_open=window_open
        )


class OlympusPhysicsSimulation:
    """Main simulation orchestrator"""
    
    def __init__(self):
        self.environment = ApartmentEnvironment()
        self.room = RoomPhysics()
        
        # Create sensor nodes at realistic positions
        self.nodes = [
            NodeSimulation(0, (1.0, 1.0, 1.5), self.room),  # Living room corner
            NodeSimulation(1, (4.0, 1.0, 1.5), self.room),  # Living room center
            NodeSimulation(2, (7.0, 1.0, 1.5), self.room),  # Kitchen
            NodeSimulation(3, (1.0, 4.0, 1.5), self.room),  # Bedroom
            NodeSimulation(4, (7.0, 4.0, 1.5), self.room),  # Bathroom
        ]
        
        # MQTT connection
        self.mqtt_client = None
        self.uart_server = None
        self.running = False
        
        # Performance tracking
        self.last_kpi_calculation = time.time()
        self.sensor_readings = []
        self.motion_events = []
        
    def connect_mqtt(self):
        """Connect to MQTT broker"""
        try:
            import paho.mqtt.client as mqtt
            
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.connect('mqtt', 1883, 60)
            self.mqtt_client.loop_start()
            log.info("Connected to MQTT broker")
            return True
        except Exception as e:
            log.error(f"Failed to connect to MQTT: {e}")
            return False
    
    def start_uart_server(self):
        """Start UART simulation server"""
        def uart_handler():
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('0.0.0.0', 3333))
            server.listen(5)
            log.info("UART server listening on port 3333")
            
            while self.running:
                try:
                    client, addr = server.accept()
                    log.info(f"UART client connected: {addr}")
                    
                    while self.running:
                        try:
                            # Send simulated UART output
                            uart_data = self.generate_uart_output()
                            client.send(uart_data.encode('utf-8'))
                            time.sleep(2)
                        except:
                            break
                    
                    client.close()
                except:
                    continue
            
            server.close()
        
        threading.Thread(target=uart_handler, daemon=True).start()
    
    def generate_uart_output(self) -> str:
        """Generate realistic ESP32 UART output"""
        env_state = self.environment.get_current_state()
        
        output = []
        output.append(f"[{time.strftime('%H:%M:%S')}] Olympus Node Status")
        output.append(f"Environment: {env_state.time_of_day:.1f}h, {env_state.human_occupancy} occupants")
        output.append(f"Weather: T={env_state.outdoor_temp:.1f}°C, H={env_state.outdoor_humidity:.1f}%")
        
        for node in self.nodes:
            battery_v = node.battery.get_voltage(node.base_current_ma, 25.0, 1.0)
            output.append(f"Node{node.node_id}: BAT={battery_v:.2f}V, POS=({node.position[0]:.1f},{node.position[1]:.1f})")
        
        output.append("")
        return "\n".join(output) + "\n"
    
    def calculate_kpis(self):
        """Calculate system KPIs"""
        now = time.time()
        
        # Battery life projection
        avg_battery_hours = 0
        for node in self.nodes:
            remaining_mah = node.battery.capacity_mah - node.battery.charge_consumed
            hours_remaining = remaining_mah / node.base_current_ma
            avg_battery_hours += hours_remaining
        avg_battery_hours /= len(self.nodes)
        
        # Pipeline latency (sensor->MQTT->dashboard)
        sensor_to_mqtt_latency = 50  # ESP32 processing + WiFi
        mqtt_to_dashboard_latency = 20  # Network + processing
        dashboard_render_latency = 30  # UI update
        total_latency = sensor_to_mqtt_latency + mqtt_to_dashboard_latency + dashboard_render_latency
        
        # Network performance (simulate mesh)
        mesh_packet_loss = random.uniform(0.01, 0.05)  # 1-5% typical for good mesh
        
        # Detection performance
        recent_detections = len([e for e in self.motion_events if now - e < 60])
        
        kpis = {
            'projected_battery_hours': avg_battery_hours,
            'total_pipeline_latency_ms': total_latency,
            'mesh_packet_loss': mesh_packet_loss,
            'actor_detections_per_minute': recent_detections,
            'simulation_time': now - self.environment.start_time,
            'timestamp': now
        }
        
        # Send to MQTT
        if self.mqtt_client:
            self.mqtt_client.publish('olympus/metrics/kpis', json.dumps(kpis))
        
        return kpis
    
    def run(self):
        """Main simulation loop"""
        log.info("Starting Olympus Physics Simulation")
        
        if not self.connect_mqtt():
            log.error("Cannot continue without MQTT connection")
            return
        
        self.running = True
        self.start_uart_server()
        
        last_time = time.time()
        
        try:
            while self.running:
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time
                
                # Get current environment
                env_state = self.environment.get_current_state()
                
                # Simulate all nodes
                for node in self.nodes:
                    node_data = node.simulate_step(env_state, dt)
                    
                    # Publish sensor data
                    for sensor_type, sensor_data in node_data.items():
                        topic = f'olympus/sensors/{node.node_id}/{sensor_type}'
                        
                        if self.mqtt_client:
                            self.mqtt_client.publish(topic, json.dumps(sensor_data))
                        
                        # Track for KPI calculation
                        if sensor_type == 'motion' and sensor_data.get('detected'):
                            self.motion_events.append(current_time)
                            # Keep only last 5 minutes of events
                            self.motion_events = [t for t in self.motion_events if current_time - t < 300]
                
                # Calculate and publish KPIs every 10 seconds
                if current_time - self.last_kpi_calculation >= 10:
                    self.calculate_kpis()
                    self.last_kpi_calculation = current_time
                
                # Simulation step rate (10 Hz)
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            log.info("Simulation stopped by user")
        finally:
            self.running = False
            if self.mqtt_client:
                self.mqtt_client.disconnect()


if __name__ == "__main__":
    simulation = OlympusPhysicsSimulation()
    simulation.run() 