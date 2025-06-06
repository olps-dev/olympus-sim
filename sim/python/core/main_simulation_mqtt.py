import time
import random
import argparse

from sim.python.olympus_nodes import Node
from sim.python.mqtt_real import MQTTClientAdapter
from sim.python.hardware_sensor_bridge import HardwareSensorBridge

# Simulation Configuration
NUM_MID_TIER_NODES = 2
NUM_SENSORS_PER_MID_TIER = 3
SIMULATION_DURATION_SECONDS = 60  # Run for 60 seconds by default
SIMULATION_TICK_INTERVAL = 1.0  # Seconds per tick

def main(broker_host="localhost", broker_port=1883, duration=SIMULATION_DURATION_SECONDS):
    print(f"[{time.strftime('%H:%M:%S')}] Olympus Simulation Starting with Real MQTT...")
    print(f"[{time.strftime('%H:%M:%S')}] Connecting to MQTT broker at {broker_host}:{broker_port}")
    print("-" * 50)

    # 1. Create Network Components
    #    - Zeus Node
    #    - Mid-Tier Nodes
    #    - Sensor Nodes

    # Zeus Layer
    zeus_node = Node(node_id="zeus-1", node_type="zeus")
    zeus_mqtt = MQTTClientAdapter(client_id_node=zeus_node, broker_host=broker_host, broker_port=broker_port)
    zeus_node.set_mqtt_client(zeus_mqtt)

    mid_tier_nodes = []
    sensor_nodes = []

    # Mid-Tier and Sensor Layers
    for i in range(NUM_MID_TIER_NODES):
        mid_tier_id = f"mid-tier-{i+1}"
        mid_tier_node = Node(node_id=mid_tier_id, node_type="mid_tier")
        mid_tier_mqtt = MQTTClientAdapter(client_id_node=mid_tier_node, broker_host=broker_host, broker_port=broker_port)
        mid_tier_node.set_mqtt_client(mid_tier_mqtt)
        
        # Mid-tier subscribes to data from its sensors
        # Example: mid-tier-1 will get sensor_data/sensor-1/bme680, sensor_data/sensor-1/battery etc.
        zeus_node.add_child_node(mid_tier_node)  # Zeus is parent of mid-tier
        mid_tier_nodes.append(mid_tier_node)

        for j in range(NUM_SENSORS_PER_MID_TIER):
            sensor_id = f"sensor-{i*NUM_SENSORS_PER_MID_TIER + j + 1}"
            # Create sensor node
            sensor_node = Node(node_id=sensor_id, node_type="sensor_node")
            
            # Create MQTT client for the sensor node
            sensor_mqtt = MQTTClientAdapter(client_id_node=sensor_node, broker_host=broker_host, broker_port=broker_port)
            sensor_node.set_mqtt_client(sensor_mqtt)
            
            # Create and assign HardwareSensorBridge to the sensor node
            # The bridge uses the sensor's MQTT client to publish data
            hardware_bridge = HardwareSensorBridge(mqtt_client_sim=sensor_mqtt, node_id=sensor_id)
            sensor_node.set_hardware_bridge(hardware_bridge)

            mid_tier_node.add_child_node(sensor_node)  # Mid-tier is parent of sensor
            sensor_nodes.append(sensor_node)

            # Mid-tier node subscribes to topics from its sensors on the MQTT broker
            mid_tier_mqtt.subscribe(f"olympus/sensor_data/{sensor_id}/bme680")
            mid_tier_mqtt.subscribe(f"olympus/sensor_data/{sensor_id}/battery")
            mid_tier_mqtt.subscribe(f"olympus/sensor_data/{sensor_id}/display_status")
    
    # Zeus node subscribes to data from mid-tier nodes
    for mt_node in mid_tier_nodes:
        zeus_mqtt.subscribe(f"olympus/processed_mid_tier/{mt_node.node_id}")
        zeus_mqtt.subscribe(f"olympus/sensor_data/{mt_node.node_id}/#")  # If mid-tiers forward raw data too

    print("-" * 50)
    print(f"[{time.strftime('%H:%M:%S')}] Simulation Setup Complete. Starting main loop for {duration}s...")
    print("-" * 50)

    # 2. Simulation Loop
    start_time = time.time()
    current_tick = 0
    try:
        while (time.time() - start_time) < duration:
            current_tick_time = time.time()
            print(f"\n[{time.strftime('%H:%M:%S')}] ----- Simulation Tick: {current_tick + 1} -----")

            # Tick all nodes (Sensor nodes will use their bridge to publish)
            for node in sensor_nodes + mid_tier_nodes + [zeus_node]:
                node.tick()
            
            current_tick += 1
            # Wait for the next tick, accounting for processing time
            time_to_sleep = SIMULATION_TICK_INTERVAL - (time.time() - current_tick_time)
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)
            else:
                print(f"[{time.strftime('%H:%M:%S')}] Warning: Tick processing time ({-(time_to_sleep*1000):.2f}ms) exceeded interval {SIMULATION_TICK_INTERVAL*1000:.0f}ms.")

    except KeyboardInterrupt:
        print(f"\n[{time.strftime('%H:%M:%S')}] Simulation interrupted by user.")
    finally:
        print("-" * 50)
        print(f"[{time.strftime('%H:%M:%S')}] Simulation Finished after {current_tick} ticks.")
        print("-" * 50)
        
        # Clean up MQTT connections
        print(f"[{time.strftime('%H:%M:%S')}] Disconnecting MQTT clients...")
        # Disconnecting clients in reverse order
        for node in sensor_nodes + mid_tier_nodes + [zeus_node]:
            if hasattr(node, 'mqtt_client') and node.mqtt_client is not None:
                node.mqtt_client.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Olympus Simulation with MQTT')
    parser.add_argument('--host', default='localhost', help='MQTT broker hostname')
    parser.add_argument('--port', type=int, default=1883, help='MQTT broker port')
    parser.add_argument('--duration', type=int, default=SIMULATION_DURATION_SECONDS, help='Simulation duration in seconds')
    
    args = parser.parse_args()
    main(broker_host=args.host, broker_port=args.port, duration=args.duration)
