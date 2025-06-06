import time
import random

from ..olympus_nodes import Node
from ..mqtt_broker_sim import MQTTBrokerSim, MQTTSimulatedClient
from ..hardware_sensor_bridge import HardwareSensorBridge

# Simulation Configuration
NUM_MID_TIER_NODES = 2
NUM_SENSORS_PER_MID_TIER = 3
SIMULATION_DURATION_SECONDS = 10 # Run for 10 seconds
SIMULATION_TICK_INTERVAL = 1.0 # Seconds per tick

def main():
    print(f"[{time.strftime('%H:%M:%S')}] Olympus Simulation Starting...")
    print("-" * 50)

    # 1. Create Network Components
    #    - Zeus Node and its Broker
    #    - Mid-Tier Nodes and their Brokers
    #    - Sensor Nodes

    # Zeus Layer
    zeus_node = Node(node_id="zeus-1", node_type="zeus")
    zeus_broker = MQTTBrokerSim(broker_id="zeus-broker-1", owner_node=zeus_node)
    zeus_node.set_local_broker(zeus_broker)

    mid_tier_nodes = []
    sensor_nodes = []

    # Mid-Tier and Sensor Layers
    for i in range(NUM_MID_TIER_NODES):
        mid_tier_id = f"mid-tier-{i+1}"
        mid_tier_node = Node(node_id=mid_tier_id, node_type="mid_tier", parent_broker=zeus_broker)
        mid_tier_broker = MQTTBrokerSim(broker_id=f"{mid_tier_id}-broker", owner_node=mid_tier_node)
        mid_tier_node.set_local_broker(mid_tier_broker)
        
        # Mid-tier subscribes to its own local broker for data from its sensors
        # And also subscribes to the Zeus broker if it expects commands (not shown here)
        mid_tier_mqtt_client_for_zeus = MQTTSimulatedClient(client_id_node=mid_tier_node, broker_sim=zeus_broker)
        # mid_tier_node.parent_broker is already zeus_broker, client is for publishing upwards

        zeus_node.add_child_node(mid_tier_node) # Zeus is parent of mid-tier
        mid_tier_nodes.append(mid_tier_node)

        for j in range(NUM_SENSORS_PER_MID_TIER):
            sensor_id = f"sensor-{i*NUM_SENSORS_PER_MID_TIER + j + 1}"
            # Sensor node's parent broker is the mid-tier broker it reports to
            sensor_node = Node(node_id=sensor_id, node_type="sensor_node", parent_broker=mid_tier_broker)
            
            # Each sensor node gets an MQTT client to publish to its mid-tier broker
            sensor_mqtt_client = MQTTSimulatedClient(client_id_node=sensor_node, broker_sim=mid_tier_broker)
            
            # Create and assign HardwareSensorBridge to the sensor node
            # The bridge uses the sensor's MQTT client to publish data
            hardware_bridge = HardwareSensorBridge(node_id=sensor_id, mqtt_client_sim=sensor_mqtt_client)
            sensor_node.set_hardware_bridge(hardware_bridge)

            mid_tier_node.add_child_node(sensor_node) # Mid-tier is parent of sensor
            sensor_nodes.append(sensor_node)

            # Mid-tier node needs to subscribe to topics from its sensors on its local broker
            # Example: mid-tier-1-broker will get sensor_data/sensor-1/bme680, sensor_data/sensor-1/battery etc.
            mid_tier_broker.subscribe(client_node=mid_tier_node, topic=f"sensor_data/{sensor_id}/bme680")
            mid_tier_broker.subscribe(client_node=mid_tier_node, topic=f"sensor_data/{sensor_id}/battery")
            mid_tier_broker.subscribe(client_node=mid_tier_node, topic=f"sensor_data/{sensor_id}/display_status")
    
    # Zeus node subscribes to data from mid-tier nodes on its broker
    for mt_node in mid_tier_nodes:
        zeus_broker.subscribe(client_node=zeus_node, topic=f"processed_mid_tier/{mt_node.node_id}")
        zeus_broker.subscribe(client_node=zeus_node, topic=f"sensor_data/{mt_node.node_id}/#") # If mid-tiers forward raw data too

    print("-" * 50)
    print(f"[{time.strftime('%H:%M:%S')}] Simulation Setup Complete. Starting main loop for {SIMULATION_DURATION_SECONDS}s...")
    print("-" * 50)

    # 2. Simulation Loop
    start_time = time.time()
    current_tick = 0
    try:
        while (time.time() - start_time) < SIMULATION_DURATION_SECONDS:
            current_tick_time = time.time()
            print(f"\n[{time.strftime('%H:%M:%S')}] ----- Simulation Tick: {current_tick + 1} -----")

            # Tick all nodes (Sensor nodes will use their bridge to publish)
            for node in sensor_nodes + mid_tier_nodes + [zeus_node]:
                node.tick()
            
            # Tick all brokers (they process their message queues)
            # Brokers are ticked by their owner nodes if they have specific logic,
            # but their main message processing happens on publish or their own tick if needed.
            # For this setup, MQTTBrokerSim.publish calls tick() internally, so explicit broker ticking might be redundant
            # unless brokers have other periodic tasks.
            # zeus_broker.tick() # Already called by zeus_node.tick() if broker has tasks
            # for mt_node in mid_tier_nodes:
            #     if mt_node.local_mqtt_broker:
            #         mt_node.local_mqtt_broker.tick()

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

if __name__ == "__main__":
    main()
