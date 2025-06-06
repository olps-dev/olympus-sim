import time
from collections import defaultdict

class MQTTBrokerSim:
    """Simulates an MQTT Broker for inter-node communication."""
    def __init__(self, broker_id, owner_node=None, network_simulator=None):
        self.broker_id = broker_id
        self.owner_node = owner_node # The Node instance that owns this broker (Zeus or Mid-Tier)
        self.network_simulator = network_simulator # Instance of NetworkSimulator
        self.topics = defaultdict(list)  # topic -> list of (client_id, qos, callback_node)
        self.retained_messages = {}
        self.message_queue = [] # (publisher_id, topic, message, qos)
        print(f"[{time.strftime('%H:%M:%S')}] MQTT Broker {self.broker_id} created.")

    def subscribe(self, client_node, topic, qos=0):
        # client_node is the Node instance that is subscribing
        if client_node not in [sub[0] for sub in self.topics[topic]]:
            self.topics[topic].append((client_node, qos))
            print(f"[{time.strftime('%H:%M:%S')}] Broker {self.broker_id}: Node {client_node.node_id} subscribed to '{topic}' with QoS {qos}")
            # Send retained message if any
            if topic in self.retained_messages:
                retained_msg = self.retained_messages[topic]
                print(f"[{time.strftime('%H:%M:%S')}] Broker {self.broker_id}: Sending retained message on '{topic}' to {client_node.node_id}")
                # Simulate network delay if network_simulator is present
                if self.network_simulator:
                    self.network_simulator.simulate_transmission(self.owner_node, client_node, retained_msg['message'])
                client_node.receive_message(topic, retained_msg['message'], retained_msg['publisher_id'])
        else:
            print(f"[{time.strftime('%H:%M:%S')}] Broker {self.broker_id}: Node {client_node.node_id} already subscribed to '{topic}'.")

    def unsubscribe(self, client_node, topic):
        self.topics[topic] = [(sub_node, q) for sub_node, q in self.topics[topic] if sub_node != client_node]
        print(f"[{time.strftime('%H:%M:%S')}] Broker {self.broker_id}: Node {client_node.node_id} unsubscribed from '{topic}'")

    def publish(self, publisher_node_id, topic, message, qos=0, retain=False):
        print(f"[{time.strftime('%H:%M:%S')}] Broker {self.broker_id}: Received publish from {publisher_node_id} on topic '{topic}': {message}")
        self.message_queue.append({'publisher_id': publisher_node_id, 'topic': topic, 'message': message, 'qos': qos, 'retain': retain})
        if retain:
            self.retained_messages[topic] = {'publisher_id': publisher_node_id, 'message': message, 'qos': qos}
            print(f"[{time.strftime('%H:%M:%S')}] Broker {self.broker_id}: Retained message on topic '{topic}'")
        # Process queue immediately for simplicity in this simulation step
        self.tick()

    def tick(self):
        """Processes the message queue, delivering messages to subscribers."""
        if not self.message_queue:
            return

        queued_message = self.message_queue.pop(0)
        topic = queued_message['topic']
        message = queued_message['message']
        publisher_id = queued_message['publisher_id']
        # qos = queued_message['qos'] # QoS handling can be expanded later

        subscribers_to_topic = self.topics.get(topic, [])
        if not subscribers_to_topic:
            print(f"[{time.strftime('%H:%M:%S')}] Broker {self.broker_id}: No subscribers for topic '{topic}', message from {publisher_id} dropped.")
            return

        for subscriber_node, sub_qos in subscribers_to_topic:
            print(f"[{time.strftime('%H:%M:%S')}] Broker {self.broker_id}: Forwarding message on '{topic}' from {publisher_id} to {subscriber_node.node_id}")
            # Simulate network delay/conditions if network_simulator is present
            # The owner_node of the broker is the source for this transmission leg
            if self.network_simulator and self.owner_node:
                self.network_simulator.simulate_transmission(self.owner_node, subscriber_node, message)
            
            subscriber_node.receive_message(topic, message, publisher_id)

class MQTTSimulatedClient:
    """A simple MQTT client for a Node to interact with a MQTTBrokerSim."""
    def __init__(self, client_id_node, broker_sim):
        self.client_id_node = client_id_node # This is the Node instance
        self.broker = broker_sim
        print(f"[{time.strftime('%H:%M:%S')}] MQTT Client for Node {self.client_id_node.node_id} connected to Broker {self.broker.broker_id}")

    def subscribe(self, topic, qos=0):
        self.broker.subscribe(self.client_id_node, topic, qos)

    def publish(self, topic, message, qos=0, retain=False):
        self.broker.publish(self.client_id_node.node_id, topic, message, qos, retain)

    def unsubscribe(self, topic):
        self.broker.unsubscribe(self.client_id_node, topic)
