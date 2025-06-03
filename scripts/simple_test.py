#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import json
import time
import random

client = mqtt.Client()
client.connect('mqtt', 1883, 60)
client.loop_start()

print('✅ Connected to MQTT - Publishing test data...')
print('📊 Check IRIS Dashboard: http://localhost:8080')

for i in range(20):
    for node_id in range(5):
        temp = 22.0 + random.uniform(-2, 2)
        data = {
            'temperature': round(temp, 1),
            'iaq': round(55 + random.uniform(-5, 5), 1),
            'timestamp': time.time(),
            'node_id': node_id
        }
        client.publish(f'olympus/sensors/{node_id}/bme680', json.dumps(data))
        print(f'📤 Node{node_id}: T={temp:.1f}°C')
    time.sleep(2)

print('✅ Test complete!') 