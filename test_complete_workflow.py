#!/usr/bin/env python3

import requests
import json
import time

BASE_URL = 'http://localhost:3001'

def test_complete_workflow():
    """Test the complete workflow: room -> node -> sensor"""
    
    print("🧪 Testing Complete Olympus Workflow...")
    
    try:
        # 1. Check initial state
        print("\n1️⃣ Checking initial state...")
        response = requests.get(f'{BASE_URL}/simulation_state')
        initial_state = response.json()
        print(f"   Initial: {len(initial_state.get('nodes', []))} nodes, {len(initial_state.get('sensors', []))} sensors, {len(initial_state.get('rooms', []))} rooms")
        
        # 2. Create a room
        print("\n2️⃣ Creating a test room...")
        room_data = {
            'name': 'Test Lab',
            'dimensions': {'width': 8, 'height': 3, 'depth': 6}
        }
        response = requests.post(f'{BASE_URL}/create_room', 
                               json=room_data,
                               headers={'Content-Type': 'application/json'})
        
        if response.status_code == 200:
            result = response.json()
            if result.get('success'):
                room_id = result.get('room', {}).get('id')
                print(f"   ✅ Room created: {room_id}")
                print(f"   📊 State now has {len(result.get('state', {}).get('rooms', []))} rooms")
            else:
                print("   ❌ Room creation failed")
                return
        else:
            print(f"   ❌ Room request failed: {response.status_code}")
            return
            
        # 3. Create a Zeus node
        print("\n3️⃣ Creating Zeus node...")
        node_data = {
            'type': 'zeus',
            'room': 'Test Lab',
            'position': {'x': 2, 'y': 1, 'z': 3}
        }
        response = requests.post(f'{BASE_URL}/create_node',
                               json=node_data,
                               headers={'Content-Type': 'application/json'})
        
        if response.status_code == 200:
            result = response.json()
            if result.get('success'):
                node_id = result.get('node', {}).get('id')
                print(f"   ✅ Zeus node created: {node_id}")
                print(f"   📊 State now has {len(result.get('state', {}).get('nodes', []))} nodes")
            else:
                print("   ❌ Node creation failed")
                return
        else:
            print(f"   ❌ Node request failed: {response.status_code}")
            return
            
        # 4. Create multiple sensors
        sensors_to_create = [
            {'type': 'mmwave', 'name': 'mmWave Radar'},
            {'type': 'temperature', 'name': 'Temperature Sensor'},
            {'type': 'camera', 'name': 'Security Camera'}
        ]
        
        for i, sensor_info in enumerate(sensors_to_create, 1):
            print(f"\n4️⃣.{i} Creating {sensor_info['name']}...")
            sensor_data = {
                'type': sensor_info['type'],
                'nodeId': node_id,
                'specifications': {'updateRate': 10, 'range': 5}
            }
            response = requests.post(f'{BASE_URL}/create_sensor',
                                   json=sensor_data,
                                   headers={'Content-Type': 'application/json'})
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success'):
                    sensor_id = result.get('sensor', {}).get('id')
                    print(f"   ✅ {sensor_info['name']} created: {sensor_id}")
                    print(f"   📊 State now has {len(result.get('state', {}).get('sensors', []))} sensors")
                else:
                    print(f"   ❌ {sensor_info['name']} creation failed")
            else:
                print(f"   ❌ {sensor_info['name']} request failed: {response.status_code}")
        
        # 5. Start simulation
        print("\n5️⃣ Starting simulation...")
        response = requests.post(f'{BASE_URL}/start_simulation',
                               json={},
                               headers={'Content-Type': 'application/json'})
        
        if response.status_code == 200:
            result = response.json()
            if result.get('success'):
                is_running = result.get('state', {}).get('isRunning', False)
                print(f"   ✅ Simulation started. Running: {is_running}")
            else:
                print("   ❌ Simulation start failed")
        else:
            print(f"   ❌ Simulation start request failed: {response.status_code}")
        
        # 6. Check final state
        print("\n6️⃣ Final state check...")
        response = requests.get(f'{BASE_URL}/simulation_state')
        if response.status_code == 200:
            final_state = response.json()
            print(f"   📊 Final state:")
            print(f"      • Rooms: {len(final_state.get('rooms', []))}")
            print(f"      • Nodes: {len(final_state.get('nodes', []))}")
            print(f"      • Sensors: {len(final_state.get('sensors', []))}")
            print(f"      • Running: {final_state.get('isRunning', False)}")
            
            # Show detailed info
            for room in final_state.get('rooms', []):
                print(f"      🏠 Room: {room['name']} ({room['id']})")
                
            for node in final_state.get('nodes', []):
                print(f"      🖥️  Node: {node['name']} ({node['type']}) in {node['room']}")
                print(f"           Sensors: {len(node.get('sensors', []))}")
                
            for sensor in final_state.get('sensors', []):
                print(f"      📡 Sensor: {sensor['name']} ({sensor['type']}) on {sensor['nodeId']}")
        
        print("\n🎉 Complete workflow test finished!")
        print("\n💡 Now try starting the React dashboard with: ./start_full_dashboard.sh")
        print("   Then create nodes/sensors through the UI to see them appear in 3D!")
        
    except requests.exceptions.ConnectionError:
        print("❌ Could not connect to backend. Start it first with: python3 simple_backend.py")
    except Exception as e:
        print(f"❌ Test failed with error: {e}")

if __name__ == '__main__':
    test_complete_workflow()