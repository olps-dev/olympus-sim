#!/usr/bin/env python3

import requests
import json
import time

BASE_URL = 'http://localhost:3001'

def test_backend():
    """Test the simple backend API"""
    
    print("🧪 Testing Olympus Simple Backend...")
    
    try:
        # Test health check
        print("\n1️⃣ Testing health check...")
        response = requests.get(f'{BASE_URL}/health')
        if response.status_code == 200:
            print("✅ Health check passed")
        else:
            print(f"❌ Health check failed: {response.status_code}")
            return
            
        # Test initial state
        print("\n2️⃣ Testing initial simulation state...")
        response = requests.get(f'{BASE_URL}/simulation_state')
        if response.status_code == 200:
            state = response.json()
            print(f"✅ Got simulation state: {len(state.get('nodes', []))} nodes, {len(state.get('sensors', []))} sensors")
        else:
            print(f"❌ Failed to get simulation state: {response.status_code}")
            return
            
        # Test room creation
        print("\n3️⃣ Testing room creation...")
        room_data = {
            'name': 'Test Room',
            'dimensions': {'width': 5, 'height': 3, 'depth': 4}
        }
        response = requests.post(f'{BASE_URL}/create_room', 
                               json=room_data,
                               headers={'Content-Type': 'application/json'})
        if response.status_code == 200:
            result = response.json()
            if result.get('success'):
                print("✅ Room created successfully")
            else:
                print("❌ Room creation failed")
        else:
            print(f"❌ Room creation request failed: {response.status_code}")
            
        # Test node creation
        print("\n4️⃣ Testing Zeus node creation...")
        node_data = {
            'type': 'zeus',
            'room': 'Test Room',
            'position': {'x': 0, 'y': 1, 'z': 0}
        }
        response = requests.post(f'{BASE_URL}/create_node',
                               json=node_data,
                               headers={'Content-Type': 'application/json'})
        if response.status_code == 200:
            result = response.json()
            if result.get('success'):
                node_id = result.get('node', {}).get('id')
                print(f"✅ Zeus node created successfully: {node_id}")
                
                # Test sensor creation
                print("\n5️⃣ Testing mmWave sensor creation...")
                sensor_data = {
                    'type': 'mmwave',
                    'nodeId': node_id,
                    'specifications': {'range': 8, 'fov': 60, 'updateRate': 10}
                }
                response = requests.post(f'{BASE_URL}/create_sensor',
                                       json=sensor_data,
                                       headers={'Content-Type': 'application/json'})
                if response.status_code == 200:
                    result = response.json()
                    if result.get('success'):
                        print("✅ mmWave sensor created successfully")
                    else:
                        print("❌ Sensor creation failed")
                else:
                    print(f"❌ Sensor creation request failed: {response.status_code}")
            else:
                print("❌ Node creation failed")
        else:
            print(f"❌ Node creation request failed: {response.status_code}")
            
        # Test simulation start
        print("\n6️⃣ Testing simulation start...")
        response = requests.post(f'{BASE_URL}/start_simulation',
                               json={},
                               headers={'Content-Type': 'application/json'})
        if response.status_code == 200:
            result = response.json()
            if result.get('success'):
                print("✅ Simulation started successfully")
            else:
                print("❌ Simulation start failed")
        else:
            print(f"❌ Simulation start request failed: {response.status_code}")
            
        # Check final state
        print("\n7️⃣ Testing final simulation state...")
        response = requests.get(f'{BASE_URL}/simulation_state')
        if response.status_code == 200:
            state = response.json()
            print(f"✅ Final state: {len(state.get('nodes', []))} nodes, {len(state.get('sensors', []))} sensors, running: {state.get('isRunning', False)}")
        else:
            print(f"❌ Failed to get final simulation state: {response.status_code}")
            
        print("\n🎉 Backend test completed successfully!")
        
    except requests.exceptions.ConnectionError:
        print("❌ Could not connect to backend. Make sure it's running on port 3001.")
    except Exception as e:
        print(f"❌ Test failed with error: {e}")

if __name__ == '__main__':
    test_backend()