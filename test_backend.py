#!/usr/bin/env python3

import socketio
import time

# Create a Socket.IO client
sio = socketio.SimpleClient()

try:
    print("Connecting to backend bridge...")
    sio.connect('http://localhost:3001')
    print("✅ Connected successfully!")
    
    # Test getting simulation state
    print("\n📊 Requesting simulation state...")
    sio.emit('get_simulation_state')
    
    # Wait for response
    time.sleep(1)
    
    # Test creating a room
    print("\n🏠 Creating test room...")
    room_config = {
        'name': 'Test Room',
        'dimensions': {'width': 5, 'height': 3, 'depth': 4}
    }
    sio.emit('create_room', room_config)
    time.sleep(1)
    
    # Test creating a Zeus node
    print("\n🖥️ Creating Zeus node...")
    node_config = {
        'type': 'zeus',
        'room': 'Test Room',
        'position': {'x': 0, 'y': 1, 'z': 0}
    }
    sio.emit('create_node', node_config)
    time.sleep(1)
    
    print("\n✅ Backend test completed successfully!")
    
except Exception as e:
    print(f"❌ Backend test failed: {e}")

finally:
    sio.disconnect()
    print("🔌 Disconnected from backend")