#!/usr/bin/env python3
"""
Interactive scene manipulation for Olympus mmWave simulation
Allows moving objects and testing sensor response without GUI
"""

import subprocess
import time
import sys

class SceneManipulator:
    def __init__(self):
        print("Scene Manipulator initialized")
        
    def move_entity(self, entity_name, x, y, z, roll=0, pitch=0, yaw=0):
        """Move an entity to a new position using Gazebo transport"""
        cmd = [
            'gz', 'service', '-s', '/world/mmwave_test/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', f'name: "{entity_name}" position: {{x: {x}, y: {y}, z: {z}}} orientation: {{x: {roll}, y: {pitch}, z: {yaw}, w: 1.0}}'
        ]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print(f'Moved {entity_name} to ({x}, {y}, {z})')
                return True
            else:
                print(f'Failed to move {entity_name}: {result.stderr}')
                return False
        except subprocess.TimeoutExpired:
            print(f'⏱Timeout moving {entity_name}')
            return False
        except Exception as e:
            print(f'Error moving {entity_name}: {e}')
            return False
    
    def list_entities(self):
        """List all entities in the world"""
        cmd = ['gz', 'model', '-l']
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                entities = result.stdout.strip().split('\n')
                print('Available entities:')
                for entity in entities:
                    if entity.strip():
                        print(f'  - {entity.strip()}')
                return entities
            else:
                print(f'Failed to list entities: {result.stderr}')
                return []
        except Exception as e:
            print(f'Error listing entities: {e}')
            return []

def main():
    manipulator = SceneManipulator()
    
    print("\n=== Olympus mmWave Scene Manipulator ===")
    print("This tool allows you to move objects and test the mmWave sensor response")
    print(" Current object positions:")
    print("  - box_obstacle_1: (2, 2, 0.5)")
    print("  - box_obstacle_2: (-2, 3, 0.75)")
    print("  - cylinder_obstacle: (3, -2, 0.5)")
    print("  - mmwave_sensor: (0, 0, 1)")
    print("\nAvailable commands:")
    print("  list - List all entities in the scene")
    print("  move <entity> <x> <y> <z> - Move entity to position")
    print("  test - Run predefined test scenarios")
    print("  quit - Exit")
    print()
    
    # List initial entities
    manipulator.list_entities()
    
    while True:
        try:
            command = input("\n🎮 Enter command: ").strip().split()
            
            if not command:
                continue
                
            if command[0] == 'quit':
                break
                
            elif command[0] == 'list':
                manipulator.list_entities()
                
            elif command[0] == 'move' and len(command) >= 5:
                entity = command[1]
                x, y, z = float(command[2]), float(command[3]), float(command[4])
                manipulator.move_entity(entity, x, y, z)
                print("Check RViz2 to see how the mmWave sensor response changes!")
                
            elif command[0] == 'test':
                print("Running test scenarios...")
                
                # Test 1: Move box closer to sensor
                print("\nTest 1: Moving box_obstacle_1 closer to sensor")
                manipulator.move_entity('box_obstacle_1', 1.0, 1.0, 0.5)
                time.sleep(2)
                
                # Test 2: Move cylinder to different position
                print("\nTest 2: Moving cylinder_obstacle to new position")
                manipulator.move_entity('cylinder_obstacle', -1.5, -1.5, 0.5)
                time.sleep(2)
                
                # Test 3: Move box to sensor's path
                print("\nTest 3: Moving box_obstacle_2 directly in front of sensor")
                manipulator.move_entity('box_obstacle_2', 0.5, 0.0, 0.75)
                time.sleep(2)
                
                print("Test scenarios complete! Check RViz2 to see sensor response changes.")
                
            else:
                print("Invalid command. Use: list, move <entity> <x> <y> <z>, test, or quit")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    print("👋 Exiting scene manipulator...")

if __name__ == '__main__':
    main()
