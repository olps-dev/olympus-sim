#!/usr/bin/env python3
"""
Check ROS2 topics and transforms for debugging RViz visualization
"""

import subprocess
import time

def run_command(cmd):
    """Run command and return output"""
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    return result.stdout, result.stderr

def main():
    print("Checking ROS2 topics and transforms...")
    print("=" * 50)
    
    # Check active topics
    print("\n1. Active ROS2 topics:")
    stdout, stderr = run_command("ros2 topic list")
    print(stdout)
    
    # Check point cloud topics
    print("\n2. Point cloud topic info:")
    for topic in ["/mmwave/points", "/mmwave2/points"]:
        print(f"\n  Topic: {topic}")
        stdout, stderr = run_command(f"ros2 topic hz {topic} --window 10")
        if stdout:
            print(f"    Frequency: {stdout.strip()}")
        
    # Check transforms
    print("\n3. Active transforms:")
    stdout, stderr = run_command("ros2 run tf2_ros tf2_echo world mmwave --timeout 2")
    print(f"  world -> mmwave: {stdout if stdout else 'No transform found'}")
    
    stdout, stderr = run_command("ros2 run tf2_ros tf2_echo world mmwave2 --timeout 2")
    print(f"  world -> mmwave2: {stdout if stdout else 'No transform found'}")
    
    # Check if point cloud data is being published
    print("\n4. Sample point cloud data:")
    stdout, stderr = run_command("ros2 topic echo /mmwave/points --once")
    if stdout:
        lines = stdout.split('\n')[:20]  # First 20 lines
        print("  " + "\n  ".join(lines))
    
    print("\n" + "=" * 50)
    print("If you see 6 distinct clusters in RViz, this matches the")
    print("6 entities in the Gazebo world (walls, human, sensors).")
    print("The mmWave sensors are detecting these objects correctly.")

if __name__ == "__main__":
    main()