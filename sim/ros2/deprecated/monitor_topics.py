#!/usr/bin/env python3

import sys
import time
import subprocess
import argparse

def check_ros2_topic(topic_name, timeout_sec=30):
    """Check if a ROS2 topic exists and print info about it."""
    print(f"\nMonitoring for ROS2 topic: {topic_name}")
    print("=" * 50)
    
    start_time = time.time()
    found = False
    
    # Try until timeout
    while time.time() - start_time < timeout_sec:
        try:
            # Check topic list
            result = subprocess.run(
                ["ros2", "topic", "list"], 
                capture_output=True, 
                text=True,
                check=True
            )
            topics = result.stdout.strip().split("\n")
            
            print(f"Available ROS2 topics ({len(topics)}):")
            for t in topics:
                print(f"  - {t}")
                
            if topic_name in topics:
                found = True
                print(f"\nTopic {topic_name} FOUND!")
                
                # Get topic info
                info_result = subprocess.run(
                    ["ros2", "topic", "info", topic_name],
                    capture_output=True,
                    text=True
                )
                print(f"\nTopic info for {topic_name}:\n{info_result.stdout}")
                
                # Try to echo one message
                print(f"\nAttempting to echo one message from {topic_name}...")
                echo_result = subprocess.run(
                    ["timeout", "5", "ros2", "topic", "echo", "--once", topic_name],
                    capture_output=True,
                    text=True
                )
                
                if echo_result.returncode == 0:
                    print(f"Success! Received message:\n{echo_result.stdout}")
                else:
                    print(f"No message received within timeout")
                    
                break
            else:
                print(f"\nTopic {topic_name} NOT found yet...")
                
        except subprocess.CalledProcessError as e:
            print(f"Error running ROS2 commands: {e}")
        
        # Check for nodes
        try:
            node_result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True
            )
            print(f"\nActive ROS2 nodes:\n{node_result.stdout}")
        except subprocess.CalledProcessError:
            pass
            
        print("\nChecking again in 3 seconds...")
        time.sleep(3)
    
    if not found:
        print(f"\nTopic {topic_name} NOT found after {timeout_sec} seconds")
        return False
        
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monitor for a specific ROS2 topic")
    parser.add_argument("--topic", type=str, default="/mmwave/pointcloud", 
                        help="ROS2 topic to monitor")
    parser.add_argument("--timeout", type=int, default=30,
                        help="Timeout in seconds")
    
    args = parser.parse_args()
    
    check_ros2_topic(args.topic, args.timeout)
