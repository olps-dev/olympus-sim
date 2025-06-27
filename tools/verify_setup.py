#!/usr/bin/env python3
"""
Olympus Simulation Setup Verification
Checks that all components are properly organized and accessible
"""

import os
import sys
from pathlib import Path

def check_file(path, description):
    """Check if a file exists and is accessible"""
    if path.exists():
        print(f"✅ {description}: {path}")
        return True
    else:
        print(f"❌ {description}: {path} (NOT FOUND)")
        return False

def check_directory(path, description):
    """Check if a directory exists"""
    if path.exists() and path.is_dir():
        print(f"✅ {description}: {path}")
        return True
    else:
        print(f"❌ {description}: {path} (NOT FOUND)")
        return False

def main():
    print("🔍 Olympus Simulation Setup Verification\n")
    
    project_root = Path(__file__).parent
    all_good = True
    
    print("📁 Directory Structure:")
    dirs_to_check = [
        (project_root / "automation", "Automation directory"),
        (project_root / "tests", "Tests directory"),
        (project_root / "tools", "Tools directory"),
        (project_root / "legacy_launch", "Legacy launch directory"),
        (project_root / "sim" / "gazebo" / "plugins", "Gazebo plugins"),
        (project_root / "sim" / "ros2" / "launch", "ROS2 launch files"),
    ]
    
    for path, desc in dirs_to_check:
        all_good &= check_directory(path, desc)
    
    print("\n🔧 Core Components:")
    files_to_check = [
        (project_root / "olympus", "Main launcher wrapper"),
        (project_root / "tools" / "launch_olympus.py", "Python launcher"),
        (project_root / "automation" / "automation_demo.py", "Automation controller"),
        (project_root / "tests" / "test_automation_loop.py", "Automation test"),
        (project_root / "tests" / "test_mmwave_mqtt.py", "MQTT test"),
        (project_root / "tools" / "manipulate_scene.py", "Scene manipulation"),
    ]
    
    for path, desc in files_to_check:
        all_good &= check_file(path, desc)
    
    print("\n📚 Documentation:")
    docs_to_check = [
        (project_root / "README.md", "Main README"),
        (project_root / "LAUNCHER_README.md", "Launcher README"),
        (project_root / "AUTOMATION_README.md", "Automation README"),
        (project_root / "automation" / "README.md", "Automation dir README"),
        (project_root / "tests" / "README.md", "Tests dir README"),
        (project_root / "tools" / "README.md", "Tools dir README"),
    ]
    
    for path, desc in docs_to_check:
        all_good &= check_file(path, desc)
    
    print("\n🚀 ROS2 & Gazebo Components:")
    sim_files = [
        (project_root / "sim" / "ros2" / "launch" / "olympus_gazebo.launch.py", "ROS2 launch file"),
        (project_root / "sim" / "ros2" / "mmwave_mqtt_bridge.py", "mmWave MQTT bridge"),
        (project_root / "sim" / "gazebo" / "worlds" / "mmwave_test.sdf", "Gazebo world file"),
    ]
    
    for path, desc in sim_files:
        all_good &= check_file(path, desc)
    
    print(f"\n{'='*50}")
    if all_good:
        print("🎉 All components verified successfully!")
        print("\n🚀 Ready to launch:")
        print("   ./olympus full --automation")
        print("   ./olympus --list-modes")
        print("\n🧪 Ready to test:")
        print("   python3 tests/test_automation_loop.py 30")
        print("   python3 tests/test_mmwave_mqtt.py")
    else:
        print("⚠️  Some components are missing or misplaced")
        print("Please check the errors above")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
