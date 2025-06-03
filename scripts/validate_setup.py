#!/usr/bin/env python3
"""
Setup validation script for Project Olympus
Checks that all required files are present and properly configured.
"""

import os
import sys
from pathlib import Path

def check_file_exists(filepath, description):
    """Check if a file exists and report status."""
    if os.path.exists(filepath):
        print(f"✓ {description}: {filepath}")
        return True
    else:
        print(f"✗ {description}: {filepath} (MISSING)")
        return False

def check_directory_exists(dirpath, description):
    """Check if a directory exists and report status."""
    if os.path.isdir(dirpath):
        print(f"✓ {description}: {dirpath}")
        return True
    else:
        print(f"✗ {description}: {dirpath} (MISSING)")
        return False

def main():
    """Main validation function."""
    print("Project Olympus Setup Validation")
    print("=" * 40)
    
    all_good = True
    
    # Check directory structure
    directories = [
        ("sim/renode", "Renode simulation directory"),
        ("sim/python", "Python stubs directory"),
        ("firmware/components/sensor_iface", "Sensor interface component"),
        ("firmware/main", "Main firmware directory"),
        ("tests", "Test directory"),
        ("infra", "Infrastructure directory"),
    ]
    
    for dirpath, description in directories:
        if not check_directory_exists(dirpath, description):
            all_good = False
    
    print()
    
    # Check required files
    files = [
        ("sim/renode/esp32_wroom32.repl", "Renode platform description"),
        ("sim/renode/run.resc", "Renode run script"),
        ("sim/renode/README.md", "Renode documentation"),
        ("sim/python/sensor_stubs.py", "Sensor stub implementations"),
        ("sim/python/sensor_bridge.py", "Sensor bridge script"),
        ("firmware/components/sensor_iface/CMakeLists.txt", "Sensor component CMake"),
        ("firmware/components/sensor_iface/sensor_iface.c", "Sensor interface implementation"),
        ("firmware/components/sensor_iface/include/sensor_iface.h", "Sensor interface header"),
        ("firmware/main/CMakeLists.txt", "Main component CMake"),
        ("firmware/main/main.c", "Main application"),
        ("firmware/CMakeLists.txt", "Firmware CMake"),
        ("firmware/sdkconfig.defaults", "SDK configuration"),
        ("tests/test_peripherals.py", "Peripheral tests"),
        ("tests/__init__.py", "Test package init"),
        ("infra/Dockerfile", "Docker container definition"),
        ("infra/docker-compose.yml", "Docker compose configuration"),
        ("Makefile", "Build automation"),
        ("README.md", "Project documentation"),
        ("requirements.txt", "Python requirements"),
        ("pytest.ini", "Pytest configuration"),
    ]
    
    for filepath, description in files:
        if not check_file_exists(filepath, description):
            all_good = False
    
    print()
    
    # Check file contents for critical configurations
    print("Configuration Checks:")
    print("-" * 20)
    
    # Check Makefile for sim_phase1 target
    try:
        with open("Makefile", "r") as f:
            makefile_content = f.read()
            if "sim_phase1:" in makefile_content:
                print("✓ Makefile contains sim_phase1 target")
            else:
                print("✗ Makefile missing sim_phase1 target")
                all_good = False
    except Exception as e:
        print(f"✗ Error reading Makefile: {e}")
        all_good = False
    
    # Check Dockerfile for required packages
    try:
        with open("infra/Dockerfile", "r") as f:
            dockerfile_content = f.read()
            if "mono-runtime" in dockerfile_content and "python3-netifaces" in dockerfile_content:
                print("✓ Dockerfile contains required Renode dependencies")
            else:
                print("✗ Dockerfile missing required Renode dependencies")
                all_good = False
    except Exception as e:
        print(f"✗ Error reading Dockerfile: {e}")
        all_good = False
    
    print()
    
    if all_good:
        print("🎉 All checks passed! Project Olympus is ready for Phase 1 & 2.")
        print("\nNext steps:")
        print("1. Build the container: docker compose -f infra/docker-compose.yml build")
        print("2. Run simulation: make sim_phase1")
        print("3. Run tests: make test")
        return 0
    else:
        print("❌ Some checks failed. Please review the missing files/configurations above.")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 