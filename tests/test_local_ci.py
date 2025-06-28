#!/usr/bin/env python3
"""
Local CI Test Runner
Validates CI functionality before pushing to GitHub Actions

Usage:
    python tests/test_local_ci.py
    python tests/test_local_ci.py --quick  # 15 second test
    python tests/test_local_ci.py --scenario poor  # Test with poor network
"""

import argparse
import subprocess
import sys
import time
import os
from pathlib import Path

def run_local_ci_test(duration: int = 30, network_condition: str = "good") -> bool:
    """Run the CI test locally with specified parameters"""
    
    print("Local CI Test Runner")
    print("=" * 40)
    print(f"Duration: {duration}s")
    print(f"Network condition: {network_condition}")
    print()
    
    # Set environment variables
    env = os.environ.copy()
    env.update({
        'TEST_DURATION_SEC': str(duration),
        'TEST_SCENARIO_NAME': 'local_test',
        'TEST_NETWORK_CONDITION': network_condition,
        'TEST_EXPECTED_LATENCY_MS': '100',
        'MAX_LATENCY_P95_MS': '300',
        'MAX_DROPPED_MESSAGES': '2'
    })
    
    try:
        # Check if MQTT broker is running
        print("Checking MQTT broker...")
        broker_check = subprocess.run(['pgrep', 'mosquitto'], capture_output=True)
        if broker_check.returncode != 0:
            print("Starting MQTT broker...")
            subprocess.Popen(['mosquitto', '-d'])
            time.sleep(2)
        else:
            print("MQTT broker already running")
        
        # Run the CI test
        print("Running CI simulation test...")
        result = subprocess.run(
            [sys.executable, 'tests/test_ci_simulation.py'],
            env=env,
            timeout=duration + 30  # Add buffer for startup/shutdown
        )
        
        success = result.returncode == 0
        
        if success:
            print("\nSUCCESS: Local CI test passed")
            print("The CI pipeline should work in GitHub Actions")
        else:
            print("\nFAILURE: Local CI test failed")
            print("Fix issues before pushing to GitHub Actions")
        
        return success
        
    except subprocess.TimeoutExpired:
        print("\nERROR: Test timed out")
        return False
    except KeyboardInterrupt:
        print("\nINTERRUPTED: Test cancelled by user")
        return False
    except Exception as e:
        print(f"\nERROR: Test failed with exception: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Local CI test runner")
    parser.add_argument('--duration', type=int, default=30,
                       help='Test duration in seconds (default: 30)')
    parser.add_argument('--scenario', choices=['excellent', 'good', 'fair', 'poor'],
                       default='good', help='Network condition (default: good)')
    parser.add_argument('--quick', action='store_true',
                       help='Run quick 15-second test')
    
    args = parser.parse_args()
    
    if args.quick:
        args.duration = 15
    
    success = run_local_ci_test(args.duration, args.scenario)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()