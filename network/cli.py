#!/usr/bin/env python3
"""
Network Realism CLI for Olympus Simulation
Provides command-line interface for configuring network conditions
"""

import argparse
import sys
import json
from pathlib import Path
from config import NetworkConfigManager, get_network_config_manager
from network_realism import NetworkCondition

def cmd_set_condition(args):
    """Set network condition for simulation"""
    try:
        condition = NetworkCondition(args.condition)
        manager = get_network_config_manager()
        
        if args.component:
            # Set for specific component
            config = manager.load_config()
            component_config = NetworkConfig.from_condition(condition)
            
            if args.component == 'sensor_bridge':
                config.sensor_bridge_config = component_config
            elif args.component == 'automation':
                config.automation_config = component_config
            else:
                print(f"ERROR: Unknown component: {args.component}")
                return False
            
            manager.save_config(config)
            print(f"SUCCESS: Set {args.component} network condition to: {condition.value}")
        else:
            # Set global condition
            manager.set_scenario(condition.value)
            print(f"SUCCESS: Set global network condition to: {condition.value}")
        
        return True
        
    except ValueError as e:
        available = [c.value for c in NetworkCondition]
        print(f"ERROR: Invalid condition '{args.condition}'. Available: {available}")
        return False
    except Exception as e:
        print(f"ERROR: Error setting condition: {e}")
        return False

def cmd_set_scenario(args):
    """Set predefined network scenario"""
    try:
        manager = get_network_config_manager()
        manager.set_scenario(args.scenario)
        print(f"SUCCESS: Applied scenario: {args.scenario}")
        return True
    except ValueError as e:
        print(f"ERROR: {e}")
        return False
    except Exception as e:
        print(f"ERROR: Error setting scenario: {e}")
        return False

def cmd_set_custom(args):
    """Set custom network parameters"""
    try:
        manager = get_network_config_manager()
        manager.update_runtime_conditions(
            packet_loss_rate=args.packet_loss,
            latency_ms=args.latency,
            connection_reliability=args.reliability
        )
        print("SUCCESS: Updated network conditions with custom parameters")
        return True
    except Exception as e:
        print(f"ERROR: Error setting custom conditions: {e}")
        return False

def cmd_status(args):
    """Show current network configuration status"""
    try:
        manager = get_network_config_manager()
        
        # Global status
        summary = manager.get_statistics_summary()
        print("Network Configuration Status")
        print("=" * 40)
        print(f"Enabled: {summary['enabled']}")
        print(f"Default condition: {summary['default_condition']}")
        print(f"Environment-based: {summary['environment_based']}")
        print(f"Sensor override: {summary['sensor_override']}")
        print(f"Automation override: {summary['automation_override']}")
        print()
        
        # Component-specific configs
        for component in ['sensor_bridge', 'automation']:
            config = manager.get_config_for_component(component)
            print(f"{component.replace('_', ' ').title()}:")
            print(f"  Enabled: {config.enabled}")
            if config.enabled:
                print(f"  Condition: {config.condition.value}")
                print(f"  Packet loss: {config.packet_loss_rate:.1%}")
                print(f"  Base latency: {config.base_latency_ms:.1f}ms")
                print(f"  Jitter: {config.jitter_ms:.1f}ms")
                print(f"  Connection drops: {config.connection_drop_rate:.3f}")
        
        print()
        print(f"Available scenarios: {', '.join(summary['available_scenarios'])}")
        
        return True
        
    except Exception as e:
        print(f"ERROR: Error getting status: {e}")
        return False

def cmd_disable(args):
    """Disable network realism"""
    try:
        manager = get_network_config_manager()
        manager.set_scenario('disabled')
        print("SUCCESS: Network realism disabled")
        return True
    except Exception as e:
        print(f"ERROR: Error disabling network realism: {e}")
        return False

def cmd_test(args):
    """Run network realism test"""
    print("Running network realism test...")
    
    import subprocess
    try:
        result = subprocess.run([sys.executable, 'test_step5_network.py'], 
                               capture_output=False, text=True)
        return result.returncode == 0
    except Exception as e:
        print(f"ERROR: Test error: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description="Network Realism CLI for Olympus Simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s status                          # Show current configuration
  %(prog)s condition good                  # Set global condition to 'good'
  %(prog)s condition poor --component sensor_bridge  # Set sensor condition
  %(prog)s scenario realistic              # Apply realistic scenario
  %(prog)s custom --packet-loss 0.1 --latency 50     # Custom parameters
  %(prog)s disable                         # Disable network realism
  %(prog)s test                           # Run test suite
        """
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Status command
    status_parser = subparsers.add_parser('status', help='Show network configuration status')
    status_parser.set_defaults(func=cmd_status)
    
    # Condition command
    condition_parser = subparsers.add_parser('condition', help='Set network condition')
    condition_parser.add_argument('condition', 
                                 choices=[c.value for c in NetworkCondition],
                                 help='Network condition to set')
    condition_parser.add_argument('--component', 
                                 choices=['sensor_bridge', 'automation'],
                                 help='Apply to specific component only')
    condition_parser.set_defaults(func=cmd_set_condition)
    
    # Scenario command
    scenario_parser = subparsers.add_parser('scenario', help='Set predefined scenario')
    scenario_parser.add_argument('scenario',
                                choices=['perfect', 'realistic', 'challenging', 'unreliable', 'disabled'],
                                help='Predefined scenario to apply')
    scenario_parser.set_defaults(func=cmd_set_scenario)
    
    # Custom command
    custom_parser = subparsers.add_parser('custom', help='Set custom network parameters')
    custom_parser.add_argument('--packet-loss', type=float, metavar='RATE',
                              help='Packet loss rate (0.0-1.0)')
    custom_parser.add_argument('--latency', type=float, metavar='MS',
                              help='Base latency in milliseconds')
    custom_parser.add_argument('--reliability', type=float, metavar='RATE',
                              help='Connection reliability (0.0-1.0)')
    custom_parser.set_defaults(func=cmd_set_custom)
    
    # Disable command
    disable_parser = subparsers.add_parser('disable', help='Disable network realism')
    disable_parser.set_defaults(func=cmd_disable)
    
    # Test command
    test_parser = subparsers.add_parser('test', help='Run network realism test')
    test_parser.set_defaults(func=cmd_test)
    
    # Parse arguments
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return 1
    
    # Execute command
    try:
        success = args.func(args)
        return 0 if success else 1
    except Exception as e:
        print(f"ERROR: Command failed: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())