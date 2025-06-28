#!/usr/bin/env python3
"""
Network Configuration for Olympus Simulation
Provides configuration management for network realism settings
"""

import json
import os
from pathlib import Path
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict
from network_realism import NetworkConfig, NetworkCondition

@dataclass
class GlobalNetworkConfig:
    """Global network configuration for the entire simulation"""
    # Default network condition for all components
    default_condition: NetworkCondition = NetworkCondition.GOOD
    
    # Component-specific overrides
    sensor_bridge_config: Optional[NetworkConfig] = None
    automation_config: Optional[NetworkConfig] = None
    
    # Environment-based conditions
    environment_based: bool = True  # Use apartment location for network quality
    
    # Simulation control
    enabled: bool = True
    log_network_events: bool = True
    
    # Performance settings
    max_queue_size: int = 1000
    queue_warning_threshold: int = 100
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization"""
        result = asdict(self)
        result['default_condition'] = self.default_condition.value
        
        if self.sensor_bridge_config:
            result['sensor_bridge_config'] = asdict(self.sensor_bridge_config)
            result['sensor_bridge_config']['condition'] = self.sensor_bridge_config.condition.value
        
        if self.automation_config:
            result['automation_config'] = asdict(self.automation_config)
            result['automation_config']['condition'] = self.automation_config.condition.value
        
        return result
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'GlobalNetworkConfig':
        """Create from dictionary"""
        # Convert condition strings back to enums
        if 'default_condition' in data:
            data['default_condition'] = NetworkCondition(data['default_condition'])
        
        if 'sensor_bridge_config' in data and data['sensor_bridge_config']:
            config_data = data['sensor_bridge_config']
            if 'condition' in config_data:
                config_data['condition'] = NetworkCondition(config_data['condition'])
            data['sensor_bridge_config'] = NetworkConfig(**config_data)
        
        if 'automation_config' in data and data['automation_config']:
            config_data = data['automation_config']
            if 'condition' in config_data:
                config_data['condition'] = NetworkCondition(config_data['condition'])
            data['automation_config'] = NetworkConfig(**config_data)
        
        return cls(**data)

class NetworkConfigManager:
    """Manages network configuration loading, saving, and environment-based adaptation"""
    
    def __init__(self, config_path: str = "network/network_config.json"):
        self.config_path = Path(config_path)
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        self._config: Optional[GlobalNetworkConfig] = None
        
        # Predefined scenario configurations
        self.scenarios = {
            'perfect': GlobalNetworkConfig(
                default_condition=NetworkCondition.EXCELLENT,
                enabled=True
            ),
            'realistic': GlobalNetworkConfig(
                default_condition=NetworkCondition.GOOD,
                enabled=True
            ),
            'challenging': GlobalNetworkConfig(
                default_condition=NetworkCondition.FAIR,
                enabled=True,
                sensor_bridge_config=NetworkConfig.from_condition(NetworkCondition.POOR)
            ),
            'unreliable': GlobalNetworkConfig(
                default_condition=NetworkCondition.POOR,
                enabled=True,
                sensor_bridge_config=NetworkConfig.from_condition(NetworkCondition.UNRELIABLE),
                automation_config=NetworkConfig.from_condition(NetworkCondition.POOR)
            ),
            'disabled': GlobalNetworkConfig(
                enabled=False
            )
        }
    
    def load_config(self) -> GlobalNetworkConfig:
        """Load configuration from file or create default"""
        if self._config is not None:
            return self._config
        
        if self.config_path.exists():
            try:
                with open(self.config_path, 'r') as f:
                    data = json.load(f)
                self._config = GlobalNetworkConfig.from_dict(data)
                print(f"Loaded network config from {self.config_path}")
            except Exception as e:
                print(f"WARNING: Error loading network config: {e}, using defaults")
                self._config = GlobalNetworkConfig()
        else:
            self._config = GlobalNetworkConfig()
            self.save_config()  # Save default config
        
        return self._config
    
    def save_config(self, config: GlobalNetworkConfig = None):
        """Save configuration to file"""
        if config is None:
            config = self._config or GlobalNetworkConfig()
        
        try:
            with open(self.config_path, 'w') as f:
                json.dump(config.to_dict(), f, indent=2)
            print(f"Saved network config to {self.config_path}")
        except Exception as e:
            print(f"ERROR: Error saving network config: {e}")
    
    def get_config_for_component(self, component: str) -> NetworkConfig:
        """Get network configuration for a specific component"""
        global_config = self.load_config()
        
        if not global_config.enabled:
            return NetworkConfig(enabled=False)
        
        # Check for component-specific override
        if component == 'sensor_bridge' and global_config.sensor_bridge_config:
            return global_config.sensor_bridge_config
        elif component == 'automation' and global_config.automation_config:
            return global_config.automation_config
        
        # Use default condition
        return NetworkConfig.from_condition(global_config.default_condition)
    
    def set_scenario(self, scenario_name: str):
        """Set predefined scenario configuration"""
        if scenario_name not in self.scenarios:
            available = list(self.scenarios.keys())
            raise ValueError(f"Unknown scenario '{scenario_name}'. Available: {available}")
        
        self._config = self.scenarios[scenario_name]
        self.save_config()
        print(f"Set network scenario: {scenario_name}")
    
    def get_environment_config(self, sensor_position: tuple = None, 
                             room_type: str = None) -> NetworkConfig:
        """Get network config based on environmental factors"""
        global_config = self.load_config()
        
        if not global_config.environment_based:
            return self.get_config_for_component('sensor_bridge')
        
        # Simulate environment-based network conditions
        # In a real deployment, this could use actual signal strength measurements
        
        base_condition = global_config.default_condition
        
        # Adjust based on room type (simulated)
        if room_type:
            room_adjustments = {
                'living_room': 0,      # Central, good coverage
                'bedroom': -1,         # Further from router
                'kitchen': -1,         # Interference from appliances
                'bathroom': -2,        # Poor coverage + humidity
                'basement': -2,        # Physical barriers
                'garage': -3           # Outside main coverage area
            }
            
            adjustment = room_adjustments.get(room_type, 0)
            condition_values = list(NetworkCondition)
            current_index = condition_values.index(base_condition)
            
            # Adjust condition (higher index = worse condition)
            new_index = max(0, min(len(condition_values) - 1, current_index + adjustment))
            adjusted_condition = condition_values[new_index]
            
            print(f"Adjusted network condition for {room_type}: {adjusted_condition.value}")
            return NetworkConfig.from_condition(adjusted_condition)
        
        return NetworkConfig.from_condition(base_condition)
    
    def update_runtime_conditions(self, packet_loss_rate: float = None,
                                latency_ms: float = None,
                                connection_reliability: float = None):
        """Update network conditions at runtime"""
        config = self.load_config()
        
        # Create new default config with updated parameters
        new_config = NetworkConfig.from_condition(config.default_condition)
        
        if packet_loss_rate is not None:
            new_config.packet_loss_rate = max(0.0, min(1.0, packet_loss_rate))
        
        if latency_ms is not None:
            new_config.base_latency_ms = max(0.0, latency_ms)
        
        if connection_reliability is not None:
            # Convert reliability (0-1) to disconnection rate
            new_config.connection_drop_rate = max(0.0, 1.0 - connection_reliability)
        
        # Update global config
        config.sensor_bridge_config = new_config
        config.automation_config = new_config
        
        self._config = config
        self.save_config()
        
        print(f"Updated runtime network conditions:")
        print(f"   Packet loss: {new_config.packet_loss_rate:.1%}")
        print(f"   Latency: {new_config.base_latency_ms:.1f}ms")
        print(f"   Disconnection rate: {new_config.connection_drop_rate:.3f}")
    
    def get_statistics_summary(self) -> Dict[str, Any]:
        """Get summary of current network configuration"""
        config = self.load_config()
        
        return {
            'enabled': config.enabled,
            'default_condition': config.default_condition.value,
            'environment_based': config.environment_based,
            'sensor_override': config.sensor_bridge_config is not None,
            'automation_override': config.automation_config is not None,
            'available_scenarios': list(self.scenarios.keys())
        }

# Global config manager instance
_config_manager = None

def get_network_config_manager() -> NetworkConfigManager:
    """Get global network configuration manager"""
    global _config_manager
    if _config_manager is None:
        _config_manager = NetworkConfigManager()
    return _config_manager

def get_network_config_for_component(component: str) -> NetworkConfig:
    """Convenience function to get network config for a component"""
    return get_network_config_manager().get_config_for_component(component)