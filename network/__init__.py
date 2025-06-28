"""
Olympus Network Realism Package

Provides network simulation capabilities for realistic IoT testing:
- Packet loss simulation
- Latency injection with jitter
- Connection reliability modeling
- QoS-aware network behaviors
- Environment-based condition adaptation
"""

from .network_realism import (
    NetworkConfig,
    NetworkCondition,
    NetworkSimulator,
    NetworkRealisticMQTTClient,
    create_realistic_mqtt_client
)

from .config import (
    GlobalNetworkConfig,
    NetworkConfigManager,
    get_network_config_manager,
    get_network_config_for_component
)

__all__ = [
    'NetworkConfig',
    'NetworkCondition', 
    'NetworkSimulator',
    'NetworkRealisticMQTTClient',
    'create_realistic_mqtt_client',
    'GlobalNetworkConfig',
    'NetworkConfigManager',
    'get_network_config_manager',
    'get_network_config_for_component'
]