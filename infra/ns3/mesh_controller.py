#!/usr/bin/env python3
"""
Project Olympus - Simple Mesh Network Controller
Lightweight mesh network simulation for Docker container
"""

import time
import argparse
import json
import logging
import signal
import sys
from typing import Dict, Any

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
log = logging.getLogger('MeshController')

class SimpleMeshController:
    """Simple mesh network controller for Docker environment."""
    
    def __init__(self, nodes: int = 5, loss_rate: float = 0.05):
        self.nodes = nodes
        self.loss_rate = loss_rate
        self.running = False
        
        # Network simulation state
        self.network_stats = {
            'packet_delivery_ratio': 1.0 - loss_rate,
            'avg_latency_ms': 15.0,
            'avg_throughput_mbps': 25.0,
            'active_nodes': nodes
        }
        
        log.info(f"Initialized mesh controller with {nodes} nodes, {loss_rate:.1%} loss rate")
    
    def start_simulation(self):
        """Start the mesh network simulation."""
        log.info("🌐 Starting Olympus Mesh Network Simulation")
        log.info(f"Nodes: {self.nodes}, Base Loss Rate: {self.loss_rate:.1%}")
        
        self.running = True
        
        try:
            while self.running:
                # Simulate network activity
                self.update_network_stats()
                
                # Log statistics every 30 seconds
                log.info(f"Network: PDR={self.network_stats['packet_delivery_ratio']:.1%}, "
                        f"Latency={self.network_stats['avg_latency_ms']:.1f}ms, "
                        f"Throughput={self.network_stats['avg_throughput_mbps']:.1f}Mbps")
                
                time.sleep(30)
                
        except KeyboardInterrupt:
            log.info("Mesh simulation interrupted")
        finally:
            self.cleanup()
    
    def update_network_stats(self):
        """Update network statistics based on realistic models."""
        import random
        
        # Simulate some variability in network performance
        base_pdr = 1.0 - self.loss_rate
        
        # Add random variation (±5%)
        self.network_stats['packet_delivery_ratio'] = max(0.8, min(1.0, 
            base_pdr + random.uniform(-0.05, 0.05)))
        
        # Latency varies with packet loss
        base_latency = 15.0
        loss_penalty = (1.0 - self.network_stats['packet_delivery_ratio']) * 50
        self.network_stats['avg_latency_ms'] = base_latency + loss_penalty + random.uniform(-2, 2)
        
        # Throughput decreases with packet loss
        base_throughput = 25.0
        self.network_stats['avg_throughput_mbps'] = base_throughput * self.network_stats['packet_delivery_ratio'] + random.uniform(-1, 1)
    
    def get_network_stats(self) -> Dict[str, Any]:
        """Get current network statistics."""
        return {
            'timestamp': time.time(),
            'mesh': self.network_stats
        }
    
    def cleanup(self):
        """Clean up simulation resources."""
        log.info("Cleaning up mesh simulation...")
        self.running = False

def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Olympus Mesh Network Controller')
    parser.add_argument('--nodes', type=int, default=5, help='Number of mesh nodes')
    parser.add_argument('--loss_rate', type=float, default=0.05, help='Base packet loss rate')
    parser.add_argument('--duration', type=int, default=0, help='Simulation duration (0 = infinite)')
    
    args = parser.parse_args()
    
    # Create and start controller
    controller = SimpleMeshController(args.nodes, args.loss_rate)
    
    # Set up signal handlers
    def signal_handler(signum, frame):
        log.info(f"Received signal {signum}, shutting down...")
        controller.running = False
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Start simulation
    if args.duration > 0:
        log.info(f"Running simulation for {args.duration} seconds")
        controller.running = True
        time.sleep(args.duration)
        controller.cleanup()
    else:
        controller.start_simulation()

if __name__ == '__main__':
    main() 