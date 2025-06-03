#!/usr/bin/env python3
"""
Project Olympus - ns-3 802.11s Mesh Network Controller
Simulates realistic wireless mesh networking with RSSI, packet loss, and roaming
"""

import sys
import time
import json
import threading
import logging
import argparse
import subprocess
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
from pathlib import Path

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
log = logging.getLogger('MeshController')

@dataclass
class NodePosition:
    """3D position of a mesh node."""
    x: float
    y: float  
    z: float
    node_id: int

@dataclass
class MeshLink:
    """Link between two mesh nodes with quality metrics."""
    node_a: int
    node_b: int
    rssi: float  # dBm
    packet_loss: float  # 0.0 to 1.0
    latency: float  # seconds
    bandwidth: float  # Mbps

class NS3MeshController:
    """Controls ns-3 802.11s mesh network simulation."""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.num_nodes = config.get('nodes', 5)
        self.loss_rate = config.get('loss_rate', 0.05)  # 5% base packet loss
        self.interference_level = config.get('interference', 0.1)
        
        # Network topology
        self.node_positions: Dict[int, NodePosition] = {}
        self.mesh_links: Dict[tuple, MeshLink] = {}
        
        # Simulation state
        self.running = False
        self.simulation_time = 0.0
        self.packet_stats = {
            'sent': 0,
            'received': 0,
            'dropped': 0,
            'retransmissions': 0
        }
        
        # ns-3 simulation script path
        self.ns3_script = Path(__file__).parent / "olympus_mesh.cc"
        self.ns3_build_dir = "/opt/ns3/ns-allinone-3.42/ns-3.42"
        
    def generate_apartment_topology(self):
        """Generate realistic apartment mesh topology."""
        # Apartment layout: 3 rooms + hallway + balcony
        apartment_positions = [
            (2.0, 2.0, 1.5),    # Living room (HERMES hub)
            (8.0, 2.0, 1.5),    # Bedroom 1
            (8.0, 6.0, 1.5),    # Bedroom 2  
            (2.0, 6.0, 1.5),    # Kitchen
            (5.0, 4.0, 1.5),    # Hallway (relay node)
        ]
        
        for i, (x, y, z) in enumerate(apartment_positions[:self.num_nodes]):
            self.node_positions[i] = NodePosition(x, y, z, i)
            log.info(f"Node {i} positioned at ({x:.1f}, {y:.1f}, {z:.1f})")
    
    def calculate_link_quality(self, node_a: int, node_b: int) -> MeshLink:
        """Calculate realistic link quality based on distance and environment."""
        pos_a = self.node_positions[node_a]
        pos_b = self.node_positions[node_b]
        
        # Calculate 3D distance
        distance = ((pos_a.x - pos_b.x)**2 + 
                   (pos_a.y - pos_b.y)**2 + 
                   (pos_a.z - pos_b.z)**2)**0.5
        
        # WiFi path loss model (indoor, 2.4GHz)
        # RSSI = -30 - 20*log10(distance) - wall_attenuation
        base_rssi = -30 - (20 * (distance / 3.0))  # Normalize for apartment scale
        
        # Add wall attenuation (simplified room-based model)
        wall_penalty = 0
        if abs(pos_a.x - pos_b.x) > 4 or abs(pos_a.y - pos_b.y) > 4:
            wall_penalty = 15  # dB penalty for wall penetration
        
        rssi = base_rssi - wall_penalty
        
        # Calculate packet loss based on RSSI
        if rssi > -50:
            packet_loss = 0.001  # Excellent signal
        elif rssi > -70:
            packet_loss = 0.01   # Good signal  
        elif rssi > -80:
            packet_loss = 0.05   # Fair signal
        elif rssi > -90:
            packet_loss = 0.15   # Poor signal
        else:
            packet_loss = 0.50   # Very poor signal
        
        # Add base interference
        packet_loss += self.interference_level
        packet_loss = min(packet_loss, 0.8)  # Cap at 80%
        
        # Calculate latency (base + queuing + propagation)
        base_latency = 0.001  # 1ms base
        queuing_latency = packet_loss * 0.010  # More loss = more retx = more delay
        latency = base_latency + queuing_latency
        
        # Calculate available bandwidth (54 Mbps * (1 - loss))
        bandwidth = 54.0 * (1.0 - packet_loss)
        
        return MeshLink(
            node_a=node_a,
            node_b=node_b,
            rssi=rssi,
            packet_loss=packet_loss,
            latency=latency,
            bandwidth=bandwidth
        )
    
    def build_mesh_topology(self):
        """Build complete mesh topology with link qualities."""
        log.info("Building 802.11s mesh topology...")
        
        # Calculate all pairwise links
        for i in range(self.num_nodes):
            for j in range(i + 1, self.num_nodes):
                link = self.calculate_link_quality(i, j)
                self.mesh_links[(i, j)] = link
                
                log.info(f"Link {i}-{j}: RSSI={link.rssi:.1f}dBm, "
                        f"Loss={link.packet_loss:.1%}, "
                        f"Latency={link.latency*1000:.1f}ms, "
                        f"BW={link.bandwidth:.1f}Mbps")
    
    def generate_ns3_script(self) -> str:
        """Generate ns-3 C++ simulation script."""
        script_content = f"""
/* Project Olympus - ns-3 802.11s Mesh Simulation */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mesh-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/tap-bridge-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("OlympusMesh");

class OlympusMeshSimulation
{{
public:
  void Run();
  void SetupNodes();
  void SetupMobility();
  void SetupMesh();
  void SetupApplications();
  void SetupTapBridges();
  
private:
  NodeContainer nodes;
  NetDeviceContainer meshDevices;
  Ipv4InterfaceContainer interfaces;
  uint32_t m_nNodes = {self.num_nodes};
}};

void OlympusMeshSimulation::SetupNodes()
{{
  nodes.Create(m_nNodes);
  
  // Install internet stack
  InternetStackHelper internet;
  internet.Install(nodes);
}}

void OlympusMeshSimulation::SetupMobility()
{{
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  
  // Apartment positions
"""
        
        # Add node positions
        for node_id, pos in self.node_positions.items():
            script_content += f"  positionAlloc->Add(Vector({pos.x}, {pos.y}, {pos.z}));\n"
        
        script_content += f"""
  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);
}}

void OlympusMeshSimulation::SetupMesh()
{{
  // 802.11s mesh configuration
  MeshHelper mesh = MeshHelper::Default();
  mesh.SetStackInstaller("ns3::Dot11sStack");
  mesh.SetMacType("RandomStart", TimeValue(Seconds(0.1)));
  mesh.SetNumberOfInterfaces(1);
  
  // WiFi PHY and channel
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
  
  // Add path loss model with realistic indoor propagation
  wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                 "Exponent", DoubleValue(3.0),
                                 "ReferenceDistance", DoubleValue(1.0),
                                 "ReferenceLoss", DoubleValue(40.0));
                                 
  // Add random packet loss to simulate interference
  wifiChannel.AddPropagationLoss("ns3::RandomPropagationLossModel",
                                 "Variable", StringValue("ns3::UniformRandomVariable[Min=0|Max={self.loss_rate}]"));
  
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
  wifiPhy.SetChannel(wifiChannel.Create());
  
  // Install mesh
  meshDevices = mesh.Install(wifiPhy, nodes);
  
  // Assign IP addresses
  Ipv4AddressHelper address;
  address.SetBase("192.168.100.0", "255.255.255.0");
  interfaces = address.Assign(meshDevices);
}}

void OlympusMeshSimulation::SetupTapBridges()
{{
  // Create TAP bridges for each node to connect to QEMU
  TapBridgeHelper tapBridge;
  tapBridge.SetAttribute("Mode", StringValue("UseLocal"));
  
  for (uint32_t i = 0; i < m_nNodes; ++i)
  {{
    std::string tapName = "tap-esp32-" + std::to_string(i);
    tapBridge.SetAttribute("DeviceName", StringValue(tapName));
    tapBridge.Install(nodes.Get(i), meshDevices.Get(i));
    
    NS_LOG_INFO("TAP bridge " << tapName << " connected to node " << i);
  }}
}}

void OlympusMeshSimulation::SetupApplications()
{{
  // MQTT traffic simulation (sensor data to HERMES/ZEUS)
  // Node 0 is the HERMES hub, others are sensor nodes
  
  uint16_t port = 1883; // MQTT port
  
  // MQTT broker on node 0 (HERMES)
  PacketSinkHelper sinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
  ApplicationContainer sinkApp = sinkHelper.Install(nodes.Get(0));
  sinkApp.Start(Seconds(1.0));
  
  // MQTT clients on sensor nodes
  for (uint32_t i = 1; i < m_nNodes; ++i)
  {{
    // Periodic sensor data transmission
    OnOffHelper client("ns3::TcpSocketFactory", Address(InetSocketAddress(interfaces.GetAddress(0), port)));
    client.SetAttribute("OnTime", StringValue("ns3::ExponentialRandomVariable[Mean=1.0]"));
    client.SetAttribute("OffTime", StringValue("ns3::ExponentialRandomVariable[Mean=9.0]"));
    client.SetAttribute("DataRate", DataRateValue(DataRate("100bps"))); // Small sensor packets
    client.SetAttribute("PacketSize", UintegerValue(64)); // Typical MQTT sensor packet
    
    ApplicationContainer clientApp = client.Install(nodes.Get(i));
    clientApp.Start(Seconds(2.0 + i * 0.1)); // Stagger starts
  }}
}}

void OlympusMeshSimulation::Run()
{{
  SetupNodes();
  SetupMobility();
  SetupMesh();
  SetupTapBridges();
  SetupApplications();
  
  // Enable routing
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();
  
  // Run simulation
  Simulator::Stop(Seconds(3600.0)); // 1 hour simulation
  NS_LOG_INFO("Starting Olympus mesh simulation with " << m_nNodes << " nodes");
  Simulator::Run();
  Simulator::Destroy();
}}

int main(int argc, char *argv[])
{{
  LogComponentEnable("OlympusMesh", LOG_LEVEL_INFO);
  
  OlympusMeshSimulation simulation;
  simulation.Run();
  
  return 0;
}}
"""
        return script_content
    
    def compile_ns3_simulation(self) -> bool:
        """Compile the ns-3 simulation script."""
        try:
            # Write C++ script
            with open(self.ns3_script, 'w') as f:
                f.write(self.generate_ns3_script())
            
            log.info("Compiling ns-3 mesh simulation...")
            
            # Copy to ns-3 scratch directory
            scratch_dir = Path(self.ns3_build_dir) / "scratch"
            scratch_file = scratch_dir / "olympus_mesh.cc"
            
            subprocess.run(['cp', str(self.ns3_script), str(scratch_file)], check=True)
            
            # Configure and build
            subprocess.run([
                './ns3', 'configure', '--build-profile=optimized', '--enable-examples'
            ], cwd=self.ns3_build_dir, check=True, capture_output=True)
            
            subprocess.run([
                './ns3', 'build', 'olympus_mesh'
            ], cwd=self.ns3_build_dir, check=True, capture_output=True)
            
            log.info("ns-3 simulation compiled successfully")
            return True
            
        except subprocess.CalledProcessError as e:
            log.error(f"ns-3 compilation failed: {e}")
            return False
        except Exception as e:
            log.error(f"Compilation error: {e}")
            return False
    
    def start_simulation(self) -> bool:
        """Start the ns-3 mesh simulation."""
        try:
            log.info("Starting ns-3 802.11s mesh simulation...")
            
            # Run ns-3 simulation in background
            self.ns3_process = subprocess.Popen([
                './ns3', 'run', 'olympus_mesh'
            ], cwd=self.ns3_build_dir, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            # Monitor simulation output
            def monitor_output():
                for line in iter(self.ns3_process.stdout.readline, b''):
                    if line:
                        log.info(f"ns-3: {line.decode().strip()}")
            
            output_thread = threading.Thread(target=monitor_output, daemon=True)
            output_thread.start()
            
            # Wait for simulation to start
            time.sleep(5)
            
            if self.ns3_process.poll() is None:
                log.info("ns-3 mesh simulation started successfully")
                self.running = True
                return True
            else:
                log.error("ns-3 simulation failed to start")
                return False
                
        except Exception as e:
            log.error(f"Failed to start ns-3 simulation: {e}")
            return False
    
    def get_network_stats(self) -> Dict[str, Any]:
        """Get current network performance statistics."""
        # In a real implementation, this would query ns-3 for live stats
        # For now, we'll return simulated metrics based on our link calculations
        
        total_links = len(self.mesh_links)
        avg_packet_loss = sum(link.packet_loss for link in self.mesh_links.values()) / total_links
        avg_latency = sum(link.latency for link in self.mesh_links.values()) / total_links
        avg_rssi = sum(link.rssi for link in self.mesh_links.values()) / total_links
        
        return {
            'timestamp': time.time(),
            'nodes': self.num_nodes,
            'active_links': total_links,
            'avg_packet_loss': avg_packet_loss,
            'avg_latency_ms': avg_latency * 1000,
            'avg_rssi_dbm': avg_rssi,
            'packets_sent': self.packet_stats['sent'],
            'packets_received': self.packet_stats['received'],
            'packet_delivery_ratio': (self.packet_stats['received'] / max(1, self.packet_stats['sent'])),
            'topology': {
                'nodes': {str(k): {'x': v.x, 'y': v.y, 'z': v.z} for k, v in self.node_positions.items()},
                'links': {f"{k[0]}-{k[1]}": {
                    'rssi': v.rssi,
                    'loss': v.packet_loss,
                    'latency_ms': v.latency * 1000,
                    'bandwidth_mbps': v.bandwidth
                } for k, v in self.mesh_links.items()}
            }
        }
    
    def inject_interference(self, level: float):
        """Inject interference to test mesh resilience."""
        log.info(f"Injecting interference level: {level:.1%}")
        self.interference_level = level
        
        # Recalculate link qualities with new interference
        self.build_mesh_topology()
    
    def simulate_node_failure(self, node_id: int):
        """Simulate node failure to test mesh healing."""
        log.info(f"Simulating failure of node {node_id}")
        
        # Remove all links involving the failed node
        failed_links = [(k, v) for k, v in self.mesh_links.items() if node_id in k]
        for link_key, _ in failed_links:
            del self.mesh_links[link_key]
            log.info(f"Link {link_key} failed due to node {node_id} failure")
    
    def run_controller(self):
        """Main controller loop."""
        log.info("🌐 Starting Project Olympus Mesh Controller")
        
        # Generate topology
        self.generate_apartment_topology()
        self.build_mesh_topology()
        
        # Compile simulation
        if not self.compile_ns3_simulation():
            log.error("Failed to compile ns-3 simulation")
            return False
        
        # Start simulation
        if not self.start_simulation():
            log.error("Failed to start ns-3 simulation")
            return False
        
        # Main monitoring loop
        try:
            while self.running:
                time.sleep(10)  # Update every 10 seconds
                
                # Update simulation time
                self.simulation_time += 10
                
                # Log network stats
                stats = self.get_network_stats()
                log.info(f"Network: {stats['packet_delivery_ratio']:.1%} PDR, "
                        f"{stats['avg_latency_ms']:.1f}ms latency, "
                        f"{stats['avg_packet_loss']:.1%} loss")
                
                # Simulate some packet traffic for stats
                self.packet_stats['sent'] += 50
                self.packet_stats['received'] += int(50 * (1 - stats['avg_packet_loss']))
                
        except KeyboardInterrupt:
            log.info("Mesh controller interrupted")
        finally:
            self.cleanup()
        
        return True
    
    def cleanup(self):
        """Clean up ns-3 simulation."""
        log.info("Cleaning up mesh controller...")
        self.running = False
        
        if hasattr(self, 'ns3_process'):
            try:
                self.ns3_process.terminate()
                self.ns3_process.wait(timeout=5)
            except:
                self.ns3_process.kill()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Project Olympus ns-3 Mesh Controller')
    parser.add_argument('--nodes', type=int, default=5, help='Number of mesh nodes')
    parser.add_argument('--loss-rate', type=float, default=0.05, help='Base packet loss rate')
    parser.add_argument('--interference', type=float, default=0.1, help='Interference level')
    parser.add_argument('--config', help='JSON configuration file')
    
    args = parser.parse_args()
    
    config = {
        'nodes': args.nodes,
        'loss_rate': args.loss_rate,
        'interference': args.interference
    }
    
    if args.config:
        with open(args.config, 'r') as f:
            config.update(json.load(f))
    
    controller = NS3MeshController(config)
    success = controller.run_controller()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main() 