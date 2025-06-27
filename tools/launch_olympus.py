#!/usr/bin/env python3
"""
Unified Olympus Simulation Launcher
Single entry point for all simulation modes with clear options
"""

import os
import sys
import subprocess
import argparse
import time
from pathlib import Path

class OlympusLauncher:
    def __init__(self):
        self.project_root = Path(__file__).parent.parent.absolute()
        self.gazebo_worlds = self.project_root / "sim" / "gazebo" / "worlds"
        self.ros2_launch = self.project_root / "sim" / "ros2" / "launch"
        
    def setup_environment(self):
        """Setup environment variables for Gazebo and ROS2"""
        # Gazebo environment
        gazebo_models = self.project_root / "sim" / "gazebo" / "models"
        gazebo_plugins = self.project_root / "sim" / "gazebo" / "plugins" / "build"
        
        os.environ['GZ_SIM_RESOURCE_PATH'] = f"{gazebo_models}:/usr/share/gazebo-11/models"
        os.environ['GZ_SIM_PLUGIN_PATH'] = str(gazebo_plugins)
        os.environ['LD_LIBRARY_PATH'] = f"{gazebo_plugins}:{os.environ.get('LD_LIBRARY_PATH', '')}"
        
        # ROS2 environment - source setup.bash and add library paths
        ros2_setup_cmd = 'source /opt/ros/jazzy/setup.bash && env'
        result = subprocess.run(['bash', '-c', ros2_setup_cmd], 
                              capture_output=True, text=True, check=False)
        
        if result.returncode == 0:
            # Parse environment variables from sourced ROS2 setup
            for line in result.stdout.split('\n'):
                if '=' in line and not line.startswith('_'):
                    key, value = line.split('=', 1)
                    os.environ[key] = value
        
        print("✅ Environment configured")
    
    def get_ros2_env(self):
        """Get environment dictionary with ROS2 setup"""
        env = os.environ.copy()
        
        # Ensure ROS2 library paths are included
        ros2_lib_path = '/opt/ros/jazzy/lib'
        current_ld_path = env.get('LD_LIBRARY_PATH', '')
        if ros2_lib_path not in current_ld_path:
            env['LD_LIBRARY_PATH'] = f"{ros2_lib_path}:{current_ld_path}"
        
        return env

    def launch_gazebo_simulation(self, gui=False, world="mmwave_minimal.sdf"):
        """Launch Gazebo with the specified world"""
        world_file = self.gazebo_worlds / world
        
        if not world_file.exists():
            print(f"❌ World file not found: {world_file}")
            return None
        
        cmd = ['gz', 'sim', '-r', '-v', '4', str(world_file)]
        if not gui:
            cmd.append('-s')  # Server mode (headless)
        
        print(f"🚀 Starting Gazebo {'with GUI' if gui else 'headless'}...")
        return subprocess.Popen(cmd)
    
    def launch_ros2_bridge(self):
        """Launch ROS2-Gazebo bridge"""
        cmd = [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/mmwave/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ]
        
        print("🌉 Starting ROS2-Gazebo bridge...")
        return subprocess.Popen(cmd, env=self.get_ros2_env())
    
    def launch_mmwave_mqtt_bridge(self):
        """Launch mmWave MQTT bridge"""
        bridge_script = self.project_root / "sim" / "ros2" / "mmwave_mqtt_bridge.py"
        
        if not bridge_script.exists():
            print(f"❌ mmWave bridge not found: {bridge_script}")
            return None
        
        cmd = ['python3', str(bridge_script)]
        print("📡 Starting mmWave MQTT bridge...")
        return subprocess.Popen(cmd, env=self.get_ros2_env())
    
    def launch_rviz(self):
        """Launch RViz2 with mmWave configuration"""
        config_file = self.project_root / "sim" / "ros2" / "config" / "olympus_rviz.rviz"
        
        cmd = ['ros2', 'run', 'rviz2', 'rviz2']
        if config_file.exists():
            cmd.extend(['-d', str(config_file)])
        
        print("👁️ Starting RViz2...")
        return subprocess.Popen(cmd, env=self.get_ros2_env())
    
    def launch_automation_demo(self):
        """Launch the automation demo controller"""
        automation_script = self.project_root / "automation" / "automation_demo.py"
        
        if not automation_script.exists():
            print(f"❌ Automation demo not found: {automation_script}")
            return None
        
        cmd = ['python3', str(automation_script)]
        print("🏠 Starting automation controller...")
        return subprocess.Popen(cmd)
    
    def run_simulation(self, mode="full", gui=False, rviz=False, automation=False):
        """Run the simulation with specified components"""
        print("🎯 OLYMPUS SIMULATION LAUNCHER")
        print("=" * 50)
        print(f"Mode: {mode}")
        print(f"GUI: {gui}")
        print(f"RViz: {rviz}")
        print(f"Automation: {automation}")
        print("=" * 50)
        
        # Setup environment
        self.setup_environment()
        
        processes = []
        
        try:
            if mode in ["full", "gazebo", "sensor"]:
                # Launch Gazebo
                gazebo_proc = self.launch_gazebo_simulation(gui=gui)
                if gazebo_proc:
                    processes.append(("Gazebo", gazebo_proc))
                    time.sleep(3)  # Wait for Gazebo to start
                
                # Launch ROS2 bridge
                bridge_proc = self.launch_ros2_bridge()
                if bridge_proc:
                    processes.append(("ROS2 Bridge", bridge_proc))
                    time.sleep(2)
                
                # Launch mmWave MQTT bridge
                if mode in ["full", "sensor"]:
                    mqtt_bridge_proc = self.launch_mmwave_mqtt_bridge()
                    if mqtt_bridge_proc:
                        processes.append(("mmWave MQTT Bridge", mqtt_bridge_proc))
                        time.sleep(1)
            
            # Launch RViz if requested
            if rviz and mode in ["full", "gazebo", "sensor"]:
                rviz_proc = self.launch_rviz()
                if rviz_proc:
                    processes.append(("RViz2", rviz_proc))
                    time.sleep(2)
            
            # Launch automation if requested
            if automation and mode in ["full", "automation", "sensor"]:
                auto_proc = self.launch_automation_demo()
                if auto_proc:
                    processes.append(("Automation", auto_proc))
            
            if not processes:
                print("❌ No processes started!")
                return
            
            print(f"\n✅ Started {len(processes)} processes:")
            for name, proc in processes:
                print(f"   - {name} (PID: {proc.pid})")
            
            print("\n🎮 Simulation running! Press Ctrl+C to stop all processes.")
            print("📊 Monitor topics: mosquitto_sub -t sensor/#")
            print("🧪 Test automation: python3 tests/test_automation_loop.py")
            
            # Wait for keyboard interrupt
            try:
                while True:
                    time.sleep(1)
                    # Check if any critical process died
                    for name, proc in processes:
                        if proc.poll() is not None:
                            print(f"⚠️ {name} process ended unexpectedly")
            except KeyboardInterrupt:
                print("\n\n👋 Shutting down simulation...")
        
        finally:
            # Clean shutdown
            for name, proc in processes:
                try:
                    print(f"🛑 Stopping {name}...")
                    proc.terminate()
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print(f"⚡ Force killing {name}...")
                    proc.kill()
                except Exception as e:
                    print(f"❌ Error stopping {name}: {e}")
            
            print("✅ All processes stopped")

def main():
    parser = argparse.ArgumentParser(description="Olympus Simulation Launcher")
    
    parser.add_argument('mode', nargs='?', default='full',
                       choices=['full', 'gazebo', 'sensor', 'automation'],
                       help='Simulation mode (default: full)')
    
    parser.add_argument('--gui', action='store_true',
                       help='Launch Gazebo with GUI (default: headless)')
    
    parser.add_argument('--rviz', action='store_true',
                       help='Launch RViz2 for visualization')
    
    parser.add_argument('--automation', action='store_true',
                       help='Launch automation controller')
    
    parser.add_argument('--list-modes', action='store_true',
                       help='List available simulation modes')
    
    args = parser.parse_args()
    
    if args.list_modes:
        print("Available simulation modes:")
        print("  full       - Complete simulation (Gazebo + ROS2 + MQTT + bridges)")
        print("  gazebo     - Gazebo simulation only (with ROS2 bridge)")
        print("  sensor     - Sensor simulation (Gazebo + bridges, no automation)")
        print("  automation - Automation controller only (requires running sensor)")
        return
    
    launcher = OlympusLauncher()
    launcher.run_simulation(
        mode=args.mode,
        gui=args.gui,
        rviz=args.rviz,
        automation=args.automation
    )

if __name__ == "__main__":
    main()
