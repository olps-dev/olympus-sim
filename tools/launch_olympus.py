#!/usr/bin/env python3
"""
Unified Olympus Simulation Launcher
Single entry point for all simulation modes with clear options
"""

import os
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
        
        # WSL-specific environment setup
        if os.getenv('WSL_DISTRO_NAME') or os.path.exists('/proc/sys/fs/binfmt_misc/WSLInterop'):
            print("Setting up WSL-compatible Gazebo environment...")
            os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
            os.environ['MESA_GL_VERSION_OVERRIDE'] = '3.3'
            os.environ['GZ_RENDERING_ENGINE'] = 'ogre'
            os.environ['OGRE_RTT_MODE'] = 'Copy'
        
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
        
        print("Environment configured")
    
    def get_ros2_env(self):
        """Get environment dictionary with ROS2 setup"""
        env = os.environ.copy()
        
        # Ensure ROS2 library paths are included
        ros2_lib_path = '/opt/ros/jazzy/lib'
        current_ld_path = env.get('LD_LIBRARY_PATH', '')
        if ros2_lib_path not in current_ld_path:
            env['LD_LIBRARY_PATH'] = f"{ros2_lib_path}:{current_ld_path}"
        
        # Add Gazebo plugin path for mmWave sensor
        plugin_build_path = str(self.project_root / "sim" / "gazebo" / "plugins" / "build")
        current_plugin_path = env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        if plugin_build_path not in current_plugin_path:
            env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = f"{plugin_build_path}:{current_plugin_path}"
        
        return env

    def launch_gazebo_simulation(self, gui=False, world="mmwave_test.sdf"):
        """Launch Gazebo with the specified world"""
        # Use WSL-specific world file if in WSL
        if os.getenv('WSL_DISTRO_NAME') or os.path.exists('/proc/sys/fs/binfmt_misc/WSLInterop'):
            world = "mmwave_test_wsl.sdf"
            print("WSL detected: Using WSL-compatible world file")
        
        world_file = self.gazebo_worlds / world
        
        if not world_file.exists():
            print(f"ERROR: World file not found: {world_file}")
            return None
        
        cmd = ['gz', 'sim', '-r', '-v', '4', str(world_file)]
        if not gui:
            cmd.append('-s')  # Server mode (headless)
        
        print(f"Starting Gazebo {'with GUI' if gui else 'headless'}...")
        return subprocess.Popen(cmd, env=self.get_ros2_env())
    
    def launch_ros2_bridge(self):
        """Launch ROS2-Gazebo bridge for both mmwave sensors"""
        cmd = [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/mmwave/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/mmwave2/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ]
        
        print(" Starting ROS2-Gazebo bridge for dual sensors...")
        bridge_process = subprocess.Popen(cmd, env=self.get_ros2_env())
        
        # Start static transform publishers for both mmwave sensor frames
        tf_cmd1 = [
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '0', '0', '1', '0', '0', '0', 'world', 'mmwave'
        ]
        print(" Starting static transform publisher for mmwave frame...")
        tf_process1 = subprocess.Popen(tf_cmd1, env=self.get_ros2_env())
        
        tf_cmd2 = [
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '5', '0', '1', '0', '0', '3.14159', 'world', 'mmwave2'
        ]
        print(" Starting static transform publisher for mmwave2 frame...")
        tf_process2 = subprocess.Popen(tf_cmd2, env=self.get_ros2_env())
        
        return [bridge_process, tf_process1, tf_process2]
    
    def launch_mmwave_mqtt_bridge(self):
        """Launch multi-mmWave MQTT bridge"""
        bridge_script = self.project_root / "sim" / "ros2" / "multi_mmwave_mqtt_bridge.py"
        
        if not bridge_script.exists():
            print(f"ERROR: Multi-mmWave bridge not found: {bridge_script}")
            # Fall back to single sensor bridge
            bridge_script = self.project_root / "sim" / "ros2" / "mmwave_mqtt_bridge.py"
            if not bridge_script.exists():
                print(f"ERROR: mmWave bridge not found: {bridge_script}")
                return None
        
        cmd = ['python3', str(bridge_script)]
        print(" Starting multi-mmWave MQTT bridge...")
        return subprocess.Popen(cmd, env=self.get_ros2_env())
    
    def launch_rviz(self):
        """Launch RViz2 with mmWave configuration"""
        config_file = self.project_root / "sim" / "ros2" / "config" / "olympus_rviz.rviz"
        
        # Set up environment for ROS2
        env = self.get_ros2_env()
        
        # For WSL, ensure proper display setup
        env['DISPLAY'] = ':0'
        env['LIBGL_ALWAYS_SOFTWARE'] = '1'
        env['MESA_GL_VERSION_OVERRIDE'] = '3.3'
        
        cmd = ['ros2', 'run', 'rviz2', 'rviz2']
        if config_file.exists():
            cmd.extend(['-d', str(config_file)])
        
        print("Starting RViz2...")
        print(f"   Config: {config_file}")
        print(f"   Display: {env.get('DISPLAY')}")
        
        try:
            # Start RViz2 with proper environment
            process = subprocess.Popen(cmd, env=env)
            
            # Give it time to initialize
            import time
            time.sleep(3)
            
            # Check if still running
            if process.poll() is None:
                print(" RViz2 started successfully")
                return process
            else:
                print("ERROR: RViz2 failed to start")
                return None
        except Exception as e:
            print(f"ERROR: Failed to start RViz2: {e}")
            return None
    
    def launch_automation_demo(self):
        """Launch the automation demo controller"""
        automation_script = self.project_root / "automation" / "live_automation.py"
        
        if not automation_script.exists():
            print(f"ERROR: Automation controller not found: {automation_script}")
            return None
        
        cmd = ['python3', str(automation_script)]
        print(" Starting automation controller...")
        return subprocess.Popen(cmd)
    
    def launch_threejs_ros2_bridge(self):
        """Launch Three.js ROS2 bridge"""
        bridge_script = self.project_root / "sim" / "threejs" / "threejs_ros2_bridge.py"
        
        if not bridge_script.exists():
            print(f"ERROR: Three.js ROS2 bridge not found: {bridge_script}")
            return None
        
        cmd = ['python3', str(bridge_script)]
        print(" Starting Three.js ROS2 bridge on http://localhost:5555...")
        return subprocess.Popen(cmd, env=self.get_ros2_env())
    
    def launch_threejs_mqtt_bridge(self):
        """Launch Three.js MQTT bridge (direct mode)"""
        bridge_script = self.project_root / "sim" / "threejs" / "mqtt_bridge.py"
        
        if not bridge_script.exists():
            print(f"ERROR: Three.js MQTT bridge not found: {bridge_script}")
            return None
        
        cmd = ['python3', str(bridge_script)]
        print(" Starting Three.js MQTT bridge on http://localhost:5555...")
        return subprocess.Popen(cmd)
    
    def launch_dashboard(self):
        """Launch the web dashboard"""
        dashboard_script = self.project_root / "dashboard" / "app_simple.py"
        
        if not dashboard_script.exists():
            print(f"ERROR: Dashboard not found: {dashboard_script}")
            return None
        
        cmd = ['python3', str(dashboard_script)]
        print(" Starting web dashboard on http://localhost:5001...")
        return subprocess.Popen(cmd, cwd=str(self.project_root / "dashboard"))
    
    def run_simulation(self, mode="full", gui=False, rviz=False, automation=False, dashboard=False):
        """Run the simulation with specified components"""
        print(" OLYMPUS SIMULATION LAUNCHER")
        print("=" * 50)
        print(f"Mode: {mode}")
        print(f"GUI: {gui}")
        print(f"RViz: {rviz}")
        print(f"Automation: {automation}")
        print(f"Dashboard: {dashboard}")
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
                bridge_procs = self.launch_ros2_bridge()
                if bridge_procs:
                    processes.extend([("ROS2 Bridge", bridge_procs[0]), ("Static Transform Publisher 1", bridge_procs[1]), ("Static Transform Publisher 2", bridge_procs[2])])
                    time.sleep(2)
                
                # Launch mmWave MQTT bridge
                if mode in ["full", "sensor"]:
                    mqtt_bridge_proc = self.launch_mmwave_mqtt_bridge()
                    if mqtt_bridge_proc:
                        processes.append(("mmWave MQTT Bridge", mqtt_bridge_proc))
                        time.sleep(1)
            
            elif mode == "threejs":
                # Launch Three.js ROS2 bridge (default for scalability)
                threejs_bridge_proc = self.launch_threejs_ros2_bridge()
                if threejs_bridge_proc:
                    processes.append(("Three.js ROS2 Bridge", threejs_bridge_proc))
                    time.sleep(2)
                
                # Launch static transform publishers for Three.js sensor frames
                tf_cmd1 = [
                    'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                    '-5', '0', '1', '0', '0', '0', 'world', 'mmwave1_frame'
                ]
                print(" Starting static transform publisher for mmwave1 frame...")
                tf_process1 = subprocess.Popen(tf_cmd1, env=self.get_ros2_env())
                processes.append(("Static Transform Publisher mmwave1", tf_process1))
                
                tf_cmd2 = [
                    'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                    '5', '0', '1', '0', '0', '3.14159', 'world', 'mmwave2_frame'
                ]
                print(" Starting static transform publisher for mmwave2 frame...")
                tf_process2 = subprocess.Popen(tf_cmd2, env=self.get_ros2_env())
                processes.append(("Static Transform Publisher mmwave2", tf_process2))
                
                # Launch mmWave MQTT bridge to convert ROS2 to MQTT
                mqtt_bridge_proc = self.launch_mmwave_mqtt_bridge()
                if mqtt_bridge_proc:
                    processes.append(("mmWave MQTT Bridge", mqtt_bridge_proc))
                    time.sleep(1)
            
            # Launch RViz if requested
            if rviz and mode in ["full", "gazebo", "sensor", "threejs"]:
                rviz_proc = self.launch_rviz()
                if rviz_proc:
                    processes.append(("RViz2", rviz_proc))
                    time.sleep(2)
            
            # Launch automation if requested
            if automation and mode in ["full", "automation", "sensor", "threejs"]:
                auto_proc = self.launch_automation_demo()
                if auto_proc:
                    processes.append(("Automation", auto_proc))
            
            # Launch dashboard if requested
            if dashboard and mode in ["full", "sensor", "threejs"]:
                dashboard_proc = self.launch_dashboard()
                if dashboard_proc:
                    processes.append(("Dashboard", dashboard_proc))
                    time.sleep(2)
            
            if not processes:
                print("ERROR: No processes started!")
                return
            
            print(f"\n Started {len(processes)} processes:")
            for name, proc in processes:
                print(f"   - {name} (PID: {proc.pid})")
            
            print("\n Simulation running! Press Ctrl+C to stop all processes.")
            print(" Monitor topics: mosquitto_sub -t sensor/#")
            print(" Test automation: python3 tests/test_automation_loop.py")
            
            # Wait for keyboard interrupt
            try:
                while True:
                    time.sleep(1)
                    # Check if any critical process died
                    dead_processes = []
                    for name, proc in processes:
                        if proc.poll() is not None and name not in dead_processes:
                            print(f"WARNING: {name} process ended unexpectedly")
                            dead_processes.append(name)
                            # Remove from active processes to avoid repeated warnings
                            processes = [(n, p) for n, p in processes if n != name]
            except KeyboardInterrupt:
                print("\n\n Shutting down simulation...")
        
        finally:
            # Clean shutdown
            for name, proc in processes:
                try:
                    if proc.poll() is None:  # Process is still running
                        print(f"Stopping {name}...")
                        proc.terminate()
                        try:
                            proc.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            print(f"Force killing {name}...")
                            proc.kill()
                            proc.wait()  # Wait for kill to complete
                except Exception as e:
                    print(f"ERROR: Error stopping {name}: {e}")
            
            print("All processes stopped")

def main():
    parser = argparse.ArgumentParser(description="Olympus Simulation Launcher")
    
    parser.add_argument('mode', nargs='?', default='full',
                       choices=['full', 'gazebo', 'sensor', 'automation', 'threejs'],
                       help='Simulation mode (default: full)')
    
    parser.add_argument('--gui', action='store_true',
                       help='Launch Gazebo with GUI (default: headless)')
    
    parser.add_argument('--rviz', action='store_true',
                       help='Launch RViz2')
    
    parser.add_argument('--automation', action='store_true',
                       help='Launch automation controller')
    
    parser.add_argument('--dashboard', action='store_true',
                       help='Launch web dashboard on http://localhost:5001')
    
    parser.add_argument('--list-modes', action='store_true',
                       help='List available simulation modes')
    
    args = parser.parse_args()
    
    if args.list_modes:
        print("Available simulation modes:")
        print("  full       - Complete simulation (Gazebo + ROS2 + MQTT + bridges)")
        print("  gazebo     - Gazebo simulation only (with ROS2 bridge)")
        print("  sensor     - Sensor simulation (Gazebo + bridges, no automation)")
        print("  threejs    - Three.js browser simulation (ROS2 + MQTT bridges)")
        print("  automation - Automation controller only (requires running sensor)")
        print("")
        print("Platform compatibility:")
        print("  full/gazebo/sensor - Linux/Mac (native), WSL2 (limited)")
        print("  threejs           - Universal (any platform with browser)")
        return
    
    launcher = OlympusLauncher()
    launcher.run_simulation(
        mode=args.mode,
        gui=args.gui,
        rviz=args.rviz,
        automation=args.automation,
        dashboard=args.dashboard
    )

if __name__ == "__main__":
    main()
