#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Unified Launch Script for Unitree Go Robot with ROS1/ROS2 Bridge
Launches all necessary components with proper sequencing and ROS bridge integration
"""

import os
import sys
import subprocess
import time
import signal
import threading
import yaml
from pathlib import Path

class UnitreeRobotLauncher:
    def __init__(self, config_file="robot_config.yaml"):
        self.processes = []
        self.running = True
        
        # Get the script directory
        self.script_dir = Path(__file__).parent.absolute()
        
        # Define ROS1 and ROS2 workspaces
        self.ros1_ws = self.script_dir / "unitree_ros1"
        self.ros2_ws = self.script_dir / "unitree_ros2"
        
        # Load configuration
        self.config = self.load_config(config_file)
        
        # Check if workspaces exist
        if not self.ros1_ws.exists():
            print(f"[ERROR] ROS1 workspace not found at {self.ros1_ws}")
            sys.exit(1)
            
        if not self.ros2_ws.exists():
            print(f"[ERROR] ROS2 workspace not found at {self.ros2_ws}")
            sys.exit(1)
    
    def setup_environment(self):
        """Setup ROS1 and ROS2 environments"""
        print("[INFO] Setting up ROS environments...")
        
        # ROS1 environment
        ros1_setup = self.ros1_ws / "devel" / "setup.bash"
        if ros1_setup.exists():
            self.ros1_env = f"source {ros1_setup}"
            print("[OK] ROS1 environment found")
        else:
            print("[WARN] ROS1 workspace not built. Run: cd unitree_ros1 && catkin_make")
            self.ros1_env = ""
        
        # ROS2 environment
        setup_script = self.ros2_ws / "setup.sh"
        if setup_script.exists():
            self.setup_script_env = f"source {setup_script}"
            print("[OK] Setup script found")
        else:
            print("[WARN] Setup script not found")
            self.setup_script_env = ""
    
    def load_config(self, config_file):
        """Load configuration from YAML file"""
        config_path = self.script_dir / config_file
        
        if not config_path.exists():
            print(f"[WARN] Configuration file {config_file} not found. Using default settings.")
            return self.get_default_config()
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            print(f"[OK] Configuration loaded from {config_file}")
            return config
        except Exception as e:
            print(f"[ERROR] Failed to load configuration: {e}")
            print("[INFO] Using default configuration")
            return self.get_default_config()
    
    def get_default_config(self):
        """Return default configuration"""
        return {
            'core_infrastructure': {'ros1_core': True, 'ros_bridge': True},
            'realsense': {'enabled': True},
            'yolo': {'enabled': False},
            'arm_module': {'enabled': False},
            'lidar_driver': {'enabled': True},
            'slam': {'enabled': True},
            'navigation': {'enabled': True},
            'message_converters': {'imu_converter': True, 'tf_publishers': True},
            'delays': {
                'ros_bridge': 3, 'ros2_systems': 5, 'imu_converter': 2,
                'tf_publishers': 1, 'joint_relay': 1, 'lidar_driver': 2,
                'slam': 3, 'navigation': 5
            },
            'log_filtering': {
                'enabled': True,
                'suppress_verbose_logs': True,
                'suppress_timestamps': True
            }
        }
    
    def print_config_summary(self):
        """Print configuration summary"""
        print("\n📋 Configuration Summary:")
        print(f"   Realsense: {'✅' if self.config['realsense']['enabled'] else '❌'}")
        print(f"   YOLO: {'✅' if self.config['yolo']['enabled'] else '❌'}")
        print(f"   Arm Module: {'✅' if self.config['arm_module']['enabled'] else '❌'}")
        print(f"   LiDAR Driver: {'✅' if self.config['lidar_driver']['enabled'] else '❌'}")
        print(f"   SLAM (Faster-LIO): {'✅' if self.config['slam']['enabled'] else '❌'}")
        print(f"   Navigation: {'✅' if self.config['navigation']['enabled'] else '❌'}")
        print(f"   IMU Converter: {'✅' if self.config['message_converters']['imu_converter'] else '❌'}")
        print(f"   TF Publishers: {'✅' if self.config['message_converters']['tf_publishers'] else '❌'}")
        
        # Log filtering status
        log_filtering = self.config.get('log_filtering', {})
        if log_filtering.get('enabled', True):
            print(f"   Log Filtering: ✅ (Verbose logs suppressed)")
        else:
            print(f"   Log Filtering: ❌ (All logs shown)")
    
    def launch_component(self, component_name, launch_command, delay=0, env_type="ros2"):
        """Launch a component with proper environment"""
        if delay > 0:
            print(f"[WAIT] Waiting {delay} seconds before launching {component_name}...")
            time.sleep(delay)
        
        print(f"[LAUNCH] Launching {component_name}...")
        
        # Determine environment
        if env_type == "ros1":
            if not self.ros1_env:
                print(f"⚠️  Skipping ROS1 component: {component_name}")
                return
            cmd = f"bash -c '{self.ros1_env} && {launch_command}'"
        elif env_type == "ros2":
            if not self.setup_script_env:
                print(f"⚠️  Skipping ROS2 component: {component_name}")
                return
            
            # Create ROS2 launch command with CycloneDDS
            env_vars = ""
            if self.setup_script_env:
                env_vars += f"{self.setup_script_env} && "
            
            cmd = f"bash -c '{env_vars}{self.setup_script_env} && {launch_command}'"
        elif env_type == "bridge":
            # ROS bridge needs both environments
            if not self.ros1_env or not self.setup_script_env:
                print(f"⚠️  Skipping bridge component: {component_name}")
                return
            
            env_vars = ""
            if self.setup_script_env:
                env_vars += f"{self.setup_script_env} && "
            
            cmd = f"bash -c '{env_vars}{self.ros1_env} && {self.setup_script_env} && {launch_command}'"
        else:
            print(f"⚠️  Unknown environment type: {env_type}")
            return
        
        # Prepare environment for subprocess
        env = os.environ.copy()
        
        # Add PyTorch TLS fix environment variables for YOLO components
        if component_name == "YOLO":
            env.update({
                'OMP_NUM_THREADS': '1',
                'MKL_NUM_THREADS': '1',
                'OPENBLAS_NUM_THREADS': '1',
                'VECLIB_MAXIMUM_THREADS': '1',
                'NUMEXPR_NUM_THREADS': '1',
                'CUDA_VISIBLE_DEVICES': '',
                'AMENT_TRACE_SETUP_FILES': '0',
                'AMENT_PYTHON_EXECUTABLE': '/usr/bin/python3',
                'AMENT_PREFIX_PATH': '',
                'PYTHONPATH': '',
                'ROS_DISTRO': '',
                'ROS_PYTHON_VERSION': '',
                'ROS_VERSION': ''
            })
        
        # Start process
        process = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1,
            preexec_fn=os.setsid,  # Create new process group
            env=env  # Pass environment variables
        )
        
        self.processes.append(process)
        
        # Start output monitoring thread
        thread = threading.Thread(
            target=self.monitor_output,
            args=(process, component_name)
        )
        thread.daemon = True
        thread.start()
        
        print(f"✅ {component_name} started with PID {process.pid}")
    
    def monitor_output(self, process, component_name):
        """Monitor process output with filtering"""
        try:
            for line in iter(process.stdout.readline, ''):
                if line and self.running:
                    line_stripped = line.strip()
                    
                    # Filter out verbose logs
                    if self.should_filter_line(line_stripped, component_name):
                        continue
                    
                    print(f"[{component_name}] {line_stripped}")
        except:
            pass
    
    def should_filter_line(self, line, component_name):
        """Determine if a line should be filtered out"""
        # Check if log filtering is enabled
        log_filtering = self.config.get('log_filtering', {})
        if not log_filtering.get('enabled', True):
            return False
        
        # Filter LiDAR driver frame logs
        if component_name == "LiDAR-Driver" and "frame:" in line and "points:" in line and "start time:" in line:
            return True
        
        # Filter SLAM verbose mapping logs
        if component_name == "SLAM-FasterLIO" and "[ mapping ]:" in line and "In num:" in line:
            return True
        
        # Filter ROS2 verbose logs
        if "I0421" in line and "laser_mapping.cc" in line:
            return True
        
        # Filter timestamp-heavy logs
        if log_filtering.get('suppress_timestamps', True):
            if any(pattern in line for pattern in [
                "start time:", "end time:", "downsamp", "Map grid num:", "effect num"
            ]):
                return True
        
        # Filter verbose logs
        if log_filtering.get('suppress_verbose_logs', True):
            if any(pattern in line for pattern in [
                "frame:", "points:", "packet:", "In num:", "downsamp", "Map grid num:", "effect num"
            ]):
                return True
        
        return False
    
    def launch_all_components(self):
        """Launch all robot components in proper sequence based on configuration"""
        print("🤖 Starting Unitree Go Robot System with ROS Bridge...")
        
        # Print configuration summary
        self.print_config_summary()
        
        # Phase 1: Core Infrastructure
        print("\n📡 Phase 1: Starting Core Infrastructure...")
        
        # 1. ROS1 Core (always required)
        if self.config['core_infrastructure']['ros1_core']:
            self.launch_component(
                "ROS1-Core",
                "roscore",
                delay=0,
                env_type="ros1"
            )
        
        # 2. ROS Bridge (always required)
        if self.config['core_infrastructure']['ros_bridge']:
            self.launch_component(
                "ROS-Bridge",
                "ros2 run ros1_bridge dynamic_bridge --bridge-all-topics",
                delay=self.config['delays']['ros_bridge'],
                env_type="bridge"
            )
        
        # Phase 2: ROS2 Systems
        print("\n🔧 Phase 2: Starting ROS2 Systems...")
        
        # 3. ROS2 Systems Launch, this includes realsense, yolo, piper
        if self.config['realsense']['enabled']:
            self.launch_component(
                "Realsense",
                f"cd {self.ros2_ws} && ros2 launch launch/realsense.launch.py",
                delay=self.config['delays']['ros2_systems'],
                env_type="ros2"
            )
        
        if self.config['yolo']['enabled']:
            # Get YOLO workspace and launch file
            yolo_ws = self.config['yolo'].get('workspace', '/home/unitree/yolo_ws')
            launch_file = self.config['yolo'].get('launch_file', 'yolov9.launch.py')
            
            self.launch_component(
                "YOLO",
                f"cd {yolo_ws} && ros2 launch yolov8_bringup {launch_file} model:={self.config['yolo']['model_weights']} input_image_topic:={self.config['yolo']['input_image_topic']} device:={self.config['yolo']['device']} image_reliability:={self.config['yolo']['image_reliability']} threshold:={self.config['yolo']['threshold']}",
                delay=self.config['delays']['ros2_systems'],
                env_type="ros2"
            )
        
        # Phase 3: Message Converters and TF Publishers
        print("\n🔄 Phase 3: Starting Message Converters...")
        
        # 4. IMU Converter
        if self.config['message_converters']['imu_converter']:
            self.launch_component(
                "IMU-Converter",
                f"cd {self.ros2_ws} && ros2 run unitree_msg_converter imu_converter",
                delay=self.config['delays']['imu_converter'],
                env_type="ros2"
            )
        
        # 5. TF Publishers
        if self.config['message_converters']['tf_publishers']:
            self.launch_component(
                "TF-Publishers",
                f"cd {self.ros2_ws} && ros2 launch launch/ros2_tf_publishers.launch.py",
                delay=self.config['delays']['tf_publishers'],
                env_type="ros2"
            )
        
        # Phase 3.5: Arm Module
        if self.config['arm_module']['enabled']:
            print("\n🦾 Phase 3.5: Starting Arm Module...")
            
            # Joint Relay (part of arm module)
            self.launch_component(
                "Joint-Relay",
                f"cd {self.ros2_ws} && python3 src/unitree_msg_converter/unitree_msg_converter/relay_joint.py",
                delay=self.config['delays']['joint_relay'],
                env_type="ros2"
            )
            
            # Piper Arm (placeholder for future integration)
            self.launch_component(
                "Piper-Arm",
                f"cd {self.ros2_ws} && ros2 launch piper_arm piper_arm.launch.py",
                delay=2,
                env_type="ros2"
            )
            
            # Run robot state publisher for piper
            self.launch_component(
                "Piper-Robot-State-Publisher",
                f"ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"$(xacro $(ros2 pkg prefix piper_description)/share/piper_description/urdf/piper_description.xacro)\"",
                delay=1,
                env_type="ros2"
            )
        
        # Phase 4: ROS1 Hardware Drivers
        print("\n🔌 Phase 4: Starting Hardware Drivers...")
        
        # 7. LiDAR Driver
        if self.config['lidar_driver']['enabled']:
            driver = self.config['lidar_driver'].get('driver', 'hesai_ros_driver_node')
            self.launch_component(
                "LiDAR-Driver",
                f"cd {self.ros1_ws} && rosrun hesai_ros_driver {driver}",
                delay=self.config['delays']['lidar_driver'],
                env_type="ros1"
            )
        
        # Phase 5: SLAM and Navigation
        print("\n🗺️  Phase 5: Starting SLAM and Navigation...")
        
        # 8. SLAM (Faster-LIO)
        if self.config['slam']['enabled']:
            algorithm = self.config['slam'].get('algorithm', 'faster_lio')
            launch_file = self.config['slam'].get('launch_file', 'mapping_hesai.launch')
            rviz_enabled = self.config['slam'].get('rviz', False)
            
            self.launch_component(
                "SLAM-FasterLIO",
                f"cd {self.ros1_ws} && roslaunch {algorithm} {launch_file} rviz:={str(rviz_enabled).lower()}",
                delay=self.config['delays']['slam'],
                env_type="ros1"
            )
        
        # 9. Navigation
        if self.config['navigation']['enabled']:
            params_file = self.config['navigation'].get('params_file', 'src/nav2_cloud_bringup/launch/nav2_params_online_fasterlio.yaml')
            self.launch_component(
                "Navigation",
                f"cd {self.ros2_ws} && ros2 launch nav2_bringup navigation_launch.py params_file:={params_file}",
                delay=self.config['delays']['navigation'],
                env_type="ros2"
            )
        
        # Phase 6: YOLO Object Detection
        if self.config['yolo']['enabled']:
            print("\n👁️  Phase 6: Starting YOLO Object Detection...")
            self.launch_component(
                "YOLO-Detection",
                f"cd {self.ros2_ws} && ros2 run yolo_detection yolo_node",
                delay=3,
                env_type="ros2"
            )
        
        print("\n✅ All components launched!")
        print("📊 System Status:")
        print(f"   - Total Processes: {len(self.processes)}")
        print("   - ROS Bridge: Active")
        print("   - Data Flow: ROS1 ↔ ROS2")
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print("\n🛑 Shutting down robot system...")
        self.running = False
        
        # Terminate all processes using process groups
        for i, process in enumerate(self.processes):
            try:
                if process.poll() is None:  # Process is still running
                    print(f"[TERMINATE] Terminating process group {i+1} (PID {process.pid})")
                    # Kill the entire process group
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except Exception as e:
                print(f"[ERROR] Failed to terminate process group {i+1}: {e}")
                # Fallback to regular terminate
                try:
                    process.terminate()
                except:
                    pass
        
        # Wait for processes to terminate gracefully
        print("[WAIT] Waiting for processes to terminate gracefully...")
        time.sleep(2)
        
        # Force kill if still running
        for i, process in enumerate(self.processes):
            try:
                if process.poll() is None:  # Process is still running
                    print(f"[KILL] Force killing process group {i+1} (PID {process.pid})")
                    # Kill the entire process group with SIGKILL
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception as e:
                print(f"[ERROR] Failed to kill process group {i+1}: {e}")
                # Fallback to regular kill
                try:
                    process.kill()
                except:
                    pass
        
        # Additional cleanup - kill any remaining ROS processes
        print("[CLEANUP] Cleaning up remaining ROS processes...")
        try:
            # Kill ROS1 processes
            subprocess.run(["pkill", "-f", "roscore"], check=False)
            subprocess.run(["pkill", "-f", "rosmaster"], check=False)
            subprocess.run(["pkill", "-f", "hesai_ros_driver"], check=False)
            subprocess.run(["pkill", "-f", "faster_lio"], check=False)
            
            # Kill ROS2 processes
            subprocess.run(["pkill", "-f", "ros2"], check=False)
            subprocess.run(["pkill", "-f", "realsense2_camera"], check=False)
            subprocess.run(["pkill", "-f", "robot_state_publisher"], check=False)
            subprocess.run(["pkill", "-f", "imu_converter"], check=False)
            subprocess.run(["pkill", "-f", "relay_joint"], check=False)
            
            # Kill bridge processes
            subprocess.run(["pkill", "-f", "ros1_bridge"], check=False)
            
        except Exception as e:
            print(f"[ERROR] Error during cleanup: {e}")
        
        print("✅ All processes terminated")
        sys.exit(0)
    
    def cleanup_all_processes(self):
        """Manual cleanup method for killing all ROS processes"""
        print("🧹 Manual cleanup: Killing all ROS processes...")
        
        try:
            # Kill ROS1 processes
            subprocess.run(["pkill", "-f", "roscore"], check=False)
            subprocess.run(["pkill", "-f", "rosmaster"], check=False)
            subprocess.run(["pkill", "-f", "hesai_ros_driver"], check=False)
            subprocess.run(["pkill", "-f", "faster_lio"], check=False)
            
            # Kill ROS2 processes
            subprocess.run(["pkill", "-f", "ros2"], check=False)
            subprocess.run(["pkill", "-f", "realsense2_camera"], check=False)
            subprocess.run(["pkill", "-f", "robot_state_publisher"], check=False)
            subprocess.run(["pkill", "-f", "imu_converter"], check=False)
            subprocess.run(["pkill", "-f", "relay_joint"], check=False)
            
            # Kill bridge processes
            subprocess.run(["pkill", "-f", "ros1_bridge"], check=False)
            
            print("✅ Manual cleanup completed")
            
        except Exception as e:
            print(f"[ERROR] Error during manual cleanup: {e}")
    
    def run(self):
        """Main run method"""
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Setup environment
        self.setup_environment()
        
        # Launch all components
        self.launch_all_components()

        print("🤖 All components launched!")
        print("📊 System Status:")
        print(f"   - Total Processes: {len(self.processes)}")
        print("   - ROS Bridge: Active")
        print("   - Data Flow: ROS1 ↔ ROS2")
        
        # Keep running until interrupted
        try:
            while self.running:
                time.sleep(1)
                
                # Check if any process died
                dead_processes = []
                for i, process in enumerate(self.processes):
                    if process.poll() is not None:
                        dead_processes.append(i)
                
                # Remove dead processes
                for i in reversed(dead_processes):
                    self.processes.pop(i)
                
                if not self.processes:
                    print("⚠️  All processes have stopped")
                    break
                    
        except KeyboardInterrupt:
            self.signal_handler(signal.SIGINT, None)

def main():
    """Main entry point"""
    print("🤖 Unitree Go Robot Unified Launcher with ROS Bridge")
    print("=" * 60)
    
    # Check for cleanup command
    if len(sys.argv) > 1 and sys.argv[1] == "--cleanup":
        print("🧹 Running cleanup mode...")
        launcher = UnitreeRobotLauncher()
        launcher.cleanup_all_processes()
        return
    
    # Allow custom config file via command line argument
    config_file = "robot_config.yaml"
    if len(sys.argv) > 1:
        config_file = sys.argv[1]
        print(f"[INFO] Using custom config file: {config_file}")
    
    launcher = UnitreeRobotLauncher(config_file)
    launcher.run()

if __name__ == "__main__":
    main()
