#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Quick cleanup script for Unitree Go Robot processes
Kills all ROS1, ROS2, and related processes
"""

import subprocess
import sys

def cleanup_all_processes():
    """Kill all ROS processes"""
    print("🧹 Cleaning up all ROS processes...")
    
    processes_to_kill = [
        "roscore",
        "rosmaster", 
        "hesai_ros_driver",
        "faster_lio",
        "ros2",
        "realsense2_camera",
        "robot_state_publisher",
        "imu_converter",
        "relay_joint",
        "ros1_bridge",
        "static_transform_publisher",
        "nav2",
        "controller_server",
        "planner_server",
        "bt_navigator",
        "lifecycle_manager"
    ]
    
    killed_count = 0
    for process in processes_to_kill:
        try:
            result = subprocess.run(["pkill", "-f", process], 
                                  capture_output=True, text=True, check=False)
            if result.returncode == 0:
                print(f"✅ Killed {process} processes")
                killed_count += 1
            else:
                print(f"ℹ️  No {process} processes found")
        except Exception as e:
            print(f"❌ Error killing {process}: {e}")
    
    print(f"\n🎯 Cleanup complete! Killed {killed_count} process types.")
    
    # Show remaining processes
    print("\n📊 Remaining ROS-related processes:")
    try:
        subprocess.run(["ps", "aux"], check=False)
    except:
        pass

if __name__ == "__main__":
    cleanup_all_processes()
