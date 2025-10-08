import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    realsense_camera = ExecuteProcess(
        cmd=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', 'enable_rgbd:=true', 'enable_sync:=true', 'align_depth.enable:=true', 'enable_color:=true', 'enable_depth:=true', 'use_system_time:=true', 'use_device_time:=false', 'depth_module.profile:=640x480x6', 'rgb_camera.profile:=640x480x6'], #, 'pointcloud.enable:=true'
        output='screen'
    )

    realsense_pointcloud = ExecuteProcess(
        cmd=['ros2', 'run', 'realsense2_camera', 'realsense2_camera_node', '--ros=args -p', 'pointcloud.enable:=true',],
        output='screen'
    )

    return LaunchDescription([
    	realsense_camera,
    	#realsense_pointcloud,
    ]) 
