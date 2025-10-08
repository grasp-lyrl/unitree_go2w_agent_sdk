import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolov8_bringup'),
                'launch',
                'yolov8.launch.py'
            )
        ),
        launch_arguments={'model': 'yolov8m-seg.pt', 'input_image_topic' : '/camera/color/image_raw', 'device' : 'cuda:0'}.items()
    )

    imu_converter = ExecuteProcess(
        cmd=['ros2', 'run', 'unitree_msg_converter', 'imu_converter'],
        output='screen'
    )

    realsense_camera = ExecuteProcess(
        cmd=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', 'enable_rgbd:=true', 'enable_sync:=true', 'align_depth.enable:=true', 'enable_color:=true', 'enable_depth:=true', 'use_system_time:=true', 'use_device_time:=false', 'depth_module.profile:=640x480x6', 'rgb_camera.profile:=640x480x6'], #, 'pointcloud.enable:=true'
        output='screen'
    )

    realsense_pointcloud = ExecuteProcess(
        cmd=['ros2', 'run', 'realsense2_camera', 'realsense2_camera_node', '--ros=args -p', 'pointcloud.enable:=true',],
        output='screen'
    )


    piper_arm = ExecuteProcess(
        cmd=['ros2', 'launch', 'piper', 'start_single_piper.launch.py'],
        output='screen'
    )

    tf_camera_to_base = ExecuteProcess(
        #initial_cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0.01790760861365421', '0.13882315060757172', '0.10302285961063637', '0.45451948', '0.45451948', '0.54167522', '-0.54167522', 'link6', 'camera_link'],
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0.0175', '0.1254', '0.116', '0.45451948', '0.45451948', '0.54167522', '-0.54167522', 'link6', 'camera_link'],
        output='screen'
    )

    return LaunchDescription([
        #piper_arm,
    	realsense_camera,
    	#realsense_pointcloud,
        #yolov8_launch,
        #imu_converter,
        #tf_camera_to_base,
    ]) 
