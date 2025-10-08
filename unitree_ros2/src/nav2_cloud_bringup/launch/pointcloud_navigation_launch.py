#!/usr/bin/env python3

# Copyright (c) 2025 Custom Navigation Setup
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    use_pointcloud_direct = LaunchConfiguration('use_pointcloud_direct')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(os.path.dirname(__file__), 'nav2_params_online_fasterlio.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_pointcloud_topic_cmd = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/cloud_registered',
        description='PointCloud2 topic to use for navigation')

    declare_use_pointcloud_direct_cmd = DeclareLaunchArgument(
        'use_pointcloud_direct',
        default_value='true',
        description='Use PointCloud2 directly (true) or convert to LaserScan (false)')

    # Optional: PointCloud to LaserScan conversion node (if needed)
    # pointcloud_to_laserscan_node = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='pointcloud_to_laserscan',
    #     condition=lambda context: context.launch_configurations['use_pointcloud_direct'] == 'false',
    #     remappings=[
    #         ('cloud_in', pointcloud_topic),
    #         ('scan', '/scan')
    #     ],
    #     parameters=[{
    #         'target_frame': 'base_link',  # Adjust to your robot's base frame
    #         'transform_tolerance': 0.01,
    #         'min_height': 0.0,
    #         'max_height': 1.5,
    #         'angle_min': -3.14159,  # -180 degrees
    #         'angle_max': 3.14159,   # 180 degrees
    #         'angle_increment': 0.0087,  # ~0.5 degrees
    #         'scan_time': 0.1,
    #         'range_min': 0.1,
    #         'range_max': 100.0,
    #         'use_inf': True,
    #         'inf_epsilon': 1.0
    #     }]
    # )

    # Static transform publisher for map -> odom identity transform
    # This provides a basic map frame when no localization is available
    body_to_base_link_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_base_link',
        arguments=['0.02557', '0', '-0.04232', '0', '0', '0', 'body', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    # map --> camera_init --> body --> base_link
    # map_to_camera_init_transform = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_to_camera_init',
    #     arguments=['0', '0', '0', '1.5708', '0', '0', 'map', 'camera_init'],
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    map_to_camera_init_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    map_to_odom_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

# working ones: aug 18:
# ros2 run tf2_ros static_transform_publisher 0.02557 0 -0.04232 -1.5708 0 0 body base_link
# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_init
    # Alternative: If you want odom -> base_link directly (identity transform)
    # Uncomment this and comment out the odom_to_tf_node above if /Odometry topic is not available
    # odom_to_base_link_transform = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='odom_to_base_link',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # Launch Nav2 navigation stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'use_lifecycle_mgr': 'true',
                          'map_subscribe_transient_local': 'true'}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_pointcloud_topic_cmd)
    ld.add_action(declare_use_pointcloud_direct_cmd)

    # Add the nodes
    #ld.add_action(body_to_base_link_transform)
    ld.add_action(body_to_base_link_transform)
    ld.add_action(map_to_camera_init_transform)
    ld.add_action(map_to_odom_transform)
    ld.add_action(nav2_launch)

    return ld
