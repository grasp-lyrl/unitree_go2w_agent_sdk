#!/usr/bin/env python3

# Copyright (c) 2025 Custom Local-Only Navigation Setup
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('nav2_cloud_bringup')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')

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
        default_value=os.path.join(os.path.dirname(__file__), 'pointcloud_nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_pointcloud_topic_cmd = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/cloud_registered_body',
        description='PointCloud2 topic to use for navigation')

    # Transform publishers
    odom_to_base_link_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    odom_to_body_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_body',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'body'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Odom to TF node - converts odometry messages to TF transforms
    map_to_odom_transform = Node(
        package='odom_to_tf',
        executable='odom_to_tf',
        name='odom_to_tf',
        parameters=[
            {'odom_topic': '/Odometry'},
            {'parent_frame': 'map'},
            {'child_frame': 'odom'},
            {'use_sim_time': use_sim_time}
        ]
    )

    # LOCAL-ONLY Nav2 stack (NO planner_server, NO map_server, NO amcl)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        namespace=namespace
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        namespace=namespace
    )

    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        namespace=namespace
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        namespace=namespace
    )

    # Lifecycle manager for local-only navigation (NO planner_server in the list)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'bt_navigator',
                'recoveries_server',
                'waypoint_follower',
            ]}
        ],
        namespace=namespace
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_pointcloud_topic_cmd)

    # Add the transform nodes
    ld.add_action(map_to_odom_transform)
    ld.add_action(odom_to_base_link_transform)
    ld.add_action(odom_to_body_transform)

    # Add Nav2 nodes (LOCAL-ONLY)
    ld.add_action(controller_server)
    ld.add_action(bt_navigator)
    ld.add_action(recoveries_server)
    ld.add_action(waypoint_follower)
    ld.add_action(lifecycle_manager)

    return ld
