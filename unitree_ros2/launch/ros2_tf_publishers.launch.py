import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = False

    body_to_base_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_base',
        arguments=['0.02557', '0', '-0.04232', '-1.5708', '0', '0', 'body', 'base'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
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
        arguments=['0', '0', '0', '-1.5708', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # transform from dog base (base) to arm base (base_link)
    dog_to_arm_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='dog_to_arm',
        arguments=['0', '0', '0.057', '0', '0', '0', 'base', 'world'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    link6_to_camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link6_to_camera',
        arguments=['0.0175', '0.1454', '0.096', '0.45451948', '0.45451948', '0.54167522', '-0.54167522', 'link6', 'camera_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )



    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(body_to_base_transform)
    ld.add_action(map_to_camera_init_transform)
    ld.add_action(map_to_odom_transform)
    ld.add_action(dog_to_arm_transform)
    ld.add_action(link6_to_camera_transform)
    return ld
