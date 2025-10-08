from setuptools import setup

package_name = 'nav2_cloud_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/nav2_cloud_bringup']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/nav2_cloud_sim.launch.py',
            'launch/pointcloud_navigation_launch.py',
            'launch/nav2_params_online_fasterlio.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yifei',
    maintainer_email='yifei@example.com',
    description='Nav2 bringup configured to use Faster-LIO point clouds',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'unitree_nav2 = nav2_cloud_bringup.unitree_nav2:main',
        ],
    },
)


