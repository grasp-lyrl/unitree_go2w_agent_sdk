from setuptools import setup

package_name = 'unitree_msg_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yifei',
    maintainer_email='yishao@seas.upenn.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_converter = unitree_msg_converter.imu_converter:main',
            'joycon_converter = unitree_msg_converter.joycon_converter:main',
            'tf_pub = unitree_msg_converter.tf_pub:main',
            'relay_joint = unitree_msg_converter.relay_joint:main',,
        ],
    },
)
