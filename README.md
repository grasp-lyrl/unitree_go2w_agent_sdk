# Unitree Go Robot ROS Packages Installation Guide

This repository contains comprehensive ROS packages for the Unitree Go robot system, supporting both ROS1 and ROS2 environments. The system includes SLAM, navigation, object detection, robotic arm control, and camera integration capabilities.

## Table of Contents
- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Quick Start](#quick-start)
- [ROS1 Installation](#ros1-installation)
- [ROS2 Installation](#ros2-installation)
- [Package Overview](#package-overview)
- [Usage Examples](#usage-examples)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Advanced Usage](#advanced-usage)

## Overview

This repository provides a complete robotics software stack for the Unitree Go robot, featuring:

- **SLAM & Navigation**: Faster-LIO for high-performance LiDAR-inertial odometry
- **Object Detection**: YOLOv8/YOLOv9 with 3D capabilities
- **Robotic Arm Control**: Piper arm with CAN-based communication
- **Camera Integration**: Intel RealSense RGB-D cameras
- **Multi-Sensor Fusion**: LiDAR, IMU, camera, and depth sensors

## System Requirements

### Hardware Requirements
- **Robot Platform**: Unitree Go robot
- **Computing Platform**: NVIDIA Jetson (recommended) or x86_64 system
- **Sensors**: 
  - LiDAR (Hesai, Velodyne, Ouster, or Livox)
  - Intel RealSense camera (D400 series or T265)
  - IMU/Gyroscope
- **Interfaces**: CAN interface for robotic arm control

### Software Requirements

#### ROS1 Environment
- **OS**: Ubuntu 18.04 (ROS Melodic) or Ubuntu 20.04 (ROS Noetic)
- **ROS**: ROS Melodic or ROS Noetic
- **Compiler**: GCC 9+ (for Ubuntu 18.04)

#### ROS2 Environment
- **OS**: Ubuntu 20.04 (ROS2 Foxy) or Ubuntu 22.04 (ROS2 Humble)
- **ROS**: ROS2 Foxy or ROS2 Humble
- **CUDA**: For YOLO object detection acceleration

## Quick Start

### Choose Your ROS Version

```bash
# Clone the repository
git clone <repository-url> unitree_go_jetson
cd unitree_go_jetson

# For ROS1 (Ubuntu 18.04/20.04)
cd unitree_ros1
source /opt/ros/noetic/setup.bash  # or melodic
catkin build
source devel/setup.bash

# For ROS2 (Ubuntu 20.04/22.04)
cd unitree_ros2
source /opt/ros/foxy/setup.bash  # or humble
colcon build --symlink-install
source install/setup.bash
```

## ROS1 Installation

### 1. System Dependencies

```bash
# Update package lists
sudo apt update

# Install ROS1 dependencies
sudo apt install -y \
    ros-$ROS_DISTRO-catkin \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-tf \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-message-generation \
    ros-$ROS_DISTRO-message-runtime \
    ros-$ROS_DISTRO-actionlib \
    ros-$ROS_DISTRO-actionlib-msgs \
    ros-$ROS_DISTRO-control-msgs \
    ros-$ROS_DISTRO-trajectory-msgs

# Install Faster-LIO dependencies
sudo apt install -y \
    libgoogle-glog-dev \
    libeigen3-dev \
    libpcl-dev \
    libyaml-cpp-dev

# Install Piper robotic arm dependencies
sudo apt install -y \
    python3-can \
    python3-wstool \
    python3-catkin-tools \
    python3-rosdep \
    ros-$ROS_DISTRO-ruckig \
    ros-$ROS_DISTRO-eigen-stl-containers \
    ros-$ROS_DISTRO-geometric-shapes \
    ros-$ROS_DISTRO-pybind11-catkin \
    ros-$ROS_DISTRO-moveit-resources-panda-moveit-config \
    ros-$ROS_DISTRO-ompl \
    ros-$ROS_DISTRO-warehouse-ros \
    ros-$ROS_DISTRO-eigenpy \
    ros-$ROS_DISTRO-rosparam-shortcuts \
    ros-$ROS_DISTRO-moveit-msgs \
    ros-$ROS_DISTRO-srdfdom

# Install Python dependencies
pip3 install piper_sdk
```

### 2. Compiler Setup (Ubuntu 18.04)

```bash
# Upgrade GCC for C++17 support
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-9 g++-9
cd /usr/bin
sudo rm gcc g++
sudo ln -s gcc-9 gcc
sudo ln -s g++-9 g++
```

### 3. Build ROS1 Workspace

```bash
cd unitree_ros1

# Initialize catkin workspace
catkin init

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
catkin build

# Source workspace
source devel/setup.bash
```

## ROS2 Installation

### 1. System Dependencies

```bash
# Update package lists
sudo apt update

# Install ROS2 dependencies
sudo apt install -y \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-nav2-costmap-2d \
    ros-$ROS_DISTRO-nav2-controller \
    ros-$ROS_DISTRO-nav2-planner \
    ros-$ROS_DISTRO-nav2-bt-navigator \
    ros-$ROS_DISTRO-nav2-waypoint-follower \
    ros-$ROS_DISTRO-nav2-behavior-tree \
    ros-$ROS_DISTRO-odom-to-tf \
    ros-$ROS_DISTRO-rclpy \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-diagnostic-updater

# Install RealSense dependencies
sudo apt install -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg

# Install CycloneDDS
sudo apt install -y \
    ros-$ROS_DISTRO-rmw-cyclonedx-cpp \
    ros-$ROS_DISTRO-cyclonedx-cpp

# Install Python dependencies
pip3 install -r unitree_ros2/src/yolo_ros/requirements.txt
pip3 install python-can piper_sdk
```

### 2. CUDA Setup (for YOLO)

```bash
# Install CUDA toolkit
sudo apt install -y nvidia-cuda-toolkit

# Verify installation
nvcc --version
nvidia-smi
```

### 3. Build ROS2 Workspace

```bash
cd unitree_ros2

# Source ROS2
source /opt/ros/$ROS_DISTRO/setup.bash

# Build CycloneDDS workspace
cd cyclonedx_ws
colcon build --symlink-install
source install/setup.bash
cd ..

# Build main workspace
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 4. Environment Configuration

Create setup script:

```bash
cat > unitree_ros2/setup_env.sh << 'EOF'
#!/bin/bash
echo "Setup unitree ros2 environment"
export GEMINI_KEY='AIzaSyBP8ctJEwHanxQnO3hdPNWqBaNKAfqtrVA'
source /opt/ros/foxy/setup.bash
source ~/unitree_go_jetson/unitree_ros2/cyclonedx_ws/install/setup.bash
source ~/unitree_go_jetson/unitree_ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export CYCLONEDX_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                            </Interfaces></General><Discovery>
                        </Discovery></Domain></CycloneDDS>'
EOF

chmod +x unitree_ros2/setup_env.sh
```

## Package Overview

### ROS1 Packages

#### 1. faster-lio
- **Purpose**: High-performance LiDAR-inertial odometry
- **Features**: 
  - 1k-2k Hz processing for solid-state LiDARs
  - Support for multiple LiDAR types
  - Real-time mapping and localization
- **Dependencies**: PCL, Eigen, glog, yaml-cpp

#### 2. HesaiLidar_ROS_2.0
- **Purpose**: ROS driver for Hesai LiDAR sensors
- **Features**: Real-time point cloud publishing, configurable parameters
- **Dependencies**: ROS sensor_msgs, std_msgs

#### 3. piper_ros
- **Purpose**: Piper robotic arm control
- **Features**: CAN communication, MoveIt integration, gripper control
- **Dependencies**: python-can, piper_sdk, MoveIt

### ROS2 Packages

#### 1. nav2_cloud_bringup
- **Purpose**: Nav2 navigation with point cloud integration
- **Features**: Local/global planning, Faster-LIO integration, RViz configs
- **Dependencies**: Nav2 stack, TF2, sensor_msgs

#### 2. yolo_ros
- **Purpose**: YOLOv8/YOLOv9 object detection and tracking
- **Features**: Real-time detection, 3D object detection, pose estimation
- **Dependencies**: OpenCV, Ultralytics, CUDA

#### 3. realsense-ros
- **Purpose**: Intel RealSense camera integration
- **Features**: RGB-D support, IMU integration, calibration
- **Dependencies**: librealsense2, OpenCV

#### 4. unitree_msg_converter
- **Purpose**: Message conversion utilities
- **Features**: ROS1 to ROS2 conversion, custom message types
- **Dependencies**: ROS2 message types

## Usage Examples

### ROS1 Usage

#### SLAM with Faster-LIO

```bash
# Source environment
source unitree_ros1/devel/setup.bash

# Launch SLAM with different LiDAR types
roslaunch faster_lio mapping_hesai.launch    # Hesai LiDAR
roslaunch faster_lio mapping_velodyne.launch   # Velodyne LiDAR
roslaunch faster_lio mapping_avia.launch      # Livox Avia

# Offline processing
./unitree_ros1/build/devel/lib/faster_lio/run_mapping_offline \
    --bag_file /path/to/bag/file \
    --config_file ./config/avia.yaml
```

#### Robotic Arm Control

```bash
# Setup CAN interface
bash unitree_ros1/src/piper_ros/can_activate.sh can0 1000000

# Launch arm control
roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true

# Control commands
rosservice call /enable_srv "enable_request: true"
rostopic pub /joint_states sensor_msgs/JointState "
header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['']
position: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.01]
velocity: [0,0,0,0,0,0,10]
effort: [0,0,0,0,0,0,0.5]"
```

### ROS2 Usage

#### Navigation System

```bash
# Source environment
source unitree_ros2/setup_env.sh

# Launch navigation
ros2 launch nav2_cloud_bringup pointcloud_navigation_launch.py

# Launch with simulation
ros2 launch nav2_cloud_bringup nav2_cloud_sim.launch.py
```

#### Object Detection

```bash
# Launch YOLOv8 detection
ros2 launch yolov8_bringup yolov8.launch.py

# 3D object detection
ros2 launch yolov8_bringup yolov8_3d.launch.py

# Custom model
ros2 launch yolov8_bringup yolov8.launch.py model:=yolov8m-seg.pt
```

#### Camera Integration

```bash
# Launch RealSense camera
ros2 launch realsense2_camera rs_launch.py

# With specific configuration
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true \
    enable_imu:=true
```

## Configuration

### Environment Variables

#### ROS1 Configuration
```bash
# Add to ~/.bashrc
echo "source ~/unitree_go_jetson/unitree_ros1/devel/setup.bash" >> ~/.bashrc
```

#### ROS2 Configuration
```bash
# Network configuration
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# CycloneDDS configuration
export CYCLONEDX_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                            </Interfaces></General><Discovery>
                        </Discovery></Domain></CycloneDDS>'

# Gemini API key (for AI features)
export GEMINI_KEY='your_gemini_api_key_here'
```

### Hardware Configuration

#### CAN Interface Setup
```bash
# Single CAN device
bash can_activate.sh can0 1000000

# Multiple CAN devices
bash find_all_can_port.sh
bash can_activate.sh can_piper 1000000 "1-2:1.0"
```

#### Camera Permissions
```bash
# RealSense camera permissions
sudo chmod 666 /dev/bus/usb/$(lsusb | grep -i intel | awk '{print $2"/"$4}' | sed 's/://')
```

## Troubleshooting

### Common Issues

#### 1. Compilation Errors

**ROS1 Faster-LIO compilation fails**:
```bash
# For Ubuntu 18.04, use provided TBB library
cd unitree_ros1/src/faster-lio/thirdparty
tar -xvf tbb2018_20170726oss_lin.tgz
cd ../../build
cmake .. -DCUSTOM_TBB_DIR=`pwd`/../src/faster-lio/thirdparty/tbb2018_20170726oss
```

**ROS2 build fails**:
```bash
# Clean and rebuild
rm -rf unitree_ros2/build/ unitree_ros2/install/ unitree_ros2/log/
colcon build --symlink-install --event-handlers console_direct+
```

#### 2. Hardware Issues

**CAN device not found**:
```bash
# Check CAN devices
bash find_all_can_port.sh
ifconfig | grep can

# Verify CAN activation
bash can_activate.sh can0 1000000
```

**Camera not detected**:
```bash
# Test with RealSense tools
realsense-viewer

# Check camera info
ros2 topic echo /camera/color/camera_info
```

**CUDA/YOLO issues**:
```bash
# Verify CUDA installation
nvidia-smi
nvcc --version

# Test PyTorch CUDA support
python3 -c "import torch; print(torch.cuda.is_available())"
```

#### 3. Communication Issues

**ROS2 nodes cannot communicate**:
```bash
# Check DDS implementation
echo $RMW_IMPLEMENTATION

# Verify network configuration
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
```

**MoveIt compilation issues**:
```bash
# Install official MoveIt
sudo apt install ros-$ROS_DISTRO-moveit

# Remove problematic directory
rm -rf unitree_ros1/src/piper_ros/src/piper_moveit/moveit-1.1.11
```

### Performance Optimization

#### System Optimization
```bash
# Set CPU governor to performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Increase network buffer sizes
sudo sysctl -w net.core.rmem_max=16777216
sudo sysctl -w net.core.wmem_max=16777216
```

#### YOLO Performance
```bash
# Use smaller model for faster inference
ros2 launch yolov8_bringup yolov8.launch.py model:=yolov8n.pt

# Adjust detection threshold
ros2 launch yolov8_bringup yolov8.launch.py threshold:=0.3
```

## Advanced Usage

### Multi-Robot Systems

#### ROS2 Multi-Robot Setup
```bash
# Robot 1
export ROS_DOMAIN_ID=1
ros2 launch nav2_cloud_bringup pointcloud_navigation_launch.py

# Robot 2
export ROS_DOMAIN_ID=2
ros2 launch nav2_cloud_bringup pointcloud_navigation_launch.py
```

### Custom Launch Files

#### Combined System Launch
```python
# custom_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # YOLO detection
        Node(
            package='yolov8_ros',
            executable='yolov8_node',
            name='yolov8_node'
        ),
        
        # RealSense camera
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera'
        ),
        
        # Navigation
        Node(
            package='nav2_cloud_bringup',
            executable='unitree_nav2.py',
            name='nav2_node'
        )
    ])
```

### Integration Examples

#### Complete Robot Stack
```bash
# Terminal 1: SLAM
source unitree_ros1/devel/setup.bash
roslaunch faster_lio mapping_hesai.launch

# Terminal 2: Navigation
source unitree_ros2/setup_env.sh
ros2 launch nav2_cloud_bringup pointcloud_navigation_launch.py

# Terminal 3: Object Detection
ros2 launch yolov8_bringup yolov8_3d.launch.py

# Terminal 4: Robotic Arm
ros2 run piper_ros piper_ctrl_single_node.py --ros-args -p can_port:=can0
```

## File Structure

```
unitree_go_jetson/
├── unitree_ros1/                 # ROS1 packages
│   ├── src/
│   │   ├── faster-lio/          # SLAM system
│   │   ├── HesaiLidar_ROS_2.0/  # LiDAR driver
│   │   └── piper_ros/           # Robotic arm control
│   ├── build/                   # Build directory
│   ├── devel/                   # Development files
│   └── README.md                # ROS1 specific documentation
├── unitree_ros2/                # ROS2 packages
│   ├── src/
│   │   ├── nav2_cloud_bringup/  # Navigation system
│   │   ├── yolo_ros/            # Object detection
│   │   ├── realsense-ros/       # Camera integration
│   │   ├── piper_ros/           # Robotic arm control
│   │   └── unitree_msg_converter/ # Message conversion
│   ├── cyclonedx_ws/            # CycloneDDS workspace
│   ├── build/                   # Build directory
│   ├── install/                 # Installation files
│   └── README.md                # ROS2 specific documentation
└── README.md                    # This unified guide
```

## Support and Contributing

### Getting Help
1. Check the troubleshooting section above
2. Review package-specific README files in each directory
3. Check ROS community forums for ROS-specific issues
4. Contact maintainers for package-specific problems

### Contributing
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project contains multiple packages with different licenses:
- **Faster-LIO**: BSD License
- **Hesai LiDAR Driver**: BSD License
- **Nav2 Cloud Bringup**: Apache-2.0
- **YOLO ROS**: GPL-3
- **RealSense ROS**: Apache License 2.0
- **Piper ROS**: See individual package licenses

Please review individual package licenses for specific terms and conditions.

## Acknowledgments

- Faster-LIO: Based on FastLIO2 by Chunge Bai et al.
- YOLO ROS: Ultralytics YOLOv8/YOLOv9 integration
- RealSense ROS: Intel RealSense SDK integration
- Piper ROS: AgileX Robotics arm control system

---

For detailed information about specific packages, refer to the individual README files in each package directory.