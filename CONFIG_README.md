# Running ROS2 Software System on Jetson

This document explains how to use the new configuration system for the Unitree Go Robot launcher.

## Overview

The `launch_ros.py` script now supports a YAML configuration file that allows you to selectively enable/disable different modules. This gives you fine-grained control over which components are launched.

## Configuration File

The main configuration file is `robot_config.yaml`. You can also use custom configuration files by passing them as command line arguments.

### Usage

```bash
# Use default configuration (with log filtering)
python3 launch_ros.py

# Use custom configuration file
python3 launch_ros.py robot_config.yaml
```

## Cleanup Commands

If processes don't terminate properly after Ctrl+C, use these cleanup commands:

```bash
# Cleanup using the launcher script
python3 launch_ros.py --cleanup

# Or use the dedicated cleanup script
python3 cleanup_ros.py
```

## Configuration Options

### Core Infrastructure (Always Required)
- `ros1_core`: Enable ROS1 core (roscore)
- `ros_bridge`: Enable ROS1/ROS2 bridge

### ROS2 Systems
- `enabled`: Enable/disable ROS2 systems launch
- `launch_file`: Path to the ROS2 systems launch file

### Arm Module
- `enabled`: Enable/disable the entire arm module
- `joint_relay`: Enable/disable joint relay (can run independently)
- `piper_arm`: Enable/disable piper arm integration (placeholder for future)

### LiDAR Driver
- `enabled`: Enable/disable LiDAR driver
- `driver`: Driver executable name

### SLAM (Faster-LIO)
- `enabled`: Enable/disable SLAM
- `algorithm`: SLAM algorithm name
- `launch_file`: SLAM launch file
- `rviz`: Enable/disable RViz visualization

### Navigation
- `enabled`: Enable/disable navigation
- `params_file`: Navigation parameters file

### YOLO Object Detection
- `enabled`: Enable/disable YOLO detection (placeholder for future)

### Message Converters
- `imu_converter`: Enable/disable IMU converter
- `tf_publishers`: Enable/disable TF publishers

### Launch Delays
Configure delays (in seconds) between component launches to ensure proper startup sequence.

### Log Filtering
Control the verbosity of output logs:
- `enabled`: Enable/disable log filtering
- `suppress_verbose_logs`: Filter out verbose component logs (frame data, mapping info, etc.)
- `suppress_timestamps`: Filter out timestamp-heavy logs

## Example Configurations

See `config_examples.yaml` for different configuration scenarios:

1. **Minimal Configuration**: Core + ROS2 Systems only
2. **SLAM + Navigation**: Full mapping and navigation setup
3. **Arm Module**: Arm-specific configuration
4. **YOLO Detection**: Object detection setup

Additional configuration files:
- `robot_config_verbose.yaml`: Shows all logs including timestamps and verbose output

## Creating Custom Configurations

1. Copy one of the example configurations from `config_examples.yaml`
2. Modify the settings according to your needs
3. Save as a new YAML file (e.g., `my_config.yaml`)
4. Run with: `python3 launch_ros.py my_config.yaml`

## Configuration Validation

The launcher will:
- Load the specified configuration file
- Fall back to default settings if the file doesn't exist
- Print a configuration summary before launching
- Show which modules are enabled/disabled

## Troubleshooting

- If a configuration file is missing, the launcher will use default settings
- Invalid YAML syntax will cause the launcher to fall back to defaults
- Check the configuration summary output to verify your settings

## Future Extensions

The configuration system is designed to be easily extensible. When new modules are added (like piper arm or YOLO), simply add new configuration sections following the same pattern.
