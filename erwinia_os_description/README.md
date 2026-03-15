# erwinia_os_description

URDF/Xacro robot description, meshes, and visualization configurations for the Erwinia OS (Amiga platform).

## Overview

This package contains:
- URDF/Xacro files defining the Amiga robot structure
- STL meshes for visualization and collision
- RViz configuration files
- Modular macros for platform, wheels, and sensors

## Package Contents

- `urdf/erwinia_os.urdf.xacro` - Top-level robot assembly
- `urdf/amiga/amiga_macro.urdf.xacro` - Amiga platform base
- `urdf/amiga/amiga_wheel_macro.urdf.xacro` - Wheel definitions
- `urdf/amiga/amiga_ros2_control.xacro` - Amiga ros2_control configuration
- `urdf/sensors/amiga_camera_macro.urdf.xacro` - Camera sensor
- `urdf/sensors/amiga_lidar_macro.urdf.xacro` - LiDAR sensor
- `urdf/xarm6/xarm6_macro.urdf.xacro` - xArm manipulator
- `urdf/xarm6/xarm6_ros2_control.urdf.xacro` - xArm ros2_control configuration
- `meshes/` - STL mesh files
- `rviz/view.rviz` - RViz configuration

## Robot Structure

The amiga is currently outfitted with an IMU, lidar, and robotic arm (with gnss and front facing camera to come).

## Usage

The robot description is typically loaded through launch files in other packages, but can be viewed directly:

```bash
# View URDF in RViz
ros2 launch erwinia_os_description description.launch.py
```

## Parameters

The main URDF accepts these xacro arguments:
- `use_gazebo` - Enable Gazebo simulation mode
- `config_file` - Path to controller configuration YAML
- `manipulator_prefix`
- `manipulator_ns`
- `platform_prefix` - Prefix for frame/joint names (default: "")
- `platform_ns` - Namespace for platform (default: amiga)

## Dependencies

- `xacro`
- `robot_state_publisher`
- `joint_state_publisher` (for visualization without controllers)
- `xarm_description`

