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
- `urdf/amiga/amiga_ros2_control.xacro` - ros2_control configuration
- `urdf/sensors/amiga_camera_macro.urdf.xacro` - Camera sensor
- `urdf/sensors/amiga_lidar_macro.urdf.xacro` - LiDAR sensor
- `meshes/` - STL mesh files
- `rviz/view.rviz` - RViz configuration

## Robot Structure

**Amiga Platform:**
- 4-wheel differential drive base
- Wheelbase: 0.5316 m
- Track width: 0.65856 m
- Wheel radius: 0.2159 m

**Sensors:**
- Front-facing camera with optical frame
- 3D LiDAR with adjustable pitch

**Features:**
- Namespace/prefix support for multi-robot systems
- Conditional ros2_control hardware interfaces (Gazebo/fake/real)
- Gazebo sensor plugins for camera and LiDAR

## Usage

The robot description is typically loaded through launch files in other packages, but can be viewed directly:

```bash
# View URDF in RViz
ros2 launch erwinia_os_description view_robot.launch.py

# Process xacro to URDF
xacro $(ros2 pkg prefix erwinia_os_description)/share/erwinia_os_description/urdf/erwinia_os.urdf.xacro
```

## Parameters

The main URDF accepts these xacro arguments:
- `use_gazebo` - Enable Gazebo simulation mode
- `use_fake_hardware` - Use fake hardware interface
- `config_file` - Path to controller configuration YAML
- `platform_ns` - Namespace for platform (default: amiga)
- `platform_prefix` - Prefix for frame/joint names (default: "")

## Dependencies

- `xacro`
- `robot_state_publisher`
- `joint_state_publisher` (for visualization without controllers)

