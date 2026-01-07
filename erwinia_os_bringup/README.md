# erwinia_os_bringup

Launch package for starting the complete Erwinia OS (Amiga platform) stack.

## Overview

This package provides orchestration launch files that bring up the entire robot system including:
- Gazebo simulation (optional)
- ros2_control with controllers
- RViz visualization (optional)

## Usage

### Launch with Gazebo Simulation

```bash
ros2 launch erwinia_os_bringup bringup.launch.py use_gazebo:=true
```

### Launch with Fake Hardware (Testing)

```bash
ros2 launch erwinia_os_bringup bringup.launch.py use_fake_hardware:=true use_sim_time:=false
```

### Launch with Real Hardware

```bash
ros2 launch erwinia_os_bringup bringup.launch.py use_sim_time:=false use_rviz:=true
```

### Custom World

```bash
ros2 launch erwinia_os_bringup bringup.launch.py \
    use_gazebo:=true \
    world:=/path/to/custom_world.sdf
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_gazebo` | false | Enable Gazebo simulation |
| `use_fake_hardware` | false | Use fake hardware interface for testing |
| `use_sim_time` | true | Use simulation time |
| `world` | empty | Gazebo world to load |
| `use_rviz` | true | Launch RViz for visualization |
| `platform_ns` | amiga | Namespace for platform |
| `platform_prefix` | "" | Prefix for platform frames/joints |

## Dependencies

- `erwinia_os_description`
- `erwinia_os_control`
- `erwinia_os_gz`
- `rviz2`
- `tf2_ros`

