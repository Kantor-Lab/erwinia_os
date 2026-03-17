# erwinia_os_control

ROS2 control configuration and launch files for the Amiga platform (Erwinia OS).

## Overview

This package provides:
- Dynamic controller configuration generation with namespace and prefix support (generates yaml temp files)
- Launch files for ros2_control system
- DiffDriveController configuration for 4-wheel Amiga platform

## Package Contents

- `erwinia_os_control/generate_controllers_yaml.py` - Dynamic YAML configuration generator
- `launch/ros2_control.launch.py` - Main launch file for ros2_control system

## Usage

### Launch with Fake Hardware (Testing)

```bash
ros2 launch erwinia_os_control ros2_control.launch.py use_fake_hardware:=true
```

### Launch with Gazebo Simulation

```bash
ros2 launch erwinia_os_control ros2_control.launch.py use_gazebo:=true use_sim_time:=true
```

### Custom Namespace/Prefix

```bash
ros2 launch erwinia_os_control ros2_control.launch.py \
    platform_ns:=my_robot \
    platform_prefix:=amiga_ \
    use_fake_hardware:=true
```

## Controller Configuration

### Platform Velocity Controller

The `platform_velocity_controller` uses `diff_drive_controller` for 4-wheel differential drive:

**Wheel Configuration:**
- Left wheels: `FL_wheel_joint`, `RL_wheel_joint`
- Right wheels: `FR_wheel_joint`, `RR_wheel_joint`
- Wheel separation: 0.65856 m
- Wheel radius: 0.2156 m

**Velocity Limits:**
- Max linear velocity: 2.54 m/s
- Max angular velocity: 2.0 rad/s
- Max linear acceleration: 3.0 m/s²
- Max angular acceleration: 1.5 rad/s²

**Control Interface:**
- Subscribe to: `/platform_velocity_controller/cmd_vel_unstamped`
- Publish odometry: `/odom`

### Joint State Broadcaster

Publishes joint states to `/joint_states` topic.

## Standalone Configuration Generation

Generate controller YAML manually:

```bash
python3 -m erwinia_os_control.generate_controllers_yaml \
    --platform-ns amiga \
    --platform-prefix "" \
    --use-fake-hardware \
    --output config/controllers.yaml
```

## Parameters

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_gazebo` | false | Enable Gazebo simulation mode |
| `use_fake_hardware` | false | Use fake hardware interface |
| `use_sim_time` | false | Use simulated time |
| `platform_ns` | amiga | Namespace for Amiga platform |
| `platform_prefix` | "" | Prefix for platform joint/link names |

## Dependencies

- `ros2_control`
- `controller_manager`
- `diff_drive_controller`
- `joint_state_broadcaster`
- `robot_state_publisher`
- `erwinia_os_description`

## Testing

After launching, test the platform controller:

```bash
# Publish velocity commands
ros2 topic pub /platform_velocity_controller/cmd_vel_unstamped \
    geometry_msgs/msg/Twist \
    "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"

# Monitor joint states
ros2 topic echo /joint_states

# Monitor odometry
ros2 topic echo /odom
```
