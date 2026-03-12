#!/usr/bin/env python3
"""
Generate controller configuration YAML for Amiga platform with configurable namespaces and prefixes.
"""

import sys
import yaml
from pathlib import Path


def generate_controllers_yaml(manipulator_ns='xarm', manipulator_prefix='xarm6_',
                              platform_ns='amiga', platform_prefix='', 
                              use_gazebo=False):
    """
    Generate controller configuration with proper namespacing for Amiga platform.
    
    Args:
        platform_ns: Namespace for the mobile platform
        platform_prefix: Prefix for platform joint/link names
        manipulator_ns: Namespace for the manipulator
        manipulator_prefix: Prefix for manipulator joint names
        use_gazebo: Whether using Gazebo simulation
        use_fake_hardware: Whether using fake hardware interface
    
    Returns:
        dict: Controller configuration dictionary
    """
    # Build namespace prefix
    platform_ns_prefix = f"{platform_ns}/{platform_prefix}"
    manipulator_ns_prefix = f"{manipulator_ns}/{manipulator_prefix}"
    
    # Start with base controller manager config
    controller_manager_params = {
        'update_rate': 100,
        'joint_state_broadcaster': {
            'type': 'joint_state_broadcaster/JointStateBroadcaster'
        },
        'platform_velocity_controller': {
            'type': 'diff_drive_controller/DiffDriveController'
        },
        'xarm6_traj_controller': {
            'type': 'joint_trajectory_controller/JointTrajectoryController'
        }
    }
    
    config = {
        'controller_manager': {
            'ros__parameters': controller_manager_params
        }
    }
    
    # Add the platform velocity controller configuration
    config['platform_velocity_controller'] = {
        'ros__parameters': {
            'left_wheel_names': [
                f'{platform_ns_prefix}FL_wheel_joint',
                f'{platform_ns_prefix}RL_wheel_joint'
            ],
            'right_wheel_names': [
                f'{platform_ns_prefix}FR_wheel_joint',
                f'{platform_ns_prefix}RR_wheel_joint'
            ],
            'wheel_separation': 0.65856, # Amiga track width
            'wheel_radius': 0.2159, # Amiga wheel radius
            'publish_rate': 50.0,
            'base_frame_id': f'{platform_ns_prefix}base_footprint',
            'odom_frame_id': 'odom',
            'use_stamped_vel': False,
            'linear.x.has_velocity_limits': True,
            'linear.x.max_velocity': 2.54,
            'linear.x.has_acceleration_limits': True,
            'linear.x.max_acceleration': 3.0,
            'angular.z.has_velocity_limits': True,
            'angular.z.max_velocity': 2.0,
            'angular.z.has_acceleration_limits': True,
            'angular.z.max_acceleration': 1.5
        }
    }

    # Add the manipulator trajectory controller configuration
    config['xarm6_traj_controller'] = {
        'ros__parameters': {
            'joints': [
                f'{manipulator_ns_prefix}joint1',
                f'{manipulator_ns_prefix}joint2',
                f'{manipulator_ns_prefix}joint3',
                f'{manipulator_ns_prefix}joint4',
                f'{manipulator_ns_prefix}joint5',
                f'{manipulator_ns_prefix}joint6'
            ],
            'command_interfaces': ['position'],
            'state_interfaces': ['position', 'velocity'],
            'state_publish_rate': 100.0,
            'action_monitor_rate': 20.0,
            'allow_partial_joints_goal': False,
            'constraints': {
                'stopped_velocity_tolerance': 0.01,
                'goal_time': 0.0
            }
        }
    }
    
    return config


def main():
    """Generate and output YAML configuration."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate controller configuration YAML')
    parser.add_argument('--manipulator-ns', default='xarm', help='Manipulator namespace')
    parser.add_argument('--manipulator-prefix', default='xarm6_', help='Manipulator prefix')
    parser.add_argument('--platform-ns', default='amiga', help='Platform namespace')
    parser.add_argument('--platform-prefix', default='', help='Platform prefix')
    parser.add_argument('--use-gazebo', action='store_true', help='Using Gazebo simulation')
    parser.add_argument('--use-fake-hardware', action='store_true', help='Using fake hardware interface')
    parser.add_argument('--output', '-o', type=str, help='Output file path (optional)')
    
    args = parser.parse_args()
    
    # Generate configuration
    config = generate_controllers_yaml(
        manipulator_ns=args.manipulator_ns,
        manipulator_prefix=args.manipulator_prefix,
        platform_ns=args.platform_ns,
        platform_prefix=args.platform_prefix,
        use_gazebo=args.use_gazebo,
        use_fake_hardware=args.use_fake_hardware
    )
    
    # Output YAML
    yaml_str = yaml.dump(config, default_flow_style=False, sort_keys=False)
    
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(yaml_str)
        print(f"Generated controller configuration: {output_path}", file=sys.stderr)
    else:
        print(yaml_str)
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
