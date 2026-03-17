#!/usr/bin/env python3
"""
Generate MoveIt controller configuration YAML with configurable namespaces and prefixes.
"""

import sys
import yaml
from pathlib import Path


def generate_moveit_controllers_yaml(manipulator_ns='xarm', manipulator_prefix='xarm6_'):
    """
    Generate MoveIt controller configuration with proper namespacing.
    
    Args:
        manipulator_ns: Namespace for the manipulator
        manipulator_prefix: Prefix for manipulator joint names
    
    Returns:
        dict: MoveIt controller configuration dictionary
    """
    # Build namespace prefix
    manipulator_ns_prefix = f"{manipulator_ns}/{manipulator_prefix}"
    
    config = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': {
            'controller_names': [
                'xarm6_traj_controller'
            ],
            'xarm6_traj_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': [
                    f'{manipulator_ns_prefix}joint1',
                    f'{manipulator_ns_prefix}joint2',
                    f'{manipulator_ns_prefix}joint3',
                    f'{manipulator_ns_prefix}joint4',
                    f'{manipulator_ns_prefix}joint5',
                    f'{manipulator_ns_prefix}joint6'
                ]
            }
        },
        'trajectory_execution': {
            'manage_controllers': False,
            'allowed_execution_duration_scaling': 1.2,
            'allowed_goal_duration_margin': 0.5,
            'allowed_start_tolerance': 0.01
        }
    }
    
    return config


def main():
    """Generate and output YAML configuration."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate MoveIt controller configuration YAML')
    parser.add_argument('--manipulator-ns', default='xarm', help='Manipulator namespace')
    parser.add_argument('--manipulator-prefix', default='xarm6_', help='Manipulator prefix')
    parser.add_argument('--output', '-o', type=str, help='Output file path (optional)')
    
    args = parser.parse_args()
    
    # Generate configuration
    config = generate_moveit_controllers_yaml(
        manipulator_ns=args.manipulator_ns,
        manipulator_prefix=args.manipulator_prefix
    )
    
    # Output YAML
    yaml_str = yaml.dump(config, default_flow_style=False, sort_keys=False)
    
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(yaml_str)
        print(f"Generated MoveIt controller configuration: {output_path}", file=sys.stderr)
    else:
        print(yaml_str)
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
