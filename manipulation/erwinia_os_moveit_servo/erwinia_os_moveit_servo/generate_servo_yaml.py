#!/usr/bin/env python3
"""
Generate MoveIt Servo configuration YAML with configurable namespaces and prefixes.
"""

import sys
import yaml
from pathlib import Path


def generate_servo_yaml(
    manipulator_ns='xarm',
    manipulator_prefix='xarm6_',
    platform_prefix='',
    planning_frame='base_link',
    ee_frame_name='xarm_tool0',
    move_group_name='xarm6_manipulator',
    use_gazebo=False,
    command_in_type='unitless',
    linear_scale=0.15,
    rotational_scale=0.4,
    publish_period=0.01,
    low_pass_filter_coeff=2.0,
    incoming_command_timeout=0.1,
    num_outgoing_halt_msgs=4,
    joint_topic='/joint_states',
    command_out_topic='xarm6_traj_controller',
    publish_joint_positions=False,
    publish_joint_velocities=True,
    publish_joint_accelerations=False
):
    """
    Generate MoveIt Servo configuration with proper namespacing.
    
    Args:
        manipulator_ns: Namespace for the manipulator
        manipulator_prefix: Prefix for manipulator joint names
        platform_prefix: Prefix for platform (if any)
        planning_frame: Base frame for planning
        ee_frame_name: End effector frame name
        move_group_name: Name of the MoveIt planning group
        use_gazebo: Whether running in Gazebo simulation
        command_in_type: Input command type ('unitless' or 'speed_units')
        linear_scale: Scaling factor for linear velocity
        rotational_scale: Scaling factor for rotational velocity
        publish_period: Period for publishing commands (seconds)
        low_pass_filter_coeff: Low pass filter coefficient
        incoming_command_timeout: Timeout for incoming commands (seconds)
        num_outgoing_halt_msgs: Number of halt messages to publish on stop
        joint_topic: Topic for joint states
        command_out_topic: Topic/controller name for output commands
        publish_joint_positions: Whether to publish joint positions
        publish_joint_velocities: Whether to publish joint velocities
        publish_joint_accelerations: Whether to publish joint accelerations
    
    Returns:
        dict: MoveIt Servo configuration dictionary
    """
    
    # Construct frame names with proper prefixes
    if platform_prefix:
        full_planning_frame = f"{platform_prefix}{planning_frame}"
    else:
        full_planning_frame = planning_frame
    
    # EE frame may have manipulator namespace prefix
    if manipulator_ns and not ee_frame_name.startswith(manipulator_ns):
        full_ee_frame = f"{manipulator_ns}_{ee_frame_name}"
    else:
        full_ee_frame = ee_frame_name
    
    # ROS 2 Servo expects parameters under "moveit_servo" namespace
    # The node is "servo_node_main" or "servo_node" but params go under "moveit_servo"
    config = {
        'moveit_servo': {
            'planning_frame': full_planning_frame,
            'ee_frame_name': full_ee_frame,
            'move_group_name': move_group_name,
            
            # Command configuration
            'command_in_type': command_in_type,
            'command_out_type': 'trajectory_msgs/JointTrajectory',
            'command_out_topic': f'/{command_out_topic}/joint_trajectory',
            
            # Scaling
            'scale': {
                'linear': linear_scale,
                'rotational': rotational_scale
            },
            
            # Publishing configuration
            'publish_period': publish_period,
            'publish_joint_positions': publish_joint_positions,
            'publish_joint_velocities': publish_joint_velocities,
            'publish_joint_accelerations': publish_joint_accelerations,
            
            # Filtering and safety
            'low_pass_filter_coeff': low_pass_filter_coeff,
            'incoming_command_timeout': incoming_command_timeout,
            'num_outgoing_halt_msgs_to_publish': num_outgoing_halt_msgs,
            
            # System configuration
            'use_gazebo': use_gazebo,
            'joint_topic': joint_topic,
        }
    }
    
    return config


def main():
    """Generate and output YAML configuration."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate MoveIt Servo configuration YAML')
    parser.add_argument('--manipulator-ns', default='xarm', help='Manipulator namespace')
    parser.add_argument('--manipulator-prefix', default='xarm6_', help='Manipulator prefix')
    parser.add_argument('--platform-prefix', default='', help='Platform prefix')
    parser.add_argument('--planning-frame', default='base_link', help='Planning frame')
    parser.add_argument('--ee-frame-name', default='xarm_tool0', help='End effector frame name')
    parser.add_argument('--move-group-name', default='xarm6_manipulator', help='MoveIt planning group name')
    parser.add_argument('--use-gazebo', action='store_true', help='Use Gazebo simulation')
    parser.add_argument('--command-in-type', default='unitless', help='Input command type (unitless or speed_units)')
    parser.add_argument('--linear-scale', type=float, default=0.15, help='Linear velocity scale')
    parser.add_argument('--rotational-scale', type=float, default=0.4, help='Rotational velocity scale')
    parser.add_argument('--joint-topic', default='/joint_states', help='Joint states topic')
    parser.add_argument('--command-out-topic', default='xarm6_traj_controller', help='Controller name for output')
    parser.add_argument('--output', '-o', type=str, help='Output file path (optional)')
    
    args = parser.parse_args()
    
    # Generate configuration
    config = generate_servo_yaml(
        manipulator_ns=args.manipulator_ns,
        manipulator_prefix=args.manipulator_prefix,
        platform_prefix=args.platform_prefix,
        planning_frame=args.planning_frame,
        ee_frame_name=args.ee_frame_name,
        move_group_name=args.move_group_name,
        use_gazebo=args.use_gazebo,
        command_in_type=args.command_in_type,
        linear_scale=args.linear_scale,
        rotational_scale=args.rotational_scale,
        joint_topic=args.joint_topic,
        command_out_topic=args.command_out_topic
    )
    
    # Output YAML
    yaml_str = yaml.dump(config, default_flow_style=False, sort_keys=False)
    
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(yaml_str)
        print(f"Generated MoveIt Servo configuration: {output_path}", file=sys.stderr)
    else:
        print(yaml_str)
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
