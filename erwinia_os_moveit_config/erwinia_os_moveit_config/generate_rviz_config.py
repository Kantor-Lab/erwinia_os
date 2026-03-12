#!/usr/bin/env python3
"""
Generate RViz configuration with configurable namespaces and prefixes.
"""

import sys
import yaml
from pathlib import Path


def generate_rviz_config(platform_ns='amiga', platform_prefix='',
                        manipulator_ns='xarm', manipulator_prefix='xarm6_'):
    """
    Generate RViz configuration with proper namespacing.
    
    Args:
        platform_ns: Namespace for the mobile platform
        platform_prefix: Prefix for platform joint/link names
        manipulator_ns: Namespace for the manipulator
        manipulator_prefix: Prefix for manipulator joint names
    
    Returns:
        dict: RViz configuration dictionary
    """
    # Build full prefixes with namespaces for frame names
    plat_frame_prefix = f'{platform_ns}/{platform_prefix}'
    manip_frame_prefix = f'{manipulator_ns}/{manipulator_prefix}'
    
    # Build prefixes without namespace for planning group names
    manip_prefix = manipulator_prefix
    
    config = {
        'Panels': [
            {'Class': 'rviz_common/Displays', 'Help Height': 78, 'Name': 'Displays',
             'Property Tree Widget': {'Expanded': ['/Global Options1', '/Status1'], 'Splitter Ratio': 0.5},
             'Tree Height': 236},
            {'Class': 'rviz_common/Selection', 'Name': 'Selection'},
            {'Class': 'rviz_common/Tool Properties',
             'Expanded': ['/2D Goal Pose1', '/Publish Point1'],
             'Name': 'Tool Properties', 'Splitter Ratio': 0.5886790156364441},
            {'Class': 'rviz_common/Views',
             'Expanded': ['/Current View1'],
             'Name': 'Views', 'Splitter Ratio': 0.5},
            {'Class': 'rviz_common/Time', 'Experimental': False, 'Name': 'Time',
             'SyncMode': 0, 'SyncSource': ''}
        ],
        'Visualization Manager': {
            'Class': '',
            'Displays': [
                {'Alpha': 0.5, 'Cell Size': 1, 'Class': 'rviz_default_plugins/Grid',
                 'Color': '160; 160; 164', 'Enabled': True,
                 'Line Style': {'Line Width': 0.029999999329447746, 'Value': 'Lines'},
                 'Name': 'Grid', 'Normal Cell Count': 0,
                 'Offset': {'X': 0, 'Y': 0, 'Z': 0},
                 'Plane': 'XY', 'Plane Cell Count': 10,
                 'Reference Frame': '<Fixed Frame>', 'Value': True},
                {
                    'Acceleration_Scaling_Factor': 0.1,
                    'Class': 'moveit_rviz_plugin/MotionPlanning',
                    'Enabled': True,
                    'Move Group Namespace': '',
                    'MoveIt_Allow_Approximate_IK': False,
                    'MoveIt_Allow_External_Program': False,
                    'MoveIt_Allow_Replanning': False,
                    'MoveIt_Allow_Sensor_Positioning': False,
                    'MoveIt_Planning_Attempts': 10,
                    'MoveIt_Planning_Time': 5,
                    'MoveIt_Use_Cartesian_Path': False,
                    'MoveIt_Use_Constraint_Aware_IK': False,
                    'MoveIt_Workspace': {
                        'Center': {'X': 0, 'Y': 0, 'Z': 0},
                        'Size': {'X': 2, 'Y': 2, 'Z': 2}
                    },
                    'Name': 'MotionPlanning',
                    'Planned Path': {
                        'Color Enabled': False,
                        'Interrupt Display': False,
                        'Links': {
                            'All Links Enabled': True,
                            'Expand Joint Details': False,
                            'Expand Link Details': False,
                            'Expand Tree': False,
                            'Link Tree Style': 'Links in Alphabetic Order',
                            f'{plat_frame_prefix}base_footprint': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
                            f'{plat_frame_prefix}base_link': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{plat_frame_prefix}default_mount': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
                            f'{plat_frame_prefix}front_bumper_mount': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
                            f'{plat_frame_prefix}front_left_wheel': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{plat_frame_prefix}front_right_wheel': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{plat_frame_prefix}inertial_link': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
                            f'{plat_frame_prefix}rear_bumper_mount': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
                            f'{plat_frame_prefix}rear_left_wheel': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{plat_frame_prefix}rear_right_wheel': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{plat_frame_prefix}top_chassis_link': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            'x_rail_riser_link': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}base_link': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}link1': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}link2': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}link3': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}link4': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}link5': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}link6': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True}
                        },
                        'Loop Animation': False,
                        'Robot Alpha': 0.5,
                        'Robot Color': '150; 50; 150',
                        'Show Robot Collision': False,
                        'Show Robot Visual': True,
                        'Show Trail': False,
                        'State Display Time': '3x',
                        'Trail Step Size': 1,
                        'Trajectory Topic': '/display_planned_path',
                        'Use Sim Time': False
                    },
                    'Planning Metrics': {
                        'Payload': 1,
                        'Show Joint Torques': False,
                        'Show Manipulability': False,
                        'Show Manipulability Index': False,
                        'Show Weight Limit': False,
                        'TextHeight': 0.07999999821186066
                    },
                    'Planning Request': {
                        'Colliding Link Color': '255; 0; 0',
                        'Goal State Alpha': 1,
                        'Goal State Color': '250; 128; 0',
                        'Interactive Marker Size': 0.2,
                        'Joint Violation Color': '255; 0; 255',
                        'Planning Group': f'{manip_prefix}manipulator',
                        'Query Goal State': True,
                        'Query Start State': False,
                        'Show Workspace': False,
                        'Start State Alpha': 1,
                        'Start State Color': '0; 255; 0'
                    },
                    'Planning Scene Topic': '/monitored_planning_scene',
                    'Robot Description': 'robot_description',
                    'Scene Geometry': {
                        'Scene Alpha': 0.8999999761581421,
                        'Scene Color': '50; 230; 50',
                        'Scene Display Time': 0.009999999776482582,
                        'Show Scene Geometry': True,
                        'Voxel Coloring': 'Z-Axis',
                        'Voxel Rendering': 'Occupied Voxels'
                    },
                    'Scene Robot': {
                        'Attached Body Color': '150; 50; 150',
                        'Links': {
                            'All Links Enabled': True,
                            'Expand Joint Details': False,
                            'Expand Link Details': False,
                            'Expand Tree': False,
                            'Link Tree Style': 'Links in Alphabetic Order',
                            f'{plat_frame_prefix}base_footprint': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
                            f'{plat_frame_prefix}base_link': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{plat_frame_prefix}default_mount': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
                            f'{plat_frame_prefix}front_bumper_mount': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
                            f'{plat_frame_prefix}front_left_wheel': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{plat_frame_prefix}front_right_wheel': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{plat_frame_prefix}inertial_link': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
                            f'{plat_frame_prefix}rear_bumper_mount': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False},
                            f'{plat_frame_prefix}rear_left_wheel': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{plat_frame_prefix}rear_right_wheel': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{plat_frame_prefix}top_chassis_link': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            'x_rail_riser_link': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}base_link': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}link1': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}link2': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}link3': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}link4': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}link5': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True},
                            f'{manip_frame_prefix}link6': {'Alpha': 1, 'Show Axes': False, 'Show Trail': False, 'Value': True}
                        },
                        'Robot Alpha': 1,
                        'Show Robot Collision': False,
                        'Show Robot Visual': True
                    },
                    'Value': True,
                    'Velocity_Scaling_Factor': 0.1
                }
            ],
            'Enabled': True,
            'Global Options': {
                'Background Color': '48; 48; 48',
                'Fixed Frame': f'{plat_frame_prefix}base_link',
                'Frame Rate': 30
            },
            'Name': 'root',
            'Tools': [
                {'Class': 'rviz_default_plugins/Interact', 'Hide Inactive Objects': True},
                {'Class': 'rviz_default_plugins/MoveCamera'},
                {'Class': 'rviz_default_plugins/Select'},
                {'Class': 'rviz_default_plugins/FocusCamera'},
                {'Class': 'rviz_default_plugins/Measure', 'Line color': '128; 128; 0'},
                {'Class': 'rviz_default_plugins/SetInitialPose', 'Covariance x': 0.25,
                 'Covariance y': 0.25, 'Covariance yaw': 0.06853891909122467,
                 'Topic': {'Depth': 5, 'Durability Policy': 'Volatile',
                          'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable',
                          'Value': '/initialpose'}},
                {'Class': 'rviz_default_plugins/SetGoal',
                 'Topic': {'Depth': 5, 'Durability Policy': 'Volatile',
                          'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable',
                          'Value': '/goal_pose'}},
                {'Class': 'rviz_default_plugins/PublishPoint', 'Single click': True,
                 'Topic': {'Depth': 5, 'Durability Policy': 'Volatile',
                          'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable',
                          'Value': '/clicked_point'}}
            ],
            'Transformation': {'Current': {'Class': 'rviz_default_plugins/TF'}},
            'Value': True,
            'Views': {
                'Current': {
                    'Class': 'rviz_default_plugins/Orbit',
                    'Distance': 4.241325855255127,
                    'Enable Stereo Rendering': {
                        'Stereo Eye Separation': 0.05999999865889549,
                        'Stereo Focal Distance': 1,
                        'Swap Stereo Eyes': False,
                        'Value': False
                    },
                    'Focal Point': {'X': -0.2993867099285126, 'Y': -0.1544569581747055, 'Z': 0.32091614603996277},
                    'Focal Shape Fixed Size': True,
                    'Focal Shape Size': 0.05000000074505806,
                    'Invert Z Axis': False,
                    'Name': 'Current View',
                    'Near Clip Distance': 0.009999999776482582,
                    'Pitch': 0.5603981614112854,
                    'Target Frame': '<Fixed Frame>',
                    'Value': 'Orbit (rviz)',
                    'Yaw': 1.6353976726531982
                },
                'Saved': None
            }
        },
        'Window Geometry': {
            '': {'collapsed': False},
            ' - Trajectory Slider': {'collapsed': False},
            'Displays': {'collapsed': False},
            'Height': 1013,
            'Hide Left Dock': False,
            'Hide Right Dock': True,
            'QMainWindow State': '000000ff00000000fd0000000400000000000001f300000357fc020000000afb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d00000177000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb000000280020002d0020005400720061006a006500630074006f0072007900200053006c00690064006500720000000000ffffffff0000004100fffffffbffffffff01000001ba000001da0000017d00ffffff000000010000010f000002b0fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d000002b0000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000006900000003efc0100000002fb0000000800540069006d0065010000000000000690000002fb00fffffffb0000000800540069006d00650100000000000004500000000000000000000004970000035700000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000',
            'Selection': {'collapsed': False},
            'Time': {'collapsed': False},
            'Tool Properties': {'collapsed': False},
            'Views': {'collapsed': True},
            'Width': 1680,
            'X': 0,
            'Y': 0
        }
    }
    
    return config


def main():
    """Generate and output RViz configuration."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate RViz configuration')
    parser.add_argument('--platform-ns', default='amiga', help='Platform namespace')
    parser.add_argument('--platform-prefix', default='', help='Platform prefix')
    parser.add_argument('--manipulator-ns', default='xarm', help='Manipulator namespace')
    parser.add_argument('--manipulator-prefix', default='xarm6_', help='Manipulator prefix')
    parser.add_argument('--output', '-o', type=str, help='Output file path (optional)')
    
    args = parser.parse_args()
    
    # Generate configuration
    config = generate_rviz_config(
        platform_ns=args.platform_ns,
        platform_prefix=args.platform_prefix,
        manipulator_ns=args.manipulator_ns,
        manipulator_prefix=args.manipulator_prefix
    )
    
    # Output YAML
    yaml_str = yaml.dump(config, default_flow_style=False, sort_keys=False)
    
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(yaml_str)
        print(f"Generated RViz configuration: {output_path}", file=sys.stderr)
    else:
        print(yaml_str)
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
