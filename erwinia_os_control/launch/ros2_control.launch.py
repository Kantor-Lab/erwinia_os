#!/usr/bin/env python3
"""
Launch file for Amiga ros2_control system.
"""

import os
import yaml
import tempfile
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import load_xacro
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from erwinia_os_control.generate_controllers_yaml import generate_controllers_yaml


def get_xacro_content(context, xacro_file, **kwargs):
    xacro_file = Path(xacro_file.perform(context)) if isinstance(xacro_file, LaunchConfiguration) else Path(xacro_file) if isinstance(xacro_file, str) else xacro_file
    
    def get_param_str(param):
        val = param if isinstance(param, str) else 'false' if param == False else 'true' if param == True else (param.perform(context) if context is not None else param) if isinstance(param, LaunchConfiguration) else str(param)
        return val if not val else val[1:-1] if isinstance(val, str) and val[0] in ['"', '\''] and val[-1] in ['"', '\''] else val

    mappings = {}
    for k, v in kwargs.items():
        mappings[k] = get_param_str(v)
    return load_xacro(xacro_file, mappings=mappings)


def launch_setup(context, *args, **kwargs):
    """
    Setup launch nodes with context-dependent configuration.
    """
    description_pkg = get_package_share_directory('erwinia_os_description')
    xacro_file = Path(description_pkg) / 'urdf' / 'erwinia_os.urdf.xacro'
    control_pkg = get_package_share_directory('erwinia_os_control')

    # Get launch arguments
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    manipulator_ns = LaunchConfiguration('manipulator_ns')
    manipulator_prefix = LaunchConfiguration('manipulator_prefix')
    platform_ns = LaunchConfiguration('platform_ns')
    platform_prefix = LaunchConfiguration('platform_prefix')
    
    # Perform context evaluation
    use_gazebo_val = use_gazebo.perform(context).lower() == 'true'
    use_fake_hardware_val = use_fake_hardware.perform(context).lower() == 'true'
    use_sim_time_val = use_sim_time.perform(context).lower() == 'true'
    config_file_val = config_file.perform(context)
    manipulator_ns_val = manipulator_ns.perform(context)
    manipulator_prefix_val = manipulator_prefix.perform(context)
    platform_ns_val = platform_ns.perform(context)
    platform_prefix_val = platform_prefix.perform(context)
    
    # Validate configuration
    if use_gazebo_val and use_fake_hardware_val:
        raise ValueError('Cannot use both Gazebo and fake hardware simultaneously')
    
    # Generate controller configuration
    if config_file_val == '':
        config = generate_controllers_yaml(
            manipulator_ns=manipulator_ns_val,
            manipulator_prefix=manipulator_prefix_val,
            platform_ns=platform_ns_val,
            platform_prefix=platform_prefix_val,
            use_gazebo=use_gazebo_val,
            use_fake_hardware=use_fake_hardware_val
        )
        
        # Write config to temporary file
        controllers_yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(config, controllers_yaml_file, default_flow_style=False, sort_keys=False)
        controllers_yaml_file.close()
        controllers_yaml_path = Path(controllers_yaml_file.name)
    else:
        controllers_yaml_path = Path(config_file_val)

    robot_description = get_xacro_content(
        context,
        xacro_file=xacro_file,
        use_gazebo=use_gazebo,
        use_fake_hardware='true' if use_fake_hardware_val else 'false',
        config_file=controllers_yaml_path,
        manipulator_prefix=manipulator_prefix,
        manipulator_ns=manipulator_ns,
        platform_prefix=platform_prefix,
        platform_ns=platform_ns
    )
    
    # Publish the robot state to tf (use simulated time)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}, 
            {'robot_description': robot_description}
        ]
    )

    # Fake controllers for simulation
    if use_gazebo_val:

        # Use ros_gz_sim entity creation for Ignition Fortress
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'erwinia_os',
                '-z', '0.3', # slight z offset to avoid collision with ground plane
                # Initial joint positions to avoid collision
                # '-j', 'xarm6_joint1', '0.0',
                # '-j', 'xarm6_joint2', '-0.5',
                # '-j', 'xarm6_joint3', '0.5',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )

        joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            parameters=[{'use_sim_time': use_sim_time}]
        )

        platform_velocity_controller = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'platform_velocity_controller',
                '--controller-manager', '/controller_manager',
                '--param-file', str(controllers_yaml_path)
                ],
            parameters=[{'use_sim_time': use_sim_time}]
        )

        # xarm6_traj_controller = Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=[
        #         'xarm6_traj_controller',
        #         '--controller-manager', '/controller_manager',
        #         '--param-file', str(controllers_yaml_path)
        #         ],
        #     parameters=[{'use_sim_time': use_sim_time}]
        # )

        # Start platform controller and arm controller after joint state broadcaster
        jsb_then_controllers = RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster, 
                on_exit=[
                    platform_velocity_controller, 
                    # xarm6_traj_controller
                ])
        )

        # Control bridge - handles robot control topics only
        # Clock is now handled by the gazebo.launch.py file
        # Example: /cmd_vel@geometry_msgs/msg/Twist means the bridge will expose a ROS topic /cmd_vel with type geometry_msgs/msg/Twist.
        # We are then remapping the ros-side /cmd_vel to the ros2_control topic /platform_velocity_controller/cmd_vel_unstamped
        control_bridge = Node(
            name='control_bridge',
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            ],
            remappings=[
                ('/cmd_vel', '/platform_velocity_controller/cmd_vel_unstamped')
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )

        return [
            robot_state_publisher,
            spawn_entity,
            joint_state_broadcaster,
            jsb_then_controllers,
            control_bridge,
        ]
    
    # Not gazebo sim - need to explicitly start controller_manager
    else:
        ros2_control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controllers_yaml_path,
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
        )

        # Build controller list based on configuration
        controllers = [
            'joint_state_broadcaster', 
            # 'xarm6_traj_controllers',
            'platform_velocity_controller'
        ]

        controller_nodes = []
        for controller in controllers:
            controller_nodes.append(Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=[
                    controller,
                    '--controller-manager', '/controller_manager'
                ],
            ))

        return [
            robot_state_publisher,
            # joint_state_publisher_node,
            ros2_control_node,
        ] + controller_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gazebo', 
            default_value='false', 
            description='Enable Gazebo simulation mode'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware', 
            default_value='true', 
            description='Use fake hardware interface if true (default for testing)'
        ),
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false', 
            description='Use simulated time if true'
        ),
        DeclareLaunchArgument(
            'config_file', 
            default_value='',
            description='Path to controllers YAML configuration file'
        ),
        DeclareLaunchArgument(
            'manipulator_ns', 
            default_value='xarm', 
            description='Namespace for manipulator'
        ),
        DeclareLaunchArgument(
            'manipulator_prefix', 
            default_value='xarm6_', 
            description='Prefix for manipulator joint names or frames'
        ),
        DeclareLaunchArgument(
            'platform_ns', 
            default_value='amiga', 
            description='Namespace for Amiga platform'
        ),
        DeclareLaunchArgument(
            'platform_prefix', 
            default_value='', 
            description='Prefix for platform joint names or frames'
        ),
        OpaqueFunction(function=launch_setup)
    ])
