from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """
    Main bringup orchestration for Erwinia OS (Amiga platform):
    1. Start Gazebo environment if use_gazebo=true
    2. Spawn the robot and start ros2_control
    3. Optionally launch RViz for visualization
    
    Logic:
    - If use_gazebo=true, then use_fake_hardware is forced to false
    - If use_gazebo=false, then use_fake_hardware can be true or false
    """
    
    # Get launch configurations
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    use_foxglove = LaunchConfiguration('use_foxglove')
    manipulator_prefix = LaunchConfiguration('manipulator_prefix')
    manipulator_ns = LaunchConfiguration('manipulator_ns')
    platform_prefix = LaunchConfiguration('platform_prefix')
    platform_ns = LaunchConfiguration('platform_ns')
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type')
    
    # Get package directories
    bringup_pkg = get_package_share_directory('erwinia_os_bringup')
    gz_pkg = get_package_share_directory('erwinia_os_gz')
    control_pkg = get_package_share_directory('erwinia_os_control')
    moveit_pkg = get_package_share_directory('erwinia_os_moveit_config')
    
    launch_actions = []

    use_gazebo_val = use_gazebo.perform(context).lower() == 'true'
    if use_gazebo_val:
        # Launch Gazebo with the selected world
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world,
            }.items()
        )
        launch_actions.append(gazebo_launch)
        
        # Launch ros2_control after a delay to ensure Gazebo is ready
        control_launch = TimerAction(
            period=3.0,  # 3 second delay
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(control_pkg, 'launch', 'ros2_control.launch.py')
                    ),
                    launch_arguments={
                        'use_gazebo': use_gazebo,
                        'use_sim_time': use_sim_time,
                        'manipulator_prefix': manipulator_prefix,
                        'manipulator_ns': manipulator_ns,
                        'platform_prefix': platform_prefix,
                        'platform_ns': platform_ns,
                        'robot_ip': robot_ip,
                        'report_type': report_type,
                    }.items()
                )
            ]
        )
        launch_actions.append(control_launch)
    else:
        # No Gazebo - launch ros2_control immediately
        control_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_pkg, 'launch', 'ros2_control.launch.py')
            ),
            launch_arguments={
                'use_gazebo': use_gazebo,
                'use_sim_time': use_sim_time,
                'manipulator_prefix': manipulator_prefix,
                'manipulator_ns': manipulator_ns,
                'platform_prefix': platform_prefix,
                'platform_ns': platform_ns,
                'robot_ip': robot_ip,
                'report_type': report_type,
            }.items()
        )
        launch_actions.append(control_launch)

    # Launch MoveIt move_group
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_gazebo': use_gazebo,
            'manipulator_prefix': manipulator_prefix,
            'platform_prefix': platform_prefix,
            'manipulator_ns': manipulator_ns,
            'platform_ns': platform_ns,
        }.items()
    )
    launch_actions.append(moveit_launch)

    # Publish map->odom transform if using gazebo (since odom comes from simulation)
    if use_gazebo_val:
        map_frame_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        )
        launch_actions.append(map_frame_publisher)
    
    # # Optionally launch Foxglove Bridge
    # if use_foxglove.perform(context).lower() == 'true':
    #     foxglove_bridge_node = Node(
    #         package='foxglove_bridge',
    #         executable='foxglove_bridge',
    #         name='foxglove_bridge',
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         output='screen'
    #     )
    #     launch_actions.append(foxglove_bridge_node)

    # Optionally launch RViz
    if use_rviz.perform(context).lower() == 'true':
        rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_pkg, 'launch', 'rviz.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'manipulator_prefix': manipulator_prefix,
                'platform_prefix': platform_prefix,
                'manipulator_ns': manipulator_ns,
                'platform_ns': platform_ns,
                'rviz_config': os.path.join(bringup_pkg, 'rviz', 'view.rviz'),
            }.items()
        )
        launch_actions.append(rviz_launch)
    
    return launch_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='true',
            description='Whether to use Gazebo simulation'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='World to load: "empty" or path to .sdf file'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz'
        ),
        DeclareLaunchArgument(
            'use_foxglove',
            default_value='false',
            description='Whether to launch Foxglove Bridge'
        ),
        DeclareLaunchArgument(
            'manipulator_prefix',
            default_value='xarm6_',
            description='Prefix for manipulator joint names'
        ),
        DeclareLaunchArgument(
            'manipulator_ns',
            default_value='xarm',
            description='Namespace for xArm platform'
        ),
        DeclareLaunchArgument(
            'platform_prefix',
            default_value='',
            description='Prefix for platform joint names or frames'
        ),
        DeclareLaunchArgument(
            'platform_ns',
            default_value='amiga',
            description='Namespace for Amiga platform'
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.205',
            description='IP address of the xArm6 robot'
        ),
        DeclareLaunchArgument(
            'report_type',
            default_value='normal',
            description='Report type for xArm (normal, rich, dev)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
