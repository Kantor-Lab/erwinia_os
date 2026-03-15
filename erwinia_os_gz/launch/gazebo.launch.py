import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def launch_setup(context, *args, **kwargs):
    """
    Launch Gazebo with the specified world.
    Sets up resource paths for custom models and worlds.
    """
    world = LaunchConfiguration('world')
    world_value = world.perform(context)
    
    generate_markers = LaunchConfiguration('generate_markers')
    generate_markers_value = generate_markers.perform(context)
    
    num_markers = LaunchConfiguration('num_markers')
    num_markers_value = num_markers.perform(context)
    
    marker_dict = LaunchConfiguration('marker_dict')
    marker_dict_value = marker_dict.perform(context)
    
    marker_size = LaunchConfiguration('marker_size')
    marker_size_value = marker_size.perform(context)
    
    pkg_gz = get_package_share_directory('erwinia_os_gz')
    worlds_dir = os.path.join(pkg_gz, 'worlds')
    models_dir = os.path.join(pkg_gz, 'models')

    # All currently-sourced ROS packages' share dirs
    ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    packages_paths = [os.path.join(p, 'share') for p in ament_prefix.split(':') if p]

    # Existing resource paths (if any)
    ign_existing = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    gz_existing  = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    # Compose new paths: put our package first, then all ROS shares, then previous value
    ign_path = ':'.join(
        [worlds_dir, models_dir] +
        packages_paths +
        ([ign_existing] if ign_existing else [])
    )
    gz_path = ':'.join(
        [worlds_dir, models_dir] +
        packages_paths +
        ([gz_existing] if gz_existing else [])
    )
    
    # Determine the .sdf world files available
    worlds = {f[:-4] for f in os.listdir(worlds_dir) if f.endswith('.sdf')}
    if world_value in worlds:
        # Use one of our custom worlds
        world_file = PathJoinSubstitution([pkg_gz, 'worlds', f'{world_value}.sdf'])
    elif os.path.isabs(world_value):
        # Absolute path provided
        world_file = world_value
    else:
        # Assume it's a relative path from our worlds directory
        world_file = PathJoinSubstitution([pkg_gz, 'worlds', world_value])

    # Build launch actions list
    launch_actions = [
        # Set for Ignition Fortress
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=ign_path
        ),
        # Set for newer Gazebo Sim (gz sim)
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_path
        ),
    ]

    # # Launch Gazebo after marker generation completes
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    #     ),
    #     launch_arguments={
    #         'gz_args': ['-r ', world_file]
    #     }.items()
    # )
    gazebo_launch = GroupAction([
        # # Force Gazebo GUI rendering on the NVIDIA GPU (PRIME offload)
        # SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        # SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        # # Optional (often helps Vulkan/Optimus selection, harmless if unused)
        # SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                )
            ),
            launch_arguments={
                'gz_args': ['-r ', world_file]
            }.items()
        )
    ])

    # Conditionally add marker generation executable
    if generate_markers_value.lower() in ['true', '1', 'yes']:
        # Get the path to the installed script
        pkg_prefix = get_package_prefix('erwinia_os_gz')
        script_path = os.path.join(pkg_prefix, 'lib', 'erwinia_os_gz', 'generate_aruco_models.py')
        
        marker_generator = ExecuteProcess(
            cmd=[
                os.path.join(pkg_prefix, 'lib', 'erwinia_os_gz', 'generate_aruco_models'),
                '--count', num_markers_value,
                '--dict', marker_dict_value,
                '--marker_size', marker_size_value,
            ],
            name='generate_aruco_markers',
            output='screen',
        )
        launch_actions.append(marker_generator)
        
        # # Launch Gazebo after marker generation completes
        # gazebo_launch = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        #     ),
        #     launch_arguments={
        #         'gz_args': ['-r ', world_file]
        #     }.items()
        # )
        
        # Register event handler to launch Gazebo after marker generation
        launch_actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=marker_generator,
                    on_exit=[gazebo_launch]
                )
            )
        )

    else:
        # Launch Gazebo immediately if not generating markers
        # launch_actions.append(
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(
        #             os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        #         ),
        #         launch_arguments={
        #             'gz_args': ['-r ', world_file]
        #         }.items()
        #     )
        # )
        launch_actions.append(gazebo_launch)
    
    # Clock bridge - always add
    if LaunchConfiguration('launch_clock_bridge').perform(context).lower() in ['true', '1', 'yes']:
        launch_actions.append(
            Node(
                name='clock_bridge',
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
                ],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        )

    return launch_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='apple_orchard',
            description='World to load: "empty", "apple_orchard", custom world name, or absolute path to .sdf file'
        ),
        DeclareLaunchArgument(
            'generate_markers',
            default_value='true',
            description='Whether to generate ArUco marker models before launching (true/false)'
        ),
        DeclareLaunchArgument(
            'num_markers',
            default_value='20',
            description='Number of ArUco markers to generate (0 to n-1)'
        ),
        DeclareLaunchArgument(
            'marker_dict',
            default_value='DICT_4X4_50',
            description='ArUco dictionary to use for marker generation'
        ),
        DeclareLaunchArgument(
            'marker_size',
            default_value='0.05',
            description='Physical size of markers in meters'
        ),
        DeclareLaunchArgument(
            'launch_clock_bridge',
            default_value='true',
            description='Whether to launch the ROS-Gazebo clock bridge (true/false)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
