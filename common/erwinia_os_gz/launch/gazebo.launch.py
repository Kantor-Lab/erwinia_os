import os
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from erwinia_os_gz.generate_world_sdf import generate_world_sdf


# World names that are individual model scenes and should use the
# dynamic SDF generator instead of the static .sdf world file.
APPLE_TREE_WORLDS = {'apple_tree_1', 'apple_tree_2', 'apple_tree_3', 'apple_tree_4', 'apple_tree_5'}


def launch_setup(context, *args, **kwargs):
    """
    Launch Gazebo with the specified world.
    Sets up resource paths for custom models and worlds.
    """
    world_value    = LaunchConfiguration('world').perform(context)
    position_value = LaunchConfiguration('position').perform(context)
    rotation_value = LaunchConfiguration('rotation').perform(context)

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

    # Determine the world file to load
    if world_value in APPLE_TREE_WORLDS:
        # Generate a temporary world SDF with the model at the given pose
        pos = position_value.split()
        rot = rotation_value.split()
        sdf_str = generate_world_sdf(
            model_name=world_value,
            x=float(pos[0]), y=float(pos[1]), z=float(pos[2]),
            roll=float(rot[0]), pitch=float(rot[1]), yaw=float(rot[2]),
        )
        tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False)
        tmp.write(sdf_str)
        tmp.close()
        world_file = tmp.name
    else:
        worlds = {f[:-4] for f in os.listdir(worlds_dir) if f.endswith('.sdf')}
        if world_value in worlds:
            world_file = PathJoinSubstitution([pkg_gz, 'worlds', f'{world_value}.sdf'])
        elif os.path.isabs(world_value):
            world_file = world_value
        else:
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
            default_value='empty',
            description='World to load: apple_tree_1..5, "empty", "apple_orchard", or absolute path to .sdf'
        ),
        DeclareLaunchArgument(
            'position',
            default_value='0.0 -1.6 0.0',
            description="Model position as 'x y z' in meters (used for apple_tree worlds)"
        ),
        DeclareLaunchArgument(
            'rotation',
            default_value='0.0 0.0 1.5708',
            description="Model rotation as 'roll pitch yaw' in radians (used for apple_tree worlds)"
        ),
        DeclareLaunchArgument(
            'launch_clock_bridge',
            default_value='true',
            description='Whether to launch the ROS-Gazebo clock bridge (true/false)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
