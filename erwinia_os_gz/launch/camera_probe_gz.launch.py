import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import load_xacro
from launch_ros.parameter_descriptions import ParameterValue


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
    launch_nodes = []

    # ------------------------------------------------------------
    # 1) Launch Gazebo world (your existing launch)
    # ------------------------------------------------------------
    pkg_gz = get_package_share_directory('erwinia_os_gz')
    launch_nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gz, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'generate_markers': 'false',
                'launch_clock_bridge': 'false', # disable clock bridge in nested launch, we'll add it ourselves
            }.items()
        )
    )
    clock_bridge = Node(
            name='clock_bridge',
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    launch_nodes.append(clock_bridge)

    # ------------------------------------------------------------
    # 2) Initialize the camera rig model (spawn as Gazebo entity)
    # ------------------------------------------------------------
    description_pkg = get_package_share_directory('multi_camera_rig_description')
    xacro_file = Path(description_pkg) / 'urdf' / 'multi_camera_rig_description.urdf.xacro'

    # Process the xacro files
    robot_description = get_xacro_content(
        context,
        xacro_file=xacro_file,
        use_gazebo='true',
    )

    # Publish the robot state to tf (use simulated time)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True}, 
            {'robot_description': robot_description},
        ]
    )
    launch_nodes.append(robot_state_publisher)

    map_frame_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'husky/a200_base_footprint'],
        output='screen'
    )
    launch_nodes.append(map_frame_publisher)

    # Spawn into Gazebo as entity camera_probe
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'multi_camera_rig',
                '-topic', 'robot_description',
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'),
            ],
        )
    launch_nodes.append(spawn_node)

    # ------------------------------------------------------------
    # 3) Initialize the camera rig image capture pipeline (trigger + transport)
    # ------------------------------------------------------------
    firefly_bringup_pkg = get_package_share_directory('firefly-ros2-wrapper-bringup')
    firefly_bringup_launch = os.path.join(firefly_bringup_pkg, 'launch', 'bringup.launch.py')
    
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(firefly_bringup_launch),
        launch_arguments={
            'use_gazebo': 'true',
            'trigger_auto_connect': 'true',
            'trigger_auto_start': 'false',
        }.items()
    )
    launch_nodes.append(camera_launch)

    # ------------------------------------------------------------
    # 4) Initialize the rectify/scale nodes (one per camera)
    # ------------------------------------------------------------
    for cam_name, in_img, in_info, out_rect in [
        (f'firefly_left',  f'/firefly_left/image_raw',  f'/firefly_left/camera_info',  LaunchConfiguration('left_rect_image').perform(context)),
    ]:
        launch_nodes.append(
            Node(
                package='multi_camera_rig_reconstruction',
                executable='stereo_rectify_scale_node',
                name=f'{cam_name}_rectify_scale',
                output='screen',
                parameters=[{
                    'in_image_topic': in_img,
                    'in_info_topic': in_info,
                    'out_rect_image_topic': out_rect,
                    'out_rect_info_topic': f'/{cam_name}/camera_info_rect',
                    'publish_scaled': False,

                    'output_width': 1440,
                    'output_height': 1088,
                    'interpolation': 'linear',

                    # QoS (match your pipeline)
                    'sub_qos.reliability': 'reliable',
                    'sub_qos.durability': 'volatile',
                    'sub_qos.history': 'keep_last',
                    'sub_qos.depth': 5,
                    'pub_qos.reliability': 'best_effort',
                    'pub_qos.durability': 'volatile',
                    'pub_qos.history': 'keep_last',
                    'pub_qos.depth': 5,
                }]
            )
        )

    # ------------------------------------------------------------
    # 5) Image saver nodes (let these do all disk saving)
    # ------------------------------------------------------------
    save_root = LaunchConfiguration('save_directory').perform(context)
    if save_root:
        for cam_name, rect_topic in [
            ('firefly_left', LaunchConfiguration('left_rect_image').perform(context)),
        ]:
            launch_nodes.append(
                Node(
                    package='multi_camera_rig_bringup',  # <-- where image_saver_node is
                    executable='image_saver_node',
                    name=f'{cam_name}_image_saver',
                    output='screen',
                    parameters=[{
                        'image_topic': rect_topic,
                        'save_directory': os.path.join(save_root, cam_name),
                        'image_prefix': cam_name,
                        'image_format': LaunchConfiguration('image_format').perform(context),
                        'qos_depth': 5,
                        'qos_reliability': 'best_effort',
                        'qos_durability': 'volatile',
                        'qos_history': 'keep_last',
                    }]
                )
            )

    # ------------------------------------------------------------
    # 6) Capture mover node (teleport + trigger + metadata)
    # ------------------------------------------------------------
    world_name = LaunchConfiguration('world').perform(context)  # apple_tree_1
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_bridge',
        arguments=[
            f'/world/{LaunchConfiguration("world").perform(context)}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    launch_nodes.append(pose_bridge)

    capture_node = Node(
        package='erwinia_os_gz',
        executable='gazebo_viewpoint_capture_node',
        name='gazebo_viewpoint_capture_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,

            # ---- GT + world ----
            'gt_yaml': LaunchConfiguration('gt_yaml'),
            'world_name': LaunchConfiguration('world'),
            'camera_entity': 'multi_camera_rig',

            # ---- Frames (fixed TF lookup) ----
            'entity_frame': 'multi_camera_rig_mount',
            'optical_frame': 'firefly_left_camera_optical_frame',

            # ---- Viewpoint pattern ----
            # front  = decreasing Y
            # back   = increasing Y
            'side': LaunchConfiguration('side'), # 'front' or 'back'
            'distances': [0.2, 0.5],
            'angle_deg': LaunchConfiguration('angle_deg'), # 0 for front/back, 90 for left/right

            # ---- Timing ----
            'settle_s': 0.1,
            'pose_timeout_s': 0.2,
            'trigger_timeout_s': 0.2,

            # ---- Trigger ----
            'use_trigger': True,
            'trigger_service': LaunchConfiguration('trigger_service'),

            # ---- Loop ----
            'loop': False,
        }]
    )

    # Start capture node *after* spawn completes, with a small delay
    launch_nodes.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_node,
                on_exit=[
                    TimerAction(period=2.0, actions=[capture_node])
                ],
            )
        )
    )

    return launch_nodes


def generate_launch_description():
    return LaunchDescription([
        # Gazebo world args
        DeclareLaunchArgument('world', default_value='apple_orchard'),
        DeclareLaunchArgument('side', default_value='front'),
        DeclareLaunchArgument('angle_deg', default_value='30.0'),

        # Spawn camera rig entity
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='1.2'),

        # Trigger node executable location
        DeclareLaunchArgument('trigger_service', default_value='/trigger/send_trigger'),

        # Rectified output topics
        DeclareLaunchArgument('left_rect_image', default_value='/firefly_left/image_rect'),
        DeclareLaunchArgument('right_rect_image', default_value='/firefly_right/image_rect'),

        # Dataset params
        DeclareLaunchArgument('gt_yaml', default_value=''),
        DeclareLaunchArgument('save_directory', default_value='/home/hayden/tmp/saved_images'),
        DeclareLaunchArgument('image_format', default_value='png'),

        OpaqueFunction(function=launch_setup),
    ])