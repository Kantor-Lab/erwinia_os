# nbv_demo.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_param_builder import load_xacro
from pathlib import Path
from datetime import datetime
import yaml
import tempfile
import os
import glob

from erwinia_os_moveit_config.generate_moveit_controllers_yaml import generate_moveit_controllers_yaml


def load_yaml(path):
    return yaml.safe_load(Path(path).read_text())


def _xacro_param(xacro_path, *args):
    """Build a ParameterValue wrapping a xacro Command.

    Args in *args may include strings and Substitutions (like LaunchConfiguration).
    """
    cmd = ["xacro ", str(xacro_path)] + list(args)
    return ParameterValue(Command(cmd), value_type=str)


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo = LaunchConfiguration('use_gazebo')
    manipulator_prefix = LaunchConfiguration('manipulator_prefix')
    manipulator_ns = LaunchConfiguration('manipulator_ns')
    platform_prefix = LaunchConfiguration('platform_prefix')
    platform_ns = LaunchConfiguration('platform_ns')
    manipulator_group_name = LaunchConfiguration('manipulator_group_name')
    learn_workspace = LaunchConfiguration('learn_workspace')

    # Number of repeated runs
    n_runs = int(LaunchConfiguration('n_runs').perform(context))
    if n_runs < 1:
        n_runs = 1

    # Get package share directory and setup data directory
    package_share_dir = get_package_share_directory('erwinia_os_nbv_planner')
    data_dir = os.path.join(package_share_dir, 'data')
    os.makedirs(data_dir, exist_ok=True)

    # Setup metrics directory structure
    metrics_dir = LaunchConfiguration('metrics_dir').perform(context)
    # if not os.path.isabs(metrics_dir):
    #     metrics_dir = os.path.join(package_share_dir, metrics_dir)
    run_dir = LaunchConfiguration('run').perform(context)
    if run_dir == '':
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = f'run_{timestamp}'

    base_run_dir = os.path.join(metrics_dir, run_dir)

    # If n_runs > 1:
    #   - pass base_run_dir to the C++ node and it will create run_001/... subfolders
    # If n_runs == 1:
    #   - keep old behavior: base_run_dir/{plots,data}
    if n_runs > 1:
        metrics_plots_dir = base_run_dir
        metrics_data_dir = base_run_dir
        os.makedirs(base_run_dir, exist_ok=True)
    else:
        metrics_plots_dir = os.path.join(base_run_dir, 'plots')
        metrics_data_dir = os.path.join(base_run_dir, 'data')
        os.makedirs(metrics_plots_dir, exist_ok=True)
        os.makedirs(metrics_data_dir, exist_ok=True)

    print(f"[nbv_demo.launch.py] n_runs: {n_runs}")
    print(f"[nbv_demo.launch.py] Metrics plots directory: {metrics_plots_dir}")
    print(f"[nbv_demo.launch.py] Metrics data directory: {metrics_data_dir}")

    # Determine workspace file path
    learn_workspace_val = learn_workspace.perform(context).lower() == 'true'

    if learn_workspace_val:
        # Learning mode: generate timestamped filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        workspace_file_path = os.path.join(data_dir, f'workspace_{timestamp}.bin')
        print(f"[nbv_demo.launch.py] Will learn and save workspace to: {workspace_file_path}")
    else:
        # Loading mode: find most recent workspace file
        workspace_files = glob.glob(os.path.join(data_dir, 'workspace_*.bin'))

        if workspace_files:
            workspace_files.sort(key=os.path.getmtime, reverse=True)
            workspace_file_path = workspace_files[0]
            print(f"[nbv_demo.launch.py] Found {len(workspace_files)} workspace file(s), loading most recent: {os.path.basename(workspace_file_path)}")
        else:
            workspace_file_path = os.path.join(data_dir, 'workspace_latest.bin')
            print(f"[nbv_demo.launch.py] WARNING: No workspace files found in {data_dir}")
            print(f"[nbv_demo.launch.py] Set learn_workspace:=true to learn a new workspace")

    # Ground truth file path
    gt_points_file = os.path.join(
        LaunchConfiguration('gt_points_dir').perform(context),
        LaunchConfiguration('gt_points_file').perform(context)
    )

    # Build parameters dictionary for saving and node configuration
    node_parameters = {
        # Number of repeated runs
        'n_runs': n_runs,
        'planner_type': LaunchConfiguration('planner_type').perform(context),

        # Workspace Parameters
        'manipulator_group_name': manipulator_group_name.perform(context),
        'learn_workspace': learn_workspace.perform(context).lower() == 'true',
        'num_samples': 10000000,
        'manipulation_workspace_file': workspace_file_path,

        # Octomap Parameters
        'octomap_topic': '/octomap_binary',
        'min_unknown_neighbors': 1,

        # Manipulation Parameters
        'planning_pipeline_id': 'ompl',
        'planner_id': 'RRTConnect',
        'planning_time': 0.5,
        'num_planning_attempts': 1,
        'max_velocity_scaling_factor': 0.3,
        'max_acceleration_scaling_factor': 0.3,
        'num_ik_seeds': 10,
        'plans_per_seed': 1,
        'ik_timeout': 0.01,
        'ik_attempts': 10,

        # Camera Parameters
        'capture_type': LaunchConfiguration('capture_type').perform(context),
        'camera_optical_link': LaunchConfiguration('camera_optical_link').perform(context),
        'camera_horizontal_fov_deg': 45.0,
        'camera_vertical_fov_deg': 35.0,
        'camera_scaled_width': int(LaunchConfiguration('camera_scaled_width').perform(context)),
        'camera_scaled_height': int(LaunchConfiguration('camera_scaled_height').perform(context)),
        'camera_max_range': float(LaunchConfiguration('camera_max_range').perform(context)),
        'ideal_camera_distance': 0.4,
        'ideal_distance_tolerance': 0.1,
        'num_camera_rays': int(LaunchConfiguration('num_camera_rays').perform(context)),

        # Evaluation Parameters
        'enable_evaluation': LaunchConfiguration('enable_evaluation').perform(context).lower() == 'true',
        'eval_threshold_radius': float(LaunchConfiguration('eval_threshold_radius').perform(context)),
        'gt_points_file': gt_points_file,
        'metrics_plots_dir': metrics_plots_dir,
        'metrics_data_dir': metrics_data_dir,

        # General Parameters
        # 'init_joint_angles_deg': [0.0, -45.0, -45.0, 0.0, 0.0, 90.0],
        'init_joint_angles_deg': [0.0, -20.0, -70.0, 0.0, 0.0, 90.0],
        # Flat list of preset joint configurations executed before NBV planning (groups of 6, in degrees).
        'preset_joint_configs_deg': [
            # 0.0, -70.0, -40.0, 0.0, 20.0, 90.0,   # top
            6.0, -70.0, -42.0, 36.0, 27.0, 55.0,   # top left
            -6.0, -70.0, -42.0, -36.0, 27.0, 125.0,   # top right
            -13.0, -64.0, -57.0, 28.0, 75.0, 55.0,   # bottom left
            13.0, -64.0, -57.0, -28.0, 75.0, 125.0,   # bottom right
        ],
        # Flat list of hint IK seed configs (groups of 6, in degrees).
        # Each group of 6 values is one joint configuration tried before jittered seeds.
        'hint_joint_angles_deg': [
            -42.0, -8.0, -119.0, 57.0, 54.0, 48.0, # Top Right
            42.0, -8.0, -119.0, 57.0, 54.0, 132.0, # Top Left
            -38.0, -35.0, -53.0, 92.0, 38.0, -3.0, # Mid Right
            38.0, -34.0, -54.0, -92.0, 38.0, 183.0, # Mid Left
            -50.0, 27.0, -31.0, 130.0, 87.0, -87.0, # Bottom Right
            50.0, 27.0, -31.0, 230.0, 87.0, -93.0, # Bottom Right
            -56.0, 97.0, -92.0, 124.0, 93.0, -94.0, # Far Bottom Right
            -56.0, 97.0, -92.0, 124.0, 93.0, -94.0, # Far Bottom Right
            56.0, 97.0, -91.0, 230.0, 93.0, -85.0, # Far Bottom Left
        ],
        'map_frame': LaunchConfiguration('map_frame').perform(context),

        # NBV Planning Parameters
        'max_iterations': int(LaunchConfiguration('max_iterations').perform(context)),
        'min_information_gain': float(LaunchConfiguration('min_information_gain').perform(context)),
        'alpha_cost_weight': float(LaunchConfiguration('alpha_cost_weight').perform(context)),
        'beta_semantic_weight': float(LaunchConfiguration('beta_semantic_weight').perform(context)),
        'conf_thresh': float(LaunchConfiguration('conf_thresh').perform(context)),
        'semantic_confidence_boost': float(LaunchConfiguration('semantic_confidence_boost').perform(context)),
        'semantic_mismatch_penalty': float(LaunchConfiguration('semantic_mismatch_penalty').perform(context)),
        'max_semantic_certainty': float(LaunchConfiguration('max_semantic_certainty').perform(context)), 

        # Viewpoint Parameters
        'plane_half_extent': 1.0,  # Not used in planner
        'plane_spatial_resolution': 0.1,  # Not used in planner
        'cap_max_theta_deg': 60.0,  # Not used in planner
        'cap_min_theta_deg': 30.0,  # Not used in planner
        'num_viewpoints_per_frontier': int(LaunchConfiguration('num_viewpoints_per_frontier').perform(context)),
        'z_bias_sigma': 0.5,
        'viewpoint_overlap_ratio': 0.40,

        # Debug Parameters
        'visualize': LaunchConfiguration('visualize_nbv').perform(context).lower() == 'true',
        'visualization_topic': LaunchConfiguration('visualization_topic').perform(context),
        'keep_alive': LaunchConfiguration('keep_alive').perform(context).lower() == 'true',
    }

    # Capture firefly parameters
    firefly_parameters = {
        'trigger_serial_port': LaunchConfiguration('trigger_serial_port').perform(context),
        'trigger_baudrate': int(LaunchConfiguration('trigger_baudrate').perform(context)),
        'trigger_flash_duration_ms': int(LaunchConfiguration('trigger_flash_duration_ms').perform(context)),
        'trigger_frame_rate_hz': int(LaunchConfiguration('trigger_frame_rate_hz').perform(context)),
        'trigger_auto_start': LaunchConfiguration('trigger_auto_start').perform(context).lower() == 'true',
        'output_width': int(LaunchConfiguration('camera_scaled_width').perform(context)),
        'output_height': int(LaunchConfiguration('camera_scaled_height').perform(context)),
        'stereo_matcher_model_trt': LaunchConfiguration('stereo_matcher_model_trt').perform(context),
        'use_semantics': LaunchConfiguration('use_semantics').perform(context).lower() == 'true',
        'use_seg_detection': LaunchConfiguration('use_seg_detection').perform(context).lower() == 'true',
        'enable_detection': LaunchConfiguration('enable_detection').perform(context).lower() == 'true',
        'detection_model_trt': LaunchConfiguration('detection_model_trt').perform(context),
    }

    # Capture octomap parameters
    octomap_parameters = {
        'resolution': float(LaunchConfiguration('octomap_resolution').perform(context)),
        'max_range': float(LaunchConfiguration('camera_max_range').perform(context)),
        'min_range': 0.01,
        'use_moveit': LaunchConfiguration('octomap_use_moveit').perform(context).lower() == 'true',
        'use_bbox': LaunchConfiguration('octomap_use_bbox').perform(context).lower() == 'true',
        'map_frame': LaunchConfiguration('map_frame').perform(context),
        'use_semantics': LaunchConfiguration('octomap_use_semantics').perform(context).lower() == 'true',
        'pointcloud_topic': LaunchConfiguration('octomap_pointcloud_topic').perform(context),
        'bbx_min_x': float(LaunchConfiguration('octomap_bbx_min_x').perform(context)),
        'bbx_min_y': float(LaunchConfiguration('octomap_bbx_min_y').perform(context)),
        'bbx_min_z': float(LaunchConfiguration('octomap_bbx_min_z').perform(context)),
        'bbx_max_x': float(LaunchConfiguration('octomap_bbx_max_x').perform(context)),
        'bbx_max_y': float(LaunchConfiguration('octomap_bbx_max_y').perform(context)),
        'bbx_max_z': float(LaunchConfiguration('octomap_bbx_max_z').perform(context)),
        'prob_hit': 0.7,
        'prob_miss': 0.4,
        'clamp_min': 0.12,
        'clamp_max': 0.97,
        'occupancy_threshold': 0.5,
        'semantic_confidence_boost': float(LaunchConfiguration('semantic_confidence_boost').perform(context)),
        'semantic_mismatch_penalty': float(LaunchConfiguration('semantic_mismatch_penalty').perform(context)),
        'conf_thresh': float(LaunchConfiguration('conf_thresh').perform(context)),
    }

    # Save parameters to YAML file (always in base_run_dir)
    os.makedirs(base_run_dir, exist_ok=True)
    params_file_path = os.path.join(base_run_dir, 'run_parameters.yaml')
    params_data = {
        'run_info': {
            'timestamp': run_dir,
            'launch_file': 'nbv_demo.launch.py',
            'node_name': 'nbv_demo',
            'description': 'NBV Demo - Systematic grid coverage',
        },
        'launch_arguments': {
            'use_sim_time': use_sim_time.perform(context),
            'use_gazebo': use_gazebo.perform(context),
            'manipulator_prefix': manipulator_prefix.perform(context),
            'manipulator_ns': manipulator_ns.perform(context),
            'platform_prefix': platform_prefix.perform(context),
            'platform_ns': platform_ns.perform(context),
            'n_runs': n_runs,
        },
        'parameters': node_parameters,
        'firefly_parameters': firefly_parameters,
        'octomap_parameters': octomap_parameters
    }

    with open(params_file_path, 'w') as f:
        yaml.dump(params_data, f, default_flow_style=False, sort_keys=False)

    print(f"[nbv_demo.launch.py] Saved run parameters to: {params_file_path}")

    description_pkg = Path(get_package_share_directory('erwinia_os_description'))
    xacro_file = description_pkg / 'urdf' / 'erwinia_os.urdf.xacro'
    control_pkg = Path(get_package_share_directory('erwinia_os_control'))
    moveit_config_pkg = Path(get_package_share_directory('erwinia_os_moveit_config'))

    # Generate the moveit_controllers.yaml file dynamically
    config = generate_moveit_controllers_yaml(
        manipulator_ns=manipulator_ns.perform(context),
        manipulator_prefix=manipulator_prefix.perform(context)
    )

    moveit_controllers_yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(config, moveit_controllers_yaml_file, default_flow_style=False, sort_keys=False)
    moveit_controllers_yaml_file.close()

    controller_config = yaml.safe_load(Path(moveit_controllers_yaml_file.name).read_text())

    # robot_description from URDF xacro
    robot_description = {'robot_description': _xacro_param(
        description_pkg / 'urdf' / 'erwinia_os.urdf.xacro',
        ' use_gazebo:=', use_gazebo,
        ' manipulator_prefix:=', manipulator_prefix,
        ' manipulator_ns:=', manipulator_ns,
        ' platform_prefix:=', platform_prefix,
        ' platform_ns:=', platform_ns
    )}

    # robot_description_semantic from SRDF xacro
    robot_description_semantic = {'robot_description_semantic': _xacro_param(
        description_pkg / 'srdf' / 'erwinia_os.srdf.xacro',
        ' manipulator_prefix:=', manipulator_prefix,
        ' manipulator_ns:=', manipulator_ns,
        ' platform_prefix:=', platform_prefix,
        ' platform_ns:=', platform_ns
    )}

    # Load kinematics configuration
    kinematic_yaml = load_yaml(moveit_config_pkg / 'config' / 'kinematics.yaml')
    kinematics_config = {'robot_description_kinematics': kinematic_yaml}

    # Load joint limits configuration
    joint_limits_yaml = load_yaml(moveit_config_pkg / 'config' / 'joint_limits.yaml')
    joint_limits_config = {'robot_description_planning_joint_limits': joint_limits_yaml}

    # Load the planning configs
    ompl_yaml = load_yaml(moveit_config_pkg / 'config' / 'ompl_planning.yaml')
    planner_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_yaml
    }

    # Load the planning scene monitor defaults
    planning_scene_monitor_config = {
        'planning_scene_monitor_options': {
            'name': 'planning_scene_monitor',
            'publish_robot_description': True,
            'publish_robot_description_semantic': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        }
    }

    # Firefly camera bringup
    firefly_bringup_pkg = get_package_share_directory('multi_camera_rig_bringup')
    firefly_bringup_launch = os.path.join(firefly_bringup_pkg, 'launch', 'firefly_bringup.launch.py')

    firefly_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(firefly_bringup_launch),
        launch_arguments={
            'use_gazebo': use_gazebo,
            'calib_dir': LaunchConfiguration('calib_dir'),
            'spinnaker_config_file': LaunchConfiguration('spinnaker_config_file'),
            'spinnaker_param_file': LaunchConfiguration('spinnaker_param_file'),
            'trigger_serial_port': LaunchConfiguration('trigger_serial_port'),
            'trigger_baudrate': LaunchConfiguration('trigger_baudrate'),
            'trigger_flash_duration_ms': LaunchConfiguration('trigger_flash_duration_ms'),
            'trigger_frame_rate_hz': LaunchConfiguration('trigger_frame_rate_hz'),
            'trigger_auto_start': LaunchConfiguration('trigger_auto_start'),
            'output_width': LaunchConfiguration('camera_scaled_width'),
            'output_height': LaunchConfiguration('camera_scaled_height'),
            'stereo_matcher_model_dir': LaunchConfiguration('stereo_matcher_model_dir'),
            'stereo_matcher_model_trt': LaunchConfiguration('stereo_matcher_model_trt'),
            'use_semantics': LaunchConfiguration('use_semantics'),
            'use_seg_detection': LaunchConfiguration('use_seg_detection'),
            'enable_detection': LaunchConfiguration('enable_detection'),
            'detection_model_dir': LaunchConfiguration('detection_model_dir'),
            'detection_model_trt': LaunchConfiguration('detection_model_trt'),
            'conf_thresh': LaunchConfiguration('conf_thresh'),
        }.items()
    )

    # Octomap server node
    octomap_server_node = Node(
        package='erwinia_os_occupancy_map',
        executable='octomap_server',
        name='octomap_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': LaunchConfiguration('octomap_resolution'),
            'map_frame': LaunchConfiguration('map_frame'),
            'max_range': LaunchConfiguration('camera_max_range'),
            'min_range': 0.01,
            'prob_hit': 0.7,
            'prob_miss': 0.4,
            'clamp_min': 0.12,
            'clamp_max': 0.97,
            'occupancy_threshold': 0.5,
            'use_bounding_box': LaunchConfiguration('octomap_use_bbox'),
            'bbx_min_x': LaunchConfiguration('octomap_bbx_min_x'),
            'bbx_min_y': LaunchConfiguration('octomap_bbx_min_y'),
            'bbx_min_z': LaunchConfiguration('octomap_bbx_min_z'),
            'bbx_max_x': LaunchConfiguration('octomap_bbx_max_x'),
            'bbx_max_y': LaunchConfiguration('octomap_bbx_max_y'),
            'bbx_max_z': LaunchConfiguration('octomap_bbx_max_z'),
            'use_semantics': LaunchConfiguration('octomap_use_semantics'),
            'semantic_confidence_boost': LaunchConfiguration('semantic_confidence_boost'),
            'semantic_mismatch_penalty': LaunchConfiguration('semantic_mismatch_penalty'),
            'pointcloud_topic': LaunchConfiguration('octomap_pointcloud_topic'),
            'use_moveit': LaunchConfiguration('octomap_use_moveit'),
            'planning_scene_world_topic': '/planning_scene_world',
            'publish_free_voxels': True,
            'enable_visualization': True,
            'visualization_topic': 'occupancy_map_markers',
            'visualization_rate': 1.0,
            'octomap_publish_rate': 1.0,
            'nbv_octomap_topic': '/octomap_binary',
            'nbv_octomap_qos_transient_local': True,
        }]
    )

    if LaunchConfiguration('planner_type').perform(context).lower() == 'semantic':
        nbv_node = Node(
            package='erwinia_os_nbv_planner',
            executable='nbv_semantic_planner_demo',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                robot_description,
                robot_description_semantic,
                kinematics_config,
                joint_limits_config,
                planner_config,
                controller_config,
                planning_scene_monitor_config,
                node_parameters,
            ],
        )
    elif LaunchConfiguration('planner_type').perform(context).lower() == 'volumetric':
        nbv_node = Node(
            package='erwinia_os_nbv_planner',
            executable='nbv_volumetric_planner_demo',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                robot_description,
                robot_description_semantic,
                kinematics_config,
                joint_limits_config,
                planner_config,
                controller_config,
                planning_scene_monitor_config,
                node_parameters,
            ],
        )
    # elif LaunchConfiguration('planner_type').perform(context).lower() == 'paper': # This is just a demo for the paper figures
    #     nbv_node = Node(
    #         package='erwinia_os_nbv_planner',
    #         executable='paper_demo',
    #         output='screen',
    #         parameters=[
    #             {'use_sim_time': use_sim_time},
    #             robot_description,
    #             robot_description_semantic,
    #             kinematics_config,
    #             joint_limits_config,
    #             planner_config,
    #             controller_config,
    #             planning_scene_monitor_config,
    #             node_parameters,
    #         ],
    #     )
    else:
        nbv_node = Node(
            package='erwinia_os_nbv_planner',
            executable='nbv_baseline_demo',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                robot_description,
                robot_description_semantic,
                kinematics_config,
                joint_limits_config,
                planner_config,
                controller_config,
                planning_scene_monitor_config,
                node_parameters,
            ],
        )
    

    actions = [firefly_launch, octomap_server_node, nbv_node]

    if not node_parameters['keep_alive']:
        actions.append(RegisterEventHandler(
            OnProcessExit(
                target_action=nbv_node,
                on_exit=[EmitEvent(event=Shutdown())]
            )
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_gazebo', default_value='false'),
        DeclareLaunchArgument('manipulator_prefix', default_value='xarm6_'),
        DeclareLaunchArgument('manipulator_ns', default_value='xarm'),
        DeclareLaunchArgument('platform_prefix', default_value=''),
        DeclareLaunchArgument('platform_ns', default_value='amiga'),
        DeclareLaunchArgument('manipulator_group_name', default_value='xarm6_manipulator'),
        DeclareLaunchArgument('planner_type', default_value='baseline',
                              description='Type of planner to use (e.g., baseline, volumetric)'),
        DeclareLaunchArgument('learn_workspace', default_value='false',
                              description='True to learn new workspace, False to load existing'),
        DeclareLaunchArgument('visualize_nbv', default_value='false',
                              description='Visualize nbv planner operation in RViz'),
        DeclareLaunchArgument('visualization_topic', default_value='nbv_planner_visualization',
                              description='Topic for workspace visualization markers'),
        DeclareLaunchArgument('keep_alive', default_value='false',
                              description='Keep node alive after planning completes (spin until Ctrl+C)'),
        DeclareLaunchArgument('n_runs', default_value='1',
                              description='Number of times to repeat the nbv planner (clears /occupancy_map/clear between runs)'),

        # NBV Planner Parameters
        DeclareLaunchArgument('max_iterations', default_value='100',
                              description='Maximum number of NBV planning iterations'),
        DeclareLaunchArgument('min_information_gain', default_value='1.0',
                              description='Minimum information gain threshold for termination'), # Decreased from 5.0
        DeclareLaunchArgument('alpha_cost_weight', default_value='0.1',
                              description='Weight for cost in utility function (IG - alpha*cost)'),
        DeclareLaunchArgument('beta_semantic_weight', default_value='0.7',
                              description='Weight for semantic information in utility function'),
        DeclareLaunchArgument('num_viewpoints_per_frontier', default_value='7',
                              description='Number of viewpoint candidates per frontier cluster'),

        # Camera Parameters
        DeclareLaunchArgument('capture_type', default_value='triggered',
                              description='Whether to use triggered captures ("triggered") or simulated continuous captures ("continuous")'),
        DeclareLaunchArgument('camera_optical_link', default_value='firefly_left_camera_optical_frame',
                              description='TF frame of the camera optical link'),
        DeclareLaunchArgument('camera_scaled_width', default_value='448',
                              description='Camera scaled image width (pixels)'),
        DeclareLaunchArgument('camera_scaled_height', default_value='224',
                              description='Camera scaled image height (pixels)'),
        DeclareLaunchArgument('camera_max_range', default_value='0.9',
                              description='Camera maximum sensing range (meters)'),
        DeclareLaunchArgument('num_camera_rays', default_value='25',
                              description='Number of rays for information gain computation'),
        DeclareLaunchArgument('map_frame', default_value='amiga/base_footprint',
                              description='Fixed frame for visualization markers'),

        # Ground Truth Evaluation Parameters
        DeclareLaunchArgument(
            'gt_points_dir',
            default_value=PathJoinSubstitution([FindPackageShare('erwinia_os_nbv_planner'), 'metrics', 'gt_points']),
            description='Directory containing ground truth points YAML files for semantic evaluation'),
        DeclareLaunchArgument('gt_points_file', default_value='aruco_gt_points_lab.yaml',
                              description='Path to the ground truth points YAML file for semantic evaluation'),
        DeclareLaunchArgument('enable_evaluation', default_value='false',
                              description='Enable semantic octomap evaluation against ground truth'),
        DeclareLaunchArgument('eval_threshold_radius', default_value='0.10',
                              description='Threshold radius (meters) for matching clusters to ground truth points'),
        DeclareLaunchArgument('metrics_dir', default_value='metrics',
                              description='Directory for saving metrics (plots and CSV data)'),
        DeclareLaunchArgument('run', default_value='',
                              description='Directory for saving run data'),

        # Firefly Camera Parameters
        DeclareLaunchArgument('calib_dir', default_value=PathJoinSubstitution([FindPackageShare('firefly-ros2-wrapper-bringup'), 'calibs']),
                              description='Directory containing camera calibration YAML files'),
        DeclareLaunchArgument('spinnaker_config_file', default_value=PathJoinSubstitution([FindPackageShare('firefly-ros2-wrapper-bringup'), 'configs', 'firefly.yaml']),
                              description='Path to the Spinnaker camera configuration YAML file'),
        DeclareLaunchArgument('spinnaker_param_file', default_value=PathJoinSubstitution([FindPackageShare('firefly-ros2-wrapper-bringup'), 'params', 'firefly.yaml']),
                              description='Path to the Spinnaker camera parameter definitions YAML file'),
        DeclareLaunchArgument('trigger_serial_port', default_value='/dev/ttyUSB0',
                              description='Serial port for trigger connection (real hardware only)'),
        DeclareLaunchArgument('trigger_baudrate', default_value='9600',
                              description='Baudrate for trigger serial connection (real hardware only)'),
        DeclareLaunchArgument('trigger_flash_duration_ms', default_value='200',
                              description='Flash duration in milliseconds (0-300)'),
        DeclareLaunchArgument('trigger_frame_rate_hz', default_value='1',
                              description='Trigger frame rate in Hz (1-20)'),
        DeclareLaunchArgument('trigger_auto_start', default_value='false',
                              description='Automatically start video triggering on launch'),
        DeclareLaunchArgument('stereo_matcher_model_dir', default_value=PathJoinSubstitution([FindPackageShare('multi_camera_rig_reconstruction'), 'models']),
                              description='Directory containing TensorRT engine (.plan) files'),
        DeclareLaunchArgument('stereo_matcher_model_trt', default_value='fs_224x448_vit-small_iters5.plan',
                              description='TensorRT engine file for the foundation stereo model'),
        DeclareLaunchArgument('use_semantics', default_value='true',
                              description='Enable semantic mode with detections for point cloud'),
        DeclareLaunchArgument('use_seg_detection', default_value='true',
                              description='Use segmentation detections for semantic point cloud coloring'),
        DeclareLaunchArgument('enable_detection', default_value='true',
                              description='Enable YOLO detection node'),
        DeclareLaunchArgument('detection_model_dir', default_value=PathJoinSubstitution([FindPackageShare('multi_camera_rig_detection'), 'models']),
                              description='Directory containing TensorRT engine (.plan) files'),
        DeclareLaunchArgument('detection_model_trt', default_value='best_lab_seg_v2.plan',
                              description='TensorRT engine file for YOLO detection model'),
        DeclareLaunchArgument('conf_thresh', default_value='0.3',
                              description='Confidence threshold for YOLO detections'),

        # Octomap Parameters
        DeclareLaunchArgument('octomap_resolution', default_value='0.04',
                              description='Octomap resolution in meters'),
        DeclareLaunchArgument('octomap_use_moveit', default_value='false',
                              description='Enable MoveIt planning scene integration'),
        DeclareLaunchArgument('octomap_use_bbox', default_value='true',
                              description='Enable bounding box for octomap updates'),
        DeclareLaunchArgument('octomap_use_semantics', default_value='true',
                              description='Enable semantic occupancy mapping mode'),
        DeclareLaunchArgument('octomap_pointcloud_topic', default_value='/firefly_left/points2',
                              description='PointCloud2 topic for PointCloudUpdater'),
        DeclareLaunchArgument('octomap_bbx_min_x', default_value='-0.5'),
        DeclareLaunchArgument('octomap_bbx_max_x', default_value='0.5'),
        DeclareLaunchArgument('octomap_bbx_min_y', default_value='-2.1'),
        DeclareLaunchArgument('octomap_bbx_max_y', default_value='-1.1'),
        DeclareLaunchArgument('octomap_bbx_min_z', default_value='0.1'), # Increased this to avoid floor
        DeclareLaunchArgument('octomap_bbx_max_z', default_value='2.0'),
        DeclareLaunchArgument('semantic_confidence_boost', default_value='0.3',
                              description='Confidence boost for semantic occupancy updates'),
        DeclareLaunchArgument('semantic_mismatch_penalty', default_value='0.2',
                              description='Penalty for semantic label mismatches'),
        DeclareLaunchArgument('max_semantic_certainty', default_value='1.0',
                              description='Voxels with confidence above this threshold are considered '
                                           'fully certain and excluded from uncertain voxel search'),
        OpaqueFunction(function=launch_setup),
    ])