from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_param_builder import load_xacro
from pathlib import Path
import yaml
import tempfile

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

    description_pkg = Path(get_package_share_directory('erwinia_os_description'))
    xacro_file = description_pkg / 'urdf' / 'erwinia_os.urdf.xacro'
    control_pkg = Path(get_package_share_directory('erwinia_os_control'))
    moveit_config_pkg = Path(get_package_share_directory('erwinia_os_moveit_config'))

    # Generate the moveit_controllers.yaml file dynamically
    # 1. Generate the config dictionary
    config = generate_moveit_controllers_yaml(
        manipulator_ns=manipulator_ns.perform(context),
        manipulator_prefix=manipulator_prefix.perform(context)
    )
    # 2. Create a temporary file to hold the YAML
    # We use NamedTemporaryFile so it persists long enough for the nodes to read it
    # OS cleans /tmp automatically on reboot, or you can manage cleanup.
    moveit_controllers_yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(config, moveit_controllers_yaml_file, default_flow_style=False, sort_keys=False)
    moveit_controllers_yaml_file.close()  # Close so other processes can read it
    
    # Load the dynamically generated controllers
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

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description,
            robot_description_semantic,
            kinematics_config,
            planner_config,
            controller_config,
            joint_limits_config,
            planning_scene_monitor_config,
        ],
    )

    return [move_group_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_gazebo', default_value='false'),
        DeclareLaunchArgument('manipulator_prefix', default_value='xarm6_'),
        DeclareLaunchArgument('manipulator_ns', default_value='xarm'),
        DeclareLaunchArgument('platform_prefix', default_value=''),
        DeclareLaunchArgument('platform_ns', default_value='amiga'),
        OpaqueFunction(function=launch_setup),
    ])
