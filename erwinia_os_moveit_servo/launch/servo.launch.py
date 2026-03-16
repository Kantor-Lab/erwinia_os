import os
import yaml
import tempfile
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue
import xacro

from erwinia_os_moveit_config.generate_moveit_controllers_yaml import generate_moveit_controllers_yaml


def _xacro_param(xacro_path, *args):
    """Build a ParameterValue wrapping a xacro Command.

    Args in *args may include strings and Substitutions (like LaunchConfiguration).
    """
    cmd = ["xacro ", str(xacro_path)] + list(args)
    return ParameterValue(Command(cmd), value_type=str)

def load_yaml(path):
    return yaml.safe_load(Path(path).read_text())

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo = LaunchConfiguration('use_gazebo')
    manipulator_prefix = LaunchConfiguration('manipulator_prefix')
    manipulator_ns = LaunchConfiguration('manipulator_ns')
    platform_prefix = LaunchConfiguration('platform_prefix')
    platform_ns = LaunchConfiguration('platform_ns')

    description_pkg = Path(get_package_share_directory('erwinia_os_description'))
    moveit_config_pkg = Path(get_package_share_directory('erwinia_os_moveit_config'))
    servo_pkg = Path(get_package_share_directory('erwinia_os_moveit_servo'))

    # Generate the moveit_controllers.yaml file dynamically
    config = generate_moveit_controllers_yaml(
        manipulator_ns=manipulator_ns.perform(context),
        manipulator_prefix=manipulator_prefix.perform(context)
    )
    moveit_controllers_yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(config, moveit_controllers_yaml_file, default_flow_style=False, sort_keys=False)
    moveit_controllers_yaml_file.close()
    
    # controller_config = yaml.safe_load(Path(moveit_controllers_yaml_file.name).read_text())

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
    kinematics_yaml = load_yaml(moveit_config_pkg / 'config' / 'kinematics.yaml')
    if kinematics_yaml is None:
        raise RuntimeError(f"Failed to load kinematics.yaml from {moveit_config_pkg / 'config' / 'kinematics.yaml'}")
    kinematics_config = {'robot_description_kinematics': kinematics_yaml}

    # Load joint limits configuration
    joint_limits_yaml = load_yaml(moveit_config_pkg / 'config' / 'joint_limits.yaml')
    if joint_limits_yaml is None:
        raise RuntimeError(f"Failed to load joint_limits.yaml from {moveit_config_pkg / 'config' / 'joint_limits.yaml'}")
    joint_limits_config = {'robot_description_planning_joint_limits': joint_limits_yaml}

    # Get parameters for the Servo node
    servo_config_path = servo_pkg / "config" / "servo.yaml"
    servo_yaml = load_yaml(servo_config_path)
    if servo_yaml is None:
        raise RuntimeError(f"Failed to load servo.yaml from {servo_config_path}. Please create this file.")
    servo_params = {"moveit_servo": servo_yaml}

    # # RViz (optional - adjust config path as needed)
    # rviz_config_file = str(moveit_config_pkg / "config" / "moveit.rviz")
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         kinematics_config,
    #         {'use_sim_time': use_sim_time}
    #     ],
    # )

    # # ros2_controllers path
    # ros2_controllers_path = str(moveit_config_pkg / "config" / "ros2_controllers.yaml")

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # ComposableNode(
            #     package="robot_state_publisher",
            #     plugin="robot_state_publisher::RobotStatePublisher",
            #     name="robot_state_publisher",
            #     parameters=[robot_description, {'use_sim_time': use_sim_time}],
            # ),
            ComposableNode(
                package="erwinia_os_moveit_servo",
                plugin="erwinia_os_moveit_servo::XboxToServoPub",
                name="xbox_to_servo",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "joy_topic": "/joy",
                    "twist_topic": "/servo_node/delta_twist_cmds",
                    "joint_topic": "/servo_node/delta_joint_cmds",
                    "base_frame_id": "xarm/xarm6_base_link",
                    "ee_frame_id": "xarm/xarm6_link_eef",
                    "enable_button": 5,  # RB by default on xbox mapping above; adjust if needed
                    "autostart_servo": True,
                    "enable_joint_jog": False,
                }],
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ],
        output="screen",
    )

    # Launch a standalone Servo node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            {'use_sim_time': use_sim_time}
        ],
        output="screen",
    )

    return [
        # rviz_node,
        servo_node,
        container,
    ]


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