from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    interactive = LaunchConfiguration("interactive")
    interactive_input_device = LaunchConfiguration("interactive_input_device")
    joy_topic = LaunchConfiguration("joy_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    bt_executor = Node(
        package="erwinia_bt",
        executable="bt_executor",
        name="bt_executor",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"interactive_input_device": interactive_input_device},
            {"interactive_joy_topic": joy_topic},
        ],
    )

    amiga_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("amiga_control"), "launch", "control.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "joy_topic": joy_topic,
            "cmd_vel_topic": cmd_vel_topic,
        }.items(),
        condition=IfCondition(interactive),
    )

    navigation_server = Node(
        package="erwinia_nav_server",
        executable="navigation_server",
        name="navigation_server",
        output="screen",
        condition=UnlessCondition(interactive),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "interactive",
                default_value="true",
                description="Run BT executor in interactive mode with manual navigation.",
            ),
            DeclareLaunchArgument(
                "interactive_input_device",
                default_value="xbox",
                description="Interactive BT input device. Use 'xbox' for controller-driven flow.",
            ),
            DeclareLaunchArgument(
                "joy_topic",
                default_value="joy",
                description="Joystick topic used by the BT interactive input and teleop.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="cmd_vel",
                description="Velocity command topic published by teleop in interactive mode.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS clock published on /clock.",
            ),
            SetEnvironmentVariable("BT_INTERACTIVE", "1", condition=IfCondition(interactive)),
            SetEnvironmentVariable("BT_INTERACTIVE", "0", condition=UnlessCondition(interactive)),
            bt_executor,
            amiga_control_launch,
            navigation_server,
        ]
    )
