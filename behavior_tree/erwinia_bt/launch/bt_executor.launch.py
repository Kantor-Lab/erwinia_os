from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    interactive = LaunchConfiguration("interactive")
    interactive_input_device = LaunchConfiguration("interactive_input_device")
    joy_topic = LaunchConfiguration("joy_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    num_trees = LaunchConfiguration("num_trees")

    bt_executor = Node(
        package="erwinia_bt",
        executable="bt_executor",
        name="bt_executor",
        output="screen",
        parameters=[
            {"num_trees": num_trees},
            {"use_sim_time": use_sim_time},
            {"interactive_input_device": interactive_input_device},
            {"interactive_joy_topic": joy_topic},
        ],
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
                "num_trees",
                default_value="2",
                description="Number of sequential BT tree targets to generate as tree_1..tree_N.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS clock published on /clock.",
            ),
            SetEnvironmentVariable("BT_INTERACTIVE", "1", condition=IfCondition(interactive)),
            SetEnvironmentVariable("BT_INTERACTIVE", "0", condition=UnlessCondition(interactive)),
            bt_executor,
        ]
    )
