from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Arduino serial port used by the sprayer",
    )
    baud_arg = DeclareLaunchArgument(
        "baud",
        default_value="115200",
        description="Serial baud rate for the sprayer controller",
    )
    pulse_ms_arg = DeclareLaunchArgument(
        "pulse_ms",
        default_value="3000",
        description="Default pulse duration in milliseconds",
    )
    mark_action_name_arg = DeclareLaunchArgument(
        "mark_action_name",
        default_value="mark_server",
        description="Name of the MarkLocation action server",
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="false",
        description="Run without opening the serial port",
    )

    spray_node = Node(
        package="erwinia_spray",
        executable="spray_node.py",
        name="spray_node",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "baud": LaunchConfiguration("baud"),
                "pulse_ms": LaunchConfiguration("pulse_ms"),
                "mark_action_name": LaunchConfiguration("mark_action_name"),
                "dry_run": LaunchConfiguration("dry_run"),
            }
        ],
    )

    return LaunchDescription(
        [
            port_arg,
            baud_arg,
            pulse_ms_arg,
            mark_action_name_arg,
            dry_run_arg,
            spray_node,
        ]
    )
