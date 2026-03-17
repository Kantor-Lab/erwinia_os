from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    joy_topic = LaunchConfiguration("joy_topic")

    amiga_control_node = Node(
        package="amiga_control",
        executable="amiga_control",
        name="amiga_control_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", cmd_vel_topic)],
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {
                "require_enable_button": True,
                "enable_button": 7,
                "enable_turbo_button": 4,
                "axis_linear.x": 1,
                "scale_linear.x": 0.6,
                "scale_linear_turbo.x": 1.0,
                "axis_angular.yaw": 0,
                "scale_angular.yaw": 0.6,
                "scale_angular_turbo.yaw": 1.0,
            },
        ],
        remappings=[
            ("cmd_vel", cmd_vel_topic),
            ("joy", joy_topic),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS clock published on /clock.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="cmd_vel",
                description="Twist topic used between teleop and Amiga control.",
            ),
            DeclareLaunchArgument(
                "joy_topic",
                default_value="joy",
                description="Joystick topic consumed by teleop_twist_joy.",
            ),
            amiga_control_node,
            teleop_twist_joy_node,
        ]
    )
