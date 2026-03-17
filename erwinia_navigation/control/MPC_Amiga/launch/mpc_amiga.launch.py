# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue

# def generate_launch_description():
#     # Declare the 'fresh_start' argument
#     fresh_start_arg = DeclareLaunchArgument(
#         'fresh_start', default_value='false', description='Fresh start argument'
#     )

#     # Define the launch description
#     return LaunchDescription([
#         # Declare launch arguments
#         fresh_start_arg,

#         # Robot controller node
#         Node(
#             package='mpc_amiga',
#             executable='mpc_warthog_v4.py',  # Make sure this matches your ROS2 executable name
#             name='robot_controller',
#             output='screen',
#             parameters=[
#                 {'pruning_status': True},  # Set parameter pruning_status to True
#                 {'nav_stat': True}      # Set parameter nav_status to True
#             ],
#             arguments=[LaunchConfiguration('fresh_start')]  # Pass 'fresh_start' argument
#         )
#     ])
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the RViz config file in the current package
    rviz_config_file = os.path.join(
        get_package_share_directory('mpc_amiga'), 'rviz', 'view_path.rviz'
    )

    return LaunchDescription([
        # Robot controller node
        Node(
            package='mpc_amiga',
            executable='mpc_warthog_v4.py',  # Ensure this matches your ROS2 executable
            name='robot_controller',
            output='screen',
            parameters=[
                {'pruning_status': True},  # Set parameter pruning_status to True
                {'nav_stat': True}         # Set parameter nav_stat to True
            ]
        ),

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]  # Load the specified RViz config file
        ),

        # Additional node from another package
        # Node(
        #     package='mpc_amiga',  # Replace with the actual package name
        #     executable='convert_from_swiftnav.py',  # Ensure this is the correct executable/script
        #     name='convert_ned_enu',
        #     output='screen'
        # )
    ])
