from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('vectornav'),
                '/launch/vectornav.launch.py',
            ]
        )
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('velodyne'),
                '/launch/velodyne-all-nodes-VLP16-launch.py',
            ]
        )
    )

    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('swiftnav_ros2_driver'),
                '/launch/start.py',
            ]
        )
    )

    return LaunchDescription([
        vectornav_launch,
        velodyne_launch,
        gps_launch,
    ])
