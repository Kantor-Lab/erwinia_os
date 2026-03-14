import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_desc = LaunchDescription()

    # First instance: receiver1
    config1 = os.path.join(
        get_package_share_directory('swiftnav_ros2_driver'),
        'config',
        'settings1.yaml'
    )
    node1 = Node(
        package='swiftnav_ros2_driver',
        executable='sbp-to-ros',
        namespace='sw_ns_1',  
        name='node',
        parameters=[config1]
    )

    # Second instance: receiver2
    config2 = os.path.join(
        get_package_share_directory('swiftnav_ros2_driver'),
        'config',
        'settings2.yaml'
    )
    node2 = Node(
        package='swiftnav_ros2_driver',
        executable='sbp-to-ros',
        namespace='sw_ns_2',
        name='node',
        parameters=[config2]
    )

    sync_node = Node(
        package='swiftnav_ros2_driver',
        executable='synced_baseline_node',
        name='sync_node',
    )    

    launch_desc.add_action(node1)
    launch_desc.add_action(node2)
    launch_desc.add_action(sync_node)

    return launch_desc

