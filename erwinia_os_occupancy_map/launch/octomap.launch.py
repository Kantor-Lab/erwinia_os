"""
Launch file for occupancy map server
Optionally publishes to MoveIt planning scene if use_moveit is enabled
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Core args ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.04',
        description='Octomap resolution in meters'
    )

    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='0.6',
        description='Maximum sensor range in meters'
    )

    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='false',
        description='Enable MoveIt planning scene integration'
    )

    use_bbox_arg = DeclareLaunchArgument(
        'use_bbox',
        default_value='true',
        description='Enable bounding box for octomap updates'
    )

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='amiga/base_footprint',
        description='Frame for the occupancy map'
    )

    use_semantics_arg = DeclareLaunchArgument(
        'use_semantics',
        default_value='true',
        description='Enable semantic occupancy mapping mode'
    )

    # --- PointCloud updater topic ---
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/firefly_left/points2',
        description='PointCloud2 topic for PointCloudUpdater'
    )

    # Octomap server node with inline parameters
    octomap_server_node = Node(
        package='erwinia_os_occupancy_map',
        executable='octomap_server',
        name='octomap_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),

            # Octomap parameters
            'resolution': LaunchConfiguration('resolution'),
            'map_frame': LaunchConfiguration('map_frame'),

            # Sensor model
            'max_range': LaunchConfiguration('max_range'),
            'min_range': 0.01,
            'prob_hit': 0.7,
            'prob_miss': 0.4,
            'clamp_min': 0.12,
            'clamp_max': 0.97,
            'occupancy_threshold': 0.5,

            # # Ground filtering
            # 'filter_ground_plane': False, # Not using ground filtering
            # 'ground_distance_threshold': 0.04,

            # Bounding box (limits octomap updates to this region)
            'use_bounding_box': LaunchConfiguration('use_bbox'),
            # # Side Box
            # 'bbx_min_x': -1.0,
            # 'bbx_min_y': -2.0,
            # 'bbx_min_z': 0.0,
            # 'bbx_max_x': 1.0,
            # 'bbx_max_y': -0.5,
            # 'bbx_max_z': 2.0,
            # Side Box (smaller)
            'bbx_min_x': -0.6,
            'bbx_min_y': -1.8,
            'bbx_min_z': 0.0,
            'bbx_max_x': 0.6,
            'bbx_max_y': -0.6,
            'bbx_max_z': 2.0,
            # # Rear Box
            # 'bbx_min_x': -2.0,
            # 'bbx_min_y': -1.0,
            # 'bbx_min_z': 0.0,
            # 'bbx_max_x': -0.5,
            # 'bbx_max_y': 1.0,
            # 'bbx_max_z': 2.0,

            # Semantic mode
            'use_semantics': LaunchConfiguration('use_semantics'),

            # Semantic fusion parameters
            'semantic_confidence_boost': 0.05,
            'semantic_mismatch_penalty': 0.1,

            # PointCloud updater
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),

            # MoveIt + publishing
            'use_moveit': LaunchConfiguration('use_moveit'),
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

    return LaunchDescription([
        use_sim_time_arg,
        resolution_arg,
        max_range_arg,
        use_moveit_arg,
        use_bbox_arg,
        map_frame_arg,

        use_semantics_arg,
        pointcloud_topic_arg,

        octomap_server_node
    ])
