"""
MPC + MPPI combo launch for the real robot.

Architecture:
  MPC  → cubic spline /aPath (high quality, speed-profiled)
  MPPI → follows /aPath + obstacle avoidance → /cmd_vel (sole controller)

Topic flow:
  MPC reads barn_field_waypoints.txt → publishes /aPath (cubic spline)
  MPC's own /cmd_vel → /mpc_cmd_vel_unused  (discarded)
  path_relay sends /aPath → controller_server via FollowPath action
  controller_server (MPPI) → /cmd_vel → robot

Usage:
  ros2 launch amiga_local_planner mpc_mppi_combo.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_lp  = get_package_share_directory('amiga_local_planner')
    costmap_config = os.path.join(pkg_lp, 'config', 'costmap.yaml')
    mppi_config    = os.path.join(pkg_lp, 'config', 'mppi.yaml')

    # 0. Zenoh router — required for rmw_zenoh_cpp topic discovery
    zenoh_router = ExecuteProcess(
        cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
        output='screen',
    )

    # 1. MPC node — cubic spline path + direct control output → /mpc_cmd_vel
    mpc_node = Node(
        package='mpc_amiga',
        executable='mpc_warthog_v4.py',
        name='robot_controller',
        output='screen',
        parameters=[{
            'pruning_status': True,
            'nav_stat': True,
            'use_teb': False,
        }],
        remappings=[('cmd_vel', 'mpc_cmd_vel')],
    )

    # 2. MPPI controller_server → /local_planner_cmd_vel (obstacle avoidance)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[costmap_config, mppi_config],
        remappings=[('cmd_vel', 'local_planner_cmd_vel')],
    )

    # 3. Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mppi',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['controller_server']}],
    )

    # 4. path_relay — sends MPC's /aPath to MPPI via FollowPath action
    path_relay = Node(
        package='amiga_local_planner',
        executable='path_relay.py',
        name='path_relay',
        output='screen',
    )

    # 5. RealSense camera
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        output='screen',
        parameters=[{'enable_color': True, 'enable_depth': False}],
    )

    # 6. Obstacle detector — checks costmap ahead of robot → /obstacle_nearby
    obstacle_detector = Node(
        package='amiga_local_planner',
        executable='obstacle_detector.py',
        name='obstacle_detector',
        output='screen',
    )

    # 6. cmd_vel switcher — selects MPC or MPPI based on /obstacle_nearby
    cmd_vel_switcher = Node(
        package='amiga_local_planner',
        executable='cmd_vel_switcher.py',
        name='cmd_vel_switcher',
        output='screen',
    )

    # MPPI starts at 10s (needs MPC's /aPath to exist first)
    delayed_nav = TimerAction(period=10.0, actions=[
        controller_server,
        lifecycle_manager,
        path_relay,
    ])

    ld = LaunchDescription()
    ld.add_action(zenoh_router)
    ld.add_action(mpc_node)
    ld.add_action(realsense)
    ld.add_action(obstacle_detector)
    ld.add_action(cmd_vel_switcher)
    ld.add_action(delayed_nav)
    return ld
