"""
MPC + MPPI combo launch for the real robot.

Architecture:
  MPC  → cubic spline /aPath (high quality, speed-profiled)
  MPPI → follows /aPath + obstacle avoidance → /cmd_vel (sole controller)

Topic flow:
  MPC reads barn_field_waypoints.txt → publishes /aPath (cubic spline)
  MPC's own /cmd_vel → /mpc_cmd_vel  (routed through switcher)
  path_relay sends /aPath → controller_server via FollowPath action
  controller_server (MPPI) → /local_planner_cmd_vel → switcher → /cmd_vel

Usage:
  ros2 launch amiga_local_planner mpc_mppi_combo.launch.py planner:=combo dry_run:=true
  ros2 launch amiga_local_planner mpc_mppi_combo.launch.py planner:=mpc
  ros2 launch amiga_local_planner mpc_mppi_combo.launch.py planner:=mppi dry_run:=false

Arguments:
  planner   mpc | mppi | combo  (default: combo)
              mpc   — only MPC runs; no MPPI stack launched
              mppi  — MPC generates path, MPPI always controls
              combo — obstacle-based switching between MPC and MPPI
  dry_run   true | false        (default: true)
              true  — publish zero velocity (visualization only)
"""
import os
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_lp  = get_package_share_directory('amiga_local_planner')
    costmap_config = os.path.join(pkg_lp, 'config', 'costmap.yaml')
    mppi_config    = os.path.join(pkg_lp, 'config', 'mppi.yaml')

    # ── Launch arguments ────────────────────────────────────────────────────────
    planner_arg = DeclareLaunchArgument(
        'planner',
        default_value='combo',
        choices=['mpc', 'mppi', 'combo'],
        description='Which planner(s) to run: mpc | mppi | combo',
    )
    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='true',
        description='Publish zero velocity when true (visualization only)',
    )

    planner = LaunchConfiguration('planner')
    dry_run = LaunchConfiguration('dry_run')

    # MPPI stack is needed for 'mppi' and 'combo'
    use_mppi     = IfCondition(PythonExpression(["'", planner, "' in ('mppi', 'combo')"]))
    # Obstacle detector is only needed for 'combo' (switching logic)
    use_detector = IfCondition(PythonExpression(["'", planner, "' == 'combo'"]))

    # ── Nodes ───────────────────────────────────────────────────────────────────

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
        condition=use_mppi,
    )

    # 3. Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mppi',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['controller_server']}],
        condition=use_mppi,
    )

    # 4. path_relay — sends MPC's /aPath to MPPI via FollowPath action
    path_relay = Node(
        package='amiga_local_planner',
        executable='path_relay.py',
        name='path_relay',
        output='screen',
        condition=use_mppi,
    )

    # 5. RealSense camera
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        output='screen',
        parameters=[{'enable_color': True, 'enable_depth': False}],
    )

    # 6. Obstacle detector — only needed for combo switching
    obstacle_detector = Node(
        package='amiga_local_planner',
        executable='obstacle_detector.py',
        name='obstacle_detector',
        output='screen',
        condition=use_detector,
    )

    # 7. cmd_vel switcher — mode and dry_run passed from launch args
    cmd_vel_switcher = Node(
        package='amiga_local_planner',
        executable='cmd_vel_switcher.py',
        name='cmd_vel_switcher',
        output='screen',
        parameters=[{
            'mode':    planner,
            'dry_run': dry_run,
        }],
    )

    # 8. RViz2 — same config as standalone MPC launch
    rviz_config = os.path.join(
        get_package_share_directory('mpc_amiga'), 'rviz', 'view_path.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # MPPI starts at 10s (needs MPC's /aPath to exist first)
    delayed_nav = TimerAction(period=10.0, actions=[
        controller_server,
        lifecycle_manager,
        path_relay,
    ])

    ld = LaunchDescription()
    ld.add_action(planner_arg)
    ld.add_action(dry_run_arg)
    ld.add_action(mpc_node)
    # ld.add_action(realsense)  # disabled: camera not used for now
    ld.add_action(obstacle_detector)
    ld.add_action(cmd_vel_switcher)
    ld.add_action(rviz)
    ld.add_action(delayed_nav)
    return ld
