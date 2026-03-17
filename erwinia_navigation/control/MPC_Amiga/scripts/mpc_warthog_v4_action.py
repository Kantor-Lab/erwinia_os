#!/usr/bin/env python3
import math
import os
import sys
import threading
import time

import acado
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(
    0,
    "/home/appleseed_labs/erwinia_test_2_ws/install/mpc_amiga/lib/python3.10/dist-packages/",
)

import common.cubic_spline_planner as cubic_spline_planner
import common.global_defs as defs
import common.robot_motion_skid_steer as bot_model
import common.utils as utils
from mpc_amiga.action import NavToTree


robot_state = bot_model.kinematics(0, 0, 0, 0)
dt = 0.1
yaw_prev_ = None
vel_up = 0.0
vel_down = defs.TARGET_SPEED
w_up = 0.0


class MPCWarthog(Node):
    def __init__(self):
        super().__init__("mpc_warthog")
        self.declare_parameter("action_name", "go_to_tree")
        self.declare_parameter("stopping_point_frame", "enu")
        self.declare_parameter("global_path_file", "barn_field_waypoints.txt")

        self.local_path_pub = self.create_publisher(MarkerArray, "/pruning_points", 10)
        self.single_marker_pub = self.create_publisher(Marker, "/local_goal_point", 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, "/odometry/global", self.callback_filtered_odom, 10
        )
        self.control_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/aPath", 10)

        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            NavToTree,
            self.get_parameter("action_name").value,
            execute_callback=self.execute_go_to_tree,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel,
            callback_group=self._callback_group,
        )

        self._odom_received = False
        self._goal_lock = threading.Lock()
        self._goal_active = False
        self._stopping_frame = self.get_parameter("stopping_point_frame").value
        self._gps_dir = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "../gps_coordinates")
        )

        self.Q_mpc = np.diag([1.0, 1.0, 1.0, 1.0, 0.01])
        self.get_logger().info(
            f"MPC action server ready on '{self.get_parameter('action_name').value}'"
        )

    def publish_single_marker(self, pose_x, pose_y):
        marker = Marker()
        marker.header.frame_id = self._stopping_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose_x
        marker.pose.position.y = pose_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.single_marker_pub.publish(marker)

    def publish_marker(self, marker_pose_x, marker_pose_y, scale=None, color=None):
        if scale is None:
            scale = [0.5, 0.5, 0.05]
        if color is None:
            color = [1.0, 0.0, 0.0]

        markers_array_msg = MarkerArray()
        markers_array = []

        for count, (x, y) in enumerate(zip(marker_pose_x, marker_pose_y)):
            mark = Marker()
            mark.header.stamp = self.get_clock().now().to_msg()
            mark.header.frame_id = self._stopping_frame
            mark.type = Marker.CYLINDER
            mark.action = Marker.ADD
            mark.ns = "waypoints"
            mark.id = count
            mark.pose.position.x = x
            mark.pose.position.y = y
            mark.pose.position.z = 0.0
            mark.pose.orientation.w = 1.0
            mark.scale.x = scale[0]
            mark.scale.y = scale[1]
            mark.scale.z = scale[2]
            mark.color.a = 1.0
            mark.color.r = color[0]
            mark.color.g = color[1]
            mark.color.b = color[2]
            markers_array.append(mark)

        markers_array_msg.markers = markers_array
        self.local_path_pub.publish(markers_array_msg)

    def iterative_linear_mpc_control(self, xref, dref, oa, ow):
        x0 = [robot_state.x, robot_state.y, robot_state.v, robot_state.yaw, robot_state.w]
        if oa is None or ow is None:
            oa = [0.0] * defs.T
            ow = [0.0] * defs.T

        for _ in range(defs.MAX_ITER):
            xbar = robot_state.predict_motion(oa, ow, defs.T)
            poa, podw = oa[:], ow[:]
            oa, ow, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa - poa)) + sum(abs(ow - podw))
            if du <= defs.DU_TH:
                break
        else:
            self.get_logger().info("Iterative MPC hit max iterations")

        return oa, ow, ox, oy, oyaw, ov

    def linear_mpc_control(self, xref, xbar, x0, dref):
        _x0 = np.zeros((1, defs.NX))
        X = np.zeros((defs.T + 1, defs.NX))
        U = np.zeros((defs.T, defs.NU))
        Y = np.zeros((defs.T, defs.NY))
        yN = np.zeros((1, defs.NYN))

        _x0[0, :] = np.transpose(x0)
        for t_idx in range(defs.T):
            Y[t_idx, :] = np.transpose(xref[:, t_idx])
            X[t_idx, :] = np.transpose(xbar[:, t_idx])
        X[-1, :] = X[-2, :]
        yN[0, :] = Y[-1, :defs.NYN]

        X, U = acado.mpc(
            0,
            1,
            _x0,
            X,
            U,
            Y,
            yN,
            np.transpose(np.tile(defs.Q, defs.T)),
            defs.Qf,
            0,
        )
        ox_mpc = utils.get_nparray_from_matrix(X[:, 0])
        oy_mpc = utils.get_nparray_from_matrix(X[:, 1])
        ov_mpc = utils.get_nparray_from_matrix(X[:, 2])
        oyaw_mpc = utils.get_nparray_from_matrix(X[:, 3])
        oa_mpc = utils.get_nparray_from_matrix(U[:, 0])
        ow_mpc = utils.get_nparray_from_matrix(U[:, 1])

        return oa_mpc, ow_mpc, ox_mpc, oy_mpc, oyaw_mpc, ov_mpc

    def quaternion_to_yaw(self, quaternion):
        x, y, z, w = quaternion
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))

    def callback_filtered_odom(self, odom_msg):
        global yaw_prev_

        x_meas = odom_msg.pose.pose.position.x
        y_meas = odom_msg.pose.pose.position.y
        quat_pose = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
        )
        euler_meas = self.quaternion_to_yaw(quat_pose)

        v_meas = odom_msg.twist.twist.linear.x
        if abs(v_meas) < 1e-4:
            v_meas = 0.0
        w_meas = odom_msg.twist.twist.angular.z

        if yaw_prev_ is None:
            yaw_prev_ = euler_meas
        yaw_in_range = utils.wrapTopm2Pi(euler_meas, yaw_prev_)
        robot_state.set_meas(x_meas, y_meas, yaw_in_range, v_meas, w_meas)
        yaw_prev_ = yaw_in_range
        self._odom_received = True

    def reset_motion_state(self):
        global vel_up, vel_down, w_up
        vel_up = 0.0
        vel_down = defs.TARGET_SPEED
        w_up = 0.0

    def stop_robot(self):
        self.control_pub.publish(Twist())

    def build_path_msg(self, cx, cy):
        path_msg = Path()
        path_msg.header.frame_id = self._stopping_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in zip(cx, cy):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg

    def load_global_path(self, dl=0.1):
        path = os.path.join(self._gps_dir, self.get_parameter("global_path_file").value)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Global path file not found: {path}")

        points = np.atleast_2d(np.loadtxt(path, delimiter=",", dtype=float))
        ax = points[:, 0].tolist()
        ay = points[:, 1].tolist()
        cx, cy, cyaw, ck, _ = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)
        return cx, cy, cyaw, ck

    def load_stopping_points(self):
        path = os.path.join(self._gps_dir, "stopping_points.txt")
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Stopping points file not found: {path}")

        pruning_points = np.atleast_2d(np.loadtxt(path, delimiter=",", dtype=float))
        px = pruning_points[:, 0].tolist()
        py = pruning_points[:, 1].tolist()
        return px, py

    def wait_for_odom(self, timeout_sec=5.0):
        start_time = time.monotonic()
        while rclpy.ok() and not self._odom_received:
            if (time.monotonic() - start_time) >= timeout_sec:
                return False
            time.sleep(0.05)
        return self._odom_received

    def resolve_tree_goal(self, tree_number, ppx, ppy):
        if tree_number < 1 or tree_number > len(ppx):
            raise IndexError(
                f"tree_number={tree_number} is out of range. Valid tree numbers: 1..{len(ppx)}"
            )

        target_index = tree_number - 1
        return target_index, ppx[target_index], ppy[target_index]

    def build_local_plan(self, goal_x, goal_y, global_cx, global_cy, global_cyaw, global_ck, global_sp):
        cx, cy, cyaw, ck, sp = utils.crop_global_plan(
            global_cx, global_cy, global_cyaw, global_ck, global_sp, goal_x, goal_y, 0
        )

        if not cx:
            raise RuntimeError("Failed to build a local plan to the requested stopping point")

        if len(cx) > defs.OFFSET_TO_GOAL:
            goal = [cx[-defs.OFFSET_TO_GOAL], cy[-defs.OFFSET_TO_GOAL]]
            offset_stop = defs.OFFSET_TO_GOAL
        else:
            goal = [cx[-1], cy[-1]]
            offset_stop = 0

        return cx, cy, utils.smooth_yaw(cyaw), ck, sp, goal, offset_stop

    def make_twist_msg(self, accel, acc_omega, goal_data, warn_w=False):
        global vel_up, vel_down, w_up

        cmd = Twist()

        if not goal_data[0]:
            cmd_vel_ = vel_up + dt * defs.TARGET_SPEED / defs.T_RAMP_UP
            vel_up = cmd_vel_

            w_up = acc_omega
            if defs.MIN_TARGET_SPEED < cmd_vel_ < defs.MAX_TARGET_SPEED:
                cmd.linear.x = cmd_vel_
            elif cmd_vel_ > defs.MAX_TARGET_SPEED:
                cmd.linear.x = defs.MAX_TARGET_SPEED
            else:
                cmd.linear.x = defs.MIN_TARGET_SPEED

            cmd.angular.z = 0.0 if warn_w else w_up
        else:
            d_to_goal = max(goal_data[1], 1e-3)
            cmd_vel_ = vel_down - dt * vel_down * vel_down / (5 * d_to_goal)

            if d_to_goal < defs.DIST_TO_GOAL_STOP:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                vel_down = defs.TARGET_SPEED
                w_up = 0.0
            else:
                cmd.linear.x = max(cmd_vel_, 0.0)
                vel_down = cmd.linear.x
                cmd.angular.z = 0.3 * w_up

        return cmd

    def handle_goal(self, goal_request):
        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warning("Rejecting goal because another tree navigation goal is active")
                return GoalResponse.REJECT

            if goal_request.tree_number < 1:
                self.get_logger().warning("Rejecting goal because tree_number must be >= 1")
                return GoalResponse.REJECT

            self._goal_active = True

        self.get_logger().info(f"Accepted goal for tree_number={goal_request.tree_number}")
        return GoalResponse.ACCEPT

    def handle_cancel(self, _goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def finish_goal_state(self):
        with self._goal_lock:
            self._goal_active = False

    def execute_go_to_tree(self, goal_handle):
        result = NavToTree.Result()
        result.success = False
        result.final_distance = float("inf")

        try:
            if not self.wait_for_odom():
                result.message = "Timed out waiting for odometry on /odometry/global"
                goal_handle.abort()
                return result

            self.reset_motion_state()

            dl = 0.1
            init_accel = 0.1
            global_cx, global_cy, global_cyaw, global_ck = self.load_global_path(dl=dl)
            global_cyaw = utils.smooth_yaw(global_cyaw)
            global_sp = utils.calc_speed_profile_2(
                global_cx, global_cy, global_cyaw, defs.TARGET_SPEED
            )
            ppx, ppy = self.load_stopping_points()

            target_index, goal_x, goal_y = self.resolve_tree_goal(goal_handle.request.tree_number, ppx, ppy)
            cx, cy, cyaw, ck, sp, goal, offset_stop = self.build_local_plan(
                goal_x, goal_y, global_cx, global_cy, global_cyaw, global_ck, global_sp
            )

            my_path = self.build_path_msg(global_cx, global_cy)
            self.publish_marker(ppx, ppy)
            self.get_logger().info(
                f"Driving to tree {goal_handle.request.tree_number} at ({goal_x:.3f}, {goal_y:.3f})"
            )

            target_ind, _ = utils.calc_nearest_index(robot_state, cx, cy, cyaw, 0)
            oa, ow = None, None
            feedback = NavToTree.Feedback()
            rate = self.create_rate(10.0)

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.stop_robot()
                    result.message = f"Goal canceled while navigating to tree {goal_handle.request.tree_number}"
                    result.final_distance = feedback.distance_remaining
                    goal_handle.canceled()
                    return result

                self.path_pub.publish(my_path)
                self.publish_marker(ppx, ppy)

                robot_state.get_current_meas_state()
                target_ind, _ = utils.calc_nearest_index(robot_state, cx, cy, cyaw, target_ind)
                xref, target_ind_move, dref = utils.calc_ref_trajectory_v1(
                    robot_state, cx, cy, cyaw, ck, sp, init_accel, dl, dt, target_ind
                )

                self.publish_single_marker(cx[target_ind_move], cy[target_ind_move])

                if robot_state.yaw - cyaw[target_ind_move] >= math.pi:
                    robot_state.yaw -= math.pi * 2.0
                elif robot_state.yaw - cyaw[target_ind_move] <= -math.pi:
                    robot_state.yaw += math.pi * 2.0

                oa, ow, _, _, _, _ = self.iterative_linear_mpc_control(xref, dref, oa, ow)
                wi, ai = ow[0], oa[0]
                if abs(robot_state.v) < 0.05:
                    if sp[target_ind_move] < 0:
                        ai = -0.1
                    else:
                        ai = init_accel
                        wi = 0.01

                init_accel = oa[0]
                goal_data = utils.check_goal(
                    robot_state.get_current_pos_meas(), goal, target_ind_move, len(cx) - offset_stop
                )

                feedback.tree_number = goal_handle.request.tree_number
                feedback.distance_remaining = float(goal_data[1])
                goal_handle.publish_feedback(feedback)

                cmd_command = self.make_twist_msg(ai, wi, goal_data)
                self.control_pub.publish(cmd_command)

                if goal_data[0] and goal_data[1] < defs.DIST_TO_GOAL_STOP:
                    self.stop_robot()
                    result.success = True
                    result.message = (
                        f"Reached stopping point for tree {goal_handle.request.tree_number}"
                    )
                    result.final_distance = float(goal_data[1])
                    goal_handle.succeed()
                    return result

                rate.sleep()

            result.message = "ROS shutdown requested before goal completion"
            goal_handle.abort()
            return result
        except Exception as exc:
            self.stop_robot()
            result.message = str(exc)
            goal_handle.abort()
            self.get_logger().error(f"Failed to execute go_to_tree goal: {exc}")
            return result
        finally:
            self.finish_goal_state()


def main(args=None):
    rclpy.init(args=args)
    mpc_warthog_node = MPCWarthog()
    executor = MultiThreadedExecutor()
    executor.add_node(mpc_warthog_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        mpc_warthog_node.stop_robot()
        mpc_warthog_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
