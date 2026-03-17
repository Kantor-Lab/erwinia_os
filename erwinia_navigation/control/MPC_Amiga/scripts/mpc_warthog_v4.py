#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
import sys
sys.path.insert(0, "/home/appleseed_labs/erwinia_test_2_ws/install/mpc_amiga/lib/python3.10/dist-packages/")
# from tf_transformations import euler_from_quaternion
import numpy as np
import acado
import math
import scipy.io as sio
import common.global_defs as defs
import common.utils as utils
import common.robot_motion_skid_steer as bot_model
import os
import time
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
import threading
first_seen = False
isInitState = True

# Initial robot state
robot_state = bot_model.kinematics(0, 0, 0, 0)
last_time = None
dt = 0.1
yaw_prev_ = None
vel_up = 0
vel_down = defs.TARGET_SPEED
w_up = 0
count_init = 0
can_delete_file = True
nav_glob_finished = False

class MPCWarthog(Node):
    def __init__(self):
        super().__init__('mpc_warthog')
        self.declare_parameter('fresh_start', True)
        self.declare_parameter('nav_stat', True)
        self.declare_parameter('pruning_status', False)

        self.local_path_pub = self.create_publisher(MarkerArray, '/pruning_points', 10)
        self.single_marker_pub = self.create_publisher(Marker, '/local_goal_point', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry/global', self.callback_filtered_odom, 10) #/rtk_enu, /odometry/filtered
        self.control_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/aPath', 10)

        self.last_time = self.get_clock().now()
        self.Q_mpc = np.diag([1.0, 1.0, 1.0, 1.0, 0.01])

    def publish_single_marker(self, pose_x, pose_y):
        marker = Marker()
        marker.header.frame_id = 'enu'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = pose_x
        marker.pose.position.y = pose_y
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.r = 0.
        marker.color.g = 1.
        marker.color.b = 1.
        marker.color.a = 1.

        self.single_marker_pub.publish(marker)

    def publish_marker(self, marker_pose_x, marker_pose_y, scale=[0.5, 0.5, 0.05], color=[1., 0., 0.]):
        markers_array_msg = MarkerArray()
        markers_array = []
        count = 0

        for x, y in zip(marker_pose_x, marker_pose_y):
            mark = Marker()
            mark.header.stamp = self.get_clock().now().to_msg()
            mark.header.frame_id = 'enu'
            mark.type = Marker.CYLINDER
            mark.action = Marker.ADD
            mark.ns = "waypoints"
            mark.id = count
            mark.pose.position.x = x
            mark.pose.position.y = y
            mark.pose.position.z = 0.0
            mark.pose.orientation.x = 0.0
            mark.pose.orientation.y = 0.0
            mark.pose.orientation.z = 0.0
            mark.pose.orientation.w = 1.0

            mark.scale.x = scale[0]
            mark.scale.y = scale[1]
            mark.scale.z = scale[2]
            mark.color.a = 1.0
            mark.color.r = color[0]
            mark.color.g = color[1]
            mark.color.b = color[2]

            markers_array.append(mark)
            count += 1

        markers_array_msg.markers = markers_array
        self.local_path_pub.publish(markers_array_msg)

    def iterative_linear_mpc_control(self, xref, dref, oa, ow):
        
        x0 = [robot_state.x, robot_state.y, robot_state.v, robot_state.yaw, robot_state.w]
        if oa is None or ow is None:
            oa = [0.0] * defs.T
            ow = [0.0] * defs.T

        for i in range(defs.MAX_ITER):
            xbar = robot_state.predict_motion(oa, ow, defs.T)
            poa, podw = oa[:], ow[:]
            oa, ow, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa - poa)) + sum(abs(ow - podw))
            if du <= defs.DU_TH:
                break
        else:
            self.get_logger().info("Iterative is max iter")
        # self.get_logger().info(f"HERE MPC: {oa}")
        return oa, ow, ox, oy, oyaw, ov

    def linear_mpc_control(self, xref, xbar, x0, dref):
        _x0 = np.zeros((1, defs.NX))
        X = np.zeros((defs.T + 1, defs.NX))
        U = np.zeros((defs.T, defs.NU))
        Y = np.zeros((defs.T, defs.NY))
        yN = np.zeros((1, defs.NYN))

        _x0[0, :] = np.transpose(x0)
        for t in range(defs.T):
            Y[t, :] = np.transpose(xref[:, t])
            X[t, :] = np.transpose(xbar[:, t])
        X[-1, :] = X[-2, :]
        yN[0, :] = Y[-1, :defs.NYN]

        X, U = acado.mpc(0, 1, _x0, X, U, Y, yN, np.transpose(np.tile(defs.Q, defs.T)), defs.Qf, 0)
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
        # self.get_logger().info("in Odom Callback")
        global yaw_prev_
        current_time = self.get_clock().now()
        x_meas = odom_msg.pose.pose.position.x
        y_meas = odom_msg.pose.pose.position.y
        quat_pose = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w)
        euler_meas = self.quaternion_to_yaw(quat_pose)

        v_meas = odom_msg.twist.twist.linear.x
        if abs(v_meas) < 1e-4:
            v_meas = 0
        w_meas = odom_msg.twist.twist.angular.z

        if yaw_prev_ is None:
            yaw_prev_ = euler_meas
        yaw_in_range = utils.wrapTopm2Pi(euler_meas, yaw_prev_)
        robot_state.set_meas(x_meas, y_meas, yaw_in_range, v_meas, w_meas)
        yaw_prev_ = yaw_in_range

    def make_twist_msg(self, accel, acc_omega, goal_data, warn_w, yaw_meas):
        global vel_up, vel_down, w_up
        dt_in = 0.1
        cmd = Twist()

        if not goal_data[0]:
            cmd_vel_ = vel_up + dt_in * defs.TARGET_SPEED / defs.T_RAMP_UP
            vel_up = cmd_vel_

            cmd_w_ = w_up + dt_in * acc_omega
            w_up = cmd_w_

            #directly set w_up = acc_omega
            w_up = acc_omega
            if defs.MIN_TARGET_SPEED < cmd_vel_ < defs.MAX_TARGET_SPEED:
                cmd.linear.x = cmd_vel_
            elif cmd_vel_ > defs.MAX_TARGET_SPEED:
                cmd.linear.x = defs.MAX_TARGET_SPEED
            elif cmd_vel_ < defs.MIN_TARGET_SPEED:
                cmd.linear.x = defs.MIN_TARGET_SPEED

            # cmd.angular.z = 0 if warn_w else w_up
            if not warn_w:
                cmd.angular.z =  w_up# + acc_omega*dt_in
            else:
                w_up = 0
                cmd.angular.z =  0.0
        else:
            cmd_w_ = w_up + dt_in * acc_omega
            w_up = cmd_w_
            d_to_goal = goal_data[1]
            cmd_vel_ = vel_down - dt_in * vel_down * vel_down / (5 * d_to_goal)

            if d_to_goal < defs.DIST_TO_GOAL_STOP:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                vel_down = defs.MIN_TARGET_SPEED
                self.delete_pruning_points_from_file()
                rclpy.parameter.Parameter(
                    'my_parameter',
                    rclpy.Parameter.Type.STRING,
                    'world'
                )
                self.set_parameters([Parameter('nav_stat', Parameter.Type.BOOL, True)])
            else:
                cmd.linear.x = cmd_vel_
                vel_down = cmd_vel_
                cmd.angular.z = 0.3 * w_up

        cmd.linear.y = cmd.linear.z = 0.0
        cmd.angular.x = cmd.angular.y = 0.0

        return cmd

    def delete_pruning_points_from_file(self):
        global can_delete_file, nav_glob_finished
        if can_delete_file:
            try:
                path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../gps_coordinates") + "/"
                filename = "pruning_points_real"
                full_path = path + filename + "_copied.txt"
                with open(full_path, "r") as a_file:
                    lines = a_file.readlines()

                with open(full_path, "w") as new_file:
                    for line in lines[1:]:
                        new_file.write(line)
            except:
                nav_glob_finished = True

        can_delete_file = False

    def mpc_node(self, is_fresh_start):
        global can_delete_file, yaw_prev_
        init_route = 1
        self.get_logger().info(f"is_fresh_start:{is_fresh_start}")
        # Getting pruning and global paths
        dl = 0.1
        init_accel = 0.1
        self.get_logger().info("Get gloabl path")
        global_cx, global_cy, global_cyaw, global_ck = utils.get_course_from_file(self, True, False, dl)
        self.get_logger().info(f"x_value: {global_cx}")
        global_sp = utils.calc_speed_profile_2(global_cx, global_cy, global_cyaw, defs.TARGET_SPEED)
        ppx, ppy = utils.get_pruning_points(is_fresh_start)

        self.get_logger().info("Starting MPC control node")
        rate = self.create_rate(10) # 10Hz

        # Publish the global path to RViz
        my_path = Path()
        my_path.header.frame_id = 'enu'
        my_path.header.stamp = self.get_clock().now().to_msg()
        for x, y in zip(global_cx, global_cy):
            pose = PoseStamped()
            pose.header.frame_id = 'enu'
            pose.header.stamp = my_path.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            my_path.poses.append(pose)

        cx, cy, cyaw, ck, sp = None, None, None, None, None
        robot_state.get_current_meas_state()
        if is_fresh_start:
            target_ind = 0
            warn_w = False
            flag_start_stragiht = False
        else:
            target_ind = utils.calc_nearest_index_pruning(robot_state.x, robot_state.y, global_cx, global_cy, 0)

        ow, oa = None, None
        prune_done = 1
        prune_done_ = 1
        index_pruning = 0
        global_cyaw = utils.smooth_yaw(global_cyaw)
         

        while rclpy.ok():
            # rclpy.spin(self)
            # self.get_logger().info(f'In prune done:{prune_done}')
            # prune_done = self.get_parameter('/pruning_status').get_parameter_value().bool_value
            prune_done = self.get_parameter_or('pruning_status', Parameter('pruning_status', Parameter.Type.BOOL, False)).value
            # rclpy.spin(self)

            self.path_pub.publish(my_path)
            self.publish_marker(ppx, ppy)

            if not prune_done or init_route:
                self.get_logger().info('In not prune done')
                can_delete_file = True
                robot_state.get_current_meas_state()
                self.get_logger().info(f"target ind: {target_ind}")

                if index_pruning < len(ppx):
                    if not init_route:
                        target_ind = utils.calc_nearest_index_pruning(robot_state.x, robot_state.y, global_cx, global_cy, 0)
                    
                    cx, cy, cyaw, ck, sp = utils.crop_global_plan(
                        global_cx, global_cy, global_cyaw, global_ck, global_sp,
                        ppx[index_pruning], ppy[index_pruning], 0
                    )
                    goal = [cx[-defs.OFFSET_TO_GOAL], cy[-defs.OFFSET_TO_GOAL]]
                    offset_stop = defs.OFFSET_TO_GOAL
                else:
                    target_ind_ = utils.calc_nearest_index_pruning(robot_state.x, robot_state.y, global_cx, global_cy, 0)

                    cx, cy, cyaw, ck, sp = global_cx[target_ind_:], global_cy[target_ind_:], global_cyaw[target_ind_:], global_ck[target_ind_:], global_sp[target_ind_:]
                    goal = [cx[-1], cy[-1]]
                    # offset_stop = defs.OFFSET_TO_GOAL
                    offset_stop = 0

                target_ind, _ = utils.calc_nearest_index(robot_state, cx, cy, cyaw, 0)
                # initial yaw compensation
                if robot_state.yaw - cyaw[target_ind] >= math.pi:
                    robot_state.yaw -= math.pi * 2.0
                elif robot_state.yaw - cyaw[target_ind] <= -math.pi:
                    robot_state.yaw += math.pi * 2.0
                prune_done_ = prune_done
                # init_route = 0
            # self.get_logger().info(f'prune done:{prune_done}')
            if prune_done:
                # self.get_logger().info('In prune done')
                
                diff_prune = prune_done_ - prune_done
                if diff_prune != 0 or init_route:
                    current_time_ = self.get_clock().now()
                    self.set_parameters([Parameter('nav_stat', Parameter.Type.BOOL, False)])
                    index_pruning += 1
                    target_ind = utils.calc_nearest_index_pruning(robot_state.x, robot_state.y, global_cx, global_cy, 0)
                    target_ind_move = target_ind
                    if not init_route:
                        flag_start_stragiht = True
                    init_route = 0
                
                prune_done_ = prune_done
                robot_state.get_current_meas_state()
                # self.get_logger().info(f"Cropped path length: {len(cx)}")
                target_ind, _ = utils.calc_nearest_index(robot_state, cx, cy, cyaw, target_ind_move) #recalculate the index
                # self.get_logger().info(f"Current target index: {target_ind}")
                xref, target_ind_move, dref = utils.calc_ref_trajectory_v1(
                    robot_state, cx, cy, cyaw, ck, sp, init_accel, dl, dt, target_ind
                )
                # After cropping the path:
                
                self.get_logger().info(f"Current target index: {target_ind_move}")
                
                self.publish_single_marker(cx[target_ind_move], cy[target_ind_move])
                
                if robot_state.yaw - cyaw[target_ind_move] >= math.pi:
                    robot_state.yaw -= math.pi * 2.0
                elif robot_state.yaw - cyaw[target_ind_move] <= -math.pi:
                    robot_state.yaw += math.pi * 2.0

                oa, ow, ox, oy, oyaw, ov = self.iterative_linear_mpc_control(
                    xref, dref, oa, ow)

                if ow is not None:
                    wi, ai = ow[0], oa[0]
                # self.get_logger().info(f"MPC result: {ai}")
                if True: #target_ind < 10:
                    if abs(robot_state.v) < 0.05:
                        if sp[target_ind_move]<0:
                            ai = -0.1
                        else:
                            #print(robot_state.v)
                            ai = init_accel
                            wi = 0.01

                init_accel = oa[0]

                # goal_data = utils.check_goal(robot_state.get_current_pos_meas(), goal, target_ind, len(cx) - offset_stop)
                goal_data = utils.check_goal(robot_state.get_current_pos_meas(), goal, target_ind_move, len(cx) - offset_stop)
                
                if not is_fresh_start or flag_start_stragiht:    
                    latest_time = self.get_clock().now()
                    if (abs(latest_time - current_time_) < 2.0):
                        w_i = 0.0 
                        warn_w = True
                        ow = [0.0] * defs.T
                        # print("Here")
                    else:
                        flag_start_stragiht = False
                        warn_w = False
                self.get_logger().info(f"Yaw: {robot_state.yaw}")
                self.get_logger().info(f"Goal yaw: {cyaw[target_ind_move]}")
                # self.get_logger().info(f"warn: {warn_w}")
                # print(warn_w)
                cmd_command = self.make_twist_msg(ai, wi, goal_data, warn_w, robot_state.yaw)
                # self.get_logger().info(f"vel: {cmd_command.linear.x}")
                if nav_glob_finished:
                    self.get_logger().info("Global navigation finished. - Exiting ...")
                    break
                # self.get_logger().info("Going to publish velocity")
                self.control_pub.publish(cmd_command)
                # rclpy.spin(self)
            rate.sleep()
        # rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    mpc_warthog_node = MPCWarthog()
    try:
        # Retrieve the 'fresh_start' parameter
        is_fresh_start = mpc_warthog_node.get_parameter('fresh_start').value
        # mpc_warthog_node.mpc_node(str(is_fresh_start).lower())  # Convert to string and pass
        # # rclpy.spin(mpc_warthog_node)
        # executor = MultiThreadedExecutor()
        # executor.add_node(mpc_warthog_node)
        # executor.spin()
        mpc_thread = threading.Thread(target=mpc_warthog_node.mpc_node, args=(str(is_fresh_start).lower(),))
        mpc_thread.daemon = True  # Set the thread as a daemon so it exits when the main program exits
        mpc_thread.start()

        # Use a MultiThreadedExecutor to spin the node
        executor = MultiThreadedExecutor()
        executor.add_node(mpc_warthog_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        mpc_warthog_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
       
