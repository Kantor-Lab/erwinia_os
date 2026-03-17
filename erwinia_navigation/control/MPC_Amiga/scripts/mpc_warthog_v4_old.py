#!/usr/bin/env python3
import rclpy
import numpy as np
import acado
import math

import scipy.io as sio

import common.global_defs as defs
import common.utils as utils
import common.robot_motion_skid_steer as bot_model
#import common.utils_viz as visual_tools

import sys
import os
import time

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
import tf

first_seen = False
isInitState = True

#initial robot state
robot_state = bot_model.kinematics(0, 0, 0, 0)
last_time = rospy.Time()
dt = 0.1
yaw_prev_ = 0
vel_up = 0
vel_down = defs.TARGET_SPEED
w_up = 0
count_init = 0
can_delete_file = True
nav_glob_finished = False

local_path_pub = rospy.Publisher("/pruning_points", MarkerArray, queue_size=10)
single_marker_pub = rospy.Publisher("/local_goal_point", Marker, queue_size=10)

Q_mpc = np.diag([1.0, 1.0, 1.0, 1.0, 0.01])

def pubish_single_marker(pose_x, pose_y):
    marker = Marker()
    marker.header.frame_id = 'odom'
    marker.type = marker.SPHERE
    marker.action = marker.ADD

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

    single_marker_pub.publish(marker)

def publish_marker(marker_pose_x, marker_pose_y, scale=[0.5,0.5,0.05], color=[1.,0.,0.]):
    markers_array_msg = MarkerArray()
    markers_array = []
    count=0
    #print(marker_pose_x)
    for x,y in zip(marker_pose_x, marker_pose_y):
        #print(x)
        mark = Marker()
        mark.header.stamp = rospy.Time.now()
        mark.header.frame_id = "odom"
        mark.type = mark.CYLINDER
        mark.action = mark.ADD
        mark.ns = "waypoints"
        mark.id = count
        mark.pose.position.x = x
        mark.pose.position.y = y
        mark.pose.position.z = 0
        mark.pose.orientation.x = 0
        mark.pose.orientation.y = 0
        mark.pose.orientation.z = 0
        mark.pose.orientation.w = 1

        #mark.action = mark.ADD
        
        mark.scale.x = scale[0]
        mark.scale.y = scale[1]
        mark.scale.z = scale[2]
        mark.color.a = 1
        mark.color.r = color[0]
        mark.color.g = color[1]
        mark.color.b = color[2]
        mark.lifetime = rospy.Duration(0)

        markers_array.append(mark)
        count+=1

    markers_array_msg.markers = markers_array
    local_path_pub.publish(markers_array_msg)

# def publish_local_path(path, color=[1.,0.,0.]):
#     my_path = Path()
#     my_path.header.frame_id = 'odom'
#     # my_path.color.r = color[0]
#     # my_path.color.g = color[1]
#     # my_path.color.b = color[2]
#     for column in path.T:
#         pose = PoseStamped()
#         pose.pose.position.x = column[0]
#         pose.pose.position.y = column[1]
#         my_path.poses.append(pose)
#     local_path_pub.publish(my_path)


def iterative_linear_mpc_control(xref, dref, oa, ow):
    """
    MPC contorl with updating operational point iteraitvely
    """
    x0 = [robot_state.x, robot_state.y, robot_state.v, robot_state.yaw, robot_state.w]  
    if oa is None or ow is None:
        oa = [0.0] * defs.T
        ow = [0.0] * defs.T

    for i in range(defs.MAX_ITER):
        xbar = robot_state.predict_motion(oa, ow, defs.T)
        #print(xref.shape)
        poa, podw = oa[:], ow[:]
        oa, ow, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
        du = sum(abs(oa - poa)) + sum(abs(ow - podw))  # calc u change value
        if du <= defs.DU_TH:
            break
    else:
        print("Iterative is max iter")

    #robot_state.refreshState()
    return oa, ow, ox, oy, oyaw, ov

# MPC using ACADO
def linear_mpc_control(xref, xbar, x0, dref):
    # see acado.c for parameter details
    _x0=np.zeros((1, defs.NX))  
    X=np.zeros((defs.T+1, defs.NX))
    U=np.zeros((defs.T, defs.NU))    
    Y=np.zeros((defs.T, defs.NY))    
    yN=np.zeros((1, defs.NYN))    
    _x0[0,:]=np.transpose(x0)  # initial state    
    for t in range(defs.T):
      Y[t,:] = np.transpose(xref[:,t])  # reference state
      X[t,:] = np.transpose(xbar[:,t])  # predicted state
    X[-1,:] = X[-2,:]    
    yN[0,:]=Y[-1,:defs.NYN]         # reference terminal state
    #print(Y.shape)
    X, U = acado.mpc(0, 1, _x0, X,U,Y,yN, np.transpose(np.tile(defs.Q,defs.T)), defs.Qf, 0)    
    ox_mpc = utils.get_nparray_from_matrix(X[:,0])
    oy_mpc = utils.get_nparray_from_matrix(X[:,1])
    ov_mpc = utils.get_nparray_from_matrix(X[:,2])
    oyaw_mpc = utils.get_nparray_from_matrix(X[:,3])
    oa_mpc = utils.get_nparray_from_matrix(U[:,0])
    ow_mpc = utils.get_nparray_from_matrix(U[:,1])
    return oa_mpc, ow_mpc, ox_mpc, oy_mpc, oyaw_mpc, ov_mpc    
    
def callbackFilteredOdom(odom_msg):
    #global last_time
    global yaw_prev_
    current_time = rospy.Time.now()
    #robotPoseEstimate = PoseStamped()
    #robotPoseEstimate.pose.position = odom_msg.pose.pose.position
    # robotPoseEstimate.pose.orientation = odom_msg.pose.pose.orientation
    x_meas = odom_msg.pose.pose.position.x
    y_meas = odom_msg.pose.pose.position.y
    quat_pose = (
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w)
    euler_meas = tf.transformations.euler_from_quaternion(quat_pose) #RPY

    v_meas = odom_msg.twist.twist.linear.x
    if abs(v_meas)<1e-4:
        v_meas = 0
    w_meas = odom_msg.twist.twist.angular.z
    
    #yaw_inRange = euler_meas[2]%(2*math.pi)
    yaw_inRange = utils.wrapTopm2Pi(euler_meas[2], yaw_prev_)
    #if yaw_inRange < 0:
    #    yaw_inRange = 2*math.pi + yaw_inRange
    #elif yaw_inRange > 2*math.pi:
    #    yaw_inRange = 2*math.pi - yaw_inRange
    
    # print("Here")
    # print(yaw_inRange*180/math.pi)
    #print(yaw_prev_*180/math.pi)
    robot_state.set_meas(x_meas, y_meas, yaw_inRange, v_meas, w_meas)
    yaw_prev_ = yaw_inRange

    # if (last_time.to_nsec()==0):
    #     print("Here")
    #     robot_state.update_state(x_meas, y_meas, euler_meas[2], v_meas, dt)
    #     last_time = current_time
    # elif (last_time.to_nsec() > 0):
    #     dt = current_time.to_sec() - last_time.to_sec() 
    #     if (~robot_state.IsFresh and dt>=0.1):
    #         print(dt)
    #         robot_state.update_state(x_meas, y_meas, euler_meas[2], v_meas, dt)
    #         last_time = current_time         

#here I need to create the msg to send - chech in case of a carlike
def make_twist_msg(accel, acc_omega, goalData, warn_w, yaw_meas):
    global vel_up
    global vel_down
    global w_up
    global latest_yaw
    dt_in = 0.1
    cmd = Twist()
    if not goalData[0]:
        cmd_vel_ = vel_up + dt_in*defs.TARGET_SPEED/defs.T_RAMP_UP
        #cmd_vel_ = vel_up + dt_in*accel
        vel_up = cmd_vel_

        cmd_w_ = w_up + dt_in*acc_omega
        w_up = cmd_w_ 

        if cmd_vel_ < defs.MAX_TARGET_SPEED and cmd_vel_ >defs.MIN_TARGET_SPEED: #if cmd_vel_ < defs.TARGET_SPEED:
            cmd.linear.x = cmd_vel_
        elif cmd_vel_ > defs.MAX_TARGET_SPEED:    
            cmd.linear.x = defs.MAX_TARGET_SPEED #cmd.linear.x = defs.TARGET_SPEED
        elif cmd_vel_ < defs.MIN_TARGET_SPEED:
            cmd.linear.x = defs.MIN_TARGET_SPEED
        if not warn_w:
            cmd.angular.z =  w_up# + acc_omega*dt_in
        else:
            w_up = 0
            cmd.angular.z =  0# + acc_omega*dt_in
    else:
        cmd_w_ = w_up + dt_in*acc_omega
        w_up = cmd_w_ 
        dToGoal = goalData[1]
        #cmd_vel_ = vel_down - dt_in*vel_down/defs.T_RAMP_DOWN
        #cmd_vel_ = vel_down - dt_in*defs.TARGET_SPEED/defs.T_RAMP_DOWN
        cmd_vel_ = vel_down - dt_in*vel_down*vel_down/(5*dToGoal)
        print(dToGoal)
        if dToGoal < defs.DIST_TO_GOAL_STOP: #was .1
            cmd.linear.x = 0
            cmd.angular.z = 0
            print("Goal Reached")
            vel_down = defs.MIN_TARGET_SPEED
            #latest_yaw = yaw_meas
            delete_pruning_points_from_file()
            rospy.set_param('nav_stat', True)
            
        else:
            cmd.linear.x = cmd_vel_
            vel_down = cmd_vel_
            cmd.angular.z = 0.3*w_up

        #cmd.angular.z = w_up
    #print(cmd.linear.x)
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    
    return cmd

def delete_pruning_points_from_file():
    global can_delete_file
    global nav_glob_finished
    if can_delete_file:
        try:
            path = "/home/amiga/navigation_ws/src/mpc_controller/gps_coordinates/"
            filename = "pruning_points_real"
            full_path = path + filename + "_copied.txt"
            a_file = open(full_path, "r")
            lines = a_file.readlines()
            a_file.close()
            #print(lines)
            del lines[0]

            new_file = open(full_path, "w+")
            for line in lines:
                new_file.write(line)
            new_file.close()
        except:
            nav_glob_finished = True
    can_delete_file = False

def mpc_node():
    global can_delete_file
    global yaw_prev_
    global latest_yaw
    init_route = 1
    #target_ind = 0
    target_ind_move = 0

    rospy.init_node('mpc_warthog', anonymous=True)

    args = rospy.myargv(argv=sys.argv)
    #print(args)
    if len(args)!=2:
        print("ERROR:Provide fresh_start argument ")
        sys.exit(1)
    is_fresh_start = args[1]
    #print(is_fresh_start)

    odomSubs = rospy.Subscriber("/odometry/filtered", Odometry, callbackFilteredOdom)
    # odomSubs = rospy.Subscriber("/odometry/filtered", Odometry, callbackFilteredOdom)
    controlPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pathPub = rospy.Publisher("/aPath", Path, queue_size=10)

    last_time = rospy.Time.now().to_sec()

    x_ref_all = np.zeros((5, 1), dtype = float) #debugging
    init_accel = 0.1
    #generate the paths
    dl = 0.1
    #global_cx, global_cy, global_cyaw, global_ck = utils.get_vineyard_course(dl)
    global_cx, global_cy, global_cyaw, global_ck = utils.get_course_from_file(True, False, dl)
    # global_cx, global_cy, global_cyaw, global_ck = utils.get_course_from_file_legacy(dl)

    #sio.savemat('ck.mat', {'ck':global_ck})
    #sio.savemat('ck.mat', {'ck':ck})

    #get the pruning points
    ppx, ppy = utils.get_pruning_points(is_fresh_start) #if is a fresh_start use the original_file, otherwise use the cropped one

    # global_sp = utils.calc_speed_profile(global_cx, global_cy, global_cyaw, defs.TARGET_SPEED)
    # global_sp = utils.calc_speed_profile_1(global_cx, global_cy, global_cyaw, global_ck)
    global_sp = utils.calc_speed_profile_2(global_cx, global_cy, global_cyaw, defs.TARGET_SPEED)
    # sio.savemat('/home/fyandun/Documents/catkin_ws/iowa_navigation/src/MPC_Amiga/debug_files/cx_global.mat', {'global_cx':global_cx})
    # sio.savemat('/home/fyandun/Documents/catkin_ws/iowa_navigation/src/MPC_Amiga/debug_files/cy_global.mat', {'global_cy':global_cy})
    # sio.savemat('/home/fyandun/Documents/catkin_ws/iowa_navigation/src/MPC_Amiga/debug_files/cyaw_global.mat', {'global_cyaw':global_cyaw})
    # sio.savemat('/home/fyandun/Documents/catkin_ws/iowa_navigation/src/MPC_Amiga/debug_files/sp_global.mat', {'global_sp':global_sp})
    # print(global_sp)
    rate = rospy.Rate(10) # 10hz
    
    cx, cy, cyaw, ck, sp = None, None, None, None, None

    #this is used to visualize the path on rviz
    my_path = Path()
    my_path.header.frame_id = 'odom'
    for x,y in zip(global_cx, global_cy):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y        
        my_path.poses.append(pose)
    

    # initial yaw compensation
    #if robot_state.yaw - cyaw[0] >= math.pi:
    #    robot_state.yaw -= math.pi * 2.0
    #elif robot_state.yaw - cyaw[0] <= -math.pi:
    #    robot_state.yaw += math.pi * 2.0
    robot_state.get_current_meas_state()
    #print(is_fresh_start)
    if is_fresh_start == "1":
        #print("Here1")
        target_ind = 0
        warn_w = False
        flag_start_stragiht = False
    else:
        #print("Here0")
        target_ind = utils.calc_nearest_index_pruning(robot_state.x, robot_state.y, global_cx, global_cy, 0)
        #target_ind+=defs.OFFSET_TO_GOAL #+20
    #goal = [cx[-1], cy[-1]]
    
    ow, oa = None, None
    global_cyaw = utils.smooth_yaw(global_cyaw)
    #sio.savemat('cyaw_smoothed.mat', {'cyaw_smoothed':cyaw})
    prune_done = 1 #comes from the parameter server, 1 when the robot is good to go
    prune_done_ = 1
    index_pruning = 0
    #latest_yaw = robot_state.yaw
    #delete_pruning_points_from_file()
    while not rospy.is_shutdown():
        prune_done = rospy.get_param("/pruning_status")
        pathPub.publish(my_path)
        publish_marker(ppx, ppy)
        #print(len(global_cx))
        #current_time = rospy.Time.now().to_sec()
        #if current_time - last_time > 10:
        #    publish_marker(ppx, ppy)  
        #    last_time = current_time

        #current_time = rospy.Time.now()
        if not prune_done or init_route:
            can_delete_file = True
            robot_state.get_current_meas_state()
            print("Target_ind", target_ind)
            if index_pruning < len(ppx):
                if not init_route:
                    #target_ind = target_ind_move - defs.OFFSET_TO_GOAL
                    target_ind = utils.calc_nearest_index_pruning(robot_state.x, robot_state.y, global_cx, global_cy, 0)

                cx, cy, cyaw, ck, sp = utils.crop_global_plan(global_cx, global_cy, global_cyaw, global_ck, global_sp, ppx[index_pruning], ppy[index_pruning], target_ind)
                #sio.savemat('/home/fyandun/Documentos/simulation/catkin_ws/src/mpc_controller_warthog/cx_test.mat', {'cx':cx})
                #sio.savemat('/home/fyandun/Documentos/simulation/catkin_ws/src/mpc_controller_warthog/cy_test.mat', {'cy':cy})
                #sio.savemat('/home/fyandun/Documentos/simulation/catkin_ws/src/mpc_controller_warthog/cyaw_test.mat', {'cyaw':cyaw})
                #print(len(cx))
                goal = [cx[-defs.OFFSET_TO_GOAL], cy[-defs.OFFSET_TO_GOAL]]
                #print(target_ind)
                offset_stop = defs.OFFSET_TO_GOAL
            else:
                target_ind_ = utils.calc_nearest_index_pruning(robot_state.x, robot_state.y, global_cx, global_cy, 0)
                cx = global_cx[target_ind_:]
                cy = global_cy[target_ind_:]
                cyaw = global_cyaw[target_ind_:]
                ck = global_ck[target_ind_:]
                sp = global_sp[target_ind_:]
                goal = [cx[-1], cy[-1]]
                #print(goal)
                offset_stop = 0
            #print(goal)

            target_ind, _ = utils.calc_nearest_index(robot_state, cx, cy, cyaw, 0)

            # initial yaw compensation
            if robot_state.yaw - cyaw[target_ind] >= math.pi:
                robot_state.yaw -= math.pi * 2.0
            elif robot_state.yaw - cyaw[target_ind] <= -math.pi:
                robot_state.yaw += math.pi * 2.0                        
            
            prune_done_ = prune_done
            #latest_read = robot_state.yaw
            #init_route = 0

        if prune_done:
            
            diff_prune = prune_done_ - prune_done


            if diff_prune != 0 or init_route:
                current_time_ = rospy.Time.now().to_sec()
                rospy.set_param('nav_stat', False)
                index_pruning+=1
                target_ind_move = target_ind
                if not init_route:
                    flag_start_stragiht = True
                init_route = 0


            prune_done_ = prune_done                                                         

            robot_state.get_current_meas_state()
            #   robot_state.yaw = -1*robot_state.yaw
            #xref, target_ind_move, dref = utils.calc_ref_trajectory(
            #    robot_state, cx, cy, cyaw, ck, sp, dl, dt, target_ind_move)
            xref, target_ind_move, dref = utils.calc_ref_trajectory_v1(
                robot_state, cx, cy, cyaw, ck, sp, init_accel, dl, dt, target_ind_move)
            pubish_single_marker(cx[target_ind_move], cy[target_ind_move])
            #x_ref_all = np.append(x_ref_all, xref,axis = 1)
            #sio.savemat('/home/agvbotics/ros/nav_ws/src/mpc_controller_warthog/x_ref_all.mat', {'x_ref_all':x_ref_all})
            #print(target_ind_move)
            

            if robot_state.yaw - cyaw[target_ind_move] >= math.pi:
                robot_state.yaw -= math.pi * 2.0
            elif robot_state.yaw - cyaw[target_ind_move] <= -math.pi:
                robot_state.yaw += math.pi * 2.0
            #publish_marker(Marker.POINTS, xref)
            #print(xref[3,:])
            oa, ow, ox, oy, oyaw, ov = iterative_linear_mpc_control(
                xref, dref, oa, ow)

            if ow is not None:
                wi, ai = ow[0], oa[0]
                    
            # warm-up solver
            if True: #target_ind < 10:
                if abs(robot_state.v) < 0.05:
                    if sp[target_ind_move]<0:
                        ai = -0.1
                    else:
                        #print(robot_state.v)
                        ai = init_accel
                        wi = 0.01

            init_accel = oa[0]
            #print(goal)
            #apply the control signals
            #dt_cmd = rospy.Time.now().to_sec() - current_time.to_sec()

            goalData = utils.check_goal(robot_state.get_current_pos_meas(), goal, target_ind_move, len(cx)-offset_stop)
            #print(dt_cmd)
            #warn_w = False
            #if is_fresh_start == "0" or diff_prune != 0:
            if is_fresh_start == "0" or flag_start_stragiht:    
                latest_time = rospy.Time.now().to_sec()
                if (abs(latest_time - current_time_) < 2.0):
                    w_i = 0.0 
                    warn_w = True
                    ow = [0.0] * defs.T
                    print("Here")
                    #yaw_prev_ = latest_yaw#((-yaw_prev_ + math.pi) % (2*math.pi) - math.pi)*-1
                    #if robot_state.yaw - cyaw[target_ind] >= math.pi:
                    #    robot_state.yaw -= math.pi * 2.0
                    #elif robot_state.yaw - cyaw[target_ind] <= -math.pi:
                    #    robot_state.yaw += math.pi * 2.0                        

                else:
                    flag_start_stragiht = False
                    warn_w = False
            print('Yaw:', robot_state.yaw)
            print('Goal yaw', cyaw[target_ind_move])
            print(warn_w)
            cmd_command = make_twist_msg(ai, wi, goalData, warn_w, robot_state.yaw)

            if nav_glob_finished:
                print("Global navigation finished - Exiting ...")
                break
            controlPub.publish(cmd_command)
            

        
        rate.sleep()
    
if __name__ == '__main__':
    #args = rospy.myargv(argv=sys.argv)
    #print(args)
    mpc_node()

