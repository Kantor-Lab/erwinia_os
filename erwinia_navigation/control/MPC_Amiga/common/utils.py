#!/usr/bin/env python3
import numpy as np
import sys, os
from pathlib import Path

current_dir = os.path.dirname(__file__)
sys.path.append(current_dir)	
from . import cubic_spline_planner
import math
from . import global_defs as defs
from . import robot_motion_skid_steer as state
from . import quintic_polynomial_planner as qp
import scipy.io as sio
import csv
from sklearn.neighbors import NearestNeighbors
import shutil

import pdb
import argparse
#import pandas as pd


def parse_args(args):
    args = args[1:]
    parser = argparse.ArgumentParser(description='Changing global to local planning')
    parser.add_argument('fresh_start', type=str, help='Mode of operation')
    parser.add_argument('--barn_field', type=lambda x: (str(x).lower() == 'true'), default=False, help='Enable or disable feature')
    parser.add_argument('--load_backup_plan', type=lambda x: (str(x).lower() == 'true'), default=False, help='Enable or disable feature')
    parsed_args = parser.parse_args(args)
    is_fresh_start = parsed_args.fresh_start
    is_global_nav = parsed_args.barn_field
    load_backup = parsed_args.load_backup_plan
    # print(is_fresh_start)
    # print(args)
    if len(args)!=5:
        print("ERROR:Provide fresh_start, global_local and load_backup arguments ")
        sys.exit(1)    
    return is_fresh_start, is_global_nav, load_backup


def get_course_from_file_legacy(dl=1.0):
    path = os.path.join(current_dir, 'gps_coordinates/') 
    file_name = 'barn_field_waypoints'
    full_path = path + file_name + '.txt'
    points = np.loadtxt(full_path, delimiter=',', dtype=float) #test
    ax = points[:,0].tolist()
    ay = points[:,1].tolist()
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax,ay,ds=dl)
    pdb.set_trace()
    return cx, cy, cyaw, ck

def get_course_from_file(node, is_global_nav, load_backup, dl=1.0):
    cx = []
    cy = []
    cyaw = []
    ck = []
    node.get_logger().info(f"Current Working Directory: {current_dir}")
    # print("Current Working Directory:", current_dir)
    if not is_global_nav and not load_backup:
        return cx, cy, cyaw, ck

    # path = os.path.join(current_dir, 'gps_coordinates/') 
    path = "/home/appleseed_labs/erwinia_os/src/erwinia_navigation/control/MPC_Amiga/gps_coordinates/"
    
    if is_global_nav:
        file_name = 'barn_field_waypoints'
        # dl = 1.0
    else:
        file_name = 'field_waypoints'
        # dl = 0.5
    full_path = path + file_name + '.txt'
    print("full path is:", full_path)
    if os.path.isfile(full_path):
        points = np.loadtxt(full_path, delimiter=',', dtype=float) #test  
        if len(points)>0:      
            ax = points[:,0].tolist()
            ay = points[:,1].tolist()
            ayaw = points[:,3] #will assume it is in format ypr
            cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
                ax,ay,ds=dl)
            # pdb.set_trace()
            return cx, cy, cyaw, ck
    else:
        print("FILE NOT FOUND, RETURNING EMPTY ARRAYS!!!")
        print("Trying to read filename: ", file_name)

    return cx, cy, cyaw, ck

def get_course_from_file_iter(id, dl=1.0):
    cx = []
    cy = []
    cyaw = []
    ck = []


    path = os.path.join(current_dir, '../gps_coordinates/') 
    file_name = 'rows_'
        # dl = 1.0

        # dl = 0.5
    full_path = path + file_name + str(id) + '.txt'
    
    if os.path.isfile(full_path):
        points = np.loadtxt(full_path, delimiter=',', dtype=float) #test  
        if len(points)>0:    
            ax = points[:,0].tolist()
            ay = points[:,1].tolist()
            ayaw = points[:,3].tolist() #will assume it is in format ypr
            if (id+1)%2==0:  
                ax.reverse()
                ay.reverse()
                ayaw.reverse()            
                ax = ax[3:]
                ay = ay[3:]
                ayaw = ayaw[3:]
                
            cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
                ax,ay,ds=dl)
            # pdb.set_trace()
            return cx, cy, cyaw, ck
    else:
        print("FILE NOT FOUND, RETURNING EMPTY ARRAYS!!!")
        print("Trying to read filename: ", file_name)

    return cx, cy, cyaw, ck

def get_vineyard_course(dl=1.0):
    ax = [2.4583730063, 6.7518804365, 10.8136917663, 11.5632410271, 11.9180539295, 11.0372334458, 9.7314,
        8.4255910527, 4.1747550788, 1.33828323, 0.3561238461, 0.2816183203, 1.3911472059, 5.5725694352, 11.3138612183]
    ay = [-0.0001247327, -0.000342074, -0.0005405796, 0.4684504951, 1.4804265236, 2.7482054954, 2.9777, 3.2071415071, 
        2.9856358855, 2.9250751269, 3.4217359359, 4.6814972473, 5.579789281, 5.6680133088, 5.6021789348]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax,ay,ds=dl)

    #sio.savemat('cx.mat', {'cx':cx})
    #sio.savemat('cy.mat', {'cy':cy})
    #sio.savemat('cyaw.mat', {'cyaw':cyaw})
    return cx, cy, cyaw, ck

def get_straight_course(dl=1.0):
    ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course2(dl=1.0):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course3(dl=1.0):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cyaw = [i - math.pi for i in cyaw]

    return cx, cy, cyaw, ck


def get_forward_course(dl=1.0):
    # ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    # ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]
    ax = [0.0, 6.0, 12.50, 5.0, 7.5, 3.0, -1.0]
    ay = [0.0, 0.0, 5.0, 6.50, 3.0, 5.0, -2.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck

def delete_pruning_points_from_file(can_delete_file_):    
    if can_delete_file_:
        try:
            path = os.path.join(current_dir, '../gps_coordinates/') 
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
    return can_delete_file, nav_glob_finished


def get_switch_back_course(dl=1.0):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    return cx, cy, cyaw, ck


def calc_curvature(x, y):
    x = np.array(x)
    y = np.array(y)
    # Assuming equal spacing between points
    t = np.arange(len(x))
    
    # Central differences for first derivatives
    dx = np.gradient(x, t)
    dy = np.gradient(y, t)
    
    # Second derivatives
    ddx = np.gradient(dx, t)
    ddy = np.gradient(dy, t)

    ds = np.sqrt(dx**2 + dy**2)

    # Curvature calculation
    curvatures = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5

    d_theta = curvatures * ds
    orientations = np.cumsum(d_theta)

    initial_orientation = np.arctan2(dy[0], dx[0])
    orientations = initial_orientation + orientations

    return curvatures.tolist(), orientations.tolist()

def get_online_course(ax, ay, dl=0.1):
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    return cx, cy, cyaw, ck

def save_path_backup(ax, ay, ayaw):
    path = os.path.join(current_dir, '../gps_coordinates/') 
    filename = 'field_waypoints'
    full_path = path + filename + '.txt'
    with open(full_path, 'w') as file:
        for x,y, yaw in zip(ax, ay, ayaw):
            file.write(f'{x}, {y}, 0.0, {yaw}, 0.0, 0.0\n') #will assume it is in format ypr

def get_pruning_points(is_fresh_start):

    path = os.path.join(current_dir, '../gps_coordinates/') 
    filename = 'stopping_points'
    orig_src_full_path = path + filename + '.txt'
    copied_src_path = path + filename + '_copied.txt'
    if  is_fresh_start == "true":
        shutil.copy(orig_src_full_path, copied_src_path)
    path_to_load = copied_src_path
    
    pruning_points = np.loadtxt(path_to_load, delimiter=',', dtype=float) #pruning_points
    px = pruning_points[:,0].tolist()
    py = pruning_points[:,1].tolist()

    return px, py

def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile 

def calc_speed_profile_1(cx, cy, cyaw, ck):

    #speed_profile = [defs.MIN_TARGET_SPEED] * len(cx)
    direction = 1.0  # forward
    abs_ck = [abs(value) for value in ck]
    smoothed_k = mavg(abs_ck) 
    speed_profile = [element*(defs.MIN_TARGET_SPEED - defs.MAX_TARGET_SPEED)/max(smoothed_k) + defs.MAX_TARGET_SPEED for element in smoothed_k]

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - speed_profile[i]
        else:
            speed_profile[i] = speed_profile[i]

    speed_profile[-1] = 0.0

    return speed_profile    

def calc_speed_profile_2(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -direction
            else:
                direction = direction

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile 

def calc_speed_profile_3(sp):
    window_size = 15
    return mavg(sp, window_size=window_size)


def mavg(array_in, window_size = 81): #remember to put an odd number here!
    length = len(array_in)
    count = window_size
    buffer = None
    moving_average = []
    for i in range(length):
        if (i < (window_size-1)/2):
            buffer = array_in[0:i+1+math.floor(window_size/2)]
            out_val = np.mean(buffer)
        elif (i > length-(math.floor(window_size-1)/2)):
            buffer = array_in[(i-math.floor(window_size/2)):]
            out_val = np.mean(buffer)
        else:
            buffer = array_in[(i-math.floor(window_size/2)):i+1+math.floor(window_size/2)]
            out_val = np.mean(buffer)
        moving_average.append(out_val)
    return moving_average



def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle       


def calc_nearest_index(state, cx, cy, cyaw, pind):
    dx = [state.x - icx for icx in cx[pind:(pind + defs.N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + defs.N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1
    return ind, mind
# def calc_nearest_index(state, cx, cy, cyaw, pind):
#     # Print search range and robot position
#     print(f"Searching indices: {pind} to {pind + defs.N_IND_SEARCH}")
#     print(f"Robot position: ({state.x:.2f}, {state.y:.2f})")

#     dx = [state.x - icx for icx in cx[pind:(pind + defs.N_IND_SEARCH)]]
#     dy = [state.y - icy for icy in cy[pind:(pind + defs.N_IND_SEARCH)]]
#     d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

#     mind = min(d)
#     ind = d.index(mind) + pind  # Absolute index in the full path

#     # Print intermediate results
#     print(f"Nearest index in window: {d.index(mind)} (local), {ind} (global)")
#     print(f"Distance to nearest point: {math.sqrt(mind):.2f}")

#     dxl = cx[ind] - state.x
#     dyl = cy[ind] - state.y
#     angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))

#     if angle < 0:
#         mind *= -1
#     return ind, mind

def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, dt, pind):
    xref = np.zeros((defs.NY, defs.T + 1))
    dref = np.zeros((1, defs.T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    # if pind >= ind:
    #     ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    #if ind-1>0:
    #    xref[4, 0] = (cyaw[ind] - cyaw[ind-1])/dt
    #    print(cyaw[ind] - cyaw[ind-1])
    #else:
    #    xref[4, 0] = 0
    
    xref[4, 0] = 0.0#(cyaw[ind] - state.yaw)/dt/dt#0.00
    #xref[5, 0] = 0.0
    
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0
    #fut_yaw = state.yaw
    for i in range(defs.T + 1):
        travel += abs(state.v) * dt
        #fut_yaw += state.w*dt
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            xref[4, i] = 0.0#(cyaw[ind + dind] - fut_yaw)/dt/dt#(-state.yaw + cyaw[ind + dind])/dt
            #xref[5, i] = 0.0
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            xref[4, i] = 0.0
            #xref[5, i] = 0.0
            dref[0, i] = 0.0

    return xref, ind, dref    

def calc_ref_trajectory_v1(state, cx, cy, cyaw, ck, sp, sa, dl, dt, pind):
    xref = np.zeros((defs.NY, defs.T + 1))
    dref = np.zeros((1, defs.T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)
    # print("index is ", ind)
    pind = ind
    dist_to_next_gp = np.linalg.norm([cx[ind]-state.x, cy[ind]-state.y])
    if (dist_to_next_gp > 3):
        #print("HERE QP")
        xref = use_quintic_polynomials(state, cx, cy, cyaw, ck, sp, sa, dt, dist_to_next_gp, ind)
        dref = []
    else:

        if pind >= ind:
            ind = pind

        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind]
        xref[2, 0] = sp[ind]
        xref[3, 0] = cyaw[ind]
        xref[4, 0] = 0.0
        dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0

        for i in range(defs.T + 1):
            travel += abs(state.v) * dt
            dind = int(round(travel / dl))

            if (ind + dind) < ncourse:
                xref[0, i] = cx[ind + dind]
                xref[1, i] = cy[ind + dind]
                xref[2, i] = sp[ind + dind]
                xref[3, i] = cyaw[ind + dind]
                xref[4, i] = 0.0
                dref[0, i] = 0.0
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2, i] = sp[ncourse - 1]
                xref[3, i] = cyaw[ncourse - 1]
                xref[4, i] = 0.0
                dref[0, i] = 0.0
    # print("index1 is ", pind)
    return xref, ind, dref

def use_quintic_polynomials(state, cx, cy, cyaw, ck, sp, sa, dt, dist_to_gp, ind):
    ga = 0.1 #goal acceleration
    sx = state.x
    vxs = state.v*math.cos(state.yaw)
    axs = sa*math.cos(state.yaw) #start accel in x coords
    vxg = sp[ind]*math.cos(cyaw[ind])
    axg = ga*math.cos(cyaw[ind])
    
    sy = state.y
    vys = state.v*math.sin(state.yaw)
    ays = sa*math.sin(state.yaw) #start accel in y coords
    vyg = sp[ind]*math.sin(cyaw[ind])
    ayg = ga*math.sin(cyaw[ind])

    vel_max = 0.4 #5
    T = dist_to_gp/vel_max

    xqp = qp.QuinticPolynomial(sx, vxs, axs, cx[ind], vxg, axg, T)
    yqp = qp.QuinticPolynomial(sy, vys, ays, cy[ind], vyg, ayg, T)


    ra, rj = [], []
    xref = np.zeros((defs.NY, defs.T + 1))
    t = 0
    for i in range(defs.T + 1):
        
        xref[0, i] = xqp.calc_point(t)
        xref[1, i] = yqp.calc_point(t)

        vx = xqp.calc_first_derivative(t)
        vy = yqp.calc_first_derivative(t)
        xref[2, i] = math.sqrt(vx*vx + vy*vy)

        xref[3,i] = math.atan2(vy, vx)

        ax = xqp.calc_second_derivative(t)
        ay = yqp.calc_second_derivative(t)
        a = math.sqrt(ax*ax + ay*ay)
        if xref[3,i] - xref[3,i-1] < 0:
            a = -1*a
        ra.append(a)

        jx = xqp.calc_third_derivative(t)
        jy = yqp.calc_third_derivative(t)
        j = math.sqrt(jx*jx + jy*jy)
        if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
            j *= -1

        rj.append(j)

        t+=dt

    xref[3,:] = smooth_yaw(xref[3,:])
    xref[4, i] = 0.0
    if max([abs(i) for i in ra]) >= defs.MAX_ACCEL and max([abs(i) for i in rj]) >= defs.MAX_JERK:
        print("Warn: Jerk threshold exceeded")

    return xref

def calc_nearest_index_pruning(px, py, cx, cy, pind):
    
    global_path = np.vstack((cx[pind:], cy[pind:]))
    query_point = np.vstack((px, py))
    #print(np.shape(query_point))
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(global_path.T)
    _, indices = nbrs.kneighbors(query_point.T)
    #print(indices[0,0])
    return indices[0,0] 


def crop_global_plan(global_cx, global_cy, global_cyaw, global_ck, global_sp, pruning_point_x, pruning_point_y, pind):
    index_to_crop = calc_nearest_index_pruning(pruning_point_x, pruning_point_y, global_cx, global_cy, pind)
    local_cx = global_cx[pind:pind + index_to_crop + defs.OFFSET_TO_GOAL]
    local_cy = global_cy[pind:pind + index_to_crop + defs.OFFSET_TO_GOAL]
    local_cyaw = global_cyaw[pind:pind + index_to_crop + defs.OFFSET_TO_GOAL]
    local_ck = global_ck[pind:pind + index_to_crop + defs.OFFSET_TO_GOAL]
    local_sp = global_sp[pind:pind + index_to_crop + defs.OFFSET_TO_GOAL]
    return local_cx, local_cy, local_cyaw, local_ck, local_sp


def check_goal(current_pos, goal, tind, nind):

    # check goal
    dx = current_pos[0] - goal[0]
    dy = current_pos[1] - goal[1]
    d = math.sqrt(dx ** 2 + dy ** 2)
    print("Distance to goal", d)
    isgoal = (d <= defs.GOAL_DIS)

    if abs(tind - nind) >= 50: #50
        isgoal = False

    # isstop = (abs(state.v) <= defs.STOP_SPEED)
    # if isgoal and isstop:
    #     return True
    if isgoal:
        #return True
        return [1, d]

    #return False
    return[0, d]    

def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw

def wrapTopm2Pi(yaw, yaw_prev):
    dyaw = yaw - yaw_prev
    if (dyaw >= math.pi / 2.0):
        yaw -= math.pi * 2.0
    elif(dyaw <= -math.pi / 2.0):
        yaw += math.pi * 2.0
    return yaw

def get_nparray_from_matrix(x):
    return np.array(x).flatten()
