import numpy as np


NX = 5  # [x, y, v, yaw]
NY = 5  # reference state variables
NYN = 4   # reference terminal state variables
NU = 2  # [accel, delta]
T = 10  # horizon length ------ was 5

# mpc parameters
R = np.diag([1, 0.01])  # input cost matrix
#Rd = np.diag([0.01, 1.0])  # input differenc cost matrix -> bicycle
Rd = np.diag([0.01, 0.01])  # input difference cost matrix -> skid_steer
Q = np.diag([1.0, 1.0, 1.0, 1.0, 0.1])  # state cost matrix ---  Q = np.diag([1.0, 1.0, 0.5, 1.0, 0.01]), Q = np.diag([1.0, 1.0, 1.0, 1.0, 0.01])
Qf = np.diag([1.0, 1.0, 0.5, 1.0])  # state final matrix  Qf = np.diag([1.0, 1.0, 0.5, 1.0])  
#Qf = np.diag([1.0, 1.0, 0.5, 5.0])
GOAL_DIS = 0.6  # goal distance, was 0.5 abhi
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 1000.0  # max simulation time
OFFSET_TO_GOAL = 15 # to trick the path planner and get a proper orientation at the pruning point, was 20

# iterative paramter
MAX_ITER = 30  # Max iteration
DU_TH = 0.5  # iteration finish param

TARGET_SPEED = 0.3#10.0 / 3.6  # [m/s] target speed
MAX_TARGET_SPEED = 0.7#10.0 / 3.6  # [m/s] target speed, was 0.5
MIN_TARGET_SPEED = 0.15# 0.210.0 / 3.6  # [m/s] target speed, was 0.1
N_IND_SEARCH = 10  # Search index number
T_RAMP_UP = 2. #0.5, was 1
T_RAMP_DOWN = 5. #, was 5.0
DIST_TO_GOAL_STOP = 0.2  # abhi 0.15 this parameter controls the stoping distance to one pruning location, was 0.2

# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 1.32  # [m]

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(45.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]
MAX_JERK = 0.5 # max jerk [m/sss]
