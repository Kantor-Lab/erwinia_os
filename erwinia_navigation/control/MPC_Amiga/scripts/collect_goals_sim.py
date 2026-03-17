#!/usr/bin/env python 

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
# from sensor_msgs.msg import Imu
# from move_base_msgs.msg import MoveBaseActionGoal
from rosgraph_msgs.msg import Clock
import numpy as np
import time
from actionlib_msgs.msg import GoalStatusArray
import os
from tf.transformations import euler_from_quaternion

current_dir = os.path.dirname(__file__)


next = False
class Goal_reacher:
	def __init__(self):
		self.robot_position_odom = []
		self.robot_position_gps = []
		self.curr_time = Clock()
		self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.callback)
		# self.sub1 = rospy.Subscriber('rover/piksi/position_receiver_0/ros/transform_enu', TransformStamped, self.callback1)
		# self.sub2 = rospy.Subscriber('/clock', Clock, self.callback2 )
		# self.sub3 = rospy.Subscriber('/imu/data', Imu, self.callback_yaw)
		#self.sub3 = rospy.Subscriber('move_base/status', GoalStatusArray, self.callback3)
	
		self.stamp_old = 0

	def callback3(self, data):
		
		m = data.status_list[-1]
		stamp = data.header.stamp.secs
		if m.text == "Goal reached." and stamp-self.stamp_old>1.0:
			print('STATUS : ', m.status)
			self.pub_next()
			print(data.header)
			self.stamp_old = stamp
			
	def callback(self, data):
		# self.robot_position_odom.append([data.transform.translation.x, data.transform.translation.y, 0., 0., 0., 0.])
		self.robot_position_odom.append([data.pose.pose.position.x, data.pose.pose.position.y, 0., 0., 0., 0.])
	
	def callback1(self, data):
		robot_orientation = data.transform.rotation
		r_quaternion_list = [robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w]
		_, _, yaw = euler_from_quaternion(r_quaternion_list)
		self.robot_position_gps.append([data.transform.translation.x, data.transform.translation.y, yaw, 0., 0., 0.])
	
	def callback2(self, data):
		self.curr_time = data

if __name__ == "__main__":
	
	# f1 = 'data/2023-05-26/2023-05-26_flagstaff_oval_ekf.txt'
	# '/home/amiga/navigation_ws/src/mpc_controller/gps_coordinates/rows_real.txt'
	f2 = os.path.join(current_dir, '../gps_coordinates/rows_1.txt') 

	mode = 'subsample'
	subsample_rate = 20

	gr = Goal_reacher()
	rospy.init_node('goal_reacher', anonymous=True)
	if mode == 'capture':
		while True:
			x = input('Save_point?')	
			if x == 'y':
				c1 = np.loadtxt(f1,delimiter=',')
				if(len(c1) == 0):
					np.savetxt(f1,[gr.robot_position_odom[-1]],delimiter=',')
				else:
					np.savetxt(f1,np.vstack([c1,gr.robot_position_odom[-1]]),delimiter=',')

				print(c1)
	else:
		while True:
			x = input('Done?: ')
			if x == 'y':
				break

		ekf = np.stack(gr.robot_position_odom)[::subsample_rate]
		# gps = np.stack(gr.robot_position_gps)[::subsample_rate]
		np.savetxt(f2, ekf, delimiter=',')
		# np.savetxt(f2, gps, delimiter=',')
		print('Saved subsampled trajectories')
	
	rospy.spin()
