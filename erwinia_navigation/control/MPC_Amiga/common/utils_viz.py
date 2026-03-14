import rospy
import time
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
import geometry_msgs.msg
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path

	

class markerVisualization:

	def __init__(self, mpc_pure_pusuit=True):
		#MPC VIZ
		if mpc_pure_pusuit:
			self.local_path_pub = rospy.Publisher("/pruning_points", MarkerArray, queue_size=10)
			self.single_marker_pub = rospy.Publisher("/local_goal_point", Marker, queue_size=10)
			self.local_goal_pub = rospy.Publisher("/local_end_point", Marker, queue_size=10)
			self.pathPub = rospy.Publisher("/aPath", Path, queue_size=10)
		else:
		#PURE PURSUIT VIZ
			self.pub_marker_waypts = rospy.Publisher('/visualization_waypoint', MarkerArray, queue_size=1)
			self.pub_lines_waypts = rospy.Publisher('/visualization_waypoint_lines', MarkerArray, queue_size=1)
			self.pub_marker_robot_pose = rospy.Publisher('/visualization_robot_pose', MarkerArray, queue_size=1)
			self.pub_marker_lookahead_circle = rospy.Publisher('/visualization_lookahead', MarkerArray, queue_size=1)
			self.pub_marker_goal = rospy.Publisher('/visualization_goal', MarkerArray, queue_size=1)
			self.pub_marker_pts_curv = rospy.Publisher('/visualization_pts_curv', MarkerArray, queue_size=1)

		# rospy.init_node('waypoint_visualizer_py', anonymous=True)
		# self.rate = rospy.Rate(100) # 10hz

#PURE PURSUIT VIZ METHODS
	def publish_marker_waypts(self, waypoints):

		markers_array_msg = MarkerArray()
		markers_array = []

		for i,waypoint in enumerate(waypoints):
			# pass
			# print(waypoint)
			marker = Marker()
			marker.ns = "waypoints"
			marker.id = i
			marker.header.frame_id = "odom"
			marker.type = marker.CYLINDER
			marker.action = marker.ADD

			marker.scale.x = 0.5
			marker.scale.y = 0.5
			marker.scale.z = 0.05

			marker.color.r = 0.0
			marker.color.g = 1.0
			marker.color.b = 0.0
			marker.color.a = 1.0

			marker.pose.position.x = waypoint[0]
			marker.pose.position.y = waypoint[1]
			marker.pose.position.z = 0

			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0

			marker.lifetime = rospy.Duration(0)
			markers_array.append(marker)

		markers_array_msg.markers = markers_array
		self.pub_marker_waypts.publish(markers_array_msg)
		# self.rate.sleep()


	def publish_lines_waypts(self, waypoints):

		markers_array_msg = MarkerArray()
		markers_array = []

		marker = Marker()
		marker.ns = "waypoints_line"
		marker.id = 1
		marker.header.frame_id = "odom"
		marker.type = marker.LINE_STRIP
		marker.action = marker.ADD

		marker.scale.x = 0.06

		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 0.5

		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0

		for i,waypoint in enumerate(waypoints):
			marker.points.append(Point(waypoint[0], waypoint[1], 0))
		
		marker.lifetime = rospy.Duration(0)
		markers_array.append(marker)

		markers_array_msg.markers = markers_array
		self.pub_lines_waypts.publish(markers_array_msg)
		# self.rate.sleep()

	def publish_marker_robot_pose(self, robot_pose):

		markers_array_msg = MarkerArray()
		markers_array = []

		marker = Marker()
		marker.ns = "robot_pose"
		marker.id = 1
		marker.header.frame_id = "odom"
		marker.type = marker.ARROW
		marker.action = marker.ADD

		marker.scale.x = 1
		marker.scale.y = 0.1
		marker.scale.z = 0.1

		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0
		marker.color.a = 1

		marker.pose = robot_pose
		
		marker.lifetime = rospy.Duration(0)
		markers_array.append(marker)

		markers_array_msg.markers = markers_array
		self.pub_marker_robot_pose.publish(markers_array_msg)
		# self.rate.sleep()

	def publish_marker_lookahead_circle(self, robot_pose, lookahead):

		markers_array_msg = MarkerArray()
		markers_array = []

		marker = Marker()
		marker.ns = "lookahead_circle"
		marker.id = 1
		marker.header.frame_id = "odom"
		marker.type = marker.CYLINDER
		marker.action = marker.ADD

		marker.scale.x = lookahead*2
		marker.scale.y = lookahead*2
		marker.scale.z = 0.05

		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 0.4

		marker.pose = robot_pose
		
		marker.lifetime = rospy.Duration(0)
		markers_array.append(marker)

		markers_array_msg.markers = markers_array
		self.pub_marker_lookahead_circle.publish(markers_array_msg)
		# self.rate.sleep()

	def publish_marker_goal(self, pg):

		markers_array_msg = MarkerArray()
		markers_array = []

		marker = Marker()
		marker.ns = "goal_pt"
		marker.id = 1
		marker.header.frame_id = "odom"
		marker.type = marker.SPHERE
		marker.action = marker.ADD

		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.3

		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0
		marker.color.a = 1.0
		
		marker.pose.position.x = pg[0]
		marker.pose.position.y = pg[1]
		marker.pose.position.z = 0

		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0

		
		marker.lifetime = rospy.Duration(0)
		markers_array.append(marker)

		markers_array_msg.markers = markers_array
		self.pub_marker_goal.publish(markers_array_msg)

	def publish_marker_pts_curv(self, waypoints, waypts_curvature):

		markers_array_msg = MarkerArray()
		markers_array = []

		for i in range(0,waypoints.shape[0]):
			# pass
			# print(waypoint)
			waypoint = waypoints[i]
			marker = Marker()
			marker.ns = "waypoints_curvature"
			marker.id = i
			marker.header.frame_id = "odom"
			marker.type = marker.CYLINDER
			marker.action = marker.ADD

			marker.scale.x = 0.5+waypts_curvature[i]
			marker.scale.y = 0.5+waypts_curvature[i]
			marker.scale.z = 0.05*0.75

			marker.color.r = 1.0
			marker.color.g = 0.549
			marker.color.b = 0.0
			marker.color.a = 1.0

			marker.pose.position.x = waypoint[0]
			marker.pose.position.y = waypoint[1]
			marker.pose.position.z = 0

			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0

			marker.lifetime = rospy.Duration(0)
			markers_array.append(marker)

		markers_array_msg.markers = markers_array
		self.pub_marker_pts_curv.publish(markers_array_msg)
	
#MPC VIZ METHODS
	def pubish_single_marker(self, pose_x, pose_y, flag_goal=False):
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

		if not flag_goal:
			marker.color.r = 0.
			marker.color.g = 1.
			marker.color.b = 1.
			marker.color.a = 1.
			self.single_marker_pub.publish(marker)
		else:
			rgb = [0, 1., 0., 1.]
			marker.color.r = 0.
			marker.color.g = 1.
			marker.color.b = 0.
			marker.color.a = 1.
			self.local_goal_pub.publish(marker)	

	def publish_marker(self, marker_pose_x, marker_pose_y, scale=[0.5,0.5,0.05], color=[1.,0.,0.]):
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
		self.local_path_pub.publish(markers_array_msg)			

	def create_path(self, x, y):
		my_path = Path()
		my_path.header.frame_id = 'odom'
		for x,y in zip(x, y):
			pose = PoseStamped()
			pose.pose.position.x = x
			pose.pose.position.y = y        
			my_path.poses.append(pose)
		return my_path		