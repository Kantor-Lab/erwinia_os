#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Quaternion
from tf_transformations import quaternion_from_euler
from rosgraph_msgs.msg import Clock
from swiftnav_ros2_driver.msg import Baseline
import numpy as np
import os

class enu_odom_publisher(Node):
    def __init__(self):
        super().__init__('rtk_enu_publisher')

        self.subscription = self.create_subscription(
            Baseline,
            '/ns/baseline',
            self.rtk_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'rtk_enu', 10)

    def rtk_callback(self, msg):
        pos_x_ned = msg.baseline_n_m
        pos_y_ned = msg.baseline_e_m
        pos_z_ned = msg.baseline_d_m
        yaw_ned_deg = msg.baseline_dir_deg

        pos_x_enu = pos_y_ned
        pos_y_enu = pos_x_ned
        pos_z_enu = -pos_z_ned
        
        yaw_enu = (np.pi/180)*(90-yaw_ned_deg)
        roll_enu = 0.0
        pitch_enu = 0.0
        # self.get_logger().info(f"enu yaw: {yaw_enu*180/np.pi}")

        q = quaternion_from_euler(roll_enu, pitch_enu, yaw_enu)
        quat = Quaternion()
        quat.x = q[0]
        quat.y = q[1]
        quat.z = q[2]
        quat.w = q[3]
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'enu'

        pose_msg.pose.pose.position.x = pos_x_enu
        pose_msg.pose.pose.position.y = pos_y_enu
        pose_msg.pose.pose.position.z = pos_z_enu
        pose_msg.pose.pose.orientation = quat

        # Fill covariance with example values or zeros
        pose_msg.pose.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.1,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.1,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.1
        ]

        self.pose_publisher.publish(pose_msg)        
        # odom_msg = Odometry()
        # odom_msg.header.stamp = self.get_clock().now().to_msg()
        # odom_msg.header.frame_id = 'swiftnav-gnss-enu'
        # # odom_msg.child_frame_id = 'base_link'

        # odom_msg.pose.pose.position.x = pos_x_enu
        # odom_msg.pose.pose.position.y = pos_y_enu
        # odom_msg.pose.pose.position.z = pos_z_enu

        # odom_msg.pose.pose.orientation = quat
        # self.odom_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)

    rtk_enu_publisher = enu_odom_publisher()

    rclpy.spin(rtk_enu_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rtk_enu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
