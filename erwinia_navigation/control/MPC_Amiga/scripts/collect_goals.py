#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from rosgraph_msgs.msg import Clock
from swiftnav_ros2_driver.msg import Baseline
import numpy as np
import os

import threading
# from sensor_msgs.msg import Imu
# from action_msgs.msg import GoalStatusArray
# from tf_transformations import euler_from_quaternion

class GoalReacher(Node):
    def __init__(self):
        super().__init__('goal_reacher')

        self.robot_position_odom = []
        self.robot_position_gps = []
        self.curr_time = Clock()
        self.stamp_old = 0

        # Subscriptions
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.callback_odom,
            10
        )

        # Uncomment if needed
        # self.sub_gps = self.create_subscription(
        #     TransformStamped,
        #     'rover/piksi/position_receiver_0/ros/transform_enu',
        #     self.callback_gps,
        #     10
        # )

        # self.sub_clock = self.create_subscription(
        #     Clock,
        #     '/clock',
        #     self.callback_clock,
        #     10
        # )

        # self.sub_status = self.create_subscription(
        #     GoalStatusArray,
        #     'move_base/status',
        #     self.callback_status,
        #     10
        # )

    def callback_odom(self, msg):
        # Using pose.pose instead of transform for Odometry
        position = msg.pose.pose.position
        self.robot_position_odom.append([position.x, position.y, 0., 0., 0., 0.])
        # self.get_logger().info(f"position: {position}")

    # Uncomment if needed
    # def callback_gps(self, msg):
    #     orientation = msg.transform.rotation
    #     quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    #     _, _, yaw = euler_from_quaternion(quat)
    #     self.robot_position_gps.append([msg.transform.translation.x, msg.transform.translation.y, yaw, 0., 0., 0.])

    # def callback_clock(self, msg):
    #     self.curr_time = msg

    # def callback_status(self, msg):
    #     if not msg.status_list:
    #         return
    #     m = msg.status_list[-1]
    #     stamp = msg.header.stamp.sec
    #     if m.text == "Goal reached." and stamp - self.stamp_old > 1.0:
    #         self.get_logger().info(f'STATUS: {m.status}')
    #         self.stamp_old = stamp


def main(args=None):
    rclpy.init(args=args)
    goal_reacher = GoalReacher()

    # Start spinning the node in a background thread
    thread = threading.Thread(target=rclpy.spin, args=(goal_reacher,), daemon=True)
    thread.start()

    f2 = '/home/appleseed_labs/erwinia_os/src/erwinia_navigation/control/MPC_Amiga/gps_coordinates/barn_field_waypoints.txt'
    mode = 'subsample'  # or 'capture'
    subsample_rate = 20

    try:
        if mode == 'capture':
            while True:
                x = input('Save_point? ')
                if x.lower() == 'y':
                    if os.path.exists(f2):
                        c1 = np.loadtxt(f2, delimiter=',')
                    else:
                        c1 = np.empty((0, 6))

                    if len(c1) == 0:
                        np.savetxt(f2, [goal_reacher.robot_position_odom[-1]], delimiter=',')
                    else:
                        np.savetxt(f2, np.vstack([c1, goal_reacher.robot_position_odom[-1]]), delimiter=',')

                    print(c1)
                rclpy.spin_once(goal_reacher, timeout_sec=0.1)

        else:  # subsample mode

            # while True:
            #     x = input('Done?: ')
            #     if x.lower() == 'y':
            #         break
            input("Press Enter when ready to save subsampled trajectory...")
            if goal_reacher.robot_position_odom:

                gps = np.stack(goal_reacher.robot_position_odom)[::subsample_rate]
                np.savetxt(f2, gps, delimiter=',')
                print('Saved subsampled trajectories')

    except KeyboardInterrupt:
        pass
    finally:
        goal_reacher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
