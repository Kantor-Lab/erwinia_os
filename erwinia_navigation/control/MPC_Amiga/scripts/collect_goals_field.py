#!/usr/bin/env python3

import os
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from rosgraph_msgs.msg import Clock
# from actionlib_msgs.msg import GoalStatusArray

# If you later re-enable GPS yaw extraction:
#   sudo apt install ros-<distro>-tf-transformations
# then:
# from tf_transformations import euler_from_quaternion

current_dir = os.path.dirname(__file__)
ignore_first_n_entries = 4


class GoalReacher(Node):
    def __init__(self):
        super().__init__('goal_reacher')

        self.robot_position_odom = []
        self.robot_position_gps = []
        # self.curr_time = Clock()

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # Optional subscriptions you had in ROS1 (left commented):
        # self.sub_gps = self.create_subscription(
        #     TransformStamped,
        #     'rover/piksi/position_receiver_0/ros/transform_enu',
        #     self.gps_callback,
        #     10
        # )
        # self.sub_clock = self.create_subscription(
        #     Clock,
        #     '/clock',
        #     self.clock_callback,
        #     10
        # )
        # self.sub_status = self.create_subscription(
        #     GoalStatusArray,
        #     'move_base/status',
        #     self.status_callback,
        #     10
        # )

        self.stamp_old_sec = 0

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.robot_position_odom.append([p.x, p.y, p.z, 0.0, 0.0, 0.0])

    # def gps_callback(self, msg: TransformStamped):
    #     q = msg.transform.rotation
    #     r_quat = [q.x, q.y, q.z, q.w]
    #     _, _, yaw = euler_from_quaternion(r_quat)
    #     t = msg.transform.translation
    #     self.robot_position_gps.append([t.x, t.y, yaw, 0.0, 0.0, 0.0])

    # def clock_callback(self, msg: Clock):
    #     self.curr_time = msg

    # def status_callback(self, msg: GoalStatusArray):
    #     if not msg.status_list:
    #         return
    #     m = msg.status_list[-1]
    #     stamp_sec = msg.header.stamp.sec
    #     if m.text == "Goal reached." and (stamp_sec - self.stamp_old_sec) > 1:
    #         self.get_logger().info(f"STATUS: {m.status}")
    #         # self.pub_next()
    #         self.stamp_old_sec = stamp_sec


def spin_in_thread(node: Node):
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    th = threading.Thread(target=executor.spin, daemon=True)
    th.start()
    return executor, th


def main():
    # Files (same as your ROS1 script)
    f2 = os.path.join(current_dir, '../gps_coordinates/barn_field_waypoints.txt')
    f3 = os.path.join(current_dir, '../gps_coordinates/stopping_points.txt')
    f4 = os.path.join(current_dir, '../gps_coordinates/stopping_points_copied.txt')

    mode = 'subsample'
    subsample_rate = 20

    rclpy.init()
    node = GoalReacher()
    executor, th = spin_in_thread(node)

    try:
        if mode == 'capture':
            while rclpy.ok():
                x = input('Save_point? ')
                if x == 'y':
                    if len(node.robot_position_odom) == 0:
                        node.get_logger().warn("No odometry received yet.")
                        continue

                    new_row = node.robot_position_odom[-1]

                    # Append-to-file logic similar to your original
                    try:
                        c1 = np.loadtxt(f2, delimiter=',')
                        # If file has a single row, loadtxt returns shape (6,), so normalize
                        if c1.ndim == 1 and c1.size == 6:
                            c1 = c1.reshape(1, -1)
                        out = np.vstack([c1, new_row]) if c1.size else np.array([new_row])
                    except Exception:
                        out = np.array([new_row])

                    np.savetxt(f2, out, delimiter=',')
                    print(out)

        else:
            while rclpy.ok():
                x = input('Done?: ')
                if x == 'y':
                    break

            if len(node.robot_position_odom) == 0:
                node.get_logger().error("No odometry received; nothing to save.")
                return

            ekf = np.stack(node.robot_position_odom)[::subsample_rate]

            print(f'Removed the first {ignore_first_n_entries} entries out of {len(ekf)}')
            print(ekf[0:ignore_first_n_entries])
            ekf = ekf[ignore_first_n_entries:]

            np.savetxt(f2, ekf, delimiter=',')
            print('Saved subsampled trajectories')

        # Write repeated last row to stopping points files (same behavior)
        last_row = np.loadtxt(f2, delimiter=',')[-1]
        repeated_last_row = np.tile(last_row, (2, 1))
        np.savetxt(f3, repeated_last_row, delimiter=',')
        np.savetxt(f4, repeated_last_row, delimiter=',')
        print('Saved stopping points (repeated last row)')

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
