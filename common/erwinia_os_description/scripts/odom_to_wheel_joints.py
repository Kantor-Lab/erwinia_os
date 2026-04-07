#!/usr/bin/env python3
"""
Derives approximate wheel joint positions from /odom twist and publishes
them as sensor_msgs/JointState so robot_state_publisher can animate the wheels.

Skid-steer model:
  v_left  = v_x - omega_z * (track / 2)
  v_right = v_x + omega_z * (track / 2)
  wheel_angle_rate = v_side / wheel_radius
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class OdomToWheelJoints(Node):
    def __init__(self):
        super().__init__('odom_to_wheel_joints')

        self.declare_parameter('wheel_radius', 0.2159)
        self.declare_parameter('track', 0.98)
        self.declare_parameter('joint_prefix', 'amiga/')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track = self.get_parameter('track').value
        self.joint_prefix = self.get_parameter('joint_prefix').value

        self._left_pos = 0.0
        self._right_pos = 0.0
        self._last_stamp = None

        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self._sub = self.create_subscription(Odometry, 'odom', self._odom_cb, 10)

    def _odom_cb(self, msg: Odometry):
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9

        if self._last_stamp is not None:
            dt = t - self._last_stamp
            if dt <= 0.0:
                return

            v_x = msg.twist.twist.linear.x
            omega_z = msg.twist.twist.angular.z
            half_track = self.track / 2.0

            v_left  = v_x - omega_z * half_track
            v_right = v_x + omega_z * half_track

            self._left_pos  += (v_left  / self.wheel_radius) * dt
            self._right_pos += (v_right / self.wheel_radius) * dt

        self._last_stamp = t

        p = self.joint_prefix
        js = JointState()
        js.header.stamp = stamp
        js.name = [
            f'{p}front_left_wheel_joint',
            f'{p}rear_left_wheel_joint',
            f'{p}front_right_wheel_joint',
            f'{p}rear_right_wheel_joint',
        ]
        js.position = [
            self._left_pos,
            self._left_pos,
            self._right_pos,
            self._right_pos,
        ]
        js.velocity = []
        js.effort = []
        self._pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToWheelJoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
