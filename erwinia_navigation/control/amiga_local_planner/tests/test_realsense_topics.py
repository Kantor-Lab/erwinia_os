#!/usr/bin/env python3
"""
Unit test — verify RealSense camera is publishing ROS 2 topics.

Run the RealSense driver first:
  ros2 launch realsense2_camera rs_launch.py

Then run this test:
  python3 tests/test_realsense_topics.py
"""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

TIMEOUT_SEC    = 5.0
COLOR_TOPIC    = '/camera/camera/color/image_raw'
CAMINFO_TOPIC  = '/camera/camera/color/camera_info'


class RealsenseTester(Node):
    def __init__(self):
        super().__init__('realsense_tester')
        self._image_received    = False
        self._caminfo_received  = False

        self.create_subscription(Image,      COLOR_TOPIC,   self._image_cb,   1)
        self.create_subscription(CameraInfo, CAMINFO_TOPIC, self._caminfo_cb, 1)
        self.create_timer(TIMEOUT_SEC, self._timeout_cb)

        self.get_logger().info(f'Waiting up to {TIMEOUT_SEC}s for RealSense topics...')

    def _image_cb(self, msg: Image):
        if not self._image_received:
            self._image_received = True
            self.get_logger().info(
                f'[PASS] {COLOR_TOPIC}  '
                f'{msg.width}x{msg.height}  encoding={msg.encoding}')
        self._check_done()

    def _caminfo_cb(self, msg: CameraInfo):
        if not self._caminfo_received:
            self._caminfo_received = True
            self.get_logger().info(
                f'[PASS] {CAMINFO_TOPIC}  '
                f'{msg.width}x{msg.height}  '
                f'fx={msg.k[0]:.1f}  fy={msg.k[4]:.1f}')
        self._check_done()

    def _check_done(self):
        if self._image_received and self._caminfo_received:
            self.get_logger().info('All topics OK — RealSense is publishing correctly.')
            raise SystemExit(0)

    def _timeout_cb(self):
        failures = []
        if not self._image_received:
            failures.append(COLOR_TOPIC)
        if not self._caminfo_received:
            failures.append(CAMINFO_TOPIC)
        self.get_logger().error(f'TIMEOUT — no messages received on: {failures}')
        self.get_logger().error('Is the RealSense driver running?  '
                                'ros2 launch realsense2_camera rs_launch.py')
        raise SystemExit(1)


def main():
    rclpy.init()
    node = RealsenseTester()
    try:
        rclpy.spin(node)
    except SystemExit as e:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(e.code)


if __name__ == '__main__':
    main()
