#!/usr/bin/env python3
"""
Detects obstacles and humans ahead of the robot.

Subscribes:
  /local_costmap/costmap  (nav_msgs/OccupancyGrid)  — obstacle detection
  /odometry/filtered      (nav_msgs/Odometry)        — robot pose
  /camera/image_raw       (sensor_msgs/Image)        — human detection (YOLO)

Publishes:
  /obstacle_nearby        (std_msgs/Bool)  True = obstacle ahead → use MPPI
  /human_detected         (std_msgs/Bool)  True = human ahead    → stop robot

Human detection logic (YOLO):
  - Run YOLO inference on incoming camera frames
  - Detect COCO class 0 (person) with confidence >= HUMAN_CONF_THRESH
"""
import math
from collections import deque
import cv2
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

# Costmap obstacle detection
SCAN_LENGTH   = 4.0   # m ahead of robot
SCAN_WIDTH    = 3.0   # m lateral width
COST_THRESH   = 50    # costmap cost threshold
PUBLISH_HZ    = 10.0

# YOLO human detection
YOLO_MODEL        = 'yolov8n.pt'   # nano — swap to yolov8s/m/l for better accuracy
HUMAN_CONF_THRESH = 0.5            # minimum confidence to count as a person
CAMERA_TOPIC      = '/camera/camera/color/image_raw'
PERSON_CLASS_ID   = 0              # COCO class 0 = person

# Temporal smoothing — asymmetric hysteresis to prevent mode flickering
DETECT_WINDOW     = 10   # rolling window size (frames)
DETECT_MIN_FRAMES = 5    # minimum frames before making a decision
CONFIRM_RATIO     = 0.6  # ratio to enter STOP  (6/10 frames see person)
CLEAR_RATIO       = 0.2  # ratio to exit  STOP  (must drop below 2/10 frames)


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self._costmap   = None
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0
        self._image          = None
        self._bridge         = CvBridge()
        self._detect_window  = deque(maxlen=DETECT_WINDOW)
        self._human_confirmed = False

        # self.declare_parameter('show_window', True)
        # self._show_window = self.get_parameter('show_window').value

        # self.get_logger().info(f'Loading YOLO model: {YOLO_MODEL}')
        # self._yolo = YOLO(YOLO_MODEL)
        # self.get_logger().info('YOLO model loaded')

        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 self._costmap_cb, 1)
        self.create_subscription(Odometry, '/odometry/global',
                                 self._odom_cb, 10)
        # self.create_subscription(Image, CAMERA_TOPIC,
        #                          self._image_cb, 1)

        self._obs_pub   = self.create_publisher(Bool, '/obstacle_nearby', 10)
        self._human_pub = self.create_publisher(Bool, '/human_detected', 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._check)

    def _odom_cb(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

    def _costmap_cb(self, msg: OccupancyGrid):
        self._costmap = msg

    def _image_cb(self, msg: Image):
        self._image = msg

    # ------------------------------------------------------------------
    # Obstacle detection (costmap)
    # ------------------------------------------------------------------
    def _check_obstacle(self):
        if self._costmap is None:
            return False
        cm  = self._costmap
        res = cm.info.resolution
        ox  = cm.info.origin.position.x
        oy  = cm.info.origin.position.y
        W   = cm.info.width
        H   = cm.info.height
        cos_y = math.cos(self._robot_yaw)
        sin_y = math.sin(self._robot_yaw)
        steps_fwd = int(SCAN_LENGTH / res)
        steps_lat = int((SCAN_WIDTH / 2.0) / res)
        for fi in range(1, steps_fwd + 1):
            for li in range(-steps_lat, steps_lat + 1):
                fwd = fi * res
                lat = li * res
                wx = self._robot_x + cos_y * fwd - sin_y * lat
                wy = self._robot_y + sin_y * fwd + cos_y * lat
                cx = int((wx - ox) / res)
                cy = int((wy - oy) / res)
                if 0 <= cx < W and 0 <= cy < H:
                    if cm.data[cy * W + cx] >= COST_THRESH:
                        return True
        return False

    # ------------------------------------------------------------------
    # Human detection (YOLO)
    # ------------------------------------------------------------------
    def _check_human(self):
        if self._image is None:
            return self._human_confirmed

        try:
            cv_img = self._bridge.imgmsg_to_cv2(self._image, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return self._human_confirmed

        # Run YOLO — check if any person box meets confidence threshold
        results = self._yolo(cv_img, verbose=False)
        person_this_frame = any(
            int(box.cls[0]) == PERSON_CLASS_ID and float(box.conf[0]) >= HUMAN_CONF_THRESH
            for result in results
            for box in result.boxes
        )
        self._detect_window.append(person_this_frame)

        # Draw detections and show window
        if self._show_window:
            vis = cv_img.copy()
            for result in results:
                for box in result.boxes:
                    if int(box.cls[0]) == PERSON_CLASS_ID and float(box.conf[0]) >= HUMAN_CONF_THRESH:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = float(box.conf[0])
                        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(vis, f'person {conf:.2f}', (x1, y1 - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            status = 'STOP (human confirmed)' if self._human_confirmed else 'clear'
            color  = (0, 0, 255) if self._human_confirmed else (200, 200, 200)
            cv2.putText(vis, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
            cv2.imshow('obstacle_detector', vis)
            cv2.waitKey(1)

        # Need at least DETECT_MIN_FRAMES before deciding
        if len(self._detect_window) < DETECT_MIN_FRAMES:
            return self._human_confirmed

        ratio = sum(self._detect_window) / len(self._detect_window)
        prev = self._human_confirmed

        if not self._human_confirmed and ratio >= CONFIRM_RATIO:
            self._human_confirmed = True
            self.get_logger().warn(
                f'Human CONFIRMED (ratio={ratio:.2f})')
        elif self._human_confirmed and ratio < CLEAR_RATIO:
            self._human_confirmed = False
            self.get_logger().info(
                f'Human CLEARED (ratio={ratio:.2f})')

        return self._human_confirmed

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def _check(self):
        human    = False  # self._check_human()  # camera disabled
        obstacle = self._check_obstacle()

        h_msg = Bool(); h_msg.data = human
        o_msg = Bool(); o_msg.data = obstacle
        self._human_pub.publish(h_msg)
        self._obs_pub.publish(o_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
