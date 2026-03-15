#!/usr/bin/env python3
import math
import time
from typing import Optional, Tuple, List

import numpy as np
import yaml

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose

import tf2_ros
from tf2_ros import TransformException


def _normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < eps:
        return v * 0.0
    return v / n


def _rotmat_from_quat_xyzw(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    x, y, z, w = qx, qy, qz, qw
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ], dtype=float)


def _quat_from_rotmat(R: np.ndarray) -> Tuple[float, float, float, float]:
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S

    q = np.array([qx, qy, qz, qw], dtype=float)
    q = _normalize(q)
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def _look_at_optical_R(world_target: np.ndarray,
                      world_cam_opt: np.ndarray,
                      world_up: np.ndarray = np.array([0.0, 0.0, 1.0], dtype=float)) -> np.ndarray:
    """
    optical frame: +Z forward, +X right, +Y down
    Build world_R_optical so optical +Z points to target.
    """
    z = _normalize(world_target - world_cam_opt)  # forward

    # create optical +Y (down) approximately opposite world_up, orthogonalized against z
    y0 = -_normalize(world_up)
    y = y0 - float(np.dot(y0, z)) * z
    y = _normalize(y)

    # handle degenerate case (looking straight up/down)
    if np.linalg.norm(y) < 1e-6:
        y0 = np.array([0.0, -1.0, 0.0], dtype=float)
        y = y0 - float(np.dot(y0, z)) * z
        y = _normalize(y)

    x = _normalize(np.cross(y, z))
    return np.column_stack((x, y, z))


class GazeboViewpointCapture(Node):
    def __init__(self):
        super().__init__("gazebo_viewpoint_capture_node")

        # do NOT redeclare use_sim_time; launch sets it

        # ---- Params ----
        self.declare_parameter("gt_yaml", "")
        self.declare_parameter("world_name", "default")
        self.declare_parameter("camera_entity", "multi_camera_rig")

        # frames for fixed TF: entity_frame -> optical_frame
        self.declare_parameter("entity_frame", "multi_camera_rig_mount")
        self.declare_parameter("optical_frame", "firefly_left_camera_optical_frame")

        # viewpoint pattern
        self.declare_parameter("side", "front")  # "front" (increasing y) or "back" (decreasing y)
        self.declare_parameter("distances", [0.1, 0.3, 0.5])
        self.declare_parameter("angle_deg", 45.0)

        # timing / timeouts
        self.declare_parameter("settle_s", 0.25)
        self.declare_parameter("pose_timeout_s", 2.0)
        self.declare_parameter("trigger_timeout_s", 2.0)

        self.declare_parameter("use_trigger", True)
        self.declare_parameter("trigger_service", "/trigger/send_trigger")

        self.declare_parameter("loop", False)

        # ---- Read params ----
        self.gt_yaml = str(self.get_parameter("gt_yaml").value)
        self.world_name = str(self.get_parameter("world_name").value)
        self.camera_entity = str(self.get_parameter("camera_entity").value)

        self.entity_frame = str(self.get_parameter("entity_frame").value)
        self.optical_frame = str(self.get_parameter("optical_frame").value)

        self.side = str(self.get_parameter("side").value).strip().lower()
        self.distances = [float(d) for d in self.get_parameter("distances").value]
        self.angle_deg = float(self.get_parameter("angle_deg").value)

        self.settle_s = float(self.get_parameter("settle_s").value)
        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)
        self.trigger_timeout_s = float(self.get_parameter("trigger_timeout_s").value)

        self.use_trigger = bool(self.get_parameter("use_trigger").value)
        self.trigger_service_name = str(self.get_parameter("trigger_service").value)
        self.loop = bool(self.get_parameter("loop").value)

        if not self.gt_yaml:
            raise RuntimeError("gt_yaml is empty. Provide the GT markers YAML path.")
        if self.side not in ("front", "back"):
            raise RuntimeError("side must be 'front' or 'back'")

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Clients (created once, but we may recreate on timeout) ----
        self.pose_service_name = f"/world/{self.world_name}/set_pose"
        self.pose_cli = self.create_client(SetEntityPose, self.pose_service_name)

        self.trigger_cli = None
        if self.use_trigger:
            self.trigger_cli = self.create_client(Trigger, self.trigger_service_name)

        # Fixed entity->optical TF cache
        self._entity_R_opt = None  # R_entity_optical
        self._entity_t_opt = None  # t_entity_optical in entity frame

        # markers
        self._markers = []
        self.frame_id = "world"

        # internal “one outstanding call” guard
        self._pose_inflight = False
        self._trigger_inflight = False

        self.get_logger().info("Gazebo viewpoint capture starting.")
        self.get_logger().info(f"  gt_yaml: {self.gt_yaml}")
        self.get_logger().info(f"  world_name: {self.world_name}")
        self.get_logger().info(f"  pose_service: {self.pose_service_name}")
        self.get_logger().info(f"  camera_entity: {self.camera_entity}")
        self.get_logger().info(f"  side: {self.side}  distances={self.distances}  angle_deg={self.angle_deg}")
        self.get_logger().info(f"  use_trigger: {self.use_trigger} ({self.trigger_service_name})")

        # run once after startup
        self.create_timer(0.1, self._start_once)
        self._started = False

    # ---------------- utilities ----------------

    def _wait_for_service(self, client, name: str, timeout_s: float = 60.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if client.wait_for_service(timeout_sec=0.2):
                return True
            rclpy.spin_once(self, timeout_sec=0.0)
        return False

    def _load_markers(self):
        with open(self.gt_yaml, "r") as f:
            data = yaml.safe_load(f)
        if "markers" not in data:
            raise RuntimeError("GT YAML missing 'markers' list")
        self.frame_id = data.get("frame_id", "world")
        self._markers = data["markers"]
        self.get_logger().info(f"  frame_id: {self.frame_id}")
        self.get_logger().info(f"  markers: {len(self._markers)}")

    def _lookup_fixed_entity_to_optical_tf(self) -> bool:
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.entity_frame,
                self.optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed ({self.entity_frame} -> {self.optical_frame}): {e}")
            return False

        t = tfm.transform.translation
        q = tfm.transform.rotation
        self._entity_t_opt = np.array([t.x, t.y, t.z], dtype=float)
        self._entity_R_opt = _rotmat_from_quat_xyzw(q.x, q.y, q.z, q.w)

        self.get_logger().info("Fixed TF cached:")
        self.get_logger().info(f"  entity->optical translation (entity frame): {self._entity_t_opt.tolist()}")
        return True

    def _make_view_dirs(self) -> List[Tuple[str, np.ndarray]]:
        """
        ROS convention assumed: x forward, y left, z up.

        side=front: approach from increasing y, so camera is at target + (0,-1,0)*d
        side=back : approach from decreasing y, so camera is at target + (0,+1,0)*d

        The view directions here are "from target to camera".
        """
        base = np.array([0.0, -1.0, 0.0], dtype=float) if self.side == "back" else np.array([0.0, 1.0, 0.0], dtype=float)
        base = _normalize(base)

        a = math.radians(self.angle_deg)
        c, s = math.cos(a), math.sin(a)

        # 45 deg offsets relative to approach direction:
        # up/down add +/- z component, left/right add +/- x component now (since we're approaching along y)
        straight = base
        up = _normalize(base * c + np.array([0.0, 0.0, 1.0], dtype=float) * s)
        down = _normalize(base * c + np.array([0.0, 0.0, -1.0], dtype=float) * s)
        left = _normalize(base * c + np.array([1.0, 0.0, 0.0], dtype=float) * s)
        right = _normalize(base * c + np.array([-1.0, 0.0, 0.0], dtype=float) * s)

        return [
            ("straight", straight),
            ("down45", down),
            ("up45", up),
            ("left45", left),
            ("right45", right),
        ]

    def _spin_wait_future(self, fut, timeout_s: float) -> bool:
        """
        Wait for future completion with spin_once.
        Returns True if done, False if timed out.
        """
        t0 = time.time()
        while rclpy.ok() and not fut.done():
            if time.time() - t0 > timeout_s:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.done()

    def _recreate_pose_client(self):
        # Recreate to flush Zenoh query backlog state
        try:
            self.destroy_client(self.pose_cli)
        except Exception:
            pass
        self.pose_cli = self.create_client(SetEntityPose, self.pose_service_name)
        self.get_logger().warn("Recreated pose client to clear stalled requests (Zenoh query backlog).")

    def _recreate_trigger_client(self):
        if self.trigger_cli is None:
            return
        try:
            self.destroy_client(self.trigger_cli)
        except Exception:
            pass
        self.trigger_cli = self.create_client(Trigger, self.trigger_service_name)
        self.get_logger().warn("Recreated trigger client to clear stalled requests (Zenoh query backlog).")

    def _call_set_pose(self, ent_pos_world: np.ndarray, world_R_ent: np.ndarray) -> Optional[bool]:
        if self._pose_inflight:
            self.get_logger().warn("Pose call skipped: previous pose request still in-flight.")
            return None

        req = SetEntityPose.Request()
        req.entity.name = self.camera_entity
        req.pose = Pose()
        req.pose.position.x = float(ent_pos_world[0])
        req.pose.position.y = float(ent_pos_world[1])
        req.pose.position.z = float(ent_pos_world[2])

        qx, qy, qz, qw = _quat_from_rotmat(world_R_ent)
        req.pose.orientation.x = qx
        req.pose.orientation.y = qy
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw

        self._pose_inflight = True
        fut = self.pose_cli.call_async(req)

        done = self._spin_wait_future(fut, self.pose_timeout_s)
        self._pose_inflight = False

        if not done:
            # Cancel if possible and recreate client so we don't pile up queries
            try:
                fut.cancel()
            except Exception:
                pass
            self._recreate_pose_client()
            return None

        res = fut.result()
        if res is None:
            return None
        return bool(res.success)

    def _call_trigger(self) -> Optional[bool]:
        if not self.use_trigger or self.trigger_cli is None:
            return True
        if self._trigger_inflight:
            self.get_logger().warn("Trigger call skipped: previous trigger request still in-flight.")
            return None

        self._trigger_inflight = True
        fut = self.trigger_cli.call_async(Trigger.Request())

        done = self._spin_wait_future(fut, self.trigger_timeout_s)
        self._trigger_inflight = False

        if not done:
            try:
                fut.cancel()
            except Exception:
                pass
            self._recreate_trigger_client()
            return None

        res = fut.result()
        if res is None:
            return None
        return bool(res.success)

    # ---------------- main ----------------

    def _start_once(self):
        if self._started:
            return
        self._started = True

        # wait services
        if not self._wait_for_service(self.pose_cli, self.pose_service_name, timeout_s=60.0):
            self.get_logger().error(f"Pose service not available: {self.pose_service_name}")
            return
        self.get_logger().info(f"Service available: {self.pose_service_name}")

        if self.use_trigger and self.trigger_cli is not None:
            if not self._wait_for_service(self.trigger_cli, self.trigger_service_name, timeout_s=60.0):
                self.get_logger().error(f"Trigger service not available: {self.trigger_service_name}")
                return
            self.get_logger().info(f"Service available: {self.trigger_service_name}")

        # load markers
        self._load_markers()

        # cache TF (retry a bit)
        ok_tf = False
        for _ in range(50):
            if self._lookup_fixed_entity_to_optical_tf():
                ok_tf = True
                break
            time.sleep(0.1)
        if not ok_tf:
            self.get_logger().error("Could not cache TF entity_frame->optical_frame. Check frame names / robot_state_publisher.")
            return

        view_dirs = self._make_view_dirs()

        # Run capture sequence
        while rclpy.ok():
            pose_ok = pose_fail = pose_noresp = 0
            trig_ok = trig_fail = trig_noresp = 0

            for mi, m in enumerate(self._markers, start=1):
                mid = int(m.get("id", -1))
                cls = int(m.get("class_id", -1))
                p = m["position"]
                target = np.array([p["x"], p["y"], p["z"]], dtype=float)

                self.get_logger().info(
                    f"[marker {mi}/{len(self._markers)}] id={mid} class={cls} target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})"
                )

                for d in self.distances:
                    for label, dir_from_target in view_dirs:
                        cam_opt_pos_world = target + dir_from_target * float(d)

                        # optical orientation: +Z to target
                        world_R_opt = _look_at_optical_R(target, cam_opt_pos_world)

                        # entity pose from fixed entity->optical
                        # world_R_ent * R_ent_opt = world_R_opt  => world_R_ent = world_R_opt * inv(R_ent_opt)
                        world_R_ent = world_R_opt @ np.linalg.inv(self._entity_R_opt)

                        # cam_opt = ent_pos + world_R_ent * t_ent_opt => ent_pos = cam_opt - world_R_ent * t_ent_opt
                        ent_pos_world = cam_opt_pos_world - (world_R_ent @ self._entity_t_opt)

                        self.get_logger().info(f"  view {label} @ d={d:.2f}")

                        res_pose = self._call_set_pose(ent_pos_world, world_R_ent)
                        if res_pose is True:
                            pose_ok += 1
                            self.get_logger().info("    pose: OK")
                        elif res_pose is False:
                            pose_fail += 1
                            self.get_logger().error("    pose: FAIL (success=false)")
                        else:
                            pose_noresp += 1
                            self.get_logger().error("    pose: FAIL (no_response)")

                        time.sleep(self.settle_s)

                        res_trig = self._call_trigger()
                        if res_trig is True:
                            trig_ok += 1
                            self.get_logger().info("    trigger: OK")
                        elif res_trig is False:
                            trig_fail += 1
                            self.get_logger().error("    trigger: FAIL (success=false)")
                        else:
                            trig_noresp += 1
                            self.get_logger().error("    trigger: FAIL (no_response)")

            self.get_logger().info(
                f"Done. pose_ok={pose_ok} pose_fail={pose_fail} pose_noresp={pose_noresp} "
                f"trig_ok={trig_ok} trig_fail={trig_fail} trig_noresp={trig_noresp}"
            )

            if not self.loop:
                break

        self.get_logger().info("Sequence complete; node idle.")


def main():
    rclpy.init()
    node = GazeboViewpointCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()