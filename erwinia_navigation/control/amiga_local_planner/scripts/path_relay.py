#!/usr/bin/env python3
"""
Relay /aPath to controller_server (MPPI) via FollowPath action.
- Waits 5s for controller to initialize, then sends path.
- On failure: retries after 3s cooldown.
- On quick repeated failure (<2s): assumes end of path, stops.
- On success: stops.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath

COOLDOWN    = 3.0   # seconds between retries
INIT_DELAY  = 5.0   # seconds to wait for controller_server to activate
QUICK_FAIL  = 2.0   # seconds — failure faster than this = "0 poses" = done


class PathRelay(Node):
    def __init__(self):
        super().__init__('path_relay')
        self._client   = ActionClient(self, FollowPath, 'follow_path')
        self._path_sub = self.create_subscription(Path, '/aPath', self._path_cb, 10)
        self._path     = None
        self._active   = False
        self._done     = False
        self._quick_fails = 0
        self._sent_at  = None
        # Wait for controller to initialize before sending (one-shot)
        self._init_timer = self.create_timer(INIT_DELAY, self._on_ready)

    def _path_cb(self, msg: Path):
        if len(msg.poses) >= 2:
            self._path = msg

    def _on_ready(self):
        self._init_timer.cancel()
        self.get_logger().info('Ready — sending path to controller')
        self._send()

    def _send(self):
        if self._done or self._active:
            return
        if self._path is None:
            self.get_logger().warn('No path received yet — retrying...')
            self.create_timer(COOLDOWN, self._send)
            return
        if not self._client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('controller_server not available, retrying...')
            self.create_timer(COOLDOWN, self._send)
            return
        self._active = True
        self._sent_at = self.get_clock().now()
        self.get_logger().info(f'Sending path ({len(self._path.poses)} poses)')
        goal = FollowPath.Goal()
        goal.path = self._path
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Path rejected — retrying after cooldown')
            self._active = False
            self.create_timer(COOLDOWN, self._send)
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        self._active = False
        elapsed = (self.get_clock().now() - self._sent_at).nanoseconds / 1e9

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded — path complete')
            self._done = True
            return

        if elapsed < QUICK_FAIL:
            # Failed instantly = 0 poses = robot passed end of path
            self._quick_fails += 1
            if self._quick_fails >= 3:
                self.get_logger().info('Path complete — robot passed all waypoints')
                self._done = True
                return
        else:
            # Ran for a while then failed = stuck, reset counter and retry
            self._quick_fails = 0
            self.get_logger().warn(f'Robot stuck after {elapsed:.1f}s — retrying')

        self.create_timer(COOLDOWN, self._send)


def main(args=None):
    rclpy.init(args=args)
    node = PathRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
