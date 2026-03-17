#!/usr/bin/env python3
"""
Switches /cmd_vel between MPC, MPPI, and STOP based on detection state.

Subscribes:
  /mpc_cmd_vel              (MPC direct control)
  /local_planner_cmd_vel    (MPPI — obstacle avoidance)
  /obstacle_nearby          (std_msgs/Bool)
  /human_detected           (std_msgs/Bool)

Publishes:
  /cmd_vel                  (selected output)
  /active_controller        (std_msgs/String) — 'MPC', 'MPPI', or 'STOP'

Priority logic (combo mode):
  human_detected=True            → STOP (zero velocity, highest priority)
  obstacle_nearby=True, no human → MPPI
  no obstacle, no human          → MPC (after CLEAR_DELAY hysteresis)

Parameters:
  mode     (str, default 'combo') — 'mpc' | 'mppi' | 'combo'
  dry_run  (str, default 'true')  — 'true' publishes zero velocity (visualization only)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

CLEAR_DELAY  = 0.5   # seconds of clear before switching back to MPC
CMD_TIMEOUT  = 0.5   # seconds — if no cmd received, send zero
DECEL_RATE   = 0.3   # m/s² — deceleration rate when human detected
ACCEL_RATE   = 0.15  # m/s² — ramp-up rate after STOP clears (half of decel)


class CmdVelSwitcher(Node):
    def __init__(self):
        super().__init__('cmd_vel_switcher')
        self.declare_parameter('mode',    'combo')
        self.declare_parameter('dry_run', True)
        self._mode    = self.get_parameter('mode').value   # 'mpc', 'mppi', 'combo'
        self._dry_run = self.get_parameter('dry_run').value
        self._use_mppi      = False
        self._human         = False
        self._clear_since   = None
        self._mpc_cmd       = None
        self._mppi_cmd      = None
        self._mpc_time      = None
        self._mppi_time     = None
        self._last_vx       = 0.0   # tracked for deceleration ramp
        self._last_pub_time = None

        self.create_subscription(Twist, '/mpc_cmd_vel',   self._mpc_cb,   10)
        self.create_subscription(Twist, '/local_planner_cmd_vel', self._mppi_cb, 10)
        self.create_subscription(Bool,  '/obstacle_nearby', self._obstacle_cb, 10)
        self.create_subscription(Bool,  '/human_detected',  self._human_cb,   10)

        self._pub      = self.create_publisher(Twist,  '/cmd_vel',           10)
        self._mode_pub = self.create_publisher(String, '/active_controller', 10)
        self.create_timer(0.05, self._publish)   # 20 Hz

    def _mpc_cb(self, msg):
        self._mpc_cmd  = msg
        self._mpc_time = self.get_clock().now()

    def _mppi_cb(self, msg):
        self._mppi_cmd  = msg
        self._mppi_time = self.get_clock().now()

    def _human_cb(self, msg: Bool):
        self._human = msg.data

    def _obstacle_cb(self, msg: Bool):
        now = self.get_clock().now()
        if msg.data:
            self._use_mppi    = True
            self._clear_since = None
        else:
            if self._use_mppi:
                if self._clear_since is None:
                    self._clear_since = now
                elif (now - self._clear_since).nanoseconds / 1e9 >= CLEAR_DELAY:
                    self._use_mppi    = False
                    self._clear_since = None

    def _publish(self):
        now = self.get_clock().now()

        def fresh(t):
            return t is not None and (now - t).nanoseconds / 1e9 < CMD_TIMEOUT

        # dt for deceleration ramp
        dt = 0.05
        if self._last_pub_time is not None:
            dt = max(0.001, (now - self._last_pub_time).nanoseconds / 1e9)
        self._last_pub_time = now

        # Determine which branch to use based on mode param
        use_mppi_branch = (
            self._mode == 'mppi' or
            (self._mode == 'combo' and (self._human or self._use_mppi))
        )

        if use_mppi_branch:
            if fresh(self._mppi_time):
                target = self._mppi_cmd.linear.x
                if self._last_vx < target:
                    self._last_vx = min(target, self._last_vx + ACCEL_RATE * dt)
                else:
                    self._last_vx = target
                cmd = Twist()
                if not self._dry_run:
                    cmd.linear.x  = self._last_vx
                    cmd.angular.z = self._mppi_cmd.angular.z
                self._pub.publish(cmd)
            else:
                self._pub.publish(Twist())
                self._last_vx = 0.0
            mode = 'MPPI'

        # Priority 3: clear → MPC (ramp up if coming from STOP)
        else:
            if fresh(self._mpc_time):
                target = self._mpc_cmd.linear.x
                if self._last_vx < target:
                    self._last_vx = min(target, self._last_vx + ACCEL_RATE * dt)
                else:
                    self._last_vx = target
                cmd = Twist()
                if not self._dry_run:
                    cmd.linear.x  = self._last_vx
                    cmd.angular.z = self._mpc_cmd.angular.z
                self._pub.publish(cmd)
            else:
                self._pub.publish(Twist())
                self._last_vx = 0.0
            mode = 'MPC'

        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
