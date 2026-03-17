#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger

from erwinia_msgs.action import MarkLocation
from hardware_interface import SerialSprayer


class SprayNode(Node):
    def __init__(self):
        super().__init__("spray_node")

        self.declare_parameter(
            "port",
            "/dev/ttyACM0",
            ParameterDescriptor(description="Arduino serial port, e.g. /dev/ttyACM0 or /dev/ttyUSB0"),
        )
        self.declare_parameter("baud", 115200)
        self.declare_parameter(
            "pulse_ms",
            3000,
            ParameterDescriptor(description="Default pulse duration for /spray/pulse (ms)"),
        )
        self.declare_parameter(
            "mark_action_name",
            "mark_server",
            ParameterDescriptor(description="Action name used by the behavior tree for marking"),
        )
        self.declare_parameter(
            "dry_run",
            False,
            ParameterDescriptor(description="If true, do not open serial; services return success for testing"),
        )

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.mark_action_name = (
            self.get_parameter("mark_action_name").get_parameter_value().string_value
        )
        self.dry_run = self.get_parameter("dry_run").get_parameter_value().bool_value

        if self.dry_run:
            self.get_logger().warn(
                "DRY RUN enabled: not opening serial port. Service calls will succeed without hardware."
            )
            self.sprayer = None
        else:
            self.get_logger().info(f"Opening serial port {self.port} @ {self.baud}...")
            self.sprayer = SerialSprayer(port=self.port, baud=int(self.baud))

        self.create_service(Trigger, "spray/start", self.cb_start)
        self.create_service(Trigger, "spray/stop", self.cb_stop)
        self.create_service(Trigger, "spray/pulse", self.cb_pulse)
        self._mark_action_server = ActionServer(
            self,
            MarkLocation,
            self.mark_action_name,
            execute_callback=self.execute_mark_callback,
            goal_callback=self.handle_mark_goal,
            cancel_callback=self.handle_mark_cancel,
        )

        self.get_logger().info(
            f"Ready: services /spray/start, /spray/stop, /spray/pulse; "
            f"MarkLocation action '{self.mark_action_name}'"
        )

    def _perform_pulse(self, pulse_ms):
        if self.dry_run:
            return True, f"dry_run: would pulse {pulse_ms}ms"

        try:
            ok = bool(self.sprayer.pulse(int(pulse_ms)))
        except Exception as exc:
            self.get_logger().error(f"Spray pulse failed with exception: {exc}")
            return False, str(exc)

        return ok, (f"pulsed {pulse_ms}ms" if ok else "failed")

    def cb_start(self, req, resp):
        del req
        if self.dry_run:
            resp.success = True
            resp.message = "dry_run: would start"
            return resp

        try:
            ok = self.sprayer.start()
        except Exception as exc:
            self.get_logger().error(f"Failed to start sprayer: {exc}")
            ok = False
        resp.success = bool(ok)
        resp.message = "started" if ok else "failed"
        return resp

    def cb_stop(self, req, resp):
        del req
        if self.dry_run:
            resp.success = True
            resp.message = "dry_run: would stop"
            return resp

        try:
            ok = self.sprayer.stop()
        except Exception as exc:
            self.get_logger().error(f"Failed to stop sprayer: {exc}")
            ok = False
        resp.success = bool(ok)
        resp.message = "stopped" if ok else "failed"
        return resp

    def cb_pulse(self, req, resp):
        del req
        pulse_ms = self.get_parameter("pulse_ms").get_parameter_value().integer_value

        ok, message = self._perform_pulse(int(pulse_ms))
        resp.success = bool(ok)
        resp.message = message
        return resp

    def handle_mark_goal(self, goal_request):
        del goal_request
        self.get_logger().info("Received MarkLocation goal")
        return GoalResponse.ACCEPT

    def handle_mark_cancel(self, goal_handle):
        del goal_handle
        self.get_logger().info("Received MarkLocation cancel request")
        return CancelResponse.ACCEPT

    def execute_mark_callback(self, goal_handle):
        pulse_ms = self.get_parameter("pulse_ms").get_parameter_value().integer_value
        result = MarkLocation.Result()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.marked = False
            result.error = "goal canceled before spraying"
            self.get_logger().info("MarkLocation goal canceled before execution")
            return result

        marked, message = self._perform_pulse(int(pulse_ms))
        result.marked = bool(marked)
        result.error = "" if marked else message

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            if result.marked:
                result.marked = False
                result.error = "goal canceled during spraying"
            self.get_logger().info("MarkLocation goal canceled during execution")
            return result

        if marked:
            goal_handle.succeed()
            self.get_logger().info(f"MarkLocation goal succeeded: {message}")
        else:
            goal_handle.abort()
            self.get_logger().error(f"MarkLocation goal failed: {message}")

        return result

    def destroy_node(self):
        try:
            if hasattr(self, "_mark_action_server"):
                self._mark_action_server.destroy()
            if self.sprayer is not None:
                self.sprayer.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SprayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
