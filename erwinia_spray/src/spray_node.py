import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger

from .hardware_interface import SerialSprayer


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
            "dry_run",
            False,
            ParameterDescriptor(description="If true, do not open serial; services return success for testing"),
        )

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
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

        self.get_logger().info("Ready: services /spray/start, /spray/stop, /spray/pulse")

    def cb_start(self, req, resp):
        if self.dry_run:
            resp.success = True
            resp.message = "dry_run: would start"
            return resp

        ok = self.sprayer.start()
        resp.success = bool(ok)
        resp.message = "started" if ok else "failed"
        return resp

    def cb_stop(self, req, resp):
        if self.dry_run:
            resp.success = True
            resp.message = "dry_run: would stop"
            return resp

        ok = self.sprayer.stop()
        resp.success = bool(ok)
        resp.message = "stopped" if ok else "failed"
        return resp

    def cb_pulse(self, req, resp):
        pulse_ms = self.get_parameter("pulse_ms").get_parameter_value().integer_value

        if self.dry_run:
            resp.success = True
            resp.message = f"dry_run: would pulse {pulse_ms}ms"
            return resp

        ok = self.sprayer.pulse(int(pulse_ms))
        resp.success = bool(ok)
        resp.message = f"pulsed {pulse_ms}ms" if ok else "failed"
        return resp

    def destroy_node(self):
        try:
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