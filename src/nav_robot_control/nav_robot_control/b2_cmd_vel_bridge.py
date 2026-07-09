import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _apply_deadband(value: float, deadband: float) -> float:
    return 0.0 if abs(value) < deadband else value


def _twist_is_zero(msg: Twist) -> bool:
    return (
        math.isclose(msg.linear.x, 0.0, abs_tol=1e-6)
        and math.isclose(msg.linear.y, 0.0, abs_tol=1e-6)
        and math.isclose(msg.angular.z, 0.0, abs_tol=1e-6)
    )


class B2CmdVelBridge(Node):
    """Safety wrapper from Navigation cmd_vel to a B2 SDK bridge input topic.

    This node intentionally does not import Unitree SDK2. It keeps the Navigation
    side stable while allowing the actual B2 SDK bridge to subscribe to the
    configured output topic.
    """

    def __init__(self) -> None:
        super().__init__("b2_cmd_vel_bridge")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe")
        self.declare_parameter("nav_stop_topic", "/nav_stop")
        self.declare_parameter("nav_start_topic", "/nav_start")
        self.declare_parameter("status_topic", "/robot_control/status")
        self.declare_parameter("b2_cmd_vel_topic", "/unitree/b2/cmd_vel")
        self.declare_parameter("watchdog_timeout_sec", 0.3)
        self.declare_parameter("publish_period_sec", 0.05)
        self.declare_parameter("require_nav_start", False)
        self.declare_parameter("max_linear_x", 0.15)
        self.declare_parameter("max_linear_y", 0.10)
        self.declare_parameter("max_angular_z", 0.40)
        self.declare_parameter("deadband_linear", 0.01)
        self.declare_parameter("deadband_angular", 0.02)

        self.watchdog_timeout_sec = float(self.get_parameter("watchdog_timeout_sec").value)
        publish_period_sec = max(0.02, float(self.get_parameter("publish_period_sec").value))
        self.require_nav_start = bool(self.get_parameter("require_nav_start").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_linear_y = float(self.get_parameter("max_linear_y").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.deadband_linear = float(self.get_parameter("deadband_linear").value)
        self.deadband_angular = float(self.get_parameter("deadband_angular").value)

        self.latest_cmd: Optional[Twist] = None
        self.latest_cmd_time = 0.0
        self.nav_started = not self.require_nav_start
        self.estop = False
        self.last_published_zero = False
        self.last_status = "idle"
        self.last_status_message = "B2 cmd_vel bridge initialized"

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("b2_cmd_vel_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._on_cmd_vel,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("nav_stop_topic").value),
            self._on_nav_stop,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("nav_start_topic").value),
            self._on_nav_start,
            10,
        )
        self.create_timer(publish_period_sec, self._publish_loop)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            "B2 cmd_vel bridge: "
            f"{self.get_parameter('cmd_vel_topic').value} -> "
            f"{self.get_parameter('b2_cmd_vel_topic').value}"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = self._limited_twist(msg)
        self.latest_cmd_time = time.monotonic()
        if not self.estop:
            self.last_status = "ready"
            self.last_status_message = "received cmd_vel"

    def _on_nav_stop(self, msg: Bool) -> None:
        if msg.data:
            self.estop = True
            self.nav_started = False if self.require_nav_start else self.nav_started
            self._publish_zero(force=True)
            self.last_status = "estop"
            self.last_status_message = "nav_stop received"

    def _on_nav_start(self, msg: Bool) -> None:
        self.nav_started = bool(msg.data)
        if self.nav_started:
            self.estop = False
            self.last_status = "ready"
            self.last_status_message = "nav_start received"
        else:
            self._publish_zero(force=True)
            self.last_status = "idle"
            self.last_status_message = "nav_start false"

    def _publish_loop(self) -> None:
        now = time.monotonic()
        if self.estop:
            self._publish_zero()
            return
        if self.require_nav_start and not self.nav_started:
            self._publish_zero()
            return
        if self.latest_cmd is None or now - self.latest_cmd_time > self.watchdog_timeout_sec:
            self._publish_zero()
            self.last_status = "watchdog_stop"
            self.last_status_message = "cmd_vel watchdog timeout"
            return
        self.cmd_pub.publish(self.latest_cmd)
        self.last_published_zero = _twist_is_zero(self.latest_cmd)
        self.last_status = "commanding" if not self.last_published_zero else "ready"
        self.last_status_message = "forwarding limited cmd_vel"

    def _publish_zero(self, force: bool = False) -> None:
        if force or not self.last_published_zero:
            self.cmd_pub.publish(Twist())
        self.last_published_zero = True

    def _limited_twist(self, msg: Twist) -> Twist:
        out = Twist()
        out.linear.x = _apply_deadband(
            _clamp(float(msg.linear.x), self.max_linear_x), self.deadband_linear
        )
        out.linear.y = _apply_deadband(
            _clamp(float(msg.linear.y), self.max_linear_y), self.deadband_linear
        )
        out.angular.z = _apply_deadband(
            _clamp(float(msg.angular.z), self.max_angular_z), self.deadband_angular
        )
        return out

    def _publish_status(self) -> None:
        payload = {
            "adapter": "b2_cmd_vel_bridge",
            "status": self.last_status,
            "message": self.last_status_message,
            "output_topic": str(self.get_parameter("b2_cmd_vel_topic").value),
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = B2CmdVelBridge()
    try:
        rclpy.spin(node)
    finally:
        node._publish_zero(force=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
