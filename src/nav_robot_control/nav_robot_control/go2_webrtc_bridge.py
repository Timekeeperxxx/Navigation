import asyncio
import json
import math
import threading
import time
from typing import Any, Optional

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


class Go2WebRTCBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_webrtc_bridge")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe")
        self.declare_parameter("nav_stop_topic", "/nav_stop")
        self.declare_parameter("nav_start_topic", "/nav_start")
        self.declare_parameter("status_topic", "/robot_control/status")
        self.declare_parameter("connection_method", "LocalSTA")
        self.declare_parameter("robot_ip", "192.168.8.181")
        self.declare_parameter("serial_number", "")
        self.declare_parameter("aes_128_key", "")
        self.declare_parameter("reconnect_delay_sec", 3.0)
        self.declare_parameter("command_period_sec", 0.05)
        self.declare_parameter("watchdog_timeout_sec", 0.3)
        self.declare_parameter("require_nav_start", False)
        self.declare_parameter("ensure_motion_mode", True)
        self.declare_parameter("motion_mode", "normal")
        self.declare_parameter("stand_on_connect", True)
        self.declare_parameter("stand_command", "BalanceStand")
        self.declare_parameter("stand_wait_sec", 1.0)
        self.declare_parameter("continuous_gait_on_connect", True)
        self.declare_parameter("continuous_gait_value", 1)
        self.declare_parameter("speed_level_on_connect", False)
        self.declare_parameter("speed_level", 1)
        self.declare_parameter("move_command_mode", "request")
        self.declare_parameter("mcf_linear_scale", 1.0)
        self.declare_parameter("mcf_angular_scale", 1.0)
        self.declare_parameter("mcf_max_x", 1.0)
        self.declare_parameter("mcf_max_y", 1.0)
        self.declare_parameter("mcf_max_z", 1.0)
        self.declare_parameter("wireless_linear_scale", 3.0)
        self.declare_parameter("wireless_angular_scale", 2.0)
        self.declare_parameter("soft_stop_mode", "zero_velocity")
        self.declare_parameter("hard_stop_on_nav_stop", True)
        self.declare_parameter("use_remote_command_from_api", True)
        self.declare_parameter("enable_builtin_obstacle_avoidance", False)
        self.declare_parameter("stop_on_shutdown", True)
        self.declare_parameter("max_linear_x", 0.15)
        self.declare_parameter("max_linear_y", 0.10)
        self.declare_parameter("max_angular_z", 0.40)
        self.declare_parameter("deadband_linear", 0.01)
        self.declare_parameter("deadband_angular", 0.02)

        self.reconnect_delay_sec = float(self.get_parameter("reconnect_delay_sec").value)
        self.command_period_sec = max(0.02, float(self.get_parameter("command_period_sec").value))
        self.watchdog_timeout_sec = float(self.get_parameter("watchdog_timeout_sec").value)
        self.require_nav_start = bool(self.get_parameter("require_nav_start").value)
        self.ensure_motion_mode = bool(self.get_parameter("ensure_motion_mode").value)
        self.motion_mode = str(self.get_parameter("motion_mode").value)
        self.stand_on_connect = bool(self.get_parameter("stand_on_connect").value)
        self.stand_command = str(self.get_parameter("stand_command").value)
        self.stand_wait_sec = float(self.get_parameter("stand_wait_sec").value)
        self.continuous_gait_on_connect = bool(
            self.get_parameter("continuous_gait_on_connect").value
        )
        self.continuous_gait_value = int(self.get_parameter("continuous_gait_value").value)
        self.speed_level_on_connect = bool(self.get_parameter("speed_level_on_connect").value)
        self.speed_level = int(self.get_parameter("speed_level").value)
        self.move_command_mode = str(self.get_parameter("move_command_mode").value)
        self.mcf_linear_scale = float(self.get_parameter("mcf_linear_scale").value)
        self.mcf_angular_scale = float(self.get_parameter("mcf_angular_scale").value)
        self.mcf_max_x = float(self.get_parameter("mcf_max_x").value)
        self.mcf_max_y = float(self.get_parameter("mcf_max_y").value)
        self.mcf_max_z = float(self.get_parameter("mcf_max_z").value)
        self.wireless_linear_scale = float(self.get_parameter("wireless_linear_scale").value)
        self.wireless_angular_scale = float(self.get_parameter("wireless_angular_scale").value)
        self.soft_stop_mode = str(self.get_parameter("soft_stop_mode").value)
        self.hard_stop_on_nav_stop = bool(self.get_parameter("hard_stop_on_nav_stop").value)
        self.use_remote_command_from_api = bool(
            self.get_parameter("use_remote_command_from_api").value
        )
        self.enable_builtin_obstacle_avoidance = bool(
            self.get_parameter("enable_builtin_obstacle_avoidance").value
        )
        self.stop_on_shutdown = bool(self.get_parameter("stop_on_shutdown").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_linear_y = float(self.get_parameter("max_linear_y").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.deadband_linear = float(self.get_parameter("deadband_linear").value)
        self.deadband_angular = float(self.get_parameter("deadband_angular").value)

        self.lock = threading.Lock()
        self.latest_cmd: Optional[Twist] = None
        self.latest_cmd_time = 0.0
        self.nav_started = not self.require_nav_start
        self.estop = False
        self.shutdown_requested = False
        self.connected = False
        self.last_status = "idle"
        self.last_status_message = "Go2 WebRTC bridge initialized"
        self.conn: Any = None
        self.last_go2_command: dict[str, Any] = {}

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
        self.create_timer(1.0, self._publish_status)

        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, name="go2-webrtc-loop", daemon=True)
        self.thread.start()
        self.get_logger().info(
            "Go2 WebRTC bridge subscribing to "
            f"{self.get_parameter('cmd_vel_topic').value}"
        )

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._driver_loop())

    def _on_cmd_vel(self, msg: Twist) -> None:
        limited = self._limited_twist(msg)
        with self.lock:
            self.latest_cmd = limited
            self.latest_cmd_time = time.monotonic()
            if not self.estop:
                self.last_status = "ready"
                self.last_status_message = (
                    f"received cmd_vel x={limited.linear.x:.3f}, "
                    f"y={limited.linear.y:.3f}, yaw={limited.angular.z:.3f}"
                )

    def _on_nav_stop(self, msg: Bool) -> None:
        if msg.data:
            with self.lock:
                self.estop = True
                if self.require_nav_start:
                    self.nav_started = False
                self.last_status = "estop"
                self.last_status_message = "nav_stop received"
            self._schedule_hard_stop() if self.hard_stop_on_nav_stop else self._schedule_soft_stop()

    def _on_nav_start(self, msg: Bool) -> None:
        with self.lock:
            self.nav_started = bool(msg.data)
            if self.nav_started:
                self.estop = False
                self.last_status = "ready"
                self.last_status_message = "nav_start received"
            else:
                self.last_status = "idle"
                self.last_status_message = "nav_start false"
        if not msg.data:
            self._schedule_soft_stop()

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

    async def _driver_loop(self) -> None:
        while not self.shutdown_requested:
            try:
                await self._connect()
                await self._command_loop()
            except Exception as exc:
                with self.lock:
                    self.connected = False
                    self.last_status = "error"
                    self.last_status_message = f"{type(exc).__name__}: {exc}"
                self.get_logger().error(f"Go2 WebRTC bridge error: {exc}")
                await asyncio.sleep(self.reconnect_delay_sec)
        if self.stop_on_shutdown:
            await self._send_stop()

    async def _connect(self) -> None:
        try:
            from unitree_webrtc_connect.constants import OBSTACLES_AVOID_API, RTC_TOPIC
            from unitree_webrtc_connect.webrtc_driver import (
                UnitreeWebRTCConnection,
                WebRTCConnectionMethod,
            )
        except ImportError as exc:
            raise RuntimeError(
                "unitree_webrtc_connect is not installed; install it before enabling go2_webrtc"
            ) from exc

        method_name = str(self.get_parameter("connection_method").value)
        method = getattr(WebRTCConnectionMethod, method_name, None)
        if method is None:
            raise RuntimeError(f"unsupported Go2 connection_method: {method_name}")

        kwargs: dict[str, str] = {}
        robot_ip = str(self.get_parameter("robot_ip").value).strip()
        serial_number = str(self.get_parameter("serial_number").value).strip()
        aes_128_key = str(self.get_parameter("aes_128_key").value).strip()
        if robot_ip and method_name == "LocalSTA":
            kwargs["ip"] = robot_ip
        if serial_number:
            kwargs["serialNumber"] = serial_number
        if aes_128_key:
            kwargs["aes_128_key"] = aes_128_key

        with self.lock:
            self.last_status = "connecting"
            self.last_status_message = f"connecting to Go2 via {method_name}"

        self.conn = UnitreeWebRTCConnection(method, **kwargs)
        await self.conn.connect()
        with self.lock:
            self.connected = True
            self.last_status = "connected"
            self.last_status_message = "Go2 WebRTC connected"
        self.get_logger().info("Go2 WebRTC connected")

        if self.ensure_motion_mode:
            await self._ensure_motion_mode(RTC_TOPIC)
        if self.stand_on_connect:
            await self._send_stand()
        if self.continuous_gait_on_connect:
            await self._send_continuous_gait(True)
        if self.speed_level_on_connect:
            await self._send_speed_level()
        if self.use_remote_command_from_api:
            await self._set_remote_command_from_api(RTC_TOPIC, OBSTACLES_AVOID_API)
        if self.enable_builtin_obstacle_avoidance:
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["OBSTACLES_AVOID"],
                {
                    "api_id": OBSTACLES_AVOID_API["SWITCH_SET"],
                    "parameter": {"enable": True},
                },
            )

    async def _set_remote_command_from_api(
        self,
        rtc_topic: dict[str, str],
        obstacles_avoid_api: dict[str, int],
    ) -> None:
        self.get_logger().info("Enabling Go2 remote command from API")
        response = await self.conn.datachannel.pub_sub.publish_request_new(
            rtc_topic["OBSTACLES_AVOID"],
            {
                "api_id": obstacles_avoid_api["USE_REMOTE_COMMAND_FROM_API"],
                "parameter": {"is_remote_commands_from_api": True},
            },
        )
        code = response.get("data", {}).get("header", {}).get("status", {}).get("code")
        self.get_logger().info(f"Go2 remote command from API response code: {code}")

    async def _ensure_motion_mode(self, rtc_topic: dict[str, str]) -> None:
        self.get_logger().info("Checking Go2 motion mode")
        response = await self.conn.datachannel.pub_sub.publish_request_new(
            rtc_topic["MOTION_SWITCHER"],
            {"api_id": 1001},
        )
        current_mode = ""
        if response.get("data", {}).get("header", {}).get("status", {}).get("code") == 0:
            data = response.get("data", {}).get("data", "")
            if data:
                current_mode = json.loads(data).get("name", "")
        self.get_logger().info(f"Go2 current motion mode: {current_mode or 'unknown'}")
        if current_mode != self.motion_mode:
            self.get_logger().info(f"Switching Go2 motion mode to {self.motion_mode}")
            await self.conn.datachannel.pub_sub.publish_request_new(
                rtc_topic["MOTION_SWITCHER"],
                {"api_id": 1002, "parameter": {"name": self.motion_mode}},
            )
            await asyncio.sleep(5.0)

    async def _command_loop(self) -> None:
        sent_stop = False
        while not self.shutdown_requested:
            cmd = self._active_command()
            if cmd is None or _twist_is_zero(cmd):
                if not sent_stop:
                    await self._send_soft_stop()
                    sent_stop = True
                await asyncio.sleep(self.command_period_sec)
                continue
            await self._send_move(cmd)
            sent_stop = False
            with self.lock:
                self.last_status = "commanding"
                self.last_status_message = (
                    f"x={cmd.linear.x:.3f}, y={cmd.linear.y:.3f}, yaw={cmd.angular.z:.3f}"
                )
            await asyncio.sleep(self.command_period_sec)

    def _active_command(self) -> Optional[Twist]:
        now = time.monotonic()
        with self.lock:
            if self.estop:
                return None
            if self.require_nav_start and not self.nav_started:
                return None
            if self.latest_cmd is None:
                return None
            if now - self.latest_cmd_time > self.watchdog_timeout_sec:
                self.last_status = "watchdog_stop"
                self.last_status_message = "cmd_vel watchdog timeout"
                return None
            return self.latest_cmd

    async def _send_move(self, cmd: Twist) -> None:
        if self.move_command_mode == "obstacles_avoid_move":
            self._send_obstacles_avoid_move(cmd)
            return
        if self.move_command_mode == "wireless_controller":
            self._send_wireless_controller(cmd)
            return
        if self.move_command_mode == "no_reply":
            self._send_move_no_reply(cmd)
            return
        from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {
                "api_id": SPORT_CMD["Move"],
                "parameter": {"x": cmd.linear.x, "y": cmd.linear.y, "z": cmd.angular.z},
            },
        )

    def _send_move_no_reply(self, cmd: Twist) -> None:
        from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD_MCF

        x = _clamp(cmd.linear.x * self.mcf_linear_scale, self.mcf_max_x)
        y = _clamp(cmd.linear.y * self.mcf_linear_scale, self.mcf_max_y)
        z = _clamp(cmd.angular.z * self.mcf_angular_scale, self.mcf_max_z)
        generated_id = int(time.time() * 1000) % 2147483648
        payload = {
            "header": {
                "identity": {"id": generated_id, "api_id": SPORT_CMD_MCF["Move"]},
                "policy": {"priority": 0, "noreply": True},
            },
            "parameter": json.dumps({"x": x, "y": y, "z": z}),
            "binary": [],
        }
        self.last_go2_command = {
            "mode": "mcf_no_reply",
            "x": x,
            "y": y,
            "z": z,
            "source_x": cmd.linear.x,
            "source_y": cmd.linear.y,
            "source_z": cmd.angular.z,
        }
        self.conn.datachannel.pub_sub.publish_without_callback(RTC_TOPIC["SPORT_MOD"], payload)

    def _send_wireless_controller(self, cmd: Twist) -> None:
        from unitree_webrtc_connect.constants import RTC_TOPIC

        # Unitree's WebRTC joystick topic uses normalized axes, not m/s.
        # The example maps forward to ly and rotation to rx.
        lx = _clamp(cmd.linear.y * self.wireless_linear_scale, 1.0)
        ly = _clamp(cmd.linear.x * self.wireless_linear_scale, 1.0)
        rx = _clamp(cmd.angular.z * self.wireless_angular_scale, 1.0)
        self.last_go2_command = {"mode": "wireless_controller", "lx": lx, "ly": ly, "rx": rx}
        self.conn.datachannel.pub_sub.publish_without_callback(
            RTC_TOPIC["WIRELESS_CONTROLLER"],
            {"lx": lx, "ly": ly, "rx": rx, "ry": 0.0, "keys": 0},
        )

    def _send_obstacles_avoid_move(self, cmd: Twist) -> None:
        from unitree_webrtc_connect.constants import OBSTACLES_AVOID_API, RTC_TOPIC

        parameter = {
            "x": cmd.linear.x,
            "y": cmd.linear.y,
            "yaw": cmd.angular.z,
            "mode": 0,
        }
        self.last_go2_command = {"mode": "obstacles_avoid_move", **parameter}
        request_payload = {
            "header": {
                "identity": {
                    "id": int(time.time() * 1000) % 2147483648,
                    "api_id": OBSTACLES_AVOID_API["MOVE"],
                },
                "policy": {"priority": 0, "noreply": True},
            },
            "parameter": json.dumps(parameter),
            "binary": [],
        }
        self.conn.datachannel.pub_sub.publish_without_callback(
            RTC_TOPIC["OBSTACLES_AVOID"],
            request_payload,
        )

    async def _send_stand(self) -> None:
        from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD_MCF

        api_id = SPORT_CMD_MCF.get(self.stand_command)
        if api_id is None:
            raise RuntimeError(f"unsupported Go2 stand_command: {self.stand_command}")
        self.get_logger().info(f"Sending Go2 stand command: {self.stand_command}")
        response = await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": api_id},
        )
        code = response.get("data", {}).get("header", {}).get("status", {}).get("code")
        self.get_logger().info(f"Go2 stand command response code: {code}")
        await asyncio.sleep(max(0.0, self.stand_wait_sec))

    async def _send_continuous_gait(self, enabled: bool) -> None:
        from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD_MCF

        value = self.continuous_gait_value if enabled else 0
        self.get_logger().info(f"Setting Go2 ContinuousGait: {value}")
        response = await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD_MCF["ContinuousGait"], "parameter": {"data": value}},
        )
        code = response.get("data", {}).get("header", {}).get("status", {}).get("code")
        self.get_logger().info(f"Go2 ContinuousGait response code: {code}")

    async def _send_speed_level(self) -> None:
        from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD_MCF

        self.get_logger().info(f"Setting Go2 SpeedLevel: {self.speed_level}")
        response = await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD_MCF["SpeedLevel"], "parameter": {"data": self.speed_level}},
        )
        code = response.get("data", {}).get("header", {}).get("status", {}).get("code")
        self.get_logger().info(f"Go2 SpeedLevel response code: {code}")

    async def _send_stop(self) -> None:
        if self.conn is None:
            return
        try:
            from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"],
                {"api_id": SPORT_CMD["StopMove"]},
            )
        except Exception as exc:
            with self.lock:
                self.last_status = "error"
                self.last_status_message = f"stop failed: {type(exc).__name__}: {exc}"

    async def _send_soft_stop(self) -> None:
        if self.conn is None:
            return
        if self.soft_stop_mode == "stop_move":
            await self._send_stop()
            return
        await self._send_move(Twist())

    def _schedule_soft_stop(self) -> None:
        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self._send_soft_stop(), self.loop)

    def _schedule_hard_stop(self) -> None:
        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self._send_stop(), self.loop)

    def _publish_status(self) -> None:
        with self.lock:
            payload = {
                "adapter": "go2_webrtc_bridge",
                "status": self.last_status,
                "message": self.last_status_message,
                "connected": self.connected,
                "robot_ip": str(self.get_parameter("robot_ip").value),
                "move_command_mode": self.move_command_mode,
                "last_go2_command": self.last_go2_command,
                "timestamp": self.get_clock().now().nanoseconds / 1e9,
            }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def destroy_node(self) -> bool:
        self.shutdown_requested = True
        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self._send_stop(), self.loop)
        self.thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2WebRTCBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
