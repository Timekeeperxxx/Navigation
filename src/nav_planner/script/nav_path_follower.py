#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class Point3:
    x: float
    y: float
    z: float


@dataclass
class RobotPose:
    position: Point3
    yaw: float


class NavPathFollower(Node):
    """按局部路径生成保守速度命令，用于无底盘或轻量控制链路测试。"""

    def __init__(self) -> None:
        super().__init__("nav_path_follower")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("path_topic", "/nav/local_path")
        self.declare_parameter("fallback_path_topic", "/global_path")
        self.declare_parameter("obstacle_status_topic", "/nav/obstacle_status")
        self.declare_parameter("nav_start_topic", "/nav_start")
        self.declare_parameter("nav_stop_topic", "/nav_stop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("nav_status_topic", "/nav_status")
        self.declare_parameter("require_nav_start", True)
        self.declare_parameter("control_period_sec", 0.1)
        self.declare_parameter("path_timeout_sec", 2.0)
        self.declare_parameter("status_timeout_sec", 1.0)
        self.declare_parameter("lookahead_distance", 0.8)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("path_deviation_tolerance", 1.2)
        self.declare_parameter("linear_slow_radius", 1.0)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 0.7)
        self.declare_parameter("yaw_gain", 1.5)
        self.declare_parameter("heading_only_angle", 0.9)
        self.declare_parameter("caution_speed_scale", 0.35)
        self.declare_parameter("publish_zero_when_idle", True)

        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.require_nav_start = bool(self.get_parameter("require_nav_start").value)
        self.path_timeout_sec = float(self.get_parameter("path_timeout_sec").value)
        self.status_timeout_sec = float(self.get_parameter("status_timeout_sec").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.path_deviation_tolerance = float(
            self.get_parameter("path_deviation_tolerance").value
        )
        self.linear_slow_radius = float(self.get_parameter("linear_slow_radius").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.heading_only_angle = float(self.get_parameter("heading_only_angle").value)
        self.caution_speed_scale = float(self.get_parameter("caution_speed_scale").value)
        self.publish_zero_when_idle = bool(
            self.get_parameter("publish_zero_when_idle").value
        )

        self.active = not self.require_nav_start
        self.path_points: list[Point3] = []
        self.fallback_path_points: list[Point3] = []
        self.last_path_time: Optional[float] = None
        self.last_fallback_path_time: Optional[float] = None
        self.last_obstacle_status_time: Optional[float] = None
        self.obstacle_status = "unknown"
        self.obstacle_action = "none"
        self.last_nav_status = "idle"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.nav_status_pub = self.create_publisher(
            String, str(self.get_parameter("nav_status_topic").value), 10
        )

        self.create_subscription(
            Path,
            str(self.get_parameter("path_topic").value),
            self._on_local_path,
            10,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("fallback_path_topic").value),
            self._on_fallback_path,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("obstacle_status_topic").value),
            self._on_obstacle_status,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("nav_start_topic").value),
            self._on_nav_start,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("nav_stop_topic").value),
            self._on_nav_stop,
            10,
        )

        period = max(float(self.get_parameter("control_period_sec").value), 0.05)
        self.create_timer(period, self._control_loop)
        self.get_logger().info(
            "保守路径跟随器已启动："
            f"path={self.get_parameter('path_topic').value}, "
            f"cmd_vel={self.get_parameter('cmd_vel_topic').value}, "
            f"require_nav_start={self.require_nav_start}"
        )

    def _on_local_path(self, msg: Path) -> None:
        points = self._points_from_path(msg)
        if points:
            self.path_points = points
            self.last_path_time = self._now_sec()

    def _on_fallback_path(self, msg: Path) -> None:
        points = self._points_from_path(msg)
        if points:
            self.fallback_path_points = points
            self.last_fallback_path_time = self._now_sec()

    def _on_obstacle_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("收到无法解析的 /nav/obstacle_status")
            return
        self.obstacle_status = str(payload.get("status", "unknown"))
        self.obstacle_action = str(payload.get("action", "none"))
        self.last_obstacle_status_time = self._now_sec()

    def _on_nav_start(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.active = True
        self._publish_nav_status("accepted", "收到导航启动信号")

    def _on_nav_stop(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.active = False
        self._publish_zero()
        self._publish_nav_status("estop", "收到导航停止信号")

    def _control_loop(self) -> None:
        now = self._now_sec()
        if not self.active:
            if self.publish_zero_when_idle:
                self._publish_zero()
            self._publish_nav_status(self.last_nav_status, "等待 /nav_start")
            return

        if self._obstacle_requires_stop(now):
            self._publish_zero()
            self._publish_nav_status("estop", "动态避障要求停车")
            return

        path = self._active_path(now)
        if not path:
            self._publish_zero()
            self._publish_nav_status("failed", "没有可用路径")
            return

        robot, error = self._get_robot_pose()
        if robot is None:
            self._publish_zero()
            self._publish_nav_status("failed", f"无法获取机器人位姿：{error}")
            return

        final_distance = self._distance_2d(robot.position, path[-1])
        if final_distance <= self.goal_tolerance:
            self.active = not self.require_nav_start
            self._publish_zero()
            self._publish_nav_status("reached", "已到达路径终点")
            return

        nearest_index, nearest_distance = self._nearest_path_index(robot.position, path)
        if nearest_distance > self.path_deviation_tolerance:
            self._publish_zero()
            self._publish_nav_status("failed", "机器人偏离路径过远")
            return

        target = self._lookahead_target(nearest_index, path)
        cmd_vel = self._compute_cmd_vel(robot, target, final_distance)
        if self._obstacle_requires_slow(now):
            cmd_vel.linear.x *= self.caution_speed_scale
            cmd_vel.angular.z *= self.caution_speed_scale
        self.cmd_vel_pub.publish(cmd_vel)
        self._publish_nav_status(
            "moving",
            "路径跟随中",
            {
                "distance_to_goal": final_distance,
                "target": {"x": target.x, "y": target.y, "z": target.z},
                "nearest_path_distance": nearest_distance,
            },
        )

    def _points_from_path(self, msg: Path) -> list[Point3]:
        points: list[Point3] = []
        for pose in msg.poses:
            points.append(
                Point3(
                    float(pose.pose.position.x),
                    float(pose.pose.position.y),
                    float(pose.pose.position.z),
                )
            )
        return points

    def _active_path(self, now: float) -> list[Point3]:
        if (
            self.path_points
            and self.last_path_time is not None
            and now - self.last_path_time <= self.path_timeout_sec
        ):
            return self.path_points
        if (
            self.fallback_path_points
            and self.last_fallback_path_time is not None
            and now - self.last_fallback_path_time <= self.path_timeout_sec
        ):
            return self.fallback_path_points
        return []

    def _get_robot_pose(self) -> tuple[Optional[RobotPose], str]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            return None, str(exc)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = self._yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return RobotPose(Point3(translation.x, translation.y, translation.z), yaw), ""

    def _nearest_path_index(self, robot: Point3, path: list[Point3]) -> tuple[int, float]:
        index = min(range(len(path)), key=lambda idx: self._distance_2d(robot, path[idx]))
        return index, self._distance_2d(robot, path[index])

    def _lookahead_target(self, nearest_index: int, path: list[Point3]) -> Point3:
        distance = 0.0
        last = path[nearest_index]
        for index in range(nearest_index + 1, len(path)):
            current = path[index]
            distance += self._distance_2d(last, current)
            if distance >= self.lookahead_distance:
                return current
            last = current
        return path[-1]

    def _compute_cmd_vel(
        self, robot: RobotPose, target: Point3, final_distance: float
    ) -> Twist:
        dx = target.x - robot.position.x
        dy = target.y - robot.position.y
        local_x = math.cos(robot.yaw) * dx + math.sin(robot.yaw) * dy
        local_y = -math.sin(robot.yaw) * dx + math.cos(robot.yaw) * dy
        heading_error = math.atan2(local_y, local_x)

        cmd_vel = Twist()
        angular = self._clamp(self.yaw_gain * heading_error, -self.max_angular_speed, self.max_angular_speed)
        if abs(heading_error) >= self.heading_only_angle:
            linear = 0.0
        else:
            speed_scale = min(1.0, max(0.0, final_distance / self.linear_slow_radius))
            heading_scale = max(0.0, math.cos(heading_error))
            linear = self.max_linear_speed * speed_scale * heading_scale
        cmd_vel.linear.x = self._clamp(linear, -self.max_linear_speed, self.max_linear_speed)
        cmd_vel.angular.z = angular
        return cmd_vel

    def _obstacle_requires_stop(self, now: float) -> bool:
        if self._status_stale(now):
            return False
        return self.obstacle_action == "stop" or self.obstacle_status in {
            "blocked",
            "replan_requested",
            "sensor_lost",
            "deviated",
        }

    def _obstacle_requires_slow(self, now: float) -> bool:
        if self._status_stale(now):
            return False
        return self.obstacle_action == "slow" or self.obstacle_status == "caution"

    def _status_stale(self, now: float) -> bool:
        if self.last_obstacle_status_time is None:
            return True
        return now - self.last_obstacle_status_time > self.status_timeout_sec

    def _publish_zero(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _publish_nav_status(
        self, status: str, message: str, extra: Optional[dict[str, object]] = None
    ) -> None:
        self.last_nav_status = status
        payload: dict[str, object] = {
            "status": status,
            "message": message,
            "timestamp": self._now_sec(),
            "active": self.active,
        }
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.nav_status_pub.publish(msg)

    def _yaw_from_quaternion(self, qx: float, qy: float, qz: float, qw: float) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def _distance_2d(self, a: Point3, b: Point3) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return min(upper, max(lower, value))

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = NavPathFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
