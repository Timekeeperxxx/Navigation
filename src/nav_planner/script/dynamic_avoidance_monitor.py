#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from sensor_msgs_py import point_cloud2
except ImportError:  # pragma: no cover - 兼容极简 ROS 安装环境
    point_cloud2 = None


@dataclass
class Point3:
    x: float
    y: float
    z: float


@dataclass
class PathWindow:
    prune_points: list[Point3]
    forward_points: list[Point3]
    nearest_index: int
    robot_to_path_distance: float


@dataclass
class ObstacleCheck:
    path_blocked: bool
    blocker_count: int
    self_filtered_count: int
    nearest_obstacle_distance: Optional[float]
    nearest_blocker_distance: Optional[float]
    nearest_path_distance: Optional[float]
    clearance_escape: bool = False


class DynamicAvoidanceMonitor(Node):
    """裁剪全局路径，检测局部障碍，并输出安全状态和速度过滤结果。"""

    def __init__(self) -> None:
        super().__init__("dynamic_avoidance_monitor")

        self.declare_parameter("enabled", True)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("global_path_topic", "/global_path")
        self.declare_parameter("scan_execution_path_topic", "/scan/execution_path")
        self.declare_parameter("scan_execution_path_timeout", 0.5)
        self.declare_parameter("scan_execution_path_global_tolerance", 1.0)
        self.declare_parameter("obstacle_topic", "/nav/local_obstacles")
        self.declare_parameter("sensor_heartbeat_topic", "")
        self.declare_parameter("status_topic", "/nav/obstacle_status")
        self.declare_parameter("replan_request_topic", "/nav/replan_request")
        self.declare_parameter("prune_plan_topic", "/prune_plan")
        self.declare_parameter("local_path_topic", "/nav/local_path")
        self.declare_parameter("cmd_vel_in_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_safe_topic", "/cmd_vel_safe")
        self.declare_parameter(
            "safety_execution_frozen_topic", "/planning/safety_execution_frozen"
        )
        self.declare_parameter("nav_start_topic", "/nav_start")
        self.declare_parameter("nav_stop_topic", "/nav_stop")
        self.declare_parameter("lookahead_distance", 2.0)
        self.declare_parameter("backward_prune_distance", 0.5)
        self.declare_parameter("path_deviation_tolerance", 1.0)
        # /grid_map/occupancy_inflate is already expanded for the B2 footprint.
        # Keep only a half-diagonal voxel matching tolerance here; a body-sized
        # corridor would inflate the same obstacle twice.
        self.declare_parameter("path_corridor_radius", 0.04)
        # The inflated occupancy grid represents one collision circle.  Check
        # that grid along both longitudinal circle-centre sweeps, matching the
        # footprint used by SCAN instead of checking only the body origin.
        self.declare_parameter("footprint_center_offsets", [-0.22, -0.63])
        self.declare_parameter("footprint_sweep_max_step", 0.05)
        self.declare_parameter(
            "footprint_sweep_max_yaw_step", math.radians(5.0)
        )
        # The execution path is lifted to the body planning height.  A broad
        # band BELOW the path includes traversable floor voxels and permanently
        # freezes otherwise collision-free trajectories, so the downward
        # tolerance stays tight.
        self.declare_parameter("z_tolerance", 0.10)
        # Upward the band must cover the full machine: B2 back plus the Mid360
        # mast tops out near 0.97 m while the path sits at the 0.30 m body
        # slice.  Occupancy voxels are already inflated 0.33 m downwards, so
        # 0.45 m above the path catches any physical overhang whose lowest
        # point is below about 1.08 m; higher clearance passes underneath.
        self.declare_parameter("z_tolerance_up", 0.45)
        self.declare_parameter("stop_distance", 0.6)
        self.declare_parameter("slow_distance", 1.2)
        # The grid map now removes body returns with a pose-aware mask.  Keep
        # only a small discretization margin here so close walls are not
        # mistaken for the robot itself.
        self.declare_parameter("robot_self_clear_radius", 0.35)
        self.declare_parameter("replan_blocked_duration", 2.0)
        self.declare_parameter("blocked_clear_duration", 2.0)
        self.declare_parameter("transient_clear_duration", 0.4)
        self.declare_parameter("allow_clearance_escape", True)
        self.declare_parameter("clearance_escape_max_distance", 0.6)
        self.declare_parameter("clearance_escape_max_drop", 0.015)
        self.declare_parameter("clearance_escape_min_improvement", 0.02)
        self.declare_parameter("clearance_escape_speed_scale", 0.60)
        self.declare_parameter("sensor_timeout", 1.5)
        self.declare_parameter("check_period_sec", 0.2)
        self.declare_parameter("obstacle_processing_period_sec", 0.2)
        self.declare_parameter("max_obstacle_points", 8000)
        self.declare_parameter("slow_speed_scale", 0.35)
        self.declare_parameter("slow_nearby_obstacles", True)
        self.declare_parameter("require_obstacle_stream", False)
        self.declare_parameter("allow_path_start_without_tf", False)
        self.declare_parameter("enable_cmd_vel_filter", True)
        self.declare_parameter("enforce_path_blocking", True)
        self.declare_parameter("require_nav_start", True)
        self.declare_parameter("publish_zero_on_stop", True)
        self.declare_parameter("publish_prune_plan", True)
        self.declare_parameter("replan_publish_period", 1.0)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.scan_execution_path_timeout = max(
            float(self.get_parameter("scan_execution_path_timeout").value), 0.1
        )
        self.scan_execution_path_global_tolerance = max(
            float(self.get_parameter("scan_execution_path_global_tolerance").value),
            0.1,
        )
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.backward_prune_distance = float(
            self.get_parameter("backward_prune_distance").value
        )
        self.path_deviation_tolerance = float(
            self.get_parameter("path_deviation_tolerance").value
        )
        self.path_corridor_radius = float(
            self.get_parameter("path_corridor_radius").value
        )
        configured_footprint_offsets = tuple(
            float(value)
            for value in self.get_parameter("footprint_center_offsets").value
        )
        self.footprint_center_offsets = (
            configured_footprint_offsets
            if configured_footprint_offsets
            else (-0.22, -0.63)
        )
        self.footprint_sweep_max_step = max(
            float(self.get_parameter("footprint_sweep_max_step").value), 0.01
        )
        self.footprint_sweep_max_yaw_step = max(
            float(self.get_parameter("footprint_sweep_max_yaw_step").value),
            math.radians(1.0),
        )
        self.z_tolerance = float(self.get_parameter("z_tolerance").value)
        self.z_tolerance_up = max(
            float(self.get_parameter("z_tolerance_up").value), self.z_tolerance
        )
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.slow_distance = float(self.get_parameter("slow_distance").value)
        self.robot_self_clear_radius = max(
            float(self.get_parameter("robot_self_clear_radius").value), 0.0
        )
        self.replan_blocked_duration = float(
            self.get_parameter("replan_blocked_duration").value
        )
        self.blocked_clear_duration = max(
            float(self.get_parameter("blocked_clear_duration").value), 0.0
        )
        self.transient_clear_duration = max(
            float(self.get_parameter("transient_clear_duration").value), 0.0
        )
        self.allow_clearance_escape = bool(
            self.get_parameter("allow_clearance_escape").value
        )
        self.clearance_escape_max_distance = max(
            float(self.get_parameter("clearance_escape_max_distance").value),
            0.0,
        )
        self.clearance_escape_max_drop = max(
            float(self.get_parameter("clearance_escape_max_drop").value), 0.0
        )
        self.clearance_escape_min_improvement = max(
            float(
                self.get_parameter("clearance_escape_min_improvement").value
            ),
            0.0,
        )
        self.clearance_escape_speed_scale = min(
            1.0,
            max(
                0.0,
                float(
                    self.get_parameter("clearance_escape_speed_scale").value
                ),
            ),
        )
        self.sensor_timeout = float(self.get_parameter("sensor_timeout").value)
        self.obstacle_processing_period = max(
            float(self.get_parameter("obstacle_processing_period_sec").value), 0.05
        )
        self.max_obstacle_points = max(
            int(self.get_parameter("max_obstacle_points").value), 1
        )
        self.slow_speed_scale = float(self.get_parameter("slow_speed_scale").value)
        self.slow_nearby_obstacles = bool(
            self.get_parameter("slow_nearby_obstacles").value
        )
        self.require_obstacle_stream = bool(
            self.get_parameter("require_obstacle_stream").value
        )
        self.sensor_heartbeat_topic = str(
            self.get_parameter("sensor_heartbeat_topic").value
        ).strip()
        self.allow_path_start_without_tf = bool(
            self.get_parameter("allow_path_start_without_tf").value
        )
        self.enable_cmd_vel_filter = bool(
            self.get_parameter("enable_cmd_vel_filter").value
        )
        self.enforce_path_blocking = bool(
            self.get_parameter("enforce_path_blocking").value
        )
        self.require_nav_start = bool(self.get_parameter("require_nav_start").value)
        self.publish_zero_on_stop = bool(
            self.get_parameter("publish_zero_on_stop").value
        )
        self.publish_prune_plan = bool(self.get_parameter("publish_prune_plan").value)
        self.replan_publish_period = float(
            self.get_parameter("replan_publish_period").value
        )

        self.path_points: list[Point3] = []
        self.path_frame = self.global_frame
        self.execution_path_points: list[Point3] = []
        self.last_execution_path_time: Optional[float] = None
        self.execution_path_generation_time: Optional[float] = None
        self.last_global_path_time: Optional[float] = None
        self.active_path_points: list[Point3] = []
        self.active_path_source = "none"
        self.execution_path_age: Optional[float] = None
        self.execution_path_generation_matches = False
        self.execution_path_spatially_matches = False
        self.obstacles: list[Point3] = []
        self.last_obstacle_time: Optional[float] = None
        self.last_sensor_heartbeat_time: Optional[float] = None
        self.last_obstacle_processing_time: Optional[float] = None
        self.blocked_since: Optional[float] = None
        self.clear_since: Optional[float] = None
        self.blocked_execution_generation_time: Optional[float] = None
        self.replan_confirmation_required = False
        self.last_replan_publish_time = 0.0
        self.replan_active = False
        self.current_status = "idle"
        self.current_action = "none"
        self.current_path_blocked = False
        self.navigation_enabled = not self.require_nav_start
        self.robot_yaw = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.replan_pub = self.create_publisher(
            Bool, str(self.get_parameter("replan_request_topic").value), 10
        )
        self.prune_plan_pub = self.create_publisher(
            Path, str(self.get_parameter("prune_plan_topic").value), 10
        )
        self.local_path_pub = self.create_publisher(
            Path, str(self.get_parameter("local_path_topic").value), 10
        )
        self.cmd_vel_safe_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_safe_topic").value), 10
        )
        self.execution_frozen_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("safety_execution_frozen_topic").value),
            10,
        )

        self.create_subscription(
            Path,
            str(self.get_parameter("global_path_topic").value),
            self._on_global_path,
            10,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("scan_execution_path_topic").value),
            self._on_execution_path,
            10,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("obstacle_topic").value),
            self._on_obstacles,
            1,
        )
        if self.sensor_heartbeat_topic:
            self.create_subscription(
                PointCloud2,
                self.sensor_heartbeat_topic,
                self._on_sensor_heartbeat,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_in_topic").value),
            self._on_cmd_vel,
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

        period = max(float(self.get_parameter("check_period_sec").value), 0.05)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(
            "动态避障监测已启动："
            f"path={self.get_parameter('global_path_topic').value}, "
            f"execution_path={self.get_parameter('scan_execution_path_topic').value}, "
            f"obstacles={self.get_parameter('obstacle_topic').value}, "
            f"sensor_heartbeat={self.sensor_heartbeat_topic or 'disabled'}, "
            f"require_obstacle_stream={self.require_obstacle_stream}, "
            f"obstacle_period={self.obstacle_processing_period:.2f}s, "
            f"max_obstacle_points={self.max_obstacle_points}"
        )

    def _on_global_path(self, msg: Path) -> None:
        points = self._points_from_path(msg)
        self.path_points = points
        self.path_frame = self.global_frame
        self.last_global_path_time = self._now_sec()
        if not points:
            self.get_logger().warn("收到空的全局路径，动态避障进入 no_path 状态")

    def _on_execution_path(self, msg: Path) -> None:
        self.execution_path_points = self._points_from_path(msg)
        self.last_execution_path_time = self._now_sec()
        stamp = msg.header.stamp
        generation_time = float(stamp.sec) + float(stamp.nanosec) / 1e9
        self.execution_path_generation_time = (
            generation_time if generation_time > 0.0 else self.last_execution_path_time
        )

    def _points_from_path(self, msg: Path) -> list[Point3]:
        frame_id = msg.header.frame_id or self.global_frame
        points: list[Point3] = []
        for pose in msg.poses:
            pose_frame = pose.header.frame_id or frame_id
            point = Point3(
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            transformed = self._transform_point(point, pose_frame)
            if transformed is not None:
                points.append(transformed)
        return points

    def _on_obstacles(self, msg: PointCloud2) -> None:
        now = self._now_sec()
        self.last_obstacle_time = now
        if not self.navigation_enabled:
            self.obstacles = []
            return
        if (
            self.last_obstacle_processing_time is not None
            and now - self.last_obstacle_processing_time
            < self.obstacle_processing_period
        ):
            return
        self.last_obstacle_processing_time = now

        points = self._read_cloud_points(msg)
        transformed: list[Point3] = []
        frame_id = msg.header.frame_id or self.global_frame
        for point in points:
            global_point = self._transform_point(point, frame_id)
            if global_point is not None:
                transformed.append(global_point)
        self.obstacles = transformed

    def _on_sensor_heartbeat(self, _msg: PointCloud2) -> None:
        # Use receipt time instead of the message stamp: a driver or bag can
        # legitimately use a different clock, while safety only needs to know
        # whether fresh fused LiDAR output is still arriving.
        self.last_sensor_heartbeat_time = self._now_sec()

    def _on_cmd_vel(self, msg: Twist) -> None:
        if not self.enable_cmd_vel_filter:
            return
        if not self.navigation_enabled:
            self.cmd_vel_safe_pub.publish(Twist())
            return
        self.cmd_vel_safe_pub.publish(self._filter_twist(msg))

    def _on_nav_start(self, msg: Bool) -> None:
        self.navigation_enabled = bool(msg.data)
        if not self.navigation_enabled:
            self.cmd_vel_safe_pub.publish(Twist())

    def _on_nav_stop(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.navigation_enabled = False
        self.cmd_vel_safe_pub.publish(Twist())

    def _on_timer(self) -> None:
        now = self._now_sec()
        self.active_path_points, self.active_path_source = self._active_path(now)
        if not self.navigation_enabled:
            self.cmd_vel_safe_pub.publish(Twist())
        robot_point, robot_error = self._get_robot_point()
        status_payload: dict[str, object]

        if not self.enabled:
            status_payload = self._status_payload(
                "disabled", "pass", False, now, message="动态避障已关闭"
            )
            self._set_state(status_payload)
            self._publish_status(status_payload)
            return

        if not self.active_path_points:
            self._reset_block_tracking()
            status_payload = self._status_payload(
                "no_path", "idle", False, now, message="尚未收到可用导航路径"
            )
            self._set_state(status_payload)
            self._publish_status(status_payload)
            return

        if robot_point is None:
            self._reset_block_tracking()
            status_payload = self._status_payload(
                "waiting_tf",
                "stop",
                False,
                now,
                message=f"无法获取机器人位姿：{robot_error}",
            )
            self._set_state(status_payload)
            self._publish_status(status_payload)
            self._publish_stop_if_needed()
            return

        window = self._make_path_window(robot_point, self.active_path_points)
        if window is None:
            self._reset_block_tracking()
            status_payload = self._status_payload(
                "deviated",
                "stop",
                False,
                now,
                robot=robot_point,
                message=f"机器人偏离{self.active_path_source}过远，停止输出安全速度",
            )
            self._set_state(status_payload)
            self._publish_status(status_payload)
            self._publish_stop_if_needed()
            return

        self._publish_path_window(window)

        if self._obstacle_stream_lost(now):
            self._reset_block_tracking()
            status_payload = self._status_payload(
                "sensor_lost",
                "stop",
                False,
                now,
                robot=robot_point,
                prune_plan_size=len(window.prune_points),
                message="原始传感器或局部障碍点云超时",
            )
            self._set_state(status_payload)
            self._publish_status(status_payload)
            self._publish_stop_if_needed()
            return

        check = self._check_obstacles(robot_point, window.forward_points)
        status_payload = self._decide_status(now, robot_point, window, check)
        self._set_state(status_payload)
        self._publish_status(status_payload)
        self._publish_stop_if_needed()

    def _active_path(self, now: float) -> tuple[list[Point3], str]:
        self.execution_path_age = (
            None
            if self.last_execution_path_time is None
            else max(0.0, now - self.last_execution_path_time)
        )
        execution_fresh = (
            bool(self.execution_path_points)
            and self.execution_path_age is not None
            and self.execution_path_age <= self.scan_execution_path_timeout
        )
        # A controller from the previous goal may still be alive briefly.  Its
        # B-spline keeps its original generation stamp even though the remaining
        # path is republished, so do not let it override a newer global goal.
        self.execution_path_generation_matches = (
            self.last_global_path_time is None
            or self.execution_path_generation_time is None
            or self.execution_path_generation_time >= self.last_global_path_time - 0.2
        )
        self.execution_path_spatially_matches = self._execution_path_matches_global()
        if execution_fresh and (
            self.execution_path_generation_matches
            or self.execution_path_spatially_matches
        ):
            return self.execution_path_points, "scan_execution_path"
        if self.path_points:
            return self.path_points, "global_path"
        return [], "none"

    def _execution_path_matches_global(self) -> bool:
        if not self.execution_path_points:
            return False
        if not self.path_points:
            return True

        tolerance = self.scan_execution_path_global_tolerance
        execution_start = self.execution_path_points[0]
        execution_end = self.execution_path_points[-1]
        if self._distance_3d(execution_start, self.path_points[0]) > tolerance:
            return False
        if min(
            self._distance_3d(execution_end, point) for point in self.path_points
        ) > tolerance:
            return False

        global_span = self._distance_3d(self.path_points[0], self.path_points[-1])
        execution_length = sum(
            self._distance_3d(start, end)
            for start, end in zip(
                self.execution_path_points[:-1], self.execution_path_points[1:]
            )
        )
        # A completed one-point trajectory from the previous goal often sits at
        # the new global path start.  It is spatially close but must not override
        # a new long navigation request.
        if global_span > 1.0 and execution_length < 0.3:
            return False
        return True

    def _read_cloud_points(self, msg: PointCloud2) -> list[Point3]:
        if point_cloud2 is None:
            self.get_logger().error("缺少 sensor_msgs_py，无法读取 PointCloud2")
            return []
        raw_points = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        point_count = len(raw_points)
        if point_count > self.max_obstacle_points:
            stride = math.ceil(point_count / self.max_obstacle_points)
            raw_points = raw_points[::stride]
        return [
            Point3(float(point[0]), float(point[1]), float(point[2]))
            for point in raw_points
        ]

    def _transform_point(self, point: Point3, source_frame: str) -> Optional[Point3]:
        if not source_frame or source_frame == self.global_frame:
            return point
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"点云坐标转换失败：{source_frame} -> {self.global_frame}: {exc}"
            )
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rotated = self._rotate_point(point, rotation.x, rotation.y, rotation.z, rotation.w)
        return Point3(
            rotated.x + float(translation.x),
            rotated.y + float(translation.y),
            rotated.z + float(translation.z),
        )

    def _rotate_point(self, point: Point3, qx: float, qy: float, qz: float, qw: float) -> Point3:
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-9:
            return point
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        tx = 2.0 * (qy * point.z - qz * point.y)
        ty = 2.0 * (qz * point.x - qx * point.z)
        tz = 2.0 * (qx * point.y - qy * point.x)
        return Point3(
            point.x + qw * tx + (qy * tz - qz * ty),
            point.y + qw * ty + (qz * tx - qx * tz),
            point.z + qw * tz + (qx * ty - qy * tx),
        )

    def _get_robot_point(self) -> tuple[Optional[Point3], str]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            t = transform.transform.translation
            q = transform.transform.rotation
            self.robot_yaw = math.atan2(
                2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
                1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2),
            )
            return Point3(float(t.x), float(t.y), float(t.z)), ""
        except TransformException as exc:
            if self.allow_path_start_without_tf and self.path_points:
                if len(self.path_points) >= 2:
                    start = self.path_points[0]
                    end = self.path_points[1]
                    self.robot_yaw = math.atan2(end.y - start.y, end.x - start.x)
                return self.path_points[0], "使用路径起点作为无 TF 离线兜底"
            return None, str(exc)

    def _make_path_window(
        self, robot: Point3, path_points: list[Point3]
    ) -> Optional[PathWindow]:
        nearest_index = min(
            range(len(path_points)),
            key=lambda index: self._distance_3d(robot, path_points[index]),
        )
        robot_distance = self._distance_3d(robot, path_points[nearest_index])
        if robot_distance > self.path_deviation_tolerance:
            return None

        backward = self._collect_backward(nearest_index, path_points)
        forward = self._collect_forward(nearest_index, path_points)
        prune_points = list(reversed(backward))
        if prune_points and forward and self._same_point(prune_points[-1], forward[0]):
            prune_points.extend(forward[1:])
        else:
            prune_points.extend(forward)
        return PathWindow(prune_points, forward, nearest_index, robot_distance)

    def _collect_backward(
        self, nearest_index: int, path_points: list[Point3]
    ) -> list[Point3]:
        points = [path_points[nearest_index]]
        remaining = self.backward_prune_distance
        last = path_points[nearest_index]
        for index in range(nearest_index - 1, -1, -1):
            current = path_points[index]
            remaining -= self._distance_3d(last, current)
            points.append(current)
            last = current
            if remaining <= 0.0:
                break
        return points

    def _collect_forward(
        self, nearest_index: int, path_points: list[Point3]
    ) -> list[Point3]:
        points = [path_points[nearest_index]]
        remaining = self.lookahead_distance
        last = path_points[nearest_index]
        for index in range(nearest_index + 1, len(path_points)):
            current = path_points[index]
            remaining -= self._distance_3d(last, current)
            points.append(current)
            last = current
            if remaining <= 0.0:
                break
        return points

    def _publish_path_window(self, window: PathWindow) -> None:
        if not self.publish_prune_plan:
            return
        self.prune_plan_pub.publish(self._path_from_points(window.prune_points))
        self.local_path_pub.publish(self._path_from_points(window.forward_points))

    def _path_from_points(self, points: Iterable[Point3]) -> Path:
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for point in points:
            pose = path.poses.add() if hasattr(path.poses, "add") else None
            if pose is None:
                from geometry_msgs.msg import PoseStamped

                pose = PoseStamped()
                path.poses.append(pose)
            pose.header = path.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z
            pose.pose.orientation.w = 1.0
        return path

    def _obstacle_stream_lost(self, now: float) -> bool:
        if not self.require_obstacle_stream:
            return False
        stream_time = (
            self.last_sensor_heartbeat_time
            if self.sensor_heartbeat_topic
            else self.last_obstacle_time
        )
        if stream_time is None:
            return True
        return now - stream_time > self.sensor_timeout

    def _check_obstacles(self, robot: Point3, forward_path: list[Point3]) -> ObstacleCheck:
        body_path = self._path_with_robot_start(robot, forward_path)
        footprint_paths = self._footprint_sweep_paths(body_path, self.robot_yaw)
        footprint_reach = max(
            (abs(offset) for offset in self.footprint_center_offsets),
            default=0.0,
        )
        path_check_radius = (
            self.path_deviation_tolerance
            + self.lookahead_distance
            + footprint_reach
            + self.path_corridor_radius
        )
        nearby_check_radius = (
            self.slow_distance if self.slow_nearby_obstacles else 0.0
        )
        obstacle_check_radius = max(path_check_radius, nearby_check_radius)

        if not self.obstacles or not footprint_paths:
            return ObstacleCheck(False, 0, 0, None, None, None)

        obstacle_array = np.asarray(
            [(point.x, point.y, point.z) for point in self.obstacles],
            dtype=np.float64,
        )
        robot_distances = np.hypot(
            obstacle_array[:, 0] - robot.x,
            obstacle_array[:, 1] - robot.y,
        )
        self_mask = robot_distances <= self.robot_self_clear_radius
        self_filtered_count = int(np.count_nonzero(self_mask))
        candidate_mask = (~self_mask) & (robot_distances <= obstacle_check_radius)
        if not np.any(candidate_mask):
            return ObstacleCheck(
                False, 0, self_filtered_count, None, None, None
            )

        candidates = obstacle_array[candidate_mask]
        candidate_robot_distances = robot_distances[candidate_mask]
        nearest_obstacle_distance = float(np.min(candidate_robot_distances))
        path_distances = np.full(len(candidates), math.inf, dtype=np.float64)
        z_distances = np.full(len(candidates), math.inf, dtype=np.float64)
        for footprint_path in footprint_paths:
            candidate_path_distances, candidate_z_distances = (
                self._distances_to_path(candidates, footprint_path)
            )
            closer = candidate_path_distances < path_distances
            path_distances[closer] = candidate_path_distances[closer]
            z_distances[closer] = candidate_z_distances[closer]

        nearest_path_distance = float(np.min(path_distances))
        blocker_mask = (
            (path_distances <= self.path_corridor_radius)
            & (z_distances >= -self.z_tolerance)
            & (z_distances <= self.z_tolerance_up)
        )
        blocker_count = int(np.count_nonzero(blocker_mask))
        nearest_blocker_distance = (
            float(np.min(candidate_robot_distances[blocker_mask]))
            if blocker_count
            else None
        )
        clearance_escape = (
            blocker_count > 0
            and self.allow_clearance_escape
            and self._is_clearance_escape(footprint_paths, candidates)
        )

        return ObstacleCheck(
            path_blocked=blocker_count > 0,
            blocker_count=blocker_count,
            self_filtered_count=self_filtered_count,
            nearest_obstacle_distance=nearest_obstacle_distance,
            nearest_blocker_distance=nearest_blocker_distance,
            nearest_path_distance=nearest_path_distance,
            clearance_escape=clearance_escape,
        )

    def _is_clearance_escape(
        self, footprint_paths: list[list[Point3]], obstacles: np.ndarray
    ) -> bool:
        if not footprint_paths or len(obstacles) == 0:
            return False
        sample_count = min(len(path) for path in footprint_paths)
        if sample_count < 2:
            return False

        joint_clearance = np.full(sample_count, math.inf, dtype=np.float64)
        for path in footprint_paths:
            samples = np.asarray(
                [(point.x, point.y, point.z) for point in path[:sample_count]],
                dtype=np.float64,
            )
            dx = samples[:, None, 0] - obstacles[None, :, 0]
            dy = samples[:, None, 1] - obstacles[None, :, 1]
            distances = np.hypot(dx, dy)
            obstacle_z_offset = (
                obstacles[None, :, 2] - samples[:, None, 2]
            )
            z_valid = (
                (obstacle_z_offset >= -self.z_tolerance)
                & (obstacle_z_offset <= self.z_tolerance_up)
            )
            distances[~z_valid] = math.inf
            joint_clearance = np.minimum(
                joint_clearance, np.min(distances, axis=1)
            )

        # Recovery is only valid when the current footprint is already on the
        # occupied-voxel boundary.  A collision farther along the path remains
        # an ordinary hard stop.
        current_clearance = float(joint_clearance[0])
        if current_clearance > self.path_corridor_radius:
            return False
        blocked_indices = np.flatnonzero(
            joint_clearance <= self.path_corridor_radius
        )
        if len(blocked_indices) == 0:
            return False
        last_blocked_index = int(blocked_indices[-1])
        if last_blocked_index >= sample_count - 1:
            return False

        # Both circle-centre paths share the same pose samples.  Use the larger
        # per-step circle displacement as a conservative recovery arc length.
        travelled = np.zeros(sample_count, dtype=np.float64)
        for index in range(1, sample_count):
            step = max(
                self._distance_3d(path[index - 1], path[index])
                for path in footprint_paths
            )
            travelled[index] = travelled[index - 1] + step
        if travelled[last_blocked_index] > self.clearance_escape_max_distance:
            return False

        escape_section = joint_clearance[: last_blocked_index + 1]
        if float(np.min(escape_section)) < (
            current_clearance - self.clearance_escape_max_drop
        ):
            return False
        clear_section = joint_clearance[last_blocked_index + 1 :]
        return float(np.max(clear_section)) >= (
            self.path_corridor_radius
            + self.clearance_escape_min_improvement
        )

    def _distances_to_path(
        self, points: np.ndarray, path: list[Point3]
    ) -> tuple[np.ndarray, np.ndarray]:
        if not path:
            return (
                np.full(len(points), math.inf, dtype=np.float64),
                np.full(len(points), math.inf, dtype=np.float64),
            )
        if len(path) == 1:
            target = path[0]
            return (
                np.hypot(points[:, 0] - target.x, points[:, 1] - target.y),
                points[:, 2] - target.z,
            )

        best_distance_sq = np.full(len(points), math.inf, dtype=np.float64)
        # Signed offset (point above path is positive) so callers can apply an
        # asymmetric vertical band: tight below (floor voxels), tall above
        # (overhangs at body/lidar height).
        best_z_distance = np.full(len(points), math.inf, dtype=np.float64)
        for start, end in zip(path[:-1], path[1:]):
            vx = end.x - start.x
            vy = end.y - start.y
            vz = end.z - start.z
            length_sq = vx * vx + vy * vy
            if length_sq <= 1e-9:
                ratio = np.zeros(len(points), dtype=np.float64)
            else:
                ratio = (
                    (points[:, 0] - start.x) * vx
                    + (points[:, 1] - start.y) * vy
                ) / length_sq
                ratio = np.clip(ratio, 0.0, 1.0)
            projected_x = start.x + vx * ratio
            projected_y = start.y + vy * ratio
            distance_sq = (
                (points[:, 0] - projected_x) ** 2
                + (points[:, 1] - projected_y) ** 2
            )
            closer = distance_sq < best_distance_sq
            best_distance_sq[closer] = distance_sq[closer]
            projected_z = start.z + vz * ratio
            best_z_distance[closer] = (
                points[closer, 2] - projected_z[closer]
            )
        return np.sqrt(best_distance_sq), best_z_distance

    def _path_with_robot_start(
        self, robot: Point3, forward_path: list[Point3]
    ) -> list[Point3]:
        if not forward_path:
            return [robot]

        # TF generally reports base_footprint at z=0 while SCAN plans at the
        # body slice (about z=0.30 m).  Use the path slice for the live pose so
        # the independent vertical filter checks the same collision volume.
        robot_on_path_slice = Point3(robot.x, robot.y, forward_path[0].z)
        if self._distance_2d(robot_on_path_slice, forward_path[0]) <= 1e-6:
            return [robot_on_path_slice, *forward_path[1:]]
        return [robot_on_path_slice, *forward_path]

    def _footprint_sweep_paths(
        self, body_path: list[Point3], initial_yaw: float
    ) -> list[list[Point3]]:
        if not body_path:
            return []

        yaws = self._estimate_path_yaws(body_path, initial_yaw)
        sampled_poses: list[tuple[Point3, float]] = [(body_path[0], yaws[0])]
        for start, end, start_yaw, end_yaw in zip(
            body_path[:-1], body_path[1:], yaws[:-1], yaws[1:]
        ):
            distance = self._distance_3d(start, end)
            yaw_delta = self._normalize_angle(end_yaw - start_yaw)
            sample_count = max(
                1,
                math.ceil(distance / self.footprint_sweep_max_step),
                math.ceil(abs(yaw_delta) / self.footprint_sweep_max_yaw_step),
            )
            for sample_index in range(1, sample_count + 1):
                ratio = sample_index / sample_count
                pose = Point3(
                    start.x + (end.x - start.x) * ratio,
                    start.y + (end.y - start.y) * ratio,
                    start.z + (end.z - start.z) * ratio,
                )
                sampled_poses.append(
                    (pose, self._normalize_angle(start_yaw + yaw_delta * ratio))
                )

        return [
            [
                Point3(
                    pose.x + offset * math.cos(yaw),
                    pose.y + offset * math.sin(yaw),
                    pose.z,
                )
                for pose, yaw in sampled_poses
            ]
            for offset in self.footprint_center_offsets
        ]

    def _estimate_path_yaws(
        self, path: list[Point3], initial_yaw: float
    ) -> list[float]:
        if len(path) <= 1:
            return [initial_yaw]

        segment_yaws: list[float] = []
        previous_yaw = initial_yaw
        for start, end in zip(path[:-1], path[1:]):
            dx = end.x - start.x
            dy = end.y - start.y
            if dx * dx + dy * dy <= 1e-9:
                segment_yaws.append(previous_yaw)
            else:
                previous_yaw = math.atan2(dy, dx)
                segment_yaws.append(previous_yaw)

        point_yaws = [initial_yaw]
        for incoming, outgoing in zip(segment_yaws[:-1], segment_yaws[1:]):
            point_yaws.append(
                self._normalize_angle(
                    incoming + 0.5 * self._normalize_angle(outgoing - incoming)
                )
            )
        point_yaws.append(segment_yaws[-1])
        return point_yaws

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _distance_to_path(self, point: Point3, path: list[Point3]) -> tuple[float, float]:
        if not path:
            return math.inf, math.inf
        if len(path) == 1:
            return self._distance_2d(point, path[0]), abs(point.z - path[0].z)

        best_distance = math.inf
        best_z = math.inf
        for start, end in zip(path[:-1], path[1:]):
            vx = end.x - start.x
            vy = end.y - start.y
            vz = end.z - start.z
            length_sq = vx * vx + vy * vy
            if length_sq <= 1e-9:
                projected = start
            else:
                ratio = ((point.x - start.x) * vx + (point.y - start.y) * vy) / length_sq
                ratio = max(0.0, min(1.0, ratio))
                projected = Point3(
                    start.x + vx * ratio,
                    start.y + vy * ratio,
                    start.z + vz * ratio,
                )
            distance = self._distance_2d(point, projected)
            if distance < best_distance:
                best_distance = distance
                best_z = abs(point.z - projected.z)
        return best_distance, best_z

    def _decide_status(
        self, now: float, robot: Point3, window: PathWindow, check: ObstacleCheck
    ) -> dict[str, object]:
        if check.path_blocked and self.enforce_path_blocking:
            if check.clearance_escape:
                if self.replan_active:
                    self._publish_replan(False, now)
                self._reset_block_tracking()
                return self._status_payload(
                    "clearance_escape",
                    "escape",
                    True,
                    now,
                    robot=robot,
                    window=window,
                    check=check,
                    message="当前足迹位于膨胀边界，低速执行单调增距脱困轨迹",
                )
            self.clear_since = None
            if self.blocked_since is None:
                self.blocked_since = now
            blocked_duration = now - self.blocked_since
            if blocked_duration >= self.replan_blocked_duration:
                self.replan_confirmation_required = True
                self._remember_blocked_execution_generation()
                self._publish_replan(True, now)
                return self._status_payload(
                    "replan_requested",
                    "stop",
                    True,
                    now,
                    robot=robot,
                    window=window,
                    check=check,
                    blocked_duration=blocked_duration,
                    message="路径前方障碍持续存在，已请求重规划",
                )
            if (
                check.nearest_blocker_distance is not None
                and check.nearest_blocker_distance <= self.stop_distance
            ):
                return self._status_payload(
                    "blocked",
                    "stop",
                    True,
                    now,
                    robot=robot,
                    window=window,
                    check=check,
                    blocked_duration=blocked_duration,
                    message="路径前方障碍进入停车距离",
                )
            return self._status_payload(
                "caution",
                "slow",
                True,
                now,
                robot=robot,
                window=window,
                check=check,
                blocked_duration=blocked_duration,
                message="路径前方存在障碍，输出限速建议",
            )

        if self.blocked_since is not None:
            blocked_duration = now - self.blocked_since
            if self.replan_active and not self._has_new_execution_generation():
                return self._status_payload(
                    "waiting_replan",
                    "stop",
                    False,
                    now,
                    robot=robot,
                    window=window,
                    check=check,
                    blocked_duration=blocked_duration,
                    message="障碍短暂消失，等待更新后的安全轨迹",
                )
            # A new collision-free generation is now available.  Stop asking
            # SCAN for yet another trajectory while we keep the controller
            # frozen for the continuous-clearance confirmation window.
            #
            # Leaving /nav/replan_request asserted here caused a handoff
            # deadlock: every request replaced the candidate path, an
            # occasional transient candidate reset clear_since, and the
            # controller could therefore remain stopped forever even though a
            # valid lateral bypass had already been found.
            if self.replan_active:
                self._publish_replan(False, now)
            if self.clear_since is None:
                self.clear_since = now
            clear_duration = now - self.clear_since
            required_clear_duration = (
                self.blocked_clear_duration
                if self.replan_confirmation_required
                else self.transient_clear_duration
            )
            if clear_duration < required_clear_duration:
                return self._status_payload(
                    "clearing",
                    "stop",
                    False,
                    now,
                    robot=robot,
                    window=window,
                    check=check,
                    blocked_duration=blocked_duration,
                    message="新轨迹正在连续安全确认，保持停车",
                )
            if self.replan_active:
                self._publish_replan(False, now)
            self._reset_block_tracking()

        if (
            self.slow_nearby_obstacles
            and check.nearest_obstacle_distance is not None
            and check.nearest_obstacle_distance <= self.slow_distance
        ):
            return self._status_payload(
                "caution",
                "slow",
                False,
                now,
                robot=robot,
                window=window,
                check=check,
                message="机器人附近存在障碍，输出限速建议",
            )

        return self._status_payload(
            "clear",
            "pass",
            False,
            now,
            robot=robot,
            window=window,
            check=check,
            message="路径前方安全",
        )

    def _remember_blocked_execution_generation(self) -> None:
        if (
            self.active_path_source != "scan_execution_path"
            or self.execution_path_generation_time is None
        ):
            return
        if (
            self.blocked_execution_generation_time is None
            or self.execution_path_generation_time
            > self.blocked_execution_generation_time
        ):
            self.blocked_execution_generation_time = (
                self.execution_path_generation_time
            )

    def _has_new_execution_generation(self) -> bool:
        if self.blocked_execution_generation_time is None:
            return True
        return (
            self.active_path_source == "scan_execution_path"
            and self.execution_path_generation_time is not None
            and self.execution_path_generation_time
            > self.blocked_execution_generation_time + 1e-3
        )

    def _reset_block_tracking(self) -> None:
        self.blocked_since = None
        self.clear_since = None
        self.blocked_execution_generation_time = None
        self.replan_confirmation_required = False

    def _publish_replan(self, active: bool, now: float) -> None:
        if active and now - self.last_replan_publish_time < self.replan_publish_period:
            return
        if not active and not self.replan_active:
            return
        msg = Bool()
        msg.data = active
        self.replan_pub.publish(msg)
        self.replan_active = active
        self.last_replan_publish_time = now

    def _status_payload(
        self,
        status: str,
        action: str,
        path_blocked: bool,
        now: float,
        robot: Optional[Point3] = None,
        window: Optional[PathWindow] = None,
        check: Optional[ObstacleCheck] = None,
        blocked_duration: float = 0.0,
        prune_plan_size: Optional[int] = None,
        message: str = "",
    ) -> dict[str, object]:
        payload: dict[str, object] = {
            "status": status,
            "action": action,
            "path_blocked": path_blocked,
            "timestamp": now,
            "message": message,
            "global_frame": self.global_frame,
            "robot_frame": self.robot_frame,
            "has_path": bool(self.active_path_points),
            "path_source": self.active_path_source,
            "has_global_path": bool(self.path_points),
            "has_scan_execution_path": bool(self.execution_path_points),
            "execution_path_age": self.execution_path_age,
            "execution_path_generation_matches": self.execution_path_generation_matches,
            "execution_path_spatially_matches": self.execution_path_spatially_matches,
            "obstacle_points": len(self.obstacles),
            "sensor_heartbeat_topic": self.sensor_heartbeat_topic,
            "sensor_heartbeat_age": (
                None
                if self.last_sensor_heartbeat_time is None
                else max(0.0, now - self.last_sensor_heartbeat_time)
            ),
            "blocked_duration": blocked_duration,
        }
        if robot is not None:
            payload["robot"] = {"x": robot.x, "y": robot.y, "z": robot.z}
        if window is not None:
            payload["nearest_path_index"] = window.nearest_index
            payload["robot_to_path_distance"] = window.robot_to_path_distance
            payload["prune_plan_size"] = len(window.prune_points)
            payload["local_path_size"] = len(window.forward_points)
        elif prune_plan_size is not None:
            payload["prune_plan_size"] = prune_plan_size
        if check is not None:
            payload["blocker_count"] = check.blocker_count
            payload["self_filtered_count"] = check.self_filtered_count
            payload["nearest_obstacle_distance"] = check.nearest_obstacle_distance
            payload["nearest_blocker_distance"] = check.nearest_blocker_distance
            payload["nearest_path_distance"] = check.nearest_path_distance
            payload["clearance_escape"] = check.clearance_escape
        return payload

    def _set_state(self, payload: dict[str, object]) -> None:
        self.current_status = str(payload.get("status", "unknown"))
        self.current_action = str(payload.get("action", "none"))
        self.current_path_blocked = bool(payload.get("path_blocked", False))
        frozen = Bool()
        frozen.data = self.navigation_enabled and self.current_action == "stop"
        self.execution_frozen_pub.publish(frozen)

    def _publish_status(self, payload: dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_pub.publish(msg)

    def _publish_stop_if_needed(self) -> None:
        if self.publish_zero_on_stop and self.current_action == "stop":
            self.cmd_vel_safe_pub.publish(Twist())

    def _filter_twist(self, msg: Twist) -> Twist:
        if self.current_action == "stop":
            return Twist()
        filtered = Twist()
        if self.current_action == "slow":
            scale = self.slow_speed_scale
        elif self.current_action == "escape":
            scale = self.clearance_escape_speed_scale
        else:
            scale = 1.0
        filtered.linear.x = msg.linear.x * scale
        filtered.linear.y = msg.linear.y * scale
        # B2 SportClient exposes planar vx/vy/yaw only.  Never propagate
        # unsupported Twist axes even if an upstream node populates them.
        filtered.linear.z = 0.0
        filtered.angular.x = 0.0
        filtered.angular.y = 0.0
        filtered.angular.z = msg.angular.z * scale
        return filtered

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _distance_2d(self, a: Point3, b: Point3) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _distance_3d(self, a: Point3, b: Point3) -> float:
        dx = a.x - b.x
        dy = a.y - b.y
        dz = a.z - b.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _same_point(self, a: Point3, b: Point3) -> bool:
        return self._distance_3d(a, b) <= 1e-6

    def _min_optional(self, current: Optional[float], value: float) -> float:
        return value if current is None else min(current, value)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DynamicAvoidanceMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
