#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64, String
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
    nearest_obstacle_point: Optional[Point3]
    nearest_obstacle_dx: Optional[float]
    nearest_obstacle_dy: Optional[float]
    nearest_obstacle_dz: Optional[float]
    nearest_obstacle_body_x: Optional[float]
    nearest_obstacle_body_y: Optional[float]
    nearest_obstacle_side: str
    nearest_blocker_distance: Optional[float]
    nearest_path_distance: Optional[float]
    nearest_path_z_distance: Optional[float]
    nearest_path_z_delta: Optional[float]
    nearest_path_obstacle: Optional[Point3]
    nearest_path_projection: Optional[Point3]
    nearest_blocker_obstacle: Optional[Point3]
    nearest_blocker_path_distance: Optional[float]
    nearest_blocker_z_distance: Optional[float]
    nearest_blocker_z_delta: Optional[float]


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
        self.declare_parameter("global_path_height_offset", 0.0)
        self.declare_parameter("obstacle_topic", "/nav/local_obstacles")
        self.declare_parameter("status_topic", "/nav/obstacle_status")
        self.declare_parameter("replan_request_topic", "/nav/replan_request")
        self.declare_parameter("prune_plan_topic", "/prune_plan")
        self.declare_parameter("local_path_topic", "/nav/local_path")
        self.declare_parameter("cmd_vel_in_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_safe_topic", "/cmd_vel_safe")
        self.declare_parameter(
            "safety_execution_frozen_topic", "/planning/safety_execution_frozen"
        )
        self.declare_parameter("safety_speed_scale_topic", "/planning/safety_speed_scale")
        self.declare_parameter("nav_start_topic", "/nav_start")
        self.declare_parameter("nav_stop_topic", "/nav_stop")
        self.declare_parameter("lookahead_distance", 2.0)
        self.declare_parameter("backward_prune_distance", 0.5)
        self.declare_parameter("path_deviation_tolerance", 1.0)
        # /grid_map/occupancy_inflate is already expanded for the B2 footprint.
        # Keep only a two-voxel discretization margin here; a body-sized
        # corridor would inflate the same obstacle twice.
        self.declare_parameter("path_corridor_radius", 0.10)
        # The execution path is lifted to the body planning height.  A broad
        # vertical band includes traversable floor voxels below that path and
        # permanently freezes otherwise collision-free trajectories.
        self.declare_parameter("z_tolerance", 0.10)
        self.declare_parameter("z_tolerance_down", 0.10)
        self.declare_parameter("z_tolerance_up", 0.10)
        self.declare_parameter("stop_distance", 0.6)
        self.declare_parameter("slow_distance", 1.2)
        self.declare_parameter("robot_self_clear_radius", 0.90)
        self.declare_parameter("replan_blocked_duration", 2.0)
        self.declare_parameter("sensor_timeout", 1.5)
        self.declare_parameter("check_period_sec", 0.2)
        self.declare_parameter("obstacle_processing_period_sec", 0.2)
        self.declare_parameter("obstacle_voxel_leaf_size", 0.0)
        self.declare_parameter("max_obstacle_points", 8000)
        self.declare_parameter("slow_speed_scale", 0.35)
        self.declare_parameter("slow_nearby_obstacles", True)
        self.declare_parameter("require_obstacle_stream", False)
        self.declare_parameter("allow_path_start_without_tf", False)
        self.declare_parameter("enable_cmd_vel_filter", True)
        self.declare_parameter("enforce_path_blocking", True)
        self.declare_parameter("stop_on_path_blocked", False)
        self.declare_parameter("require_nav_start", True)
        self.declare_parameter("publish_zero_on_stop", True)
        self.declare_parameter("publish_prune_plan", True)
        self.declare_parameter("replan_publish_period", 1.0)
        self.declare_parameter("diagnostic_log_period", 1.0)
        self.declare_parameter("obstacle_timing_log_enabled", False)
        self.declare_parameter("obstacle_timing_log_period", 1.0)
        self.declare_parameter("safety_check_timing_log_enabled", False)
        self.declare_parameter("safety_check_timing_log_period", 1.0)

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
        self.global_path_height_offset = float(
            self.get_parameter("global_path_height_offset").value
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
        self.z_tolerance = float(self.get_parameter("z_tolerance").value)
        self.z_tolerance_down = max(
            float(self.get_parameter("z_tolerance_down").value), 0.0
        )
        self.z_tolerance_up = max(
            float(self.get_parameter("z_tolerance_up").value), 0.0
        )
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.slow_distance = float(self.get_parameter("slow_distance").value)
        self.robot_self_clear_radius = max(
            float(self.get_parameter("robot_self_clear_radius").value), 0.0
        )
        self.replan_blocked_duration = float(
            self.get_parameter("replan_blocked_duration").value
        )
        self.sensor_timeout = float(self.get_parameter("sensor_timeout").value)
        self.obstacle_timing_log_enabled = bool(
            self.get_parameter("obstacle_timing_log_enabled").value
        )
        self.obstacle_timing_log_period = max(
            float(self.get_parameter("obstacle_timing_log_period").value), 0.1
        )
        self.last_obstacle_timing_log_time: Optional[float] = None
        self.safety_check_timing_log_enabled = bool(
            self.get_parameter("safety_check_timing_log_enabled").value
        )
        self.safety_check_timing_log_period = max(
            float(self.get_parameter("safety_check_timing_log_period").value), 0.1
        )
        self.last_safety_check_timing_log_time: Optional[float] = None
        self.obstacle_processing_period = max(
            float(self.get_parameter("obstacle_processing_period_sec").value), 0.05
        )
        self.obstacle_voxel_leaf_size = max(
            float(self.get_parameter("obstacle_voxel_leaf_size").value), 0.0
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
        self.allow_path_start_without_tf = bool(
            self.get_parameter("allow_path_start_without_tf").value
        )
        self.enable_cmd_vel_filter = bool(
            self.get_parameter("enable_cmd_vel_filter").value
        )
        self.enforce_path_blocking = bool(
            self.get_parameter("enforce_path_blocking").value
        )
        self.stop_on_path_blocked = bool(
            self.get_parameter("stop_on_path_blocked").value
        )
        self.require_nav_start = bool(self.get_parameter("require_nav_start").value)
        self.publish_zero_on_stop = bool(
            self.get_parameter("publish_zero_on_stop").value
        )
        self.publish_prune_plan = bool(self.get_parameter("publish_prune_plan").value)
        self.replan_publish_period = float(
            self.get_parameter("replan_publish_period").value
        )
        self.diagnostic_log_period = max(
            float(self.get_parameter("diagnostic_log_period").value), 0.2
        )

        self.path_points: list[Point3] = []
        self.global_safety_path_points: list[Point3] = []
        self.path_frame = self.global_frame
        self.execution_path_points: list[Point3] = []
        self.last_execution_path_time: Optional[float] = None
        self.execution_path_callback_gap: Optional[float] = None
        self.execution_path_generation_time: Optional[float] = None
        self.last_global_path_time: Optional[float] = None
        self.active_path_points: list[Point3] = []
        self.active_path_source = "none"
        self.execution_path_age: Optional[float] = None
        self.execution_path_generation_matches = False
        self.execution_path_spatially_matches = False
        self.execution_path_start_distance: Optional[float] = None
        self.execution_path_end_distance: Optional[float] = None
        self.execution_path_length: Optional[float] = None
        self.global_path_span: Optional[float] = None
        self.obstacles: list[Point3] = []
        self.last_obstacle_time: Optional[float] = None
        self.last_obstacle_processing_time: Optional[float] = None
        self.last_robot_yaw: Optional[float] = None
        self.blocked_since: Optional[float] = None
        self.last_replan_publish_time = 0.0
        self.replan_active = False
        self.current_status = "idle"
        self.current_action = "none"
        self.current_path_blocked = False
        self.navigation_enabled = not self.require_nav_start
        self.last_state_log_key: Optional[tuple[str, str, bool, str]] = None
        self.last_state_log_time = 0.0
        self.last_global_path_log_time = 0.0
        self.last_global_path_log_size = 0
        self.last_execution_path_log_time = 0.0
        self.last_execution_path_log_size = 0
        self.last_diagnostic_log_time = 0.0
        self.last_raw_obstacle_count = 0
        self.last_transformed_obstacle_count = 0
        self.last_obstacle_frame = ""
        self.last_safety_check_ms = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Obstacle checks are CPU-heavy pure Python.  Keep velocity and the
        # lightweight path callbacks in independent groups so the timer cannot
        # make /cmd_vel unsafe or make a healthy 10Hz execution path look stale.
        self.command_callback_group = MutuallyExclusiveCallbackGroup()
        self.path_callback_group = MutuallyExclusiveCallbackGroup()

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
        self.safety_speed_scale_pub = self.create_publisher(
            Float64,
            str(self.get_parameter("safety_speed_scale_topic").value),
            10,
        )

        self.create_subscription(
            Path,
            str(self.get_parameter("global_path_topic").value),
            self._on_global_path,
            10,
            callback_group=self.path_callback_group,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("scan_execution_path_topic").value),
            self._on_execution_path,
            10,
            callback_group=self.path_callback_group,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("obstacle_topic").value),
            self._on_obstacles,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_in_topic").value),
            self._on_cmd_vel,
            10,
            callback_group=self.command_callback_group,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("nav_start_topic").value),
            self._on_nav_start,
            10,
            callback_group=self.command_callback_group,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("nav_stop_topic").value),
            self._on_nav_stop,
            10,
            callback_group=self.command_callback_group,
        )

        period = max(float(self.get_parameter("check_period_sec").value), 0.05)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(
            "动态避障监测已启动："
            f"path={self.get_parameter('global_path_topic').value}, "
            f"execution_path={self.get_parameter('scan_execution_path_topic').value}, "
            f"obstacles={self.get_parameter('obstacle_topic').value}, "
            f"require_obstacle_stream={self.require_obstacle_stream}, "
            f"obstacle_period={self.obstacle_processing_period:.2f}s, "
            f"obstacle_voxel_leaf_size={self.obstacle_voxel_leaf_size:.3f}m, "
            f"max_obstacle_points={self.max_obstacle_points}, "
            f"global_path_height_offset={self.global_path_height_offset:.3f}m, "
            f"path_corridor_radius={self.path_corridor_radius:.3f}m, "
            f"z_tolerance={self.z_tolerance:.3f}m, "
            f"z_tolerance_down={self.z_tolerance_down:.3f}m, "
            f"z_tolerance_up={self.z_tolerance_up:.3f}m, "
            f"robot_self_clear_radius={self.robot_self_clear_radius:.3f}m, "
            f"diagnostic_log_period={self.diagnostic_log_period:.2f}s"
        )

    def _on_global_path(self, msg: Path) -> None:
        points = self._points_from_path(msg)
        self.path_points = points
        self.global_safety_path_points = [
            Point3(point.x, point.y, point.z + self.global_path_height_offset)
            for point in points
        ]
        self.path_frame = self.global_frame
        self.last_global_path_time = self._now_sec()
        if not points:
            self.get_logger().warn("收到空的全局路径，动态避障进入 no_path 状态")
        elif (
            self.last_global_path_log_size != len(points)
            or self.last_global_path_time - self.last_global_path_log_time >= 2.0
        ):
            self.last_global_path_log_time = self.last_global_path_time
            self.last_global_path_log_size = len(points)
            self.get_logger().info(
                "收到全局路径: "
                f"points={len(points)} start=({points[0].x:.2f},{points[0].y:.2f},{points[0].z:.2f}) "
                f"end=({points[-1].x:.2f},{points[-1].y:.2f},{points[-1].z:.2f})"
            )

    def _on_execution_path(self, msg: Path) -> None:
        now = self._now_sec()
        self.execution_path_callback_gap = (
            None
            if self.last_execution_path_time is None
            else max(0.0, now - self.last_execution_path_time)
        )
        self.execution_path_points = self._points_from_path(msg)
        self.last_execution_path_time = now
        stamp = msg.header.stamp
        generation_time = float(stamp.sec) + float(stamp.nanosec) / 1e9
        self.execution_path_generation_time = (
            generation_time if generation_time > 0.0 else self.last_execution_path_time
        )
        if (
            self.execution_path_points
            and (
                self.last_execution_path_log_size != len(self.execution_path_points)
                or self.last_execution_path_time - self.last_execution_path_log_time >= 2.0
            )
        ):
            self.last_execution_path_log_time = self.last_execution_path_time
            self.last_execution_path_log_size = len(self.execution_path_points)
            points = self.execution_path_points
            self.get_logger().info(
                "收到SCAN执行路径: "
                f"points={len(points)} start=({points[0].x:.2f},{points[0].y:.2f},{points[0].z:.2f}) "
                f"end=({points[-1].x:.2f},{points[-1].y:.2f},{points[-1].z:.2f})"
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
        callback_start = time.perf_counter()
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

        read_start = time.perf_counter()
        points = self._read_cloud_points(msg)
        read_ms = (time.perf_counter() - read_start) * 1000.0
        transformed: list[Point3] = []
        frame_id = msg.header.frame_id or self.global_frame
        transform_start = time.perf_counter()
        for point in points:
            global_point = self._transform_point(point, frame_id)
            if global_point is not None:
                transformed.append(global_point)
        transform_ms = (time.perf_counter() - transform_start) * 1000.0
        self.obstacles = transformed
        self.last_raw_obstacle_count = len(points)
        self.last_transformed_obstacle_count = len(transformed)
        self.last_obstacle_frame = frame_id
        total_ms = (time.perf_counter() - callback_start) * 1000.0
        self._log_obstacle_timing_if_needed(
            now=now,
            msg=msg,
            points=len(points),
            transformed=len(transformed),
            frame_id=frame_id,
            read_ms=read_ms,
            transform_ms=transform_ms,
            total_ms=total_ms,
        )

    def _log_obstacle_timing_if_needed(
        self,
        *,
        now: float,
        msg: PointCloud2,
        points: int,
        transformed: int,
        frame_id: str,
        read_ms: float,
        transform_ms: float,
        total_ms: float,
    ) -> None:
        if not self.obstacle_timing_log_enabled:
            return
        if (
            self.last_obstacle_timing_log_time is not None
            and now - self.last_obstacle_timing_log_time
            < self.obstacle_timing_log_period
        ):
            return
        self.last_obstacle_timing_log_time = now
        input_points = int(getattr(msg, "width", 0) or 0) * int(
            getattr(msg, "height", 0) or 0
        )
        slowest = "read_cloud"
        slowest_ms = read_ms
        if transform_ms > slowest_ms:
            slowest = "transform"
            slowest_ms = transform_ms
        self.get_logger().info(
            "[dynamic_avoidance timing] "
            f"slowest={slowest} {slowest_ms:.2f}ms total={total_ms:.2f}ms "
            f"input_points={input_points} used_points={points} transformed={transformed} "
            f"frame={frame_id} read_cloud={read_ms:.2f}ms transform={transform_ms:.2f}ms "
            f"voxel_leaf={self.obstacle_voxel_leaf_size:.3f} max_points={self.max_obstacle_points}"
        )

    def _log_safety_check_timing_if_needed(
        self, now: float, window: PathWindow, check: ObstacleCheck
    ) -> None:
        if not self.safety_check_timing_log_enabled:
            return
        if (
            self.last_safety_check_timing_log_time is not None
            and now - self.last_safety_check_timing_log_time
            < self.safety_check_timing_log_period
        ):
            return
        self.last_safety_check_timing_log_time = now
        self.get_logger().info(
            "[dynamic_avoidance safety timing] "
            f"check_ms={self.last_safety_check_ms:.2f} "
            f"path_source={self.active_path_source} "
            f"path_points={len(window.forward_points)} "
            f"obstacle_points={len(self.obstacles)} "
            f"blocker_count={check.blocker_count} "
            f"execution_age={self._fmt_optional(self.execution_path_age)} "
            f"execution_callback_gap={self._fmt_optional(self.execution_path_callback_gap)}"
        )

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
            self._publish_safety_speed_scale(0.0)
        else:
            self._publish_safety_speed_scale(1.0)

    def _on_nav_stop(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.navigation_enabled = False
        self.cmd_vel_safe_pub.publish(Twist())
        self._publish_safety_speed_scale(0.0)

    def _on_timer(self) -> None:
        now = self._now_sec()
        self._drop_stale_obstacles_if_optional(now)
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
            self.blocked_since = None
            status_payload = self._status_payload(
                "no_path", "idle", False, now, message="尚未收到可用导航路径"
            )
            self._set_state(status_payload)
            self._publish_status(status_payload)
            return

        if robot_point is None:
            self.blocked_since = None
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
        fallback_path_points = getattr(
            self, "global_safety_path_points", self.path_points
        )
        if (
            window is None
            and self.active_path_source == "scan_execution_path"
            and fallback_path_points
        ):
            fallback_window = self._make_path_window(
                robot_point, fallback_path_points
            )
            if fallback_window is not None:
                self.active_path_points = fallback_path_points
                self.active_path_source = "global_path_lifted_fallback"
                window = fallback_window

        if window is None:
            self.blocked_since = None
            status_payload = self._status_payload(
                "deviated",
                "stop",
                False,
                now,
                robot=robot_point,
                message=f"机器人偏离{self.active_path_source}过远，无法可靠检查路径障碍，停车等待定位/路径恢复",
            )
            self._set_state(status_payload)
            self._publish_status(status_payload)
            self._publish_stop_if_needed()
            return

        self._publish_path_window(window)

        if self._obstacle_stream_lost(now):
            self.blocked_since = None
            status_payload = self._status_payload(
                "sensor_lost",
                "stop",
                False,
                now,
                robot=robot_point,
                prune_plan_size=len(window.prune_points),
                message="局部障碍点云超时",
            )
            self._set_state(status_payload)
            self._publish_status(status_payload)
            self._publish_stop_if_needed()
            return

        safety_check_started = time.perf_counter()
        check = self._check_obstacles(robot_point, window.forward_points)
        self.last_safety_check_ms = (
            time.perf_counter() - safety_check_started
        ) * 1000.0
        self._log_safety_check_timing_if_needed(now, window, check)
        status_payload = self._decide_status(now, robot_point, window, check)
        self._set_state(status_payload)
        self._publish_status(status_payload)
        self._log_diagnostics(status_payload, check)
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
        if self.global_safety_path_points:
            return self.global_safety_path_points, "global_path_lifted"
        return [], "none"

    def _execution_path_matches_global(self) -> bool:
        self.execution_path_start_distance = None
        self.execution_path_end_distance = None
        self.execution_path_length = None
        self.global_path_span = None
        if not self.execution_path_points:
            return False
        if not self.path_points:
            return True

        tolerance = self.scan_execution_path_global_tolerance
        execution_start = self.execution_path_points[0]
        execution_end = self.execution_path_points[-1]
        self.execution_path_start_distance = self._distance_3d(
            execution_start, self.path_points[0]
        )
        self.execution_path_end_distance = min(
            self._distance_3d(execution_end, point) for point in self.path_points
        )
        self.global_path_span = self._distance_3d(
            self.path_points[0], self.path_points[-1]
        )
        self.execution_path_length = sum(
            self._distance_3d(start, end)
            for start, end in zip(
                self.execution_path_points[:-1], self.execution_path_points[1:]
            )
        )
        if self.execution_path_start_distance > tolerance:
            return False
        if self.execution_path_end_distance > tolerance:
            return False

        # A completed one-point trajectory from the previous goal often sits at
        # the new global path start.  It is spatially close but must not override
        # a new long navigation request.
        if self.global_path_span > 1.0 and self.execution_path_length < 0.3:
            return False
        return True

    def _read_cloud_points(self, msg: PointCloud2) -> list[Point3]:
        if point_cloud2 is None:
            self.get_logger().error("缺少 sensor_msgs_py，无法读取 PointCloud2")
            return []
        raw_points = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if self.obstacle_voxel_leaf_size > 0.0:
            raw_points = self._voxel_downsample_points(
                raw_points, self.obstacle_voxel_leaf_size
            )
        else:
            raw_points = list(raw_points)
        point_count = len(raw_points)
        if point_count > self.max_obstacle_points:
            stride = math.ceil(point_count / self.max_obstacle_points)
            raw_points = raw_points[::stride]
        return [
            Point3(float(point[0]), float(point[1]), float(point[2]))
            for point in raw_points
        ]

    @staticmethod
    def _voxel_downsample_points(
        raw_points: Iterable[tuple[float, float, float]], leaf_size: float
    ) -> list[tuple[float, float, float]]:
        if leaf_size <= 0.0:
            return list(raw_points)

        inv_leaf = 1.0 / leaf_size
        voxels: dict[tuple[int, int, int], tuple[float, float, float]] = {}
        for point in raw_points:
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            key = (
                math.floor(x * inv_leaf),
                math.floor(y * inv_leaf),
                math.floor(z * inv_leaf),
            )
            if key not in voxels:
                voxels[key] = (x, y, z)
        return list(voxels.values())

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
            self.last_robot_yaw = self._yaw_from_quaternion(
                float(q.x), float(q.y), float(q.z), float(q.w)
            )
            return Point3(float(t.x), float(t.y), float(t.z)), ""
        except TransformException as exc:
            self.last_robot_yaw = None
            if self.allow_path_start_without_tf and self.path_points:
                return self.path_points[0], "使用路径起点作为无 TF 离线兜底"
            return None, str(exc)

    def _yaw_from_quaternion(self, qx: float, qy: float, qz: float, qw: float) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

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
        if self.last_obstacle_time is None:
            return True
        return now - self.last_obstacle_time > self.sensor_timeout

    def _drop_stale_obstacles_if_optional(self, now: float) -> None:
        if self.require_obstacle_stream:
            return
        if self.last_obstacle_time is None:
            return
        if now - self.last_obstacle_time > self.sensor_timeout:
            self.obstacles = []

    def _check_obstacles(self, robot: Point3, forward_path: list[Point3]) -> ObstacleCheck:
        nearest_obstacle_distance: Optional[float] = None
        nearest_obstacle_point: Optional[Point3] = None
        nearest_obstacle_dx: Optional[float] = None
        nearest_obstacle_dy: Optional[float] = None
        nearest_obstacle_dz: Optional[float] = None
        nearest_obstacle_body_x: Optional[float] = None
        nearest_obstacle_body_y: Optional[float] = None
        nearest_obstacle_side = "none"
        nearest_blocker_distance: Optional[float] = None
        nearest_path_distance: Optional[float] = None
        nearest_path_z_distance: Optional[float] = None
        nearest_path_z_delta: Optional[float] = None
        nearest_path_obstacle: Optional[Point3] = None
        nearest_path_projection: Optional[Point3] = None
        nearest_blocker_obstacle: Optional[Point3] = None
        nearest_blocker_path_distance: Optional[float] = None
        nearest_blocker_z_distance: Optional[float] = None
        nearest_blocker_z_delta: Optional[float] = None
        blocker_count = 0
        self_filtered_count = 0
        path_check_radius = (
            self.path_deviation_tolerance
            + self.lookahead_distance
            + self.path_corridor_radius
        )
        nearby_check_radius = (
            self.slow_distance if self.slow_nearby_obstacles else 0.0
        )
        obstacle_check_radius = max(
            path_check_radius,
            nearby_check_radius,
        )

        if self.obstacles and forward_path:
            # The old implementation evaluated every obstacle against every
            # path segment in nested Python loops. On the Jetson, ~4k points
            # took 0.45-0.55 s and made a newly appearing person visible to the
            # stop logic far too late. Keep the segment loop small and perform
            # all point projection/distance operations in NumPy.
            obstacle_xyz = np.asarray(
                [(point.x, point.y, point.z) for point in self.obstacles],
                dtype=np.float64,
            )
            finite_mask = np.all(np.isfinite(obstacle_xyz), axis=1)
            robot_dx_all = obstacle_xyz[:, 0] - robot.x
            robot_dy_all = obstacle_xyz[:, 1] - robot.y
            robot_distance_sq_all = robot_dx_all**2 + robot_dy_all**2
            self_mask = finite_mask & (
                robot_distance_sq_all <= self.robot_self_clear_radius**2
            )
            self_filtered_count = int(np.count_nonzero(self_mask))
            candidate_mask = (
                finite_mask
                & ~self_mask
                & (robot_distance_sq_all <= obstacle_check_radius**2)
            )
            source_indices = np.flatnonzero(candidate_mask)

            if source_indices.size:
                points = obstacle_xyz[source_indices]
                obstacle_dx_values = robot_dx_all[source_indices]
                obstacle_dy_values = robot_dy_all[source_indices]
                robot_distance_values = np.sqrt(
                    robot_distance_sq_all[source_indices]
                )

                nearest_index = int(np.argmin(robot_distance_values))
                nearest_source_index = int(source_indices[nearest_index])
                nearest_obstacle_distance = float(
                    robot_distance_values[nearest_index]
                )
                nearest_obstacle_point = self.obstacles[nearest_source_index]
                nearest_obstacle_dx = float(obstacle_dx_values[nearest_index])
                nearest_obstacle_dy = float(obstacle_dy_values[nearest_index])
                nearest_obstacle_dz = float(points[nearest_index, 2] - robot.z)
                if self.last_robot_yaw is not None:
                    cos_yaw = math.cos(self.last_robot_yaw)
                    sin_yaw = math.sin(self.last_robot_yaw)
                    nearest_obstacle_body_x = float(
                        cos_yaw * nearest_obstacle_dx
                        + sin_yaw * nearest_obstacle_dy
                    )
                    nearest_obstacle_body_y = float(
                        -sin_yaw * nearest_obstacle_dx
                        + cos_yaw * nearest_obstacle_dy
                    )
                    nearest_obstacle_side = self._body_side(
                        nearest_obstacle_body_x, nearest_obstacle_body_y
                    )

                path_xyz = np.asarray(
                    [(point.x, point.y, point.z) for point in forward_path],
                    dtype=np.float64,
                )
                best_distance_sq = np.full(len(points), np.inf, dtype=np.float64)
                best_z_delta = np.full(len(points), np.inf, dtype=np.float64)
                best_projection = np.full((len(points), 3), np.nan, dtype=np.float64)

                if len(path_xyz) == 1:
                    projected = np.broadcast_to(path_xyz[0], points.shape)
                    delta_xy = points[:, :2] - projected[:, :2]
                    best_distance_sq[:] = np.einsum(
                        "ij,ij->i", delta_xy, delta_xy
                    )
                    best_z_delta[:] = points[:, 2] - projected[:, 2]
                    best_projection[:] = projected
                else:
                    starts = path_xyz[:-1]
                    segments = path_xyz[1:] - starts
                    segment_length_sq = np.einsum(
                        "ij,ij->i", segments[:, :2], segments[:, :2]
                    )
                    safe_length_sq = np.where(
                        segment_length_sq > 1e-9, segment_length_sq, 1.0
                    )
                    relative_xy = (
                        points[:, None, :2] - starts[None, :, :2]
                    )
                    ratios = np.clip(
                        np.sum(
                            relative_xy * segments[None, :, :2], axis=2
                        )
                        / safe_length_sq[None, :],
                        0.0,
                        1.0,
                    )
                    ratios[:, segment_length_sq <= 1e-9] = 0.0
                    delta_x = (
                        relative_xy[:, :, 0]
                        - ratios * segments[None, :, 0]
                    )
                    delta_y = (
                        relative_xy[:, :, 1]
                        - ratios * segments[None, :, 1]
                    )
                    distance_sq_by_segment = delta_x**2 + delta_y**2
                    nearest_segments = np.argmin(
                        distance_sq_by_segment, axis=1
                    )
                    point_indices = np.arange(len(points))
                    nearest_ratios = ratios[point_indices, nearest_segments]
                    best_distance_sq[:] = distance_sq_by_segment[
                        point_indices, nearest_segments
                    ]
                    best_projection[:] = (
                        starts[nearest_segments]
                        + nearest_ratios[:, None] * segments[nearest_segments]
                    )
                    best_z_delta[:] = points[:, 2] - best_projection[:, 2]

                path_distance_values = np.sqrt(best_distance_sq)
                nearest_path_index = int(np.argmin(path_distance_values))
                nearest_path_source_index = int(
                    source_indices[nearest_path_index]
                )
                nearest_path_distance = float(
                    path_distance_values[nearest_path_index]
                )
                nearest_path_z_delta = float(best_z_delta[nearest_path_index])
                nearest_path_z_distance = abs(nearest_path_z_delta)
                nearest_path_obstacle = self.obstacles[nearest_path_source_index]
                nearest_path_projection = Point3(
                    *map(float, best_projection[nearest_path_index])
                )

                blocker_mask = (
                    (path_distance_values <= self.path_corridor_radius)
                    & (best_z_delta >= -self.z_tolerance_down)
                    & (best_z_delta <= self.z_tolerance_up)
                )
                blocker_indices = np.flatnonzero(blocker_mask)
                blocker_count = int(blocker_indices.size)
                if blocker_count:
                    nearest_blocker_local = int(
                        blocker_indices[
                            np.argmin(robot_distance_values[blocker_indices])
                        ]
                    )
                    nearest_blocker_source = int(
                        source_indices[nearest_blocker_local]
                    )
                    nearest_blocker_distance = float(
                        robot_distance_values[nearest_blocker_local]
                    )
                    nearest_blocker_obstacle = self.obstacles[
                        nearest_blocker_source
                    ]
                    nearest_blocker_path_distance = float(
                        path_distance_values[nearest_blocker_local]
                    )
                    nearest_blocker_z_delta = float(
                        best_z_delta[nearest_blocker_local]
                    )
                    nearest_blocker_z_distance = abs(
                        nearest_blocker_z_delta
                    )

        return ObstacleCheck(
            path_blocked=blocker_count > 0,
            blocker_count=blocker_count,
            self_filtered_count=self_filtered_count,
            nearest_obstacle_distance=nearest_obstacle_distance,
            nearest_obstacle_point=nearest_obstacle_point,
            nearest_obstacle_dx=nearest_obstacle_dx,
            nearest_obstacle_dy=nearest_obstacle_dy,
            nearest_obstacle_dz=nearest_obstacle_dz,
            nearest_obstacle_body_x=nearest_obstacle_body_x,
            nearest_obstacle_body_y=nearest_obstacle_body_y,
            nearest_obstacle_side=nearest_obstacle_side,
            nearest_blocker_distance=nearest_blocker_distance,
            nearest_path_distance=nearest_path_distance,
            nearest_path_z_distance=nearest_path_z_distance,
            nearest_path_z_delta=nearest_path_z_delta,
            nearest_path_obstacle=nearest_path_obstacle,
            nearest_path_projection=nearest_path_projection,
            nearest_blocker_obstacle=nearest_blocker_obstacle,
            nearest_blocker_path_distance=nearest_blocker_path_distance,
            nearest_blocker_z_distance=nearest_blocker_z_distance,
            nearest_blocker_z_delta=nearest_blocker_z_delta,
        )

    def _body_side(self, body_x: float, body_y: float) -> str:
        if abs(body_x) >= abs(body_y):
            return "front" if body_x >= 0.0 else "back"
        return "left" if body_y >= 0.0 else "right"

    def _distance_to_path(
        self, point: Point3, path: list[Point3]
    ) -> tuple[float, float, float, Optional[Point3]]:
        if not path:
            return math.inf, math.inf, math.inf, None
        if len(path) == 1:
            z_delta = point.z - path[0].z
            return self._distance_2d(point, path[0]), abs(z_delta), z_delta, path[0]

        best_distance_sq = math.inf
        best_z = math.inf
        best_z_delta = math.inf
        best_projection_xyz: Optional[tuple[float, float, float]] = None
        for start, end in zip(path[:-1], path[1:]):
            vx = end.x - start.x
            vy = end.y - start.y
            vz = end.z - start.z
            length_sq = vx * vx + vy * vy
            if length_sq <= 1e-9:
                projected_x = start.x
                projected_y = start.y
                projected_z = start.z
            else:
                ratio = ((point.x - start.x) * vx + (point.y - start.y) * vy) / length_sq
                ratio = max(0.0, min(1.0, ratio))
                projected_x = start.x + vx * ratio
                projected_y = start.y + vy * ratio
                projected_z = start.z + vz * ratio
            dx = point.x - projected_x
            dy = point.y - projected_y
            distance_sq = dx * dx + dy * dy
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_z_delta = point.z - projected_z
                best_z = abs(best_z_delta)
                best_projection_xyz = (
                    projected_x,
                    projected_y,
                    projected_z,
                )
        best_projection = (
            None
            if best_projection_xyz is None
            else Point3(*best_projection_xyz)
        )
        return math.sqrt(best_distance_sq), best_z, best_z_delta, best_projection

    def _within_z_tolerance(self, z_delta: float) -> bool:
        return -self.z_tolerance_down <= z_delta <= self.z_tolerance_up

    def _decide_status(
        self, now: float, robot: Point3, window: PathWindow, check: ObstacleCheck
    ) -> dict[str, object]:
        if check.path_blocked and self.enforce_path_blocking:
            if self.blocked_since is None:
                self.blocked_since = now
            blocked_duration = now - self.blocked_since
            if self.stop_on_path_blocked:
                self._publish_replan(True, now)
                return self._status_payload(
                    "blocked",
                    "stop",
                    True,
                    now,
                    robot=robot,
                    window=window,
                    check=check,
                    blocked_duration=blocked_duration,
                    stop_reason="path_blocked_stop",
                    message="路径前方存在障碍，保守停车并请求重规划",
                )
            if (
                check.nearest_blocker_distance is not None
                and check.nearest_blocker_distance <= self.stop_distance
            ):
                self._publish_replan(True, now)
                return self._status_payload(
                    "blocked",
                    "stop",
                    True,
                    now,
                    robot=robot,
                    window=window,
                    check=check,
                    blocked_duration=blocked_duration,
                    stop_reason="path_blocker_distance_stop",
                    message="路径前方障碍进入停车距离，立即停车并请求重规划",
                )
            if blocked_duration >= self.replan_blocked_duration:
                self._publish_replan(True, now)
                return self._status_payload(
                    "replan_requested",
                    "slow",
                    True,
                    now,
                    robot=robot,
                    window=window,
                    check=check,
                    blocked_duration=blocked_duration,
                    message="路径前方障碍持续存在，低速通过并请求重规划",
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
            self._publish_replan(False, now)
        self.blocked_since = None

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
        stop_reason: str = "",
    ) -> dict[str, object]:
        payload: dict[str, object] = {
            "status": status,
            "action": action,
            "path_blocked": path_blocked,
            "timestamp": now,
            "message": message,
            "stop_reason": stop_reason,
            "global_frame": self.global_frame,
            "robot_frame": self.robot_frame,
            "has_path": bool(self.active_path_points),
            "path_source": self.active_path_source,
            "has_global_path": bool(self.path_points),
            "has_scan_execution_path": bool(self.execution_path_points),
            "execution_path_age": self.execution_path_age,
            "execution_path_callback_gap": getattr(
                self, "execution_path_callback_gap", None
            ),
            "execution_path_generation_matches": self.execution_path_generation_matches,
            "execution_path_spatially_matches": self.execution_path_spatially_matches,
            "execution_path_start_distance": self.execution_path_start_distance,
            "execution_path_end_distance": self.execution_path_end_distance,
            "execution_path_length": self.execution_path_length,
            "global_path_span": self.global_path_span,
            "obstacle_points": len(self.obstacles),
            "safety_check_ms": getattr(self, "last_safety_check_ms", 0.0),
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
            payload["nearest_obstacle_point"] = self._point_payload(
                check.nearest_obstacle_point
            )
            payload["nearest_obstacle_dx"] = check.nearest_obstacle_dx
            payload["nearest_obstacle_dy"] = check.nearest_obstacle_dy
            payload["nearest_obstacle_dz"] = check.nearest_obstacle_dz
            payload["nearest_obstacle_body_x"] = check.nearest_obstacle_body_x
            payload["nearest_obstacle_body_y"] = check.nearest_obstacle_body_y
            payload["nearest_obstacle_side"] = check.nearest_obstacle_side
            payload["nearest_blocker_distance"] = check.nearest_blocker_distance
            payload["nearest_path_distance"] = check.nearest_path_distance
            payload["nearest_path_z_distance"] = check.nearest_path_z_distance
            payload["nearest_path_z_delta"] = check.nearest_path_z_delta
            payload["nearest_path_obstacle"] = self._point_payload(
                check.nearest_path_obstacle
            )
            payload["nearest_path_projection"] = self._point_payload(
                check.nearest_path_projection
            )
            payload["nearest_blocker_obstacle"] = self._point_payload(
                check.nearest_blocker_obstacle
            )
            payload["nearest_blocker_path_distance"] = (
                check.nearest_blocker_path_distance
            )
            payload["nearest_blocker_z_distance"] = check.nearest_blocker_z_distance
            payload["nearest_blocker_z_delta"] = check.nearest_blocker_z_delta
        return payload

    def _set_state(self, payload: dict[str, object]) -> None:
        self.current_status = str(payload.get("status", "unknown"))
        self.current_action = str(payload.get("action", "none"))
        self.current_path_blocked = bool(payload.get("path_blocked", False))
        frozen = Bool()
        frozen.data = self.navigation_enabled and self.current_action == "stop"
        self.execution_frozen_pub.publish(frozen)
        self._publish_safety_speed_scale(self._current_speed_scale())
        self._log_state_reason(payload, frozen.data)

    def _current_speed_scale(self) -> float:
        if not self.navigation_enabled or self.current_action == "stop":
            return 0.0
        if self.current_action == "slow":
            return max(0.0, min(float(self.slow_speed_scale), 1.0))
        return 1.0

    def _publish_safety_speed_scale(self, scale: float) -> None:
        msg = Float64()
        msg.data = max(0.0, min(float(scale), 1.0))
        self.safety_speed_scale_pub.publish(msg)

    def _publish_status(self, payload: dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_pub.publish(msg)

    def _fmt_optional(self, value: object) -> str:
        if value is None:
            return "none"
        if isinstance(value, (int, float)):
            return f"{float(value):.3f}"
        return str(value)

    def _fmt_point(self, value: object) -> str:
        if not isinstance(value, dict):
            return "none"
        return (
            f"({self._fmt_optional(value.get('x'))},"
            f"{self._fmt_optional(value.get('y'))},"
            f"{self._fmt_optional(value.get('z'))})"
        )

    def _point_payload(self, point: Optional[Point3]) -> Optional[dict[str, float]]:
        if point is None:
            return None
        return {"x": point.x, "y": point.y, "z": point.z}

    def _log_state_reason(self, payload: dict[str, object], frozen: bool) -> None:
        now = float(payload.get("timestamp", self._now_sec()) or self._now_sec())
        status = str(payload.get("status", "unknown"))
        action = str(payload.get("action", "none"))
        message = str(payload.get("message", ""))
        stop_reason = str(payload.get("stop_reason", ""))
        path_blocked = bool(payload.get("path_blocked", False))
        key = (status, action, path_blocked, message, stop_reason)

        changed = key != self.last_state_log_key
        repeat_due = now - self.last_state_log_time >= 2.0
        if not changed and not (action == "stop" and repeat_due):
            return

        self.last_state_log_key = key
        self.last_state_log_time = now
        if stop_reason in ("path_blocked_stop", "path_blocker_distance_stop"):
            log_prefix = "动态避障路径阻塞停车"
        else:
            log_prefix = "动态避障状态"

        text = (
            f"{log_prefix}: status={status} action={action} frozen={frozen} "
            f"path_blocked={path_blocked} reason={message or 'none'} "
            f"stop_reason={stop_reason or 'none'} "
            f"path_source={payload.get('path_source', 'none')} "
            f"has_path={payload.get('has_path', False)} "
            f"has_global_path={payload.get('has_global_path', False)} "
            f"has_scan_execution_path={payload.get('has_scan_execution_path', False)} "
            f"execution_path_age={self._fmt_optional(payload.get('execution_path_age'))} "
            f"execution_callback_gap={self._fmt_optional(payload.get('execution_path_callback_gap'))} "
            f"execution_path_generation_matches={payload.get('execution_path_generation_matches', False)} "
            f"execution_path_spatially_matches={payload.get('execution_path_spatially_matches', False)} "
            f"execution_start_dist={self._fmt_optional(payload.get('execution_path_start_distance'))} "
            f"execution_end_dist={self._fmt_optional(payload.get('execution_path_end_distance'))} "
            f"execution_len={self._fmt_optional(payload.get('execution_path_length'))} "
            f"global_span={self._fmt_optional(payload.get('global_path_span'))} "
            f"obstacle_points={payload.get('obstacle_points', 0)} "
            f"safety_check_ms={self._fmt_optional(payload.get('safety_check_ms'))} "
            f"stop_distance={self.stop_distance:.3f} "
            f"corridor_radius={self.path_corridor_radius:.3f} "
            f"z_tolerance={self.z_tolerance:.3f} "
            f"z_tolerance_down={self.z_tolerance_down:.3f} "
            f"z_tolerance_up={self.z_tolerance_up:.3f} "
            f"robot_to_path_distance={self._fmt_optional(payload.get('robot_to_path_distance'))} "
            f"nearest_obstacle_distance={self._fmt_optional(payload.get('nearest_obstacle_distance'))} "
            f"nearest_obstacle_point={self._fmt_point(payload.get('nearest_obstacle_point'))} "
            f"nearest_obstacle_map_offset=({self._fmt_optional(payload.get('nearest_obstacle_dx'))},"
            f"{self._fmt_optional(payload.get('nearest_obstacle_dy'))},"
            f"{self._fmt_optional(payload.get('nearest_obstacle_dz'))}) "
            f"nearest_obstacle_body_xy=({self._fmt_optional(payload.get('nearest_obstacle_body_x'))},"
            f"{self._fmt_optional(payload.get('nearest_obstacle_body_y'))}) "
            f"nearest_obstacle_side={payload.get('nearest_obstacle_side', 'none')} "
            f"nearest_blocker={self._fmt_optional(payload.get('nearest_blocker_distance'))} "
            f"nearest_path_xy={self._fmt_optional(payload.get('nearest_path_distance'))} "
            f"nearest_path_z={self._fmt_optional(payload.get('nearest_path_z_distance'))} "
            f"nearest_path_z_delta={self._fmt_optional(payload.get('nearest_path_z_delta'))} "
            f"nearest_path_obstacle={self._fmt_point(payload.get('nearest_path_obstacle'))} "
            f"nearest_path_projection={self._fmt_point(payload.get('nearest_path_projection'))} "
            f"nearest_blocker_path_xy={self._fmt_optional(payload.get('nearest_blocker_path_distance'))} "
            f"nearest_blocker_z={self._fmt_optional(payload.get('nearest_blocker_z_distance'))} "
            f"nearest_blocker_z_delta={self._fmt_optional(payload.get('nearest_blocker_z_delta'))} "
            f"nearest_blocker_obstacle={self._fmt_point(payload.get('nearest_blocker_obstacle'))} "
            f"blocked_duration={self._fmt_optional(payload.get('blocked_duration'))}"
        )
        if action == "stop":
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def _log_diagnostics(
        self, payload: dict[str, object], check: ObstacleCheck
    ) -> None:
        now = float(payload.get("timestamp", self._now_sec()) or self._now_sec())
        if now - self.last_diagnostic_log_time < self.diagnostic_log_period:
            return
        self.last_diagnostic_log_time = now

        xy_margin = None
        z_margin = None
        if check.nearest_path_distance is not None:
            xy_margin = check.nearest_path_distance - self.path_corridor_radius
        if check.nearest_path_z_delta is not None:
            if check.nearest_path_z_delta >= 0.0:
                z_margin = check.nearest_path_z_delta - self.z_tolerance_up
            else:
                z_margin = -check.nearest_path_z_delta - self.z_tolerance_down
        stop_reason = str(payload.get("stop_reason", ""))
        if stop_reason in ("path_blocked_stop", "path_blocker_distance_stop"):
            log_prefix = "动态避障路径阻塞诊断"
        elif check.path_blocked:
            log_prefix = "动态避障路径障碍诊断"
        else:
            log_prefix = "动态避障诊断"

        text = (
            f"{log_prefix}: "
            f"status={payload.get('status', 'unknown')} "
            f"action={payload.get('action', 'none')} "
            f"path_blocked={payload.get('path_blocked', False)} "
            f"stop_reason={stop_reason or 'none'} "
            f"path_source={payload.get('path_source', 'none')} "
            f"raw_obstacles={self.last_raw_obstacle_count} "
            f"transformed_obstacles={self.last_transformed_obstacle_count} "
            f"used_obstacles={payload.get('obstacle_points', 0)} "
            f"obstacle_frame={self.last_obstacle_frame or 'none'} "
            f"self_filtered={check.self_filtered_count} "
            f"blocker_count={check.blocker_count} "
            f"stop_distance={self.stop_distance:.3f} "
            f"corridor_radius={self.path_corridor_radius:.3f} "
            f"z_tolerance={self.z_tolerance:.3f} "
            f"z_tolerance_down={self.z_tolerance_down:.3f} "
            f"z_tolerance_up={self.z_tolerance_up:.3f} "
            f"robot_self_clear_radius={self.robot_self_clear_radius:.3f} "
            f"robot_to_path={self._fmt_optional(payload.get('robot_to_path_distance'))} "
            f"local_path_size={payload.get('local_path_size', 0)} "
            f"nearest_obstacle_distance={self._fmt_optional(check.nearest_obstacle_distance)} "
            f"nearest_obstacle_point={self._fmt_point(payload.get('nearest_obstacle_point'))} "
            f"nearest_obstacle_map_offset=({self._fmt_optional(payload.get('nearest_obstacle_dx'))},"
            f"{self._fmt_optional(payload.get('nearest_obstacle_dy'))},"
            f"{self._fmt_optional(payload.get('nearest_obstacle_dz'))}) "
            f"nearest_obstacle_body_xy=({self._fmt_optional(payload.get('nearest_obstacle_body_x'))},"
            f"{self._fmt_optional(payload.get('nearest_obstacle_body_y'))}) "
            f"nearest_obstacle_side={payload.get('nearest_obstacle_side', 'none')} "
            f"nearest_path_xy={self._fmt_optional(check.nearest_path_distance)} "
            f"nearest_path_z={self._fmt_optional(check.nearest_path_z_distance)} "
            f"nearest_path_z_delta={self._fmt_optional(check.nearest_path_z_delta)} "
            f"xy_margin={self._fmt_optional(xy_margin)} "
            f"z_margin={self._fmt_optional(z_margin)} "
            f"nearest_path_obstacle={self._fmt_point(payload.get('nearest_path_obstacle'))} "
            f"nearest_path_projection={self._fmt_point(payload.get('nearest_path_projection'))} "
            f"nearest_blocker_robot_dist={self._fmt_optional(check.nearest_blocker_distance)} "
            f"nearest_blocker_z_delta={self._fmt_optional(check.nearest_blocker_z_delta)} "
            f"execution_age={self._fmt_optional(payload.get('execution_path_age'))} "
            f"execution_callback_gap={self._fmt_optional(payload.get('execution_path_callback_gap'))} "
            f"safety_check_ms={self._fmt_optional(payload.get('safety_check_ms'))} "
            f"execution_generation_matches={payload.get('execution_path_generation_matches', False)} "
            f"execution_spatially_matches={payload.get('execution_path_spatially_matches', False)}"
        )
        if check.path_blocked:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def _publish_stop_if_needed(self) -> None:
        if self.publish_zero_on_stop and self.current_action == "stop":
            self.cmd_vel_safe_pub.publish(Twist())

    def _filter_twist(self, msg: Twist) -> Twist:
        if self.current_action == "stop":
            return Twist()
        filtered = Twist()
        scale = self.slow_speed_scale if self.current_action == "slow" else 1.0
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
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
