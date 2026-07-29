#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import zlib
from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from sensor_msgs_py import point_cloud2
except ImportError:  # pragma: no cover - 兼容极简 ROS 安装环境
    point_cloud2 = None

try:
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover - package.xml declares python3-scipy
    cKDTree = None


@dataclass
class Point3:
    x: float
    y: float
    z: float
    yaw: Optional[float] = None


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
    ground_supported: bool = True
    ground_probe_count: int = 0
    unsupported_ground_probe_count: int = 0
    nearest_unsupported_ground_distance: Optional[float] = None
    first_unsupported_ground_probe: Optional[Point3] = None
    ground_error: str = ""


@dataclass
class GroundSupportCheck:
    supported: bool
    probe_count: int
    unsupported_probe_count: int
    nearest_unsupported_distance: Optional[float]
    first_unsupported_probe: Optional[Point3]
    error: str = ""


class GroundSupportIndex:
    """Static, multi-floor-aware ground support lookup."""

    def __init__(self, xy_tolerance: float, z_tolerance: float) -> None:
        self.xy_tolerance = max(float(xy_tolerance), 0.01)
        self.z_tolerance = max(float(z_tolerance), 0.01)
        self._tree: object | None = None
        self._z_values = np.empty(0, dtype=np.float64)
        self.point_count = 0
        self.error = ""

    @property
    def ready(self) -> bool:
        return self._tree is not None and self.point_count > 0

    def invalidate(self, error: str) -> None:
        self._tree = None
        self._z_values = np.empty(0, dtype=np.float64)
        self.point_count = 0
        self.error = error

    def set_points(self, points: np.ndarray) -> bool:
        if cKDTree is None:
            self.invalidate("缺少 python3-scipy，无法建立 ground 支撑索引")
            return False

        array = np.asarray(points, dtype=np.float64)
        if array.ndim != 2 or array.shape[1] < 3:
            self.invalidate("ground 点云不是 Nx3 XYZ 数组")
            return False
        xyz = np.ascontiguousarray(array[:, :3])
        xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
        if len(xyz) == 0:
            self.invalidate("ground 点云没有有限 XYZ 点")
            return False

        try:
            next_tree = cKDTree(np.ascontiguousarray(xyz[:, :2]))
        except Exception as exc:  # pragma: no cover - scipy/allocation failure
            self.invalidate(f"建立 ground 支撑索引失败：{exc}")
            return False
        self._tree = next_tree
        self._z_values = np.ascontiguousarray(xyz[:, 2])
        self.point_count = int(len(xyz))
        self.error = ""
        return True

    def supported_mask(self, probes: np.ndarray) -> np.ndarray:
        array = np.asarray(probes, dtype=np.float64)
        if array.ndim != 2 or array.shape[1] < 3:
            return np.zeros(0, dtype=bool)
        if not self.ready:
            return np.zeros(len(array), dtype=bool)

        finite = np.all(np.isfinite(array[:, :3]), axis=1)
        result = np.zeros(len(array), dtype=bool)
        if not np.any(finite):
            return result
        try:
            candidates = self._tree.query_ball_point(  # type: ignore[union-attr]
                np.ascontiguousarray(array[finite, :2]),
                r=self.xy_tolerance,
                workers=1,
            )
        except Exception as exc:  # pragma: no cover - scipy runtime failure
            self.invalidate(f"查询 ground 支撑索引失败：{exc}")
            return result

        finite_indices = np.flatnonzero(finite)
        for output_index, point_candidates in zip(finite_indices, candidates):
            if not point_candidates:
                continue
            probe_z = float(array[output_index, 2])
            candidate_z = self._z_values[np.asarray(point_candidates, dtype=np.int64)]
            result[output_index] = bool(
                np.any(np.abs(candidate_z - probe_z) <= self.z_tolerance)
            )
        return result


class DynamicAvoidanceMonitor(Node):
    """裁剪全局路径，检测局部障碍，并输出安全状态和速度过滤结果。"""

    def __init__(self) -> None:
        super().__init__("dynamic_avoidance_monitor")

        self.declare_parameter("enabled", True)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("global_path_topic", "/global_path")
        self.declare_parameter("scan_execution_path_topic", "/scan/execution_path")
        self.declare_parameter("planning_status_topic", "/nav/planning_status")
        self.declare_parameter("hold_during_goal_switch", True)
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
        self.declare_parameter(
            "final_yaw_validation_topic", "/planning/final_yaw_validation"
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
        # Static ground support belongs to the global planner. Keep the local
        # execution monitor obstacle-only unless an explicit deployment opts
        # back into the legacy ground guard.
        self.declare_parameter("require_ground_support", False)
        self.declare_parameter("ground_topic", "/mapground")
        self.declare_parameter("ground_body_height", 0.32)
        self.declare_parameter("ground_support_xy_tolerance", 0.155)
        self.declare_parameter("ground_support_z_tolerance", 0.20)
        self.declare_parameter("ground_footprint_radius", 0.27)
        self.declare_parameter("ground_footprint_probe_margin", 0.19)
        self.declare_parameter("ground_perimeter_samples", 16)
        self.declare_parameter("ground_radial_samples", 2)
        self.declare_parameter(
            "ground_outer_ring_max_missing_per_circle", 3
        )
        self.declare_parameter("ground_map_static", True)
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
        # Ground support comes from a static map and does not need the
        # positive-obstacle anti-flicker debounce.  Request a replacement on
        # the first monitor tick, then require a short continuous-safe window
        # from a newer SCAN execution generation before releasing motion.
        self.declare_parameter("ground_replan_blocked_duration", 0.0)
        self.declare_parameter("ground_replan_clear_duration", 0.4)
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
        self.declare_parameter("minimum_nonzero_planar_speed", 0.15)
        self.declare_parameter("minimum_in_place_yaw_speed", 0.20)
        self.declare_parameter("slow_nearby_obstacles", True)
        self.declare_parameter("require_obstacle_stream", False)
        self.declare_parameter("allow_path_start_without_tf", False)
        self.declare_parameter("enable_cmd_vel_filter", True)
        self.declare_parameter("enforce_path_blocking", True)
        self.declare_parameter("require_nav_start", False)
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
        self.hold_during_goal_switch = bool(
            self.get_parameter("hold_during_goal_switch").value
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
        self.require_ground_support = bool(
            self.get_parameter("require_ground_support").value
        )
        self.ground_topic = str(self.get_parameter("ground_topic").value).strip()
        self.ground_body_height = max(
            float(self.get_parameter("ground_body_height").value), 0.0
        )
        self.ground_support_xy_tolerance = max(
            float(
                self.get_parameter("ground_support_xy_tolerance").value
            ),
            0.01,
        )
        self.ground_support_z_tolerance = max(
            float(
                self.get_parameter("ground_support_z_tolerance").value
            ),
            0.01,
        )
        self.ground_footprint_radius = max(
            float(self.get_parameter("ground_footprint_radius").value), 0.0
        )
        self.ground_footprint_probe_margin = max(
            float(
                self.get_parameter("ground_footprint_probe_margin").value
            ),
            0.0,
        )
        self.ground_perimeter_samples = max(
            int(self.get_parameter("ground_perimeter_samples").value), 4
        )
        self.ground_radial_samples = max(
            int(self.get_parameter("ground_radial_samples").value), 1
        )
        self.ground_outer_ring_max_missing_per_circle = min(
            max(
                int(
                    self.get_parameter(
                        "ground_outer_ring_max_missing_per_circle"
                    ).value
                ),
                0,
            ),
            self.ground_perimeter_samples,
        )
        self.ground_map_static = bool(
            self.get_parameter("ground_map_static").value
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
        self.ground_replan_blocked_duration = max(
            float(
                self.get_parameter("ground_replan_blocked_duration").value
            ),
            0.0,
        )
        self.ground_replan_clear_duration = max(
            float(
                self.get_parameter("ground_replan_clear_duration").value
            ),
            0.0,
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
        self.minimum_nonzero_planar_speed = max(
            0.0,
            float(
                self.get_parameter("minimum_nonzero_planar_speed").value
            ),
        )
        self.minimum_in_place_yaw_speed = max(
            0.0,
            float(
                self.get_parameter("minimum_in_place_yaw_speed").value
            ),
        )
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
        self.terminal_yaw_denied_generation: Optional[float] = None
        self.terminal_yaw_denied_reason = ""
        self.goal_switch_hold = False
        self.goal_switch_generation: Optional[int] = None
        self.goal_switch_status = "idle"
        self.goal_switch_message = ""
        self.goal_switch_started_at: Optional[float] = None
        self.goal_switch_candidate_global_path_time: Optional[float] = None
        self.goal_switch_ready_global_path_time: Optional[float] = None
        self.obstacles: list[Point3] = []
        self.ground_support = GroundSupportIndex(
            self.ground_support_xy_tolerance,
            self.ground_support_z_tolerance,
        )
        self.ground_source_point_count = 0
        self.ground_source_signature: object | None = None
        self.ground_frame = ""
        self.ground_error = ""
        self.last_obstacle_time: Optional[float] = None
        self.last_sensor_heartbeat_time: Optional[float] = None
        self.last_obstacle_processing_time: Optional[float] = None
        self.blocked_since: Optional[float] = None
        self.clear_since: Optional[float] = None
        self.blocked_kind: Optional[str] = None
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
        self.final_yaw_validation_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("final_yaw_validation_topic").value),
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
        planning_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("planning_status_topic").value),
            self._on_planning_status,
            planning_status_qos,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("obstacle_topic").value),
            self._on_obstacles,
            1,
        )
        if self.require_ground_support and self.ground_topic:
            ground_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                PointCloud2,
                self.ground_topic,
                self._on_ground,
                ground_qos,
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
            f"ground={self.ground_topic if self.require_ground_support else 'disabled'}, "
            f"require_obstacle_stream={self.require_obstacle_stream}, "
            f"obstacle_period={self.obstacle_processing_period:.2f}s, "
            f"max_obstacle_points={self.max_obstacle_points}"
        )

    def _on_global_path(self, msg: Path) -> None:
        # GlobalPlanner does not publish a B2 body-attitude contract.  Keep
        # treating this as a position-only reference and infer tangent yaw
        # when it is the active fallback path.
        points = self._points_from_path(msg, include_pose_yaw=False)
        self.path_points = points
        self.path_frame = self.global_frame
        self.last_global_path_time = self._now_sec()
        if (
            self.goal_switch_hold
            and points
            and (
                self.goal_switch_started_at is None
                or self.last_global_path_time >= self.goal_switch_started_at
            )
        ):
            self.goal_switch_candidate_global_path_time = (
                self.last_global_path_time
            )
            # GlobalPlanner publishes the path immediately before path_ready.
            # Accept either callback order, but never treat a path received
            # during queued/planning as executable until path_ready for the
            # current generation has also arrived.
            if self.goal_switch_status == "path_ready":
                self.goal_switch_ready_global_path_time = (
                    self.last_global_path_time
                )
        if not points:
            self.get_logger().warn("收到空的全局路径，动态避障进入 no_path 状态")

    def _on_execution_path(self, msg: Path) -> None:
        # Unlike /global_path, every valid pose orientation on SCAN's
        # execution path is the commanded B2 body yaw and therefore must drive
        # the asymmetric double-circle footprint sweep.
        self.execution_path_points = self._points_from_path(
            msg, include_pose_yaw=True
        )
        self.last_execution_path_time = self._now_sec()
        stamp = msg.header.stamp
        generation_time = float(stamp.sec) + float(stamp.nanosec) / 1e9
        next_generation_time = (
            generation_time if generation_time > 0.0 else self.last_execution_path_time
        )
        if (
            self.terminal_yaw_denied_generation is not None
            and abs(
                next_generation_time - self.terminal_yaw_denied_generation
            )
            > 1e-3
        ):
            self.terminal_yaw_denied_generation = None
            self.terminal_yaw_denied_reason = ""
        self.execution_path_generation_time = next_generation_time
        self._release_goal_switch_hold_if_ready()

    def _on_planning_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warn("忽略无法解析的 /nav/planning_status")
            return
        if not isinstance(payload, dict):
            return

        status = str(payload.get("status", "")).strip().lower()
        raw_generation = payload.get("generation")
        try:
            generation = int(raw_generation)
        except (TypeError, ValueError):
            generation = None

        if status in {"queued", "planning"}:
            if (
                generation is not None
                and self.goal_switch_generation is not None
                and generation < self.goal_switch_generation
            ):
                return
            # A fresh single-point goal is its own execution authorization.
            # /nav_start remains available for inspection-task lifecycle, but
            # its absence must never leave BotDog with a valid path and zero
            # output velocity.
            if not self.require_nav_start:
                self.navigation_enabled = True
            self.terminal_yaw_denied_generation = None
            self.terminal_yaw_denied_reason = ""
            if not self.hold_during_goal_switch:
                return

            is_new_generation = (
                not self.goal_switch_hold
                or generation != self.goal_switch_generation
            )
            self.goal_switch_hold = True
            self.goal_switch_generation = generation
            self.goal_switch_status = status
            self.goal_switch_message = str(payload.get("message", ""))
            if is_new_generation:
                self.goal_switch_started_at = self._now_sec()
                self.goal_switch_candidate_global_path_time = None
                self.goal_switch_ready_global_path_time = None
            # Stop in the callback instead of waiting for the periodic safety
            # check. The old SCAN spline may still be publishing commands.
            if self.enable_cmd_vel_filter:
                self.cmd_vel_safe_pub.publish(Twist())
            return

        if not self.goal_switch_hold:
            return
        if (
            generation is not None
            and self.goal_switch_generation is not None
            and generation != self.goal_switch_generation
        ):
            return

        self.goal_switch_status = status
        self.goal_switch_message = str(payload.get("message", ""))
        if status == "path_ready":
            candidate_time = self.goal_switch_candidate_global_path_time
            if (
                candidate_time is not None
                and (
                    self.goal_switch_started_at is None
                    or candidate_time >= self.goal_switch_started_at
                )
            ):
                self.goal_switch_ready_global_path_time = candidate_time
            self._release_goal_switch_hold_if_ready()
        # failed/rejected intentionally remain latched. Re-enabling an older
        # trajectory after a replacement goal failed would send the robot
        # toward a target the operator has already superseded.

    def _release_goal_switch_hold_if_ready(self) -> None:
        if not self.goal_switch_hold:
            return
        ready_time = self.goal_switch_ready_global_path_time
        if (
            ready_time is None
            or self.last_execution_path_time is None
            or self.execution_path_generation_time is None
            or self.last_execution_path_time < ready_time
            or self.execution_path_generation_time < ready_time - 0.2
            or not self._execution_path_matches_global()
            or not self._execution_path_has_motion()
        ):
            return

        self.goal_switch_hold = False
        self.goal_switch_status = "execution_ready"
        self.goal_switch_message = "新目标的 SCAN 执行轨迹已就绪"
        self.get_logger().info(
            "新目标的全局路径与 SCAN 执行轨迹已匹配，解除速度 hold"
        )

    def _execution_path_has_motion(self) -> bool:
        if len(self.execution_path_points) < 2 or len(self.path_points) < 2:
            return False
        execution_length = sum(
            self._distance_3d(start, end)
            for start, end in zip(
                self.execution_path_points[:-1], self.execution_path_points[1:]
            )
        )
        global_span = self._distance_3d(
            self.path_points[0], self.path_points[-1]
        )
        # SCAN publishes a fresh, stationary emergency spline while swapping
        # references. Its timestamp is new and its point can lie on the new
        # global path, so timestamp/spatial checks alone are insufficient.
        required_length = min(0.30, max(0.03, global_span * 0.25))
        return execution_length >= required_length

    def _points_from_path(
        self, msg: Path, *, include_pose_yaw: bool = False
    ) -> list[Point3]:
        frame_id = msg.header.frame_id or self.global_frame
        points: list[Point3] = []
        for pose in msg.poses:
            pose_frame = pose.header.frame_id or frame_id
            point = Point3(
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
                (
                    self._quaternion_yaw(
                        float(pose.pose.orientation.x),
                        float(pose.pose.orientation.y),
                        float(pose.pose.orientation.z),
                        float(pose.pose.orientation.w),
                    )
                    if include_pose_yaw
                    else None
                ),
            )
            transformed = self._transform_point(point, pose_frame)
            if transformed is not None:
                points.append(transformed)
        return points

    @staticmethod
    def _quaternion_yaw(
        qx: float, qy: float, qz: float, qw: float
    ) -> Optional[float]:
        values = (qx, qy, qz, qw)
        if not all(math.isfinite(value) for value in values):
            return None
        norm_sq = sum(value * value for value in values)
        if norm_sq <= 1e-12:
            # A default-constructed ROS orientation is all zeroes and does not
            # carry an attitude.  Preserve the legacy tangent-yaw fallback.
            return None
        inverse_norm = 1.0 / math.sqrt(norm_sq)
        qx, qy, qz, qw = (
            qx * inverse_norm,
            qy * inverse_norm,
            qz * inverse_norm,
            qw * inverse_norm,
        )
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        return yaw if math.isfinite(yaw) else None

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

    def _on_ground(self, msg: PointCloud2) -> None:
        frame_id = msg.header.frame_id or self.global_frame
        signature = (
            frame_id,
            int(msg.width),
            int(msg.height),
            int(msg.point_step),
            int(msg.row_step),
            bool(msg.is_bigendian),
            bool(msg.is_dense),
            tuple(
                (field.name, field.offset, field.datatype, field.count)
                for field in msg.fields
            ),
            len(msg.data),
            zlib.crc32(msg.data),
        )
        if (
            self.ground_map_static
            and self.ground_support.ready
            and signature == self.ground_source_signature
        ):
            return

        # A changed latched map supersedes the old one.  Invalidate first so
        # malformed/empty/wrong-frame updates can never leave the robot using a
        # stale floor index.
        self.ground_support.invalidate("收到新的 ground 点云，索引重建中")
        self.ground_source_point_count = 0
        self.ground_source_signature = None
        self.ground_frame = ""
        if frame_id != self.global_frame:
            self.ground_error = (
                f"ground 坐标系 {frame_id} 与 {self.global_frame} 不一致"
            )
            self.ground_support.error = self.ground_error
            self.get_logger().error(self.ground_error)
            return
        if point_cloud2 is None:
            self.ground_error = "缺少 sensor_msgs_py，无法读取 ground 点云"
            self.ground_support.error = self.ground_error
            self.get_logger().error(self.ground_error)
            return

        try:
            raw = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
            raw_array = np.asarray(raw)
            if raw_array.dtype.names:
                xyz = np.column_stack(
                    [
                        np.asarray(raw_array[field], dtype=np.float64).reshape(-1)
                        for field in ("x", "y", "z")
                    ]
                )
            else:
                xyz = np.asarray(raw_array, dtype=np.float64).reshape(-1, 3)
        except Exception as exc:  # pragma: no cover - malformed ROS message
            self.ground_error = f"读取 ground 点云失败：{exc}"
            self.ground_support.error = self.ground_error
            self.get_logger().error(self.ground_error)
            return

        if not self.ground_support.set_points(xyz):
            self.ground_error = self.ground_support.error
            self.get_logger().error(self.ground_error)
            return

        self.ground_source_point_count = self.ground_support.point_count
        self.ground_source_signature = signature
        self.ground_frame = frame_id
        self.ground_error = ""
        self.get_logger().info(
            "ground 硬约束索引就绪："
            f"points={self.ground_source_point_count}, "
            f"xy_tolerance={self.ground_support_xy_tolerance:.3f}m, "
            f"z_tolerance={self.ground_support_z_tolerance:.2f}m"
        )

    def _on_sensor_heartbeat(self, _msg: PointCloud2) -> None:
        # Use receipt time instead of the message stamp: a driver or bag can
        # legitimately use a different clock, while safety only needs to know
        # whether fresh fused LiDAR output is still arriving.
        self.last_sensor_heartbeat_time = self._now_sec()

    def _on_cmd_vel(self, msg: Twist) -> None:
        if not self.enable_cmd_vel_filter:
            return
        if self.goal_switch_hold or not self.navigation_enabled:
            self.cmd_vel_safe_pub.publish(Twist())
            return
        self.cmd_vel_safe_pub.publish(self._filter_twist(msg))

    def _on_nav_start(self, msg: Bool) -> None:
        if bool(msg.data):
            self.navigation_enabled = True
            return
        # In single-GoTo mode, a task lifecycle's /nav_start=false must not
        # disable later goal execution. /nav_stop remains the explicit stop,
        # and the next planning generation re-enables a new GoTo.
        if self.require_nav_start:
            self.navigation_enabled = False
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

        if self.goal_switch_hold:
            self._reset_block_tracking()
            status_payload = self._status_payload(
                "planning_hold",
                "stop",
                False,
                now,
                robot=robot_point,
                message=(
                    self.goal_switch_message
                    or "等待新目标的全局路径和 SCAN 执行轨迹"
                ),
            )
            self._set_state(status_payload)
            self._publish_status(status_payload)
            self._publish_stop_if_needed()
            return

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
        if (
            self.active_path_source == "scan_execution_path"
            and self._is_in_place_rotation_path(window.forward_points)
        ):
            # This generation-matched ACK is the only authorization for the
            # controller's first non-zero final-yaw command. A stale generic
            # safety Bool must never authorize a newly published sweep.
            self._publish_final_yaw_validation(
                not check.path_blocked and check.ground_supported
            )
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
            # Point3 intentionally stores only planar yaw rather than the full
            # source pose quaternion.  A general 3-D frame transform therefore
            # cannot preserve it reliably; fall back to tangent inference.
            None,
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
            if point.yaw is not None and math.isfinite(point.yaw):
                pose.pose.orientation.z = math.sin(0.5 * point.yaw)
                pose.pose.orientation.w = math.cos(0.5 * point.yaw)
            else:
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
        ground_check = self._check_ground_support(robot, footprint_paths)
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
            return ObstacleCheck(
                path_blocked=not ground_check.supported,
                blocker_count=0,
                self_filtered_count=0,
                nearest_obstacle_distance=None,
                nearest_blocker_distance=(
                    ground_check.nearest_unsupported_distance
                    if not ground_check.supported
                    else None
                ),
                nearest_path_distance=None,
                clearance_escape=False,
                ground_supported=ground_check.supported,
                ground_probe_count=ground_check.probe_count,
                unsupported_ground_probe_count=(
                    ground_check.unsupported_probe_count
                ),
                nearest_unsupported_ground_distance=(
                    ground_check.nearest_unsupported_distance
                ),
                first_unsupported_ground_probe=(
                    ground_check.first_unsupported_probe
                ),
                ground_error=ground_check.error,
            )

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
                path_blocked=not ground_check.supported,
                blocker_count=0,
                self_filtered_count=self_filtered_count,
                nearest_obstacle_distance=None,
                nearest_blocker_distance=(
                    ground_check.nearest_unsupported_distance
                    if not ground_check.supported
                    else None
                ),
                nearest_path_distance=None,
                clearance_escape=False,
                ground_supported=ground_check.supported,
                ground_probe_count=ground_check.probe_count,
                unsupported_ground_probe_count=(
                    ground_check.unsupported_probe_count
                ),
                nearest_unsupported_ground_distance=(
                    ground_check.nearest_unsupported_distance
                ),
                first_unsupported_ground_probe=(
                    ground_check.first_unsupported_probe
                ),
                ground_error=ground_check.error,
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
            path_blocked=blocker_count > 0 or not ground_check.supported,
            blocker_count=blocker_count,
            self_filtered_count=self_filtered_count,
            nearest_obstacle_distance=nearest_obstacle_distance,
            nearest_blocker_distance=self._min_optional(
                nearest_blocker_distance,
                ground_check.nearest_unsupported_distance
                if not ground_check.supported
                else None,
            ),
            nearest_path_distance=nearest_path_distance,
            clearance_escape=clearance_escape and ground_check.supported,
            ground_supported=ground_check.supported,
            ground_probe_count=ground_check.probe_count,
            unsupported_ground_probe_count=(
                ground_check.unsupported_probe_count
            ),
            nearest_unsupported_ground_distance=(
                ground_check.nearest_unsupported_distance
            ),
            first_unsupported_ground_probe=(
                ground_check.first_unsupported_probe
            ),
            ground_error=ground_check.error,
        )

    def _check_ground_support(
        self, robot: Point3, footprint_paths: list[list[Point3]]
    ) -> GroundSupportCheck:
        if not getattr(self, "require_ground_support", False):
            return GroundSupportCheck(True, 0, 0, None, None)
        ground_support = getattr(self, "ground_support", None)
        if ground_support is None or not ground_support.ready:
            error = (
                getattr(self, "ground_error", "")
                or (ground_support.error if ground_support is not None else "")
                or "ground 支撑索引尚未就绪"
            )
            return GroundSupportCheck(
                False, 0, 1, 0.0, Point3(robot.x, robot.y, robot.z), error
            )
        if not footprint_paths:
            return GroundSupportCheck(
                False, 0, 1, 0.0, Point3(robot.x, robot.y, robot.z),
                "没有可检查的双圆轨迹",
            )

        path_height_offset = (
            self.ground_body_height
            if getattr(
                self, "active_path_source", "scan_execution_path"
            )
            == "scan_execution_path"
            else 0.0
        )
        probe_radius = (
            self.ground_footprint_radius
            + self.ground_footprint_probe_margin
        )
        probes: list[tuple[float, float, float]] = []
        probe_distances: list[float] = []
        # Each tuple contains the strict centre/inner probe indices followed
        # by the tolerant outer-ring indices for one circle at one path pose.
        probe_groups: list[tuple[list[int], list[int]]] = []
        two_pi = 2.0 * math.pi
        for circle_path in footprint_paths:
            for center in circle_path:
                ground_z = center.z - path_height_offset
                strict_indices: list[int] = []
                outer_indices: list[int] = []
                strict_indices.append(len(probes))
                probes.append((center.x, center.y, ground_z))
                probe_distances.append(
                    max(
                        0.0,
                        self._distance_2d(robot, center)
                        - self.ground_footprint_radius,
                    )
                )
                for ring in range(1, self.ground_radial_samples + 1):
                    ring_radius = (
                        probe_radius * ring / self.ground_radial_samples
                    )
                    ring_samples = (
                        self.ground_perimeter_samples
                        if ring == self.ground_radial_samples
                        else max(4, self.ground_perimeter_samples // 2)
                    )
                    for sample in range(ring_samples):
                        angle = two_pi * sample / ring_samples
                        probe_index = len(probes)
                        probes.append(
                            (
                                center.x + ring_radius * math.cos(angle),
                                center.y + ring_radius * math.sin(angle),
                                ground_z,
                            )
                        )
                        probe_distances.append(
                            max(
                                0.0,
                                self._distance_2d(robot, center)
                                - self.ground_footprint_radius,
                            )
                        )
                        if ring == self.ground_radial_samples:
                            outer_indices.append(probe_index)
                        else:
                            strict_indices.append(probe_index)
                probe_groups.append((strict_indices, outer_indices))

        if not probes:
            return GroundSupportCheck(
                False, 0, 1, 0.0, Point3(robot.x, robot.y, robot.z),
                "双圆轨迹中没有可检查的探针",
            )

        probe_array = np.asarray(probes, dtype=np.float64)
        supported = ground_support.supported_mask(probe_array)
        unsupported_indices = np.flatnonzero(~supported)
        failed = False
        for strict_indices, outer_indices in probe_groups:
            if any(not supported[index] for index in strict_indices):
                failed = True
                break
            outer_missing = sum(
                not supported[index] for index in outer_indices
            )
            if (
                outer_missing
                > self.ground_outer_ring_max_missing_per_circle
            ):
                failed = True
                break

        if not failed:
            return GroundSupportCheck(
                True,
                len(probes),
                int(len(unsupported_indices)),
                None,
                None,
                ground_support.error,
            )

        # A rejected circle is reported using the closest raw unsupported
        # probe so RViz/status diagnostics still point at the relevant area.
        first_index = min(
            (int(index) for index in unsupported_indices),
            key=lambda index: probe_distances[index],
        )
        nearest_distance = min(
            probe_distances[int(index)] for index in unsupported_indices
        )
        first_probe = probe_array[first_index]
        return GroundSupportCheck(
            supported=False,
            probe_count=len(probes),
            unsupported_probe_count=int(len(unsupported_indices)),
            nearest_unsupported_distance=float(nearest_distance),
            first_unsupported_probe=Point3(
                float(first_probe[0]),
                float(first_probe[1]),
                float(first_probe[2]),
            ),
            error=ground_support.error,
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
        robot_on_path_slice = Point3(
            robot.x, robot.y, forward_path[0].z, self.robot_yaw
        )
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
        explicit_yaws = [point.yaw for point in path]
        if explicit_yaws and all(
            yaw is not None and math.isfinite(yaw) for yaw in explicit_yaws
        ):
            # /scan/execution_path supplies the actual B2 body attitude.  Do
            # not replace it with the position-spline tangent: the robot can
            # rotate in place or keep a fixed heading while translating.
            return [
                self._normalize_angle(float(yaw))
                for yaw in explicit_yaws
                if yaw is not None
            ]

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

    def _is_in_place_rotation_path(self, path: list[Point3]) -> bool:
        if len(path) < 2:
            return False
        anchor = path[0]
        if any(self._distance_2d(anchor, point) > 1e-3 for point in path[1:]):
            return False
        explicit_yaws = [point.yaw for point in path]
        if not all(
            yaw is not None and math.isfinite(yaw) for yaw in explicit_yaws
        ):
            return False
        yaw_travel = sum(
            abs(self._normalize_angle(float(end) - float(start)))
            for start, end in zip(explicit_yaws[:-1], explicit_yaws[1:])
            if start is not None and end is not None
        )
        return yaw_travel > math.radians(1.0)

    def _publish_final_yaw_validation(self, approved: bool) -> None:
        generation = self.execution_path_generation_time
        if generation is None or not math.isfinite(generation):
            return
        msg = Float64MultiArray()
        msg.data = [float(generation), 1.0 if approved else 0.0]
        self.final_yaw_validation_pub.publish(msg)

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
        # Missing/unsupported ground is a negative obstacle.  It is always a
        # hard stop, independent of obstacle distance, escape logic, or the
        # optional positive-obstacle enforcement switch.
        if not check.ground_supported:
            self.clear_since = None
            if check.ground_error:
                # Replanning cannot repair a missing/corrupt static map. Keep
                # execution frozen without flooding SCAN until a valid latched
                # ground index is available again.
                if self.replan_active:
                    self._publish_replan(False, now)
                self._reset_block_tracking()
                return self._status_payload(
                    "ground_unavailable",
                    "stop",
                    True,
                    now,
                    robot=robot,
                    window=window,
                    check=check,
                    message=f"ground 硬约束不可用：{check.ground_error}",
                )
            if (
                self.active_path_source == "scan_execution_path"
                and self._is_in_place_rotation_path(window.forward_points)
            ):
                if self.replan_active:
                    self._publish_replan(False, now)
                self._reset_block_tracking()
                self.terminal_yaw_denied_generation = (
                    self.execution_path_generation_time
                )
                self.terminal_yaw_denied_reason = "ground"
                return self._status_payload(
                    "goal_yaw_blocked",
                    "stop",
                    True,
                    now,
                    robot=robot,
                    window=window,
                    check=check,
                    message=(
                        "已到达目标位置，但目标朝向旋转缺少同层 ground "
                        "支撑；已跳过朝向对齐，不请求路径重规划"
                    ),
                )
            if self.blocked_kind != "ground":
                self.blocked_since = now
                self.clear_since = None
                self.blocked_kind = "ground"
                self.blocked_execution_generation_time = None
                self.replan_confirmation_required = False
            elif self.blocked_since is None:
                self.blocked_since = now
            blocked_duration = now - self.blocked_since
            if blocked_duration >= self.ground_replan_blocked_duration:
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
                    message="轨迹离开同层 ground 可通行区域，已请求重规划",
                )
            return self._status_payload(
                "blocked",
                "stop",
                True,
                now,
                robot=robot,
                window=window,
                check=check,
                blocked_duration=blocked_duration,
                message="轨迹足迹缺少同层 ground 支撑，禁止执行",
            )

        if (
            check.path_blocked
            and self.active_path_source == "scan_execution_path"
            and self._is_in_place_rotation_path(window.forward_points)
        ):
            if self.replan_active:
                self._publish_replan(False, now)
            self._reset_block_tracking()
            self.terminal_yaw_denied_generation = (
                self.execution_path_generation_time
            )
            self.terminal_yaw_denied_reason = "obstacle"
            return self._status_payload(
                "goal_yaw_blocked",
                "stop",
                True,
                now,
                robot=robot,
                window=window,
                check=check,
                message=(
                    "已到达目标位置，但目标朝向的双圆旋转扫掠被障碍占用；"
                    "已跳过朝向对齐，不请求路径重规划"
                ),
            )

        terminal_yaw_was_skipped = (
            self.terminal_yaw_denied_generation is not None
            and self.execution_path_generation_time is not None
            and abs(
                self.execution_path_generation_time
                - self.terminal_yaw_denied_generation
            )
            <= 1e-3
            and self.active_path_source == "scan_execution_path"
            and not self._is_in_place_rotation_path(window.forward_points)
            and not check.path_blocked
            and check.ground_supported
        )
        if terminal_yaw_was_skipped:
            if self.replan_active:
                self._publish_replan(False, now)
            self._reset_block_tracking()
            return self._status_payload(
                "goal_yaw_skipped",
                "pass",
                False,
                now,
                robot=robot,
                window=window,
                check=check,
                message=(
                    "目标 XY 已到达；终点朝向因"
                    + (
                        "同层 ground 支撑不足"
                        if self.terminal_yaw_denied_reason == "ground"
                        else "双圆旋转扫掠遇障"
                    )
                    + "而安全跳过"
                ),
            )

        if check.path_blocked and self.enforce_path_blocking:
            if check.clearance_escape and check.ground_supported:
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
            if self.blocked_kind != "obstacle":
                self.blocked_since = now
                self.clear_since = None
                self.blocked_kind = "obstacle"
                self.blocked_execution_generation_time = None
                self.replan_confirmation_required = False
            elif self.blocked_since is None:
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
                    message=(
                        "ground 已恢复，等待新一代 SCAN ground-safe 执行轨迹"
                        if self.blocked_kind == "ground"
                        else "障碍短暂消失，等待更新后的安全轨迹"
                    ),
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
                self.ground_replan_clear_duration
                if (
                    self.replan_confirmation_required
                    and self.blocked_kind == "ground"
                )
                else self.blocked_clear_duration
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
                    message=(
                        "新一代 SCAN 轨迹正在连续 ground-safe 确认，保持停车"
                        if self.blocked_kind == "ground"
                        else "新轨迹正在连续安全确认，保持停车"
                    ),
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
            # A ground-triggered replan may start while the monitor has fallen
            # back to the global path.  In that case there is no old SCAN
            # timestamp to remember, but recovery must still wait for an
            # actual SCAN execution generation instead of clearing against the
            # global reference alone.
            if (
                self.blocked_kind == "ground"
                and self.replan_confirmation_required
            ):
                return (
                    self.active_path_source == "scan_execution_path"
                    and self.execution_path_generation_time is not None
                )
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
        self.blocked_kind = None
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
            "goal_switch_hold": self.goal_switch_hold,
            "goal_switch_generation": self.goal_switch_generation,
            "goal_switch_status": self.goal_switch_status,
            "terminal_yaw_denied_generation": (
                self.terminal_yaw_denied_generation
            ),
            "obstacle_points": len(self.obstacles),
            "ground_required": self.require_ground_support,
            "ground_ready": self.ground_support.ready,
            "ground_points": self.ground_support.point_count,
            "ground_error": self.ground_error or self.ground_support.error,
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
            payload["ground_supported"] = check.ground_supported
            payload["ground_probe_count"] = check.ground_probe_count
            payload["unsupported_ground_probe_count"] = (
                check.unsupported_ground_probe_count
            )
            payload["nearest_unsupported_ground_distance"] = (
                check.nearest_unsupported_ground_distance
            )
            payload["ground_error"] = (
                check.ground_error or payload["ground_error"]
            )
            if check.first_unsupported_ground_probe is not None:
                point = check.first_unsupported_ground_probe
                payload["first_unsupported_ground_probe"] = {
                    "x": point.x,
                    "y": point.y,
                    "z": point.z,
                }
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
        input_planar_speed = math.hypot(msg.linear.x, msg.linear.y)
        output_planar_speed = math.hypot(
            filtered.linear.x, filtered.linear.y
        )
        if (
            input_planar_speed > 1e-6
            and 1e-6 < output_planar_speed
            < self.minimum_nonzero_planar_speed
        ):
            floor_scale = (
                self.minimum_nonzero_planar_speed / output_planar_speed
            )
            filtered.linear.x *= floor_scale
            filtered.linear.y *= floor_scale
        # B2 SportClient exposes planar vx/vy/yaw only.  Never propagate
        # unsupported Twist axes even if an upstream node populates them.
        filtered.linear.z = 0.0
        filtered.angular.x = 0.0
        filtered.angular.y = 0.0
        filtered.angular.z = msg.angular.z * scale
        if (
            input_planar_speed <= 1e-6
            and abs(msg.angular.z) > 1e-6
            and 1e-6
            < abs(filtered.angular.z)
            < self.minimum_in_place_yaw_speed
        ):
            filtered.angular.z = math.copysign(
                self.minimum_in_place_yaw_speed,
                msg.angular.z,
            )
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

    def _min_optional(
        self, current: Optional[float], value: Optional[float]
    ) -> Optional[float]:
        if current is None:
            return value
        if value is None:
            return current
        return min(current, value)


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
