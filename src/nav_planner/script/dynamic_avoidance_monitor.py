#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Iterable, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
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
        # Keep only a two-voxel discretization margin here; a body-sized
        # corridor would inflate the same obstacle twice.
        self.declare_parameter("path_corridor_radius", 0.10)
        # The execution path is lifted to the body planning height.  A broad
        # vertical band includes traversable floor voxels below that path and
        # permanently freezes otherwise collision-free trajectories.
        self.declare_parameter("z_tolerance", 0.10)
        self.declare_parameter("stop_distance", 0.6)
        self.declare_parameter("slow_distance", 1.2)
        self.declare_parameter("robot_self_clear_radius", 0.90)
        self.declare_parameter("replan_blocked_duration", 2.0)
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
        self.z_tolerance = float(self.get_parameter("z_tolerance").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.slow_distance = float(self.get_parameter("slow_distance").value)
        self.robot_self_clear_radius = max(
            float(self.get_parameter("robot_self_clear_radius").value), 0.0
        )
        self.replan_blocked_duration = float(
            self.get_parameter("replan_blocked_duration").value
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
        self.last_obstacle_processing_time: Optional[float] = None
        self.blocked_since: Optional[float] = None
        self.last_replan_publish_time = 0.0
        self.replan_active = False
        self.current_status = "idle"
        self.current_action = "none"
        self.current_path_blocked = False
        self.navigation_enabled = not self.require_nav_start

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
        if window is None:
            self.blocked_since = None
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
            return Point3(float(t.x), float(t.y), float(t.z)), ""
        except TransformException as exc:
            if self.allow_path_start_without_tf and self.path_points:
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
        if self.last_obstacle_time is None:
            return True
        return now - self.last_obstacle_time > self.sensor_timeout

    def _check_obstacles(self, robot: Point3, forward_path: list[Point3]) -> ObstacleCheck:
        nearest_obstacle_distance: Optional[float] = None
        nearest_blocker_distance: Optional[float] = None
        nearest_path_distance: Optional[float] = None
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
        obstacle_check_radius = max(path_check_radius, nearby_check_radius)

        for obstacle in self.obstacles:
            robot_distance = self._distance_2d(robot, obstacle)
            # /grid_map/occupancy_inflate contains returns from the B2 body and
            # points inside its inflated footprint.  Treating those as external
            # obstacles makes the safety layer permanently report a blocker a
            # few centimetres from the robot and clamps every command to zero.
            if robot_distance <= self.robot_self_clear_radius:
                self_filtered_count += 1
                continue
            # The active window starts at most path_deviation_tolerance away
            # and contains at most lookahead_distance of path.  Points beyond
            # that geometric bound cannot block it.  Discard them before the
            # expensive point-to-every-segment calculation so the 5 Hz safety
            # timer cannot starve the incoming occupancy callback.
            if robot_distance > obstacle_check_radius:
                continue
            nearest_obstacle_distance = self._min_optional(
                nearest_obstacle_distance, robot_distance
            )
            path_distance, z_distance = self._distance_to_path(obstacle, forward_path)
            nearest_path_distance = self._min_optional(nearest_path_distance, path_distance)
            if path_distance <= self.path_corridor_radius and z_distance <= self.z_tolerance:
                blocker_count += 1
                nearest_blocker_distance = self._min_optional(
                    nearest_blocker_distance, robot_distance
                )

        return ObstacleCheck(
            path_blocked=blocker_count > 0,
            blocker_count=blocker_count,
            self_filtered_count=self_filtered_count,
            nearest_obstacle_distance=nearest_obstacle_distance,
            nearest_blocker_distance=nearest_blocker_distance,
            nearest_path_distance=nearest_path_distance,
        )

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
            if self.blocked_since is None:
                self.blocked_since = now
            blocked_duration = now - self.blocked_since
            if blocked_duration >= self.replan_blocked_duration:
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
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
