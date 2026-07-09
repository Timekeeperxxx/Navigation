#!/usr/bin/env python3
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class CloudPoint:
    x: float
    y: float
    z: float
    source: str


@dataclass(frozen=True)
class SnapResult:
    point: CloudPoint
    xy_distance: float


class Rviz2DGoalPlanner(Node):
    """Use RViz 2D Goal clicks as alternating snapped start and goal poses."""

    def __init__(self) -> None:
        super().__init__("rviz_2d_goal_planner")
        self.declare_parameter("input_goal_topic", "/rviz_2d_goal")
        self.declare_parameter("input_point_topic", "/rviz_ground_point")
        self.declare_parameter("output_goal_pose_topic", "/goal_pose")
        self.declare_parameter("ground_pcd", "")
        self.declare_parameter("planground_pcd", "")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("max_snap_xy_distance", 0.6)
        self.declare_parameter("snap_xy_to_cloud_point", True)
        self.declare_parameter("snap_prefer_source", "ground")
        self.declare_parameter("planner_ready_topic", "/nav/planner_ready")
        self.declare_parameter("tf_publish_period", 0.05)

        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.max_snap_xy_distance = float(
            self.get_parameter("max_snap_xy_distance").value
        )
        self.snap_xy_to_cloud_point = bool(
            self.get_parameter("snap_xy_to_cloud_point").value
        )
        self.snap_prefer_source = str(
            self.get_parameter("snap_prefer_source").value
        ).strip().lower()

        self.points_by_source = self._load_points()
        self.points = [
            point
            for points in self.points_by_source.values()
            for point in points
        ]
        if not self.points:
            raise RuntimeError("ground_pcd/planground_pcd 没有加载到可吸附点")

        self.waiting_for_start = True
        self.planner_ready = False
        self.start_pose: PoseStamped | None = None
        self.goal_pose: PoseStamped | None = None
        self.pending_goal_pose: PoseStamped | None = None
        self.tf_broadcaster = TransformBroadcaster(self)

        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/rviz_2d_goal_markers", marker_qos
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("output_goal_pose_topic").value), 10
        )
        ready_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_sub = self.create_subscription(
            Bool,
            str(self.get_parameter("planner_ready_topic").value),
            self._on_planner_ready,
            ready_qos,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("input_goal_topic").value),
            self._on_goal,
            10,
        )
        self.point_sub = self.create_subscription(
            PointStamped,
            str(self.get_parameter("input_point_topic").value),
            self._on_point,
            10,
        )
        self.tf_timer = self.create_timer(
            max(float(self.get_parameter("tf_publish_period").value), 0.02),
            self._publish_start_tf,
        )

        self.get_logger().info(
            "RViz 点选测试节点已启动：推荐用 Publish Point 在 ground 点云上点选；第一次设置起点，第二次设置终点并规划。"
        )

    def _load_points(self) -> dict[str, list[CloudPoint]]:
        points_by_source: dict[str, list[CloudPoint]] = {}
        planground_pcd = str(self.get_parameter("planground_pcd").value).strip()
        ground_pcd = str(self.get_parameter("ground_pcd").value).strip()

        if planground_pcd:
            planground_points = self._read_pcd(Path(planground_pcd), "planground")
            points_by_source["planground"] = planground_points
            self.get_logger().info(
                f"加载 footpoint/planground 吸附点：{planground_pcd}, points={len(planground_points)}"
            )
        if ground_pcd:
            ground_points = self._read_pcd(Path(ground_pcd), "ground")
            points_by_source["ground"] = ground_points
            self.get_logger().info(
                f"加载 ground 吸附点：{ground_pcd}, points={len(ground_points)}"
            )
        return points_by_source

    def _on_goal(self, msg: PoseStamped) -> None:
        snap = self._nearest_xy(msg.pose.position.x, msg.pose.position.y)
        pose = self._snapped_pose(msg, snap)
        self._handle_snapped_pose(pose, snap, source_name="2D Goal")

    def _on_point(self, msg: PointStamped) -> None:
        snap = self._nearest_xy(msg.point.x, msg.point.y)
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = snap.point.x
        pose.pose.position.y = snap.point.y
        pose.pose.position.z = snap.point.z
        pose.pose.orientation.w = 1.0
        self._handle_snapped_pose(pose, snap, source_name="Publish Point")

    def _handle_snapped_pose(
        self, pose: PoseStamped, snap: SnapResult, source_name: str
    ) -> None:
        role = "start" if self.waiting_for_start else "goal"

        if snap.xy_distance > self.max_snap_xy_distance:
            self.get_logger().warn(
                f"{source_name} {role} 点击点离最近点云点较远：xy_distance={snap.xy_distance:.3f}m, "
                f"仍吸附到最近 {snap.point.source} 点；建议直接点在蓝色 ground 或白色 footpoint 点云上"
            )

        if self.waiting_for_start:
            self.start_pose = pose
            self.waiting_for_start = False
            self.get_logger().info(
                f"已通过 {source_name} 设置起点："
                f"({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}), "
                "下一次点选将作为终点并触发规划"
            )
            self.pending_goal_pose = None
            self._publish_start_tf()
        else:
            self.goal_pose = pose
            self._publish_goal_when_ready(pose)
            self.waiting_for_start = True

        self._publish_markers()

    def _on_planner_ready(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self.planner_ready:
            return
        self.planner_ready = True
        self.get_logger().info("planner 已 ready，可以发送终点")
        if self.pending_goal_pose is not None:
            pending = self.pending_goal_pose
            self.pending_goal_pose = None
            self.goal_pub.publish(pending)
            self.get_logger().info(
                "已自动发送缓存终点并触发规划："
                f"({pending.pose.position.x:.3f}, {pending.pose.position.y:.3f}, {pending.pose.position.z:.3f})"
            )

    def _publish_goal_when_ready(self, pose: PoseStamped) -> None:
        if self.planner_ready:
            self.pending_goal_pose = None
            self.goal_pub.publish(pose)
            self.get_logger().info(
                "已发布终点并触发规划："
                f"({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}); "
                "下一次 2D Goal 将重新设置起点"
            )
            return

        self.pending_goal_pose = pose
        self.get_logger().warn(
            "planner 还未 ready，已缓存终点："
            f"({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})；"
            "等待 /nav/planner_ready 后自动触发规划"
        )

    def _nearest_xy(self, x: float, y: float) -> SnapResult:
        if self.snap_prefer_source in self.points_by_source:
            preferred = self._nearest_xy_in_points(
                x, y, self.points_by_source[self.snap_prefer_source]
            )
            if preferred.xy_distance <= self.max_snap_xy_distance:
                return preferred
            self.get_logger().warn(
                f"首选 {self.snap_prefer_source} 吸附点距离 {preferred.xy_distance:.3f}m "
                "超过阈值，改用全部点云最近点"
            )
        return self._nearest_xy_in_points(x, y, self.points)

    def _nearest_xy_in_points(
        self, x: float, y: float, points: Iterable[CloudPoint]
    ) -> SnapResult:
        best = min(
            points,
            key=lambda point: (point.x - x) * (point.x - x)
            + (point.y - y) * (point.y - y),
        )
        distance = math.hypot(best.x - x, best.y - y)
        return SnapResult(point=best, xy_distance=distance)

    def _snapped_pose(self, source: PoseStamped, snap: SnapResult) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        if self.snap_xy_to_cloud_point:
            pose.pose.position.x = snap.point.x
            pose.pose.position.y = snap.point.y
        else:
            pose.pose.position.x = source.pose.position.x
            pose.pose.position.y = source.pose.position.y
        pose.pose.position.z = snap.point.z
        pose.pose.orientation = source.pose.orientation
        return pose

    def _publish_start_tf(self) -> None:
        if self.start_pose is None:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.global_frame
        transform.child_frame_id = self.robot_frame
        transform.transform.translation.x = self.start_pose.pose.position.x
        transform.transform.translation.y = self.start_pose.pose.position.y
        transform.transform.translation.z = self.start_pose.pose.position.z
        transform.transform.rotation = self.start_pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        if self.start_pose is not None:
            markers.markers.append(
                self._make_marker(
                    1, self.start_pose, "start", Marker.ARROW, (1.0, 0.55, 0.05, 1.0)
                )
            )
            markers.markers.append(
                self._make_marker(
                    2, self.start_pose, "start_z", Marker.SPHERE, (1.0, 0.55, 0.05, 1.0)
                )
            )
        if self.goal_pose is not None:
            markers.markers.append(
                self._make_marker(
                    3, self.goal_pose, "goal", Marker.ARROW, (1.0, 0.1, 0.1, 1.0)
                )
            )
            markers.markers.append(
                self._make_marker(
                    4, self.goal_pose, "goal_z", Marker.SPHERE, (1.0, 0.1, 0.1, 1.0)
                )
            )
        self.marker_pub.publish(markers)

    def _make_marker(
        self,
        marker_id: int,
        pose: PoseStamped,
        namespace: str,
        marker_type: int,
        color: tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.45 if marker_type == Marker.ARROW else 0.22
        marker.scale.y = 0.08 if marker_type == Marker.ARROW else 0.22
        marker.scale.z = 0.08 if marker_type == Marker.ARROW else 0.22
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _read_pcd(self, path: Path, source: str) -> list[CloudPoint]:
        data = path.read_bytes()
        data_offset = data.find(b"DATA ")
        if data_offset < 0:
            raise RuntimeError(f"PCD 缺少 DATA header：{path}")
        line_end = data.find(b"\n", data_offset)
        if line_end < 0:
            raise RuntimeError(f"PCD DATA header 不完整：{path}")

        header_text = data[: line_end + 1].decode("latin1")
        header: dict[str, list[str]] = {}
        for raw_line in header_text.splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            header[parts[0].upper()] = parts[1:]

        fields = header.get("FIELDS", [])
        sizes = [int(value) for value in header.get("SIZE", [])]
        types = header.get("TYPE", [])
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        points_count = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
        data_type = header.get("DATA", [""])[0].lower()
        if not {"x", "y", "z"}.issubset(set(fields)):
            raise RuntimeError(f"PCD 缺少 x/y/z 字段：{path}")

        body = data[line_end + 1 :]
        if data_type == "binary":
            return self._read_binary_points(
                body, fields, sizes, types, counts, points_count, source
            )
        if data_type == "ascii":
            return self._read_ascii_points(body.decode("utf-8", errors="replace"), fields, source)
        raise RuntimeError(f"不支持的 PCD DATA 类型：{data_type}")

    def _read_binary_points(
        self,
        body: bytes,
        fields: list[str],
        sizes: list[int],
        types: list[str],
        counts: list[int],
        points_count: int,
        source: str,
    ) -> list[CloudPoint]:
        offsets: dict[str, int] = {}
        point_step = 0
        for field, size, count in zip(fields, sizes, counts):
            offsets[field] = point_step
            point_step += size * count

        points: list[CloudPoint] = []
        for index in range(points_count):
            offset = index * point_step
            if offset + point_step > len(body):
                break
            points.append(
                CloudPoint(
                    x=self._unpack_value(body, offset + offsets["x"], fields, sizes, types, "x"),
                    y=self._unpack_value(body, offset + offsets["y"], fields, sizes, types, "y"),
                    z=self._unpack_value(body, offset + offsets["z"], fields, sizes, types, "z"),
                    source=source,
                )
            )
        return points

    def _unpack_value(
        self,
        body: bytes,
        offset: int,
        fields: list[str],
        sizes: list[int],
        types: list[str],
        field: str,
    ) -> float:
        idx = fields.index(field)
        size = sizes[idx]
        value_type = types[idx].upper()
        if value_type == "F" and size == 4:
            return float(struct.unpack_from("<f", body, offset)[0])
        if value_type == "F" and size == 8:
            return float(struct.unpack_from("<d", body, offset)[0])
        if value_type == "I" and size == 4:
            return float(struct.unpack_from("<i", body, offset)[0])
        if value_type == "U" and size == 4:
            return float(struct.unpack_from("<I", body, offset)[0])
        raise RuntimeError(f"不支持的 PCD 字段类型：{field} {value_type}{size}")

    def _read_ascii_points(
        self, body: str, fields: list[str], source: str
    ) -> list[CloudPoint]:
        x_idx = fields.index("x")
        y_idx = fields.index("y")
        z_idx = fields.index("z")
        points: list[CloudPoint] = []
        for line in body.splitlines():
            if not line.strip():
                continue
            values = line.split()
            points.append(
                CloudPoint(float(values[x_idx]), float(values[y_idx]), float(values[z_idx]), source)
            )
        return points


def main() -> None:
    rclpy.init()
    node = Rviz2DGoalPlanner()
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
