#!/usr/bin/env python3
"""Headless SCAN corridor-corner integration scenario.

Run this node together with scan_planner_node and closed_loop_controller in an
isolated ROS_DOMAIN_ID.  It replaces the physical robot and LiDAR with a planar
kinematic model and a synthetic world-frame point cloud.
"""

from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String
from tf2_ros import TransformBroadcaster


BODY_RADIUS = 0.27
BODY_CENTRE_OFFSETS = (-0.22, -0.63)
SENSOR_HEIGHT = 0.90
SIM_RATE_HZ = 50.0
CLOUD_RATE_HZ = 10.0
TEST_TIMEOUT_SEC = float(os.environ.get("SCAN_OFFLINE_TIMEOUT_SEC", "75.0"))
SCENARIO = os.environ.get("SCAN_OFFLINE_SCENARIO", "corner").strip().lower()


@dataclass(frozen=True)
class Wall:
    ax: float
    ay: float
    bx: float
    by: float


def rectangle_walls(
    min_x: float, min_y: float, max_x: float, max_y: float
) -> tuple[Wall, ...]:
    return (
        Wall(min_x, min_y, max_x, min_y),
        Wall(max_x, min_y, max_x, max_y),
        Wall(max_x, max_y, min_x, max_y),
        Wall(min_x, max_y, min_x, min_y),
    )


if SCENARIO == "dynamic":
    dynamic_half_width = float(os.environ.get("SCAN_OFFLINE_CORRIDOR_HALF_WIDTH", "1.0"))
    START = (0.80, 0.0)
    GOAL = (4.50, 0.0)
    REFERENCE_ROUTE = (START, GOAL)
    EXPECTED_OUTCOME = "goal"
    STATIC_WALLS = (
        Wall(-0.50, -dynamic_half_width, 5.30, -dynamic_half_width),
        Wall(-0.50, dynamic_half_width, 5.30, dynamic_half_width),
    )
    DYNAMIC_OBSTACLE_WALLS = (
        Wall(2.55, -0.14, 2.83, -0.14),
        Wall(2.83, -0.14, 2.83, 0.14),
        Wall(2.83, 0.14, 2.55, 0.14),
        Wall(2.55, 0.14, 2.55, -0.14),
    )
elif SCENARIO == "corner":
    START = (0.80, 0.0)
    GOAL = (3.0, 2.60)
    REFERENCE_ROUTE = (START, (3.0, 0.0), GOAL)
    EXPECTED_OUTCOME = "goal"
    STATIC_WALLS = (
        Wall(-0.50, -0.65, 3.65, -0.65),
        Wall(-0.50, 0.65, 2.35, 0.65),
        Wall(2.35, 0.65, 2.35, 3.20),
        Wall(3.65, -0.65, 3.65, 3.20),
    )
    DYNAMIC_OBSTACLE_WALLS = ()
elif SCENARIO == "doorway":
    START = (0.80, 0.0)
    GOAL = (4.80, 0.0)
    REFERENCE_ROUTE = (START, GOAL)
    EXPECTED_OUTCOME = "goal"
    STATIC_WALLS = (
        Wall(-0.50, -1.00, 5.40, -1.00),
        Wall(-0.50, 1.00, 5.40, 1.00),
        Wall(2.70, -1.00, 2.70, -0.50),
        Wall(2.70, 0.50, 2.70, 1.00),
    )
    DYNAMIC_OBSTACLE_WALLS = ()
elif SCENARIO == "s_bend":
    START = (0.80, 0.0)
    GOAL = (6.00, 0.0)
    REFERENCE_ROUTE = (
        START,
        (1.55, -0.48),
        (2.85, -0.48),
        (3.35, 0.48),
        (4.75, 0.48),
        (5.45, 0.0),
        GOAL,
    )
    EXPECTED_OUTCOME = "goal"
    STATIC_WALLS = (
        Wall(-0.50, -1.15, 6.60, -1.15),
        Wall(-0.50, 1.15, 6.60, 1.15),
        *rectangle_walls(2.00, 0.05, 2.40, 1.15),
        *rectangle_walls(3.80, -1.15, 4.20, -0.05),
    )
    DYNAMIC_OBSTACLE_WALLS = ()
elif SCENARIO == "dead_end":
    START = (0.80, 0.0)
    GOAL = (4.50, 0.0)
    REFERENCE_ROUTE = (START, GOAL)
    EXPECTED_OUTCOME = "safe_blocked_stop"
    STATIC_WALLS = (
        Wall(-0.50, -0.80, 5.20, -0.80),
        Wall(-0.50, 0.80, 5.20, 0.80),
        Wall(2.80, -0.80, 2.80, 0.80),
    )
    DYNAMIC_OBSTACLE_WALLS = ()
elif SCENARIO == "sensor_dropout":
    START = (0.80, 0.0)
    GOAL = (4.50, 0.0)
    REFERENCE_ROUTE = (START, GOAL)
    EXPECTED_OUTCOME = "sensor_stop"
    STATIC_WALLS = (
        Wall(-0.50, -1.00, 5.20, -1.00),
        Wall(-0.50, 1.00, 5.20, 1.00),
    )
    DYNAMIC_OBSTACLE_WALLS = ()
else:
    raise ValueError(f"unsupported SCAN_OFFLINE_SCENARIO={SCENARIO!r}")


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def point_segment_distance(px: float, py: float, wall: Wall) -> float:
    dx = wall.bx - wall.ax
    dy = wall.by - wall.ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(px - wall.ax, py - wall.ay)
    t = ((px - wall.ax) * dx + (py - wall.ay) * dy) / length_sq
    t = min(1.0, max(0.0, t))
    qx = wall.ax + t * dx
    qy = wall.ay + t * dy
    return math.hypot(px - qx, py - qy)


def footprint_clearance(
    x: float,
    y: float,
    yaw: float,
    walls: tuple[Wall, ...] = STATIC_WALLS,
) -> float:
    result = math.inf
    c = math.cos(yaw)
    s = math.sin(yaw)
    for offset in BODY_CENTRE_OFFSETS:
        cx = x + offset * c
        cy = y + offset * s
        result = min(
            result,
            min(point_segment_distance(cx, cy, wall) for wall in walls) - BODY_RADIUS,
        )
    return result


def sample_wall(wall: Wall, spacing: float = 0.05) -> list[tuple[float, float, float]]:
    length = math.hypot(wall.bx - wall.ax, wall.by - wall.ay)
    count = max(1, int(math.ceil(length / spacing)))
    result: list[tuple[float, float, float]] = []
    for index in range(count + 1):
        ratio = index / count
        x = wall.ax + ratio * (wall.bx - wall.ax)
        y = wall.ay + ratio * (wall.by - wall.ay)
        for z in (0.05, 0.15, 0.25, 0.35, 0.45, 0.55):
            result.append((x, y, z))
    return result


def interpolate_segment(
    start: tuple[float, float],
    end: tuple[float, float],
    spacing: float = 0.20,
) -> list[tuple[float, float]]:
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, int(math.ceil(length / spacing)))
    return [
        (
            start[0] + index / count * (end[0] - start[0]),
            start[1] + index / count * (end[1] - start[1]),
        )
        for index in range(count)
    ]


class OfflineCornerScenario(Node):
    def __init__(self) -> None:
        super().__init__("scan_offline_corner_scenario")
        self.x = START[0]
        self.y = START[1]
        self.yaw = 0.0
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.wz_cmd = 0.0
        self.last_cmd_ns = 0
        self.start_ns = self.get_clock().now().nanoseconds
        self.last_step_ns = self.start_ns
        self.path_published = False
        self.path_publish_ns = 0
        self.execution_path_count = 0
        self.command_count = 0
        self.occupancy_count = 0
        self.emergency_seen = False
        self.sensor_dropout_active = False
        self.sensor_dropout_ns = 0
        self.safety_status_counts: dict[str, int] = {}
        self.latest_safety_status: dict[str, object] = {}
        self.min_monitor_path_distance = math.inf
        self.obstacle_activated = False
        self.max_planar_command = 0.0
        self.max_abs_lateral_deviation = 0.0
        self.min_body_clearance = footprint_clearance(
            self.x, self.y, self.yaw, self._active_walls()
        )
        self.min_planned_clearance = math.inf
        self.min_planned_clearance_after_obstacle = math.inf
        self.max_body_z_error = 0.0
        self.finished = False
        self.passed = False

        self.body_pub = self.create_publisher(Odometry, "/sim/body_pose", 20)
        self.sensor_pub = self.create_publisher(Odometry, "/sim/sensor_pose", 20)
        self.cloud_pub = self.create_publisher(PointCloud2, "/sim/cloud", qos_profile_sensor_data)
        self.path_pub = self.create_publisher(Path, "/sim/initial_path", 5)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, "/sim/cmd_vel", self._on_cmd, 20)
        self.create_subscription(Path, "/scan/execution_path", self._on_execution_path, 10)
        self.create_subscription(PointCloud2, "/grid_map/occupancy_inflate", self._on_occupancy, 10)
        self.create_subscription(
            String, "/nav/obstacle_status", self._on_safety_status, 10
        )
        emergency_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            "/planning/scan_emergency_stop",
            self._on_emergency,
            emergency_qos,
        )

        self.wall_points = self._build_wall_points()
        self.create_timer(1.0 / SIM_RATE_HZ, self._step)
        self.create_timer(1.0 / CLOUD_RATE_HZ, self._publish_cloud)
        self.create_timer(0.25, self._maybe_publish_path)

    def _on_cmd(self, msg: Twist) -> None:
        self.vx_cmd = float(msg.linear.x)
        self.vy_cmd = float(msg.linear.y)
        self.wz_cmd = float(msg.angular.z)
        self.last_cmd_ns = self.get_clock().now().nanoseconds
        self.command_count += 1
        self.max_planar_command = max(
            self.max_planar_command,
            math.hypot(self.vx_cmd, self.vy_cmd),
        )

    def _on_occupancy(self, _msg: PointCloud2) -> None:
        self.occupancy_count += 1

    def _on_emergency(self, msg: Bool) -> None:
        self.emergency_seen = self.emergency_seen or bool(msg.data)

    def _on_safety_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        status = str(payload.get("status", "unknown"))
        self.safety_status_counts[status] = self.safety_status_counts.get(status, 0) + 1
        self.latest_safety_status = payload
        distance = payload.get("nearest_path_distance")
        if isinstance(distance, (int, float)):
            self.min_monitor_path_distance = min(
                self.min_monitor_path_distance, float(distance)
            )

    def _on_execution_path(self, msg: Path) -> None:
        self.execution_path_count += 1
        points = msg.poses
        if not points:
            return
        for index, pose_stamped in enumerate(points):
            if index + 1 < len(points):
                next_pose = points[index + 1].pose.position
                dx = next_pose.x - pose_stamped.pose.position.x
                dy = next_pose.y - pose_stamped.pose.position.y
            elif index > 0:
                prev_pose = points[index - 1].pose.position
                dx = pose_stamped.pose.position.x - prev_pose.x
                dy = pose_stamped.pose.position.y - prev_pose.y
            else:
                dx = math.cos(self.yaw)
                dy = math.sin(self.yaw)
            yaw = math.atan2(dy, dx) if dx * dx + dy * dy > 1e-10 else self.yaw
            clearance = footprint_clearance(
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                yaw,
                self._active_walls(),
            )
            self.min_planned_clearance = min(self.min_planned_clearance, clearance)
            if self.obstacle_activated:
                self.min_planned_clearance_after_obstacle = min(
                    self.min_planned_clearance_after_obstacle,
                    clearance,
                )

    def _active_walls(self) -> tuple[Wall, ...]:
        if self.obstacle_activated:
            return STATIC_WALLS + DYNAMIC_OBSTACLE_WALLS
        return STATIC_WALLS

    def _build_wall_points(self) -> list[tuple[float, float, float]]:
        return [
            point
            for wall in self._active_walls()
            for point in sample_wall(wall)
        ]

    def _make_odom(self, z: float) -> Odometry:
        now = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = z
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        return msg

    def _step(self) -> None:
        if self.finished:
            return
        now_ns = self.get_clock().now().nanoseconds
        dt = min(0.05, max(0.0, (now_ns - self.last_step_ns) / 1e9))
        self.last_step_ns = now_ns
        command_fresh = self.last_cmd_ns > 0 and (now_ns - self.last_cmd_ns) / 1e9 <= 0.30
        vx = self.vx_cmd if command_fresh else 0.0
        vy = self.vy_cmd if command_fresh else 0.0
        wz = self.wz_cmd if command_fresh else 0.0
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        self.x += (c * vx - s * vy) * dt
        self.y += (s * vx + c * vy) * dt
        self.yaw = normalize_angle(self.yaw + wz * dt)
        self.max_abs_lateral_deviation = max(
            self.max_abs_lateral_deviation,
            abs(self.y),
        )

        elapsed = (now_ns - self.start_ns) / 1e9
        if (
            SCENARIO == "dynamic"
            and self.path_published
            and not self.obstacle_activated
            and (elapsed >= 7.0 or self.x >= 1.55)
        ):
            self.obstacle_activated = True
            self.wall_points = self._build_wall_points()
            self.get_logger().warn(
                "dynamic obstacle inserted at x=[2.55,2.83], y=[-0.14,0.14]"
            )
        if (
            SCENARIO == "sensor_dropout"
            and self.path_published
            and not self.sensor_dropout_active
            and (elapsed >= 7.0 or self.x >= 1.55)
        ):
            self.sensor_dropout_active = True
            self.sensor_dropout_ns = now_ns
            self.get_logger().warn("synthetic LiDAR stream stopped")

        body = self._make_odom(0.0)
        body.twist.twist.linear.x = c * vx - s * vy
        body.twist.twist.linear.y = s * vx + c * vy
        body.twist.twist.angular.z = wz
        self.body_pub.publish(body)
        sensor = self._make_odom(SENSOR_HEIGHT)
        sensor.child_frame_id = "lidar"
        self.sensor_pub.publish(sensor)
        transform = TransformStamped()
        transform.header = body.header
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = body.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        self.max_body_z_error = max(self.max_body_z_error, abs(body.pose.pose.position.z))
        clearance = footprint_clearance(
            self.x,
            self.y,
            self.yaw,
            self._active_walls(),
        )
        self.min_body_clearance = min(self.min_body_clearance, clearance)
        if clearance < -0.02:
            self._finish(False, "simulated footprint crossed a corridor wall")
            return

        distance = math.hypot(self.x - GOAL[0], self.y - GOAL[1])
        if (
            EXPECTED_OUTCOME == "goal"
            and self.path_published
            and distance <= 0.20
        ):
            planned_clearance = (
                self.min_planned_clearance
                if math.isfinite(self.min_planned_clearance)
                else None
            )
            rejected_path_seen = (
                self.safety_status_counts.get("blocked", 0)
                + self.safety_status_counts.get("replan_requested", 0)
                > 0
            )
            planned_path_evidence = (
                planned_clearance is not None
                and (
                    planned_clearance >= -0.02
                    or rejected_path_seen
                )
            )
            dynamic_path_evidence = (
                math.isfinite(self.min_planned_clearance_after_obstacle)
                and (
                    self.min_planned_clearance_after_obstacle >= -0.02
                    or rejected_path_seen
                )
            )
            passed = (
                self.execution_path_count > 0
                and self.command_count > 0
                and self.occupancy_count > 0
                and self.min_body_clearance >= -0.02
                and planned_path_evidence
                and (
                    SCENARIO != "dynamic"
                    or (
                        self.obstacle_activated
                        and self.max_abs_lateral_deviation >= 0.20
                        and dynamic_path_evidence
                    )
                )
            )
            self._finish(passed, "goal reached" if passed else "goal reached without complete evidence")
            return

        if EXPECTED_OUTCOME == "safe_blocked_stop" and elapsed >= 35.0:
            stop_evidence = (
                self.safety_status_counts.get("blocked", 0)
                + self.safety_status_counts.get("replan_requested", 0)
                + self.safety_status_counts.get("waiting_replan", 0)
                >= 3
            )
            stopped = math.hypot(self.vx_cmd, self.vy_cmd) <= 1e-3
            passed = (
                self.path_published
                and self.occupancy_count > 0
                and self.min_body_clearance >= -0.02
                and self.x < 2.75
                and distance > 0.50
                and stopped
                and (stop_evidence or self.emergency_seen)
            )
            self._finish(
                passed,
                "blocked corridor stopped safely"
                if passed
                else "blocked corridor did not produce a safe stop",
            )
            return

        if (
            EXPECTED_OUTCOME == "sensor_stop"
            and self.sensor_dropout_active
            and (now_ns - self.sensor_dropout_ns) / 1e9 >= 4.0
        ):
            stopped = math.hypot(self.vx_cmd, self.vy_cmd) <= 1e-3
            passed = (
                self.safety_status_counts.get("sensor_lost", 0) >= 2
                and self.min_body_clearance >= -0.02
                and distance > 0.50
                and stopped
            )
            self._finish(
                passed,
                "sensor dropout stopped safely"
                if passed
                else "sensor dropout did not stop navigation",
            )
            return

        if elapsed >= TEST_TIMEOUT_SEC:
            self._finish(False, "timeout before reaching goal")

    def _publish_cloud(self) -> None:
        if self.finished or self.sensor_dropout_active:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        cloud = point_cloud2.create_cloud_xyz32(header, self._visible_wall_points())
        self.cloud_pub.publish(cloud)

    def _visible_wall_points(self) -> list[tuple[float, float, float]]:
        """Approximate a 360-degree LiDAR by retaining the nearest return per ray.

        Publishing points through a nearer wall makes GridMap ray clearing carve
        artificial holes into that wall, which would make a corner test invalid.
        Keep independent vertical rings while removing occluded far returns.
        """
        angular_resolution = math.radians(0.5)
        nearest: dict[tuple[int, int], tuple[float, tuple[float, float, float]]] = {}
        for point in self.wall_points:
            dx = point[0] - self.x
            dy = point[1] - self.y
            distance = math.hypot(dx, dy)
            if distance < 0.10 or distance > 5.0:
                continue
            angle_bin = int(round(math.atan2(dy, dx) / angular_resolution))
            height_bin = int(round(point[2] / 0.05))
            key = (angle_bin, height_bin)
            current = nearest.get(key)
            if current is None or distance < current[0]:
                nearest[key] = (distance, point)
        return [entry[1] for entry in nearest.values()]

    def _maybe_publish_path(self) -> None:
        if self.finished or self.path_published:
            return
        elapsed = (self.get_clock().now().nanoseconds - self.start_ns) / 1e9
        if elapsed < 2.5:
            return
        if self.path_pub.get_subscription_count() < 1 or self.occupancy_count < 3:
            return

        route: list[tuple[float, float]] = []
        for start, end in zip(REFERENCE_ROUTE[:-1], REFERENCE_ROUTE[1:]):
            route.extend(interpolate_segment(start, end))
        route.append(REFERENCE_ROUTE[-1])
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for x, y in route:
            item = PoseStamped()
            item.header = msg.header
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = 0.0
            item.pose.orientation.w = 1.0
            msg.poses.append(item)
        self.path_pub.publish(msg)
        self.path_published = True
        self.path_publish_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f"published {SCENARIO} reference path with {len(msg.poses)} points"
        )

    def _finish(self, passed: bool, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        self.passed = passed
        result = {
            "scenario": SCENARIO,
            "expected_outcome": EXPECTED_OUTCOME,
            "passed": passed,
            "reason": reason,
            "final_pose": {
                "x": round(self.x, 4),
                "y": round(self.y, 4),
                "yaw": round(self.yaw, 4),
            },
            "goal_distance_m": round(math.hypot(self.x - GOAL[0], self.y - GOAL[1]), 4),
            "min_body_clearance_m": round(self.min_body_clearance, 4),
            "min_planned_clearance_m": (
                round(self.min_planned_clearance, 4)
                if math.isfinite(self.min_planned_clearance)
                else None
            ),
            "min_planned_clearance_after_obstacle_m": (
                round(self.min_planned_clearance_after_obstacle, 4)
                if math.isfinite(self.min_planned_clearance_after_obstacle)
                else None
            ),
            "max_abs_lateral_deviation_m": round(self.max_abs_lateral_deviation, 4),
            "dynamic_obstacle_activated": self.obstacle_activated,
            "sensor_dropout_active": self.sensor_dropout_active,
            "execution_path_messages": self.execution_path_count,
            "cmd_vel_messages": self.command_count,
            "max_planar_command_mps": round(self.max_planar_command, 4),
            "occupancy_messages": self.occupancy_count,
            "emergency_stop_seen": self.emergency_seen,
            "safety_status_counts": self.safety_status_counts,
            "latest_safety_status": self.latest_safety_status,
            "min_monitor_path_distance_m": (
                round(self.min_monitor_path_distance, 4)
                if math.isfinite(self.min_monitor_path_distance)
                else None
            ),
            "body_z_error_m": round(self.max_body_z_error, 6),
        }
        print("SCAN_OFFLINE_RESULT=" + json.dumps(result, sort_keys=True), flush=True)
        if not passed:
            self.get_logger().error(reason)


def main() -> int:
    rclpy.init()
    node = OfflineCornerScenario()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.10)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.finished and node.passed else 1


if __name__ == "__main__":
    sys.exit(main())
