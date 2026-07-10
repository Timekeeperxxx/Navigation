#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class ScanInitialPathAdapter(Node):
    """Downsample the global planner path before feeding SCAN reference-path mode."""

    def __init__(self) -> None:
        super().__init__("scan_initial_path_adapter")
        self.declare_parameter("input_path_topic", "/global_path")
        self.declare_parameter("output_path_topic", "/scan/initial_path")
        self.declare_parameter("min_point_spacing", 0.5)
        self.declare_parameter("max_points", 120)

        self.min_point_spacing = max(float(self.get_parameter("min_point_spacing").value), 0.0)
        self.max_points = max(int(self.get_parameter("max_points").value), 0)

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            Path, str(self.get_parameter("output_path_topic").value), output_qos
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("input_path_topic").value),
            self._on_path,
            10,
        )
        self.get_logger().info(
            "SCAN initial path adapter: "
            f"{self.get_parameter('input_path_topic').value} -> "
            f"{self.get_parameter('output_path_topic').value}, "
            f"min_spacing={self.min_point_spacing:.2f}, max_points={self.max_points}"
        )

    def _on_path(self, msg: Path) -> None:
        if not msg.poses:
            self.publisher.publish(msg)
            return

        filtered = self._filter_by_spacing(msg)
        if self.max_points > 0 and len(filtered.poses) > self.max_points:
            filtered = self._limit_points(filtered, self.max_points)

        self.publisher.publish(filtered)
        if len(filtered.poses) != len(msg.poses):
            self.get_logger().info(
                f"SCAN initial path downsampled: {len(msg.poses)} -> {len(filtered.poses)}"
            )

    def _filter_by_spacing(self, msg: Path) -> Path:
        if self.min_point_spacing <= 0.0 or len(msg.poses) <= 2:
            return msg

        result = Path()
        result.header = msg.header
        result.poses.append(msg.poses[0])
        last = msg.poses[0].pose.position
        min_dist2 = self.min_point_spacing * self.min_point_spacing

        for pose in msg.poses[1:-1]:
            point = pose.pose.position
            dx = float(point.x - last.x)
            dy = float(point.y - last.y)
            dz = float(point.z - last.z)
            if dx * dx + dy * dy + dz * dz >= min_dist2:
                result.poses.append(pose)
                last = point

        result.poses.append(msg.poses[-1])
        return result

    def _limit_points(self, msg: Path, max_points: int) -> Path:
        if len(msg.poses) <= max_points or max_points < 2:
            return msg

        result = Path()
        result.header = msg.header
        last_index = len(msg.poses) - 1
        selected_indices = {
            int(round(i * last_index / float(max_points - 1))) for i in range(max_points)
        }
        for index in sorted(selected_indices):
            result.poses.append(msg.poses[index])
        if result.poses[-1] is not msg.poses[-1]:
            result.poses.append(msg.poses[-1])
        return result


def main() -> None:
    rclpy.init()
    node = ScanInitialPathAdapter()
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
