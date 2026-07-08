#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_msgs.msg import Header

try:
    from sensor_msgs_py import point_cloud2
except ImportError:  # pragma: no cover - 兼容极简 ROS 安装环境
    point_cloud2 = None


@dataclass
class SimObstacle:
    x: float
    y: float
    z: float
    stamp: float


class LocalObstacleSimulator(Node):
    """在没有雷达时发布可控的 `/nav/local_obstacles` 测试点云。"""

    def __init__(self) -> None:
        super().__init__("local_obstacle_simulator")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("obstacle_topic", "/nav/local_obstacles")
        self.declare_parameter("clicked_obstacle_topic", "/nav/sim_obstacle_point")
        self.declare_parameter("clear_topic", "/nav/clear_sim_obstacles")
        self.declare_parameter("initial_obstacles_json", "[]")
        self.declare_parameter("publish_period_sec", 0.2)
        self.declare_parameter("obstacle_lifetime_sec", 0.0)
        self.declare_parameter("cluster_radius", 0.0)
        self.declare_parameter("cluster_points", 8)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.obstacle_lifetime_sec = float(
            self.get_parameter("obstacle_lifetime_sec").value
        )
        self.cluster_radius = float(self.get_parameter("cluster_radius").value)
        self.cluster_points = int(self.get_parameter("cluster_points").value)
        self.obstacles: list[SimObstacle] = []

        self.obstacle_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("obstacle_topic").value), 10
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("clicked_obstacle_topic").value),
            self._on_clicked_obstacle,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("clear_topic").value),
            self._on_clear,
            10,
        )

        self._load_initial_obstacles()
        period = max(float(self.get_parameter("publish_period_sec").value), 0.05)
        self.create_timer(period, self._publish)
        self.get_logger().info(
            "局部障碍仿真发布器已启动："
            f"topic={self.get_parameter('obstacle_topic').value}, "
            f"clicked={self.get_parameter('clicked_obstacle_topic').value}"
        )

    def _load_initial_obstacles(self) -> None:
        raw = str(self.get_parameter("initial_obstacles_json").value).strip()
        if not raw:
            return
        try:
            values = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"initial_obstacles_json 解析失败：{exc}")
            return
        now = self._now_sec()
        for item in values:
            point = self._parse_obstacle(item)
            if point is not None:
                self.obstacles.append(SimObstacle(point[0], point[1], point[2], now))
        if self.obstacles:
            self.get_logger().info(f"已加载 {len(self.obstacles)} 个初始仿真障碍")

    def _parse_obstacle(self, item) -> Optional[tuple[float, float, float]]:
        if isinstance(item, dict):
            return (
                float(item.get("x", 0.0)),
                float(item.get("y", 0.0)),
                float(item.get("z", 0.0)),
            )
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            z = float(item[2]) if len(item) >= 3 else 0.0
            return float(item[0]), float(item[1]), z
        self.get_logger().warn(f"跳过不支持的障碍配置：{item}")
        return None

    def _on_clicked_obstacle(self, msg: PointStamped) -> None:
        self.obstacles.append(
            SimObstacle(
                float(msg.point.x),
                float(msg.point.y),
                float(msg.point.z),
                self._now_sec(),
            )
        )
        self.get_logger().info(
            f"新增仿真障碍：x={msg.point.x:.3f}, y={msg.point.y:.3f}, z={msg.point.z:.3f}"
        )

    def _on_clear(self, msg: Bool) -> None:
        if not msg.data:
            return
        count = len(self.obstacles)
        self.obstacles.clear()
        self.get_logger().info(f"已清空 {count} 个仿真障碍")

    def _publish(self) -> None:
        self._prune_expired()
        points = self._expanded_points()
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        if point_cloud2 is None:
            self.get_logger().error("缺少 sensor_msgs_py，无法发布 PointCloud2")
            return
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.obstacle_pub.publish(cloud)

    def _expanded_points(self) -> list[tuple[float, float, float]]:
        points: list[tuple[float, float, float]] = []
        for obstacle in self.obstacles:
            points.append((obstacle.x, obstacle.y, obstacle.z))
            if self.cluster_radius <= 0.0 or self.cluster_points <= 0:
                continue
            for index in range(self.cluster_points):
                angle = 2.0 * math.pi * index / self.cluster_points
                points.append(
                    (
                        obstacle.x + self.cluster_radius * math.cos(angle),
                        obstacle.y + self.cluster_radius * math.sin(angle),
                        obstacle.z,
                    )
                )
        return points

    def _prune_expired(self) -> None:
        if self.obstacle_lifetime_sec <= 0.0:
            return
        now = self._now_sec()
        self.obstacles = [
            obstacle
            for obstacle in self.obstacles
            if now - obstacle.stamp <= self.obstacle_lifetime_sec
        ]

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LocalObstacleSimulator()
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
