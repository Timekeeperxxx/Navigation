#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_odometry(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def footprint_center(
    base_x: float, base_y: float, yaw: float, center_offset: float
) -> tuple[float, float]:
    """Transform one body-x collision-circle offset into the map frame."""
    return (
        base_x + center_offset * math.cos(yaw),
        base_y + center_offset * math.sin(yaw),
    )


class SimNavigationMarkers(Node):
    """Publish prominent robot, coordinate, goal and breadcrumb RViz markers."""

    def __init__(self) -> None:
        super().__init__("sim_navigation_markers")
        self.declare_parameter("pose_topic", "/sim/body_pose")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("marker_topic", "/nav/sim_visual_markers")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("trail_spacing", 0.08)
        self.declare_parameter("trail_max_points", 800)
        self.declare_parameter("footprint_radius", 0.27)
        self.declare_parameter("footprint_center_offsets", [-0.22, -0.63])
        self.declare_parameter("footprint_height", 0.32)

        self.global_frame = str(self.get_parameter("global_frame").value)
        self.trail_spacing = max(
            float(self.get_parameter("trail_spacing").value), 0.02
        )
        self.trail_max_points = max(
            int(self.get_parameter("trail_max_points").value), 2
        )
        self.footprint_radius = max(
            float(self.get_parameter("footprint_radius").value), 0.01
        )
        self.footprint_center_offsets = [
            float(value)
            for value in self.get_parameter("footprint_center_offsets").value
        ]
        if len(self.footprint_center_offsets) != 2:
            raise ValueError("footprint_center_offsets must contain exactly two values")
        self.footprint_height = max(
            float(self.get_parameter("footprint_height").value), 0.01
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            marker_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("pose_topic").value),
            self._on_pose,
            20,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self._on_goal,
            10,
        )
        self.pose: Optional[Odometry] = None
        self.goal: Optional[PoseStamped] = None
        self.trail: list[Point] = []

    def _on_pose(self, msg: Odometry) -> None:
        self.pose = msg
        position = msg.pose.pose.position
        if (
            not self.trail
            or math.hypot(
                position.x - self.trail[-1].x,
                position.y - self.trail[-1].y,
            )
            >= self.trail_spacing
        ):
            point = Point()
            point.x = float(position.x)
            point.y = float(position.y)
            point.z = float(position.z) + 0.05
            self.trail.append(point)
            if len(self.trail) > self.trail_max_points:
                self.trail = self.trail[-self.trail_max_points :]
        self._publish()

    def _on_goal(self, msg: PoseStamped) -> None:
        self.goal = msg
        self._publish()

    def _base_marker(self, marker_id: int, namespace: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _publish(self) -> None:
        if self.pose is None:
            return

        markers = MarkerArray()
        pose = self.pose.pose.pose
        yaw = yaw_from_odometry(self.pose)
        speed = math.hypot(
            self.pose.twist.twist.linear.x,
            self.pose.twist.twist.linear.y,
        )

        for index, center_offset in enumerate(self.footprint_center_offsets):
            center_x, center_y = footprint_center(
                float(pose.position.x),
                float(pose.position.y),
                yaw,
                center_offset,
            )
            cylinder = self._base_marker(
                10 + index, "double_circle_footprint", Marker.CYLINDER
            )
            cylinder.pose.position.x = center_x
            cylinder.pose.position.y = center_y
            cylinder.pose.position.z = (
                float(pose.position.z) + self.footprint_height / 2.0
            )
            cylinder.scale.x = 2.0 * self.footprint_radius
            cylinder.scale.y = 2.0 * self.footprint_radius
            cylinder.scale.z = self.footprint_height
            cylinder.color.r = 1.0
            cylinder.color.g = 0.42 if index == 0 else 0.22
            cylinder.color.b = 0.02
            cylinder.color.a = 0.52
            markers.markers.append(cylinder)

            outline = self._base_marker(
                20 + index, "double_circle_outline", Marker.LINE_STRIP
            )
            outline.pose.position.z = (
                float(pose.position.z) + self.footprint_height + 0.015
            )
            outline.points = [
                Point(
                    x=center_x
                    + self.footprint_radius
                    * math.cos(2.0 * math.pi * step / 64.0),
                    y=center_y
                    + self.footprint_radius
                    * math.sin(2.0 * math.pi * step / 64.0),
                    z=0.0,
                )
                for step in range(65)
            ]
            outline.scale.x = 0.025
            outline.color.r = 1.0
            outline.color.g = 0.92
            outline.color.b = 0.05
            outline.color.a = 1.0
            markers.markers.append(outline)

            center = self._base_marker(
                30 + index, "double_circle_centers", Marker.SPHERE
            )
            center.pose.position.x = center_x
            center.pose.position.y = center_y
            center.pose.position.z = (
                float(pose.position.z) + self.footprint_height + 0.035
            )
            center.scale.x = 0.07
            center.scale.y = 0.07
            center.scale.z = 0.05
            center.color.r = 1.0
            center.color.g = 1.0
            center.color.b = 1.0
            center.color.a = 1.0
            markers.markers.append(center)

        base_origin = self._base_marker(40, "base_footprint_origin", Marker.SPHERE)
        base_origin.pose.position.x = float(pose.position.x)
        base_origin.pose.position.y = float(pose.position.y)
        base_origin.pose.position.z = float(pose.position.z) + 0.08
        base_origin.scale.x = 0.09
        base_origin.scale.y = 0.09
        base_origin.scale.z = 0.09
        base_origin.color.r = 0.95
        base_origin.color.g = 0.95
        base_origin.color.b = 1.0
        base_origin.color.a = 1.0
        markers.markers.append(base_origin)

        heading = self._base_marker(41, "sim_robot_heading", Marker.ARROW)
        start = Point(
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=float(pose.position.z) + 0.48,
        )
        end = Point(
            x=start.x + 1.15 * math.cos(yaw),
            y=start.y + 1.15 * math.sin(yaw),
            z=start.z,
        )
        heading.points = [start, end]
        heading.scale.x = 0.09
        heading.scale.y = 0.20
        heading.scale.z = 0.28
        heading.color.r = 1.0
        heading.color.g = 0.90
        heading.color.b = 0.05
        heading.color.a = 1.0
        markers.markers.append(heading)

        coordinates = self._base_marker(
            42, "sim_robot_coordinates", Marker.TEXT_VIEW_FACING
        )
        coordinates.pose.position.x = float(pose.position.x)
        coordinates.pose.position.y = float(pose.position.y)
        coordinates.pose.position.z = float(pose.position.z) + 1.05
        coordinates.scale.z = 0.34
        coordinates.color.r = 1.0
        coordinates.color.g = 1.0
        coordinates.color.b = 1.0
        coordinates.color.a = 1.0
        coordinates.text = (
            f"DOUBLE CIRCLE  R={self.footprint_radius:.2f} m\n"
            f"x={pose.position.x:.2f}  y={pose.position.y:.2f}\n"
            f"yaw={math.degrees(yaw):.1f} deg  v={speed:.2f} m/s"
        )
        markers.markers.append(coordinates)

        trail = self._base_marker(43, "sim_robot_trail", Marker.LINE_STRIP)
        trail.points = self.trail
        trail.scale.x = 0.055
        trail.color.r = 1.0
        trail.color.g = 0.42
        trail.color.b = 0.02
        trail.color.a = 0.95
        markers.markers.append(trail)

        if self.goal is not None:
            goal_pose = self.goal.pose
            goal = self._base_marker(50, "sim_goal", Marker.CYLINDER)
            goal.pose.position.x = float(goal_pose.position.x)
            goal.pose.position.y = float(goal_pose.position.y)
            goal.pose.position.z = float(goal_pose.position.z) + 0.20
            goal.scale.x = 0.42
            goal.scale.y = 0.42
            goal.scale.z = 0.40
            goal.color.r = 1.0
            goal.color.g = 0.05
            goal.color.b = 0.05
            goal.color.a = 0.92
            markers.markers.append(goal)

            goal_text = self._base_marker(
                51, "sim_goal_coordinates", Marker.TEXT_VIEW_FACING
            )
            goal_text.pose.position.x = float(goal_pose.position.x)
            goal_text.pose.position.y = float(goal_pose.position.y)
            goal_text.pose.position.z = float(goal_pose.position.z) + 0.80
            goal_text.scale.z = 0.30
            goal_text.color.r = 1.0
            goal_text.color.g = 0.25
            goal_text.color.b = 0.20
            goal_text.color.a = 1.0
            goal_text.text = (
                "GOAL\n"
                f"x={goal_pose.position.x:.2f}  y={goal_pose.position.y:.2f}"
            )
            markers.markers.append(goal_text)

        self.publisher.publish(markers)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimNavigationMarkers()
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
