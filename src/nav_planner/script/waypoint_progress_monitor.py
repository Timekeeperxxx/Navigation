#!/usr/bin/env python3

import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class WaypointProgressMonitor(Node):
    """根据当前 TF 和目标点判断航点是否到达。"""

    def __init__(self) -> None:
        super().__init__("waypoint_progress_monitor")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("cancel_topic", "/cancel_navigation")
        self.declare_parameter("waypoint_reached_topic", "/waypoint_reached")
        self.declare_parameter("status_topic", "/nav/waypoint_progress")
        self.declare_parameter("reach_tolerance_xy", 0.25)
        self.declare_parameter("reach_tolerance_z", 1.0)
        self.declare_parameter("timeout_sec", 0.0)
        self.declare_parameter("check_period_sec", 0.2)

        self.global_frame = self.get_parameter("global_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.reach_tolerance_xy = float(self.get_parameter("reach_tolerance_xy").value)
        self.reach_tolerance_z = float(self.get_parameter("reach_tolerance_z").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.active_goal: Optional[PointStamped] = None
        self.active_goal_time = self.get_clock().now()
        self.last_status = "idle"

        self.reached_pub = self.create_publisher(
            Bool, self.get_parameter("waypoint_reached_topic").value, 10
        )
        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10
        )

        self.create_subscription(
            PointStamped,
            self.get_parameter("clicked_point_topic").value,
            self._on_clicked_point,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_pose_topic").value,
            self._on_goal_pose,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("cancel_topic").value,
            self._on_cancel,
            10,
        )

        period = max(float(self.get_parameter("check_period_sec").value), 0.05)
        self.create_timer(period, self._check_progress)
        self.get_logger().info(
            f"航点进度监测已启动：{self.global_frame} -> {self.robot_frame}, "
            f"xy容差={self.reach_tolerance_xy:.2f}m"
        )

    def _on_clicked_point(self, msg: PointStamped) -> None:
        self._set_goal(msg, "clicked_point")

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        goal = PointStamped()
        goal.header = msg.header
        goal.point.x = msg.pose.position.x
        goal.point.y = msg.pose.position.y
        goal.point.z = msg.pose.position.z
        self._set_goal(goal, "goal_pose")

    def _set_goal(self, goal: PointStamped, source: str) -> None:
        if not goal.header.frame_id:
            goal.header.frame_id = self.global_frame
        self.active_goal = goal
        self.active_goal_time = self.get_clock().now()
        self.last_status = "running"
        self.get_logger().info(
            f"开始监测目标({source})：frame={goal.header.frame_id}, "
            f"x={goal.point.x:.3f}, y={goal.point.y:.3f}, z={goal.point.z:.3f}"
        )
        self._publish_status("running")

    def _on_cancel(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.active_goal = None
        self.last_status = "cancelled"
        self._publish_status("cancelled")

    def _check_progress(self) -> None:
        if self.active_goal is None:
            self._publish_status(self.last_status)
            return

        now = self.get_clock().now()
        if self.timeout_sec > 0.0:
            elapsed = (now - self.active_goal_time).nanoseconds / 1e9
            if elapsed > self.timeout_sec:
                self.get_logger().warn(f"航点到达超时：{elapsed:.1f}s")
                self._publish_reached(False)
                self.active_goal = None
                self.last_status = "timeout"
                self._publish_status("timeout")
                return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.last_status = "waiting_tf"
            self._publish_status("waiting_tf", error=str(exc))
            return

        robot = transform.transform.translation
        goal = self.active_goal.point
        dx = robot.x - goal.x
        dy = robot.y - goal.y
        dz = robot.z - goal.z
        dist_xy = math.hypot(dx, dy)
        dist_z = abs(dz)

        if dist_xy <= self.reach_tolerance_xy and dist_z <= self.reach_tolerance_z:
            self.get_logger().info(
                f"航点已到达：dist_xy={dist_xy:.3f}, dist_z={dist_z:.3f}"
            )
            self._publish_reached(True)
            self.active_goal = None
            self.last_status = "reached"
            self._publish_status("reached", dist_xy=dist_xy, dist_z=dist_z)
            return

        self.last_status = "running"
        self._publish_status("running", dist_xy=dist_xy, dist_z=dist_z)

    def _publish_reached(self, reached: bool) -> None:
        msg = Bool()
        msg.data = reached
        self.reached_pub.publish(msg)

    def _publish_status(self, state: str, **extra) -> None:
        payload = {
            "state": state,
            "global_frame": self.global_frame,
            "robot_frame": self.robot_frame,
            "has_goal": self.active_goal is not None,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
        }
        if self.active_goal is not None:
            payload["goal"] = {
                "frame_id": self.active_goal.header.frame_id,
                "x": self.active_goal.point.x,
                "y": self.active_goal.point.y,
                "z": self.active_goal.point.z,
            }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointProgressMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
