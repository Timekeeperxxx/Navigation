#!/usr/bin/env python3

import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String
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
        self.declare_parameter("nav_status_topic", "/nav_status")
        self.declare_parameter("goal_yaw_topic", "goal_yaw")
        self.declare_parameter("nav_start_topic", "/nav_start")
        self.declare_parameter("nav_stop_topic", "/nav_stop")
        self.declare_parameter("reach_tolerance_xy", 0.25)
        self.declare_parameter("reach_tolerance_z", 1.0)
        self.declare_parameter("reach_tolerance_yaw", 0.15)
        self.declare_parameter("timeout_sec", 0.0)
        self.declare_parameter("check_period_sec", 0.2)

        self.global_frame = self.get_parameter("global_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.reach_tolerance_xy = float(self.get_parameter("reach_tolerance_xy").value)
        self.reach_tolerance_z = float(self.get_parameter("reach_tolerance_z").value)
        self.reach_tolerance_yaw = float(self.get_parameter("reach_tolerance_yaw").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.active_goal: Optional[PointStamped] = None
        self.active_goal_yaw: Optional[float] = None
        self.pending_goal_yaw: Optional[float] = None
        self.active_goal_time = self.get_clock().now()
        self.last_status = "idle"
        self.navigation_enabled = False

        self.reached_pub = self.create_publisher(
            Bool, self.get_parameter("waypoint_reached_topic").value, 10
        )
        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10
        )
        self.nav_status_pub = self.create_publisher(
            String, self.get_parameter("nav_status_topic").value, 10
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
        self.create_subscription(
            Float64,
            self.get_parameter("goal_yaw_topic").value,
            self._on_goal_yaw,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("nav_start_topic").value,
            self._on_nav_start,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("nav_stop_topic").value,
            self._on_nav_stop,
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

    def _on_goal_yaw(self, msg: Float64) -> None:
        yaw = self._normalize_angle(float(msg.data))
        if self.active_goal is not None:
            self.active_goal_yaw = yaw
            self.pending_goal_yaw = None
        else:
            self.pending_goal_yaw = yaw

    def _set_goal(self, goal: PointStamped, source: str) -> None:
        if not goal.header.frame_id:
            goal.header.frame_id = self.global_frame
        self.active_goal = goal
        self.active_goal_yaw = self.pending_goal_yaw
        self.pending_goal_yaw = None
        self.active_goal_time = self.get_clock().now()
        self.last_status = "running"
        self.get_logger().info(
            f"开始监测目标({source})：frame={goal.header.frame_id}, "
            f"x={goal.point.x:.3f}, y={goal.point.y:.3f}, z={goal.point.z:.3f}"
        )
        self._publish_status("running")
        self._publish_nav_status("accepted", "目标已接收")

    def _on_cancel(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.active_goal = None
        self.active_goal_yaw = None
        self.last_status = "cancelled"
        self._publish_status("cancelled")
        self._publish_nav_status("canceled", "导航已取消")

    def _on_nav_start(self, msg: Bool) -> None:
        self.navigation_enabled = bool(msg.data)
        if self.navigation_enabled and self.active_goal is not None:
            self._publish_nav_status("moving", "开始执行导航")
        elif not self.navigation_enabled and self.active_goal is not None:
            self.active_goal = None
            self.active_goal_yaw = None
            self.last_status = "cancelled"
            self._publish_nav_status("canceled", "导航启动信号已关闭")

    def _on_nav_stop(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.navigation_enabled = False
        self.active_goal = None
        self.active_goal_yaw = None
        self.last_status = "estop"
        self._publish_status("estop")
        self._publish_nav_status("estop", "收到导航急停")

    def _check_progress(self) -> None:
        if self.active_goal is None:
            self._publish_status(self.last_status)
            return
        if not self.navigation_enabled:
            self.last_status = "waiting_start"
            self._publish_status("waiting_start")
            return

        now = self.get_clock().now()
        if self.timeout_sec > 0.0:
            elapsed = (now - self.active_goal_time).nanoseconds / 1e9
            if elapsed > self.timeout_sec:
                self.get_logger().warn(f"航点到达超时：{elapsed:.1f}s")
                self._publish_reached(False)
                self.active_goal = None
                self.active_goal_yaw = None
                self.last_status = "timeout"
                self._publish_status("timeout")
                self._publish_nav_status("failed", "航点到达超时")
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
            self._publish_nav_status("moving", "等待定位 TF")
            return

        robot = transform.transform.translation
        goal = self.active_goal.point
        dx = robot.x - goal.x
        dy = robot.y - goal.y
        dz = robot.z - goal.z
        dist_xy = math.hypot(dx, dy)
        dist_z = abs(dz)
        yaw = self._yaw_from_quaternion(transform.transform.rotation)
        yaw_error = (
            abs(self._normalize_angle(self.active_goal_yaw - yaw))
            if self.active_goal_yaw is not None
            else 0.0
        )

        if (
            dist_xy <= self.reach_tolerance_xy
            and dist_z <= self.reach_tolerance_z
            and yaw_error <= self.reach_tolerance_yaw
        ):
            self.get_logger().info(
                f"航点已到达：dist_xy={dist_xy:.3f}, dist_z={dist_z:.3f}"
            )
            self._publish_reached(True)
            self.active_goal = None
            self.active_goal_yaw = None
            self.last_status = "reached"
            self._publish_status("reached", dist_xy=dist_xy, dist_z=dist_z, yaw_error=yaw_error)
            self._publish_nav_status("reached", "已到达目标", distance_to_goal=dist_xy)
            return

        self.last_status = "running"
        self._publish_status("running", dist_xy=dist_xy, dist_z=dist_z, yaw_error=yaw_error)
        self._publish_nav_status("moving", "导航中", distance_to_goal=dist_xy)

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
            if self.active_goal_yaw is not None:
                payload["goal"]["yaw"] = self.active_goal_yaw
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def _publish_nav_status(
        self, status: str, message: str, distance_to_goal: Optional[float] = None
    ) -> None:
        payload = {
            "status": status,
            "message": message,
            "distance_to_goal": distance_to_goal,
            "error_code": None,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
        }
        self.nav_status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _yaw_from_quaternion(rotation) -> float:
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointProgressMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
