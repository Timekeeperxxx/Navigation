#!/usr/bin/env python3

from typing import Optional

import rclpy
from dddmr_sys_core.action import GetPlan
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node


class ClickedDWAPlannerClient(Node):
    """把 RViz 点击目标转发给 DWA-aware 全局规划 action。"""

    def __init__(self) -> None:
        super().__init__("clicked_dwa_planner_client")

        self.declare_parameter("action_name", "/get_dwa_plan")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("activate_threading", True)
        self.declare_parameter("server_wait_sec", 1.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.activate_threading = bool(self.get_parameter("activate_threading").value)
        self.server_wait_sec = float(self.get_parameter("server_wait_sec").value)
        self.pending_goal: Optional[PoseStamped] = None
        self.goal_in_flight = False

        self.action_client = ActionClient(
            self, GetPlan, self.get_parameter("action_name").value
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

        self.get_logger().info(
            f"DWA-aware 点击规划客户端已启动：action={self.get_parameter('action_name').value}"
        )

    def _on_clicked_point(self, msg: PointStamped) -> None:
        goal = PoseStamped()
        goal.header = msg.header
        if not goal.header.frame_id:
            goal.header.frame_id = self.frame_id
        goal.pose.position.x = msg.point.x
        goal.pose.position.y = msg.point.y
        goal.pose.position.z = msg.point.z
        goal.pose.orientation.w = 1.0
        self._send_goal(goal)

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        goal = PoseStamped()
        goal.header = msg.header
        if not goal.header.frame_id:
            goal.header.frame_id = self.frame_id
        goal.pose = msg.pose
        self._send_goal(goal)

    def _send_goal(self, pose: PoseStamped) -> None:
        if self.goal_in_flight:
            self.pending_goal = pose
            self.get_logger().info("已有 DWA-aware goal 执行中，缓存最新目标")
            return

        if not self.action_client.wait_for_server(timeout_sec=self.server_wait_sec):
            self.get_logger().warn("等待 /get_dwa_plan action server 超时")
            return

        goal = GetPlan.Goal()
        goal.goal = pose
        goal.activate_threading = self.activate_threading
        self.goal_in_flight = True
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info(
            f"发送 DWA-aware goal：x={pose.pose.position.x:.3f}, "
            f"y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}"
        )

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_in_flight = False
            self.get_logger().warn("DWA-aware goal 被拒绝")
            self._send_pending_goal()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        self.goal_in_flight = False
        result = future.result().result
        self.get_logger().info(f"DWA-aware 规划返回：path poses={len(result.path.poses)}")
        self._send_pending_goal()

    def _send_pending_goal(self) -> None:
        if self.pending_goal is None:
            return
        goal = self.pending_goal
        self.pending_goal = None
        self._send_goal(goal)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClickedDWAPlannerClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
