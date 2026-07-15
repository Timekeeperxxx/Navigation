#!/usr/bin/env python3

import json
import rclpy
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import PointStamped

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('nav_start_topic', '/nav_task_start')
        self.declare_parameter('waypoint_reached_topic', '/waypoint_reached')
        self.declare_parameter('nav_status_topic', '/nav_status')

        self.waypoints_file = self.resolve_waypoints_file(self.get_parameter('waypoints_file').value)
        self.frame_id = self.get_parameter('frame_id').value
        clicked_point_topic = self.get_parameter('clicked_point_topic').value
        nav_start_topic = self.get_parameter('nav_start_topic').value
        waypoint_reached_topic = self.get_parameter('waypoint_reached_topic').value

        self.waypoints = []
        self.task_id = None
        self.task_name = None
        self.current_index = 0
        self.navigating = False
        self.retry_timer = None

        # The backend rewrites current_task.json for every task execution.  An
        # empty/missing file during localization startup is valid; task start
        # reloads it before publishing the first goal.
        self.reload_waypoints()

        self.clicked_pub = self.create_publisher(PointStamped, clicked_point_topic, 10)

        self.yaw_pub = self.create_publisher(Float64, 'goal_yaw', 10)

        self.cancel_pub = self.create_publisher(Bool, '/cancel_navigation', 10)

        self.nav_status_pub = self.create_publisher(
            String,
            self.get_parameter('nav_status_topic').value,
            10,
        )

        self.nav_sub = self.create_subscription(
            Bool,
            nav_start_topic,
            self.nav_start_callback,
            10
        )

        self.reached_sub = self.create_subscription(
            Bool,
            waypoint_reached_topic,
            self.waypoint_reached_callback,
            10
        )

        self.get_logger().info('WaypointNavigator node started')

    def resolve_waypoints_file(self, configured_path):
        if configured_path:
            return configured_path
        try:
            return str(Path(get_package_share_directory('nav_planner')) / 'data' / 'waypoints.json')
        except Exception:
            return ''

    def load_waypoints(self, filepath):
        """Load waypoints from JSON file and return list of dicts with x, y, z, name."""
        self.task_id = None
        self.task_name = None
        if not filepath:
            self.get_logger().error('未配置 waypoints_file')
            return []
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f'加载航点文件失败: {e}')
            return []

        waypoints = []

        # 支持 waypoints、steps、items 和 workflows.json 这几类任务格式。
        if isinstance(data, dict):
            self.task_id = data.get('task_id') or data.get('id')
            self.task_name = data.get('task_name') or data.get('name')
            # 新格式: 顶层 frame_id + waypoints 数组 (waypoints.json)
            if 'waypoints' in data:
                top_frame_id = data.get('frame_id', self.frame_id)
                for wp in data['waypoints']:
                    waypoints.append({
                        'name': wp.get('name', 'unknown'),
                        'x': wp['x'],
                        'y': wp['y'],
                        'z': wp['z'],
                        'yaw': wp.get('yaw', 0.0),
                        'frame_id': wp.get('frame_id', top_frame_id),
                    })
            # 新格式: 顶层 frame_id + steps 数组
            elif 'steps' in data:
                top_frame_id = data.get('frame_id', self.frame_id)
                for step in data['steps']:
                    if step.get('type') == 'navigate_waypoint':
                        waypoints.append({
                            'name': step.get('waypoint_name', step.get('waypointName', 'unknown')),
                            'waypoint_id': step.get('waypoint_id', step.get('waypointId')),
                            'x': step['x'],
                            'y': step['y'],
                            'z': step['z'],
                            'yaw': step.get('yaw', 0.0),
                            'frame_id': step.get('frame_id', top_frame_id),
                        })
            # 旧格式: items 数组
            elif 'items' in data:
                for item in data['items']:
                    waypoints.append({
                        'name': item.get('name', 'unknown'),
                        'x': item['x'],
                        'y': item['y'],
                        'z': item['z'],
                        'yaw': item.get('yaw', 0.0),
                        'frame_id': item.get('frame_id', self.frame_id),
                    })
            else:
                self.get_logger().error(f'不支持的 dict 格式, 缺少 waypoints、steps 或 items 键')
                return []
        elif isinstance(data, list):
            # workflows.json 格式: 数组中的每个元素包含 steps
            for workflow in data:
                steps = workflow.get('steps', [])
                for step in steps:
                    if step.get('type') == 'navigate_waypoint':
                        waypoints.append({
                            'name': step.get('waypointName', step.get('label', 'unknown')),
                            'x': step['x'],
                            'y': step['y'],
                            'z': step['z'],
                            'yaw': step.get('yaw', 0.0),
                            'frame_id': step.get('frameId', self.frame_id),
                        })
        else:
            self.get_logger().error(f'不支持的 JSON 格式: {type(data)}')
            return []

        return waypoints

    def reload_waypoints(self):
        waypoints = self.load_waypoints(self.waypoints_file)
        self.waypoints = waypoints
        if not waypoints:
            self.get_logger().warning(f'No waypoints loaded from {self.waypoints_file}')
            return False

        self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {self.waypoints_file}')
        for i, wp in enumerate(waypoints):
            self.get_logger().info(
                f'  [{i}] {wp["name"]}: ({wp["x"]:.3f}, {wp["y"]:.3f}, '
                f'{wp["z"]:.3f}), yaw={wp["yaw"]:.3f}'
            )
        return True

    def nav_start_callback(self, msg):
        """处理巡检开始/停止信号。"""
        if msg.data:
            if not self.reload_waypoints():
                self.navigating = False
                self.get_logger().error('接收到任务开始信号，但当前任务没有有效航点，拒绝启动')
                self.publish_task_status(
                    'failed',
                    '任务没有有效航点，无法启动',
                    task_complete=True,
                )
                return
            if self.retry_timer is not None:
                self.retry_timer.cancel()
                self.retry_timer = None
            self.get_logger().info('接收到任务开始信号(true)，从头开始导航')
            self.navigating = True
            self.current_index = 0
            self.publish_task_status('moving', '任务已开始', task_complete=False)
            self.publish_current_waypoint()
        else:
            if self.navigating:
                self.get_logger().info('接收到停止导航信号 (false), stopping navigation')
                self.navigating = False
                cancel_msg = Bool()
                cancel_msg.data = True
                self.cancel_pub.publish(cancel_msg)
                self.get_logger().info('已发送取消导航信号到 /cancel_navigation')
                self.publish_task_status(
                    'canceled',
                    '任务导航已停止',
                    task_complete=True,
                )
            else:
                self.get_logger().info('Navigation stop signal received (false), but not navigating')

    def publish_current_waypoint(self):
        """发布当前航点到 clicked_point 和 goal_yaw。
        Navigation 内的 planner 会消费 clicked_point；waypoint_progress_monitor 负责发布 waypoint_reached。
        """
        if not self.navigating:
            return

        if self.current_index >= len(self.waypoints):
            self.get_logger().info('所有航点已访问完毕，导航完成！')
            self.navigating = False
            self.publish_task_status(
                'reached',
                '所有任务航点已完成',
                task_complete=True,
            )
            return

        wp = self.waypoints[self.current_index]

        point_msg = PointStamped()
        point_msg.header.frame_id = wp['frame_id']
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = wp['x']
        point_msg.point.y = wp['y']
        point_msg.point.z = wp['z']
        yaw_msg = Float64()
        yaw_msg.data = wp['yaw']
        self.yaw_pub.publish(yaw_msg)
        self.clicked_pub.publish(point_msg)

        self.get_logger().info(
            f'发布航点 [{self.current_index}/{len(self.waypoints)-1}] '
            f'"{wp["name"]}": ({wp["x"]:.3f}, {wp["y"]:.3f}, {wp["z"]:.3f}), yaw={wp["yaw"]:.3f}'
        )

    def waypoint_reached_callback(self, msg):
        """处理航点到达或失败反馈。"""
        if not self.navigating:
            return

        wp_name = self.waypoints[self.current_index]['name'] if self.current_index < len(self.waypoints) else 'unknown'

        if msg.data:
            self.get_logger().info(
                f'已到达航点 [{self.current_index}] "{wp_name}" (SUCCESS)'
            )
            completed_index = self.current_index
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self.get_logger().info('所有航点已访问完毕，导航完成！')
                self.navigating = False
                self.publish_task_status(
                    'reached',
                    '所有任务航点已完成',
                    task_complete=True,
                    waypoint_index=completed_index,
                )
                return

            self.publish_task_status(
                'moving',
                '当前航点已到达，正在前往下一个航点',
                task_complete=False,
                waypoint_index=completed_index,
            )
            self.publish_current_waypoint()
        else:
            self.get_logger().warn(
                f'导航到航点 [{self.current_index}] "{wp_name}" 失败，将在 1 秒后重试...'
            )
            # 失败时重试当前航点，给 global_planner 留出恢复时间。
            if self.retry_timer is None:
                self.retry_timer = self.create_timer(1.0, self.retry_current_waypoint)

    def retry_current_waypoint(self):
        """重试当前航点（定时器回调）"""
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None
        if not self.navigating:
            return
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('所有航点已访问完毕，导航完成！')
            self.navigating = False
            return
        self.publish_current_waypoint()

    def publish_task_status(
        self,
        status,
        message,
        *,
        task_complete,
        waypoint_index=None,
    ):
        payload = {
            'status': status,
            'message': message,
            'task_id': self.task_id,
            'target_name': self.task_name,
            'task_complete': bool(task_complete),
            'waypoint_index': waypoint_index,
            'waypoint_count': len(self.waypoints),
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        }
        if waypoint_index is not None and 0 <= waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[waypoint_index]
            payload['waypoint_id'] = waypoint.get('waypoint_id')
            payload['waypoint_name'] = waypoint.get('name')
        self.nav_status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
