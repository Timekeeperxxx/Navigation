#!/usr/bin/env python3

"""
PCD Goal Editor - 可视化加载PCD文件，允许用户在RViz中点击目标点（包括朝向），并保存为JSON文件。

用法:
    ros2 run global_planner pcd_goal_editor.py --ros-args \
        -p pcd_file:=/path/to/file.pcd \
        -p output_file:=/path/to/output.json \
        -p frame_id:=map

在RViz中:
    - 使用 "Publish Point" 工具点击点云上的位置，记录为航点（无朝向）
    - 使用 "2D Goal Pose" 工具设置带朝向的目标点
    - 在终端中输入命令管理航点:
        list    - 列出所有航点
        delete <id> - 删除指定ID的航点
        save    - 保存到JSON文件
        clear   - 清除所有航点
        quit    - 退出
"""

import json
import os
import struct
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField


class PCDGoalEditor(Node):
    def __init__(self):
        super().__init__('pcd_goal_editor')

        # ========== 参数 ==========
        self.declare_parameter('pcd_file', '')
        self.declare_parameter('output_file', 'waypoints.json')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('pcd_topic', '/pcd_goal_editor/pointcloud')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('marker_topic', '/pcd_goal_editor/markers')
        self.declare_parameter('point_size', 0.3)
        self.declare_parameter('arrow_length', 1.0)
        self.declare_parameter('arrow_diameter', 0.1)

        pcd_file = self.get_parameter('pcd_file').value
        self.output_file = self.get_parameter('output_file').value
        self.frame_id = self.get_parameter('frame_id').value
        pcd_topic = self.get_parameter('pcd_topic').value
        clicked_point_topic = self.get_parameter('clicked_point_topic').value
        goal_pose_topic = self.get_parameter('goal_pose_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.point_size = self.get_parameter('point_size').value
        self.arrow_length = self.get_parameter('arrow_length').value
        self.arrow_diameter = self.get_parameter('arrow_diameter').value

        # ========== 航点存储 ==========
        # 每个航点: {"id": int, "x": float, "y": float, "z": float, "yaw": float, "source": str}
        self.waypoints = []
        self.next_id = 0

        # ========== 发布器 ==========
        # 使用与 RViz 兼容的 QoS 设置
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        pcd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.SYSTEM_DEFAULT,
        )
        self.pcd_pub = self.create_publisher(sensor_msgs.PointCloud2, pcd_topic, pcd_qos)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        # ========== 订阅器 ==========
        self.clicked_sub = self.create_subscription(
            PointStamped,
            clicked_point_topic,
            self.clicked_point_callback,
            10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            goal_pose_topic,
            self.goal_pose_callback,
            10
        )
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('initial_pose_topic').value,
            self.initial_pose_callback,
            10
        )

        # ========== 加载并发布PCD ==========
        if not pcd_file:
            self.get_logger().error('未指定PCD文件路径！请设置参数 pcd_file')
            self.get_logger().error('用法: --ros-args -p pcd_file:=/path/to/file.pcd')
        else:
            self.load_and_publish_pcd(pcd_file)

        # ========== 定时发布标记 ==========
        self.create_timer(1.0, self.publish_markers)

        # ========== 终端输入线程 ==========
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info('=' * 60)
        self.get_logger().info('PCD Goal Editor 已启动')
        self.get_logger().info(f'  输出文件: {self.output_file}')
        self.get_logger().info(f'  参考坐标系: {self.frame_id}')
        self.get_logger().info('')
        self.get_logger().info('在RViz中:')
        self.get_logger().info('  - 使用 "Publish Point" 点击位置添加航点（无朝向）')
        self.get_logger().info('  - 使用 "2D Goal Pose" 设置带朝向的目标点')
        self.get_logger().info('')
        self.get_logger().info('终端命令:')
        self.get_logger().info('  list    - 列出所有航点')
        self.get_logger().info('  delete <id> - 删除指定ID的航点')
        self.get_logger().info('  save    - 保存到JSON文件')
        self.get_logger().info('  clear   - 清除所有航点')
        self.get_logger().info('  quit    - 退出')
        self.get_logger().info('=' * 60)

    def load_and_publish_pcd(self, pcd_file):
        """加载PCD文件并发布为PointCloud2消息"""
        if not os.path.exists(pcd_file):
            self.get_logger().error(f'PCD文件不存在: {pcd_file}')
            return

        self.get_logger().info(f'正在加载PCD文件: {pcd_file}')

        try:
            # 手动解析PCD文件（支持ASCII和二进制格式）
            points = self.parse_pcd_file(pcd_file)
            self.get_logger().info(f'解析PCD完成: {points.shape[0]} 个点')
        except Exception as e:
            self.get_logger().error(f'解析PCD文件失败: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return

        # 创建 PointCloud2 消息
        cloud_msg = self.points_to_pointcloud2(points, self.frame_id)

        # 发布多次以确保RViz接收到
        for i in range(5):
            self.pcd_pub.publish(cloud_msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f'已发布点云到 {self.pcd_pub.topic_name}')

    def parse_pcd_file(self, filepath):
        """手动解析PCD文件，支持ASCII和二进制格式"""
        # 首先读取头部信息
        header = {}
        header_lines = []
        
        with open(filepath, 'rb') as f:
            while True:
                line = f.readline()
                if not line:
                    break
                line_str = line.decode('utf-8', errors='ignore').strip()
                header_lines.append(line_str)
                if line_str.startswith('DATA'):
                    header['DATA'] = line_str.split()[1].lower()
                    # 记录数据开始的位置
                    header['data_offset'] = f.tell()
                    break
                elif ' ' in line_str:
                    parts = line_str.split()
                    header[parts[0]] = parts[1:]

        # 解析头部字段
        fields = header.get('FIELDS', ['x', 'y', 'z'])
        types = header.get('TYPE', ['F'] * len(fields))
        sizes = [int(s) for s in header.get('SIZE', [4] * len(fields))]
        counts = [int(c) for c in header.get('COUNT', [1] * len(fields))]
        width = int(header.get('WIDTH', [0])[0])
        height = int(header.get('HEIGHT', [0])[0])
        num_points = int(header.get('POINTS', [0])[0])
        data_type = header.get('DATA', 'ascii')
        data_offset = header.get('data_offset', 0)

        self.get_logger().info(f'PCD头部: FIELDS={fields}, POINTS={num_points}, DATA={data_type}')
        self.get_logger().info(f'  TYPES={types}, SIZES={sizes}, COUNTS={counts}')

        if num_points == 0:
            self.get_logger().error('PCD文件中没有点数据')
            return np.array([], dtype=np.float32).reshape(0, 3)

        # 确定每个点的步长（字节数）
        point_step = sum(s * c for s, c in zip(sizes, counts))
        self.get_logger().info(f'  每个点步长: {point_step} 字节')

        points_list = []

        if data_type == 'ascii':
            # ASCII格式
            with open(filepath, 'r') as f:
                # 跳过头部
                for line in header_lines:
                    if line.startswith('DATA'):
                        break
                # 读取数据行
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split()
                    if len(parts) >= 3:
                        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                        if len(parts) >= 4:
                            intensity = float(parts[3])
                            points_list.append([x, y, z, intensity])
                        else:
                            points_list.append([x, y, z])
        else:
            # 二进制格式
            with open(filepath, 'rb') as f:
                f.seek(data_offset)
                # 确定需要读取的字段索引
                x_idx = fields.index('x') if 'x' in fields else -1
                y_idx = fields.index('y') if 'y' in fields else -1
                z_idx = fields.index('z') if 'z' in fields else -1
                intensity_idx = fields.index('intensity') if 'intensity' in fields else -1

                # 计算每个字段的偏移量
                offsets = []
                offset = 0
                for i in range(len(fields)):
                    offsets.append(offset)
                    offset += sizes[i] * counts[i]

                self.get_logger().info(f'  字段偏移: {dict(zip(fields, offsets))}')

                for _ in range(num_points):
                    raw_data = f.read(point_step)
                    if len(raw_data) < point_step:
                        break

                    x = struct.unpack('f', raw_data[offsets[x_idx]:offsets[x_idx]+4])[0] if x_idx >= 0 else 0.0
                    y = struct.unpack('f', raw_data[offsets[y_idx]:offsets[y_idx]+4])[0] if y_idx >= 0 else 0.0
                    z = struct.unpack('f', raw_data[offsets[z_idx]:offsets[z_idx]+4])[0] if z_idx >= 0 else 0.0

                    if intensity_idx >= 0:
                        intensity = struct.unpack('f', raw_data[offsets[intensity_idx]:offsets[intensity_idx]+4])[0]
                        points_list.append([x, y, z, intensity])
                    else:
                        points_list.append([x, y, z])

        if not points_list:
            self.get_logger().error('未能解析到任何点数据')
            return np.array([], dtype=np.float32).reshape(0, 3)

        return np.array(points_list, dtype=np.float32)

    def points_to_pointcloud2(self, points, frame_id):
        """将numpy点数组转换为PointCloud2消息"""
        header = Header(frame_id=frame_id, stamp=self.get_clock().now().to_msg())

        # 确定字段
        has_intensity = points.shape[1] >= 4

        # 构建PointCloud2消息
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = points.shape[0]
        cloud_msg.is_bigendian = False
        cloud_msg.is_dense = True
        cloud_msg.point_step = 16 if has_intensity else 12  # 4 floats * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width

        # 设置字段
        fields = []
        fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))
        if has_intensity:
            fields.append(PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1))
        cloud_msg.fields = fields

        # 序列化点数据
        cloud_msg.data = points.astype(np.float32).tobytes()

        return cloud_msg

    def clicked_point_callback(self, msg):
        """处理 Publish Point 工具点击"""
        wp = {
            'id': self.next_id,
            'x': msg.point.x,
            'y': msg.point.y,
            'z': msg.point.z,
            'yaw': 0.0,
            'source': 'clicked_point'
        }
        self.waypoints.append(wp)
        self.get_logger().info(
            f'[+] 添加航点 [{wp["id"]}]: '
            f'({wp["x"]:.3f}, {wp["y"]:.3f}, {wp["z"]:.3f}), '
            f'yaw={wp["yaw"]:.3f} (来自 Publish Point)'
        )
        self.next_id += 1
        self.publish_markers()

    def goal_pose_callback(self, msg):
        """处理 2D Goal Pose 工具点击（带朝向）"""
        # 从四元数计算偏航角
        q = msg.pose.orientation
        yaw = self.quaternion_to_yaw(q.w, q.x, q.y, q.z)

        wp = {
            'id': self.next_id,
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'yaw': yaw,
            'source': 'goal_pose'
        }
        self.waypoints.append(wp)
        self.get_logger().info(
            f'[+] 添加航点 [{wp["id"]}]: '
            f'({wp["x"]:.3f}, {wp["y"]:.3f}, {wp["z"]:.3f}), '
            f'yaw={wp["yaw"]:.3f} (来自 2D Goal Pose)'
        )
        self.next_id += 1
        self.publish_markers()

    def initial_pose_callback(self, msg):
        """处理 2D Pose Estimate 工具点击（带朝向）"""
        # 从四元数计算偏航角
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.w, q.x, q.y, q.z)

        wp = {
            'id': self.next_id,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'yaw': yaw,
            'source': 'initial_pose'
        }
        self.waypoints.append(wp)
        self.get_logger().info(
            f'[+] 添加航点 [{wp["id"]}]: '
            f'({wp["x"]:.3f}, {wp["y"]:.3f}, {wp["z"]:.3f}), '
            f'yaw={wp["yaw"]:.3f} (来自 2D Pose Estimate)'
        )
        self.next_id += 1
        self.publish_markers()

    def quaternion_to_yaw(self, w, x, y, z):
        """将四元数转换为偏航角（弧度）"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def publish_markers(self):
        """发布航点标记（球体+箭头）"""
        marker_array = MarkerArray()

        for wp in self.waypoints:
            # 球体标记（位置）
            sphere = Marker()
            sphere.header = Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg())
            sphere.ns = 'waypoints'
            sphere.id = wp['id']
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = wp['x']
            sphere.pose.position.y = wp['y']
            sphere.pose.position.z = wp['z']
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = self.point_size
            sphere.scale.y = self.point_size
            sphere.scale.z = self.point_size
            sphere.color.r = 0.0
            sphere.color.g = 1.0
            sphere.color.b = 0.0
            sphere.color.a = 0.8
            marker_array.markers.append(sphere)

            # 文本标记（显示ID）
            text = Marker()
            text.header = Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg())
            text.ns = 'waypoint_labels'
            text.id = wp['id']
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = wp['x']
            text.pose.position.y = wp['y']
            text.pose.position.z = wp['z'] + self.point_size + 0.3
            text.pose.orientation.w = 1.0
            text.scale.x = 0.4
            text.scale.y = 0.4
            text.scale.z = 0.4
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f'[{wp["id"]}]'
            marker_array.markers.append(text)

            # 箭头标记（朝向）
            arrow = Marker()
            arrow.header = Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg())
            arrow.ns = 'waypoint_arrows'
            arrow.id = wp['id']
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD

            # 设置箭头的朝向（使用偏航角）
            import math
            half_yaw = wp['yaw'] / 2.0
            arrow.pose.position.x = wp['x']
            arrow.pose.position.y = wp['y']
            arrow.pose.position.z = wp['z']
            arrow.pose.orientation.x = 0.0
            arrow.pose.orientation.y = 0.0
            arrow.pose.orientation.z = math.sin(half_yaw)
            arrow.pose.orientation.w = math.cos(half_yaw)

            arrow.scale.x = self.arrow_length
            arrow.scale.y = self.arrow_diameter
            arrow.scale.z = self.arrow_diameter
            arrow.color.r = 1.0
            arrow.color.g = 0.5
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            marker_array.markers.append(arrow)

        self.marker_pub.publish(marker_array)

    def save_waypoints(self, filepath=None):
        """保存航点到JSON文件"""
        if filepath is None:
            filepath = self.output_file

        if not self.waypoints:
            self.get_logger().warn('没有航点可保存！')
            return False

        # 构建输出数据
        output = {
            'frame_id': self.frame_id,
            'waypoints': []
        }

        for wp in self.waypoints:
            output['waypoints'].append({
                'id': wp['id'],
                'name': f'waypoint_{wp["id"]}',
                'x': wp['x'],
                'y': wp['y'],
                'z': wp['z'],
                'yaw': wp['yaw'],
                'source': wp['source']
            })

        # 确保输出目录存在
        output_dir = os.path.dirname(filepath)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)

        try:
            with open(filepath, 'w') as f:
                json.dump(output, f, indent=2)
            self.get_logger().info(f'已保存 {len(self.waypoints)} 个航点到: {filepath}')
            return True
        except Exception as e:
            self.get_logger().error(f'保存文件失败: {e}')
            return False

    def list_waypoints(self):
        """列出所有航点"""
        if not self.waypoints:
            self.get_logger().info('当前没有航点')
            return

        self.get_logger().info(f'当前航点列表 ({len(self.waypoints)} 个):')
        self.get_logger().info(f'{"ID":<5} {"X":<12} {"Y":<12} {"Z":<12} {"Yaw":<10} {"Source"}')
        self.get_logger().info('-' * 60)
        for wp in self.waypoints:
            self.get_logger().info(
                f'{wp["id"]:<5} {wp["x"]:<12.3f} {wp["y"]:<12.3f} '
                f'{wp["z"]:<12.3f} {wp["yaw"]:<10.3f} {wp["source"]}'
            )

    def delete_waypoint(self, wp_id):
        """删除指定ID的航点"""
        for i, wp in enumerate(self.waypoints):
            if wp['id'] == wp_id:
                self.waypoints.pop(i)
                self.get_logger().info(f'已删除航点 [{wp_id}]')
                self.publish_markers()
                return True
        self.get_logger().warn(f'未找到航点 [{wp_id}]')
        return False

    def clear_waypoints(self):
        """清除所有航点"""
        self.waypoints.clear()
        self.next_id = 0
        self.get_logger().info('已清除所有航点')
        self.publish_markers()

    def input_loop(self):
        """终端输入循环"""
        while rclpy.ok():
            try:
                cmd = input('> ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if not cmd:
                continue

            if cmd == 'quit' or cmd == 'q' or cmd == 'exit':
                self.get_logger().info('正在退出...')
                rclpy.shutdown()
                break
            elif cmd == 'list' or cmd == 'ls':
                self.list_waypoints()
            elif cmd == 'save' or cmd == 's':
                self.save_waypoints()
            elif cmd == 'clear':
                self.clear_waypoints()
            elif cmd.startswith('delete ') or cmd.startswith('del '):
                try:
                    parts = cmd.split()
                    wp_id = int(parts[1])
                    self.delete_waypoint(wp_id)
                except (IndexError, ValueError):
                    self.get_logger().warn('用法: delete <id>')
            else:
                self.get_logger().warn(f'未知命令: {cmd}')
                self.get_logger().info('可用命令: list, delete <id>, save, clear, quit')


def main(args=None):
    rclpy.init(args=args)
    node = PCDGoalEditor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('接收到中断信号')
    finally:
        # 自动保存航点
        if hasattr(node, 'waypoints') and node.waypoints:
            node.get_logger().info('正在自动保存航点...')
            node.save_waypoints()
        else:
            node.get_logger().info('没有航点需要保存')
        node.destroy_node()
        # 检查是否已经被 input_loop 中的 quit 命令 shutdown 了
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
