#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, Bool
import sensor_msgs.msg as sensor_msgs
from sensor_msgs_py import point_cloud2
import tf2_ros
import numpy as np
import sys
import os

class PolygonLoader(Node):
    def __init__(self):
        super().__init__('polygon_loader')
        self.declare_parameter('clicked_points_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('polygon_topic', '/loaded_polygon')
        self.declare_parameter('marker_topic', '/loaded_polygon_marker')
        self.declare_parameter('points_marker_topic', '/loaded_points_markers')
        self.declare_parameter('points_stamped_topic', '/loaded_points')
        self.declare_parameter('radius', 1.0)  # meters
        # 连续检测阈值
        self.declare_parameter('consecutive_threshold', 3)
        self.declare_parameter('local_map_topic', '/local_map_raw')

        file_path = self.get_parameter('clicked_points_file').value
        self.frame_id = self.get_parameter('frame_id').value
        polygon_topic = self.get_parameter('polygon_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        points_marker_topic = self.get_parameter('points_marker_topic').value
        points_stamped_topic = self.get_parameter('points_stamped_topic').value
        self.radius = self.get_parameter('radius').value
        self.consecutive_threshold = self.get_parameter('consecutive_threshold').value
        local_map_topic = self.get_parameter('local_map_topic').value

        self.polygon_pub = self.create_publisher(PolygonStamped, polygon_topic, 10)
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)
        self.points_marker_pub = self.create_publisher(MarkerArray, points_marker_topic, 10)
        # Additional publisher for PointStamped (for RViz Clicked Points display)
        self.point_pub = self.create_publisher(PointStamped, points_stamped_topic, 10)

        # Publisher for boolean flag
        self.flag_pub = self.create_publisher(Bool, '/in_center_radius', 10)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.points = self.load_points(file_path)
        # Compute center point
        self.center_point = None
        if self.points:
            n = len(self.points)
            cx = sum(p[0] for p in self.points) / n
            cy = sum(p[1] for p in self.points) / n
            cz = sum(p[2] for p in self.points) / n
            self.center_point = (cx, cy, cz)
            self.get_logger().info(f'Center point: ({cx:.3f}, {cy:.3f}, {cz:.3f})')
        if self.points:
            self.publish_polygon(self.points)
            self.publish_marker(self.points)
            self.publish_points_markers(self.points)
            self.publish_points_stamped(self.points)
            self.get_logger().info(f'Loaded {len(self.points)} points from {file_path}')
        else:
            self.get_logger().warn(f'No points loaded from {file_path}')

        # Subscription to local_map_raw point cloud
        self.local_map_sub = self.create_subscription(
            sensor_msgs.PointCloud2,
            local_map_topic,
            self.local_map_callback,
            10
        )
        self.get_logger().info(f'Subscribed to {local_map_topic}')

    def load_points(self, file_path):
        points = []
        if not file_path:
            self.get_logger().warn('未配置 clicked_points_file，跳过 ROI 多边形加载')
            return points
        try:
            with open(file_path, 'r') as f:
                lines = f.readlines()
                for line in lines:
                    line = line.strip()
                    if line.startswith('#') or line == '':
                        continue
                    # format could be comma-separated: index,x,y,z
                    # or space separated: index x y z
                    if ',' in line:
                        parts = line.split(',')
                    else:
                        parts = line.split()
                    # Remove empty strings
                    parts = [p for p in parts if p]
                    if len(parts) >= 4:
                        # index, x, y, z
                        x = float(parts[1])
                        y = float(parts[2])
                        z = float(parts[3])
                    elif len(parts) == 3:
                        # x y z (no index)
                        x = float(parts[0])
                        y = float(parts[1])
                        z = float(parts[2])
                    else:
                        self.get_logger().error(f'Invalid line: {line}')
                        continue
                    points.append((x, y, z))
        except Exception as e:
            self.get_logger().error(f'Failed to load file {file_path}: {e}')
        return points

    def publish_polygon(self, points):
        polygon = PolygonStamped()
        polygon.header = Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg())
        for (x, y, z) in points:
            point = Point32(x=float(x), y=float(y), z=float(z))
            polygon.polygon.points.append(point)
        self.polygon_pub.publish(polygon)

    def publish_marker(self, points):
        marker = Marker()
        marker.header = Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg())
        marker.ns = 'loaded_polygon'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        for (x, y, z) in points:
            from geometry_msgs.msg import Point
            p = Point(x=float(x), y=float(y), z=float(z))
            marker.points.append(p)
        # close polygon
        if len(points) > 0:
            (x, y, z) = points[0]
            marker.points.append(Point(x=float(x), y=float(y), z=float(z)))
        self.marker_pub.publish(marker)

    def publish_points_markers(self, points):
        marker_array = MarkerArray()
        for i, (x, y, z) in enumerate(points):
            marker = Marker()
            marker.header = Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg())
            marker.ns = 'loaded_points'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(z)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)
        # Add center point marker if available
        if self.center_point is not None:
            cx, cy, cz = self.center_point
            center_marker = Marker()
            center_marker.header = Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg())
            center_marker.ns = 'loaded_points'
            center_marker.id = len(points)  # unique ID after the existing points
            center_marker.type = Marker.SPHERE
            center_marker.action = Marker.ADD
            center_marker.pose.position.x = float(cx)
            center_marker.pose.position.y = float(cy)
            center_marker.pose.position.z = float(cz)
            center_marker.pose.orientation.w = 1.0
            center_marker.scale.x = 0.5
            center_marker.scale.y = 0.5
            center_marker.scale.z = 0.5
            center_marker.color.r = 1.0
            center_marker.color.g = 0.0
            center_marker.color.b = 0.0
            center_marker.color.a = 1.0
            marker_array.markers.append(center_marker)
        self.points_marker_pub.publish(marker_array)

    def publish_points_stamped(self, points):
        for (x, y, z) in points:
            point = PointStamped()
            point.header = Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg())
            point.point.x = float(x)
            point.point.y = float(y)
            point.point.z = float(z)
            self.point_pub.publish(point)
        # Also publish center point if available
        if self.center_point is not None:
            cx, cy, cz = self.center_point
            center_point = PointStamped()
            center_point.header = Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg())
            center_point.point.x = float(cx)
            center_point.point.y = float(cy)
            center_point.point.z = float(cz)
            self.point_pub.publish(center_point)

    def local_map_callback(self, msg):
        """
        Callback for local_map_raw point cloud.
        Transform all points to map frame, check if any point is within radius of center point.
        """
        # self.get_logger().info(f'Received local_map_raw point cloud with frame {msg.header.frame_id}, stamp {msg.header.stamp.sec}.{msg.header.stamp.nanosec}, height={msg.height}, width={msg.width}')
        if self.center_point is None:
            # No center point defined, cannot check radius, return without publishing
            self.get_logger().warn('Center point not defined, cannot check radius, returning without publishing')
            return

        # Get transform from point cloud frame to map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            self.get_logger().debug(f'Transform obtained: translation=({transform.transform.translation.x:.3f}, {transform.transform.translation.y:.3f}, {transform.transform.translation.z:.3f}), rotation=({transform.transform.rotation.x:.3f}, {transform.transform.rotation.y:.3f}, {transform.transform.rotation.z:.3f}, {transform.transform.rotation.w:.3f})')
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Transform lookup failed: {e}')
            return
        except Exception as e:
            self.get_logger().error(f'Transform error: {e}')
            return

        cx, cy, cz = self.center_point
        radius_sq = self.radius ** 2
        # self.get_logger().info(f'Checking points within radius {self.radius} of center ({cx:.3f}, {cy:.3f}, {cz:.3f})')

        # Extract points as numpy array (N,3)
        try:
            # Read all points at once for efficiency
            points_gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            # Convert to list of tuples, each tuple contains (x, y, z) as floats
            points_list = []
            for p in points_gen:
                # p is a tuple of floats or a structured array with fields
                # Ensure we extract x, y, z correctly
                if hasattr(p, '__len__') and len(p) >= 3:
                    x = float(p[0])
                    y = float(p[1])
                    z = float(p[2])
                    points_list.append([x, y, z])
                else:
                    self.get_logger().warn(f'Unexpected point format: {p}')
            if not points_list:
                self.get_logger().info('Point cloud is empty, returning without publishing')
                return
            points_np = np.array(points_list, dtype=np.float32)
            # points_np shape: (N,3)
        except Exception as e:
            self.get_logger().error(f'Failed to extract points from point cloud: {e}')
            return

        # Transform points using transformation matrix
        # Compute rotation matrix and translation vector from transform
        t = transform.transform.translation
        q = transform.transform.rotation
        # Build rotation matrix from quaternion (w, x, y, z)
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ], dtype=np.float32)
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)

        # Apply transformation: points_transformed = (R @ points_np.T).T + translation
        points_transformed = (R @ points_np.T).T + translation

        # Compute squared distances to center
        diff = points_transformed - np.array([cx, cy, cz], dtype=np.float32)
        dist_sq = np.sum(diff * diff, axis=1)

        # Check if any point within radius
        within_mask = dist_sq <= radius_sq
        current_flag = bool(np.any(within_mask))
        
        if current_flag:
            # Found at least one point within radius
            within_indices = np.where(within_mask)[0]
            first_idx = within_indices[0]
            point_pos = points_transformed[first_idx]
            distance = np.sqrt(dist_sq[first_idx])
            # self.get_logger().info(f'Point within radius: transformed=({point_pos[0]:.3f}, {point_pos[1]:.3f}, {point_pos[2]:.3f}), distance={distance:.2f} <= {self.radius}')
        else:
            self.get_logger().info(f'No point within radius. Processed {len(points_np)} points.')
        
        # Publish flag directly
        flag_msg = Bool()
        flag_msg.data = bool(current_flag)
        self.flag_pub.publish(flag_msg)
        # self.get_logger().info(f'Published flag: {bool(current_flag)}')


    def publish_all(self):
        if self.points:
            self.publish_polygon(self.points)
            self.publish_marker(self.points)
            self.publish_points_markers(self.points)
            self.publish_points_stamped(self.points)

def main(args=None):
    rclpy.init(args=args)
    node = PolygonLoader()
    # Create a timer to publish periodically (every 1 second)
    node.create_timer(1.0, node.publish_all)
    # Keep node alive
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
