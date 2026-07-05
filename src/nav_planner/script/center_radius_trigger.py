#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import numpy as np

class CenterRadiusTrigger(Node):
    def __init__(self):
        super().__init__('center_radius_trigger')
        # Parameters
        self.declare_parameter('forward_goal.x', 0.0)
        self.declare_parameter('forward_goal.y', 0.0)
        self.declare_parameter('forward_goal.z', 0.0)
        self.declare_parameter('home_goal.x', 0.0)
        self.declare_parameter('home_goal.y', 0.0)
        self.declare_parameter('home_goal.z', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('history_size', 3)

        # Load parameters
        fx = self.get_parameter('forward_goal.x').value
        fy = self.get_parameter('forward_goal.y').value
        fz = self.get_parameter('forward_goal.z').value
        hx = self.get_parameter('home_goal.x').value
        hy = self.get_parameter('home_goal.y').value
        hz = self.get_parameter('home_goal.z').value
        self.frame_id = self.get_parameter('frame_id').value
        clicked_point_topic = self.get_parameter('clicked_point_topic').value
        self.history_size = self.get_parameter('history_size').value

        # Create goal points
        self.forward_goal = self.create_point_stamped(fx, fy, fz)
        self.home_goal = self.create_point_stamped(hx, hy, hz)

        # Publisher for clicked point
        self.clicked_pub = self.create_publisher(PointStamped, clicked_point_topic, 10)

        # Subscription to /in_center_radius
        self.flag_sub = self.create_subscription(
            Bool,
            '/in_center_radius',
            self.flag_callback,
            10
        )

        # State
        self.last_value = None  # previous boolean value

        self.get_logger().info('CenterRadiusTrigger node started')
        self.get_logger().info(f'Forward goal: ({fx}, {fy}, {fz})')
        self.get_logger().info(f'Home goal: ({hx}, {hy}, {hz})')

    def create_point_stamped(self, x, y, z):
        point = PointStamped()
        point.header.frame_id = self.frame_id
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = x
        point.point.y = y
        point.point.z = z
        return point

    def flag_callback(self, msg):
        """
        Process incoming boolean flag.
        When the flag changes from false to true, publish forward goal.
        When the flag changes from true to false, publish home goal.
        """
        value = msg.data
        if self.last_value is None:
            # First message, just store the value
            self.last_value = value
            return

        if self.last_value == False and value == True:
            # Transition from false to true
            self.get_logger().info('Transition from false to true -> publish forward goal')
            self.publish_goal(self.forward_goal)
        elif self.last_value == True and value == False:
            # Transition from true to false
            self.get_logger().info('Transition from true to false -> publish home goal')
            self.publish_goal(self.home_goal)
        # else no change, do nothing

        self.last_value = value

    def publish_goal(self, goal_point):
        goal_point.header.stamp = self.get_clock().now().to_msg()
        self.clicked_pub.publish(goal_point)
        self.get_logger().info(f'Published clicked point: ({goal_point.point.x}, {goal_point.point.y}, {goal_point.point.z})')

def main(args=None):
    rclpy.init(args=args)
    node = CenterRadiusTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
