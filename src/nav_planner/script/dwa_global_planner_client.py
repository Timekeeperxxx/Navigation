#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dddmr_sys_core.action import GetPlan
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node

class DWAGlobalPlannerClient(Node):
    
    def __init__(self):

        super().__init__('dwa_global_planner_client')
        cg = MutuallyExclusiveCallbackGroup()
        self._action_client = ActionClient(self, GetPlan, 'get_dwa_plan', callback_group=cg)
        cg_loop = MutuallyExclusiveCallbackGroup()
        self.sending_loop = self.create_timer(0.5, self.sendingLoop, callback_group=cg_loop)

    def sendingLoop(self):
        self.send_goal()

    def send_goal(self):

        self.get_logger().debug("Sending Goal")
        
        self._action_client.wait_for_server()

        self.dwa_global_planner_goal = GetPlan.Goal()
        self.dwa_global_planner_goal.activate_threading = True

        a_pose = PoseStamped()
        a_pose.pose.position.x = 15.8
        a_pose.pose.position.y = 4.22
        a_pose.pose.position.z = 0.0
        a_pose.pose.orientation.w = 1.0
        a_pose.header.frame_id = "map"
        self.dwa_global_planner_goal.goal = a_pose
        
        self._send_goal_future = self._action_client.send_goal_async(self.dwa_global_planner_goal)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.get_logger().debug("Goal sent. Waiting response")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().debug('Goal rejected :(')
            return

        self.get_logger().debug('Goal accepted :)')

def main(args=None):

    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    DWAGPC = DWAGlobalPlannerClient()
    executor.add_node(DWAGPC)
    executor.spin()


if __name__ == '__main__':
    main()
