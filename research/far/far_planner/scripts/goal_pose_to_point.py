#!/usr/bin/env python3
"""
Goal Adapter: PoseStamped → PointStamped

灵途系统使用 PoseStamped 发送导航目标，
FAR Planner 接收 PointStamped。此节点做转换。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped


class GoalPoseToPoint(Node):
    def __init__(self):
        super().__init__('far_goal_adapter')
        self.sub = self.create_subscription(
            PoseStamped, '/goal_pose_in', self.cb, 1)
        self.pub = self.create_publisher(
            PointStamped, '/goal_point_out', 1)
        self.get_logger().info('FAR goal adapter ready: PoseStamped → PointStamped')

    def cb(self, msg: PoseStamped):
        pt = PointStamped()
        pt.header = msg.header
        pt.point = msg.pose.position
        self.pub.publish(pt)
        self.get_logger().info(
            f'Goal forwarded: ({pt.point.x:.1f}, {pt.point.y:.1f}, {pt.point.z:.1f})')


def main():
    rclpy.init()
    node = GoalPoseToPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
