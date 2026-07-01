#!/usr/bin/env python3
"""
Bridge: /nav/semantic/resolved_goal (PoseStamped) → /nav/way_point (PointStamped)

无全局规划器时，直接把语义目标转为 local_planner 的航点。
适用于相机导航模式（无 LiDAR 地图）。

用法:
    python3 tools/navigation/goal_to_waypoint.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PointStamped


class GoalToWaypoint(Node):
    def __init__(self):
        super().__init__('goal_to_waypoint')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.pub = self.create_publisher(PointStamped, '/nav/way_point', qos)
        # nav-semantic remaps resolved_goal → /nav/goal_pose
        self.sub = self.create_subscription(
            PoseStamped, '/nav/goal_pose', self._cb, qos)
        self.get_logger().info('goal_to_waypoint: waiting for semantic goals...')

    def _cb(self, msg: PoseStamped) -> None:
        wp = PointStamped()
        wp.header = msg.header
        wp.header.frame_id = 'map'
        wp.point = msg.pose.position
        self.pub.publish(wp)
        x, y = msg.pose.position.x, msg.pose.position.y
        self.get_logger().info(f'goal_to_waypoint: → ({x:.2f}, {y:.2f})')


def main():
    rclpy.init()
    node = GoalToWaypoint()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
