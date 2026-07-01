#!/usr/bin/env python3
"""
Relay /nav/dog_odometry → /nav/odometry

用于无 LiDAR 时为 local_planner 提供里程计。
han_dog_bridge 的 IMU 积分里程计发到 /nav/dog_odometry，
local_planner 订阅 /nav/odometry，这里做中继并修正 frame_id。

用法:
    python3 tools/navigation/odom_relay.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry


class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.pub = self.create_publisher(Odometry, '/nav/odometry', qos)
        self.sub = self.create_subscription(
            Odometry, '/nav/dog_odometry', self._cb, qos)
        self.get_logger().info('odom_relay: /nav/dog_odometry → /nav/odometry')

    def _cb(self, msg: Odometry) -> None:
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'body'
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = OdomRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
