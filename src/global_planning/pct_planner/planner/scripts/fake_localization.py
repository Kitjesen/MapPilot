#!/usr/bin/env python3
"""
测试用：无真定位时发布 TF map->body，订阅 /initialpose（RViz「2D Pose Estimate」）。
仅与 test 类 launch 配合使用，正式规划用 system_launch + 真实定位。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class FakeLocalization(Node):
    def __init__(self):
        super().__init__('fake_localization')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.current_pose = TransformStamped()
        self.current_pose.header.frame_id = 'map'
        self.current_pose.child_frame_id = 'body'
        self.current_pose.transform.rotation.w = 1.0

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.handle_initial_pose,
            10)
        self.timer = self.create_timer(0.02, self.publish_tf)
        self.get_logger().info("Fake Localization: 在 RViz 用「2D Pose Estimate」设起点。")

    def handle_initial_pose(self, msg):
        self.get_logger().info(f"起点: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}")
        self.current_pose.transform.translation.x = msg.pose.pose.position.x
        self.current_pose.transform.translation.y = msg.pose.pose.position.y
        self.current_pose.transform.translation.z = msg.pose.pose.position.z
        self.current_pose.transform.rotation = msg.pose.pose.orientation

    def publish_tf(self):
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.current_pose)


def main():
    rclpy.init()
    node = FakeLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
