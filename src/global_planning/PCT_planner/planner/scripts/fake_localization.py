#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class FakeLocalization(Node):
    def __init__(self):
        super().__init__('fake_localization')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 初始位置 (默认在原点)
        self.current_pose = TransformStamped()
        self.current_pose.header.frame_id = 'map'
        self.current_pose.child_frame_id = 'body' # 对应 planner 配置的 robot_frame
        self.current_pose.transform.rotation.w = 1.0

        # 订阅 RViz 的 2D Pose Estimate
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.handle_initial_pose,
            10)
            
        # 定时发布 TF (50Hz)
        self.timer = self.create_timer(0.02, self.publish_tf)
        self.get_logger().info("Fake Localization Started. Use '2D Pose Estimate' in RViz to set start point.")

    def handle_initial_pose(self, msg):
        self.get_logger().info(f"Set Start Pose: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}")
        self.current_pose.transform.translation.x = msg.pose.pose.position.x
        self.current_pose.transform.translation.y = msg.pose.pose.position.y
        # RViz 2D Pose Estimate default z is 0. 
        # If you need 3D start, you might need to hardcode z or add logic based on map height
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
    rclpy.shutdown()

if __name__ == '__main__':
    main()
