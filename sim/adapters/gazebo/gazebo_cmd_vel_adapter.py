#!/usr/bin/env python3
"""Convert LingTu TwistStamped commands into Gazebo Twist commands.

LingTu uses `/nav/cmd_vel` as `geometry_msgs/TwistStamped` so command frames
and timestamps remain explicit. ros_gz_bridge commonly exposes Gazebo velocity
controllers as `geometry_msgs/Twist`, so this adapter is the narrow conversion
boundary for Gazebo simulation.

The adapter also publishes a zero Twist when no LingTu command is fresh. That
keeps Gazebo's diff-drive controller actively braked during startup and between
tests, without creating any `/nav/*` command or hardware-side output.
"""

from __future__ import annotations


def main() -> None:
    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = Node("lingtu_gazebo_cmd_vel_adapter")
    node.declare_parameter("hold_publish_hz", 20.0)
    node.declare_parameter("command_timeout_sec", 0.35)
    hold_publish_hz = max(1.0, float(node.get_parameter("hold_publish_hz").value))
    command_timeout_sec = max(0.0, float(node.get_parameter("command_timeout_sec").value))
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(Twist, "/lingtu/gazebo/cmd_vel", qos)
    last_cmd = Twist()
    last_cmd_time = 0.0

    def on_cmd(msg: TwistStamped) -> None:
        nonlocal last_cmd, last_cmd_time
        last_cmd = Twist()
        last_cmd.linear = msg.twist.linear
        last_cmd.angular = msg.twist.angular
        last_cmd_time = node.get_clock().now().nanoseconds * 1e-9

    def publish_hold_command() -> None:
        age = node.get_clock().now().nanoseconds * 1e-9 - last_cmd_time
        pub.publish(last_cmd if last_cmd_time > 0.0 and age <= command_timeout_sec else Twist())

    node.create_subscription(TwistStamped, "/nav/cmd_vel", on_cmd, qos)
    node.create_timer(1.0 / hold_publish_hz, publish_hold_command)
    node.get_logger().info(
        "Gazebo cmd_vel adapter: /nav/cmd_vel TwistStamped -> "
        "/lingtu/gazebo/cmd_vel Twist with zero-hold braking"
    )

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
