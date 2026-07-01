#!/usr/bin/env python3
"""Simulation helper: mirror filtered Gazebo obstacle clouds into terrain topics.

Gazebo supplies a z/range-filtered /slam/map_cloud through the runtime adapter.
The legacy ROS localPlanner also waits for /nav/terrain_map and
/nav/terrain_map_ext, which sim_robot_node normally publishes. This node is
only for Gazebo navigation gates where sim_robot_node is disabled so Gazebo
remains the odometry source.
"""

from __future__ import annotations


def main() -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2
    except ImportError as exc:
        print(f"ROS 2 Python dependencies unavailable: {exc}", flush=True)
        return 2

    rclpy.init(args=None)
    node = Node("lingtu_terrain_passthrough")
    terrain_pub = node.create_publisher(PointCloud2, "/nav/terrain_map", 10)
    terrain_ext_pub = node.create_publisher(PointCloud2, "/nav/terrain_map_ext", 10)

    def on_cloud(msg: PointCloud2) -> None:
        terrain_pub.publish(msg)
        terrain_ext_pub.publish(msg)

    node.create_subscription(PointCloud2, "/slam/map_cloud", on_cloud, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
