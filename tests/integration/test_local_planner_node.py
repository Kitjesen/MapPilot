#!/usr/bin/env python3
"""
localPlanner + terrainAnalysis C++ 节点集成测试 (T2)

验证 localPlanner 接收 terrain_map + way_point 后输出局部路径,
以及 terrainAnalysis 正确处理合成点云。

前置启动 (另外的终端):
  source /opt/ros/humble/setup.bash
  source ~/lingtu/install/setup.bash

  # terrainAnalysis
  ros2 run terrain_analysis terrainAnalysis --ros-args \
      -r /Odometry:=/nav/odometry \
      -r /cloud_map:=/nav/map_cloud \
      -r /terrain_map:=/nav/terrain_map

  # localPlanner
  PATHS_DIR=~/lingtu/install/local_planner/share/local_planner/paths
  ros2 run local_planner localPlanner --ros-args \
      -r /Odometry:=/nav/odometry \
      -r /cloud_map:=/nav/map_cloud \
      -r /terrain_map:=/nav/terrain_map \
      -r /way_point:=/nav/way_point \
      -p pathFolder:=$PATHS_DIR \
      -p autonomyMode:=true \
      -p autonomySpeed:=1.0 \
      -p useTerrainAnalysis:=true \
      -p checkObstacle:=true

运行本测试:
  python3 tests/integration/test_local_planner_node.py
"""

import json
import math
import sys
import threading
import time

import numpy as np
import pytest

pytest.importorskip("rclpy", reason="ROS2 integration test — requires rclpy")

import rclpy
from geometry_msgs.msg import PointStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Int8

# ---------------------------------------------------------------------------
ODOM_HZ = 20
CLOUD_HZ = 5
WAYPOINT_HZ = 2
ROBOT_X, ROBOT_Y, ROBOT_Z = 0.0, 0.0, 0.5
ROBOT_YAW = 0.0

PHASE1_DURATION = 12.0  # normal operation: waypoint at (5,0)
PHASE2_DURATION = 10.0  # waypoint behind wall at (8,0), wall at x=5


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = float(math.sin(yaw / 2.0))
    q.w = float(math.cos(yaw / 2.0))
    return q


def make_xyzi_cloud(pts, frame_id, stamp):
    arr = np.ascontiguousarray(pts, dtype=np.float32)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(arr)
    msg.is_dense = False
    msg.is_bigendian = False
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step = 16 * len(arr)
    msg.data = arr.tobytes()
    return msg


def build_ground_cloud(include_wall=False):
    """Flat ground in body frame. Optionally add wall at x=5."""
    pts = []
    for x in np.arange(-5.0, 8.1, 0.4):
        for y in np.arange(-5.0, 5.1, 0.4):
            pts.append([x, y, -ROBOT_Z, 100.0])

    if include_wall:
        # Wall at x=5 (body frame), y in [-3, 3], z from ground to 1m above robot
        for y in np.arange(-3.0, 3.1, 0.2):
            for z in np.arange(-ROBOT_Z, 0.5, 0.15):
                pts.append([5.0, y, z, 100.0])

    return np.array(pts, dtype=np.float32)


class LocalPlannerTestNode(Node):
    def __init__(self):
        super().__init__("local_planner_test")
        self._lock = threading.Lock()

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, "/nav/odometry", 10)
        self.pub_cloud = self.create_publisher(PointCloud2, "/nav/map_cloud", 10)
        self.pub_waypoint = self.create_publisher(PointStamped, "/nav/way_point", 10)

        # Subscribers
        self.path_msgs = []
        self.stop_msgs = []

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Path, "/path", self._path_cb, qos)
        # localPlanner publishes /stop (internal name, no remap in manual start)
        self.create_subscription(Int8, "/stop", self._stop_cb, 10)
        self.create_subscription(Int8, "/nav/stop", self._stop_cb, 10)

    def _path_cb(self, msg):
        with self._lock:
            self.path_msgs.append(msg)

    def _stop_cb(self, msg):
        with self._lock:
            self.stop_msgs.append(msg)

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "body"
        msg.pose.pose.position.x = ROBOT_X
        msg.pose.pose.position.y = ROBOT_Y
        msg.pose.pose.position.z = ROBOT_Z
        msg.pose.pose.orientation = yaw_to_quat(ROBOT_YAW)
        self.pub_odom.publish(msg)

    def publish_cloud(self, include_wall=False):
        pts = build_ground_cloud(include_wall)
        stamp = self.get_clock().now().to_msg()
        msg = make_xyzi_cloud(pts, "body", stamp)
        self.pub_cloud.publish(msg)

    def publish_waypoint(self, x, y, z=0.0):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.pub_waypoint.publish(msg)

    def get_paths(self):
        with self._lock:
            return list(self.path_msgs)

    def get_stops(self):
        with self._lock:
            return list(self.stop_msgs)

    def clear(self):
        with self._lock:
            self.path_msgs.clear()
            self.stop_msgs.clear()


def main():
    print("=" * 60)
    print("  localPlanner + terrainAnalysis Integration Test (T2)")
    print("=" * 60)

    rclpy.init()
    node = LocalPlannerTestNode()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    results = {}

    # ── Warmup (3s) ──
    print("\n[Warmup] 3s...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        node.publish_odom()
        node.publish_cloud(include_wall=False)
        time.sleep(0.05)

    # ── Phase 1: Normal — waypoint at (5,0), no wall ──
    print(f"\n[Phase 1] Waypoint (5,0), no wall ({PHASE1_DURATION}s)...")
    node.clear()
    t0 = time.monotonic()
    while time.monotonic() - t0 < PHASE1_DURATION:
        elapsed = time.monotonic() - t0
        node.publish_odom()
        if int(elapsed * CLOUD_HZ) > int((elapsed - 0.05) * CLOUD_HZ):
            node.publish_cloud(include_wall=False)
        if int(elapsed * WAYPOINT_HZ) > int((elapsed - 0.05) * WAYPOINT_HZ):
            node.publish_waypoint(5.0, 0.0)
        time.sleep(0.05)

    paths = node.get_paths()
    path_received = len(paths) > 0
    results["path_output_received"] = path_received
    print(f"  {'PASS' if path_received else 'FAIL'} path_output_received: {len(paths)} path msgs")

    # Check path points exist and are reasonable
    if paths:
        last = paths[-1]
        n_poses = len(last.poses)
        path_has_poses = n_poses > 2
        results["path_has_poses"] = path_has_poses
        print(f"  {'PASS' if path_has_poses else 'FAIL'} path_has_poses: {n_poses} poses in last path")
    else:
        results["path_has_poses"] = False
        print("  FAIL path_has_poses: no path received")

    # ── Phase 2: Wall at x=5, waypoint at (8,0) ──
    print(f"\n[Phase 2] Waypoint (8,0), wall at x=5 ({PHASE2_DURATION}s)...")
    node.clear()
    t0 = time.monotonic()
    while time.monotonic() - t0 < PHASE2_DURATION:
        elapsed = time.monotonic() - t0
        node.publish_odom()
        if int(elapsed * CLOUD_HZ) > int((elapsed - 0.05) * CLOUD_HZ):
            node.publish_cloud(include_wall=True)
        if int(elapsed * WAYPOINT_HZ) > int((elapsed - 0.05) * WAYPOINT_HZ):
            node.publish_waypoint(8.0, 0.0)
        time.sleep(0.05)

    paths2 = node.get_paths()
    stops2 = node.get_stops()

    # Check if any path avoids the wall (no path point with 4.5 < x < 5.5 and |y| < 3)
    # OR check if stop=2 was triggered (near-field ESTOP because wall is close)
    stop_values = [m.data for m in stops2]
    has_estop = 2 in stop_values
    results["wall_triggers_response"] = has_estop or len(paths2) > 0

    if has_estop:
        print(f"  PASS wall_triggers_response: stop=2 (E-STOP) triggered ({stop_values.count(2)} times)")
    elif paths2:
        # Check if paths avoid the wall region
        wall_penetrations = 0
        for p in paths2[-3:]:  # check last few paths
            for pose in p.poses:
                px = pose.pose.position.x
                py = pose.pose.position.y
                if 4.5 < px < 5.5 and abs(py) < 2.5:
                    wall_penetrations += 1
        avoids_wall = wall_penetrations == 0
        results["wall_triggers_response"] = avoids_wall or has_estop
        print(
            f"  {'PASS' if avoids_wall else 'FAIL'} wall_triggers_response: "
            f"{wall_penetrations} wall penetrations in recent paths"
        )
    else:
        print("  FAIL wall_triggers_response: no paths or stops in Phase 2")

    # Check terrain_map was being consumed (indirectly: if localPlanner outputs
    # paths, it means terrain_map was received)
    results["terrain_chain_works"] = path_received
    print(
        f"  {'PASS' if path_received else 'FAIL'} terrain_chain_works: "
        f"localPlanner produced paths (implies terrain_map received)"
    )

    # ── Summary ──
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    all_pass = all(results.values())
    print("\n" + "=" * 60)
    for k, v in results.items():
        print(f"  [{'PASS' if v else 'FAIL'}] {k}")
    print(f"\n  {sum(results.values())}/{len(results)} passed")
    print("=" * 60)
    print(json.dumps(results, indent=2))

    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
