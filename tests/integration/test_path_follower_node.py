#!/usr/bin/env python3
"""
pathFollower C++ 节点集成测试

测试对象: local_planner::pathFollower (C++ ROS2 节点)
测试方式: 本脚本作为 publisher + subscriber，pathFollower 单独启动

运行步骤:
    # 1. SSH 到机器人 (192.168.66.190)
    # 2. 启动 pathFollower 节点:
    source /opt/ros/humble/setup.bash
    source ~/lingtu/install/setup.bash  # 或实际 workspace
    ros2 run local_planner pathFollower --ros-args \
        -r /Odometry:=/nav/odometry \
        -r /cmd_vel:=/nav/cmd_vel \
        -p autonomyMode:=true \
        -p autonomySpeed:=1.0 \
        -p stuck_timeout:=10.0 \
        -p stuck_dist_thre:=0.15

    # 3. 另一个终端运行本测试:
    source /opt/ros/humble/setup.bash
    python3 tests/integration/test_path_follower_node.py

测试阶段:
    Phase 1 (10s): 前进运动 — 发布直线路径，验证 cmd_vel 正向输出
    Phase 2 (5s):  停止信号 — 发布 stop=2，验证 cmd_vel 归零
    Phase 3 (18s): 卡死检测 — 固定 odom 不动，验证 WARN_STUCK + STUCK

输出: JSON 格式检查结果
"""

import json
import math
import sys
import threading
import time

import pytest

pytest.importorskip("rclpy", reason="ROS2 integration test — requires rclpy")

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, String


class PathFollowerTestNode(Node):
    """pathFollower 集成测试节点"""

    def __init__(self):
        super().__init__("path_follower_test")

        # ── Publishers ──
        # pathFollower 内部话题名: /Odometry, /path, /stop, /speed
        # /Odometry 和 /cmd_vel 通过 remap 映射到 /nav/ 前缀
        # /path, /stop, /speed 保持原名 (无 remap)
        self.pub_odom = self.create_publisher(Odometry, "/nav/odometry", 10)
        self.pub_path = self.create_publisher(Path, "/path", 10)
        self.pub_stop = self.create_publisher(Int8, "/stop", 10)
        self.pub_speed = self.create_publisher(Float32, "/speed", 10)

        # ── Subscribers ──
        self.cmd_vel_msgs = []
        self.planner_status_msgs = []
        self._lock = threading.Lock()

        self.create_subscription(TwistStamped, "/nav/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(String, "/nav/planner_status", self._on_planner_status, 10)

        # ── State ──
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0
        self.odom_yaw = 0.0

    def _on_cmd_vel(self, msg):
        with self._lock:
            self.cmd_vel_msgs.append(msg)

    def _on_planner_status(self, msg):
        with self._lock:
            self.planner_status_msgs.append(msg)

    def clear_cmd_vel(self):
        with self._lock:
            self.cmd_vel_msgs.clear()

    def clear_planner_status(self):
        with self._lock:
            self.planner_status_msgs.clear()

    def get_cmd_vel_snapshot(self):
        with self._lock:
            return list(self.cmd_vel_msgs)

    def get_planner_status_snapshot(self):
        with self._lock:
            return [m.data for m in self.planner_status_msgs]

    def publish_odom(self):
        """发布当前 odom 位姿"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "body"
        msg.pose.pose.position.x = self.odom_x
        msg.pose.pose.position.y = self.odom_y
        msg.pose.pose.position.z = self.odom_z
        # yaw → quaternion (roll=pitch=0)
        cy = math.cos(self.odom_yaw / 2.0)
        sy = math.sin(self.odom_yaw / 2.0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        # twist = 0 (robot not moving in odom, for stuck detection)
        self.pub_odom.publish(msg)

    def publish_straight_path(self, length=5.0, num_poses=20):
        """发布从 (0,0) 到 (length, 0) 的直线路径"""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        for i in range(num_poses):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = length * i / (num_poses - 1)
            ps.pose.position.y = 0.0
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def publish_stop(self, value):
        """发布 stop 信号"""
        msg = Int8()
        msg.data = value
        self.pub_stop.publish(msg)

    def publish_speed(self, value):
        """发布 speed 倍率"""
        msg = Float32()
        msg.data = value
        self.pub_speed.publish(msg)


def spin_for(
    node, duration, odom_hz=20, path_hz=0, stop_val=None, speed_val=None, stop_hz=2, speed_hz=2, path_length=5.0
):
    """
    在指定时间内持续 spin 节点并按频率发布消息。

    Args:
        node: PathFollowerTestNode
        duration: 持续时间 (s)
        odom_hz: odom 发布频率
        path_hz: path 发布频率 (0 = 不发布)
        stop_val: stop 值 (None = 不发布)
        speed_val: speed 值 (None = 不发布)
        stop_hz: stop 发布频率
        speed_hz: speed 发布频率
        path_length: 路径长度
    """
    start = time.monotonic()

    odom_interval = 1.0 / odom_hz if odom_hz > 0 else 999
    path_interval = 1.0 / path_hz if path_hz > 0 else 999
    stop_interval = 1.0 / stop_hz if stop_hz > 0 else 999
    speed_interval = 1.0 / speed_hz if speed_hz > 0 else 999

    last_odom = 0.0
    last_path = 0.0
    last_stop = 0.0
    last_speed = 0.0

    while True:
        elapsed = time.monotonic() - start
        if elapsed >= duration:
            break

        now = time.monotonic()

        if now - last_odom >= odom_interval:
            node.publish_odom()
            last_odom = now

        if path_hz > 0 and now - last_path >= path_interval:
            node.publish_straight_path(length=path_length)
            last_path = now

        if stop_val is not None and now - last_stop >= stop_interval:
            node.publish_stop(stop_val)
            last_stop = now

        if speed_val is not None and now - last_speed >= speed_interval:
            node.publish_speed(speed_val)
            last_speed = now

        rclpy.spin_once(node, timeout_sec=0.01)


def main():
    print("=" * 60)
    print("  pathFollower C++ Node Integration Test")
    print("=" * 60)

    try:
        rclpy.init()
    except Exception as e:
        print(f"[FATAL] ROS2 init failed: {e}")
        print("  Please run: source /opt/ros/humble/setup.bash")
        sys.exit(1)

    node = PathFollowerTestNode()

    # Start a spinner thread so callbacks fire during sleeps
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    results = {}

    try:
        # ──────────────────────────────────────────────────────────
        # Warm-up: Ensure pathFollower has received odom + path
        # stopHandler needs 3 consecutive stop=0 to clear safetyStop
        # ──────────────────────────────────────────────────────────
        print("\n[Warm-up] Sending initial odom + path + stop=0 (3s)...")
        warmup_start = time.monotonic()
        while time.monotonic() - warmup_start < 3.0:
            node.publish_odom()
            node.publish_straight_path(length=5.0)
            node.publish_stop(0)
            node.publish_speed(1.0)
            time.sleep(0.05)  # 20Hz

        # ──────────────────────────────────────────────────────────
        # Phase 1: Forward motion (10s)
        # Robot at (0,0,0) yaw=0, path from (0,0) to (5,0)
        # Expect cmd_vel.linear.x > 0.05
        # ──────────────────────────────────────────────────────────
        print("\n[Phase 1] Forward motion test (10s)...")
        node.odom_x = 0.0
        node.odom_y = 0.0
        node.odom_yaw = 0.0
        node.clear_cmd_vel()
        node.clear_planner_status()

        phase1_start = time.monotonic()
        while time.monotonic() - phase1_start < 10.0:
            node.publish_odom()
            node.publish_straight_path(length=5.0)
            node.publish_stop(0)
            node.publish_speed(1.0)
            time.sleep(0.05)  # 20Hz

        # Analyze Phase 1 cmd_vel
        cvs = node.get_cmd_vel_snapshot()
        print(f"  Received {len(cvs)} cmd_vel messages")

        if len(cvs) == 0:
            print("  [FAIL] No cmd_vel received! Is pathFollower running?")
            results["forward_cmd_vel_positive"] = False
            results["forward_lateral_minimal"] = False
            results["forward_rotation_minimal"] = False
        else:
            # Use messages from the second half (after speed ramp-up)
            half = len(cvs) // 2
            late_cvs = cvs[half:]

            fwd_speeds = [m.twist.linear.x for m in late_cvs]
            lat_speeds = [abs(m.twist.linear.y) for m in late_cvs]
            yaw_rates = [abs(m.twist.angular.z) for m in late_cvs]

            avg_fwd = sum(fwd_speeds) / len(fwd_speeds)
            max_lat = max(lat_speeds) if lat_speeds else 0.0
            max_yaw = max(yaw_rates) if yaw_rates else 0.0

            forward_ok = avg_fwd > 0.05
            lateral_ok = max_lat < 0.1
            rotation_ok = max_yaw < 0.3

            results["forward_cmd_vel_positive"] = forward_ok
            results["forward_lateral_minimal"] = lateral_ok
            results["forward_rotation_minimal"] = rotation_ok

            status = "PASS" if forward_ok else "FAIL"
            print(f"  [{status}] forward_cmd_vel_positive: avg_fwd={avg_fwd:.4f} (>0.05)")
            status = "PASS" if lateral_ok else "FAIL"
            print(f"  [{status}] forward_lateral_minimal: max_lat={max_lat:.4f} (<0.1)")
            status = "PASS" if rotation_ok else "FAIL"
            print(f"  [{status}] forward_rotation_minimal: max_yaw={max_yaw:.4f} (<0.3)")

        # ──────────────────────────────────────────────────────────
        # Phase 2: Stop signal (5s)
        # Continue odom + path, publish stop=2
        # Expect cmd_vel goes to ~zero
        # ──────────────────────────────────────────────────────────
        print("\n[Phase 2] Stop signal test (5s)...")
        node.clear_cmd_vel()

        phase2_start = time.monotonic()
        while time.monotonic() - phase2_start < 5.0:
            node.publish_odom()
            node.publish_straight_path(length=5.0)
            node.publish_stop(2)
            node.publish_speed(1.0)
            time.sleep(0.05)

        cvs = node.get_cmd_vel_snapshot()
        print(f"  Received {len(cvs)} cmd_vel messages")

        if len(cvs) == 0:
            print("  [FAIL] No cmd_vel received during stop phase")
            results["stop_signal_zeroes_cmd"] = False
        else:
            # Check messages from the second half (after stop propagation)
            half = len(cvs) // 2
            late_cvs = cvs[half:]

            max_fwd = max(abs(m.twist.linear.x) for m in late_cvs)
            max_yaw = max(abs(m.twist.angular.z) for m in late_cvs)

            stop_ok = max_fwd < 0.05 and max_yaw < 0.05
            results["stop_signal_zeroes_cmd"] = stop_ok

            status = "PASS" if stop_ok else "FAIL"
            print(f"  [{status}] stop_signal_zeroes_cmd: max_fwd={max_fwd:.4f} (<0.05), max_yaw={max_yaw:.4f} (<0.05)")

        # ──────────────────────────────────────────────────────────
        # Phase 3: Stuck detection (18s)
        # Clear stop, re-publish path, keep odom fixed
        # stuck_timeout=10s => WARN_STUCK at ~5s, STUCK at ~10s
        #
        # Note: pathHandler resets stuckCheckInit_ = false, so the
        # stuck timer restarts from the next odomHandler callback.
        # We re-publish the path once to reset stuck state, then
        # keep odom fixed and stop=0 for 18s.
        # ──────────────────────────────────────────────────────────
        print("\n[Phase 3] Stuck detection test (18s)...")
        node.clear_planner_status()
        node.clear_cmd_vel()

        # First clear the stop signal — need 3 consecutive stop=0
        # to clear safetyStop in the C++ node
        print("  Clearing stop signal (sending 10x stop=0)...")
        for _ in range(10):
            node.publish_stop(0)
            time.sleep(0.05)

        # Re-publish path to reset stuck timer via pathHandler
        # (pathHandler sets stuckCheckInit_ = false)
        node.publish_straight_path(length=5.0)
        time.sleep(0.1)

        # Now keep odom fixed at (0,0) — robot doesn't move
        # Stuck detection triggers:
        #   WARN_STUCK at stuckTimeout * 0.5 = 5s
        #   STUCK at stuckTimeout = 10s
        phase3_start = time.monotonic()
        warn_stuck_time = None
        stuck_time = None

        while time.monotonic() - phase3_start < 18.0:
            node.publish_odom()  # Fixed position
            node.publish_stop(0)
            node.publish_speed(1.0)
            time.sleep(0.05)

            # Check for stuck status messages
            statuses = node.get_planner_status_snapshot()
            if warn_stuck_time is None:
                for s in statuses:
                    if s == "WARN_STUCK":
                        warn_stuck_time = time.monotonic() - phase3_start
                        print(f"  WARN_STUCK detected at t={warn_stuck_time:.1f}s")
                        break
            if stuck_time is None:
                for s in statuses:
                    if s == "STUCK":
                        stuck_time = time.monotonic() - phase3_start
                        print(f"  STUCK detected at t={stuck_time:.1f}s")
                        break

        results["warn_stuck_detected"] = warn_stuck_time is not None
        results["stuck_detected"] = stuck_time is not None

        status = "PASS" if results["warn_stuck_detected"] else "FAIL"
        print(
            f"  [{status}] warn_stuck_detected: "
            f"{'at ' + f'{warn_stuck_time:.1f}s' if warn_stuck_time else 'NOT detected'}"
        )
        status = "PASS" if results["stuck_detected"] else "FAIL"
        print(f"  [{status}] stuck_detected: {'at ' + f'{stuck_time:.1f}s' if stuck_time else 'NOT detected'}")

    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    # ──────────────────────────────────────────────────────────
    # Output JSON summary
    # ──────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("  Results Summary")
    print("=" * 60)

    all_pass = all(results.values())
    passed = sum(1 for v in results.values() if v)
    failed = sum(1 for v in results.values() if not v)

    for check, ok in results.items():
        tag = "PASS" if ok else "FAIL"
        print(f"  [{tag}] {check}")

    print(f"\n  Passed: {passed}/{len(results)}   Failed: {failed}/{len(results)}")

    print("\n[JSON]")
    print(json.dumps(results, indent=2))

    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
