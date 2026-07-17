#!/usr/bin/env python3
"""
han_dog_bridge 驱动层集成测试 (T8)

验证 han_dog_bridge 节点:
  1. cmd_vel → gRPC Walk() 归一化正确
  2. 看门狗超时 → 自动发送零速
  3. IMU 流 → /Odometry 发布
  4. stop=2 → SitDown 调用

使用内置 mock gRPC CMS server，不需要真实机器狗。

运行方法:
  source /opt/ros/humble/setup.bash
  source ~/lingtu/install/setup.bash
  python3 tests/integration/test_dog_bridge.py
"""

import asyncio
import json
import math
import sys
import threading
import time
from concurrent import futures

import pytest

pytest.importorskip("grpc", reason="Integration test — requires grpc")
pytest.importorskip("rclpy", reason="ROS2 integration test — requires rclpy")

import brainstem_api as dog_msg
import grpc
import rclpy
from geometry_msgs.msg import TwistStamped
from grpc import aio as grpc_aio
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Int8

# ─── Mock CMS Server ───────────────────────────────────────


class MockCmsServicer(dog_msg.RobotControlServicer):
    """Mock brainstem CMS that records all gRPC calls."""

    def __init__(self):
        self.walk_calls = []  # list of (x, y, z) tuples
        self.enable_count = 0
        self.standup_count = 0
        self.sitdown_count = 0
        self.disable_count = 0
        self._imu_running = True
        self._joint_running = True
        self._lock = threading.Lock()

    async def Enable(self, request, context):
        with self._lock:
            self.enable_count += 1
        return dog_msg.Empty()

    async def Disable(self, request, context):
        with self._lock:
            self.disable_count += 1
        return dog_msg.Empty()

    async def StandUp(self, request, context):
        with self._lock:
            self.standup_count += 1
        return dog_msg.Empty()

    async def SitDown(self, request, context):
        with self._lock:
            self.sitdown_count += 1
        return dog_msg.Empty()

    async def Walk(self, request, context):
        with self._lock:
            self.walk_calls.append((request.x, request.y, request.z))
        return dog_msg.Empty()

    async def ListenImu(self, request, context):
        """Stream fake IMU data at ~50Hz."""
        while self._imu_running:
            imu = dog_msg.Imu(
                gyroscope=dog_msg.Vector3(x=0.0, y=0.0, z=0.01),
                quaternion=dog_msg.Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
            )
            yield imu
            await asyncio.sleep(0.02)

    async def ListenJoint(self, request, context):
        """Stream fake joint data at ~50Hz."""
        while self._joint_running:
            joint = dog_msg.Joint(
                all_joints=dog_msg.AllJoints(
                    position=dog_msg.Matrix4(values=[0.0] * 16),
                    velocity=dog_msg.Matrix4(values=[0.0] * 16),
                    torque=dog_msg.Matrix4(values=[0.0] * 16),
                ),
            )
            yield joint
            await asyncio.sleep(0.02)

    async def ListenHistory(self, request, context):
        """Empty history stream."""
        while self._imu_running:
            await asyncio.sleep(1.0)

    async def GetParams(self, request, context):
        return dog_msg.Params()

    async def GetStartTime(self, request, context):
        return dog_msg.Timestamp()

    async def Tick(self, request, context):
        return dog_msg.Empty()

    async def Step(self, request, context):
        return dog_msg.Empty()

    def get_walk_calls(self):
        with self._lock:
            return list(self.walk_calls)

    def stop(self):
        self._imu_running = False
        self._joint_running = False


def start_mock_server(servicer, port=13199):
    """Start mock CMS gRPC server in a background thread."""
    loop = asyncio.new_event_loop()

    async def serve():
        server = grpc_aio.server()
        dog_msg.add_RobotControlServicer_to_server(servicer, server)
        server.add_insecure_port(f"0.0.0.0:{port}")
        await server.start()
        await server.wait_for_termination()

    def run():
        asyncio.set_event_loop(loop)
        loop.run_until_complete(serve())

    t = threading.Thread(target=run, daemon=True)
    t.start()
    time.sleep(1.0)  # Wait for server to start
    return loop


# ─── Test Harness Node ──────────────────────────────────────


class BridgeTestNode(Node):
    def __init__(self):
        super().__init__("bridge_test_node")
        self._lock = threading.Lock()

        # Publishers
        self.pub_cmd_vel = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.pub_stop = self.create_publisher(Int8, "/stop", 10)

        # Subscribers
        self.odom_msgs = []
        self.watchdog_msgs = []

        self.create_subscription(Odometry, "/Odometry", self._odom_cb, 10)
        self.create_subscription(Bool, "/driver/watchdog_active", self._wd_cb, 10)

    def _odom_cb(self, msg):
        with self._lock:
            self.odom_msgs.append(msg)

    def _wd_cb(self, msg):
        with self._lock:
            self.watchdog_msgs.append(msg.data)

    def send_cmd_vel(self, vx, vy, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "body"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.pub_cmd_vel.publish(msg)

    def send_stop(self, value):
        msg = Int8()
        msg.data = int(value)
        self.pub_stop.publish(msg)

    def get_odom_count(self):
        with self._lock:
            return len(self.odom_msgs)

    def get_watchdog_states(self):
        with self._lock:
            return list(self.watchdog_msgs)


# ─── Main Test ──────────────────────────────────────────────


def main():
    MOCK_PORT = 13199

    print("=" * 60)
    print("  han_dog_bridge Integration Test (T8) — Mock CMS")
    print("=" * 60)

    # 1. Start mock CMS server
    print("\n[Step 1] Starting mock CMS server on port", MOCK_PORT)
    servicer = MockCmsServicer()
    mock_loop = start_mock_server(servicer, MOCK_PORT)

    # 2. Start han_dog_bridge via subprocess (it's a ROS2 node)
    print("[Step 2] Starting han_dog_bridge → localhost:", MOCK_PORT)

    rclpy.init()
    test_node = BridgeTestNode()

    # Import and start han_dog_bridge in-process
    # We need to set parameters before creating the node
    import importlib.util
    import os

    # Find han_dog_bridge.py
    bridge_paths = [
        os.path.expanduser("~/data/SLAM/navigation/install/robot_driver/lib/robot_driver/han_dog_bridge.py"),
        os.path.expanduser("~/data/SLAM/navigation/src/drivers/robot_driver/han_dog_bridge.py"),
    ]
    bridge_path = None
    for p in bridge_paths:
        if os.path.exists(p):
            bridge_path = p
            break

    if bridge_path is None:
        print("[ERROR] Cannot find han_dog_bridge.py")
        rclpy.shutdown()
        sys.exit(1)

    print(f"  Using: {bridge_path}")

    # Load module dynamically
    spec = importlib.util.spec_from_file_location("han_dog_bridge", bridge_path)
    bridge_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bridge_mod)

    # Create bridge node with mock server params
    # Override parameters via ROS2 parameter mechanism
    bridge_node = bridge_mod.HanDogBridge.__new__(bridge_mod.HanDogBridge)
    # We need to init it with correct params pointing to localhost
    # Use rclpy override: set params before init
    from rclpy.parameter import Parameter

    param_overrides = [
        Parameter("dog_host", Parameter.Type.STRING, "127.0.0.1"),
        Parameter("dog_port", Parameter.Type.INTEGER, MOCK_PORT),
        Parameter("cmd_vel_timeout_ms", Parameter.Type.DOUBLE, 200.0),
        Parameter("auto_enable", Parameter.Type.BOOL, True),
        Parameter("auto_standup", Parameter.Type.BOOL, True),
    ]

    # Re-init properly
    # Use rclpy.create_node alternative: just create fresh with overrides
    import subprocess

    # Actually, simpler: start han_dog_bridge as a subprocess with --ros-args
    bridge_proc = subprocess.Popen(
        [
            "python3",
            bridge_path,
            "--ros-args",
            "-p",
            "dog_host:=127.0.0.1",
            "-p",
            f"dog_port:={MOCK_PORT}",
            "-p",
            "cmd_vel_timeout_ms:=200.0",
            "-p",
            "auto_enable:=true",
            "-p",
            "auto_standup:=true",
            "-p",
            "odom_pub_rate:=10.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    print(f"  Bridge PID: {bridge_proc.pid}")

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(test_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    results = {}

    # Wait for bridge to connect to mock CMS
    print("\n[Warmup] Waiting 5s for bridge to connect + auto enable/standup...")
    time.sleep(5.0)

    # Check auto-enable and auto-standup
    results["auto_enable"] = servicer.enable_count > 0
    results["auto_standup"] = servicer.standup_count > 0
    print(f"  [{'PASS' if results['auto_enable'] else 'FAIL'}] auto_enable: {servicer.enable_count} Enable() calls")
    print(f"  [{'PASS' if results['auto_standup'] else 'FAIL'}] auto_standup: {servicer.standup_count} StandUp() calls")

    # ── Phase 1: cmd_vel → Walk() ──
    print("\n[Phase 1] cmd_vel → Walk() normalization (3s)...")
    n_before = len(servicer.get_walk_calls())
    for _ in range(30):  # 3s at 10Hz
        test_node.send_cmd_vel(0.5, 0.0, 0.1)
        time.sleep(0.1)

    walks = servicer.get_walk_calls()[n_before:]
    walk_received = len(walks) > 0
    results["walk_calls_received"] = walk_received
    print(f"  [{'PASS' if walk_received else 'FAIL'}] walk_calls_received: {len(walks)} Walk() calls")

    if walks:
        # Check normalization: vx=0.5 / max_linear=1.0 → nx=0.5
        last_walk = walks[-1]
        nx_correct = abs(last_walk[0] - 0.5) < 0.01
        nz_correct = abs(last_walk[2] - 0.1) < 0.01
        results["walk_normalization"] = nx_correct and nz_correct
        print(
            f"  [{'PASS' if nx_correct and nz_correct else 'FAIL'}] "
            f"walk_normalization: nx={last_walk[0]:.3f} (expect 0.5), "
            f"nz={last_walk[2]:.3f} (expect 0.1)"
        )
    else:
        results["walk_normalization"] = False
        print("  [FAIL] walk_normalization: no Walk() calls")

    # ── Phase 2: Watchdog ──
    print("\n[Phase 2] Watchdog — stop sending cmd_vel (2s)...")
    time.sleep(2.0)

    # Check for zero-velocity Walk calls
    recent_walks = servicer.get_walk_calls()[-10:]
    has_zero = any(abs(w[0]) < 0.01 and abs(w[1]) < 0.01 and abs(w[2]) < 0.01 for w in recent_walks)
    results["watchdog_zero_vel"] = has_zero
    print(
        f"  [{'PASS' if has_zero else 'FAIL'}] watchdog_zero_vel: zero Walk() found in last {len(recent_walks)} calls"
    )

    # Check watchdog_active topic
    wd_states = test_node.get_watchdog_states()
    has_wd_true = True in wd_states
    results["watchdog_topic_active"] = has_wd_true
    print(
        f"  [{'PASS' if has_wd_true else 'FAIL'}] watchdog_topic_active: {wd_states[-5:] if wd_states else 'no msgs'}"
    )

    # ── Phase 3: Odometry from IMU ──
    print("\n[Phase 3] Odometry publishing (check IMU → /Odometry)...")
    odom_count = test_node.get_odom_count()
    odom_received = odom_count > 0
    results["odom_published"] = odom_received
    print(f"  [{'PASS' if odom_received else 'FAIL'}] odom_published: {odom_count} Odometry msgs")

    # ── Phase 4: stop=2 → SitDown ──
    print("\n[Phase 4] stop=2 → SitDown...")
    sitdown_before = servicer.sitdown_count
    test_node.send_stop(2)
    time.sleep(1.0)
    sitdown_after = servicer.sitdown_count
    sitdown_triggered = sitdown_after > sitdown_before
    results["stop_triggers_sitdown"] = sitdown_triggered
    print(
        f"  [{'PASS' if sitdown_triggered else 'FAIL'}] stop_triggers_sitdown: "
        f"SitDown count {sitdown_before} → {sitdown_after}"
    )

    # ── Cleanup ──
    bridge_proc.terminate()
    bridge_proc.wait(timeout=5)
    servicer.stop()
    executor.shutdown()
    test_node.destroy_node()
    rclpy.shutdown()

    # ── Summary ──
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
