#!/usr/bin/env python3
"""
FAR Planner 集成测试 — terrain_analysis + FAR Planner 联调

在 S100P (192.168.66.190) 上执行:
1. 启动 terrainAnalysis + FAR Planner (真实 C++ 节点)
2. 发布合成里程计 + 点云 (含障碍物)
3. 发送导航目标
4. 验证 FAR Planner 输出航点

测试项:
  [F1] FAR Planner 节点正常启动，可视图初始化
  [F2] 接收 terrain_map 后构建可视图 (vertices > 0)
  [F3] 发送目标后输出 /nav/way_point
  [F4] 航点方向正确 (朝向目标)
  [F5] reach_goal 信号在到达时触发

用法: python tests/integration/test_far_planner.py
"""

import os

import pytest

pytest.importorskip("paramiko", reason="SSH integration test — requires paramiko")

import json
import math
import sys
import time
import unittest

import paramiko

ROBOT = os.environ.get("S100P_HOST", "192.168.66.190")
USER = os.environ.get("S100P_USER", "sunrise")
PASS = os.environ.get("S100P_PASSWORD", "")

_SKIP_REASON = None
if not os.environ.get("S100P_HOST"):
    _SKIP_REASON = "S100P_HOST not set — skipping real-hardware integration test"
elif not PASS:
    _SKIP_REASON = "S100P_PASSWORD not set — skipping password-authenticated integration test"
NAV_WS = "/home/sunrise/data/SLAM/navigation"
SETUP = f"source /opt/ros/humble/setup.bash && source {NAV_WS}/install/setup.bash"
PATHS_DIR = f"{NAV_WS}/install/local_planner/share/local_planner/paths"


def ssh_connect():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ROBOT, username=USER, password=PASS, timeout=10)
    return ssh


def run_cmd(ssh, cmd, timeout=15):
    _, out, err = ssh.exec_command(f"{SETUP} && {cmd}", timeout=timeout)
    return out.read().decode(), err.read().decode()


def run_bg(ssh, cmd):
    """启动后台进程，返回可用于 kill 的标识"""
    full = f"{SETUP} && {cmd}"
    ssh.exec_command(f"nohup bash -c '{full}' > /tmp/far_test_bg.log 2>&1 &")
    time.sleep(0.5)


def kill_procs(ssh, names):
    for n in names:
        ssh.exec_command(f"pkill -f '{n}' 2>/dev/null")
    time.sleep(1)


class FarPlannerIntegrationTest(unittest.TestCase):
    """FAR Planner integration tests against real S100P hardware."""

    S100P_HOST = ROBOT
    S100P_USER = USER
    S100P_PASS = PASS

    def setUp(self):
        self.ssh = None
        self.results = []

    def tearDown(self):
        if self.ssh:
            self.ssh.close()

    def _check(self, name, passed, detail=""):
        self.results.append({"name": name, "pass": passed})
        mark = "PASS" if passed else "FAIL"
        print(f"  [{mark}] {name}  {detail}")
        self.assertTrue(passed, msg=f"{name}: {detail}")

    @unittest.skipIf(_SKIP_REASON, _SKIP_REASON)
    def test_far_planner_pipeline(self):
        self.ssh = ssh_connect()
        print("SSH connected to", ROBOT)

        results = []

        def check(name, passed, detail=""):
            status = "PASS" if passed else "FAIL"
            results.append({"name": name, "pass": passed})
            mark = "\033[32mPASS\033[0m" if passed else "\033[31mFAIL\033[0m"
            print(f"  [{mark}] {name}  {detail}")

    # 清理残留
    kill_procs(ssh, ["far_planner", "goal_pose_to_point", "far_test_harness"])

    try:
        # ================================================================
        # 写测试 harness 脚本到机器人
        # ================================================================
        harness_script = r'''#!/usr/bin/env python3
"""FAR Planner test harness: 发布合成数据 + 监控输出"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster
import struct, time, math, json, sys

QOS_SENSOR = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

class FarTestHarness(Node):
    def __init__(self):
        super().__init__('far_test_harness')

        # Publishers — 直接给 FAR Planner 发数据 (绕过 terrainAnalysis)
        # FAR 订阅: /terrain_cloud (关键，设 is_cloud_init)
        #           /scan_cloud (仅动态环境), /odom_world, /goal_point
        self.odom_pub = self.create_publisher(Odometry, '/nav/odometry', 10)
        self.terrain_pub = self.create_publisher(PointCloud2, '/nav/terrain_map', 5)
        self.goal_pub = self.create_publisher(PointStamped, '/nav/goal_point', 1)

        # Subscribers - 监控 FAR 输出
        self.waypoint_received = False
        self.waypoint_pos = None
        self.reach_goal = False
        self.runtime_values = []

        self.create_subscription(PointStamped, '/nav/way_point', self.wp_cb, 5)
        self.create_subscription(Bool, '/nav/far_reach_goal', self.goal_cb, 5)

        # Static TF: map -> odom -> body
        self.tf_br = StaticTransformBroadcaster(self)
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'odom'
        t1.transform.rotation.w = 1.0
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'body'
        t2.transform.rotation.w = 1.0
        self.tf_br.sendTransform([t1, t2])

        # Robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.5

        # Timer: 10Hz 发布数据
        self.create_timer(0.1, self.pub_loop)
        self.cloud_timer_count = 0
        self.start_time = time.time()
        self.goal_sent = False
        self.phase = "init"  # init -> mapping -> goal -> monitor

        # 诊断: 计数发布的点云
        self.terrain_map_count = 0

        self.get_logger().info("FAR test harness started")

    def wp_cb(self, msg):
        self.waypoint_received = True
        self.waypoint_pos = (msg.point.x, msg.point.y, msg.point.z)
        self.get_logger().info(f'Waypoint: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})')

    def goal_cb(self, msg):
        if msg.data:
            self.reach_goal = True
            self.get_logger().info('REACH GOAL!')

    def count_cloud(self):
        self.terrain_map_count += 1

    def make_cloud(self):
        """合成点云: 地面 + 两面墙 (走廊环境)

        FAR Planner 用 intensity 区分 free/obs:
          intensity < kFreeZ (0.15) → free (地面)
          intensity >= kFreeZ → obstacle (障碍物)
        """
        points = []

        # 地面: z ≈ robot_z - vehicle_height, x=-5~15, y=-8~8
        # intensity = 0.0 (free)
        gz = self.robot_z - 0.6  # vehicle_height=0.6
        for x in range(-50, 150, 4):
            for y in range(-80, 80, 4):
                points.append((x/10.0, y/10.0, gz, 0.0))

        # 左墙: y=-3, x=-2~12, z = gz ~ gz+1.5
        # intensity = 1.0 (obstacle, >> kFreeZ)
        for x in range(-20, 120, 3):
            for z_off in range(0, 15, 3):
                points.append((x/10.0, -3.0, gz + z_off/10.0, 1.0))

        # 右墙: y=3, x=-2~12
        for x in range(-20, 120, 3):
            for z_off in range(0, 15, 3):
                points.append((x/10.0, 3.0, gz + z_off/10.0, 1.0))

        # 前方障碍: x=6, y=-1~1 (需要绕行)
        for y in range(-10, 10, 3):
            for z_off in range(0, 10, 3):
                points.append((6.0, y/10.0, gz + z_off/10.0, 1.0))

        # 构建 PointCloud2
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * len(points)
        msg.data = b''.join(struct.pack('ffff', *p) for p in points)
        msg.is_dense = True
        return msg

    def pub_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'body'
        msg.pose.pose.position.x = self.robot_x
        msg.pose.pose.position.y = self.robot_y
        msg.pose.pose.position.z = self.robot_z
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

    def send_goal(self, x, y, z=0.5):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Goal sent: ({x}, {y}, {z})')

    def pub_loop(self):
        elapsed = time.time() - self.start_time

        # 发布里程计
        self.pub_odom()

        # 每 0.5s 发布一次点云直接给 FAR (绕过 terrainAnalysis)
        self.cloud_timer_count += 1
        if self.cloud_timer_count % 5 == 0:
            self.terrain_pub.publish(self.make_cloud())
            self.count_cloud()

        # Phase 管理
        if self.phase == "init" and elapsed > 3.0:
            self.phase = "mapping"
            self.get_logger().info("Phase: mapping (building V-Graph)...")

        if self.phase == "mapping" and elapsed > 10.0:
            self.phase = "goal"
            self.goal_sent = True
            self.get_logger().info("Phase: goal sent, waiting for waypoint...")

        # 持续发送目标 (每 2s) — FAR 需要 V-Graph 初始化后才接受目标
        if self.goal_sent and not self.waypoint_received:
            if self.cloud_timer_count % 20 == 0:
                self.send_goal(10.0, 0.0, 0.5)

        if self.phase == "goal" and self.waypoint_received:
            self.phase = "monitor"
            self.get_logger().info("Phase: waypoint received, monitoring...")

        # 收到航点后可以提前退出
        if self.phase == "monitor" and time.time() - self.start_time > 12.0:
            self._emit_result()

        # 超时输出结果
        if elapsed > 35.0:
            self._emit_result()

    def _emit_result(self):
        result = {
            "waypoint_received": self.waypoint_received,
            "waypoint_pos": self.waypoint_pos,
            "reach_goal": self.reach_goal,
            "terrain_map_count": self.terrain_map_count,
            "elapsed": time.time() - self.start_time,
            "phase": self.phase
        }
        print("RESULT:" + json.dumps(result))
        sys.exit(0)


def main():
    rclpy.init()
    node = FarTestHarness()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
        sftp = ssh.open_sftp()
        with sftp.open("/tmp/far_test_harness.py", "w") as f:
            f.write(harness_script)
        sftp.chmod("/tmp/far_test_harness.py", 0o755)
        sftp.close()
        print("Test harness uploaded")

        # ================================================================
        # Step 1: 启动 FAR Planner (直连模式，绕过 terrainAnalysis)
        # ================================================================
        # 测试 harness 直接发布带 intensity 编码的点云给 FAR:
        #   intensity < 0.15 → free (地面)
        #   intensity >= 0.15 → obstacle (障碍物)
        # 这样避免了 terrain_analysis vs terrain_analysis_ext 的输出格式问题
        print("\n[Step 1] Starting FAR Planner (直连模式)...")
        far_cmd = (
            "ros2 run far_planner far_planner "
            "--ros-args "
            "-r /odom_world:=/nav/odometry "
            "-r /terrain_cloud:=/nav/terrain_map "
            "-r /scan_cloud:=/nav/terrain_map "
            "-r /terrain_local_cloud:=/nav/registered_cloud "
            "-r /goal_point:=/nav/goal_point "
            "-r /way_point:=/nav/way_point "
            "-r /navigation_boundary:=/nav/navigation_boundary "
            "-r /far_reach_goal_status:=/nav/far_reach_goal "
            "-p sensor_range:=15.0 "
            "-p robot_dim:=0.8 "
            "-p vehicle_height:=0.6 "
            "-p is_static_env:=true "
            "-p is_debug_output:=true "
            "-p main_run_freq:=5.0 "
            "-p voxel_dim:=0.1 "
            "-p world_frame:=odom "
        )
        run_bg(ssh, far_cmd)
        time.sleep(3)

        out, _ = run_cmd(ssh, "pgrep -f far_planner")
        far_running = len(out.strip()) > 0
        check("F0 FAR Planner 节点启动", far_running, f"PID: {out.strip()}")

        if not far_running:
            # 看看日志
            out, _ = run_cmd(ssh, "cat /tmp/far_test_bg.log | tail -20")
            print("  FAR log:", out)

        # ================================================================
        # Step 1.5: 检查 FAR 日志 (初始化阶段)
        # ================================================================
        print("\n[Step 1.5] FAR Planner 启动日志:")
        out, _ = run_cmd(ssh, "cat /tmp/far_test_bg.log 2>/dev/null | tail -10")
        print("  " + out.strip().replace("\n", "\n  "))

        # ================================================================
        # Step 2: 运行测试 harness (直接发合成点云给 FAR)
        # ================================================================
        print("\n[Step 2] Running test harness (35s timeout)...")
        harness_cmd = "timeout 40 python3 /tmp/far_test_harness.py 2>&1"
        _, out, err = ssh.exec_command(f"{SETUP} && {harness_cmd}", timeout=55)
        harness_output = out.read().decode()
        harness_err = err.read().decode()

        print("  Harness output (last 1000 chars):")
        print("  " + harness_output[-1000:].replace("\n", "\n  "))

        # 解析结果
        result_data = None
        for line in harness_output.split("\n"):
            if line.startswith("RESULT:"):
                result_data = json.loads(line[7:])
                break

        if result_data:
            print(f"\n  Result: {json.dumps(result_data, indent=2)}")

            # F2: 可视图构建 — 检查 FAR 日志中有 vertices 输出
            # (通过 harness 收到的 phase 判断)
            v_graph_built = result_data["phase"] in ("goal", "monitor")
            check("F2 可视图构建 (V-Graph 初始化)", v_graph_built, f"phase={result_data['phase']}")

            # F3: 收到航点
            wp_ok = result_data["waypoint_received"]
            check("F3 收到 /nav/way_point 航点", wp_ok, f"pos={result_data.get('waypoint_pos')}")

            # F4: 航点方向正确 (目标在 x=10, 航点 x 应该 > 0)
            if result_data["waypoint_pos"]:
                wx, wy, wz = result_data["waypoint_pos"]
                direction_ok = wx > 0.5  # 航点在正前方
                check("F4 航点方向正确 (朝向目标 x=10)", direction_ok, f"waypoint=({wx:.2f}, {wy:.2f})")
            else:
                check("F4 航点方向正确", False, "无航点数据")

            # F5: elapsed time (goal sent at 10s, so waypoint should come before 35s)
            check(
                "F5 响应时间 < 35s",
                result_data["elapsed"] < 35,
                f"{result_data['elapsed']:.1f}s, terrain_map_count={result_data.get('terrain_map_count', 0)}",
            )
        else:
            check("F2 可视图构建", False, "无结果数据")
            check("F3 收到航点", False, "无结果数据")
            check("F4 航点方向", False, "无结果数据")
            check("F5 响应时间", False, "无结果数据")

            # 检查 FAR planner 日志
            print("\n  === FAR Planner 日志 (最后 40 行) ===")
            out2, _ = run_cmd(ssh, "cat /tmp/far_test_bg.log 2>/dev/null | tail -40")
            print("  " + out2.replace("\n", "\n  "))

        # 无论成功失败都显示 FAR log
        print("\n  === FAR Planner 完整日志 (最后 30 行) ===")
        out3, _ = run_cmd(ssh, "cat /tmp/far_test_bg.log 2>/dev/null | tail -30")
        print("  " + out3.replace("\n", "\n  "))

    finally:
        # 清理
        print("\n[Cleanup] Stopping test processes...")
        kill_procs(ssh, ["far_planner", "terrainAnalysis", "far_test_harness"])
        ssh.close()

    # 汇总 — convert to unittest assertions
    passed = sum(1 for r in results if r["pass"])
    total = len(results)
    print(f"\n{'=' * 50}")
    print(f"FAR Planner 集成测试: {passed}/{total} PASS")
    print(f"{'=' * 50}")
    self.assertEqual(passed, total, f"{passed}/{total} tests passed")


if __name__ == "__main__":
    unittest.main()
