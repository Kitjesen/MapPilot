#!/usr/bin/env python3
"""
规划流水线集成测试
测试全局规划 → pct_adapters → 局部规划 → cmd_vel 完整链路

用法:
    # 先启动系统（stub 模式，无硬件）:
    ros2 launch launch/navigation_run.launch.py \\
        slam_profile:=stub planner_profile:=stub &
    sleep 10
    # 再运行测试:
    python3 tests/integration/test_planning_pipeline.py

    # 或直接运行 (自动检测是否有 ROS2 运行):
    bash tests/integration/test_planning_stub.sh

阶段:
  Stage 1  — 节点 & 话题存活检查
  Stage 2  — 话题命名规范 (/nav/* 前缀)
  Stage 3  — PCT Adapter: 注入全局路径 → 收到 way_point + status 事件
  Stage 4  — PCT Adapter: 航点推进 → waypoint_reached 事件
  Stage 5  — PCT Adapter: 到达终点 → goal_reached 事件 + 停发航点
  Stage 6  — 局部规划链: way_point + terrain_map → local_path → cmd_vel (可选)
"""

import json
import math
import sys
import threading
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, String


# ── 颜色 ──────────────────────────────────────────────────────────────────────
class C:
    GREEN  = '\033[0;32m'
    RED    = '\033[0;31m'
    YELLOW = '\033[1;33m'
    BLUE   = '\033[0;34m'
    BOLD   = '\033[1m'
    NC     = '\033[0m'


# ── QoS ───────────────────────────────────────────────────────────────────────
BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ─────────────────────────────────────────────────────────────────────────────
# 测试节点
# ─────────────────────────────────────────────────────────────────────────────
class PlanningTestNode(Node):
    """规划测试节点: 发布 fake 输入，监听输出话题"""

    def __init__(self):
        super().__init__('planning_test_node')

        # ── 发布者 ──
        self.pub_odom    = self.create_publisher(Odometry,    '/nav/odometry',      10)
        self.pub_path    = self.create_publisher(Path,        '/nav/global_path',   10)
        self.pub_way     = self.create_publisher(PointStamped,'/nav/way_point',     10)
        self.pub_terrain = self.create_publisher(PointCloud2, '/nav/terrain_map',   10)
        self.pub_te_ext  = self.create_publisher(PointCloud2, '/nav/terrain_map_ext', 10)
        self.pub_map_cld = self.create_publisher(PointCloud2, '/nav/map_cloud',     10)
        self.pub_speed   = self.create_publisher(Float32,     '/nav/speed',         10)

        # ── 订阅者 ──
        self.received: dict = defaultdict(list)
        self._lock = threading.Lock()

        self.create_subscription(PointStamped, '/nav/way_point',
                                 lambda m: self._record('/nav/way_point', m), 10)
        self.create_subscription(String, '/nav/planner_status',
                                 lambda m: self._record('/nav/planner_status', m), 10)
        self.create_subscription(Path, '/nav/local_path',
                                 lambda m: self._record('/nav/local_path', m), 10)
        self.create_subscription(TwistStamped, '/nav/cmd_vel',
                                 lambda m: self._record('/nav/cmd_vel', m),
                                 BEST_EFFORT_QOS)

        # 机器人当前位置 (odom frame)
        self.robot_x = 0.0
        self.robot_y = 0.0

        # 连续发布里程计的定时器 (10 Hz)
        self.create_timer(0.1, self._publish_odom)
        # 连续发布速度倍率 (1.0 = 正常)
        self.create_timer(0.2, self._publish_speed)

    def _record(self, topic, msg):
        with self._lock:
            self.received[topic].append(msg)

    def _publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'body'
        msg.pose.pose.position.x = self.robot_x
        msg.pose.pose.position.y = self.robot_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(msg)

    def _publish_speed(self):
        msg = Float32()
        msg.data = 1.0
        self.pub_speed.publish(msg)

    # ── 辅助: 等待某话题收到至少 n 条消息 ──────────────────────────────────────
    def wait_for(self, topic: str, count: int = 1, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            with self._lock:
                if len(self.received[topic]) >= count:
                    return True
        return False

    # ── 辅助: 检查收到的 planner_status 中是否有指定 event ─────────────────────
    def wait_for_status_event(self, event: str, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            with self._lock:
                for msg in self.received['/nav/planner_status']:
                    try:
                        data = json.loads(msg.data)
                        if data.get('event') == event:
                            return True
                    except (json.JSONDecodeError, AttributeError):
                        pass
        return False

    def clear_received(self, topic: str):
        with self._lock:
            self.received[topic].clear()

    # ── 路径构造 ─────────────────────────────────────────────────────────────
    def make_global_path(self, waypoints: list) -> Path:
        """构造 nav_msgs/Path (map frame)"""
        path = Path()
        path.header.stamp  = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for x, y in waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    def make_flat_terrain(self, radius: float = 5.0, step: float = 0.5) -> PointCloud2:
        """构造一个平坦无障碍的 terrain_map，格式: XYZI float32×4

        frame_id: odom (世界坐标系，与 terrain_analysis 期望一致)
        intensity: 0.0 = 可通行地面; 非零 = 障碍物
        z=0 = 地面高度 (terrain_analysis 以此判断 obstacle_height_thre)
        """
        import struct
        points = []
        x = -radius
        while x <= radius:
            y = -radius
            while y <= radius:
                # intensity=0 → 地面可通行，z=0 → 无高度障碍
                points.append(struct.pack('ffff', x, y, 0.0, 0.0))
                y += step
            x += step

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.height   = 1
        msg.width    = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step   = 16  # 4 * float32
        msg.row_step     = msg.point_step * msg.width
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = b''.join(points)
        return msg


# ─────────────────────────────────────────────────────────────────────────────
# 测试套件
# ─────────────────────────────────────────────────────────────────────────────
class PlanningTestSuite:
    PASS = f"{C.GREEN}✓ PASS{C.NC}"
    FAIL = f"{C.RED}✗ FAIL{C.NC}"
    WARN = f"{C.YELLOW}⚠ WARN{C.NC}"

    def __init__(self, node: PlanningTestNode):
        self.node   = node
        self.passed = 0
        self.failed = 0
        self.warned = 0

    def _ok(self, name):
        print(f"  {self.PASS}  {name}")
        self.passed += 1

    def _fail(self, name, reason=''):
        print(f"  {self.FAIL}  {name}" + (f"  ({reason})" if reason else ''))
        self.failed += 1

    def _warn(self, name, reason=''):
        print(f"  {self.WARN}  {name}" + (f"  ({reason})" if reason else ''))
        self.warned += 1

    def _spin(self, sec: float):
        deadline = time.time() + sec
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ── Stage 1: 节点 & 话题存活 ───────────────────────────────────────────
    def stage1_node_topic_liveness(self):
        print(f"\n{C.BLUE}Stage 1: 节点 & 话题存活检查{C.NC}")

        # 节点检查
        node_names = [n for n, _ in self.node.get_node_names_and_namespaces()]

        expected_nodes = ['terrainAnalysis', 'terrainAnalysisExt',
                          'localPlanner',    'pathFollower']
        for n in expected_nodes:
            if n in node_names:
                self._ok(f"节点 /{n}")
            else:
                self._fail(f"节点 /{n}", "未找到 (系统未启动?)")

        # pct_path_adapter 可选 (stub planner 模式无此节点)
        if 'pct_path_adapter' in node_names:
            self._ok("节点 /pct_path_adapter")
        else:
            self._warn("节点 /pct_path_adapter", "未运行 (stub planner 模式?)")

        # 话题检查
        self._spin(2.0)  # 等待话题注册
        all_topics = [t for t, _ in self.node.get_topic_names_and_types()]

        required_topics = [
            '/nav/terrain_map', '/nav/terrain_map_ext',
            '/nav/local_path',  '/nav/cmd_vel',
            '/nav/way_point',   '/nav/planner_status',
            '/nav/odometry',    '/nav/map_cloud',
        ]
        for t in required_topics:
            if t in all_topics:
                self._ok(f"话题 {t}")
            else:
                self._fail(f"话题 {t}", "不存在")

    # ── Stage 2: 话题命名规范 ─────────────────────────────────────────────
    def stage2_topic_naming(self):
        print(f"\n{C.BLUE}Stage 2: 话题命名规范 (/nav/* 前缀){C.NC}")
        all_topics = [t for t, _ in self.node.get_topic_names_and_types()]

        # 旧话题名不应出现 (表示 remap 失效)
        legacy_topics = [
            '/cmd_vel', '/path', '/way_point', '/terrain_map',
            '/terrain_map_ext', '/Odometry',
        ]
        bad = [t for t in legacy_topics if t in all_topics]
        if not bad:
            self._ok("无旧版裸名话题 (remap 正常)")
        else:
            for t in bad:
                self._fail(f"旧版话题 {t}", "remap 可能失效")

        # /nav/* 话题应存在
        nav_topics = [t for t in all_topics if t.startswith('/nav/')]
        if len(nav_topics) >= 6:
            self._ok(f"存在 {len(nav_topics)} 个 /nav/* 话题")
        else:
            self._warn(f"仅有 {len(nav_topics)} 个 /nav/* 话题", "期望 ≥ 6")

    # ── Stage 3: PCT Adapter 注入全局路径 ─────────────────────────────────
    def stage3_pct_adapter_path_injection(self):
        print(f"\n{C.BLUE}Stage 3: PCT Adapter — 注入全局路径{C.NC}")

        # 检查节点是否运行
        node_names = [n for n, _ in self.node.get_node_names_and_namespaces()]
        if 'pct_path_adapter' not in node_names:
            self._warn("跳过 Stage 3", "pct_path_adapter 未运行 (stub 模式?)")
            return

        self.node.clear_received('/nav/way_point')
        self.node.clear_received('/nav/planner_status')

        # 发布一条简单路径: (0,0) → (1,0) → (2,0) → (3,0) → (4,0)
        # 航点间距 1m，pct_adapter waypoint_distance=0.5 → 全部保留
        path = self.node.make_global_path(
            [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
        )
        # 先发布几帧里程计确保 odom_frame_ 初始化
        self._spin(0.5)
        self.node.pub_path.publish(path)
        print("    已发布 5 点全局路径 (map frame)")

        # 期待 planner_status: path_received
        if self.node.wait_for_status_event('path_received', timeout=5.0):
            self._ok("收到 planner_status {event: path_received}")
        else:
            self._fail("planner_status path_received", "超时 5s 未收到")

        # 期待 way_point
        if self.node.wait_for('/nav/way_point', count=1, timeout=5.0):
            with self.node._lock:
                wp = self.node.received['/nav/way_point'][-1]
            print(f"    way_point: ({wp.point.x:.2f}, {wp.point.y:.2f})")
            self._ok("收到 /nav/way_point")
        else:
            self._fail("/nav/way_point", "超时 5s 未收到")

    # ── Stage 4: PCT Adapter 航点推进 ─────────────────────────────────────
    def stage4_pct_adapter_waypoint_progress(self):
        print(f"\n{C.BLUE}Stage 4: PCT Adapter — 航点推进{C.NC}")

        node_names = [n for n, _ in self.node.get_node_names_and_namespaces()]
        if 'pct_path_adapter' not in node_names:
            self._warn("跳过 Stage 4", "pct_path_adapter 未运行")
            return

        # 先重发路径确保状态初始化
        self.node.clear_received('/nav/planner_status')
        path = self.node.make_global_path(
            [(0, 0), (2, 0), (4, 0), (6, 0), (8, 0)]
        )
        self.node.pub_path.publish(path)
        self.node.wait_for_status_event('path_received', timeout=3.0)

        # 把机器人移到第一个航点附近 (arrival_threshold=0.5m)
        # 注意: robot_x/y 由 _publish_odom 以 10Hz 持续发布给 pct_adapter
        # _spin(2s) 内会有 ~20 帧里程计送达，足够 adapter 检测到位置变化
        self.node.robot_x = 0.1
        self.node.robot_y = 0.0
        print("    机器人位置 → (0.1, 0.0)，期望到达第一航点")
        self._spin(2.0)

        # 移到第二个航点附近触发推进 (map→odom TF 由 static_transform_publisher 提供)
        self.node.robot_x = 2.1
        self.node.robot_y = 0.0
        print("    机器人位置 → (2.1, 0.0)，期望 waypoint_reached")
        self._spin(2.0)

        if self.node.wait_for_status_event('waypoint_reached', timeout=5.0):
            self._ok("收到 planner_status {event: waypoint_reached}")
        else:
            self._warn("planner_status waypoint_reached", "未收到 (TF map→odom 可能缺失)")

    # ── Stage 5: PCT Adapter goal_reached ─────────────────────────────────
    def stage5_pct_adapter_goal_reached(self):
        print(f"\n{C.BLUE}Stage 5: PCT Adapter — goal_reached + 停发航点{C.NC}")

        node_names = [n for n, _ in self.node.get_node_names_and_namespaces()]
        if 'pct_path_adapter' not in node_names:
            self._warn("跳过 Stage 5", "pct_path_adapter 未运行")
            return

        self.node.clear_received('/nav/planner_status')
        self.node.clear_received('/nav/way_point')

        # 发布短路径 (只有 2 点)
        path = self.node.make_global_path([(0, 0), (0.3, 0)])
        self.node.pub_path.publish(path)
        self.node.wait_for_status_event('path_received', timeout=3.0)

        # 机器人直接靠近终点
        self.node.robot_x = 0.2
        self.node.robot_y = 0.0
        print("    机器人位置 → (0.2, 0.0)，期望 goal_reached")

        if self.node.wait_for_status_event('goal_reached', timeout=6.0):
            self._ok("收到 planner_status {event: goal_reached}")
        else:
            self._fail("planner_status goal_reached", "超时 6s 未收到")

        # goal_reached 后应停止发布 way_point
        count_before = len(self.node.received['/nav/way_point'])
        self._spin(2.0)
        count_after = len(self.node.received['/nav/way_point'])
        if count_after == count_before:
            self._ok("goal_reached 后停发 /nav/way_point")
        else:
            self._warn("goal_reached 后仍在发 way_point", f"+{count_after-count_before} 条")

    # ── Stage 6: 局部规划链 (可选) ─────────────────────────────────────────
    def stage6_local_planning_chain(self):
        print(f"\n{C.BLUE}Stage 6: 局部规划链 way_point → local_path → cmd_vel (可选){C.NC}")

        node_names = [n for n, _ in self.node.get_node_names_and_namespaces()]
        if 'localPlanner' not in node_names or 'pathFollower' not in node_names:
            self._warn("跳过 Stage 6", "localPlanner / pathFollower 未运行")
            return

        self.node.clear_received('/nav/local_path')
        self.node.clear_received('/nav/cmd_vel')

        # 发布平坦地形 + 机器人位置 + 目标航点
        self.node.robot_x = 0.0
        self.node.robot_y = 0.0
        terrain = self.node.make_flat_terrain(radius=5.0, step=0.3)

        # 持续发布 3s，让 local_planner 积累地形
        print("    持续发布平坦地形 + 里程计 (3s)...")
        deadline = time.time() + 3.0
        while time.time() < deadline:
            self.node.pub_terrain.publish(terrain)
            self.node.pub_te_ext.publish(terrain)
            self.node.pub_map_cld.publish(terrain)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # 发布目标航点 (5m 前方)
        wp = PointStamped()
        wp.header.stamp = self.node.get_clock().now().to_msg()
        wp.header.frame_id = 'odom'
        wp.point.x = 5.0
        wp.point.y = 0.0
        wp.point.z = 0.0
        self.node.pub_way.publish(wp)
        print("    已发布目标航点 (5m 前方)")

        # 等待 local_path
        if self.node.wait_for('/nav/local_path', count=1, timeout=5.0):
            self._ok("收到 /nav/local_path")
        else:
            self._warn("/nav/local_path", "超时 5s (地形数据可能不足)")

        # 等待 cmd_vel
        if self.node.wait_for('/nav/cmd_vel', count=1, timeout=5.0):
            with self.node._lock:
                cv = self.node.received['/nav/cmd_vel'][-1]
            vx = cv.twist.linear.x
            wz = cv.twist.angular.z
            print(f"    cmd_vel: linear.x={vx:.3f}  angular.z={wz:.3f}")
            self._ok("收到 /nav/cmd_vel")
        else:
            self._warn("/nav/cmd_vel", "超时 5s")

    # ── 汇总 ──────────────────────────────────────────────────────────────
    def summary(self):
        total = self.passed + self.failed + self.warned
        print(f"\n{'='*60}")
        print(f"{C.BOLD}规划流水线测试汇总{C.NC}")
        print(f"{'='*60}")
        print(f"  {C.GREEN}通过{C.NC}: {self.passed}")
        print(f"  {C.RED}失败{C.NC}: {self.failed}")
        print(f"  {C.YELLOW}警告{C.NC}: {self.warned}")
        print(f"  总计  : {total}")
        print(f"{'='*60}")
        if self.failed == 0:
            print(f"{C.GREEN}✅ 规划流水线测试通过{C.NC}")
        else:
            print(f"{C.RED}❌ 有 {self.failed} 项失败{C.NC}")
            print("  提示: 确认系统已启动 (./tests/integration/test_planning_stub.sh)")
        print()
        return self.failed == 0


# ─────────────────────────────────────────────────────────────────────────────
def main():
    print('=' * 60)
    print(f"{C.BOLD}  规划流水线集成测试{C.NC}")
    print('=' * 60)

    try:
        rclpy.init()
    except Exception as e:
        print(f"{C.RED}❌ ROS2 初始化失败: {e}{C.NC}")
        print("  请先: source /opt/ros/humble/setup.bash && source install/setup.bash")
        sys.exit(1)

    node  = PlanningTestNode()
    suite = PlanningTestSuite(node)

    # 等待系统稳定
    print("\n等待系统稳定 (3s)...")
    deadline = time.time() + 3.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    try:
        suite.stage1_node_topic_liveness()
        suite.stage2_topic_naming()
        suite.stage3_pct_adapter_path_injection()
        suite.stage4_pct_adapter_waypoint_progress()
        suite.stage5_pct_adapter_goal_reached()
        suite.stage6_local_planning_chain()
    finally:
        ok = suite.summary()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
