#!/usr/bin/env python3
"""
pct_path_adapter 纯 Python 单元测试 (无需 ROS2)

测试核心逻辑：
  - 路径下采样 (3D 距离)
  - 路径端点始终保留
  - 航点推进条件 (arrival_threshold)
  - goal_reached 停发逻辑
  - stuck 检测时序

用法:
    python3 tests/planning/test_pct_adapter_logic.py
    # 或:
    cd tests/planning && python3 -m pytest test_pct_adapter_logic.py -v
"""

import math
import sys
import time
import unittest
from unittest.mock import patch


# ──────────────────────────────────────────────────────────────────────────────
# 从 C++ 实现 pct_path_adapter.cpp 提取的纯 Python 等效逻辑
# ──────────────────────────────────────────────────────────────────────────────
def downsample_path(poses, waypoint_distance: float):
    """
    等效 PCTPathAdapter::downsample_path()

    Args:
        poses: list of (x, y, z) tuples
        waypoint_distance: 最小航点间距 (3D 距离)

    Returns:
        list of (x, y, z): 下采样后的航点列表
    """
    if not poses:
        return []

    downsampled = [poses[0]]
    last = poses[0]

    for pose in poses[1:]:
        dx = pose[0] - last[0]
        dy = pose[1] - last[1]
        dz = pose[2] - last[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist >= waypoint_distance:
            downsampled.append(pose)
            last = pose

    # 最后一个点始终包含
    # NOTE: C++ 实现无条件 push_back(path.poses.back())，可能在末点已被 loop 添加时产生重复。
    # 本 Python 版有意去重（行为更清晰），对 adapter 逻辑无影响（重复末点不影响 goal 判断）。
    if downsampled[-1] != poses[-1]:
        downsampled.append(poses[-1])

    return downsampled


def get_distance_2d(p1, p2):
    """等效 PCTPathAdapter::get_distance() - 使用 2D 距离判断到达"""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


class PCTAdapterSimulator:
    """
    pct_path_adapter 行为模拟器
    用于在不启动 ROS2 的情况下测试核心逻辑
    """

    def __init__(self, waypoint_distance=0.5, arrival_threshold=0.5,
                 stuck_timeout_sec=30.0):
        self.waypoint_distance  = waypoint_distance
        self.arrival_threshold  = arrival_threshold
        self.stuck_timeout_sec  = stuck_timeout_sec

        self.current_path       = []
        self.current_idx        = 0
        self.path_received      = False
        self.goal_reached       = False
        self.last_progress_time = None

        # 记录发布事件
        self.published_waypoints = []
        self.status_events       = []

    def receive_path(self, poses):
        """模拟收到全局路径"""
        if not poses:
            self.current_path   = []
            self.current_idx    = 0
            self.path_received  = False
            return

        self.current_path  = downsample_path(poses, self.waypoint_distance)
        self.current_idx   = 0
        self.path_received = True
        self.goal_reached  = False
        self.last_progress_time = time.time()
        self.status_events.append({
            'event': 'path_received',
            'index': 0,
            'total': len(self.current_path),
        })

    def control_loop(self, robot_pos):
        """
        模拟 100ms 控制循环一次执行
        robot_pos: (x, y) in odom frame
        """
        if not self.path_received or not self.current_path:
            return

        # Stuck 检测
        if self.last_progress_time and not self.goal_reached:
            elapsed = time.time() - self.last_progress_time
            if elapsed > self.stuck_timeout_sec:
                self.status_events.append({
                    'event': 'stuck',
                    'index': self.current_idx,
                    'total': len(self.current_path),
                })

        target = self.current_path[self.current_idx]
        dist = get_distance_2d(robot_pos, (target[0], target[1]))

        if dist < self.arrival_threshold:
            if self.current_idx < len(self.current_path) - 1:
                self.status_events.append({
                    'event': 'waypoint_reached',
                    'index': self.current_idx,
                    'total': len(self.current_path),
                })
                self.last_progress_time = time.time()
                self.current_idx += 1
                return
            else:
                if not self.goal_reached:
                    self.status_events.append({
                        'event': 'goal_reached',
                        'index': self.current_idx,
                        'total': len(self.current_path),
                    })
                    self.goal_reached = True
                return  # goal_reached 后不发布航点

        if self.goal_reached:
            return

        self.published_waypoints.append(target)

    def events_of_type(self, event_type):
        return [e for e in self.status_events if e['event'] == event_type]


# ──────────────────────────────────────────────────────────────────────────────
# 测试用例
# ──────────────────────────────────────────────────────────────────────────────
class TestDownsamplePath(unittest.TestCase):
    """测试路径下采样逻辑"""

    def test_empty_path(self):
        result = downsample_path([], waypoint_distance=0.5)
        self.assertEqual(result, [])

    def test_single_point(self):
        # Python 版去重：单点路径返回 [(1,0,0)]
        # C++ 版因无条件追加末点，实际返回 [(1,0,0),(1,0,0)]（重复，对 adapter 无影响）
        result = downsample_path([(1, 0, 0)], waypoint_distance=0.5)
        self.assertEqual(result, [(1, 0, 0)])

    def test_two_points_far_apart(self):
        """两点距离 > waypoint_distance → 全部保留"""
        result = downsample_path([(0, 0, 0), (1, 0, 0)], waypoint_distance=0.5)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0], (0, 0, 0))
        self.assertEqual(result[-1], (1, 0, 0))

    def test_two_points_too_close(self):
        """两点距离 < waypoint_distance → 仍保留首尾"""
        result = downsample_path([(0, 0, 0), (0.1, 0, 0)], waypoint_distance=0.5)
        self.assertEqual(len(result), 2)  # 首 + 尾

    def test_first_and_last_always_kept(self):
        """首尾点必须保留"""
        poses = [(i * 0.1, 0, 0) for i in range(50)]  # 5m, 间距 0.1m
        result = downsample_path(poses, waypoint_distance=0.5)
        self.assertEqual(result[0], poses[0])
        self.assertEqual(result[-1], poses[-1])

    def test_spacing_2d(self):
        """下采样后航点间距不小于 waypoint_distance"""
        poses = [(i * 0.1, 0, 0) for i in range(100)]
        wd = 0.5
        result = downsample_path(poses, waypoint_distance=wd)
        for i in range(len(result) - 2):  # 最后一点可能略近 (end forced)
            d = math.dist(result[i], result[i + 1])
            self.assertGreaterEqual(d, wd - 1e-9,
                f"点 {i} 到 {i+1} 距离 {d:.3f} < {wd}")

    def test_3d_distance_slope(self):
        """坡道路径 (z 变化): 3D 距离大于 2D 距离，下采样结果更稀疏"""
        # 水平路径 (z=0)
        flat   = [(i * 0.3, 0, 0.0) for i in range(20)]
        # 45度坡道路径 (x, y 相同, z 也在变化)
        sloped = [(i * 0.3, 0, i * 0.3) for i in range(20)]

        result_flat   = downsample_path(flat,   waypoint_distance=0.5)
        result_sloped = downsample_path(sloped, waypoint_distance=0.5)

        # 坡道 3D 距离 ≈ √2 倍水平距离，采样点应更少
        self.assertLessEqual(len(result_sloped), len(result_flat),
            "坡道路径下采样点数应 ≤ 水平路径 (3D 距离更长)")

    def test_dense_path_count(self):
        """10m 路径，间距 0.1m，waypoint_distance=1.0m → 约 10-11 个航点"""
        poses = [(i * 0.1, 0, 0) for i in range(101)]
        result = downsample_path(poses, waypoint_distance=1.0)
        self.assertGreaterEqual(len(result), 10)
        self.assertLessEqual(len(result), 13)


class TestPCTAdapterSimulator(unittest.TestCase):
    """测试 pct_adapter 行为模拟器"""

    def setUp(self):
        self.adapter = PCTAdapterSimulator(
            waypoint_distance=0.5,
            arrival_threshold=0.5,
            stuck_timeout_sec=30.0,
        )

    def test_path_received_event(self):
        """收到路径 → 发布 path_received 事件"""
        self.adapter.receive_path([(0, 0, 0), (2, 0, 0), (4, 0, 0)])
        events = self.adapter.events_of_type('path_received')
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['total'], len(self.adapter.current_path))

    def test_empty_path_clears_state(self):
        """空路径清除状态"""
        self.adapter.receive_path([(0, 0, 0), (2, 0, 0)])
        self.assertTrue(self.adapter.path_received)
        self.adapter.receive_path([])
        self.assertFalse(self.adapter.path_received)
        self.assertEqual(self.adapter.current_path, [])

    def test_waypoint_published_when_far(self):
        """机器人远离目标航点 → 持续发布航点"""
        self.adapter.receive_path([(0, 0, 0), (5, 0, 0)])
        # 第一次: target=(0,0,0)，robot=(0,0)，dist=0 < threshold → waypoint_reached，idx→1，return
        self.adapter.control_loop((0.0, 0.0))
        # 第二次: target=(5,0,0)，robot=(0,0)，dist=5 >= threshold → 进入发布分支
        self.adapter.control_loop((0.0, 0.0))
        self.assertGreater(len(self.adapter.published_waypoints), 0)

    def test_waypoint_reached_event(self):
        """机器人到达航点 → waypoint_reached 事件 + 推进索引"""
        self.adapter.receive_path(
            [(0, 0, 0), (2, 0, 0), (4, 0, 0), (6, 0, 0)]
        )
        # 模拟机器人到达第一个航点 (arrival_threshold=0.5)
        self.adapter.control_loop((0.1, 0.0))
        events = self.adapter.events_of_type('waypoint_reached')
        self.assertGreaterEqual(len(events), 1)
        self.assertEqual(events[0]['index'], 0)

    def test_goal_reached_event(self):
        """机器人经由正常 control_loop 推进到最后一个航点 → goal_reached 事件"""
        # 使用3航点路径，通过正常流程推进（不手动篡改 current_idx）
        self.adapter.receive_path([(0, 0, 0), (2, 0, 0), (4, 0, 0)])
        # 步骤 1: 到达 (0,0,0) → waypoint_reached, idx→1
        self.adapter.control_loop((0.1, 0.0))
        # 步骤 2: 到达 (2,0,0) → waypoint_reached, idx→2
        self.adapter.control_loop((2.1, 0.0))
        # 步骤 3: 到达 (4,0,0) → goal_reached
        self.adapter.control_loop((4.1, 0.0))
        events = self.adapter.events_of_type('goal_reached')
        self.assertEqual(len(events), 1)
        self.assertTrue(self.adapter.goal_reached)

    def test_no_waypoint_after_goal_reached(self):
        """goal_reached 后停止发布 way_point"""
        # 使用间距 >= waypoint_distance 的明确路径，通过正常流程推进
        self.adapter.receive_path([(0, 0, 0), (2, 0, 0), (4, 0, 0)])
        # 依次推进到终点
        self.adapter.control_loop((0.1, 0.0))   # 到达(0,0,0) → idx→1
        self.adapter.control_loop((2.1, 0.0))   # 到达(2,0,0) → idx→2
        self.adapter.control_loop((4.1, 0.0))   # 到达(4,0,0) → goal_reached
        self.assertTrue(self.adapter.goal_reached, "前置条件: goal_reached 应为 True")

        # goal_reached 后的调用不应再增加已发布航点数
        count_before = len(self.adapter.published_waypoints)
        for _ in range(5):
            self.adapter.control_loop((4.1, 0.0))
        count_after = len(self.adapter.published_waypoints)
        self.assertEqual(count_before, count_after,
            "goal_reached 后不应再发布 way_point")

    def test_stuck_detection(self):
        """机器人长时间无进展 → stuck 事件 (mock 时钟，避免 CI 偶发失败)"""
        adapter = PCTAdapterSimulator(
            waypoint_distance=0.5,
            arrival_threshold=0.5,
            stuck_timeout_sec=30.0,
        )
        base_t = 1000.0
        # receive_path 调用 time.time() 1次 → base_t
        # control_loop  调用 time.time() 1次 → base_t + 31s > stuck_timeout
        with patch('time.time', side_effect=[base_t, base_t + 31.0]):
            adapter.receive_path([(0, 0, 0), (10, 0, 0)])
            adapter.control_loop((5.0, 5.0))  # 机器人远离所有航点
        events = adapter.events_of_type('stuck')
        self.assertGreater(len(events), 0, "应触发 stuck 事件")

    def test_replanning_resets_state(self):
        """收到新路径 → 重置索引和 goal_reached"""
        self.adapter.receive_path([(0, 0, 0), (1, 0, 0)])
        self.adapter.goal_reached = True
        self.adapter.current_idx  = 1

        self.adapter.receive_path([(0, 0, 0), (5, 0, 0), (10, 0, 0)])
        self.assertFalse(self.adapter.goal_reached)
        self.assertEqual(self.adapter.current_idx, 0)

    def test_arrival_threshold_boundary(self):
        """恰好在 arrival_threshold 边界上: <threshold → 到达, ≥threshold → 未到达"""
        self.adapter.receive_path([(0, 0, 0), (5, 0, 0)])
        target = self.adapter.current_path[0]

        # 距离 = threshold - ε → 到达
        robot_close = (target[0] + self.adapter.arrival_threshold - 0.01, target[1])
        self.adapter.control_loop(robot_close)
        events_close = self.adapter.events_of_type('waypoint_reached')

        # 重置
        self.adapter2 = PCTAdapterSimulator()
        self.adapter2.receive_path([(0, 0, 0), (5, 0, 0)])
        # 距离 = threshold + ε → 未到达
        robot_far = (target[0] + self.adapter.arrival_threshold + 0.01, target[1])
        self.adapter2.control_loop(robot_far)
        events_far = self.adapter2.events_of_type('waypoint_reached')

        self.assertGreaterEqual(len(events_close), 1, "应到达 (< threshold)")
        self.assertEqual(len(events_far), 0, "不应到达 (>= threshold)")


class TestDownsampleEdgeCases(unittest.TestCase):
    """边界情况测试"""

    def test_all_same_point(self):
        """所有点相同 → 首尾各一个"""
        poses = [(1, 1, 1)] * 5
        result = downsample_path(poses, waypoint_distance=0.5)
        self.assertEqual(result[0], (1, 1, 1))
        self.assertEqual(result[-1], (1, 1, 1))

    def test_very_dense_path(self):
        """极密路径 (间距 0.01m)，waypoint_distance=1.0 → 大幅下采样"""
        poses = [(i * 0.01, 0, 0) for i in range(1000)]  # 10m
        result = downsample_path(poses, waypoint_distance=1.0)
        # 10m / 1.0m ≈ 10 个中间点 + 首尾
        self.assertLess(len(result), 20)

    def test_path_with_z_zigzag(self):
        """Z 方向来回振荡的路径：3D 距离比水平距离长得多"""
        poses = []
        for i in range(20):
            z = 1.0 if i % 2 == 0 else -1.0
            poses.append((i * 0.1, 0, z))  # xy 间距 0.1m, z 振荡 ±1m

        # 3D 距离 ≈ sqrt(0.1² + 2²) ≈ 2.0m per step
        # waypoint_distance=0.5 → 几乎每个点都超过阈值
        result = downsample_path(poses, waypoint_distance=0.5)
        # 应保留大多数点（因为 3D 距离很大）
        self.assertGreater(len(result), 10)


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print('=' * 60)
    print('  PCT Adapter 逻辑单元测试')
    print('=' * 60)
    loader = unittest.TestLoader()
    suite  = unittest.TestSuite()
    suite.addTests(loader.loadTestsFromTestCase(TestDownsamplePath))
    suite.addTests(loader.loadTestsFromTestCase(TestPCTAdapterSimulator))
    suite.addTests(loader.loadTestsFromTestCase(TestDownsampleEdgeCases))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
