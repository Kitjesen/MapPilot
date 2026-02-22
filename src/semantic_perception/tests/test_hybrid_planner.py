#!/usr/bin/env python3
"""
混合路径规划器测试脚本

测试内容:
1. 混合规划器的基本功能
2. 拓扑层和几何层的分离
3. 性能对比 (vs 直线路径基线)
"""

import sys
from pathlib import Path

import numpy as np

# 添加模块路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from semantic_perception.geometry_extractor import GeometryExtractor
from semantic_perception.hybrid_planner import HybridPlanner, compare_planners
from semantic_perception.topology_graph import TopologySemGraph


def create_test_environment():
    """创建测试环境 (模拟 Tomogram + 拓扑图)。"""
    # 创建模拟 Tomogram
    class MockTomogram:
        def __init__(self):
            self.resolution = 0.2
            self.map_center = np.array([0.0, 0.0])
            self.map_dim_x = 200
            self.map_dim_y = 200
            # 创建可通行性地图 (大部分区域可通行)
            self.inflated_cost = np.ones((1, 200, 200), dtype=np.float32) * 0.1
            self.layers_g = np.zeros((1, 200, 200), dtype=np.float32)
            self.layers_c = np.ones((1, 200, 200), dtype=np.float32) * 2.5

    tomogram = MockTomogram()
    extractor = GeometryExtractor(tomogram)

    # 创建拓扑图
    tsg = TopologySemGraph()
    tsg.set_geometry_extractor(extractor)

    # 添加测试房间 (模拟走廊 + 办公室布局)
    scene_graph = {
        "rooms": [
            {
                "room_id": 1,
                "name": "corridor_1",
                "center": {"x": 0.0, "y": 0.0},
                "semantic_labels": ["door"],
            },
            {
                "room_id": 2,
                "name": "office_1",
                "center": {"x": 10.0, "y": 0.0},
                "semantic_labels": ["desk", "chair"],
            },
            {
                "room_id": 3,
                "name": "corridor_2",
                "center": {"x": 20.0, "y": 0.0},
                "semantic_labels": ["door"],
            },
            {
                "room_id": 4,
                "name": "office_2",
                "center": {"x": 20.0, "y": 10.0},
                "semantic_labels": ["desk", "computer"],
            },
        ],
        "topology_edges": [
            {"from_room": 1, "to_room": 2, "type": "door", "distance": 10.0},
            {"from_room": 2, "to_room": 3, "type": "door", "distance": 10.0},
            {"from_room": 3, "to_room": 4, "type": "door", "distance": 10.0},
        ],
    }

    tsg.update_from_scene_graph(scene_graph)

    return tsg, tomogram


def test_basic_planning():
    """测试混合规划器的基本功能。"""
    print("=" * 60)
    print("测试 1: 混合规划器基本功能")
    print("=" * 60)

    tsg, tomogram = create_test_environment()
    planner = HybridPlanner(tsg, tomogram)

    # 测试用例: 从房间 1 到房间 4
    start = np.array([0.0, 0.0, 0.0])
    goal = np.array([20.0, 10.0, 0.0])

    print(f"规划路径: {start[:2]} → {goal[:2]}")

    result = planner.plan_path(start, goal)

    print(f"\n结果:")
    print(f"  成功: {result.success}")
    print(f"  房间序列: {result.room_sequence}")
    print(f"  路径段数: {len(result.segments)}")
    print(f"  路径点数: {result.num_waypoints}")
    print(f"  总代价: {result.total_cost:.2f}")
    print(f"  总规划时间: {result.total_planning_time*1000:.2f}ms")
    print(f"    - 拓扑规划: {result.topology_planning_time*1000:.2f}ms")
    print(f"    - 几何规划: {result.geometry_planning_time*1000:.2f}ms")

    assert result.success, "规划应该成功"
    assert len(result.room_sequence) >= 2, "房间序列至少包含起点和终点"
    assert result.num_waypoints >= 2, "路径至少包含起点和终点"

    print(f"\n路径段详情:")
    for i, seg in enumerate(result.segments):
        print(
            f"  段 {i+1}: 房间 {seg.from_room_id} → {seg.to_room_id}, "
            f"{len(seg.waypoints)} 点, 代价={seg.cost:.2f}, "
            f"时间={seg.planning_time*1000:.2f}ms"
        )

    print(f"\n✓ 基本功能测试通过")
    print()


def test_topology_layer():
    """测试拓扑层规划。"""
    print("=" * 60)
    print("测试 2: 拓扑层规划")
    print("=" * 60)

    tsg, _ = create_test_environment()

    # 测试拓扑图的最短路径
    cost, path = tsg.shortest_path(1, 4)

    print(f"拓扑路径: {path}")
    print(f"拓扑代价: {cost:.2f}")

    assert path == [1, 2, 3, 4], "拓扑路径应该是 [1, 2, 3, 4]"
    assert cost == 30.0, "拓扑代价应该是 30.0"

    print(f"✓ 拓扑层规划正确")
    print()


def test_room_location():
    """测试房间定位功能。"""
    print("=" * 60)
    print("测试 3: 房间定位")
    print("=" * 60)

    tsg, tomogram = create_test_environment()
    planner = HybridPlanner(tsg, tomogram)

    test_positions = [
        (np.array([0.0, 0.0]), 1, "corridor_1"),
        (np.array([10.0, 0.0]), 2, "office_1"),
        (np.array([20.0, 0.0]), 3, "corridor_2"),
        (np.array([20.0, 10.0]), 4, "office_2"),
    ]

    for pos, expected_id, expected_name in test_positions:
        room_id = planner._locate_room(pos)
        room = tsg.get_node(room_id) if room_id else None

        print(f"位置 {pos} → 房间 {room_id} ({room.name if room else 'None'})")

        assert room_id == expected_id, f"应该定位到房间 {expected_id}"
        assert room.name == expected_name, f"房间名应该是 {expected_name}"

    print(f"\n✓ 房间定位正确")
    print()


def test_performance_comparison():
    """测试性能对比。"""
    print("=" * 60)
    print("测试 4: 性能对比")
    print("=" * 60)

    tsg, tomogram = create_test_environment()
    hybrid_planner = HybridPlanner(tsg, tomogram)

    # 创建简单的基线规划器 (直线路径)
    class BaselinePlanner:
        def plan_path(self, start, goal):
            num_waypoints = max(3, int(np.linalg.norm(goal - start) / 0.5))
            waypoints = []
            for i in range(num_waypoints):
                t = i / (num_waypoints - 1)
                waypoint = start * (1 - t) + goal * t
                waypoints.append(waypoint)
            return waypoints

    baseline_planner = BaselinePlanner()

    # 测试用例
    test_cases = [
        (np.array([0.0, 0.0, 0.0]), np.array([20.0, 10.0, 0.0])),  # 长距离
        (np.array([0.0, 0.0, 0.0]), np.array([10.0, 0.0, 0.0])),   # 中距离
        (np.array([10.0, 0.0, 0.0]), np.array([20.0, 0.0, 0.0])),  # 相邻房间
    ]

    results = compare_planners(hybrid_planner, baseline_planner, test_cases)

    print(f"\n性能对比结果:")
    print(f"{'测试用例':<15} {'混合时间':<12} {'基线时间':<12} {'加速比':<10}")
    print("-" * 60)

    for i, result in enumerate(results):
        print(
            f"用例 {i+1:<10} "
            f"{result.hybrid_time*1000:>8.2f}ms  "
            f"{result.baseline_time*1000:>8.2f}ms  "
            f"{result.speedup:>6.2f}×"
        )

    avg_speedup = sum(r.speedup for r in results) / len(results)
    print(f"\n平均加速比: {avg_speedup:.2f}×")

    print(f"\n✓ 性能对比完成")
    print()


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("混合路径规划器测试套件")
    print("=" * 60 + "\n")

    try:
        test_basic_planning()
        test_topology_layer()
        test_room_location()
        test_performance_comparison()

        print("=" * 60)
        print("✓ 所有测试通过!")
        print("=" * 60)
        return 0

    except Exception as e:
        print(f"\n✗ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
