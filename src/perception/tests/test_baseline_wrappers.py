#!/usr/bin/env python3
"""
基线方法包装器测试脚本

测试内容:
1. PCT A* 规划器
2. USS-Nav 规划器
3. 规划器工厂
4. 性能对比
"""

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.baseline_wrappers import (
    PCTAStarPlanner,
    USSNavPlanner,
    create_planner,
)


def create_test_environment():
    """创建测试环境。"""
    # 创建简单的点云（一个房间）
    points = []

    # 地面
    for x in np.linspace(-5, 5, 50):
        for y in np.linspace(-5, 5, 50):
            points.append([x, y, 0.0])

    # 墙壁
    for x in np.linspace(-5, 5, 20):
        for z in np.linspace(0, 3, 10):
            points.append([x, -5, z])  # 南墙
            points.append([x, 5, z])   # 北墙

    for y in np.linspace(-5, 5, 20):
        for z in np.linspace(0, 3, 10):
            points.append([-5, y, z])  # 西墙
            points.append([5, y, z])   # 东墙

    return np.array(points)


def test_pct_astar_planner():
    """测试 PCT A* 规划器。"""
    print("=" * 60)
    print("测试 1: PCT A* 规划器")
    print("=" * 60)

    planner = PCTAStarPlanner()
    planner.initialize(resolution=0.1, size=(100, 100, 40))

    # 创建测试环境
    robot_pose = np.array([0, 0, 0, 0, 0, 0])
    point_cloud = create_test_environment()

    print(f"点云数量: {len(point_cloud)}")

    # 更新地图
    planner.update(robot_pose, point_cloud)

    # 规划路径
    start = np.array([0, 0, 0])
    goal = np.array([3, 3, 0])

    path = planner.plan(start, goal)

    if path is not None:
        print("\n路径规划成功:")
        print(f"  路径点数: {len(path)}")
        print(f"  起点: {path[0]}")
        print(f"  终点: {path[-1]}")
    else:
        print("\n⚠️ 路径规划失败")

    # 获取统计信息
    stats = planner.get_statistics()
    print("\n统计信息:")
    print(f"  更新次数: {stats['update_count']}")
    print(f"  规划次数: {stats['planning_count']}")
    print(f"  平均规划时间: {stats['avg_planning_time_ms']:.2f} ms")
    print(f"  栅格大小: {stats['grid_size']}")
    print(f"  内存占用: {stats['memory_mb']:.2f} MB")

    assert stats['update_count'] > 0
    assert stats['planning_count'] > 0

    print("\n✓ PCT A* 规划器测试通过")
    print()


def test_uss_nav_planner():
    """测试 USS-Nav 规划器。"""
    print("=" * 60)
    print("测试 2: USS-Nav 规划器")
    print("=" * 60)

    try:
        planner = USSNavPlanner()
        planner.initialize()

        # 创建测试环境
        robot_pose = np.array([0, 0, 0, 0, 0, 0])
        point_cloud = create_test_environment()

        print(f"点云数量: {len(point_cloud)}")

        # 更新地图
        planner.update(robot_pose, point_cloud)

        # 获取统计信息
        stats = planner.get_statistics()
        print("\n统计信息:")
        print(f"  更新次数: {stats['update_count']}")
        print(f"  多面体数: {stats['num_polyhedra']}")
        print(f"  边数: {stats['num_edges']}")

        assert stats['update_count'] > 0

        print("\n✓ USS-Nav 规划器测试通过")
        print()

    except ImportError as e:
        print(f"\n⚠️ USS-Nav 组件未安装，跳过测试: {e}")
        print()


def test_planner_factory():
    """测试规划器工厂。"""
    print("=" * 60)
    print("测试 3: 规划器工厂")
    print("=" * 60)

    # 创建 PCT A* 规划器
    planner1 = create_planner("pct_astar", resolution=0.1, size=(100, 100, 40))
    print(f"创建规划器: {planner1.name}")
    assert planner1.name == "PCT A*"
    assert planner1.initialized

    # 创建 USS-Nav 规划器
    try:
        planner2 = create_planner("uss_nav")
        print(f"创建规划器: {planner2.name}")
        assert planner2.name == "USS-Nav"
        assert planner2.initialized
    except ImportError:
        print("⚠️ USS-Nav 组件未安装，跳过")

    print("\n✓ 规划器工厂测试通过")
    print()


def test_performance_comparison():
    """测试性能对比。"""
    print("=" * 60)
    print("测试 4: 性能对比")
    print("=" * 60)

    # 创建测试环境
    robot_pose = np.array([0, 0, 0, 0, 0, 0])
    point_cloud = create_test_environment()

    # PCT A* 规划器
    planner1 = PCTAStarPlanner()
    planner1.initialize(resolution=0.1, size=(100, 100, 40))

    start_time = time.time()
    planner1.update(robot_pose, point_cloud)
    update_time1 = (time.time() - start_time) * 1000

    start = np.array([0, 0, 0])
    goal = np.array([3, 3, 0])

    start_time = time.time()
    path1 = planner1.plan(start, goal)
    planning_time1 = (time.time() - start_time) * 1000

    stats1 = planner1.get_statistics()

    print("PCT A*:")
    print(f"  更新时间: {update_time1:.2f} ms")
    print(f"  规划时间: {planning_time1:.2f} ms")
    print(f"  内存占用: {stats1['memory_mb']:.2f} MB")
    if path1 is not None:
        print(f"  路径点数: {len(path1)}")

    # USS-Nav 规划器（如果可用）
    try:
        planner2 = USSNavPlanner()
        planner2.initialize()

        start_time = time.time()
        planner2.update(robot_pose, point_cloud)
        update_time2 = (time.time() - start_time) * 1000

        stats2 = planner2.get_statistics()

        print("\nUSS-Nav:")
        print(f"  更新时间: {update_time2:.2f} ms")
        print(f"  多面体数: {stats2['num_polyhedra']}")
        print(f"  边数: {stats2['num_edges']}")

    except ImportError:
        print("\n⚠️ USS-Nav 组件未安装，跳过对比")

    print("\n✓ 性能对比测试通过")
    print()


def test_multiple_updates():
    """测试多次更新。"""
    print("=" * 60)
    print("测试 5: 多次更新")
    print("=" * 60)

    planner = PCTAStarPlanner()
    planner.initialize(resolution=0.1, size=(100, 100, 40))

    # 模拟机器人移动并更新地图
    for i in range(5):
        robot_pose = np.array([i * 0.5, 0, 0, 0, 0, 0])
        point_cloud = create_test_environment()

        planner.update(robot_pose, point_cloud)

    stats = planner.get_statistics()
    print(f"更新次数: {stats['update_count']}")

    assert stats['update_count'] == 5

    print("\n✓ 多次更新测试通过")
    print()


def test_edge_cases():
    """测试边界情况。"""
    print("=" * 60)
    print("测试 6: 边界情况")
    print("=" * 60)

    planner = PCTAStarPlanner()
    planner.initialize(resolution=0.1, size=(100, 100, 40))

    # 空点云
    robot_pose = np.array([0, 0, 0, 0, 0, 0])
    empty_cloud = np.array([]).reshape(0, 3)
    planner.update(robot_pose, empty_cloud)
    print("✓ 空点云更新成功")

    # 超出边界的目标
    start = np.array([0, 0, 0])
    goal = np.array([100, 100, 0])  # 超出栅格
    path = planner.plan(start, goal)
    print(f"✓ 超出边界目标: path={'None' if path is None else 'found'}")

    # 起点和终点相同
    start = np.array([0, 0, 0])
    goal = np.array([0, 0, 0])
    path = planner.plan(start, goal)
    print(f"✓ 起点=终点: path={'None' if path is None else 'found'}")

    print("\n✓ 边界情况测试通过")
    print()


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("基线方法包装器测试套件")
    print("=" * 60 + "\n")

    try:
        test_pct_astar_planner()
        test_uss_nav_planner()
        test_planner_factory()
        test_performance_comparison()
        test_multiple_updates()
        test_edge_cases()

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
