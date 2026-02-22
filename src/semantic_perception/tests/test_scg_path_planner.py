#!/usr/bin/env python3
"""
SCG-based 路径规划器测试脚本

测试内容:
1. 基本路径规划
2. 多面体定位
3. A* 搜索
4. 路径平滑
5. 路径简化
6. 与混合规划器对比
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from semantic_perception.scg_path_planner import SCGPathPlanner, PathSmoother
from semantic_perception.scg_builder import SCGBuilder, SCGConfig
from semantic_perception.polyhedron_expansion import (
    PolyhedronExpander,
    PolyhedronExpansionConfig,
)


def create_test_scg():
    """创建测试用的 SCG。"""
    # 创建占据栅格
    occupancy_grid = np.ones((30, 30, 10), dtype=np.float32)
    occupancy_grid[5:25, :, :] = 0.0
    occupancy_grid[12:18, 12:18, :] = 1.0

    grid_resolution = 0.5
    grid_origin = np.array([0.0, 0.0, 0.0])

    # 多面体扩展
    config = PolyhedronExpansionConfig(
        num_sphere_samples=32,
        r_min=0.3,
        r_max=1.5,
        r_step=0.3,
        min_polyhedron_volume=0.1,
        max_polyhedra=15,
        coverage_threshold=0.5,
        collision_threshold=0.5,
    )

    expander = PolyhedronExpander(config)
    polyhedra = expander.expand(occupancy_grid, grid_resolution, grid_origin)

    print(f"生成的多面体数量: {len(polyhedra)}")

    # 构建 SCG
    scg_config = SCGConfig(
        adjacency_threshold=0.2,
        connectivity_samples=20,
        loop_closure_threshold=0.5,
    )

    scg_builder = SCGBuilder(scg_config)

    for poly in polyhedra:
        scg_builder.add_polyhedron(poly)

    scg_builder.build_edges(occupancy_grid, grid_resolution, grid_origin)

    print(f"SCG 节点数: {len(scg_builder.polyhedra)}")
    print(f"SCG 边数: {len(scg_builder.edges)}")

    return scg_builder, polyhedra


def test_polyhedron_location():
    """测试多面体定位。"""
    print("=" * 60)
    print("测试 1: 多面体定位")
    print("=" * 60)

    scg_builder, polyhedra = create_test_scg()
    planner = SCGPathPlanner(scg_builder)

    # 测试点定位
    test_points = [
        polyhedra[0].center,  # 应该在多面体 0 内
        polyhedra[0].center + np.array([10.0, 10.0, 0.0]),  # 应该不在任何多面体内
    ]

    for i, point in enumerate(test_points):
        poly_id = planner._locate_polyhedron(point)
        print(f"点 {i}: {point} -> 多面体 {poly_id}")

    print("✓ 多面体定位测试完成")
    print()


def test_path_planning():
    """测试基本路径规划。"""
    print("=" * 60)
    print("测试 2: 基本路径规划")
    print("=" * 60)

    scg_builder, polyhedra = create_test_scg()
    planner = SCGPathPlanner(scg_builder)

    if len(polyhedra) < 2:
        print("⚠️ 多面体数量不足，跳过测试")
        return

    # 选择起点和终点
    start = polyhedra[0].center
    goal = polyhedra[-1].center

    print(f"起点: {start}")
    print(f"终点: {goal}")

    # 规划路径
    result = planner.plan(start, goal, smooth=True, simplify=True)

    print(f"\n规划结果:")
    print(f"  成功: {result.success}")
    print(f"  多面体序列: {result.polyhedron_sequence}")
    print(f"  路径段数: {len(result.segments)}")
    print(f"  总距离: {result.total_distance:.2f}m")
    print(f"  规划时间: {result.planning_time*1000:.2f}ms")

    if result.success:
        print(f"\n路径段详情:")
        for i, segment in enumerate(result.segments):
            print(f"  段 {i}: 多面体 {segment.from_poly_id} -> {segment.to_poly_id}")
            print(f"    路径点数: {len(segment.waypoints)}")
            print(f"    距离: {segment.distance:.2f}m")
            print(f"    边类型: {segment.edge_type}")

    assert result.success, "路径规划应该成功"

    print("\n✓ 基本路径规划测试通过")
    print()


def test_path_smoothing():
    """测试路径平滑。"""
    print("=" * 60)
    print("测试 3: 路径平滑")
    print("=" * 60)

    smoother = PathSmoother(iterations=10, alpha=0.5)

    # 创建一个锯齿状路径
    path = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [2.0, 0.0, 0.0],
        [3.0, 1.0, 0.0],
        [4.0, 0.0, 0.0],
    ])

    print(f"原始路径点数: {len(path)}")

    # 平滑
    smoothed = smoother.smooth(path)

    print(f"平滑后路径点数: {len(smoothed)}")
    print(f"平滑前后对比:")
    for i in range(len(path)):
        print(f"  点 {i}: {path[i]} -> {smoothed[i]}")

    # 检查端点不变
    assert np.allclose(smoothed[0], path[0]), "起点应该不变"
    assert np.allclose(smoothed[-1], path[-1]), "终点应该不变"

    print("\n✓ 路径平滑测试通过")
    print()


def test_path_simplification():
    """测试路径简化。"""
    print("=" * 60)
    print("测试 4: 路径简化")
    print("=" * 60)

    smoother = PathSmoother()

    # 创建一个有冗余点的路径
    path = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [3.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
        [5.0, 1.0, 0.0],
        [6.0, 2.0, 0.0],
    ])

    print(f"原始路径点数: {len(path)}")

    # 简化
    simplified = smoother.simplify(path, tolerance=0.1)

    print(f"简化后路径点数: {len(simplified)}")
    print(f"简化后路径:")
    for i, point in enumerate(simplified):
        print(f"  点 {i}: {point}")

    assert len(simplified) < len(path), "简化后点数应该减少"

    print("\n✓ 路径简化测试通过")
    print()


def test_comparison_with_hybrid_planner():
    """测试与混合规划器的对比。"""
    print("=" * 60)
    print("测试 5: 与混合规划器对比")
    print("=" * 60)

    scg_builder, polyhedra = create_test_scg()

    if len(polyhedra) < 2:
        print("⚠️ 多面体数量不足，跳过测试")
        return

    # SCG 规划器
    scg_planner = SCGPathPlanner(scg_builder)

    # 选择起点和终点
    start = polyhedra[0].center
    goal = polyhedra[-1].center

    # SCG 规划
    scg_result = scg_planner.plan(start, goal)

    print(f"SCG 规划器:")
    print(f"  成功: {scg_result.success}")
    print(f"  多面体序列长度: {len(scg_result.polyhedron_sequence)}")
    print(f"  总距离: {scg_result.total_distance:.2f}m")
    print(f"  规划时间: {scg_result.planning_time*1000:.2f}ms")

    # 注意: 混合规划器需要 Tomogram，这里只是展示 SCG 规划器的结果
    # 实际对比需要在有 Tomogram 的环境中进行

    print("\n✓ 对比测试完成")
    print()


def test_edge_cases():
    """测试边界情况。"""
    print("=" * 60)
    print("测试 6: 边界情况")
    print("=" * 60)

    scg_builder, polyhedra = create_test_scg()
    planner = SCGPathPlanner(scg_builder)

    # 测试 1: 起点和终点相同
    if len(polyhedra) > 0:
        start = polyhedra[0].center
        goal = polyhedra[0].center

        result = planner.plan(start, goal)
        print(f"相同起点终点: 成功={result.success}, 序列={result.polyhedron_sequence}")

    # 测试 2: 起点不在任何多面体内
    start = np.array([100.0, 100.0, 0.0])
    goal = polyhedra[0].center if len(polyhedra) > 0 else np.array([0.0, 0.0, 0.0])

    result = planner.plan(start, goal)
    print(f"起点不在多面体内: 成功={result.success}")

    # 测试 3: 终点不在任何多面体内
    start = polyhedra[0].center if len(polyhedra) > 0 else np.array([0.0, 0.0, 0.0])
    goal = np.array([100.0, 100.0, 0.0])

    result = planner.plan(start, goal)
    print(f"终点不在多面体内: 成功={result.success}")

    print("\n✓ 边界情况测试完成")
    print()


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("SCG-based 路径规划器测试套件")
    print("=" * 60 + "\n")

    try:
        test_polyhedron_location()
        test_path_planning()
        test_path_smoothing()
        test_path_simplification()
        test_comparison_with_hybrid_planner()
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
