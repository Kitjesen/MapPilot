#!/usr/bin/env python3
"""
多面体扩展算法测试脚本

测试内容:
1. 球面采样
2. 凸包计算
3. 碰撞检测
4. 完整的多面体扩展流程
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from semantic_perception.polyhedron_expansion import (
    CollisionChecker,
    ConvexHullComputer,
    PolyhedronExpander,
    PolyhedronExpansionConfig,
    SphereSampler,
)


def test_sphere_sampling():
    """测试球面采样。"""
    print("=" * 60)
    print("测试 1: 球面采样")
    print("=" * 60)

    sampler = SphereSampler()

    # Fibonacci 采样
    directions = sampler.fibonacci_sphere(num_samples=48)

    print(f"采样点数: {len(directions)}")
    print(f"采样点形状: {directions.shape}")

    # 检查单位向量
    norms = np.linalg.norm(directions, axis=1)
    print(f"向量模长范围: [{norms.min():.4f}, {norms.max():.4f}]")

    assert np.allclose(norms, 1.0, atol=1e-6), "应该是单位向量"

    print(f"✓ 球面采样正确")
    print()


def test_convex_hull():
    """测试凸包计算。"""
    print("=" * 60)
    print("测试 2: 凸包计算")
    print("=" * 60)

    computer = ConvexHullComputer()

    # 创建测试点集 (立方体)
    points = np.array([
        [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1],
        [1, 1, 0], [1, 0, 1], [0, 1, 1], [1, 1, 1],
    ], dtype=np.float64)

    result = computer.compute(points)

    assert result is not None, "凸包计算应该成功"

    print(f"凸包顶点数: {len(result['vertices'])}")
    print(f"凸包面片数: {len(result['faces'])}")
    print(f"凸包体积: {result['volume']:.4f}")
    print(f"凸包中心: {result['center']}")
    print(f"外接球半径: {result['radius']:.4f}")

    # 立方体体积应该是 1
    assert np.isclose(result['volume'], 1.0, atol=0.1), "立方体体积应该接近 1"

    print(f"✓ 凸包计算正确")
    print()


def test_collision_detection():
    """测试碰撞检测。"""
    print("=" * 60)
    print("测试 3: 碰撞检测")
    print("=" * 60)

    checker = CollisionChecker()

    # 创建占据栅格 (10×10×10, 中心有障碍物)
    occupancy_grid = np.zeros((10, 10, 10), dtype=np.float32)
    occupancy_grid[4:6, 4:6, 4:6] = 1.0  # 中心障碍物

    grid_resolution = 0.5
    grid_origin = np.array([0.0, 0.0, 0.0])

    # 测试用例 1: 无碰撞多面体 (远离障碍物)
    vertices_free = np.array([
        [0.5, 0.5, 0.5], [1.5, 0.5, 0.5],
        [0.5, 1.5, 0.5], [0.5, 0.5, 1.5],
    ], dtype=np.float64)

    collision_free = checker.check_collision(
        vertices_free, occupancy_grid, grid_resolution, grid_origin
    )

    print(f"无碰撞多面体: collision={collision_free}")
    assert not collision_free, "应该无碰撞"

    # 测试用例 2: 碰撞多面体 (与障碍物重叠)
    vertices_collision = np.array([
        [2.0, 2.0, 2.0], [3.0, 2.0, 2.0],
        [2.0, 3.0, 2.0], [2.0, 2.0, 3.0],
    ], dtype=np.float64)

    collision_detected = checker.check_collision(
        vertices_collision, occupancy_grid, grid_resolution, grid_origin
    )

    print(f"碰撞多面体: collision={collision_detected}")
    assert collision_detected, "应该检测到碰撞"

    print(f"✓ 碰撞检测正确")
    print()


def test_polyhedron_expansion():
    """测试完整的多面体扩展流程。"""
    print("=" * 60)
    print("测试 4: 多面体扩展")
    print("=" * 60)

    # 创建占据栅格 (20×20×10, 模拟走廊)
    occupancy_grid = np.ones((20, 20, 10), dtype=np.float32)

    # 走廊: 中间 10×20 区域自由
    occupancy_grid[5:15, :, :] = 0.0

    # 添加一些障碍物
    occupancy_grid[8:12, 8:12, :] = 1.0

    grid_resolution = 0.5
    grid_origin = np.array([0.0, 0.0, 0.0])

    # 配置
    config = PolyhedronExpansionConfig(
        num_sphere_samples=32,
        r_min=0.5,
        r_max=2.0,
        r_step=0.5,
        min_polyhedron_volume=0.5,
        max_polyhedra=10,
        coverage_threshold=0.7,
    )

    # 扩展
    expander = PolyhedronExpander(config)
    polyhedra = expander.expand(occupancy_grid, grid_resolution, grid_origin)

    print(f"\n生成的多面体数量: {len(polyhedra)}")

    for i, poly in enumerate(polyhedra):
        print(f"\n多面体 {i}:")
        print(f"  中心: {poly.center}")
        print(f"  体积: {poly.volume:.2f} m³")
        print(f"  半径: {poly.radius:.2f} m")
        print(f"  顶点数: {len(poly.vertices)}")
        print(f"  面片数: {len(poly.faces)}")

    assert len(polyhedra) > 0, "应该生成至少一个多面体"

    print(f"\n✓ 多面体扩展成功")
    print()


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("多面体扩展算法测试套件")
    print("=" * 60 + "\n")

    try:
        test_sphere_sampling()
        test_convex_hull()
        test_collision_detection()
        test_polyhedron_expansion()

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
