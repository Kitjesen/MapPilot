#!/usr/bin/env python3
"""
全局覆盖掩码 (GCM) 测试脚本

测试内容:
1. 基本功能测试
2. 从多面体更新
3. 从局部栅格更新
4. 前沿检测
5. 覆盖率计算
6. 序列化/反序列化
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from semantic_perception.global_coverage_mask import GlobalCoverageMask, CoverageCell
from semantic_perception.polyhedron_expansion import (
    Polyhedron,
    PolyhedronExpander,
    PolyhedronExpansionConfig,
)


def test_basic_functionality():
    """测试基本功能。"""
    print("=" * 60)
    print("测试 1: 基本功能")
    print("=" * 60)

    gcm = GlobalCoverageMask(resolution=0.5)

    # 手动标记一些单元格
    gcm._mark_cell_covered(0, 0)
    gcm._mark_cell_covered(1, 0)
    gcm._mark_cell_covered(0, 1)

    print(f"总单元格数: {gcm.total_cells}")
    print(f"已覆盖单元格数: {gcm.covered_cells}")
    print(f"覆盖率: {gcm.get_coverage_ratio():.2%}")

    assert gcm.total_cells == 3
    assert gcm.covered_cells == 3
    assert gcm.get_coverage_ratio() == 1.0

    print("✓ 基本功能正确")
    print()


def test_update_from_polyhedron():
    """测试从多面体更新。"""
    print("=" * 60)
    print("测试 2: 从多面体更新")
    print("=" * 60)

    gcm = GlobalCoverageMask(resolution=0.5)

    # 创建一个简单的多面体（立方体）
    vertices = np.array([
        [0, 0, 0], [2, 0, 0], [0, 2, 0], [0, 0, 2],
        [2, 2, 0], [2, 0, 2], [0, 2, 2], [2, 2, 2],
    ], dtype=np.float64)

    polyhedron = Polyhedron(
        poly_id=0,
        vertices=vertices,
        faces=[],
        center=np.array([1.0, 1.0, 1.0]),
        volume=8.0,
        radius=np.sqrt(3),
        seed_point=np.array([1.0, 1.0, 1.0]),
        sample_points=vertices,
    )

    # 更新 GCM
    gcm.update_from_polyhedron(polyhedron)

    print(f"总单元格数: {gcm.total_cells}")
    print(f"已覆盖单元格数: {gcm.covered_cells}")
    print(f"覆盖率: {gcm.get_coverage_ratio():.2%}")

    assert gcm.covered_cells > 0

    print("✓ 从多面体更新正确")
    print()


def test_update_from_local_grid():
    """测试从局部栅格更新。"""
    print("=" * 60)
    print("测试 3: 从局部栅格更新")
    print("=" * 60)

    gcm = GlobalCoverageMask(resolution=0.5)

    # 创建一个简单的占据栅格
    occupancy_grid = np.zeros((10, 10, 5), dtype=np.float32)
    occupancy_grid[2:8, 2:8, :] = 0.0  # 中间区域自由

    grid_resolution = 0.2
    grid_origin = np.array([0.0, 0.0, 0.0])

    # 更新 GCM
    gcm.update_from_local_grid(occupancy_grid, grid_resolution, grid_origin)

    print(f"总单元格数: {gcm.total_cells}")
    print(f"已覆盖单元格数: {gcm.covered_cells}")
    print(f"覆盖率: {gcm.get_coverage_ratio():.2%}")

    assert gcm.covered_cells > 0

    print("✓ 从局部栅格更新正确")
    print()


def test_frontier_detection():
    """测试前沿检测。"""
    print("=" * 60)
    print("测试 4: 前沿检测")
    print("=" * 60)

    gcm = GlobalCoverageMask(resolution=0.5)

    # 创建一个已探索区域（5×5 正方形）
    for x in range(5):
        for y in range(5):
            gcm._mark_cell_covered(x, y)

    # 获取前沿
    frontiers = gcm.get_frontier_cells()

    print(f"前沿单元格数: {len(frontiers)}")
    print(f"前沿单元格: {frontiers[:10]}")  # 显示前 10 个

    # 边界上的单元格应该是前沿
    assert len(frontiers) > 0

    # 测试前沿聚类
    clusters = gcm.get_frontier_clusters(min_cluster_size=2)
    print(f"前沿聚类数: {len(clusters)}")

    print("✓ 前沿检测正确")
    print()


def test_coverage_ratio():
    """测试覆盖率计算。"""
    print("=" * 60)
    print("测试 5: 覆盖率计算")
    print("=" * 60)

    gcm = GlobalCoverageMask(resolution=0.5)

    # 创建一个 10×10 的区域
    for x in range(10):
        for y in range(10):
            gcm._mark_cell_covered(x, y)

    # 覆盖率应该是 100%
    coverage = gcm.get_coverage_ratio()
    print(f"覆盖率: {coverage:.2%}")

    assert coverage == 1.0

    print("✓ 覆盖率计算正确")
    print()


def test_serialization():
    """测试序列化/反序列化。"""
    print("=" * 60)
    print("测试 6: 序列化/反序列化")
    print("=" * 60)

    gcm1 = GlobalCoverageMask(resolution=0.5)

    # 添加一些数据
    for x in range(5):
        for y in range(5):
            gcm1._mark_cell_covered(x, y)

    # 保存
    filepath = "/tmp/test_gcm.pkl"
    gcm1.save(filepath)

    # 加载
    gcm2 = GlobalCoverageMask()
    gcm2.load(filepath)

    # 验证
    assert gcm2.resolution == gcm1.resolution
    assert gcm2.total_cells == gcm1.total_cells
    assert gcm2.covered_cells == gcm1.covered_cells

    print(f"原始 GCM: {gcm1.total_cells} 单元格")
    print(f"加载 GCM: {gcm2.total_cells} 单元格")

    print("✓ 序列化/反序列化正确")
    print()


def test_integration_with_polyhedron_expansion():
    """测试与多面体扩展的集成。"""
    print("=" * 60)
    print("测试 7: 与多面体扩展集成")
    print("=" * 60)

    gcm = GlobalCoverageMask(resolution=0.5)

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
        max_polyhedra=10,
        coverage_threshold=0.5,
        collision_threshold=0.5,
    )

    expander = PolyhedronExpander(config)
    polyhedra = expander.expand(occupancy_grid, grid_resolution, grid_origin)

    print(f"生成的多面体数量: {len(polyhedra)}")

    # 从每个多面体更新 GCM
    for poly in polyhedra:
        gcm.update_from_polyhedron(poly)

    # 统计信息
    stats = gcm.get_statistics()
    print(f"\nGCM 统计信息:")
    print(f"  分辨率: {stats['resolution']}m")
    print(f"  总单元格数: {stats['total_cells']}")
    print(f"  已覆盖单元格数: {stats['covered_cells']}")
    print(f"  覆盖率: {stats['coverage_ratio']:.2%}")
    print(f"  前沿数量: {stats['num_frontiers']}")

    assert stats['covered_cells'] > 0
    assert stats['coverage_ratio'] > 0

    print("\n✓ 与多面体扩展集成成功")
    print()


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("全局覆盖掩码 (GCM) 测试套件")
    print("=" * 60 + "\n")

    try:
        test_basic_functionality()
        test_update_from_polyhedron()
        test_update_from_local_grid()
        test_frontier_detection()
        test_coverage_ratio()
        test_serialization()
        test_integration_with_polyhedron_expansion()

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
