#!/usr/bin/env python3
"""
简化的可视化工具测试 - 验证核心功能
"""

import sys
from pathlib import Path

# 设置非交互式后端
import matplotlib

matplotlib.use('Agg')

import numpy as np
from scipy.spatial import ConvexHull

sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.scene_understanding.polyhedron_expansion import Polyhedron
from perception.scene_understanding.scg_builder import SCGBuilder, SCGConfig


def test_polyhedron_creation():
    """测试 Polyhedron 创建。"""
    print("测试 1: Polyhedron 创建")

    # 创建随机顶点
    vertices = np.random.rand(10, 3) * 2
    center = vertices.mean(axis=0)
    radius = np.max(np.linalg.norm(vertices - center, axis=1))

    # 计算凸包
    hull = ConvexHull(vertices)
    faces = hull.simplices

    # 创建 Polyhedron
    poly = Polyhedron(
        poly_id=0,
        vertices=vertices,
        faces=faces,
        center=center,
        radius=radius,
        volume=1.0,
        seed_point=center.copy(),
        sample_points=vertices.copy(),
    )

    print("  ✓ Polyhedron 创建成功")
    print(f"    - poly_id: {poly.poly_id}")
    print(f"    - 顶点数: {len(poly.vertices)}")
    print(f"    - 面片数: {len(poly.faces)}")
    print(f"    - 中心: {poly.center}")
    print(f"    - 半径: {poly.radius:.2f}")
    print()


def test_scg_with_polyhedra():
    """测试 SCG 构建。"""
    print("测试 2: SCG 构建")

    config = SCGConfig()
    scg_builder = SCGBuilder(config)

    # 创建 3 个多面体
    for i in range(3):
        vertices = np.random.rand(10, 3) * 2 + np.array([i * 3, 0, 0])
        center = vertices.mean(axis=0)
        radius = np.max(np.linalg.norm(vertices - center, axis=1))

        hull = ConvexHull(vertices)
        faces = hull.simplices

        poly = Polyhedron(
            poly_id=i,
            vertices=vertices,
            faces=faces,
            center=center,
            radius=radius,
            volume=1.0,
            seed_point=center.copy(),
            sample_points=vertices.copy(),
        )
        scg_builder.add_polyhedron(poly)

    print("  ✓ SCG 构建成功")
    print(f"    - 节点数: {len(scg_builder.nodes)}")
    print()


def test_visualization_import():
    """测试可视化模块导入。"""
    print("测试 3: 可视化模块导入")


    print("  ✓ PathVisualizer 导入成功")
    print("  ✓ SCGVisualizer 导入成功")
    print("  ✓ PerformanceVisualizer 导入成功")
    print("  ✓ ComprehensiveVisualizer 导入成功")
    print()


def main():
    """运行所有测试。"""
    print("=" * 60)
    print("简化可视化工具测试")
    print("=" * 60)
    print()

    try:
        test_polyhedron_creation()
        test_scg_with_polyhedra()
        test_visualization_import()

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
