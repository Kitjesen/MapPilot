#!/usr/bin/env python3
"""
可视化工具测试脚本

测试内容:
1. 路径可视化（2D/3D）
2. SCG 可视化（2D/3D）
3. 性能对比图
4. 综合可视化
"""

import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from semantic_perception.visualization_tools import (
    PathVisualizer,
    SCGVisualizer,
    PerformanceVisualizer,
    ComprehensiveVisualizer,
)


def create_test_path():
    """创建测试路径。"""
    # 创建一个螺旋路径
    t = np.linspace(0, 4 * np.pi, 100)
    x = t * np.cos(t)
    y = t * np.sin(t)
    z = t * 0.1
    return np.column_stack([x, y, z])


def create_test_occupancy_grid():
    """创建测试占据栅格。"""
    grid = np.zeros((100, 100, 40), dtype=np.float32)

    # 添加一些障碍物
    grid[20:30, 20:30, :] = 1.0
    grid[60:70, 60:70, :] = 1.0
    grid[40:50, 70:80, :] = 1.0

    return grid


def create_test_scg():
    """创建测试 SCG。"""
    from semantic_perception.scg_builder import SCGBuilder, SCGConfig
    from semantic_perception.polyhedron_expansion import Polyhedron
    from scipy.spatial import ConvexHull

    config = SCGConfig()
    scg_builder = SCGBuilder(config)

    # 创建一些测试多面体
    for i in range(5):
        vertices = np.random.rand(10, 3) * 2 + np.array([i * 3, 0, 0])
        center = vertices.mean(axis=0)
        radius = np.max(np.linalg.norm(vertices - center, axis=1))

        # 计算凸包以获取面片
        hull = ConvexHull(vertices)
        faces = hull.simplices

        # 创建种子点和采样点
        seed_point = center.copy()
        sample_points = vertices.copy()

        poly = Polyhedron(
            poly_id=i,
            vertices=vertices,
            faces=faces,
            center=center,
            radius=radius,
            volume=1.0,
            seed_point=seed_point,
            sample_points=sample_points,
        )
        scg_builder.add_polyhedron(poly)

    # 构建边
    occupancy_grid = create_test_occupancy_grid()
    scg_builder.build_edges(occupancy_grid, resolution=0.1, origin=np.array([0, 0, 0]))

    return scg_builder


def create_test_results():
    """创建测试评估结果。"""
    from semantic_perception.evaluation_framework import (
        BenchmarkResult,
        MemoryMetrics,
        UpdateMetrics,
        PathMetrics,
    )

    results = []

    # PCT A* 结果
    for i in range(3):
        result = BenchmarkResult(
            method_name="PCT A*",
            scene_id=f"scene_{i}",
            timestamp=0.0,
            memory=MemoryMetrics(
                total_memory_mb=1.5 + np.random.rand() * 0.5,
                map_memory_mb=1.0,
                graph_memory_mb=0.3,
                other_memory_mb=0.2,
                peak_memory_mb=2.0,
            ),
            update=UpdateMetrics(
                avg_update_time_ms=30.0 + np.random.rand() * 10,
                max_update_time_ms=50.0,
                min_update_time_ms=20.0,
                update_frequency_hz=30.0,
                total_updates=100,
            ),
            path=PathMetrics(
                path_length=10.0 + np.random.rand() * 5,
                path_smoothness=0.8 + np.random.rand() * 0.15,
                path_clearance=0.5,
                planning_time_ms=3.0 + np.random.rand() * 2,
                success_rate=1.0,
                num_waypoints=30,
            ),
        )
        results.append(result)

    # USS-Nav 结果
    for i in range(3):
        result = BenchmarkResult(
            method_name="USS-Nav",
            scene_id=f"scene_{i}",
            timestamp=0.0,
            memory=MemoryMetrics(
                total_memory_mb=0.5 + np.random.rand() * 0.3,
                map_memory_mb=0.3,
                graph_memory_mb=0.1,
                other_memory_mb=0.1,
                peak_memory_mb=0.8,
            ),
            update=UpdateMetrics(
                avg_update_time_ms=60.0 + np.random.rand() * 15,
                max_update_time_ms=80.0,
                min_update_time_ms=50.0,
                update_frequency_hz=15.0,
                total_updates=100,
            ),
            path=PathMetrics(
                path_length=9.0 + np.random.rand() * 4,
                path_smoothness=0.85 + np.random.rand() * 0.1,
                path_clearance=0.6,
                planning_time_ms=0.15 + np.random.rand() * 0.1,
                success_rate=1.0,
                num_waypoints=25,
            ),
        )
        results.append(result)

    return results


def test_path_visualizer_2d():
    """测试 2D 路径可视化。"""
    print("=" * 60)
    print("测试 1: 2D 路径可视化")
    print("=" * 60)

    path = create_test_path()
    occupancy_grid = create_test_occupancy_grid()

    visualizer = PathVisualizer()
    visualizer.plot_path_2d(
        path,
        occupancy_grid=occupancy_grid,
        start=path[0],
        goal=path[-1],
    )

    # 保存到临时文件
    with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
        visualizer.save(f.name)
        print(f"✓ 2D 路径可视化已保存: {f.name}")

    visualizer.close()
    print()


def test_path_visualizer_3d():
    """测试 3D 路径可视化。"""
    print("=" * 60)
    print("测试 2: 3D 路径可视化")
    print("=" * 60)

    path = create_test_path()

    # 创建测试点云
    point_cloud = np.random.rand(1000, 3) * 10

    visualizer = PathVisualizer()
    visualizer.plot_path_3d(
        path,
        point_cloud=point_cloud,
        start=path[0],
        goal=path[-1],
    )

    with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
        visualizer.save(f.name)
        print(f"✓ 3D 路径可视化已保存: {f.name}")

    visualizer.close()
    print()


def test_scg_visualizer_2d():
    """测试 2D SCG 可视化。"""
    print("=" * 60)
    print("测试 3: 2D SCG 可视化")
    print("=" * 60)

    scg_builder = create_test_scg()

    visualizer = SCGVisualizer()
    visualizer.plot_scg_2d(scg_builder)

    with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
        visualizer.save(f.name)
        print(f"✓ 2D SCG 可视化已保存: {f.name}")
        print(f"  节点数: {len(scg_builder.nodes)}")
        print(f"  边数: {len(scg_builder.edges)}")

    visualizer.close()
    print()


def test_scg_visualizer_3d():
    """测试 3D SCG 可视化。"""
    print("=" * 60)
    print("测试 4: 3D SCG 可视化")
    print("=" * 60)

    scg_builder = create_test_scg()

    visualizer = SCGVisualizer()
    visualizer.plot_scg_3d(scg_builder)

    with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
        visualizer.save(f.name)
        print(f"✓ 3D SCG 可视化已保存: {f.name}")

    visualizer.close()
    print()


def test_performance_visualizer():
    """测试性能可视化。"""
    print("=" * 60)
    print("测试 5: 性能对比可视化")
    print("=" * 60)

    results = create_test_results()

    visualizer = PerformanceVisualizer()
    fig = visualizer.plot_comparison(results)

    with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
        visualizer.save(fig, f.name)
        print(f"✓ 性能对比图已保存: {f.name}")
        print(f"  结果数量: {len(results)}")

    import matplotlib.pyplot as plt
    plt.close(fig)
    print()


def test_comprehensive_visualizer():
    """测试综合可视化。"""
    print("=" * 60)
    print("测试 6: 综合可视化")
    print("=" * 60)

    path = create_test_path()
    scg_builder = create_test_scg()
    results = create_test_results()
    occupancy_grid = create_test_occupancy_grid()

    visualizer = ComprehensiveVisualizer()

    with tempfile.TemporaryDirectory() as tmpdir:
        visualizer.visualize_all(
            path=path,
            scg_builder=scg_builder,
            results=results,
            output_dir=tmpdir,
            occupancy_grid=occupancy_grid,
        )

        # 检查生成的文件
        output_path = Path(tmpdir)
        files = list(output_path.glob("*.png"))
        print(f"✓ 综合可视化已生成 {len(files)} 个文件:")
        for f in files:
            print(f"  - {f.name}")

    print()


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("可视化工具测试套件")
    print("=" * 60 + "\n")

    try:
        test_path_visualizer_2d()
        test_path_visualizer_3d()
        test_scg_visualizer_2d()
        test_scg_visualizer_3d()
        test_performance_visualizer()
        test_comprehensive_visualizer()

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
