#!/usr/bin/env python3
"""
SCG 构建器测试脚本

测试内容:
1. 节点添加和管理
2. 邻接边检测
3. 连通边检测
4. 路径搜索
5. 回环检测
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from semantic_perception.polyhedron_expansion import Polyhedron
from semantic_perception.scg_builder import EdgeType, SCGBuilder, SCGConfig


def create_test_polyhedra() -> list:
    """创建测试多面体。"""
    polyhedra = []

    # 多面体 0: 原点附近
    polyhedra.append(Polyhedron(
        poly_id=0,
        vertices=np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]),
        faces=np.array([[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]]),
        center=np.array([0.25, 0.25, 0.25]),
        volume=0.167,
        radius=0.5,
        seed_point=np.array([0.25, 0.25, 0.25]),
        sample_points=np.array([[0, 0, 0]]),
    ))

    # 多面体 1: 邻接 (距离 0.1m)
    polyhedra.append(Polyhedron(
        poly_id=1,
        vertices=np.array([[1.1, 0, 0], [2.1, 0, 0], [1.1, 1, 0], [1.1, 0, 1]]),
        faces=np.array([[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]]),
        center=np.array([1.35, 0.25, 0.25]),
        volume=0.167,
        radius=0.5,
        seed_point=np.array([1.35, 0.25, 0.25]),
        sample_points=np.array([[1.1, 0, 0]]),
    ))

    # 多面体 2: 连通但不邻接 (距离 2m)
    polyhedra.append(Polyhedron(
        poly_id=2,
        vertices=np.array([[3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]]),
        faces=np.array([[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]]),
        center=np.array([3.25, 0.25, 0.25]),
        volume=0.167,
        radius=0.5,
        seed_point=np.array([3.25, 0.25, 0.25]),
        sample_points=np.array([[3, 0, 0]]),
    ))

    # 多面体 3: 远离 (距离 10m)
    polyhedra.append(Polyhedron(
        poly_id=3,
        vertices=np.array([[10, 0, 0], [11, 0, 0], [10, 1, 0], [10, 0, 1]]),
        faces=np.array([[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]]),
        center=np.array([10.25, 0.25, 0.25]),
        volume=0.167,
        radius=0.5,
        seed_point=np.array([10.25, 0.25, 0.25]),
        sample_points=np.array([[10, 0, 0]]),
    ))

    return polyhedra


def test_node_management():
    """测试节点管理。"""
    print("=" * 60)
    print("测试 1: 节点管理")
    print("=" * 60)

    config = SCGConfig()
    builder = SCGBuilder(config)

    polyhedra = create_test_polyhedra()

    # 添加节点
    for poly in polyhedra:
        builder.add_polyhedron(poly)

    print(f"添加节点数: {len(builder.nodes)}")
    assert len(builder.nodes) == 4, "应该有 4 个节点"

    # 获取节点
    poly0 = builder.get_polyhedron(0)
    assert poly0 is not None, "应该能获取节点 0"
    print(f"节点 0 中心: {poly0.center}")

    # 移除节点
    builder.remove_polyhedron(3)
    print(f"移除节点 3 后: {len(builder.nodes)} 个节点")
    assert len(builder.nodes) == 3, "应该剩余 3 个节点"

    print(f"✓ 节点管理正确")
    print()


def test_adjacency_detection():
    """测试邻接边检测。"""
    print("=" * 60)
    print("测试 2: 邻接边检测")
    print("=" * 60)

    config = SCGConfig(adjacency_threshold=0.2)
    builder = SCGBuilder(config)

    polyhedra = create_test_polyhedra()[:2]  # 只用前两个 (邻接)

    for poly in polyhedra:
        builder.add_polyhedron(poly)

    # 构建边 (不需要占据栅格)
    builder.build_edges()

    print(f"生成边数: {len(builder.edges)}")
    print(f"边类型统计: {builder._count_edges_by_type()}")

    # 检查邻接边
    adjacency_edges = [e for e in builder.edges if e.edge_type == EdgeType.ADJACENCY]
    print(f"邻接边数: {len(adjacency_edges)}")

    assert len(adjacency_edges) > 0, "应该检测到邻接边"

    print(f"✓ 邻接边检测正确")
    print()


def test_connectivity_detection():
    """测试连通边检测。"""
    print("=" * 60)
    print("测试 3: 连通边检测")
    print("=" * 60)

    config = SCGConfig(
        adjacency_threshold=0.2,
        connectivity_samples=20,
    )
    builder = SCGBuilder(config)

    polyhedra = create_test_polyhedra()[:3]  # 前三个

    for poly in polyhedra:
        builder.add_polyhedron(poly)

    # 创建占据栅格 (全部自由空间)
    occupancy_grid = np.zeros((50, 50, 50), dtype=np.float32)
    grid_resolution = 0.2
    grid_origin = np.array([0.0, 0.0, 0.0])

    # 构建边
    builder.build_edges(occupancy_grid, grid_resolution, grid_origin)

    print(f"生成边数: {len(builder.edges)}")
    print(f"边类型统计: {builder._count_edges_by_type()}")

    # 检查连通边
    connectivity_edges = [e for e in builder.edges if e.edge_type == EdgeType.CONNECTIVITY]
    print(f"连通边数: {len(connectivity_edges)}")

    assert len(connectivity_edges) > 0, "应该检测到连通边"

    print(f"✓ 连通边检测正确")
    print()


def test_path_search():
    """测试路径搜索。"""
    print("=" * 60)
    print("测试 4: 路径搜索")
    print("=" * 60)

    config = SCGConfig()
    builder = SCGBuilder(config)

    polyhedra = create_test_polyhedra()[:3]

    for poly in polyhedra:
        builder.add_polyhedron(poly)

    # 构建边
    occupancy_grid = np.zeros((50, 50, 50), dtype=np.float32)
    builder.build_edges(occupancy_grid, 0.2, np.array([0.0, 0.0, 0.0]))

    # 搜索路径: 0 → 2
    cost, path = builder.shortest_path(0, 2)

    print(f"路径: {path}")
    print(f"代价: {cost:.2f}")

    assert len(path) >= 2, "路径至少包含起点和终点"
    assert path[0] == 0 and path[-1] == 2, "路径应该从 0 到 2"

    print(f"✓ 路径搜索正确")
    print()


def test_loop_closure():
    """测试回环检测。"""
    print("=" * 60)
    print("测试 5: 回环检测")
    print("=" * 60)

    config = SCGConfig(
        loop_closure_threshold=0.5,
        loop_closure_volume_ratio=0.8,
    )
    builder = SCGBuilder(config)

    # 添加第一个多面体
    poly1 = create_test_polyhedra()[0]
    builder.add_polyhedron(poly1)

    print(f"添加多面体 0, 节点数: {len(builder.nodes)}")

    # 添加重复的多面体 (中心距离 0.1m)
    poly_duplicate = Polyhedron(
        poly_id=10,
        vertices=poly1.vertices + 0.1,
        faces=poly1.faces,
        center=poly1.center + 0.1,
        volume=poly1.volume,
        radius=poly1.radius,
        seed_point=poly1.seed_point + 0.1,
        sample_points=poly1.sample_points + 0.1,
    )

    builder.add_polyhedron(poly_duplicate)

    print(f"尝试添加重复多面体, 节点数: {len(builder.nodes)}")

    # 应该检测到回环，不添加重复节点
    assert len(builder.nodes) == 1, "应该检测到回环，不添加重复节点"

    print(f"✓ 回环检测正确")
    print()


def test_statistics():
    """测试统计信息。"""
    print("=" * 60)
    print("测试 6: 统计信息")
    print("=" * 60)

    config = SCGConfig()
    builder = SCGBuilder(config)

    polyhedra = create_test_polyhedra()[:3]

    for poly in polyhedra:
        builder.add_polyhedron(poly)

    occupancy_grid = np.zeros((50, 50, 50), dtype=np.float32)
    builder.build_edges(occupancy_grid, 0.2, np.array([0.0, 0.0, 0.0]))

    stats = builder.get_statistics()

    print(f"统计信息:")
    print(f"  节点数: {stats['num_nodes']}")
    print(f"  边数: {stats['num_edges']}")
    print(f"  边类型: {stats['edge_types']}")
    print(f"  平均度: {stats['avg_degree']:.2f}")

    assert stats['num_nodes'] == 3, "节点数应该是 3"
    assert stats['num_edges'] > 0, "边数应该大于 0"

    print(f"✓ 统计信息正确")
    print()


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("SCG 构建器测试套件")
    print("=" * 60 + "\n")

    try:
        test_node_management()
        test_adjacency_detection()
        test_connectivity_detection()
        test_path_search()
        test_loop_closure()
        test_statistics()

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
