#!/usr/bin/env python3
"""
USS-Nav 空间表示系统集成示例

展示如何将所有组件组合在一起:
1. 几何增强拓扑图
2. 混合路径规划器
3. 多面体扩展
4. SCG 构建
5. Leiden 区域分割
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from semantic_perception.geometry_extractor import GeometryExtractor
from semantic_perception.hybrid_planner import HybridPlanner
from semantic_perception.leiden_segmentation import LeidenConfig, LeidenSegmenter
from semantic_perception.polyhedron_expansion import (
    PolyhedronExpander,
    PolyhedronExpansionConfig,
)
from semantic_perception.scg_builder import SCGBuilder, SCGConfig
from semantic_perception.topology_graph import TopologySemGraph


def create_mock_environment():
    """创建模拟环境。"""
    print("=" * 60)
    print("创建模拟环境")
    print("=" * 60)

    # 创建模拟 Tomogram
    class MockTomogram:
        def __init__(self):
            self.resolution = 0.2
            self.map_center = np.array([0.0, 0.0, 0.0])
            self.map_dim_x = 100
            self.map_dim_y = 100

            # 创建走廊 + 房间布局
            self.inflated_cost = np.ones((100, 100, 10), dtype=np.float32)

            # 走廊 (中间 10 格宽)
            self.inflated_cost[45:55, :, :] = 0.0

            # 房间 1 (左侧)
            self.inflated_cost[30:45, 20:40, :] = 0.0

            # 房间 2 (右侧)
            self.inflated_cost[55:70, 20:40, :] = 0.0

            # 房间 3 (右侧远端)
            self.inflated_cost[55:70, 60:80, :] = 0.0

            self.layers_g = np.zeros((10, 100, 100), dtype=np.float32)
            self.layers_c = np.ones((10, 100, 100), dtype=np.float32) * 2.5

    tomogram = MockTomogram()

    # 创建占据栅格 (用于多面体扩展)
    occupancy_grid = tomogram.inflated_cost[..., 0].T  # (100, 100)
    occupancy_grid = np.expand_dims(occupancy_grid, axis=2)  # (100, 100, 1)

    print(f"✓ 环境创建完成")
    print(f"  Tomogram: {tomogram.map_dim_x}×{tomogram.map_dim_y}")
    print(f"  占据栅格: {occupancy_grid.shape}")
    print()

    return tomogram, occupancy_grid


def demo_geometry_enhanced_topology(tomogram):
    """演示几何增强拓扑图。"""
    print("=" * 60)
    print("1. 几何增强拓扑图")
    print("=" * 60)

    # 创建拓扑图
    tsg = TopologySemGraph()
    extractor = GeometryExtractor(tomogram)
    tsg.set_geometry_extractor(extractor)

    # 模拟场景图
    scene_graph = {
        "rooms": [
            {"room_id": 1, "name": "corridor", "center": {"x": 0.0, "y": 0.0}},
            {"room_id": 2, "name": "room_left", "center": {"x": -3.0, "y": -6.0}},
            {"room_id": 3, "name": "room_right", "center": {"x": 3.0, "y": -6.0}},
        ],
        "topology_edges": [
            {"from_room": 1, "to_room": 2, "type": "door", "distance": 6.0},
            {"from_room": 1, "to_room": 3, "type": "door", "distance": 6.0},
        ],
    }

    tsg.update_from_scene_graph(scene_graph)

    print(f"房间数: {len(tsg.rooms)}")
    for room in tsg.rooms:
        print(f"  {room.name}:")
        print(f"    中心: {room.center}")
        if room.bounding_box:
            print(f"    边界框: {room.bounding_box}")
        if room.traversable_area > 0:
            print(f"    可通行面积: {room.traversable_area:.2f} m²")

    print(f"✓ 几何增强拓扑图构建完成")
    print()

    return tsg


def demo_hybrid_planner(tsg, tomogram):
    """演示混合路径规划器。"""
    print("=" * 60)
    print("2. 混合路径规划器")
    print("=" * 60)

    planner = HybridPlanner(tsg, tomogram)

    # 规划路径
    start = np.array([-3.0, -6.0, 0.0])
    goal = np.array([3.0, -6.0, 0.0])

    print(f"规划路径: {start[:2]} → {goal[:2]}")

    result = planner.plan_path(start, goal)

    if result.success:
        print(f"✓ 路径规划成功")
        print(f"  房间序列: {result.room_sequence}")
        print(f"  路径点数: {result.num_waypoints}")
        print(f"  总代价: {result.total_cost:.2f}")
        print(f"  规划时间: {result.total_planning_time*1000:.2f}ms")
        print(f"    - 拓扑层: {result.topology_planning_time*1000:.2f}ms")
        print(f"    - 几何层: {result.geometry_planning_time*1000:.2f}ms")
    else:
        print(f"✗ 路径规划失败")

    print()


def demo_polyhedron_expansion(occupancy_grid):
    """演示多面体扩展。"""
    print("=" * 60)
    print("3. 多面体扩展")
    print("=" * 60)

    config = PolyhedronExpansionConfig(
        num_sphere_samples=32,
        r_min=0.5,
        r_max=2.0,
        r_step=0.5,
        min_polyhedron_volume=0.5,
        max_polyhedra=20,
        coverage_threshold=0.6,
    )

    expander = PolyhedronExpander(config)

    grid_resolution = 0.2
    grid_origin = np.array([0.0, 0.0, 0.0])

    polyhedra = expander.expand(occupancy_grid, grid_resolution, grid_origin)

    print(f"✓ 多面体扩展完成")
    print(f"  生成多面体数: {len(polyhedra)}")
    print(f"  总体积: {sum(p.volume for p in polyhedra):.2f} m³")
    print()

    return polyhedra


def demo_scg_builder(polyhedra, occupancy_grid):
    """演示 SCG 构建。"""
    print("=" * 60)
    print("4. SCG 构建")
    print("=" * 60)

    config = SCGConfig(
        adjacency_threshold=0.3,
        connectivity_samples=15,
    )

    builder = SCGBuilder(config)

    # 添加多面体
    for poly in polyhedra:
        builder.add_polyhedron(poly)

    # 构建边
    grid_resolution = 0.2
    grid_origin = np.array([0.0, 0.0, 0.0])

    builder.build_edges(occupancy_grid, grid_resolution, grid_origin)

    stats = builder.get_statistics()

    print(f"✓ SCG 构建完成")
    print(f"  节点数: {stats['num_nodes']}")
    print(f"  边数: {stats['num_edges']}")
    print(f"  边类型: {stats['edge_types']}")
    print(f"  平均度: {stats['avg_degree']:.2f}")
    print()

    return builder


def demo_leiden_segmentation(scg_builder):
    """演示 Leiden 区域分割。"""
    print("=" * 60)
    print("5. Leiden 区域分割")
    print("=" * 60)

    config = LeidenConfig(
        resolution=1.0,
        min_region_size=2,
    )

    segmenter = LeidenSegmenter(config)

    regions = segmenter.segment(scg_builder)

    print(f"✓ Leiden 分割完成")
    print(f"  区域数: {len(regions)}")

    for region in regions:
        print(f"  区域 {region.region_id} ({region.region_type}):")
        print(f"    节点数: {len(region.node_ids)}")
        print(f"    中心: {region.center}")
        print(f"    体积: {region.volume:.2f} m³")

    print()


def main():
    """运行完整演示。"""
    print("\n" + "=" * 60)
    print("USS-Nav 空间表示系统集成演示")
    print("=" * 60 + "\n")

    try:
        # 创建环境
        tomogram, occupancy_grid = create_mock_environment()

        # 1. 几何增强拓扑图
        tsg = demo_geometry_enhanced_topology(tomogram)

        # 2. 混合路径规划器
        demo_hybrid_planner(tsg, tomogram)

        # 3. 多面体扩展
        polyhedra = demo_polyhedron_expansion(occupancy_grid)

        # 4. SCG 构建
        scg_builder = demo_scg_builder(polyhedra, occupancy_grid)

        # 5. Leiden 区域分割
        demo_leiden_segmentation(scg_builder)

        print("=" * 60)
        print("✓ 完整演示成功!")
        print("=" * 60)
        return 0

    except Exception as e:
        print(f"\n✗ 演示失败: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
