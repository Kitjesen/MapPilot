#!/usr/bin/env python3
"""
几何增强拓扑图测试脚本

测试内容:
1. TopoNode 新字段的序列化/反序列化
2. GeometryExtractor 的几何提取功能
3. TopologySemGraph 与 GeometryExtractor 的集成
"""

import json
import sys
from pathlib import Path

import numpy as np

# 添加模块路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from semantic_perception.geometry_extractor import GeometryExtractor
from semantic_perception.topology_graph import TopoNode, TopologySemGraph


def test_toponode_serialization():
    """测试 TopoNode 的序列化/反序列化 (包含几何字段)。"""
    print("=" * 60)
    print("测试 1: TopoNode 序列化/反序列化")
    print("=" * 60)

    # 创建带几何信息的节点
    node = TopoNode(
        node_id=1,
        node_type="room",
        name="test_room",
        center=np.array([10.0, 20.0]),
        room_type="office",
        bounding_box={"x_min": 8.0, "x_max": 12.0, "y_min": 18.0, "y_max": 22.0},
        convex_hull=np.array([[8.0, 18.0], [12.0, 18.0], [12.0, 22.0], [8.0, 22.0]]),
        traversable_area=16.0,
        height_range={"floor": 0.0, "ceiling": 2.5},
        geometry_confidence=0.85,
    )

    # 序列化
    tsg = TopologySemGraph()
    tsg._nodes[1] = node
    data = tsg.to_dict()

    print(f"✓ 序列化成功")
    print(f"  节点数据: {json.dumps(data['nodes'][0], indent=2)}")

    # 反序列化
    tsg2 = TopologySemGraph.from_dict(data)
    node2 = tsg2.get_node(1)

    assert node2 is not None, "节点反序列化失败"
    assert node2.bounding_box == node.bounding_box, "边界框不匹配"
    assert np.allclose(node2.convex_hull, node.convex_hull), "凸包不匹配"
    assert node2.traversable_area == node.traversable_area, "面积不匹配"
    assert node2.height_range == node.height_range, "高度范围不匹配"
    assert node2.geometry_confidence == node.geometry_confidence, "置信度不匹配"

    print(f"✓ 反序列化成功，所有字段匹配")
    print()


def test_geometry_extractor_mock():
    """测试 GeometryExtractor (使用模拟 Tomogram)。"""
    print("=" * 60)
    print("测试 2: GeometryExtractor (模拟数据)")
    print("=" * 60)

    # 创建模拟 Tomogram
    class MockTomogram:
        def __init__(self):
            self.resolution = 0.2  # 20cm 分辨率
            self.map_center = np.array([0.0, 0.0])
            self.map_dim_x = 100
            self.map_dim_y = 100

            # 创建模拟可通行性地图 (中心 10×10 米区域可通行)
            self.inflated_cost = np.ones((1, 100, 100), dtype=np.float32)
            # 中心 50×50 栅格 (10×10 米) 设为可通行
            self.inflated_cost[0, 25:75, 25:75] = 0.0

            # 创建模拟高程地图
            self.layers_g = np.zeros((1, 100, 100), dtype=np.float32)  # 地面高程
            self.layers_c = np.ones((1, 100, 100), dtype=np.float32) * 2.5  # 天花板高程

    tomogram = MockTomogram()
    extractor = GeometryExtractor(tomogram)

    # 测试坐标转换
    print("测试坐标转换:")
    world_pos = np.array([2.0, 3.0])
    grid_pos = extractor.world_to_grid(world_pos)
    world_pos2 = extractor.grid_to_world(*grid_pos)
    print(f"  世界坐标: {world_pos} → 栅格坐标: {grid_pos} → 世界坐标: {world_pos2}")
    assert np.allclose(world_pos, world_pos2, atol=0.1), "坐标转换不一致"
    print(f"✓ 坐标转换正确")

    # 测试几何提取
    print("\n测试几何提取:")
    room_center = np.array([0.0, 0.0])
    geometry = extractor.extract_room_geometry(
        room_center=room_center,
        search_radius=5.0,
        cost_threshold=0.5,
    )

    print(f"  边界框: {geometry['bounding_box']}")
    print(f"  凸包顶点数: {len(geometry['convex_hull'])}")
    print(f"  可通行面积: {geometry['traversable_area']:.2f} m²")
    print(f"  高度范围: {geometry['height_range']}")
    print(f"  置信度: {geometry['confidence']:.2f}")

    assert geometry["traversable_area"] > 0, "面积应大于 0"
    assert len(geometry["convex_hull"]) >= 3, "凸包至少需要 3 个顶点"
    assert 0 <= geometry["confidence"] <= 1, "置信度应在 [0, 1] 范围内"

    print(f"✓ 几何提取成功")
    print()


def test_topology_integration():
    """测试 TopologySemGraph 与 GeometryExtractor 的集成。"""
    print("=" * 60)
    print("测试 3: TopologySemGraph 集成")
    print("=" * 60)

    # 创建模拟 Tomogram
    class MockTomogram:
        def __init__(self):
            self.resolution = 0.2
            self.map_center = np.array([0.0, 0.0])
            self.map_dim_x = 100
            self.map_dim_y = 100
            self.inflated_cost = np.ones((1, 100, 100), dtype=np.float32)
            self.inflated_cost[0, 25:75, 25:75] = 0.0
            self.layers_g = np.zeros((1, 100, 100), dtype=np.float32)
            self.layers_c = np.ones((1, 100, 100), dtype=np.float32) * 2.5

    tomogram = MockTomogram()
    extractor = GeometryExtractor(tomogram)

    # 创建拓扑图并连接几何提取器
    tsg = TopologySemGraph()
    tsg.set_geometry_extractor(extractor)

    # 模拟场景图更新
    scene_graph = {
        "rooms": [
            {
                "room_id": 1,
                "name": "office_1",
                "center": {"x": 0.0, "y": 0.0},
                "semantic_labels": ["desk", "chair", "computer"],
            },
            {
                "room_id": 2,
                "name": "corridor",
                "center": {"x": 5.0, "y": 0.0},
                "semantic_labels": ["door"],
            },
        ],
        "topology_edges": [
            {
                "from_room": 1,
                "to_room": 2,
                "type": "door",
                "distance": 5.0,
            }
        ],
    }

    tsg.update_from_scene_graph(scene_graph)

    # 验证几何信息
    room1 = tsg.get_node(1)
    room2 = tsg.get_node(2)

    assert room1 is not None, "房间 1 应存在"
    assert room2 is not None, "房间 2 应存在"

    print(f"房间 1 ({room1.name}):")
    print(f"  边界框: {room1.bounding_box}")
    print(f"  可通行面积: {room1.traversable_area:.2f} m²")
    print(f"  几何置信度: {room1.geometry_confidence:.2f}")

    print(f"\n房间 2 ({room2.name}):")
    print(f"  边界框: {room2.bounding_box}")
    print(f"  可通行面积: {room2.traversable_area:.2f} m²")
    print(f"  几何置信度: {room2.geometry_confidence:.2f}")

    assert room1.bounding_box is not None, "房间 1 应有边界框"
    assert room1.traversable_area > 0, "房间 1 应有可通行面积"
    assert room1.geometry_confidence > 0, "房间 1 应有几何置信度"

    print(f"\n✓ 拓扑图集成成功")
    print()


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("几何增强拓扑图测试套件")
    print("=" * 60 + "\n")

    try:
        test_toponode_serialization()
        test_geometry_extractor_mock()
        test_topology_integration()

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
