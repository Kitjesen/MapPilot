#!/usr/bin/env python3
"""Topology geometry-field serialization tests."""

import json

import numpy as np

from memory.spatial.topology_graph import TopologySemGraph, TopoNode


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
        convex_hull=np.array(
            [[8.0, 18.0], [12.0, 18.0], [12.0, 22.0], [8.0, 22.0]]
        ),
        traversable_area=16.0,
        height_range={"floor": 0.0, "ceiling": 2.5},
        geometry_confidence=0.85,
    )

    # 序列化
    tsg = TopologySemGraph()
    tsg._nodes[1] = node
    data = tsg.to_dict()

    print("✓ 序列化成功")
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

    print("✓ 反序列化成功，所有字段匹配")
    print()
