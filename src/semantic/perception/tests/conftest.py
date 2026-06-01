"""
conftest.py — semantic_perception 测试共享夹具

提供:
  - numpy 测试帮助函数
  - 标准 Detection3D 工厂
  - 简单场景图字典工厂
  - IndustrialKnowledgeGraph 单例 (避免重复构建)
"""

import os
import sys

# Ensure all lingtu package namespaces are importable when running pytest
# directly inside this directory (without PYTHONPATH set).
_here = os.path.dirname(os.path.abspath(__file__))
_repo = os.path.abspath(os.path.join(_here, "..", "..", "..", ".."))
_src  = os.path.join(_repo, "src")

for _p in [
    _repo,
    _src,
    os.path.join(_src, "semantic", "perception"),
    os.path.join(_src, "semantic", "planner"),
    os.path.join(_src, "semantic", "common"),
]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import pytest

# ─────────────────────────────────────────────────────────────────────────────
#  Detection3D 工厂
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def make_detection():
    """返回 Detection3D 工厂函数。"""
    from semantic.perception.projection import Detection3D

    def _factory(
        label: str = "chair",
        pos=(1.0, 2.0, 0.5),
        score: float = 0.9,
        features=None,
    ) -> Detection3D:
        if features is None:
            f = np.random.rand(512).astype(np.float32)
            f /= np.linalg.norm(f)
        else:
            f = np.asarray(features, dtype=np.float32)
        return Detection3D(
            position=np.array(pos, dtype=np.float32),
            label=label,
            score=score,
            bbox_2d=np.array([0, 0, 100, 100], dtype=np.float32),
            depth=float(np.linalg.norm(pos[:2])),
            features=f,
        )

    return _factory


# ─────────────────────────────────────────────────────────────────────────────
#  Scene graph dictionary factory
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def make_scene_graph_dict():
    """返回场景图字典构造器 (供 TopologySemGraph.update_from_scene_graph 使用)。"""

    def _factory(rooms=None, topology_edges=None):
        return {
            "rooms": rooms or [],
            "topology_edges": topology_edges or [],
        }

    return _factory


@pytest.fixture
def make_room_dict():
    """返回单个 room 字典构造器。"""

    def _factory(rid, x=0.0, y=0.0, name=None, labels=None):
        return {
            "room_id": rid,
            "name": name or f"room_{rid}",
            "center": {"x": x, "y": y},
            "semantic_labels": labels or [],
        }

    return _factory


# ─────────────────────────────────────────────────────────────────────────────
#  KG 单例 (模块级缓存，避免每个测试重建)
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def kg():
    """共享的 IndustrialKnowledgeGraph 实例 (模块范围)。"""
    from memory.knowledge.knowledge_graph import IndustrialKnowledgeGraph
    return IndustrialKnowledgeGraph()


# ─────────────────────────────────────────────────────────────────────────────
#  InstanceTracker 工厂
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def tracker():
    """新鲜的 InstanceTracker 实例。"""
    from semantic.perception.instance_tracker import InstanceTracker
    return InstanceTracker(merge_distance=0.5, clip_threshold=0.75)


# ─────────────────────────────────────────────────────────────────────────────
#  TopologySemGraph 工厂
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def tsg():
    """新鲜的 TopologySemGraph 实例。"""
    from memory.spatial.topology_graph import TopologySemGraph
    return TopologySemGraph()


@pytest.fixture
def tsg_two_rooms():
    """预配置两个房间 + 一条边的 TopologySemGraph。"""
    from memory.spatial.topology_graph import TopologySemGraph

    g = TopologySemGraph()
    sg = {
        "rooms": [
            {"room_id": 0, "name": "corridor", "center": {"x": 0.0, "y": 0.0}, "semantic_labels": ["door"]},
            {"room_id": 1, "name": "office", "center": {"x": 10.0, "y": 0.0}, "semantic_labels": ["desk", "chair"]},
        ],
        "topology_edges": [
            {"from_room": 0, "to_room": 1, "type": "door", "distance": 10.0},
        ],
    }
    g.update_from_scene_graph(sg)
    return g
