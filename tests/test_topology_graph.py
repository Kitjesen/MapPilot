"""
拓扑语义图 (TSG) 综合测试。

覆盖:
  1. 图构建与同步 (从 scene graph 更新)
  2. 前沿节点管理
  3. 穿越记忆 (房间切换检测)
  4. 信息增益计算
  5. Dijkstra 最短路径
  6. 探索目标选择 (Algorithm 2)
  7. 序列化/反序列化
  8. LLM prompt 生成
"""

import sys
import time
from pathlib import Path

import numpy as np
import pytest

# 确保可导入项目模块
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "semantic_perception"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "semantic_planner"))

from memory.spatial.topology_graph import (
    TopologySemGraph,
    TopoNode,
    TopoEdge,
    ExplorationTarget,
)


# ── 测试数据 ──────────────────────────────────────────────────

def make_scene_graph() -> dict:
    """构建一个 3 房间 + 2 拓扑边的测试场景图。

    布局:
        corridor (0,0) -- door --> office (5,0)
                       -- passage --> kitchen (0,5)
    """
    return {
        "rooms": [
            {
                "room_id": 0,
                "name": "corridor",
                "center": {"x": 0.0, "y": 0.0},
                "object_ids": [1, 2, 3],
                "group_ids": [0],
                "semantic_labels": ["door", "sign", "fire extinguisher"],
            },
            {
                "room_id": 1,
                "name": "office",
                "center": {"x": 5.0, "y": 0.0},
                "object_ids": [4, 5, 6],
                "group_ids": [1],
                "semantic_labels": ["desk", "chair", "monitor"],
            },
            {
                "room_id": 2,
                "name": "kitchen",
                "center": {"x": 0.0, "y": 5.0},
                "object_ids": [7, 8],
                "group_ids": [2],
                "semantic_labels": ["refrigerator", "sink"],
            },
        ],
        "topology_edges": [
            {
                "from_room": 0,
                "to_room": 1,
                "type": "door",
                "mediator": "door",
                "mediator_pos": {"x": 2.5, "y": 0.0},
                "distance": 5.0,
            },
            {
                "from_room": 0,
                "to_room": 2,
                "type": "passage",
                "mediator": "corridor",
                "distance": 5.0,
            },
        ],
        "frontier_nodes": [
            {
                "position": {"x": 8.0, "y": 0.0},
                "direction": {"dx": 1.0, "dy": 0.0},
                "nearest_room_id": 1,
                "frontier_size": 3.0,
                "source": "door_outward",
            },
            {
                "position": {"x": 0.0, "y": 9.0},
                "direction": {"dx": 0.0, "dy": 1.0},
                "nearest_room_id": 2,
                "frontier_size": 2.0,
                "source": "sparse_sector",
            },
        ],
    }


# ── 测试类 ──────────────────────────────────────────────────

class TestGraphConstruction:
    """图构建与同步。"""

    def test_update_from_scene_graph(self):
        tsg = TopologySemGraph()
        sg = make_scene_graph()
        tsg.update_from_scene_graph(sg)

        assert len(tsg.rooms) == 3
        assert tsg.get_node(0).name == "corridor"
        assert tsg.get_node(1).name == "office"
        assert tsg.get_node(2).name == "kitchen"

    def test_room_types_inferred(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        assert tsg.get_node(0).room_type == "corridor"
        assert tsg.get_node(1).room_type == "office"
        assert tsg.get_node(2).room_type == "kitchen"

    def test_edges_created(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        neighbors_0 = sorted(tsg.get_neighbors(0))
        assert 1 in neighbors_0
        assert 2 in neighbors_0

    def test_preserves_visit_state_on_update(self):
        tsg = TopologySemGraph()
        sg = make_scene_graph()
        tsg.update_from_scene_graph(sg)

        tsg.record_robot_position(0.0, 0.0)
        assert tsg.get_node(0).visited is True

        tsg.update_from_scene_graph(sg)
        assert tsg.get_node(0).visited is True

    def test_stale_room_removed(self):
        tsg = TopologySemGraph()
        sg = make_scene_graph()
        tsg.update_from_scene_graph(sg)
        assert tsg.get_node(2) is not None

        sg["rooms"] = sg["rooms"][:2]
        tsg.update_from_scene_graph(sg)
        assert tsg.get_node(2) is None


class TestFrontierNodes:
    """前沿节点管理。"""

    def test_add_frontier(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        fid = tsg.add_frontier(
            position=np.array([8.0, 0.0]),
            direction=np.array([1.0, 0.0]),
            nearest_room_id=1,
            frontier_size=3.0,
        )
        assert fid >= 10000
        assert len(tsg.frontiers) == 1
        assert 1 in tsg.get_neighbors(fid)

    def test_update_frontiers_from_costmap(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        points = [np.array([8.0, 0.0]), np.array([0.0, 9.0])]
        sizes = [3.0, 2.0]
        new_ids = tsg.update_frontiers_from_costmap(points, sizes)

        assert len(new_ids) == 2
        assert len(tsg.frontiers) == 2

    def test_update_clears_old_frontiers(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        tsg.update_frontiers_from_costmap([np.array([10.0, 0.0])])
        assert len(tsg.frontiers) == 1

        tsg.update_frontiers_from_costmap([np.array([0.0, 10.0]), np.array([-5.0, 0.0])])
        assert len(tsg.frontiers) == 2

    def test_too_close_frontier_rejected(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        # This point is very close to room 0 center
        new_ids = tsg.update_frontiers_from_costmap([np.array([0.5, 0.0])])
        assert len(new_ids) == 0


class TestTraversalMemory:
    """穿越记忆。"""

    def test_room_detection(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        room_id = tsg.record_robot_position(0.0, 0.0)
        assert room_id == 0

    def test_room_transition(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        tsg.record_robot_position(0.0, 0.0)
        assert tsg.current_room_id == 0

        tsg.record_robot_position(5.0, 0.0)
        assert tsg.current_room_id == 1
        assert 0 in tsg.visited_room_ids
        assert 1 in tsg.visited_room_ids

    def test_traversal_edge_created(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        tsg.record_robot_position(0.0, 0.0)
        tsg.record_robot_position(5.0, 0.0)

        edge_found = False
        for e in tsg._edges:
            if e.pair == (0, 1):
                assert e.traversal_count >= 1
                edge_found = True
                break
        assert edge_found

    def test_visit_count_increments(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        tsg.record_robot_position(0.0, 0.0)
        tsg.record_robot_position(5.0, 0.0)
        tsg.record_robot_position(0.0, 0.0)

        node0 = tsg.get_node(0)
        assert node0.visit_count == 2


class TestShortestPath:
    """Dijkstra 最短路径。"""

    def test_direct_neighbors(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        cost, path = tsg.shortest_path(0, 1)
        assert cost > 0
        assert path == [0, 1]

    def test_two_hop_path(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        cost, path = tsg.shortest_path(1, 2)
        assert len(path) == 3
        assert path[0] == 1
        assert path[1] == 0
        assert path[2] == 2

    def test_unreachable(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        cost, path = tsg.shortest_path(0, 999)
        assert cost == float("inf")
        assert path == []

    def test_hop_distances(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        hops = tsg.hop_distances(0)
        assert hops[0] == 0
        assert hops[1] == 1
        assert hops[2] == 1


class TestInformationGain:
    """信息增益计算。"""

    def test_unvisited_room_higher_ig(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        tsg.record_robot_position(0.0, 0.0)

        ig_visited = tsg.compute_information_gain(0, "find fire extinguisher")
        ig_unvisited = tsg.compute_information_gain(1, "find fire extinguisher")

        assert ig_unvisited > ig_visited

    def test_frontier_has_positive_ig(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())
        fid = tsg.add_frontier(
            position=np.array([10.0, 0.0]),
            direction=np.array([1.0, 0.0]),
            nearest_room_id=1,
        )

        ig = tsg.compute_information_gain(fid, "find something")
        assert ig > 0

    def test_semantic_engine_boosts_ig(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        ig_no_engine = tsg.compute_information_gain(2, "find refrigerator")

        sys.path.insert(0, str(
            Path(__file__).resolve().parent.parent / "src" / "semantic_planner"
        ))
        from memory.knowledge.semantic_prior import SemanticPriorEngine
        engine = SemanticPriorEngine()

        ig_with_engine = tsg.compute_information_gain(
            2, "find refrigerator", semantic_engine=engine
        )
        assert ig_with_engine >= ig_no_engine


class TestExplorationTarget:
    """Algorithm 2: 探索目标选择。"""

    def test_returns_targets(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())
        tsg.record_robot_position(0.0, 0.0)

        targets = tsg.get_best_exploration_target("find desk")
        assert len(targets) > 0
        assert all(isinstance(t, ExplorationTarget) for t in targets)

    def test_unvisited_preferred(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())
        tsg.record_robot_position(0.0, 0.0)
        tsg.record_robot_position(5.0, 0.0)

        targets = tsg.get_best_exploration_target("find refrigerator")
        assert len(targets) > 0
        best = targets[0]
        assert best.node_id == 2

    def test_frontier_included(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())
        tsg.record_robot_position(0.0, 0.0)
        tsg.record_robot_position(5.0, 0.0)
        tsg.record_robot_position(0.0, 5.0)

        fid = tsg.add_frontier(
            position=np.array([10.0, 0.0]),
            direction=np.array([1.0, 0.0]),
            nearest_room_id=1,
            predicted_room_type="stairwell",
        )

        targets = tsg.get_best_exploration_target("find fire extinguisher")
        node_ids = [t.node_id for t in targets]
        assert fid in node_ids

    def test_scores_decrease_with_visits(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())
        tsg.record_robot_position(0.0, 0.0)

        targets_before = tsg.get_best_exploration_target("find desk")
        score_before = {t.node_id: t.score for t in targets_before}

        tsg.record_robot_position(5.0, 0.0)

        targets_after = tsg.get_best_exploration_target("find desk")
        score_after = {t.node_id: t.score for t in targets_after}

        if 1 in score_before and 1 in score_after:
            assert score_after[1] < score_before[1]

    def test_top_k_limit(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())
        tsg.record_robot_position(0.0, 0.0)

        targets = tsg.get_best_exploration_target("find something", top_k=1)
        assert len(targets) <= 1


class TestSerialization:
    """序列化/反序列化。"""

    def test_roundtrip(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())
        tsg.record_robot_position(0.0, 0.0)
        tsg.add_frontier(
            position=np.array([10.0, 0.0]),
            direction=np.array([1.0, 0.0]),
            nearest_room_id=1,
            predicted_room_type="stairwell",
        )

        data = tsg.to_dict()
        tsg2 = TopologySemGraph.from_dict(data)

        assert len(tsg2.rooms) == len(tsg.rooms)
        assert len(tsg2.frontiers) == len(tsg.frontiers)
        assert tsg2.current_room_id == tsg.current_room_id

    def test_dict_is_json_serializable(self):
        import json
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())
        data = tsg.to_dict()
        json_str = json.dumps(data)
        assert len(json_str) > 0


class TestPromptGeneration:
    """LLM prompt 生成。"""

    def test_zh_prompt(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())
        tsg.record_robot_position(0.0, 0.0)
        tsg.add_frontier(
            position=np.array([10.0, 0.0]),
            direction=np.array([1.0, 0.0]),
            nearest_room_id=1,
        )

        context = tsg.to_prompt_context("zh")
        assert "已知房间" in context
        assert "corridor" in context
        assert "连通关系" in context
        assert "前沿" in context

    def test_en_prompt(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())
        tsg.record_robot_position(0.0, 0.0)

        context = tsg.to_prompt_context("en")
        assert "Rooms" in context
        assert "Connections" in context
        assert "visited" in context.lower()


class TestEdgeCases:
    """边界情况。"""

    def test_empty_graph(self):
        tsg = TopologySemGraph()
        targets = tsg.get_best_exploration_target("find something")
        assert targets == []

    def test_single_room(self):
        tsg = TopologySemGraph()
        sg = {
            "rooms": [
                {
                    "room_id": 0,
                    "name": "corridor",
                    "center": {"x": 0.0, "y": 0.0},
                    "object_ids": [1],
                    "semantic_labels": ["door"],
                }
            ],
            "topology_edges": [],
        }
        tsg.update_from_scene_graph(sg)
        tsg.record_robot_position(0.0, 0.0)

        targets = tsg.get_best_exploration_target("find desk")
        assert len(targets) == 0

    def test_robot_outside_all_rooms(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        room_id = tsg.record_robot_position(100.0, 100.0)
        assert room_id is None

    def test_get_room_by_position(self):
        tsg = TopologySemGraph()
        tsg.update_from_scene_graph(make_scene_graph())

        assert tsg.get_room_by_position(0.5, 0.5) == 0
        assert tsg.get_room_by_position(4.5, 0.0) == 1
        assert tsg.get_room_by_position(100.0, 100.0) is None


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
