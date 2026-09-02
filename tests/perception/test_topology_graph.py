"""
test_topology_graph.py — 拓扑语义图单元测试

覆盖:
  - 初始化 / 空状态
  - update_from_scene_graph() — 房间解析 + visit 保留
  - add_frontier() / update_frontiers_from_costmap()
  - record_robot_position() — 房间检测 + 穿越记忆
  - shortest_path() — Dijkstra
  - hop_distances() — BFS
  - get_best_exploration_target()
  - to_prompt_context()
"""

import unittest

import numpy as np

from memory.spatial.topology_graph import (
    ExplorationTarget,
    TopologySemGraph,
    TopoNode,
)

# ─────────────────────────────────────────────────────────────────────────────
#  Helper: 构造简单场景图字典
# ─────────────────────────────────────────────────────────────────────────────

def _simple_sg(rooms=None, topology_edges=None):
    """构造符合 update_from_scene_graph 格式的测试场景图。"""
    rooms = rooms or []
    topology_edges = topology_edges or []
    return {"rooms": rooms, "topology_edges": topology_edges}


def _room(rid, x=0.0, y=0.0, name=None, labels=None):
    return {
        "room_id": rid,
        "name": name or f"room_{rid}",
        "center": {"x": x, "y": y},
        "semantic_labels": labels or [],
    }


def _edge(from_id, to_id, edge_type="door"):
    return {"from_room": from_id, "to_room": to_id, "type": edge_type, "distance": 2.0}


# ─────────────────────────────────────────────────────────────────────────────
#  初始化测试
# ─────────────────────────────────────────────────────────────────────────────

class TestTopologyInit(unittest.TestCase):

    def test_empty_on_init(self):
        tsg = TopologySemGraph()
        self.assertEqual(len(tsg._nodes), 0)
        self.assertEqual(len(tsg._edges), 0)

    def test_current_room_none(self):
        tsg = TopologySemGraph()
        self.assertEqual(tsg._current_room_id, -1)

    def test_robot_position_none(self):
        tsg = TopologySemGraph()
        self.assertIsNone(tsg._robot_position)


# ─────────────────────────────────────────────────────────────────────────────
#  update_from_scene_graph
# ─────────────────────────────────────────────────────────────────────────────

class TestUpdateFromSceneGraph(unittest.TestCase):

    def test_adds_rooms(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(rooms=[
            _room(0, x=0.0, y=0.0, name="corridor"),
            _room(1, x=5.0, y=0.0, name="office"),
        ])
        tsg.update_from_scene_graph(sg)
        self.assertEqual(len([n for n in tsg._nodes.values() if n.node_type == "room"]), 2)

    def test_room_center_set(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(rooms=[_room(0, x=3.0, y=7.0)])
        tsg.update_from_scene_graph(sg)
        node = tsg._nodes[0]
        np.testing.assert_array_almost_equal(node.center, [3.0, 7.0])

    def test_adds_edges(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(
            rooms=[_room(0), _room(1, x=5.0)],
            topology_edges=[_edge(0, 1)],
        )
        tsg.update_from_scene_graph(sg)
        room_edges = [e for e in tsg._edges
                      if e.from_id in (0, 1) and e.to_id in (0, 1)]
        self.assertGreater(len(room_edges), 0)

    def test_preserves_visit_state(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(rooms=[_room(0)])
        tsg.update_from_scene_graph(sg)
        tsg._nodes[0].visited = True
        tsg._nodes[0].visit_count = 3

        # 再次更新不应丢失 visit 状态
        tsg.update_from_scene_graph(sg)
        self.assertTrue(tsg._nodes[0].visited)
        self.assertEqual(tsg._nodes[0].visit_count, 3)

    def test_removes_stale_rooms(self):
        tsg = TopologySemGraph()
        sg1 = _simple_sg(rooms=[_room(0), _room(1, x=5.0)])
        tsg.update_from_scene_graph(sg1)
        self.assertIn(1, tsg._nodes)

        # room_id=1 不再出现 → 应被移除
        sg2 = _simple_sg(rooms=[_room(0)])
        tsg.update_from_scene_graph(sg2)
        self.assertNotIn(1, tsg._nodes)

    def test_negative_room_id_skipped(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(rooms=[{"room_id": -1, "name": "bad", "center": {"x": 0, "y": 0}}])
        tsg.update_from_scene_graph(sg)
        self.assertEqual(len(tsg._nodes), 0)

    def test_infers_room_type_from_name(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(rooms=[_room(0, name="kitchen")])
        tsg.update_from_scene_graph(sg)
        self.assertEqual(tsg._nodes[0].room_type, "kitchen")

    def test_semantic_labels_truncated_to_12(self):
        tsg = TopologySemGraph()
        labels = [f"label_{i}" for i in range(20)]
        sg = _simple_sg(rooms=[{"room_id": 0, "name": "room_0",
                                  "center": {"x": 0, "y": 0},
                                  "semantic_labels": labels}])
        tsg.update_from_scene_graph(sg)
        self.assertLessEqual(len(tsg._nodes[0].semantic_labels), 12)


# ─────────────────────────────────────────────────────────────────────────────
#  Frontier 节点
# ─────────────────────────────────────────────────────────────────────────────

class TestFrontierNodes(unittest.TestCase):

    def setUp(self):
        self.tsg = TopologySemGraph()
        sg = _simple_sg(rooms=[_room(0, x=0.0, y=0.0)])
        self.tsg.update_from_scene_graph(sg)

    def test_add_frontier_returns_id(self):
        fid = self.tsg.add_frontier(
            position=np.array([3.0, 0.0]),
            direction=np.array([1.0, 0.0]),
            nearest_room_id=0,
        )
        self.assertGreaterEqual(fid, 10000)

    def test_add_frontier_creates_node(self):
        fid = self.tsg.add_frontier(
            position=np.array([3.0, 0.0]),
            direction=np.array([1.0, 0.0]),
            nearest_room_id=0,
        )
        self.assertIn(fid, self.tsg._nodes)
        self.assertEqual(self.tsg._nodes[fid].node_type, "frontier")

    def test_add_frontier_connects_to_room(self):
        fid = self.tsg.add_frontier(
            position=np.array([3.0, 0.0]),
            direction=np.array([1.0, 0.0]),
            nearest_room_id=0,
        )
        adj_ids = [e.to_id for e in self.tsg._adjacency[0]] + \
                  [e.from_id for e in self.tsg._adjacency[0]]
        self.assertIn(fid, adj_ids)

    def test_add_frontier_normalizes_direction(self):
        fid = self.tsg.add_frontier(
            position=np.array([3.0, 0.0]),
            direction=np.array([3.0, 4.0]),  # magnitude = 5
            nearest_room_id=0,
        )
        d = self.tsg._nodes[fid].frontier_direction
        self.assertAlmostEqual(float(np.linalg.norm(d)), 1.0, places=5)

    def test_update_frontiers_clears_old(self):
        self.tsg.add_frontier(np.array([3.0, 0.0]), np.array([1.0, 0.0]), 0)
        self.tsg.add_frontier(np.array([4.0, 0.0]), np.array([1.0, 0.0]), 0)
        # 更新前沿
        new_pts = [np.array([5.0, 0.0]), np.array([6.0, 0.0])]
        new_ids = self.tsg.update_frontiers_from_costmap(new_pts)
        # 原来的前沿应被清除
        all_frontiers = [n for n in self.tsg._nodes.values() if n.node_type == "frontier"]
        self.assertEqual(len(all_frontiers), len(new_ids))

    def test_update_frontiers_skips_too_close(self):
        """距离最近房间 < _MIN_FRONTIER_DISTANCE 的前沿应被跳过。"""
        new_pts = [np.array([0.1, 0.0])]  # 几乎在房间中心
        ids = self.tsg.update_frontiers_from_costmap(new_pts)
        self.assertEqual(len(ids), 0)


# ─────────────────────────────────────────────────────────────────────────────
#  record_robot_position
# ─────────────────────────────────────────────────────────────────────────────

class TestRobotPosition(unittest.TestCase):

    def _setup_two_rooms(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(
            rooms=[
                _room(0, x=0.0, y=0.0, name="corridor"),
                _room(1, x=10.0, y=0.0, name="office"),
            ],
            topology_edges=[_edge(0, 1)],
        )
        tsg.update_from_scene_graph(sg)
        return tsg

    def test_returns_room_id_when_nearby(self):
        tsg = self._setup_two_rooms()
        rid = tsg.record_robot_position(0.5, 0.5)
        self.assertEqual(rid, 0)

    def test_returns_none_when_far_from_all(self):
        tsg = self._setup_two_rooms()
        rid = tsg.record_robot_position(100.0, 100.0)
        self.assertIsNone(rid)

    def test_marks_room_as_visited(self):
        tsg = self._setup_two_rooms()
        tsg.record_robot_position(0.5, 0.5)
        self.assertTrue(tsg._nodes[0].visited)
        self.assertEqual(tsg._nodes[0].visit_count, 1)

    def test_room_transition_recorded(self):
        tsg = self._setup_two_rooms()
        tsg.record_robot_position(0.5, 0.0)   # 进入 room 0
        tsg.record_robot_position(10.0, 0.0)  # 进入 room 1
        self.assertEqual(tsg._current_room_id, 1)
        self.assertGreater(len(tsg._traversal_history), 0)

    def test_trajectory_appended(self):
        tsg = self._setup_two_rooms()
        tsg.record_robot_position(0.0, 0.0)
        tsg.record_robot_position(1.0, 0.0)
        self.assertGreater(len(tsg._robot_trajectory), 0)

    def test_no_rooms_returns_none(self):
        tsg = TopologySemGraph()
        rid = tsg.record_robot_position(0.0, 0.0)
        self.assertIsNone(rid)


# ─────────────────────────────────────────────────────────────────────────────
#  shortest_path
# ─────────────────────────────────────────────────────────────────────────────

class TestShortestPath(unittest.TestCase):

    def _setup_line(self):
        """3 个房间线性连接: 0 --5m-- 1 --5m-- 2"""
        tsg = TopologySemGraph()
        sg = _simple_sg(
            rooms=[
                _room(0, x=0.0),
                _room(1, x=5.0),
                _room(2, x=10.0),
            ],
            topology_edges=[
                {"from_room": 0, "to_room": 1, "type": "door", "distance": 5.0},
                {"from_room": 1, "to_room": 2, "type": "door", "distance": 5.0},
            ],
        )
        tsg.update_from_scene_graph(sg)
        return tsg

    def test_direct_edge(self):
        tsg = self._setup_line()
        cost, path = tsg.shortest_path(0, 1)
        self.assertAlmostEqual(cost, 5.0, places=1)
        self.assertIn(0, path)
        self.assertIn(1, path)

    def test_two_hops(self):
        tsg = self._setup_line()
        cost, path = tsg.shortest_path(0, 2)
        self.assertAlmostEqual(cost, 10.0, places=1)
        self.assertIn(1, path)  # 中间节点在路径中

    def test_unreachable(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(rooms=[_room(0), _room(1, x=5.0)])
        # 无边 → 不可达
        tsg.update_from_scene_graph(sg)
        cost, path = tsg.shortest_path(0, 1)
        self.assertEqual(cost, float("inf"))
        self.assertEqual(path, [])

    def test_same_node(self):
        tsg = self._setup_line()
        cost, _path = tsg.shortest_path(1, 1)
        self.assertEqual(cost, 0.0)

    def test_nonexistent_source(self):
        tsg = self._setup_line()
        cost, _path = tsg.shortest_path(99, 0)
        self.assertEqual(cost, float("inf"))


# ─────────────────────────────────────────────────────────────────────────────
#  hop_distances
# ─────────────────────────────────────────────────────────────────────────────

class TestHopDistances(unittest.TestCase):

    def _setup_line(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(
            rooms=[_room(0), _room(1, x=5.0), _room(2, x=10.0)],
            topology_edges=[
                {"from_room": 0, "to_room": 1, "type": "door", "distance": 5.0},
                {"from_room": 1, "to_room": 2, "type": "door", "distance": 5.0},
            ],
        )
        tsg.update_from_scene_graph(sg)
        return tsg

    def test_direct_neighbor_is_1_hop(self):
        tsg = self._setup_line()
        hops = tsg.hop_distances(0)
        self.assertEqual(hops.get(1), 1)

    def test_two_hops(self):
        tsg = self._setup_line()
        hops = tsg.hop_distances(0)
        self.assertEqual(hops.get(2), 2)

    def test_self_is_0(self):
        tsg = self._setup_line()
        hops = tsg.hop_distances(0)
        self.assertEqual(hops.get(0, 0), 0)

    def test_empty_graph(self):
        tsg = TopologySemGraph()
        hops = tsg.hop_distances(0)
        self.assertEqual(hops, {})

    def test_disconnected_room_not_in_result(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(rooms=[_room(0), _room(1, x=5.0)])
        tsg.update_from_scene_graph(sg)
        hops = tsg.hop_distances(0)
        self.assertNotIn(1, hops)


# ─────────────────────────────────────────────────────────────────────────────
#  get_best_exploration_target
# ─────────────────────────────────────────────────────────────────────────────

class TestGetBestExplorationTarget(unittest.TestCase):

    def test_empty_graph_returns_empty(self):
        tsg = TopologySemGraph()
        result = tsg.get_best_exploration_target("找灭火器")
        self.assertEqual(result, [])

    def test_returns_sorted_by_score(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(
            rooms=[
                _room(0, x=0.0, name="corridor"),
                _room(1, x=5.0, name="office"),
                _room(2, x=10.0, name="kitchen"),
            ],
            topology_edges=[
                {"from_room": 0, "to_room": 1, "type": "door", "distance": 5.0},
                {"from_room": 1, "to_room": 2, "type": "door", "distance": 5.0},
            ],
        )
        tsg.update_from_scene_graph(sg)
        tsg._current_room_id = 0
        tsg._robot_position = np.array([0.0, 0.0])

        results = tsg.get_best_exploration_target("找桌子")
        if results:
            scores = [r.score for r in results]
            self.assertEqual(scores, sorted(scores, reverse=True))

    def test_top_k_respected(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(
            rooms=[_room(i, x=float(i * 5)) for i in range(5)],
            topology_edges=[
                {"from_room": i, "to_room": i + 1, "type": "door", "distance": 5.0}
                for i in range(4)
            ],
        )
        tsg.update_from_scene_graph(sg)
        tsg._current_room_id = 0

        results = tsg.get_best_exploration_target("找椅子", top_k=2)
        self.assertLessEqual(len(results), 2)

    def test_result_has_required_fields(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(
            rooms=[_room(0), _room(1, x=5.0)],
            topology_edges=[{"from_room": 0, "to_room": 1, "type": "door", "distance": 5.0}],
        )
        tsg.update_from_scene_graph(sg)
        tsg._current_room_id = 0

        results = tsg.get_best_exploration_target("找东西")
        if results:
            r = results[0]
            self.assertIsInstance(r, ExplorationTarget)
            self.assertIsNotNone(r.position)
            self.assertGreaterEqual(r.score, 0.0)
            self.assertIsInstance(r.reasoning, str)


# ─────────────────────────────────────────────────────────────────────────────
#  to_prompt_context
# ─────────────────────────────────────────────────────────────────────────────

class TestToPromptContext(unittest.TestCase):

    def test_empty_graph_returns_string(self):
        tsg = TopologySemGraph()
        result = tsg.to_prompt_context()
        self.assertIsInstance(result, str)

    def test_rooms_mentioned(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(rooms=[
            _room(0, name="corridor"),
            _room(1, x=5.0, name="office"),
        ])
        tsg.update_from_scene_graph(sg)
        result = tsg.to_prompt_context()
        self.assertIn("corridor", result)
        self.assertIn("office", result)

    def test_visited_status_shown(self):
        tsg = TopologySemGraph()
        sg = _simple_sg(rooms=[_room(0, name="corridor")])
        tsg.update_from_scene_graph(sg)
        tsg._nodes[0].visited = True
        result = tsg.to_prompt_context()
        # 应该有已访问标记 (✓ 或 visited)
        self.assertTrue("✓" in result or "visited" in result.lower() or "已" in result)


# ─────────────────────────────────────────────────────────────────────────────
#  TopoNode dataclass
# ─────────────────────────────────────────────────────────────────────────────

class TestTopoNodeDataclass(unittest.TestCase):

    def test_hash_by_id(self):
        n1 = TopoNode(0, "room", "r0", np.zeros(2))
        n2 = TopoNode(0, "room", "r0_dup", np.ones(2))
        self.assertEqual(hash(n1), hash(n2))

    def test_equality_by_id(self):
        n1 = TopoNode(5, "room", "r5", np.zeros(2))
        n2 = TopoNode(5, "frontier", "f5", np.ones(2))
        self.assertEqual(n1, n2)

    def test_inequality_different_id(self):
        n1 = TopoNode(0, "room", "r0", np.zeros(2))
        n2 = TopoNode(1, "room", "r1", np.zeros(2))
        self.assertNotEqual(n1, n2)


if __name__ == "__main__":
    unittest.main()
