"""
test_topological_memory.py — 拓扑记忆单元测试

覆盖:
  - 节点创建与合并
  - 边连接
  - 文本查询
  - 回溯
  - 最少探索方向
"""

import unittest

import numpy as np

from memory.spatial.topological import TopologicalMemory


class TestNodeCreation(unittest.TestCase):
    """节点创建与位置更新测试。"""

    def test_first_position_creates_node(self):
        mem = TopologicalMemory(new_node_distance=2.0)
        node = mem.update_position(np.array([0.0, 0.0, 0.0]))
        self.assertIsNotNone(node)
        self.assertEqual(len(mem.nodes), 1)
        self.assertEqual(node.node_id, 0)

    def test_nearby_position_does_not_create_node(self):
        mem = TopologicalMemory(new_node_distance=2.0)
        mem.update_position(np.array([0.0, 0.0, 0.0]))
        result = mem.update_position(np.array([0.5, 0.5, 0.0]))
        self.assertIsNone(result)  # 太近, 不创建新节点
        self.assertEqual(len(mem.nodes), 1)
        # 但 visit_count 应增加
        self.assertEqual(next(iter(mem.nodes.values())).visit_count, 2)

    def test_far_position_creates_new_node(self):
        mem = TopologicalMemory(new_node_distance=2.0)
        mem.update_position(np.array([0.0, 0.0, 0.0]))
        node2 = mem.update_position(np.array([5.0, 5.0, 0.0]))
        self.assertIsNotNone(node2)
        self.assertEqual(len(mem.nodes), 2)

    def test_edge_created_between_consecutive_nodes(self):
        mem = TopologicalMemory(new_node_distance=2.0)
        mem.update_position(np.array([0.0, 0.0, 0.0]))
        mem.update_position(np.array([5.0, 0.0, 0.0]))
        node_0 = mem.nodes[0]
        node_1 = mem.nodes[1]
        self.assertIn(1, node_0.neighbors)
        self.assertIn(0, node_1.neighbors)
        self.assertAlmostEqual(node_0.edge_distances[1], 5.0, places=1)

    def test_visible_labels_stored(self):
        mem = TopologicalMemory(new_node_distance=2.0)
        mem.update_position(
            np.array([0.0, 0.0, 0.0]),
            visible_labels=["chair", "desk"],
        )
        node = next(iter(mem.nodes.values()))
        self.assertIn("chair", node.visible_labels)
        self.assertIn("desk", node.visible_labels)

    def test_labels_merged_on_revisit(self):
        mem = TopologicalMemory(new_node_distance=2.0)
        mem.update_position(np.array([0.0, 0.0, 0.0]), visible_labels=["chair"])
        mem.update_position(np.array([0.5, 0.0, 0.0]), visible_labels=["desk"])
        node = next(iter(mem.nodes.values()))
        self.assertIn("chair", node.visible_labels)
        self.assertIn("desk", node.visible_labels)
        # 不应重复
        self.assertEqual(node.visible_labels.count("chair"), 1)


class TestTextQuery(unittest.TestCase):
    """文本查询匹配测试。"""

    def test_query_matches_visible_labels(self):
        mem = TopologicalMemory(new_node_distance=2.0)
        mem.update_position(
            np.array([0.0, 0.0, 0.0]),
            visible_labels=["fire extinguisher", "door"],
        )
        mem.update_position(
            np.array([10.0, 0.0, 0.0]),
            visible_labels=["chair", "desk"],
        )
        results = mem.query_by_text("fire extinguisher")
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].node_id, 0)

    def test_query_no_match_returns_empty(self):
        mem = TopologicalMemory(new_node_distance=2.0)
        mem.update_position(np.array([0.0, 0.0, 0.0]), visible_labels=["chair"])
        results = mem.query_by_text("elephant")
        self.assertEqual(len(results), 0)


class TestBacktrack(unittest.TestCase):
    """回溯功能测试。"""

    def test_backtrack_one_step(self):
        mem = TopologicalMemory(new_node_distance=2.0)
        mem.update_position(np.array([0.0, 0.0, 0.0]))
        mem.update_position(np.array([5.0, 0.0, 0.0]))
        mem.update_position(np.array([10.0, 0.0, 0.0]))
        pos = mem.get_backtrack_position(steps_back=1)
        self.assertIsNotNone(pos)
        np.testing.assert_allclose(pos[:2], [5.0, 0.0], atol=0.1)

    def test_backtrack_too_far_returns_none(self):
        mem = TopologicalMemory(new_node_distance=2.0)
        mem.update_position(np.array([0.0, 0.0, 0.0]))
        pos = mem.get_backtrack_position(steps_back=5)
        self.assertIsNone(pos)


class TestLeastVisitedDirection(unittest.TestCase):
    """最少探索方向测试。"""

    def test_returns_none_for_empty_memory(self):
        mem = TopologicalMemory()
        result = mem.get_least_visited_direction(np.array([0.0, 0.0]))
        self.assertIsNone(result)

    def test_returns_direction_vector(self):
        mem = TopologicalMemory(new_node_distance=1.0)
        # 在东方添加多个节点
        for x in range(1, 6):
            mem.update_position(np.array([float(x), 0.0, 0.0]))
        direction = mem.get_least_visited_direction(np.array([3.0, 0.0]))
        self.assertIsNotNone(direction)
        self.assertEqual(direction.shape, (2,))
        # 方向应是单位向量级别
        norm = np.linalg.norm(direction)
        self.assertGreater(norm, 0.5)


class TestGraphExport(unittest.TestCase):
    """图导出测试。"""

    def test_graph_json_valid(self):
        import json
        mem = TopologicalMemory(new_node_distance=2.0)
        mem.update_position(np.array([0.0, 0.0, 0.0]), visible_labels=["a"])
        mem.update_position(np.array([5.0, 0.0, 0.0]), visible_labels=["b"])
        graph_str = mem.get_graph_json()
        graph = json.loads(graph_str)
        self.assertEqual(len(graph["nodes"]), 2)
        self.assertEqual(len(graph["edges"]), 1)

    def test_exploration_summary(self):
        mem = TopologicalMemory(new_node_distance=2.0)
        mem.update_position(np.array([0.0, 0.0, 0.0]))
        summary = mem.get_exploration_summary()
        self.assertEqual(summary["total_nodes"], 1)
        self.assertEqual(summary["total_visits"], 1)


class TestPruning(unittest.TestCase):
    """节点上限裁剪测试。"""

    def test_prune_exceeding_max(self):
        mem = TopologicalMemory(new_node_distance=0.5, max_nodes=5)
        for i in range(10):
            mem.update_position(np.array([float(i) * 2, 0.0, 0.0]))
        self.assertLessEqual(len(mem.nodes), 5)


if __name__ == "__main__":
    unittest.main()
