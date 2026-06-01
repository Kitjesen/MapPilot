"""测试 EpisodicMemory"""
import time

import numpy as np

from memory.spatial.episodic import EpisodicMemory


class TestEpisodicMemoryBasic:
    def setup_method(self):
        self.mem = EpisodicMemory()

    def test_add_and_len(self):
        self.mem.add(np.array([0.0, 0.0]), labels=["chair", "table"])
        assert len(self.mem) == 1

    def test_spatial_dedup(self):
        self.mem.add(np.array([0.0, 0.0]), labels=["chair"])
        self.mem.add(np.array([0.5, 0.0]), labels=["table"])  # 距离 < 1m，应跳过
        assert len(self.mem) == 1

    def test_no_dedup_far(self):
        self.mem.add(np.array([0.0, 0.0]), labels=["chair"])
        self.mem.add(np.array([2.0, 0.0]), labels=["table"])  # 距离 > 1m
        assert len(self.mem) == 2

    def test_keyword_search(self):
        self.mem.add(np.array([0.0, 0.0]), labels=["chair", "table"], room_type="餐厅")
        self.mem.add(np.array([5.0, 0.0]), labels=["bed", "pillow"], room_type="卧室")
        results = self.mem.query_by_text("chair", top_k=1)
        assert len(results) == 1
        assert "chair" in results[0].labels

    def test_query_near_position(self):
        self.mem.add(np.array([0.0, 0.0]), labels=["chair"])
        self.mem.add(np.array([10.0, 0.0]), labels=["bed"])
        results = self.mem.query_near_position(np.array([0.5, 0.0]), radius=2.0)
        assert len(results) == 1
        assert "chair" in results[0].labels

    def test_format_for_llm(self):
        self.mem.add(np.array([0.0, 0.0]), labels=["chair"])
        records = self.mem._records
        text = self.mem.format_for_llm(records)
        assert "秒前" in text or "分钟前" in text

    def test_get_summary_empty(self):
        summary = self.mem.get_summary()
        assert summary == ""

    def test_get_summary_nonempty(self):
        self.mem.add(np.array([0.0, 0.0]), labels=["chair"])
        summary = self.mem.get_summary()
        assert "记忆" in summary

    def test_fifo_eviction(self):
        for i in range(510):
            self.mem.add(np.array([float(i * 2), 0.0]), labels=[f"obj{i}"])
        assert len(self.mem) <= EpisodicMemory.MAX_RECORDS

    def test_max_age_filter(self):
        self.mem.add(np.array([0.0, 0.0]), labels=["old_obj"])
        # 手动设置时间戳为过去
        self.mem._records[-1].timestamp = time.time() - 3700
        self.mem.add(np.array([5.0, 0.0]), labels=["new_obj"])
        results = self.mem.query_by_text("obj", max_age_sec=3600)
        labels_in_results = [l for r in results for l in r.labels]
        assert "new_obj" in labels_in_results
        assert "old_obj" not in labels_in_results
