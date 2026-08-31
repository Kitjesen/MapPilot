"""EpisodicMemoryModule — 时空情节记忆模块 (Module 模式封装)。

将 EpisodicMemory 封装为 Module In/Out 端口接口，
记录探索历程，输出格式化记忆上下文供 LLM prompt 使用。

层级: L3 (Perception)
输入: scene_graph, odometry
输出: memory_context (格式化 LLM 上下文字符串)
"""

from __future__ import annotations

import json
import logging
from typing import Any

from runtime import In, Module, Out, skill
from runtime.msgs import Odometry, SceneGraph
from runtime.msgs.numpy_compat import np
from runtime.registry import register
from memory.modules._odom_mixin import OdomTrackingMixin
from memory.spatial.episodic import EpisodicMemory

logger = logging.getLogger(__name__)


@register("memory", "episodic", description="Spatio-temporal episodic memory recording exploration history")
class EpisodicMemoryModule(OdomTrackingMixin, Module, layer=3):
    """时空情节记忆模块。

    每次收到场景图更新时，将当前位置和可见标签记录到情节记忆中。
    发布最近记忆的格式化文本，可直接嵌入 LLM prompt。

    Config:
        max_records: 最大记录数 (default 500)
        min_distance_m: 同位置去重距离 (default 1.0)
    """

    # -- 端口声明 --
    scene_graph: In[SceneGraph]
    odometry: In[Odometry]

    memory_context: Out[str]

    def __init__(self, **config: Any) -> None:
        max_records = config.pop("max_records", None)
        min_distance_m = config.pop("min_distance_m", None)
        super().__init__(**config)

        self._memory = EpisodicMemory()
        if max_records is not None:
            self._memory.MAX_RECORDS = max_records
        if min_distance_m is not None:
            self._memory.MIN_DISTANCE_M = min_distance_m

    # -- 生命周期 --

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.scene_graph.subscribe(self._on_scene_graph)

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        if self._last_odom is None:
            return

        odom = self._last_odom
        position = np.array([odom.x, odom.y, odom.z], dtype=np.float64)
        labels = [obj.label for obj in sg.objects]

        # 添加到情节记忆
        self._memory.add(
            position=position,
            labels=labels,
        )

        # 发布格式化上下文
        summary = self._memory.get_summary()
        self.memory_context.publish(summary)

    # -- 外部 API --

    @property
    def memory(self) -> EpisodicMemory:
        """访问内部 EpisodicMemory (测试/查询用)。"""
        return self._memory

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        records = self._memory.records if hasattr(self._memory, "records") else getattr(self._memory, "_records", [])
        info["record_count"] = len(records)
        info["oldest_ts"] = records[0].timestamp if records else None
        return info

    @skill
    def get_recent_observations(self, count: int = 10) -> str:
        """Return the most recent episodic observations (labels + position)."""
        raw = self._memory.recent_n(count)
        out = []
        for r in raw:
            pos = r.position.tolist() if hasattr(r.position, "tolist") else list(r.position)
            out.append({
                "timestamp": r.timestamp,
                "labels": r.labels,
                "position": pos,
                "description": r.description,
            })
        return json.dumps({"observations": out}, ensure_ascii=False)
