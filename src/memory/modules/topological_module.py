"""TopologicalMemoryModule — 拓扑记忆模块 (Module 模式封装)。

将 TopologicalMemory 封装为 Module In/Out 端口接口，
记录机器人探索轨迹，输出摘要供 LLM 和前端消费。

层级: L3 (Perception)
输入: scene_graph, odometry
输出: topo_summary (已探索区域摘要), topo_graph (完整图 JSON)
"""

from __future__ import annotations

import json
import logging
from typing import Any

from core import In, Module, Out
from core.msgs import Odometry, SceneGraph
from core.msgs.numpy_compat import np
from core.registry import register
from memory.modules._odom_mixin import OdomTrackingMixin
from memory.spatial.topological import TopologicalMemory

logger = logging.getLogger(__name__)


@register("memory", "topological", description="Topological memory graph tracking explored regions and connectivity")
class TopologicalMemoryModule(OdomTrackingMixin, Module, layer=3):
    """拓扑记忆模块。

    每次收到里程计和场景图更新时，维护拓扑记忆图。
    发布已探索区域摘要和完整拓扑图。

    Config:
        new_node_distance: 新建节点的最小移动距离 (default 2.0)
        max_nodes: 最大节点数 (default 500)
    """

    # -- 端口声明 --
    scene_graph: In[SceneGraph]
    odometry: In[Odometry]

    topo_summary: Out[str]
    topo_graph: Out[dict]

    def __init__(self, **config: Any) -> None:
        mem_kwargs = {}
        for key in ("new_node_distance", "max_nodes"):
            if key in config:
                mem_kwargs[key] = config.pop(key)

        super().__init__(**config)
        self._memory = TopologicalMemory(**mem_kwargs)
        self._last_sg: SceneGraph | None = None

    # -- 生命周期 --

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.scene_graph.subscribe(self._on_scene_graph)

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        self._last_sg = sg
        self._update()

    # -- 核心逻辑 --

    def _update(self) -> None:
        """用当前里程计和场景图更新拓扑记忆。"""
        if self._last_odom is None:
            return

        odom = self._last_odom
        position = np.array([odom.x, odom.y, odom.z], dtype=np.float64)

        # 从场景图提取可见标签
        visible_labels = []
        if self._last_sg is not None:
            visible_labels = [obj.label for obj in self._last_sg.objects]

        # 更新拓扑记忆
        new_node = self._memory.update_position(
            position=position,
            visible_labels=visible_labels,
        )

        # 如果创建了新节点，更新区域摘要并发布
        if new_node is not None:
            self._memory.update_region_summary(
                node_id=new_node.node_id,
                visible_labels=visible_labels,
            )

        # 发布摘要
        summaries = self._memory.get_explored_summaries()
        summary_text = "\n".join(summaries) if summaries else ""
        self.topo_summary.publish(summary_text)

        # 发布拓扑图
        graph_json = self._memory.get_graph_json()
        try:
            graph_dict = json.loads(graph_json)
        except (json.JSONDecodeError, TypeError):
            graph_dict = {}
        self.topo_graph.publish(graph_dict)

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["node_count"] = getattr(self._memory, "node_count", len(getattr(self._memory, "_nodes", [])))
        info["edge_count"] = getattr(self._memory, "edge_count", len(getattr(self._memory, "_edges", [])))
        return info

    # -- 外部 API --

    @property
    def memory(self) -> TopologicalMemory:
        """访问内部 TopologicalMemory (测试/查询用)。"""
        return self._memory
