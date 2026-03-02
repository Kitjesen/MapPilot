"""
时空情节记忆（Spatiotemporal Episodic Memory）
参考：ReMEmbR (arXiv:2409.13682) + VLingNav VLingMem
轻量实现：numpy 向量相似度，无外部依赖
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class MemoryRecord:
    timestamp: float          # Unix 时间戳
    position: np.ndarray      # shape (2,) 或 (3,), [x, y] 或 [x, y, z]
    description: str          # 自然语言描述
    labels: List[str]         # 可见标签列表
    room_type: str = ""       # 推测房间类型
    embedding: Optional[np.ndarray] = None  # 文本嵌入（可选）


class EpisodicMemory:
    """
    时空情节记忆，支持：
    - 按文本语义检索（CLIP 嵌入，降级到关键词匹配）
    - 按空间位置检索
    - 按时间过滤
    - FIFO 淘汰策略（最多 MAX_RECORDS 条）
    """

    MAX_RECORDS = 500
    MIN_DISTANCE_M = 1.0   # 同一位置 1m 内不重复记录

    def __init__(self, clip_encoder: object | None = None) -> None:
        self._records: List[MemoryRecord] = []
        self._clip = clip_encoder

    # ---------- 写入 ----------

    def add(
        self,
        position: np.ndarray,
        labels: List[str],
        room_type: str = "",
        description: Optional[str] = None,
    ) -> None:
        """添加一条记忆记录"""
        pos = np.array(position, dtype=float)

        # 空间去重：距最近记录 < MIN_DISTANCE_M 则跳过
        if self._records:
            last_pos = self._records[-1].position
            if np.linalg.norm(pos[:2] - last_pos[:2]) < self.MIN_DISTANCE_M:
                return

        # 生成描述
        unique_labels = list(dict.fromkeys(labels))[:8]
        if description is None:
            label_str = "、".join(unique_labels) if unique_labels else "无可见对象"
            pos_str = f"({pos[0]:.1f}, {pos[1]:.1f})"
            room_str = f"（推测{room_type}）" if room_type else ""
            description = f"位置{pos_str}：{label_str}{room_str}"

        # 文本嵌入
        embedding = None
        if self._clip is not None:
            try:
                embedding = self._clip.encode_text(description)
            except Exception:
                pass

        record = MemoryRecord(
            timestamp=time.time(),
            position=pos,
            description=description,
            labels=unique_labels,
            room_type=room_type,
            embedding=embedding,
        )

        self._records.append(record)

        # FIFO 淘汰
        if len(self._records) > self.MAX_RECORDS:
            self._records = self._records[-self.MAX_RECORDS:]

    # ---------- 检索 ----------

    def query_by_text(
        self,
        query: str,
        top_k: int = 3,
        max_age_sec: Optional[float] = None,
    ) -> List[MemoryRecord]:
        """语义文本检索"""
        candidates = self._filter_by_age(max_age_sec)
        if not candidates:
            return []

        # CLIP 嵌入检索
        if self._clip is not None:
            try:
                q_emb = self._clip.encode_text(query)
                scores = []
                for r in candidates:
                    if r.embedding is not None:
                        sim = float(np.dot(q_emb, r.embedding) /
                                    (np.linalg.norm(q_emb) * np.linalg.norm(r.embedding) + 1e-9))
                    else:
                        sim = self._keyword_score(query, r)
                    scores.append(sim)
                ranked = sorted(zip(scores, candidates), key=lambda x: -x[0])
                return [r for _, r in ranked[:top_k]]
            except Exception:
                pass

        # 降级：关键词匹配
        scored = [(self._keyword_score(query, r), r) for r in candidates]
        scored.sort(key=lambda x: -x[0])
        return [r for _, r in scored[:top_k]]

    def query_near_position(
        self,
        position: np.ndarray,
        radius: float = 3.0,
        top_k: int = 5,
    ) -> List[MemoryRecord]:
        """空间范围检索"""
        pos = np.array(position, dtype=float)
        results = []
        for r in self._records:
            dist = float(np.linalg.norm(pos[:2] - r.position[:2]))
            if dist <= radius:
                results.append((dist, r))
        results.sort(key=lambda x: x[0])
        return [r for _, r in results[:top_k]]

    # ---------- 格式化 ----------

    def format_for_llm(self, records: List[MemoryRecord]) -> str:
        """格式化为 LLM 可读字符串"""
        if not records:
            return "（无相关历史记忆）"
        lines = []
        now = time.time()
        for r in records:
            age_sec = now - r.timestamp
            if age_sec < 60:
                age_str = f"{int(age_sec)}秒前"
            elif age_sec < 3600:
                age_str = f"{int(age_sec/60)}分钟前"
            else:
                age_str = f"{int(age_sec/3600)}小时前"
            lines.append(f"- {age_str}：{r.description}")
        return "\n".join(lines)

    def get_summary(self) -> str:
        """返回近期记忆摘要（用于 Slow Path prompt）"""
        recent = self._records[-10:]  # 最近10条
        if not recent:
            return ""
        summaries = [f"  {r.description}" for r in recent[-5:]]
        return "【近期探索记忆】\n" + "\n".join(summaries)

    def __len__(self) -> int:
        return len(self._records)

    # ---------- 内部 ----------

    def _filter_by_age(self, max_age_sec: Optional[float]) -> List[MemoryRecord]:
        if max_age_sec is None:
            return list(self._records)
        cutoff = time.time() - max_age_sec
        return [r for r in self._records if r.timestamp >= cutoff]

    def _keyword_score(self, query: str, record: MemoryRecord) -> float:
        query_words = set(query.lower().split())
        label_words = set(l.lower() for l in record.labels)
        desc_words = set(record.description.lower().split())
        all_words = label_words | desc_words
        if not all_words:
            return 0.0
        overlap = len(query_words & all_words)
        return overlap / (len(query_words) + 1e-9)
