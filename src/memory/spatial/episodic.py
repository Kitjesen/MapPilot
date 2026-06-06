"""
时空情节记忆（Spatiotemporal Episodic Memory）
参考：ReMEmbR (arXiv:2409.13682) + VLingNav VLingMem
轻量实现：numpy 向量相似度，无外部依赖

持久化：pass persist_path= to EpisodicMemory() to enable SQLite backend.
Records survive process restarts and are hydrated on startup.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from core.msgs.numpy_compat import np

logger = logging.getLogger(__name__)


@dataclass
class MemoryRecord:
    timestamp: float          # Unix 时间戳
    position: np.ndarray      # shape (2,) 或 (3,), [x, y] 或 [x, y, z]
    description: str          # 自然语言描述
    labels: list[str]         # 可见标签列表
    room_type: str = ""       # 推测房间类型
    embedding: np.ndarray | None = None  # 文本嵌入（可选）


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

    def __init__(
        self,
        clip_encoder: object | None = None,
        persist_path: str | Path | None = None,
    ) -> None:
        self._records: list[MemoryRecord] = []
        self._clip = clip_encoder
        self._lock = threading.Lock()
        self._store: SqliteEpisodicStore | None = None

        if persist_path is not None:
            from memory.spatial.episodic_store import SqliteEpisodicStore
            self._store = SqliteEpisodicStore(persist_path)
            self._store.open()
            # Hydrate in-memory cache from persisted DB (up to MAX_RECORDS)
            with self._lock:
                self._records = self._store.load_recent(self.MAX_RECORDS)
            logger.info(
                "EpisodicMemory: loaded %d records from %s",
                len(self._records), persist_path,
            )

    # ---------- 写入 ----------

    def add(
        self,
        position: np.ndarray,
        labels: list[str],
        room_type: str = "",
        description: str | None = None,
    ) -> None:
        """添加一条记忆记录"""
        pos = np.array(position, dtype=float)

        # 位置验证：拒绝 NaN/Inf 位置
        if not np.isfinite(pos[:2]).all():
            logger.debug("EpisodicMemory.add: rejected non-finite position %s", pos)
            return

        # 空间去重（快速检查，持锁时间短）
        with self._lock:
            if self._records:
                last_pos = self._records[-1].position
                if np.linalg.norm(pos[:2] - last_pos[:2]) < self.MIN_DISTANCE_M:
                    return

        # 生成描述（锁外，可能耗时）
        unique_labels = list(dict.fromkeys(labels))[:8]
        if description is None:
            label_str = "、".join(unique_labels) if unique_labels else "无可见对象"
            pos_str = f"({pos[0]:.1f}, {pos[1]:.1f})"
            room_str = f"（推测{room_type}）" if room_type else ""
            description = f"位置{pos_str}：{label_str}{room_str}"

        # 文本嵌入（锁外，CLIP 调用可能耗时）
        embedding = None
        if self._clip is not None:
            try:
                embedding = self._clip.encode_text(description)
            except (ValueError, TypeError, AttributeError) as e:
                logger.debug("CLIP encode_text failed: %s", e)

        record = MemoryRecord(
            timestamp=time.time(),
            position=pos,
            description=description,
            labels=unique_labels,
            room_type=room_type,
            embedding=embedding,
        )

        with self._lock:
            self._records.append(record)
            # FIFO 淘汰
            if len(self._records) > self.MAX_RECORDS:
                self._records = self._records[-self.MAX_RECORDS:]

        # Persist outside the lock — SQLite write is independently thread-safe
        if self._store is not None:
            self._store.save(record)
            if len(self._records) >= self.MAX_RECORDS:
                # Flush buffer before pruning so prune has actual rows to delete
                self._store.flush()
                self._store.prune_oldest(self.MAX_RECORDS)

    # ---------- 检索 ----------

    def query_by_text(
        self,
        query: str,
        top_k: int = 3,
        max_age_sec: float | None = None,
    ) -> list[MemoryRecord]:
        """语义文本检索"""
        with self._lock:
            candidates = self._filter_by_age(max_age_sec)
        if not candidates:
            return []

        # CLIP 嵌入检索 — vectorized: pre-normalize q_emb, batch matmul for all embeddings
        if self._clip is not None:
            try:
                q_emb = self._clip.encode_text(query)
                q_norm = float(np.linalg.norm(q_emb))
                if q_norm < 1e-9:
                    raise ValueError("zero query embedding")
                q_unit = q_emb / q_norm

                # Split candidates into those with/without embeddings
                with_emb = [(i, r) for i, r in enumerate(candidates) if r.embedding is not None]
                scores = [0.0] * len(candidates)

                if with_emb:
                    idx, recs = zip(*with_emb)
                    mat = np.stack([r.embedding for r in recs])          # (K, D)
                    norms = np.linalg.norm(mat, axis=1, keepdims=True) + 1e-9
                    sims = (mat / norms) @ q_unit                        # (K,) — one matmul
                    for j, i in enumerate(idx):
                        scores[i] = float(sims[j])

                # Keyword fallback for records without embeddings
                for i, r in enumerate(candidates):
                    if r.embedding is None:
                        scores[i] = self._keyword_score(query, r)

                ranked = sorted(zip(scores, candidates), key=lambda x: -x[0])
                return [r for _, r in ranked[:top_k]]
            except (ValueError, TypeError, AttributeError) as e:
                logger.debug("CLIP embedding query failed, falling back to keywords: %s", e)

        # 降级：关键词匹配
        scored = [(self._keyword_score(query, r), r) for r in candidates]
        scored.sort(key=lambda x: -x[0])
        return [r for _, r in scored[:top_k]]

    def query_near_position(
        self,
        position: np.ndarray,
        radius: float = 3.0,
        top_k: int = 5,
    ) -> list[MemoryRecord]:
        """空间范围检索 — vectorized: single np.linalg.norm call over (N,2) matrix."""
        pos = np.array(position, dtype=float)
        if not np.isfinite(pos[:2]).all():
            return []
        with self._lock:
            if not self._records:
                return []
            records_snap = list(self._records)
            positions = np.array([r.position[:2] for r in records_snap])  # (N, 2)

        dists = np.linalg.norm(positions - pos[:2], axis=1)  # one vectorized call
        in_radius = np.where(dists <= radius)[0]
        if len(in_radius) == 0:
            return []
        top_idx = in_radius[np.argsort(dists[in_radius])][:top_k]
        return [records_snap[i] for i in top_idx]

    # ---------- 格式化 ----------

    def format_for_llm(self, records: list[MemoryRecord]) -> str:
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
        with self._lock:
            recent = self._records[-5:]
        if not recent:
            return ""
        summaries = [f"  {r.description}" for r in recent]
        return "【近期探索记忆】\n" + "\n".join(summaries)

    def recent_n(self, n: int = 10) -> list[MemoryRecord]:
        """Return the last *n* records (oldest first within the slice)."""
        if n <= 0:
            return []
        with self._lock:
            return list(self._records[-n:])

    def close(self) -> None:
        """Flush and close the SQLite store (no-op if no persist_path)."""
        if self._store is not None:
            self._store.close()
            self._store = None

    def __len__(self) -> int:
        with self._lock:
            return len(self._records)

    # ---------- 内部 ----------

    def _filter_by_age(self, max_age_sec: float | None) -> list[MemoryRecord]:
        if max_age_sec is None:
            return list(self._records)
        cutoff = time.time() - max_age_sec
        return [r for r in self._records if r.timestamp >= cutoff]

    def _keyword_score(self, query: str, record: MemoryRecord) -> float:
        query_words = set(query.lower().split())
        label_words = set(label.lower() for label in record.labels)
        desc_words = set(record.description.lower().split())
        all_words = label_words | desc_words
        if not all_words:
            return 0.0
        overlap = len(query_words & all_words)
        return overlap / (len(query_words) + 1e-9)
