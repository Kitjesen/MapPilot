"""Room inference — semantic room naming and room-level reasoning.

This module was split out of ``memory.spatial.room_manager`` during the
perception Phase 3 refactor.  It consumes structural scene-graph elements
(regions, groups, objects) produced by ``perception.tracking.scene_graph_builder``
and adds room-level semantic inference:
  - room node construction
  - LLM-based room naming (when stable)
  - room CLIP-feature aggregation
  - room embedding queries

The dependency direction is reversed compared to the old design:
``memory`` consumes scene-graph data instead of ``perception`` importing a
memory-side mixin.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections.abc import Callable
from typing import TYPE_CHECKING

import numpy as np

from runtime.msgs.scene import (
    ROOM_NAMING_STABILITY_COUNT,
    ROOM_NAMING_STABILITY_SEC,
    RoomNode,
)

if TYPE_CHECKING:
    from perception.tracking.tracked_objects import TrackedObject
    from runtime.msgs.scene import GroupNode, Region

logger = logging.getLogger(__name__)


class RoomInferencer:
    """Infer semantic room nodes from scene-graph regions and groups."""

    _LLM_NAMING_MAX_CONCURRENT = 3
    _LLM_NAMING_TIMEOUT_S = 15.0

    def __init__(self) -> None:
        self._room_llm_namer: Callable | None = None
        self._room_name_cache: dict[int, str] = {}
        self._last_rooms: list[RoomNode] = []
        self._llm_pending_tasks: dict[int, asyncio.Task] = {}
        self._region_stability: dict[int, tuple[frozenset, float]] = {}

    def set_room_namer(self, namer: Callable) -> None:
        """Register async LLM room-naming callback: namer(labels) -> name."""
        self._room_llm_namer = namer

    @property
    def room_name_cache(self) -> dict[int, str]:
        return self._room_name_cache

    @property
    def last_rooms(self) -> list[RoomNode]:
        return self._last_rooms

    def compute_rooms(
        self,
        objects: dict[int, TrackedObject],
        regions: list[Region],
        groups: list[GroupNode],
    ) -> list[RoomNode]:
        """Convert regions into semantic RoomNode instances.

        Stable regions (enough objects, unchanged for a threshold duration) may
        trigger an async LLM naming task.  The result is cached and applied on
        subsequent calls.
        """
        room_to_groups: dict[int, list[int]] = {}
        for g in groups:
            room_to_groups.setdefault(g.room_id, []).append(g.group_id)

        now = time.time()
        rooms: list[RoomNode] = []
        for r in regions:
            labels = [objects[oid].label for oid in r.object_ids if oid in objects]

            room_name = r.name if r.name else f"room_{r.region_id}"
            is_llm_named = r.llm_named

            if r.region_id in self._room_name_cache:
                room_name = self._room_name_cache[r.region_id]
                is_llm_named = True
            elif self._room_llm_namer and not is_llm_named:
                current_set = frozenset(r.object_ids)
                prev = self._region_stability.get(r.region_id)
                if prev is not None and prev[0] == current_set:
                    stable_duration = now - prev[1]
                else:
                    self._region_stability[r.region_id] = (current_set, now)
                    stable_duration = 0.0

                if len(r.object_ids) >= ROOM_NAMING_STABILITY_COUNT and stable_duration >= ROOM_NAMING_STABILITY_SEC:
                    self._trigger_room_llm_naming(r.region_id, labels)

            room_clip = None
            feat_count = 0
            alpha = 0.3
            for oid in r.object_ids:
                obj = objects.get(oid)
                if obj is None or obj.features.size == 0:
                    continue
                f = np.asarray(obj.features, dtype=np.float64)
                f_norm = np.linalg.norm(f)
                if f_norm == 0:
                    continue
                f = f / f_norm
                if room_clip is None:
                    room_clip = f.copy()
                else:
                    room_clip = alpha * f + (1.0 - alpha) * room_clip
                feat_count += 1

            if room_clip is not None:
                norm = np.linalg.norm(room_clip)
                if norm > 0:
                    room_clip = room_clip / norm

            rooms.append(
                RoomNode(
                    room_id=r.region_id,
                    name=room_name,
                    center=r.center.copy(),
                    object_ids=list(r.object_ids),
                    group_ids=room_to_groups.get(r.region_id, []),
                    semantic_labels=labels,
                    llm_named=is_llm_named,
                    clip_feature=room_clip,
                    feature_count=feat_count,
                )
            )

        self._last_rooms = rooms
        return rooms

    def query_rooms_by_embedding(
        self,
        rooms: list[RoomNode],
        embedding: np.ndarray,
        top_k: int = 3,
    ) -> list[tuple[RoomNode, float]]:
        """Return the top-k rooms most similar to a CLIP embedding."""
        if not rooms:
            return []

        q = np.asarray(embedding, dtype=np.float64).ravel()
        q_norm = np.linalg.norm(q)
        if q_norm == 0:
            return []
        q = q / q_norm

        scored: list[tuple[RoomNode, float]] = []
        for room in rooms:
            if room.clip_feature is None:
                continue
            score = float(np.dot(q, room.clip_feature))
            scored.append((room, score))

        scored.sort(key=lambda x: x[1], reverse=True)
        return scored[:top_k]

    def _trigger_room_llm_naming(self, region_id: int, labels: list[str]) -> None:
        """Async LLM room naming with concurrency limit and timeout."""
        if region_id in self._room_name_cache:
            return
        if len(self._llm_pending_tasks) >= self._LLM_NAMING_MAX_CONCURRENT:
            return
        self._room_name_cache[region_id] = f"naming_room_{region_id}"

        async def _do_name():
            try:
                name = await asyncio.wait_for(
                    self._room_llm_namer(labels),
                    timeout=self._LLM_NAMING_TIMEOUT_S,
                )
                if name and isinstance(name, str) and len(name.strip()) > 0:
                    self._room_name_cache[region_id] = name.strip()
                    logger.info("LLM named room %d -> '%s' (labels: %s)", region_id, name.strip(), labels[:6])
                else:
                    del self._room_name_cache[region_id]
            except asyncio.TimeoutError:
                logger.warning("LLM room naming timed out for region %d", region_id)
                if region_id in self._room_name_cache:
                    del self._room_name_cache[region_id]
            except Exception as e:
                logger.warning("LLM room naming failed for region %d: %s", region_id, e)
                if region_id in self._room_name_cache:
                    del self._room_name_cache[region_id]
            finally:
                self._llm_pending_tasks.pop(region_id, None)

        try:
            loop = asyncio.get_running_loop()
            task = loop.create_task(_do_name())
            self._llm_pending_tasks[region_id] = task
        except RuntimeError:
            if region_id in self._room_name_cache:
                del self._room_name_cache[region_id]
