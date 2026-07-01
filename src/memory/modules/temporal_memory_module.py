"""TemporalMemoryModule — Time-indexed scene memory for temporal queries.

Records what was seen, where, and when. Enables natural language queries
like "what did you see 5 minutes ago?" or "when did you last see a person?".

Layer: L3 (Memory/Perception)
Inputs:  scene_graph (SceneGraph), odometry (Odometry)
Outputs: memory_context (str), temporal_summary (dict)
"""

from __future__ import annotations

import json
import logging
import os
import queue
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any


from runtime.module import Module, skill
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import SceneGraph
from runtime.registry import register
from runtime.stream import In, Out
from memory.modules._odom_mixin import OdomTrackingMixin
from memory.storage.temporal_store import TemporalStore

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class TemporalRecord:
    """A single time-indexed scene observation."""

    timestamp: float
    position: tuple  # (x, y, z)
    objects: list[dict[str, Any]]  # [{label, confidence, x, y, z}, ...]
    room: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "position": list(self.position),
            "objects": self.objects,
            "room": self.room,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> TemporalRecord:
        pos = d.get("position", [0.0, 0.0, 0.0])
        return cls(
            timestamp=float(d.get("timestamp", 0.0)),
            position=(float(pos[0]), float(pos[1]), float(pos[2])),
            objects=d.get("objects", []),
            room=d.get("room", ""),
        )


# ---------------------------------------------------------------------------
# Module
# ---------------------------------------------------------------------------

@register("memory", "temporal", description="Temporal scene memory with time-indexed observations")
class TemporalMemoryModule(OdomTrackingMixin, Module, layer=3):
    """Time-indexed scene memory module.

    Maintains a circular buffer of scene observations indexed by time.
    Exposes @skill methods for natural language temporal queries and
    publishes formatted context strings for LLM prompts.

    Config:
        max_records (int):              Circular buffer capacity. Default 1000.
        summary_interval (float):       Seconds between summary regeneration. Default 30.0.
        min_observation_interval (float): Minimum seconds between stored records
                                          (de-duplicate near-identical frames). Default 1.0.
        save_dir (str):                 If set, append records to a JSONL file and
                                        load existing records on start. Default "".
    """

    # -- Ports -----------------------------------------------------------------
    scene_graph: In[SceneGraph]
    odometry: In[Odometry]

    memory_context: Out[str]
    temporal_summary: Out[dict]

    # -- Init ------------------------------------------------------------------

    def __init__(
        self,
        max_records: int = 1000,
        summary_interval: float = 30.0,
        min_observation_interval: float = 1.0,
        save_dir: str = "",
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._max_records = max_records
        self._summary_interval = summary_interval
        self._min_observation_interval = min_observation_interval
        self._save_dir = save_dir

        self._buffer: deque[TemporalRecord] = deque(maxlen=max_records)
        self._lock = threading.Lock()

        self._last_record_ts: float = 0.0
        self._last_summary_ts: float = 0.0

        self._summary_cache: str = ""
        self._summary_timer: threading.Timer | None = None

        self._jsonl_path: str = ""
        self._store: TemporalStore | None = None

        # Async SQLite write queue — prevents high-freq perception callbacks
        # from blocking on disk I/O.  Background thread drains the queue.
        self._write_queue: queue.Queue = queue.Queue(maxsize=2000)
        self._write_thread: threading.Thread | None = None

    # -- Lifecycle -------------------------------------------------------------

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.scene_graph.subscribe(self._on_scene_graph)

        if self._save_dir:
            os.makedirs(self._save_dir, exist_ok=True)
            self._jsonl_path = os.path.join(self._save_dir, "temporal_memory.jsonl")
            self._load_from_disk()
            db_path = os.path.join(self._save_dir, "temporal_memory.db")
            try:
                self._store = TemporalStore(db_path)
            except Exception as exc:
                logger.warning("TemporalMemory: could not open SQLite store: %s", exc)

    def start(self) -> None:
        self._schedule_summary()
        if self._store is not None:
            self._write_thread = threading.Thread(
                target=self._write_worker, daemon=True, name="TemporalStore-writer"
            )
            self._write_thread.start()

    def stop(self) -> None:
        if self._summary_timer is not None:
            self._summary_timer.cancel()
            self._summary_timer = None
        # Signal writer thread to drain and exit
        self._write_queue.put(None)
        if self._write_thread is not None:
            self._write_thread.join(timeout=3.0)
            self._write_thread = None

    def _write_worker(self) -> None:
        """Background thread: drain the write queue into SQLite."""
        while True:
            item = self._write_queue.get()
            if item is None:  # sentinel → exit
                break
            if self._store is None:
                continue
            try:
                self._store.insert(**item)
            except Exception as exc:
                logger.debug("TemporalStore async write failed: %s", exc)

    # -- Port callbacks --------------------------------------------------------

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        now = time.time()

        # De-duplicate: skip if too soon since last record
        if now - self._last_record_ts < self._min_observation_interval:
            return

        odom = self._last_odom
        if odom is None:
            pos = (0.0, 0.0, 0.0)
        else:
            pos = (float(odom.x), float(odom.y), float(odom.z))

        objects = [
            {
                "label": obj.label,
                "confidence": round(float(obj.confidence), 3),
                "x": round(float(obj.position.x), 3),
                "y": round(float(obj.position.y), 3),
                "z": round(float(obj.position.z), 3),
            }
            for obj in sg.objects
            if obj.label
        ]

        record = TemporalRecord(
            timestamp=now,
            position=pos,
            objects=objects,
        )

        with self._lock:
            self._buffer.append(record)
            self._last_record_ts = now

        if self._jsonl_path:
            self._append_to_disk(record)

        # Enqueue per-entity SQLite writes (non-blocking; background thread drains)
        if self._store is not None:
            robot_x, robot_y = float(pos[0]), float(pos[1])
            for sg_obj in sg.objects:
                if not sg_obj.label:
                    continue
                item: dict[str, Any] = dict(
                    label=sg_obj.label,
                    confidence=round(float(sg_obj.confidence), 3),
                    pos_x=round(float(sg_obj.position.x), 3),
                    pos_y=round(float(sg_obj.position.y), 3),
                    robot_x=robot_x,
                    robot_y=robot_y,
                    ts=now,
                    embedding=getattr(sg_obj, "clip_feature", None),
                )
                try:
                    self._write_queue.put_nowait(item)
                except queue.Full:
                    logger.debug("TemporalStore write queue full, dropping observation")

        # Publish incremental context
        context = self._format_record(record)
        self.memory_context.publish(context)

    # -- Summary scheduling ----------------------------------------------------

    def _schedule_summary(self) -> None:
        self._summary_timer = threading.Timer(
            self._summary_interval, self._run_summary_tick
        )
        self._summary_timer.daemon = True
        self._summary_timer.start()

    def _run_summary_tick(self) -> None:
        self._update_summary()
        self._schedule_summary()

    def _update_summary(self) -> None:
        """Regenerate and publish the rolling summary of recent observations."""
        now = time.time()
        summary_text = self.get_timeline(minutes=5)

        with self._lock:
            self._summary_cache = summary_text
            self._last_summary_ts = now

        object_counts: dict[str, int] = {}
        with self._lock:
            records = list(self._buffer)
        for rec in records:
            for obj in rec.objects:
                lbl = obj["label"]
                object_counts[lbl] = object_counts.get(lbl, 0) + 1

        summary_dict = {
            "total_records": len(records),
            "time_range_seconds": (now - records[0].timestamp) if records else 0.0,
            "object_counts": object_counts,
            "timeline_text": summary_text,
            "updated_at": now,
        }
        self.temporal_summary.publish(summary_dict)

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        with self._lock:
            info["buffer_size"] = len(self._buffer)
        info["timer_active"] = self._summary_timer is not None and self._summary_timer.is_alive()
        return info

    # -- Query methods ---------------------------------------------------------

    def query_by_time(self, start_t: float, end_t: float) -> list[TemporalRecord]:
        """Return all records whose timestamp falls within [start_t, end_t]."""
        with self._lock:
            return [r for r in self._buffer if start_t <= r.timestamp <= end_t]

    def query_by_label(self, label: str) -> list[TemporalRecord]:
        """Return all records that contain at least one object with the given label."""
        label_lower = label.lower()
        with self._lock:
            return [
                r for r in self._buffer
                if any(o["label"].lower() == label_lower for o in r.objects)
            ]

    def query_last_seen(self, label: str) -> TemporalRecord | None:
        """Return the most recent record containing an object with the given label."""
        label_lower = label.lower()
        with self._lock:
            records = list(self._buffer)
        for rec in reversed(records):
            if any(o["label"].lower() == label_lower for o in rec.objects):
                return rec
        return None

    def get_timeline(self, minutes: float = 5) -> str:
        """Return a human-readable timeline of the last N minutes."""
        cutoff = time.time() - minutes * 60.0
        with self._lock:
            recent = [r for r in self._buffer if r.timestamp >= cutoff]

        if not recent:
            return f"No observations in the last {minutes:.0f} minutes."

        lines = [f"Timeline — last {minutes:.0f} min ({len(recent)} observations):"]
        for rec in recent:
            age_s = time.time() - rec.timestamp
            age_str = _format_age(age_s)
            labels = list({o["label"] for o in rec.objects})
            obj_str = ", ".join(labels) if labels else "nothing detected"
            pos = rec.position
            lines.append(
                f"  [{age_str}] at ({pos[0]:.1f}, {pos[1]:.1f}) — {obj_str}"
            )
        return "\n".join(lines)

    # -- @skill methods --------------------------------------------------------

    @skill
    def query_temporal(self, question: str) -> str:
        """Answer a natural language question about temporal scene memory.

        Supports patterns like:
          - "what did you see 5 minutes ago?"
          - "when did you last see a person?"
          - "what was visible 10 minutes ago?"
        """
        question_lower = question.lower()

        # Pattern: "last seen <label>" or "when did you last see <label>"
        m = re.search(r"last\s+see(?:n)?\s+(\w+)", question_lower)
        if m:
            return self.get_entity_history(m.group(1))

        # Pattern: "X minutes ago"
        m = re.search(r"(\d+(?:\.\d+)?)\s+minutes?\s+ago", question_lower)
        if m:
            minutes_ago = float(m.group(1))
            now = time.time()
            window_start = now - (minutes_ago + 1) * 60.0
            window_end = now - (minutes_ago - 1) * 60.0
            records = self.query_by_time(window_start, window_end)
            if not records:
                return f"No observations found around {minutes_ago:.0f} minutes ago."
            labels = list({o["label"] for rec in records for o in rec.objects})
            label_str = ", ".join(labels) if labels else "nothing detected"
            return (
                f"Around {minutes_ago:.0f} minutes ago I saw: {label_str}."
            )

        # Pattern: "last N minutes" / "past N minutes"
        m = re.search(r"(?:last|past)\s+(\d+(?:\.\d+)?)\s+minutes?", question_lower)
        if m:
            return self.get_timeline(minutes=float(m.group(1)))

        # Fallback: return the last 5-minute timeline
        return self.get_timeline(minutes=5)

    @skill
    def get_entity_history(self, label: str) -> str:
        """Return a formatted summary of all sightings of a specific object label."""
        records = self.query_by_label(label)
        if not records:
            return f"I have not seen any '{label}' in my memory."

        last = records[-1]
        age_str = _format_age(time.time() - last.timestamp)
        pos = last.position

        lines = [
            f"'{label}' sighted {len(records)} time(s).",
            f"Last seen {age_str} at ({pos[0]:.1f}, {pos[1]:.1f}).",
        ]
        if len(records) > 1:
            first = records[0]
            span_s = last.timestamp - first.timestamp
            lines.append(f"Observation span: {_format_age(span_s)}.")

        return " ".join(lines)

    # -- Persistence -----------------------------------------------------------

    def _append_to_disk(self, record: TemporalRecord) -> None:
        try:
            with open(self._jsonl_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(record.to_dict()) + "\n")
        except OSError as exc:
            logger.warning("TemporalMemory: failed to write record: %s", exc)

    def _load_from_disk(self) -> None:
        if not os.path.exists(self._jsonl_path):
            return
        loaded = 0
        try:
            with open(self._jsonl_path, encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        d = json.loads(line)
                        rec = TemporalRecord.from_dict(d)
                        self._buffer.append(rec)
                        loaded += 1
                    except (json.JSONDecodeError, KeyError, ValueError):
                        pass
        except OSError as exc:
            logger.warning("TemporalMemory: failed to load records: %s", exc)
            return
        logger.info("TemporalMemory: loaded %d records from %s", loaded, self._jsonl_path)

    # -- Helpers ---------------------------------------------------------------

    @staticmethod
    def _format_record(record: TemporalRecord) -> str:
        """Format a single record as a one-line LLM context string."""
        age_str = _format_age(time.time() - record.timestamp)
        labels = list({o["label"] for o in record.objects})
        obj_str = ", ".join(labels) if labels else "nothing"
        pos = record.position
        return (
            f"[{age_str}] at ({pos[0]:.1f}, {pos[1]:.1f}): {obj_str}"
        )


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def _format_age(seconds: float) -> str:
    """Convert elapsed seconds to a human-readable string."""
    seconds = max(0.0, seconds)
    if seconds < 60:
        return f"{seconds:.0f}s ago"
    minutes = seconds / 60.0
    if minutes < 60:
        return f"{minutes:.1f}min ago"
    hours = minutes / 60.0
    return f"{hours:.1f}h ago"
