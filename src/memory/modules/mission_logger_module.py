"""MissionLoggerModule — mission history recorder as a Module.

Records each navigation mission's lifecycle: start/end time, trajectory,
distance, result. Supports querying history via the @skill interface.

Layer: L3 (Memory)
Inputs: mission_status, odometry
Outputs: (none — side-effect: persists JSON records to disk)
"""

from __future__ import annotations

import json
import logging
import math
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from runtime import In, Module, skill
from runtime.msgs import Odometry
from runtime.registry import register

logger = logging.getLogger(__name__)

MAX_RECORDS = 500
TRAJECTORY_SAMPLE_INTERVAL = 1.0  # seconds


@register("memory", "mission_logger", description="Records navigation mission lifecycle and trajectory to disk")
class MissionLoggerModule(Module, layer=3):
    """Records mission history to JSON files on disk.

    Each completed navigation mission is saved as a separate JSON file,
    keyed by ISO timestamp.  The @skill methods expose history queries
    to the MCP / REPL.

    Config:
        log_dir: directory for JSON records
                 (default: ~/.lingtu/mission_history)
        max_records: max files to keep (default: 500)
        trajectory_interval: odometry sampling period in seconds (default: 1.0)
    """

    # -- ports --
    mission_status: In[dict]
    odometry: In[Odometry]

    def __init__(self, **config: Any) -> None:
        log_dir = config.pop("log_dir", os.path.expanduser("~/.lingtu/mission_history"))
        self._max_records: int = config.pop("max_records", MAX_RECORDS)
        self._traj_interval: float = config.pop("trajectory_interval", TRAJECTORY_SAMPLE_INTERVAL)
        super().__init__(**config)

        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)

        self._current: dict[str, Any] | None = None
        self._last_odom_time: float = 0.0
        self._last_x: float = 0.0
        self._last_y: float = 0.0
        self._total_distance: float = 0.0

    # -- lifecycle --

    def setup(self) -> None:
        self.mission_status.subscribe(self._on_mission_status)
        self.odometry.subscribe(self._on_odom)

    def _on_mission_status(self, status: dict) -> None:
        state = str(status.get("state", "")).upper()

        # IDLE -> PLANNING: start new mission
        if state == "PLANNING" and self._current is None:
            now = datetime.now(timezone.utc)
            goal = status.get("goal", "")
            self._current = {
                "id": now.isoformat(),
                "start_time": now.isoformat(),
                "end_time": None,
                "result": "IN_PROGRESS",
                "duration_sec": 0,
                "distance_m": 0.0,
                "goal": goal if isinstance(goal, str) else str(goal),
                "replan_count": 0,
                "trajectory": [],
            }
            self._total_distance = 0.0
            self._last_odom_time = 0.0
            logger.info("Mission started: %s", self._current["id"])

        # Track replanning events
        if state == "REPLANNING" and self._current is not None:
            self._current["replan_count"] += 1

        # COMPLETE / SUCCESS / FAILED: close mission
        if state in ("COMPLETE", "SUCCESS", "FAILED") and self._current is not None:
            now = datetime.now(timezone.utc)
            self._current["end_time"] = now.isoformat()
            self._current["result"] = state
            self._current["distance_m"] = round(self._total_distance, 2)

            try:
                start = datetime.fromisoformat(self._current["start_time"])
                self._current["duration_sec"] = round((now - start).total_seconds(), 1)
            except Exception:
                pass

            self._save_mission(self._current)
            self._enforce_max_records()

            logger.info(
                "Mission ended: %s  result=%s  duration=%.1fs  distance=%.1fm",
                self._current["id"], state,
                self._current["duration_sec"], self._current["distance_m"],
            )

            self._current = None

    def _on_odom(self, odom: Odometry) -> None:
        if self._current is None:
            return

        import time as _time
        now_sec = _time.monotonic()
        x, y = odom.x, odom.y

        if self._last_odom_time > 0.0:
            dx = x - self._last_x
            dy = y - self._last_y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist <= 5.0:  # ignore teleport jumps (>5m per sample)
                self._total_distance += dist

        if now_sec - self._last_odom_time >= self._traj_interval:
            self._current["trajectory"].append(
                [round(x, 3), round(y, 3), round(now_sec, 2)]
            )
            self._last_odom_time = now_sec

        self._last_x = x
        self._last_y = y

    # -- persistence --

    def _save_mission(self, mission: dict[str, Any]) -> None:
        safe_id = mission["id"].replace(":", "-")
        path = self._log_dir / f"{safe_id}.json"
        try:
            path.write_text(
                json.dumps(mission, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        except OSError as exc:
            logger.error("Failed to save mission record: %s", exc)

    def _enforce_max_records(self) -> None:
        files = sorted(self._log_dir.glob("*.json"))
        if len(files) > self._max_records:
            for old in files[: len(files) - self._max_records]:
                try:
                    old.unlink()
                except OSError:
                    pass

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["logged_entries"] = len(list(self._log_dir.glob("*.json")))
        info["buffer_size"] = 1 if self._current is not None else 0
        return info

    # ---------------------------------------------------------------------------
    # Query API
    #
    # Rule: @skill methods return JSON str for MCP/agent consumption while
    # internal callers use the native Python helpers below.
    # ---------------------------------------------------------------------------

    # -- Query helpers (native types) --

    def _list_missions_raw(self, count: int = 10) -> list[dict[str, Any]]:
        """Return summary dicts of the most recent *count* missions."""
        files = sorted(self._log_dir.glob("*.json"), reverse=True)
        results: list[dict[str, Any]] = []
        for f in files[:count]:
            try:
                data = json.loads(f.read_text(encoding="utf-8"))
                results.append(
                    {
                        "id": data.get("id", ""),
                        "start_time": data.get("start_time", ""),
                        "end_time": data.get("end_time", ""),
                        "result": data.get("result", ""),
                        "duration_sec": data.get("duration_sec", 0),
                        "distance_m": data.get("distance_m", 0.0),
                        "goal": data.get("goal", ""),
                        "replan_count": data.get("replan_count", 0),
                    }
                )
            except Exception as exc:
                logger.warning("Failed to read mission record %s: %s", f.name, exc)
        return results

    def _stats_raw(self) -> dict[str, Any]:
        """Return aggregate statistics dict."""
        files = list(self._log_dir.glob("*.json"))
        total = len(files)
        success = failed = 0
        total_dist = 0.0
        total_dur = 0.0
        for f in files:
            try:
                d = json.loads(f.read_text(encoding="utf-8"))
                r = d.get("result", "")
                if r in ("COMPLETE", "SUCCESS"):
                    success += 1
                elif r == "FAILED":
                    failed += 1
                total_dist += float(d.get("distance_m", 0))
                total_dur += float(d.get("duration_sec", 0))
            except Exception:
                pass
        return {
            "total": total,
            "success": success,
            "failed": failed,
            "in_progress": 1 if self._current else 0,
            "total_distance_m": round(total_dist, 1),
            "total_duration_sec": round(total_dur, 1),
            "log_dir": str(self._log_dir),
        }

    # -- MCP @skill (JSON str) — names exposed to AI agents --

    @skill
    def list_missions(self, count: int = 10) -> str:
        """List the *count* most recent navigation missions (summary, no trajectory).

        Args:
            count: Number of missions to return (default 10)
        """
        return json.dumps({"missions": self._list_missions_raw(count)})

    @skill
    def get_mission_stats(self) -> str:
        """Return aggregate statistics for all recorded navigation missions."""
        return json.dumps(self._stats_raw())

    # -- Backward-compat shims for tests / REPL that expect native dicts --

    def get_stats(self) -> dict[str, Any]:
        """Return aggregate statistics dict (REPL / direct call compat)."""
        return self._stats_raw()

    # -- Full detail (REPL only, not exposed as @skill — trajectory is large) --

    def get_mission(self, mission_id: str) -> dict[str, Any] | None:
        """Return full detail (including trajectory) for a mission by ID."""
        if not mission_id:
            return None
        safe_name = mission_id.replace(":", "-")
        candidates = list(self._log_dir.glob(f"{safe_name}*.json"))
        if not candidates:
            for f in self._log_dir.glob("*.json"):
                try:
                    data = json.loads(f.read_text(encoding="utf-8"))
                    if data.get("id") == mission_id:
                        return data
                except Exception:
                    continue
            return None
        try:
            return json.loads(candidates[0].read_text(encoding="utf-8"))
        except Exception:
            return None
