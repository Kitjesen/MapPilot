"""Frame identity validation for nav.mission."""

from __future__ import annotations

import os
import time
from collections.abc import Mapping
from typing import Any, Callable

from runtime.msgs.geometry import PoseStamped
from runtime.runtime_interface import (
    map_frame_id,
    normalize_frame_id,
)

PLANNING_FRAME_ID = map_frame_id()

class FrameContract:
    """Frame identity validation for navigation.

    Provides pure frame-checking helpers and adapter-status publishing
    for frame mismatches.  Does **not** mutate Navigation state;
    callers own the side effects.

    Parameters
    ----------
    planning_frame_id:
        The canonical planning frame (typically ``"map"``).
    publish_adapter_status:
        Callback that accepts a ``dict`` event payload.  Typically
        ``Module.adapter_status.publish``.
    """

    def __init__(
        self,
        planning_frame_id: str,
        publish_adapter_status: Callable[[dict], None],
    ) -> None:
        self._planning_frame_id = planning_frame_id
        self._adapter_status_publish = publish_adapter_status
        # Suppress duplicate reports for the same (source, frame_id) pair.
        self._reported_frame_mismatches: set[tuple[str, str]] = set()
        self._map_odom_link: tuple[str, str] | None = None

    def set_map_odom_tf(self, tf: Mapping[str, Any] | None) -> None:
        """Record an observed map->odom link for odometry frame validation."""
        self._map_odom_link = None
        if not isinstance(tf, Mapping) or tf.get("valid") is False:
            return
        parent = normalize_frame_id(tf.get("frame_id"))
        child = normalize_frame_id(tf.get("child_frame_id"))
        planning = normalize_frame_id(self._planning_frame_id) or PLANNING_FRAME_ID
        if parent == planning and child and child != planning:
            self._map_odom_link = (parent, child)

    # Pure helpers

    @staticmethod
    def goal_frame(goal: PoseStamped, planning_frame_id: str) -> str:
        """Return the frame id of *goal*, falling back to *planning_frame_id*."""
        return str(getattr(goal, "frame_id", "") or planning_frame_id)

    @staticmethod
    def runtime_contract() -> str | None:
        """Return the active runtime contract from environment variables."""
        return (
            os.environ.get("LINGTU_RUNTIME_CONTRACT")
            or os.environ.get("LINGTU_DATA_SOURCE")
        )

    def expected_frames_for_source(self, source: str) -> tuple[str, ...]:
        """Return the expected frame ids for a given data *source*."""
        planning_frame = (
            normalize_frame_id(self._planning_frame_id)
            or PLANNING_FRAME_ID
        )
        if source == "odometry" and self._map_odom_link is not None:
            parent, child = self._map_odom_link
            if parent == planning_frame:
                return tuple(dict.fromkeys((planning_frame, child)))
        return (planning_frame,)

    def expected_frame_label(self, source: str) -> str:
        """Human-readable label of expected frame(s) for *source*."""
        return ",".join(self.expected_frames_for_source(source))

    def is_frame_mismatch(self, frame_id: str, *, source: str) -> bool:
        """Return ``True`` if *frame_id* is known and differs from expected."""
        normalized = normalize_frame_id(frame_id)
        return bool(
            normalized
            and normalized != "unknown"
            and normalized not in self.expected_frames_for_source(source)
        )

    # Reporting

    def report_frame_mismatch(self, source: str, frame_id: str) -> None:
        """Publish a ``frame_mismatch`` event if this pair is new.

        Suppresses duplicate reports for the same (source, frame_id) so
        the event stream is not spammed on every odometry tick.
        """
        if not self.is_frame_mismatch(frame_id, source=source):
            return
        key = (source, frame_id)
        if key in self._reported_frame_mismatches:
            return
        self._reported_frame_mismatches.add(key)
        self._adapter_status_publish({
            "event": "frame_mismatch",
            "reason": "unsupported_frame",
            "source": source,
            "expected_frame": self.expected_frame_label(source),
            "received_frame": frame_id,
            "ts": time.time(),
        })

    # Blockers

    def planning_frame_blocker(
        self,
        odom_frame_id: str,
        costmap_frame_id: str,
    ) -> tuple[str, str] | None:
        """If odometry or costmap frame blocks planning, return ``(source, frame_id)``."""
        for source, frame_id in (
            ("odometry", odom_frame_id),
            ("costmap", costmap_frame_id),
        ):
            if self.is_frame_mismatch(frame_id, source=source):
                return source, frame_id
        return None

    @staticmethod
    def has_motion_artifacts(
        state: str,
        tracker_path_length: int,
        goal: Any,
        patrol_goals: list,
    ) -> bool:
        """Return ``True`` if the system currently has motion artifacts.

        Used to decide whether a frame mismatch should abort an active
        mission or just be logged silently.
        """
        return (
            state in (
                "PLANNING",
                "EXECUTING",
                "PAUSED",
                "RECOVERING",
                "PATROLLING",
                "STUCK",
            )
            or tracker_path_length > 0
            or goal is not None
            or bool(patrol_goals)
        )

