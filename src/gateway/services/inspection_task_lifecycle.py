"""Read-only product projection for native inspection task facts.

The native navigation endpoint owns execution.  This object deliberately keeps
two kinds of information separate:

* a business ACK proves only that a command was admitted; and
* an ``InspectionTaskEvent`` proves a native lifecycle fact.

It is a bounded projection rather than a task authority. Field deployment may
back it with an atomically written journal; local/dev callers remain
process-local unless they explicitly configure a path. Missing, reordered, and
post-restart events are surfaced instead of being filled in from command intent.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from collections import OrderedDict
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from gateway.services._journal_file import atomic_write as _atomic_write

_STATE_NAMES = {
    0: "IDLE",
    1: "VALIDATING",
    2: "PLANNING",
    3: "NAVIGATING",
    4: "DWELLING",
    5: "PAUSED",
    6: "RECOVERING",
    7: "SUCCEEDED",
    8: "FAILED",
    9: "CANCELLED",
    10: "SETTLING",
    11: "ACTION_PENDING",
    12: "PAUSING",
    13: "CANCELLING",
}
_TASK_EVENT_STATES = frozenset(_STATE_NAMES) - {0}
_PUBLIC_STATE_BY_PHASE = {
    "VALIDATING": "PLANNING",
    "PLANNING": "PLANNING",
    "NAVIGATING": "EXECUTING",
    "DWELLING": "EXECUTING",
    "PAUSED": "PAUSED",
    "RECOVERING": "RECOVERING",
    "SUCCEEDED": "SUCCESS",
    "FAILED": "FAILED",
    "CANCELLED": "CANCELLED",
    "SETTLING": "EXECUTING",
    "ACTION_PENDING": "EXECUTING",
}
_TRANSITION_BY_PHASE = {
    "PAUSING": "PAUSE_REQUESTED",
    "CANCELLING": "CANCEL_REQUESTED",
}
_EVENT_KIND_NAMES = {
    1: "TASK_ACCEPTED",
    2: "STATE_CHANGED",
    3: "MILESTONE",
    4: "STOP_CONFIRMATION_FAILED",
    5: "EVIDENCE_RECORDED",
}
_TERMINAL_STATES = {7, 8, 9}
_PAUSE_CANCEL_STATES = {1, 2, 3, 4, 6, 10, 11}
_JOURNAL_SCHEMA = "lingtu.inspection.task.journal.v1"
_JOURNAL_MAX_BYTES = 16 * 1024 * 1024
_JOURNAL_PATH_ENV = "LINGTU_INSPECTION_TASK_JOURNAL"
_PROCESS_LOCAL_RETENTION = "process_local_gateway_projection"
_DURABLE_RETENTION = "durable_gateway_projection"
_RECORDING_STATES = {"recording", "stopping", "completed", "failed"}


class InspectionTaskJournalUnavailable(RuntimeError):
    """Raised when configured task facts cannot be durably committed."""


class InspectionTaskTimeline:
    """Bounded Gateway projection of ordered native inspection task events."""

    def __init__(
        self,
        *,
        task_retention: int = 256,
        event_retention: int = 64,
        journal_path: str | os.PathLike[str] | None = None,
    ) -> None:
        self._task_retention = max(1, int(task_retention))
        self._event_retention = max(1, int(event_retention))
        self._request_binding_retention = max(
            256,
            self._task_retention * self._event_retention,
        )
        self._lock = threading.RLock()
        self._tasks: OrderedDict[str, dict[str, Any]] = OrderedDict()
        self._request_bindings: OrderedDict[str, dict[str, Any]] = OrderedDict()
        self._last_sequence_by_boot: dict[str, int] = {}
        self._recording_stop_claims: set[str] = set()
        self._journal_path = (
            Path(journal_path).expanduser() if journal_path is not None else None
        )
        self._journal_status = "disabled" if self._journal_path is None else "ready"
        self._journal_error = ""
        if self._journal_path is not None:
            self._restore_journal()

    def record_submission(
        self,
        *,
        task_id: str,
        action: str,
        request_id: str,
        route_id: str | None = None,
        map_id: str | None = None,
        map_content_epoch: int | None = None,
        route_revision: int | None = None,
        route_snapshot: Mapping[str, Any] | None = None,
        reason: str | None = None,
    ) -> None:
        """Record an admitted command without asserting execution progress."""

        normalized_task = _required_text(task_id, "task_id")
        normalized_request = _required_text(request_id, "request_id")
        normalized_action = _required_text(action, "action")
        normalized_route = _normalise_route_snapshot(route_snapshot)
        normalized_reason = str(reason or "")
        binding = _request_binding(
            task_id=normalized_task,
            action=normalized_action,
            request_id=normalized_request,
            reason=normalized_reason,
            route_snapshot=normalized_route,
        )
        with self._lock:
            self._require_journal_available_locked()
            self._reserve_request_locked(binding)
            now = time.time()
            record = self._record_for(normalized_task)
            identity = record["identity"]
            _require_identity_compatible(
                identity,
                {
                    "route_id": route_id,
                    "map_id": map_id,
                    "map_content_epoch": map_content_epoch,
                    "route_revision": route_revision,
                },
            )
            _set_identity_if_absent(identity, "route_id", route_id)
            _set_identity_if_absent(identity, "map_id", map_id)
            _set_identity_if_absent(identity, "map_content_epoch", map_content_epoch)
            _set_identity_if_absent(identity, "route_revision", route_revision)
            if normalized_route is not None:
                _require_compatible_route_snapshot(record, normalized_route)
                record["route_snapshot"] = normalized_route
            command = {
                "action": normalized_action,
                "request_id": normalized_request,
                "reason": normalized_reason,
                "accepted_at": now,
                "source": "business_ack_only",
            }
            commands = record["commands"]
            retained_command = next(
                (
                    item
                    for item in commands
                    if item.get("request_id") == normalized_request
                ),
                None,
            )
            if retained_command is None:
                commands.append(command)
                if len(commands) > self._event_retention:
                    del commands[: len(commands) - self._event_retention]
            else:
                command = retained_command
            record["last_submission"] = command
            request_binding = self._request_bindings[normalized_request]
            if request_binding.get("accepted_at") is None:
                request_binding["accepted_at"] = now
            record["updated_at"] = now
            self._persist_journal_locked()

    def reserve_submission(
        self,
        *,
        task_id: str,
        action: str,
        request_id: str,
        reason: str | None = None,
        route_snapshot: Mapping[str, Any] | None = None,
    ) -> None:
        """Durably bind one idempotency key before contacting the native endpoint."""

        normalized_route = _normalise_route_snapshot(route_snapshot)
        binding = _request_binding(
            task_id=_required_text(task_id, "task_id"),
            action=_required_text(action, "action"),
            request_id=_required_text(request_id, "request_id"),
            reason=str(reason or ""),
            route_snapshot=normalized_route,
        )
        with self._lock:
            self._require_journal_available_locked()
            self._reserve_request_locked(binding)
            self._persist_journal_locked()

    def bind_recording(
        self,
        *,
        task_id: str,
        session_id: str,
        product_session_id: str,
    ) -> dict[str, Any]:
        """Durably bind one task to the exact evidence recording it owns."""

        normalized_task = _required_text(task_id, "task_id")
        normalized_session = _required_text(session_id, "recording session_id")
        normalized_product_session_id = _required_text(
            product_session_id,
            "recording product_session_id",
        )
        with self._lock:
            self._require_journal_available_locked()
            record = self._record_for(normalized_task)
            existing = record.get("recording")
            if existing is not None:
                if (
                    existing["session_id"] != normalized_session
                    or existing["product_session_id"] != normalized_product_session_id
                ):
                    raise ValueError("inspection task recording identity mismatch")
                return dict(existing)
            now = time.time()
            recording = {
                "session_id": normalized_session,
                "product_session_id": normalized_product_session_id,
                "state": "recording",
                "error": "",
                "updated_at": now,
            }
            record["recording"] = recording
            record["updated_at"] = now
            self._persist_journal_locked()
            return dict(recording)

    def claim_recording_stop(
        self,
        task_id: str,
        *,
        recover_stopping: bool = False,
    ) -> dict[str, Any] | None:
        """Atomically claim the one allowed stop for a task recording."""

        normalized_task = _required_text(task_id, "task_id")
        with self._lock:
            self._require_journal_available_locked()
            record = self._tasks.get(normalized_task)
            if record is None or not isinstance(record.get("recording"), Mapping):
                return None
            recording = record["recording"]
            if normalized_task in self._recording_stop_claims:
                return None
            if recording["state"] != "recording" and not (
                recover_stopping and recording["state"] == "stopping"
            ):
                return None
            self._recording_stop_claims.add(normalized_task)
            now = time.time()
            recording["state"] = "stopping"
            recording["updated_at"] = now
            record["updated_at"] = now
            try:
                self._persist_journal_locked()
            except InspectionTaskJournalUnavailable:
                self._recording_stop_claims.discard(normalized_task)
                raise
            return dict(recording)

    def finish_recording_stop(
        self,
        *,
        task_id: str,
        expected_session_id: str,
        state: str,
        error: str = "",
    ) -> None:
        """Persist the result of an exact-session stop attempt."""

        normalized_task = _required_text(task_id, "task_id")
        normalized_session = _required_text(
            expected_session_id,
            "recording session_id",
        )
        if state not in {"recording", "completed", "failed"}:
            raise ValueError(
                "recording stop state must be recording, completed or failed"
            )
        with self._lock:
            self._require_journal_available_locked()
            record = self._tasks.get(normalized_task)
            recording = record.get("recording") if record is not None else None
            if not isinstance(recording, Mapping):
                raise ValueError("inspection task recording is not bound")
            if recording["session_id"] != normalized_session:
                raise ValueError("inspection task recording identity mismatch")
            now = time.time()
            recording["state"] = state
            recording["error"] = str(error or "")
            recording["updated_at"] = now
            record["updated_at"] = now
            self._persist_journal_locked()
            self._recording_stop_claims.discard(normalized_task)

    def observe(self, event: Any) -> bool:
        """Accept one monotonic native event and reject duplicates/reordering.

        A sequence gap is not rejected: the newest native fact is still useful,
        but the resulting projection is explicitly marked incomplete and cannot
        offer task controls until continuity is restored by a future product
        journal/reconciliation feature.
        """

        normalized = _normalise_event(event)
        if normalized is None:
            return False
        boot_id = normalized["boot_id"]
        sequence = normalized["event_sequence"]
        task_id = normalized["task_id"]
        with self._lock:
            self._require_journal_available_locked()
            known_boots = bool(self._last_sequence_by_boot)
            previous_sequence = self._last_sequence_by_boot.get(boot_id, 0)
            if sequence <= previous_sequence:
                return False
            if boot_id not in self._last_sequence_by_boot and known_boots:
                self._mark_endpoint_restart(boot_id)
            # The first event observed by this Gateway may itself be late.  A
            # non-one cursor means this process cannot honestly claim a full
            # task history even when no earlier local cursor exists.
            gap_detected = sequence != previous_sequence + 1
            self._last_sequence_by_boot[boot_id] = sequence

            record = self._record_for(task_id)
            identity = record["identity"]
            conflict_field = _identity_conflict_field(identity, normalized)
            if conflict_field is not None:
                if gap_detected:
                    self._mark_sequence_gap(boot_id)
                delivery = record["delivery"]
                delivery["continuity"] = "identity_conflict"
                delivery["history_complete"] = False
                delivery["reason"] = "native_event_identity_conflict"
                delivery["restored_from_journal"] = False
                record["updated_at"] = time.time()
                self._persist_journal_locked()
                return False
            _set_identity_if_absent(identity, "route_id", normalized["route_id"])
            _set_identity_if_absent(identity, "map_id", normalized["map_id"])
            _set_identity_if_absent(identity, "map_content_epoch", normalized["map_content_epoch"])
            _set_identity_if_absent(identity, "route_revision", normalized["route_revision"])
            delivery = record["delivery"]
            delivery["boot_id"] = boot_id
            delivery["event_sequence"] = sequence
            if gap_detected:
                self._mark_sequence_gap(boot_id)
            if delivery["continuity"] != "endpoint_restart_observed":
                delivery["continuity"] = "verified"
                if not delivery["history_complete"]:
                    delivery["reason"] = str(
                        delivery.get("reason") or "event_history_incomplete"
                    )
                else:
                    delivery["reason"] = ""

            record["latest_event"] = normalized
            record["timeline"].append(normalized)
            if len(record["timeline"]) > self._event_retention:
                del record["timeline"][: len(record["timeline"]) - self._event_retention]
                delivery["history_complete"] = False
                delivery["reason"] = "event_retention_exceeded"
            record["updated_at"] = time.time()
            delivery["restored_from_journal"] = False
            self._persist_journal_locked()
            return True

    def query(self, task_id: str) -> dict[str, Any]:
        """Return an honest, user-facing task snapshot for one stable task ID."""

        normalized_task = str(task_id or "").strip()
        if not normalized_task:
            return _unknown_task_snapshot("", "task_id_required", self._retention_name())
        with self._lock:
            if self._journal_status in {"corrupt", "write_failed"}:
                return _unknown_task_snapshot(
                    normalized_task,
                    f"task_journal_{self._journal_status}",
                    self._retention_name(),
                )
            record = self._tasks.get(normalized_task)
            if record is None:
                return _unknown_task_snapshot(
                    normalized_task,
                    "task_status_unknown",
                    self._retention_name(),
                )
            self._tasks.move_to_end(normalized_task)
            return _snapshot(record)

    def list_tasks(
        self,
        *,
        map_id: str | None = None,
        route_id: str | None = None,
        include_terminal: bool = False,
        limit: int = 20,
    ) -> dict[str, Any]:
        """List retained projections without inventing an execution history.

        This lets a browser refresh reconnect to a task already seen by this
        Gateway. A configured journal survives Gateway restart, but active-task
        control remains disabled until native continuity is reconciled.
        """

        normalized_map = str(map_id or "").strip()
        normalized_route = str(route_id or "").strip()
        bounded_limit = max(1, min(int(limit), self._task_retention))
        with self._lock:
            self._require_journal_available_locked()
            tasks: list[dict[str, Any]] = []
            for record in reversed(self._tasks.values()):
                snapshot = _snapshot(record)
                identity = snapshot["identity"]
                if normalized_map and identity.get("map_id") != normalized_map:
                    continue
                if normalized_route and identity.get("route_id") != normalized_route:
                    continue
                if not include_terminal and bool(snapshot["terminal"]):
                    continue
                tasks.append(snapshot)
                if len(tasks) >= bounded_limit:
                    break
            return {
                "schema_version": "lingtu.inspection.task.v1",
                "retention": self._retention_name(),
                "count": len(tasks),
                "tasks": tasks,
                "ts": time.time(),
            }

    def health(self) -> dict[str, Any]:
        """Expose projection limits and durable journal health."""

        with self._lock:
            return {
                "retention": self._retention_name(),
                "task_retention": self._task_retention,
                "event_retention_per_task": self._event_retention,
                "request_binding_retention": self._request_binding_retention,
                "request_bindings": len(self._request_bindings),
                "tracked_tasks": len(self._tasks),
                "boot_cursors": dict(self._last_sequence_by_boot),
                "journal": {
                    "status": self._journal_status,
                    "path": str(self._journal_path) if self._journal_path else "",
                    "error": self._journal_error,
                },
            }

    def require_available(self) -> None:
        """Fail before command admission when configured persistence is unavailable."""

        with self._lock:
            self._require_journal_available_locked()

    def prepare_route_snapshot(
        self,
        route: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Validate and minimize immutable report requirements before admission."""

        snapshot = _normalise_route_snapshot(route)
        if snapshot is None:
            raise ValueError("inspection task route snapshot is required")
        return snapshot

    def require_route_snapshot_compatible(
        self,
        task_id: str,
        route_snapshot: Mapping[str, Any],
    ) -> None:
        """Reject an idempotent retry that would redefine an existing task."""

        normalized_task = _required_text(task_id, "task_id")
        normalized_route = _normalise_route_snapshot(route_snapshot)
        if normalized_route is None:
            raise ValueError("inspection task route snapshot is required")
        with self._lock:
            self._require_journal_available_locked()
            record = self._tasks.get(normalized_task)
            if record is not None:
                _require_compatible_route_snapshot(record, normalized_route)

    def _reserve_request_locked(self, binding: Mapping[str, Any]) -> None:
        request_id = str(binding["request_id"])
        existing = self._request_bindings.get(request_id)
        if existing is not None:
            _require_request_binding_compatible(existing, binding)
            self._request_bindings.move_to_end(request_id)
            return
        reserved = dict(binding)
        reserved["reserved_at"] = time.time()
        reserved["accepted_at"] = None
        reserved["source"] = "gateway_pre_native_reservation"
        self._request_bindings[request_id] = reserved
        while len(self._request_bindings) > self._request_binding_retention:
            self._request_bindings.popitem(last=False)

    def _record_for(self, task_id: str) -> dict[str, Any]:
        record = self._tasks.get(task_id)
        if record is None:
            record = {
                "identity": {
                    "task_id": task_id,
                    "route_id": None,
                    "map_id": None,
                    "map_content_epoch": None,
                    "route_revision": None,
                },
                "last_submission": None,
                "commands": [],
                "route_snapshot": None,
                "latest_event": None,
                "timeline": [],
                "recording": None,
                "delivery": {
                    "continuity": "awaiting_native_event",
                    "history_complete": True,
                    "reason": "native_event_pending",
                    "boot_id": "",
                    "event_sequence": 0,
                    "retention": self._retention_name(),
                    "restored_from_journal": False,
                },
                "updated_at": time.time(),
            }
            self._tasks[task_id] = record
            while len(self._tasks) > self._task_retention:
                self._tasks.popitem(last=False)
        self._tasks.move_to_end(task_id)
        return record

    def _retention_name(self) -> str:
        return (
            _DURABLE_RETENTION
            if self._journal_path is not None
            else _PROCESS_LOCAL_RETENTION
        )

    def _require_journal_available_locked(self) -> None:
        if self._journal_status in {"corrupt", "write_failed"}:
            raise InspectionTaskJournalUnavailable(
                self._journal_error or "inspection task journal is unavailable"
            )

    def _persist_journal_locked(self) -> None:
        if self._journal_path is None:
            return
        if self._journal_status in {"corrupt", "write_failed"}:
            raise InspectionTaskJournalUnavailable(
                self._journal_error or "inspection task journal is unavailable"
            )
        body = {
            "schema_version": _JOURNAL_SCHEMA,
            "saved_at": time.time(),
            "task_retention": self._task_retention,
            "event_retention_per_task": self._event_retention,
            "last_sequence_by_boot": dict(self._last_sequence_by_boot),
            "request_bindings": list(self._request_bindings.values()),
            "tasks": list(self._tasks.values()),
        }
        payload = _canonical_json(body) + b"\n"
        if len(payload) > _JOURNAL_MAX_BYTES:
            self._journal_status = "write_failed"
            self._journal_error = "inspection task journal exceeds its size limit"
            raise InspectionTaskJournalUnavailable(self._journal_error)
        try:
            _atomic_write(self._journal_path, payload)
        except OSError as exc:
            self._journal_status = "write_failed"
            self._journal_error = f"inspection task journal write failed: {exc}"
            raise InspectionTaskJournalUnavailable(self._journal_error) from exc
        self._journal_status = "ready"
        self._journal_error = ""

    def _restore_journal(self) -> None:
        if self._journal_path is None:
            return
        if not self._journal_path.exists():
            return
        try:
            if not self._journal_path.is_file():
                raise ValueError("journal path is not a regular file")
            if self._journal_path.stat().st_size > _JOURNAL_MAX_BYTES:
                raise ValueError("journal exceeds its size limit")
            body = json.loads(self._journal_path.read_text(encoding="utf-8"))
            if not isinstance(body, Mapping):
                raise ValueError("journal body must be an object")
            if body.get("schema_version") != _JOURNAL_SCHEMA:
                raise ValueError("unsupported journal schema")
            task_values = body.get("tasks")
            if not isinstance(task_values, list):
                raise ValueError("journal tasks must be a list")
            if len(task_values) > self._task_retention:
                raise ValueError("journal task retention exceeds the configured limit")
            cursors = _restore_boot_cursors(body.get("last_sequence_by_boot"))
            request_bindings = _restore_request_bindings(
                body.get("request_bindings", []),
                retention=self._request_binding_retention,
            )
            restored: OrderedDict[str, dict[str, Any]] = OrderedDict()
            for value in task_values:
                record = _restore_record(
                    value,
                    event_retention=self._event_retention,
                    retention=self._retention_name(),
                )
                task_id = str(record["identity"]["task_id"])
                if task_id in restored:
                    raise ValueError("journal contains a duplicate task_id")
                for event in record["timeline"]:
                    if cursors.get(str(event["boot_id"]), 0) < int(
                        event["event_sequence"]
                    ):
                        raise ValueError("journal boot cursor is behind a retained event")
                for command in record["commands"]:
                    route_snapshot = (
                        record.get("route_snapshot")
                        if command["action"] == "start"
                        else None
                    )
                    restored_binding = _request_binding(
                        task_id=task_id,
                        action=command["action"],
                        request_id=command["request_id"],
                        reason=command["reason"],
                        route_snapshot=route_snapshot,
                    )
                    existing_binding = request_bindings.get(command["request_id"])
                    if existing_binding is not None:
                        _require_request_binding_compatible(
                            existing_binding,
                            restored_binding,
                        )
                    else:
                        restored_binding["reserved_at"] = command["accepted_at"]
                        restored_binding["accepted_at"] = command["accepted_at"]
                        restored_binding["source"] = "gateway_pre_native_reservation"
                        request_bindings[command["request_id"]] = restored_binding
                        while (
                            len(request_bindings) > self._request_binding_retention
                        ):
                            request_bindings.popitem(last=False)
                restored[task_id] = record
            self._tasks = restored
            self._request_bindings = request_bindings
            self._last_sequence_by_boot = cursors
            self._journal_status = "ready"
            self._journal_error = ""
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._tasks.clear()
            self._request_bindings.clear()
            self._last_sequence_by_boot.clear()
            self._journal_status = "corrupt"
            self._journal_error = f"inspection task journal load failed: {exc}"

    def _mark_sequence_gap(self, boot_id: str) -> None:
        for record in self._tasks.values():
            delivery = record["delivery"]
            if delivery.get("boot_id") == boot_id:
                delivery["history_complete"] = False
                delivery["reason"] = "event_sequence_gap"

    def _mark_endpoint_restart(self, new_boot_id: str) -> None:
        for record in self._tasks.values():
            latest = record.get("latest_event")
            if latest is not None and int(latest["state"]) in _TERMINAL_STATES:
                continue
            delivery = record["delivery"]
            if delivery.get("boot_id") == new_boot_id:
                continue
            delivery["continuity"] = "endpoint_restart_observed"
            delivery["history_complete"] = False
            delivery["reason"] = "endpoint_restarted"


def ensure_inspection_task_timeline(gateway: Any) -> InspectionTaskTimeline:
    """Return the Gateway-owned projector, creating it for lightweight hosts/tests."""

    timeline = getattr(gateway, "_inspection_task_timeline", None)
    if isinstance(timeline, InspectionTaskTimeline):
        return timeline
    timeline = create_inspection_task_timeline()
    gateway._inspection_task_timeline = timeline
    return timeline


def create_inspection_task_timeline() -> InspectionTaskTimeline:
    """Create the Host projection using an explicitly declared journal path."""

    configured = str(os.environ.get(_JOURNAL_PATH_ENV) or "").strip()
    return InspectionTaskTimeline(journal_path=configured or None)


def _required_text(value: object, label: str) -> str:
    normalized = str(value or "").strip()
    if not normalized:
        raise ValueError(f"{label} is required")
    return normalized


def _request_binding(
    *,
    task_id: str,
    action: str,
    request_id: str,
    reason: str,
    route_snapshot: Mapping[str, Any] | None,
) -> dict[str, Any]:
    return {
        "request_id": request_id,
        "task_id": task_id,
        "action": action,
        "reason": reason,
        "route_snapshot": dict(route_snapshot) if route_snapshot is not None else None,
    }


def _require_request_binding_compatible(
    existing: Mapping[str, Any],
    candidate: Mapping[str, Any],
) -> None:
    for field in (
        "task_id",
        "action",
        "reason",
        "route_snapshot",
    ):
        if existing.get(field) != candidate.get(field):
            raise ValueError(
                "inspection request_id already identifies a different " + field
            )


def _require_identity_compatible(
    identity: Mapping[str, Any],
    candidate: Mapping[str, Any],
) -> None:
    for field in ("route_id", "map_id", "map_content_epoch", "route_revision"):
        current_value = identity.get(field)
        candidate_value = candidate.get(field)
        if current_value is None or candidate_value is None:
            continue
        normalized_candidate: object = candidate_value
        if field in {"map_content_epoch", "route_revision"}:
            normalized_candidate = int(candidate_value)
        else:
            normalized_candidate = str(candidate_value).strip()
        if current_value != normalized_candidate:
            raise ValueError("inspection task identity conflicts with " + field)


def _identity_conflict_field(
    identity: Mapping[str, Any],
    event: Mapping[str, Any],
) -> str | None:
    for field in ("route_id", "map_id", "map_content_epoch", "route_revision"):
        expected = identity.get(field)
        if expected is not None and event.get(field) != expected:
            return field
    return None


def _set_identity_if_absent(identity: dict[str, Any], key: str, value: object) -> None:
    if identity.get(key) is not None or value is None:
        return
    if key in {"map_content_epoch", "route_revision"}:
        try:
            identity[key] = int(value)
        except (TypeError, ValueError):
            return
        return
    normalized = str(value).strip()
    if normalized:
        identity[key] = normalized


def _normalise_route_snapshot(
    value: Mapping[str, Any] | None,
) -> dict[str, Any] | None:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise ValueError("inspection task route snapshot must be an object")
    raw_points = value.get("points")
    if not isinstance(raw_points, list):
        raise ValueError("inspection task route snapshot points must be an array")
    points: list[dict[str, Any]] = []
    for raw_point in raw_points:
        if not isinstance(raw_point, Mapping):
            raise ValueError("inspection task route snapshot point must be an object")
        points.append(
            {
                "id": _required_text(
                    raw_point.get("id"),
                    "inspection task route point id",
                ),
                "action": str(raw_point.get("action") or "").strip(),
                "enabled": raw_point.get("enabled") is not False,
            }
        )
    map_content_epoch = value.get("map_content_epoch")
    revision = int(value.get("revision", 0) or 0)
    loop_count = int(value.get("loop_count", 1) or 1)
    if (
        isinstance(map_content_epoch, bool)
        or not isinstance(map_content_epoch, int)
        or map_content_epoch <= 0
        or revision <= 0
        or loop_count <= 0
    ):
        raise ValueError("inspection task route snapshot has invalid version fields")
    return {
        "id": _required_text(value.get("id"), "inspection task route id"),
        "map_id": _required_text(
            value.get("map_id"),
            "inspection task route map_id",
        ),
        "map_content_epoch": map_content_epoch,
        "revision": revision,
        "loop_count": loop_count,
        "failure_policy": str(value.get("failure_policy") or "stop"),
        "points": points,
    }


def _copy_route_snapshot(value: object) -> dict[str, Any] | None:
    if not isinstance(value, Mapping):
        return None
    copied = dict(value)
    raw_points = value.get("points")
    copied["points"] = (
        [dict(point) for point in raw_points if isinstance(point, Mapping)]
        if isinstance(raw_points, list)
        else []
    )
    return copied


def _require_compatible_route_snapshot(
    record: Mapping[str, Any],
    route_snapshot: Mapping[str, Any],
) -> None:
    existing_route = record.get("route_snapshot")
    if existing_route is not None and existing_route != route_snapshot:
        raise ValueError("inspection task route snapshot cannot change")
    identity = record.get("identity")
    if not isinstance(identity, Mapping):
        return
    expected = {
        "route_id": route_snapshot["id"],
        "map_id": route_snapshot["map_id"],
        "map_content_epoch": route_snapshot["map_content_epoch"],
        "route_revision": route_snapshot["revision"],
    }
    for field, expected_value in expected.items():
        current_value = identity.get(field)
        if current_value is not None and current_value != expected_value:
            raise ValueError(
                "inspection task route snapshot conflicts with " + field
            )


def _normalise_event(event: Any) -> dict[str, Any] | None:
    if hasattr(event, "to_dict"):
        payload = event.to_dict()
    elif isinstance(event, Mapping):
        payload = dict(event)
    else:
        return None
    if not isinstance(payload, Mapping):
        return None
    try:
        boot_id = _required_text(payload.get("boot_id"), "boot_id")
        task_id = _required_text(payload.get("task_id"), "task_id")
        request_id = _required_text(payload.get("request_id"), "request_id")
        sequence = int(payload.get("event_sequence", 0))
        kind = int(payload.get("kind", 0))
        state = int(payload.get("state", -1))
        timestamp = float(payload.get("ts", payload.get("timestamp_s", 0.0)))
    except (TypeError, ValueError):
        return None
    if (
        sequence <= 0
        or kind not in _EVENT_KIND_NAMES
        or state not in _TASK_EVENT_STATES
        or not math.isfinite(timestamp)
        or timestamp < 0.0
    ):
        return None
    def _integer(name: str) -> int:
        value = int(payload.get(name, 0))
        return value if value >= 0 else 0

    return {
        "event_id": f"{boot_id}:{sequence}",
        "ts": timestamp,
        "frame_id": str(payload.get("frame_id") or "map"),
        "boot_id": boot_id,
        "event_sequence": sequence,
        "kind": kind,
        "kind_name": _EVENT_KIND_NAMES[kind],
        "task_id": task_id,
        "request_id": request_id,
        "command_request_id": str(payload.get("command_request_id") or ""),
        "state": state,
        "state_name": _STATE_NAMES[state],
        "terminal": state in _TERMINAL_STATES,
        "map_id": str(payload.get("map_id") or ""),
        "map_content_epoch": _integer("map_content_epoch"),
        "route_id": str(payload.get("route_id") or ""),
        "route_revision": _integer("route_revision"),
        "point_index": _integer("point_index"),
        "point_count": _integer("point_count"),
        "loop_index": _integer("loop_index"),
        "retry_count": _integer("retry_count"),
        "point_id": str(payload.get("point_id") or ""),
        "action": str(payload.get("action") or ""),
        "action_request_id": str(payload.get("action_request_id") or ""),
        "evidence_id": str(payload.get("evidence_id") or ""),
        "reason": str(payload.get("reason") or ""),
    }


def _unknown_task_snapshot(
    task_id: str,
    reason: str,
    retention: str = _PROCESS_LOCAL_RETENTION,
) -> dict[str, Any]:
    return {
        "schema_version": "lingtu.inspection.task.v1",
        "found": False,
        "task_id": task_id,
        "current_state": None,
        "state_available": False,
        "phase": None,
        "transition": None,
        "state_source": "none",
        "execution_confirmed": False,
        "terminal": False,
        "terminal_source": "",
        "reason": reason,
        "progress": _unknown_progress(),
        "available_actions": [],
        "can_pause": False,
        "can_resume": False,
        "can_cancel": False,
        "identity": {"task_id": task_id},
        "last_submission": None,
        "latest_event": None,
        "timeline": [],
        "delivery": {
            "continuity": "unknown",
            "history_complete": False,
            "reason": reason,
            "retention": retention,
        },
        "updated_at": time.time(),
    }


def _snapshot(record: Mapping[str, Any]) -> dict[str, Any]:
    identity = dict(record["identity"])
    event = record.get("latest_event")
    delivery = dict(record["delivery"])
    timeline = [dict(item) for item in record["timeline"]]
    last_submission = record.get("last_submission")
    if isinstance(last_submission, Mapping):
        last_submission = dict(last_submission)
    progress = _task_progress(event)
    restored_from_journal = bool(delivery.get("restored_from_journal", False))
    phase = str(event["state_name"]) if isinstance(event, Mapping) else None
    transition = _TRANSITION_BY_PHASE.get(phase)

    if not isinstance(event, Mapping):
        endpoint_restarted = (
            delivery.get("continuity") == "endpoint_restart_observed"
        )
        current_state = None
        state_available = False
        if restored_from_journal:
            state_source = "durable_task_projection"
        else:
            state_source = (
                "continuity_monitor" if endpoint_restarted else "business_ack_only"
            )
        execution_confirmed = False
        terminal = False
        terminal_source = ""
        if restored_from_journal:
            reason = "gateway_restarted_awaiting_native_reconciliation"
        else:
            reason = "endpoint_restarted" if endpoint_restarted else "native_event_pending"
        actions: list[str] = []
    else:
        terminal = bool(event["terminal"])
        interrupted = (
            delivery.get("continuity") == "endpoint_restart_observed" and not terminal
        )
        if restored_from_journal and not terminal:
            current_state = None
            state_available = False
            state_source = "durable_task_projection"
            execution_confirmed = False
            reason = "gateway_restarted_awaiting_native_reconciliation"
        elif interrupted:
            current_state = None
            state_available = False
            state_source = "continuity_monitor"
            execution_confirmed = False
            reason = "endpoint_restarted"
        else:
            current_state = (
                _last_confirmed_public_state(timeline[:-1])
                if transition is not None
                else _PUBLIC_STATE_BY_PHASE[phase]
            )
            state_available = current_state is not None
            state_source = (
                "persisted_native_task_event"
                if restored_from_journal
                else "native_task_event"
            )
            execution_confirmed = True
            reason = str(event.get("reason") or "")
        terminal_source = "native_task_event" if terminal else ""
        actions = (
            []
            if restored_from_journal
            else _available_actions(
                state=int(event["state"]),
                terminal=terminal,
                delivery=delivery,
            )
        )

    return {
        "schema_version": "lingtu.inspection.task.v1",
        "found": True,
        "task_id": identity["task_id"],
        "current_state": current_state,
        "state_available": state_available,
        "phase": phase,
        "transition": transition,
        "state_source": state_source,
        "execution_confirmed": execution_confirmed,
        "terminal": terminal,
        "terminal_source": terminal_source,
        "reason": reason,
        "progress": progress,
        "available_actions": actions,
        "can_pause": "pause" in actions,
        "can_resume": "resume" in actions,
        "can_cancel": "cancel" in actions,
        "identity": identity,
        "route_snapshot": _copy_route_snapshot(record.get("route_snapshot")),
        "last_submission": last_submission,
        "latest_event": dict(event) if isinstance(event, Mapping) else None,
        "timeline": timeline,
        "recording": (
            dict(recording)
            if isinstance((recording := record.get("recording")), Mapping)
            else None
        ),
        "delivery": delivery,
        "updated_at": float(record["updated_at"]),
    }


def _last_confirmed_public_state(timeline: list[Mapping[str, Any]]) -> str | None:
    for event in reversed(timeline):
        state = _PUBLIC_STATE_BY_PHASE.get(str(event.get("state_name") or ""))
        if state is not None:
            return state
    return None


def _unknown_progress() -> dict[str, Any]:
    return {
        "known": False,
        "completed_points": 0,
        "point_count": 0,
        "current_point_number": None,
        "current_point_id": "",
        "loop_number": None,
        "retry_count": 0,
        "action": "",
        "evidence_id": "",
    }


def _task_progress(event: Any) -> dict[str, Any]:
    if not isinstance(event, Mapping):
        return _unknown_progress()
    point_count = max(0, int(event.get("point_count", 0)))
    point_index = max(0, int(event.get("point_index", 0)))
    has_current_point = point_index < point_count
    return {
        "known": True,
        "completed_points": min(point_index, point_count),
        "point_count": point_count,
        "current_point_number": point_index + 1 if has_current_point else None,
        "current_point_id": str(event.get("point_id") or "")
        if has_current_point
        else "",
        "loop_number": max(0, int(event.get("loop_index", 0))) + 1,
        "retry_count": max(0, int(event.get("retry_count", 0))),
        "action": str(event.get("action") or "") if has_current_point else "",
        "evidence_id": str(event.get("evidence_id") or ""),
    }


def _available_actions(
    *,
    state: int,
    terminal: bool,
    delivery: Mapping[str, Any],
) -> list[str]:
    if terminal or delivery.get("continuity") != "verified":
        return []
    if not bool(delivery.get("history_complete", False)):
        return []
    if state in _PAUSE_CANCEL_STATES:
        return ["pause", "cancel"]
    if state == 5:
        return ["resume", "cancel"]
    if state == 12:
        return ["cancel"]
    return []


def _canonical_json(value: object) -> bytes:
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _restore_boot_cursors(value: object) -> dict[str, int]:
    if not isinstance(value, Mapping):
        raise ValueError("journal boot cursors must be an object")
    cursors: dict[str, int] = {}
    for raw_boot_id, raw_sequence in value.items():
        boot_id = _required_text(raw_boot_id, "journal boot_id")
        if isinstance(raw_sequence, bool):
            raise ValueError("journal event sequence must be an integer")
        sequence = int(raw_sequence)
        if sequence <= 0:
            raise ValueError("journal event sequence must be positive")
        cursors[boot_id] = sequence
    return cursors


def _restore_request_bindings(
    value: object,
    *,
    retention: int,
) -> OrderedDict[str, dict[str, Any]]:
    if not isinstance(value, list) or len(value) > retention:
        raise ValueError("journal request bindings violate their retention limit")
    bindings: OrderedDict[str, dict[str, Any]] = OrderedDict()
    for raw_binding in value:
        if not isinstance(raw_binding, Mapping):
            raise ValueError("journal request binding must be an object")
        request_id = _required_text(
            raw_binding.get("request_id"),
            "journal request binding request_id",
        )
        raw_route_snapshot = raw_binding.get("route_snapshot")
        if raw_route_snapshot is not None and not isinstance(raw_route_snapshot, Mapping):
            raise ValueError("journal request binding route snapshot must be an object")
        reserved_at = float(raw_binding.get("reserved_at", -1.0))
        if not math.isfinite(reserved_at) or reserved_at < 0.0:
            raise ValueError("journal request binding has an invalid reservation time")
        raw_accepted_at = raw_binding.get("accepted_at")
        accepted_at: float | None = None
        if raw_accepted_at is not None:
            accepted_at = float(raw_accepted_at)
            if not math.isfinite(accepted_at) or accepted_at < reserved_at:
                raise ValueError("journal request binding has an invalid acceptance time")
        if raw_binding.get("source") != "gateway_pre_native_reservation":
            raise ValueError("journal request binding has an invalid source")
        if request_id in bindings:
            raise ValueError("journal contains a duplicate request binding")
        bindings[request_id] = {
            "request_id": request_id,
            "task_id": _required_text(
                raw_binding.get("task_id"),
                "journal request binding task_id",
            ),
            "action": _required_text(
                raw_binding.get("action"),
                "journal request binding action",
            ),
            "reason": str(raw_binding.get("reason") or ""),
            "route_snapshot": (
                dict(raw_route_snapshot) if raw_route_snapshot is not None else None
            ),
            "reserved_at": reserved_at,
            "accepted_at": accepted_at,
            "source": "gateway_pre_native_reservation",
        }
    return bindings


def _restore_record(
    value: object,
    *,
    event_retention: int,
    retention: str,
) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError("journal task entry must be an object")
    raw_identity = value.get("identity")
    if not isinstance(raw_identity, Mapping):
        raise ValueError("journal task identity must be an object")
    task_id = _required_text(raw_identity.get("task_id"), "journal task_id")
    identity = {
        "task_id": task_id,
        "route_id": _restored_optional_text(raw_identity.get("route_id")),
        "map_id": _restored_optional_text(raw_identity.get("map_id")),
        "map_content_epoch": _restored_optional_integer(raw_identity.get("map_content_epoch")),
        "route_revision": _restored_optional_integer(
            raw_identity.get("route_revision")
        ),
    }
    route_snapshot = _normalise_route_snapshot(value.get("route_snapshot"))
    if route_snapshot is not None:
        if (
            route_snapshot["id"] != identity["route_id"]
            or route_snapshot["map_id"] != identity["map_id"]
            or route_snapshot["map_content_epoch"] != identity["map_content_epoch"]
            or route_snapshot["revision"] != identity["route_revision"]
        ):
            raise ValueError("journal task route snapshot identity mismatch")

    raw_timeline = value.get("timeline")
    if not isinstance(raw_timeline, list) or len(raw_timeline) > event_retention:
        raise ValueError("journal task timeline violates its retention limit")
    timeline: list[dict[str, Any]] = []
    for raw_event in raw_timeline:
        try:
            event = _normalise_event(raw_event)
        except (TypeError, ValueError) as exc:
            raise ValueError("journal contains an invalid task event") from exc
        if (
            event is None
            or event["task_id"] != task_id
            or _identity_conflict_field(identity, event) is not None
        ):
            raise ValueError("journal task event identity mismatch")
        timeline.append(event)

    raw_commands = value.get("commands")
    if not isinstance(raw_commands, list) or len(raw_commands) > event_retention:
        raise ValueError("journal task commands violate their retention limit")
    commands: list[dict[str, Any]] = []
    for raw_command in raw_commands:
        if not isinstance(raw_command, Mapping):
            raise ValueError("journal task command must be an object")
        accepted_at = float(raw_command.get("accepted_at", -1.0))
        if not math.isfinite(accepted_at) or accepted_at < 0.0:
            raise ValueError("journal task command has an invalid timestamp")
        if raw_command.get("source") != "business_ack_only":
            raise ValueError("journal task command has an invalid source")
        commands.append(
            {
                "action": _required_text(
                    raw_command.get("action"), "journal command action"
                ),
                "request_id": _required_text(
                    raw_command.get("request_id"), "journal command request_id"
                ),
                "reason": str(raw_command.get("reason") or ""),
                "accepted_at": accepted_at,
                "source": "business_ack_only",
            }
        )

    raw_delivery = value.get("delivery")
    if not isinstance(raw_delivery, Mapping):
        raise ValueError("journal task delivery must be an object")
    continuity = str(raw_delivery.get("continuity") or "")
    if continuity not in {
        "awaiting_native_event",
        "verified",
        "identity_conflict",
        "endpoint_restart_observed",
        "gateway_restart_awaiting_native_reconciliation",
    }:
        raise ValueError("journal task delivery has an invalid continuity state")
    history_complete = raw_delivery.get("history_complete")
    if not isinstance(history_complete, bool):
        raise ValueError("journal task delivery history flag must be boolean")
    event_sequence = raw_delivery.get("event_sequence", 0)
    if isinstance(event_sequence, bool):
        raise ValueError("journal task delivery sequence must be an integer")
    event_sequence = int(event_sequence)
    if event_sequence < 0:
        raise ValueError("journal task delivery sequence must not be negative")
    boot_id = str(raw_delivery.get("boot_id") or "")
    if timeline:
        latest = timeline[-1]
        if boot_id != latest["boot_id"] or event_sequence != latest["event_sequence"]:
            raise ValueError("journal task delivery cursor does not match its latest event")

    restored_terminal = bool(timeline and timeline[-1]["terminal"])
    if not restored_terminal:
        continuity = "gateway_restart_awaiting_native_reconciliation"
        delivery_reason = "gateway_restarted_awaiting_native_reconciliation"
    else:
        delivery_reason = str(raw_delivery.get("reason") or "")

    updated_at = float(value.get("updated_at", -1.0))
    if not math.isfinite(updated_at) or updated_at < 0.0:
        raise ValueError("journal task has an invalid updated_at")
    recording = _restore_recording(value.get("recording"))
    return {
        "identity": identity,
        "route_snapshot": route_snapshot,
        "last_submission": commands[-1] if commands else None,
        "commands": commands,
        "latest_event": timeline[-1] if timeline else None,
        "timeline": timeline,
        "recording": recording,
        "delivery": {
            "continuity": continuity,
            "history_complete": history_complete,
            "reason": delivery_reason,
            "boot_id": boot_id,
            "event_sequence": event_sequence,
            "retention": retention,
            "restored_from_journal": True,
        },
        "updated_at": updated_at,
    }


def _restore_recording(value: object) -> dict[str, Any] | None:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise ValueError("journal task recording must be an object")
    state = str(value.get("state") or "")
    if state not in _RECORDING_STATES:
        raise ValueError("journal task recording has an invalid state")
    updated_at = float(value.get("updated_at", -1.0))
    if not math.isfinite(updated_at) or updated_at < 0.0:
        raise ValueError("journal task recording has an invalid timestamp")
    error = value.get("error", "")
    if not isinstance(error, str):
        raise ValueError("journal task recording error must be text")
    return {
        "session_id": _required_text(
            value.get("session_id"),
            "journal recording session_id",
        ),
        "product_session_id": _required_text(
            value.get("product_session_id"),
            "journal recording product_session_id",
        ),
        "state": state,
        "error": error,
        "updated_at": updated_at,
    }


def _restored_optional_text(value: object) -> str | None:
    if value is None:
        return None
    normalized = str(value).strip()
    return normalized or None


def _restored_optional_integer(value: object) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError("journal identity integer must not be boolean")
    restored = int(value)
    if restored < 0:
        raise ValueError("journal identity integer must not be negative")
    return restored


__all__ = [
    "InspectionTaskJournalUnavailable",
    "InspectionTaskTimeline",
    "create_inspection_task_timeline",
    "ensure_inspection_task_timeline",
]
