"""Durable, task-centric navigation history.

The ledger owns task identity and lifecycle history. It never dispatches motion
commands; callers must use the navigation command seam for that.
"""

from __future__ import annotations

import json
import math
import os
import sqlite3
import threading
import time
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

_DEFAULT_RELATIVE_PATH = Path("lingtu") / "navigation_tasks.sqlite3"
_GOAL_STATUS_STATES = {
    1: "planning",
    2: "executing",
    3: "failed",
    4: "reached",
    5: "cancelled",
}
_NAVIGATION_LIFECYCLE_STATES = {
    0: "idle",
    1: "planning",
    2: "executing",
    3: "paused",
    4: "recovering",
    5: "reached",
    6: "failed",
    7: "cancelled",
}
_TERMINAL_STATES = {"rejected", "reached", "failed", "cancelled", "interrupted"}
_NONTERMINAL_PROGRESS = {
    "unknown": 0,
    "admitted": 1,
    "accepted": 2,
    "planning": 3,
    "executing": 4,
    "recovering": 5,
}


class TaskLedgerConflict(ValueError):
    """Raised when an identity is reused with different immutable content."""


def default_task_ledger_path(
    environ: Mapping[str, str] | None = None,
    *,
    home: str | os.PathLike[str] | None = None,
) -> Path:
    """Return the configured persistent task-ledger path."""

    env = os.environ if environ is None else environ
    configured = str(env.get("LINGTU_TASK_LEDGER_PATH", "")).strip()
    if configured:
        return Path(configured).expanduser()
    state_home = str(env.get("XDG_STATE_HOME", "")).strip()
    if state_home:
        return Path(state_home).expanduser() / _DEFAULT_RELATIVE_PATH
    base = Path(home).expanduser() if home is not None else Path.home()
    return base / ".local" / "state" / _DEFAULT_RELATIVE_PATH


def _canonical_json(value: Any, *, label: str) -> str:
    if value is None:
        value = {}
    to_dict = getattr(value, "to_dict", None)
    if callable(to_dict):
        value = to_dict()
    try:
        return json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        )
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be JSON-safe") from exc


def _decode_json(value: str) -> Any:
    return json.loads(value)


def _required_identity(value: object, label: str) -> str:
    result = str(value or "").strip()

    if not result:
        raise ValueError(f"{label} is required")
    return result


def _evidence_dict(value: Any, label: str) -> dict[str, Any]:
    to_dict = getattr(value, "to_dict", None)
    if callable(to_dict):
        value = to_dict()
    if not isinstance(value, Mapping):
        raise TypeError(f"{label} must be a mapping or expose to_dict()")
    decoded = _decode_json(_canonical_json(dict(value), label=label))
    if not isinstance(decoded, dict):
        raise ValueError(f"{label} must encode a JSON object")
    return decoded


def _positive_timestamp(value: object, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a positive finite timestamp") from exc
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{label} must be a positive finite timestamp")
    return result


def _goal_status_state(value: object) -> str:
    if isinstance(value, bool):
        raise ValueError("goal status state is invalid")
    if isinstance(value, int):
        try:
            return _GOAL_STATUS_STATES[value]
        except KeyError as exc:
            raise ValueError(f"unknown goal status state {value!r}") from exc
    normalized = str(value or "").strip().lower()
    aliases = {
        "path_active": "executing",
        "reached": "reached",
        "failed": "failed",
        "cancelled": "cancelled",
        "planning": "planning",
        "executing": "executing",
        "recovering": "recovering",
    }
    try:
        return aliases[normalized]
    except KeyError as exc:
        raise ValueError(f"unknown goal status state {value!r}") from exc


def _navigation_lifecycle_state(value: object) -> str:
    if isinstance(value, bool):
        raise ValueError("navigation_state.lifecycle_state is invalid")
    if isinstance(value, int):
        try:
            return _NAVIGATION_LIFECYCLE_STATES[value]
        except KeyError as exc:
            raise ValueError(f"unknown navigation lifecycle state {value!r}") from exc
    normalized = str(value or "").strip().lower()
    aliases = {
        "idle": "idle",
        "planning": "planning",
        "executing": "executing",
        "paused": "paused",
        "recovering": "recovering",
        "success": "reached",
        "reached": "reached",
        "failed": "failed",
        "cancelled": "cancelled",
    }
    try:
        return aliases[normalized]
    except KeyError as exc:
        raise ValueError(f"unknown navigation lifecycle state {value!r}") from exc


def _command_kind(value: object) -> str:
    if isinstance(value, bool):
        raise ValueError("native_ack.kind is invalid")
    if isinstance(value, int):
        names = {1: "goal", 2: "cancel"}
        try:
            return names[value]
        except KeyError as exc:
            raise ValueError(f"unsupported navigation task command kind {value!r}") from exc
    normalized = str(value or "").strip().lower()
    if normalized in {"goal", "cancel"}:
        return normalized
    raise ValueError(f"unsupported navigation task command kind {value!r}")


class NavigationTaskLedger:
    """Thread-safe SQLite history for navigation tasks and command attempts."""

    def __init__(
        self,
        path: str | os.PathLike[str] | None = None,
        *,
        clock: Callable[[], float] = time.time,
    ) -> None:
        resolved = default_task_ledger_path() if path is None else Path(path).expanduser()
        self.path = resolved
        self._clock = clock
        self._lock = threading.RLock()
        self._closed = False
        if str(resolved) != ":memory:":
            resolved.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(
            str(resolved),
            timeout=5.0,
            check_same_thread=False,
        )
        self._conn.row_factory = sqlite3.Row
        self._conn.execute("PRAGMA foreign_keys=ON")
        self._conn.execute("PRAGMA busy_timeout=5000")
        if str(resolved) != ":memory:":
            self._conn.execute("PRAGMA journal_mode=WAL")
            self._conn.execute("PRAGMA synchronous=FULL")
        self._create_schema()

    def _create_schema(self) -> None:
        with self._lock, self._conn:
            self._conn.executescript(
                """
                CREATE TABLE IF NOT EXISTS navigation_tasks (
                    task_id TEXT PRIMARY KEY,
                    state TEXT NOT NULL,
                    terminal INTEGER NOT NULL DEFAULT 0,
                    reason TEXT NOT NULL DEFAULT '',
                    source TEXT NOT NULL DEFAULT '',
                    observed_only INTEGER NOT NULL DEFAULT 0,
                    target_json TEXT NOT NULL,
                    product_fingerprint TEXT NOT NULL DEFAULT '',
                    map_identity_json TEXT NOT NULL,
                    created_at REAL NOT NULL,
                    updated_at REAL NOT NULL,
                    terminal_at REAL,
                    endpoint_boot_id TEXT NOT NULL DEFAULT '',
                    active_request_id TEXT NOT NULL DEFAULT '',
                    cancel_requested_at REAL,
                    cancel_request_id TEXT NOT NULL DEFAULT '',
                    cancel_reason TEXT NOT NULL DEFAULT '',
                    can_resume INTEGER NOT NULL DEFAULT 0,
                    last_goal_status_json TEXT,
                    last_navigation_state_json TEXT
                );

                CREATE TABLE IF NOT EXISTS navigation_task_attempts (
                    task_id TEXT NOT NULL,
                    request_id TEXT NOT NULL,
                    native_request_id TEXT NOT NULL DEFAULT '',
                    kind TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    state TEXT NOT NULL DEFAULT 'admitted',
                    accepted INTEGER,
                    reason TEXT NOT NULL DEFAULT '',
                    endpoint_boot_id TEXT NOT NULL DEFAULT '',
                    native_ack_json TEXT,
                    created_at REAL NOT NULL,
                    updated_at REAL NOT NULL,
                    PRIMARY KEY (task_id, request_id),
                    FOREIGN KEY (task_id) REFERENCES navigation_tasks(task_id)
                        ON DELETE CASCADE
                );

                CREATE TABLE IF NOT EXISTS navigation_task_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    task_id TEXT NOT NULL,
                    request_id TEXT NOT NULL DEFAULT '',
                    event_type TEXT NOT NULL,
                    state TEXT NOT NULL,
                    reason TEXT NOT NULL DEFAULT '',
                    evidence_json TEXT NOT NULL,
                    created_at REAL NOT NULL,
                    dedupe_key TEXT NOT NULL,
                    UNIQUE (task_id, dedupe_key),
                    FOREIGN KEY (task_id) REFERENCES navigation_tasks(task_id)
                        ON DELETE CASCADE
                );

                CREATE INDEX IF NOT EXISTS idx_navigation_tasks_recent
                    ON navigation_tasks(updated_at DESC);
                CREATE INDEX IF NOT EXISTS idx_navigation_tasks_open
                    ON navigation_tasks(terminal, updated_at DESC);
                CREATE INDEX IF NOT EXISTS idx_navigation_events_task
                    ON navigation_task_events(task_id, id);
                CREATE UNIQUE INDEX IF NOT EXISTS idx_navigation_native_request
                    ON navigation_task_attempts(task_id, native_request_id) WHERE native_request_id <> '';
                """
            )

    def lookup_admission(
        self,
        task_id: str,
        request_id: str,
        kind: str,
        payload: Any,
        *,
        target: Any = None,
        product_fingerprint: str = "",
        map_identity: Any = None,
    ) -> dict[str, Any] | None:
        """Return an exact prior admission without changing task history."""

        task = _required_identity(task_id, "task_id")
        request = _required_identity(request_id, "request_id")
        if task == request:
            raise ValueError("task_id and request_id must be distinct")
        if "-clock-retry-" in request:
            raise ValueError("request_id uses the reserved native clock-retry namespace")

        attempt_kind = _required_identity(kind, "kind").lower()
        if attempt_kind not in {"goal", "cancel"}:
            raise ValueError("kind must be 'goal' or 'cancel'")
        payload_json = _canonical_json(payload, label="payload")
        target_json = _canonical_json(target, label="target")
        map_json = _canonical_json(map_identity, label="map_identity")

        with self._lock:
            self._require_open()
            row = self._conn.execute(
                """
                SELECT attempts.kind, attempts.payload_json,
                       tasks.target_json, tasks.product_fingerprint,
                       tasks.map_identity_json
                FROM navigation_task_attempts AS attempts
                JOIN navigation_tasks AS tasks ON tasks.task_id = attempts.task_id
                WHERE attempts.task_id = ? AND attempts.request_id = ?
                """,
                (task, request),
            ).fetchone()
            if row is None:
                return None
            self._validate_existing_admission(
                row,
                attempt_kind=attempt_kind,
                payload_json=payload_json,
                target_json=target_json,
                product_fingerprint=product_fingerprint,
                map_identity=map_identity,
                map_json=map_json,
                require_context_match=True,
            )

            record = self._task_record(task)
            assert record is not None
            return {"record": record, "replay": True}

    @staticmethod
    def _validate_existing_admission(
        row: sqlite3.Row,
        *,
        attempt_kind: str,
        payload_json: str,
        target_json: str,
        product_fingerprint: str,
        map_identity: Any,
        map_json: str,
        require_context_match: bool,
    ) -> None:
        """Validate immutable replay content while the caller holds the ledger lock."""

        if row["kind"] != attempt_kind or row["payload_json"] != payload_json:
            raise TaskLedgerConflict("request identity was already used with different content")
        if attempt_kind == "goal" and row["target_json"] != target_json:
            raise TaskLedgerConflict("task_id was already admitted with a different target")
        supplied_fingerprint = str(product_fingerprint or "").strip()
        if (
            require_context_match or supplied_fingerprint
        ) and supplied_fingerprint != row["product_fingerprint"]:
            raise TaskLedgerConflict("task_id was already admitted under a different Product")
        if (require_context_match or map_identity is not None) and row["map_identity_json"] != map_json:
            raise TaskLedgerConflict("task_id was already admitted against a different map")

    def admit(
        self,
        task_id: str,
        request_id: str,
        kind: str,
        payload: Any,
        *,
        source: str = "",
        target: Any = None,
        product_fingerprint: str = "",
        map_identity: Any = None,
    ) -> dict[str, Any]:
        """Persist one immutable attempt before it is dispatched."""

        task = _required_identity(task_id, "task_id")
        request = _required_identity(request_id, "request_id")
        if task == request:
            raise ValueError("task_id and request_id must be distinct")
        if "-clock-retry-" in request:
            raise ValueError("request_id uses the reserved native clock-retry namespace")

        attempt_kind = _required_identity(kind, "kind").lower()
        if attempt_kind not in {"goal", "cancel"}:
            raise ValueError("kind must be 'goal' or 'cancel'")
        payload_json = _canonical_json(payload, label="payload")
        target_json = _canonical_json(target, label="target")
        map_json = _canonical_json(map_identity, label="map_identity")
        now = float(self._clock())

        with self._lock, self._conn:
            self._require_open()
            existing_attempt = self._conn.execute(
                """
                SELECT attempts.kind, attempts.payload_json,
                       tasks.target_json, tasks.product_fingerprint,
                       tasks.map_identity_json
                FROM navigation_task_attempts AS attempts
                JOIN navigation_tasks AS tasks ON tasks.task_id = attempts.task_id
                WHERE attempts.task_id = ? AND attempts.request_id = ?
                """,
                (task, request),
            ).fetchone()
            if existing_attempt is not None:
                self._validate_existing_admission(
                    existing_attempt,
                    attempt_kind=attempt_kind,
                    payload_json=payload_json,
                    target_json=target_json,
                    product_fingerprint=product_fingerprint,
                    map_identity=map_identity,
                    map_json=map_json,
                    require_context_match=False,
                )

                record = self._task_record(task)
                assert record is not None
                return {"record": record, "replay": True}

            existing_task = self._conn.execute(
                """
                SELECT target_json, product_fingerprint, map_identity_json, terminal
                FROM navigation_tasks WHERE task_id = ?
                """,
                (task,),
            ).fetchone()
            if existing_task is None:
                initial_state = "admitted" if attempt_kind == "goal" else "unknown"
                observed_only = int(attempt_kind != "goal")
                self._conn.execute(
                    """
                    INSERT INTO navigation_tasks (
                        task_id, state, source, observed_only, target_json, product_fingerprint,
                        map_identity_json, created_at, updated_at
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        task,
                        initial_state,
                        str(source or "").strip(),
                        observed_only,
                        target_json,
                        str(product_fingerprint or "").strip(),
                        map_json,
                        now,
                        now,
                    ),
                )
            else:
                if bool(existing_task["terminal"]):
                    raise TaskLedgerConflict("terminal task cannot accept a new attempt")
                if attempt_kind == "goal" and existing_task["target_json"] != target_json:
                    raise TaskLedgerConflict("task_id was already admitted with a different target")
                expected_fingerprint = existing_task["product_fingerprint"]
                supplied_fingerprint = str(product_fingerprint or "").strip()
                if supplied_fingerprint and supplied_fingerprint != expected_fingerprint:
                    raise TaskLedgerConflict("task_id was already admitted under a different Product")
                if map_identity is not None and existing_task["map_identity_json"] != map_json:
                    raise TaskLedgerConflict("task_id was already admitted against a different map")

            self._conn.execute(
                """
                INSERT INTO navigation_task_attempts (
                    task_id, request_id, kind, payload_json, created_at, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?)
                """,
                (task, request, attempt_kind, payload_json, now, now),
            )
            self._insert_event(
                task,
                request,
                event_type="attempt_admitted",
                state="admitted",
                reason="",
                evidence={"kind": attempt_kind, "payload": _decode_json(payload_json)},
                created_at=now,
                dedupe_key=f"admit:{request}",
            )
            record = self._task_record(task)
            assert record is not None
            return {"record": record, "replay": False}

    def record_admission_result(
        self,
        task_id: str,
        request_id: str,
        *,
        accepted: bool,
        reason: str = "",
        endpoint_boot_id: str = "",
    ) -> dict[str, Any]:
        """Record command admission without claiming execution or cancellation."""

        task = _required_identity(task_id, "task_id")
        request = _required_identity(request_id, "request_id")
        if not isinstance(accepted, bool):
            raise ValueError("accepted must be a boolean")
        result_reason = str(reason or "").strip()
        boot_id = str(endpoint_boot_id or "").strip()
        now = float(self._clock())

        with self._lock, self._conn:
            self._require_open()
            attempt = self._conn.execute(
                """
                SELECT kind, payload_json, accepted, reason, endpoint_boot_id
                FROM navigation_task_attempts
                WHERE task_id = ? AND request_id = ?
                """,
                (task, request),
            ).fetchone()
            if attempt is None:
                raise KeyError(f"unknown navigation attempt {task}/{request}")
            if attempt["accepted"] is not None:
                if (
                    bool(attempt["accepted"]) != accepted
                    or attempt["reason"] != result_reason
                    or attempt["endpoint_boot_id"] != boot_id
                ):
                    raise TaskLedgerConflict("admission result conflicts with the recorded result")
                record = self._task_record(task)
                assert record is not None
                return record

            task_row = self._conn.execute(
                "SELECT state, terminal FROM navigation_tasks WHERE task_id = ?",
                (task,),
            ).fetchone()
            assert task_row is not None
            if bool(task_row["terminal"]):
                raise TaskLedgerConflict("terminal task cannot accept admission evidence")

            attempt_state = "accepted" if accepted else "rejected"
            self._conn.execute(
                """
                UPDATE navigation_task_attempts
                SET state = ?, accepted = ?, reason = ?, endpoint_boot_id = ?,
                    updated_at = ?
                WHERE task_id = ? AND request_id = ?
                """,
                (
                    attempt_state,
                    int(accepted),
                    result_reason,
                    boot_id,
                    now,
                    task,
                    request,
                ),
            )

            kind = str(attempt["kind"])
            event_state = attempt_state
            if kind == "goal":
                if accepted:
                    self._conn.execute(
                        """
                        UPDATE navigation_tasks
                        SET state = 'accepted', reason = ?, endpoint_boot_id = ?,
                            active_request_id = ?, updated_at = ?, can_resume = 0
                        WHERE task_id = ?
                        """,
                        (result_reason, boot_id, request, now, task),
                    )
                else:
                    self._conn.execute(
                        """
                        UPDATE navigation_tasks
                        SET state = 'rejected', terminal = 1, reason = ?,
                            endpoint_boot_id = ?, active_request_id = ?,
                            updated_at = ?, terminal_at = ?, can_resume = 0
                        WHERE task_id = ?
                        """,
                        (result_reason, boot_id, request, now, now, task),
                    )
            elif kind == "cancel" and accepted:
                event_state = "cancel_requested"
                payload = _decode_json(attempt["payload_json"])
                if isinstance(payload, dict) and "reason" in payload:
                    cancel_reason = str(payload.get("reason") or "").strip()
                else:
                    cancel_reason = result_reason

                self._conn.execute(
                    """
                    UPDATE navigation_tasks
                    SET cancel_requested_at = ?, cancel_request_id = ?,
                        cancel_reason = ?, endpoint_boot_id = ?, updated_at = ?,
                        can_resume = 0
                    WHERE task_id = ?
                    """,
                    (now, request, cancel_reason, boot_id, now, task),
                )

            self._insert_event(
                task,
                request,
                event_type="admission_result",
                state=event_state,
                reason=result_reason,
                evidence={
                    "accepted": accepted,
                    "kind": kind,
                    "endpoint_boot_id": boot_id,
                },
                created_at=now,
                dedupe_key=f"admission:{request}",
            )
            record = self._task_record(task)
            assert record is not None
            return record

    def record_accepted(
        self,
        task_id: str,
        request_id: str,
        reason: str = "",
        endpoint_boot_id: str = "",
    ) -> dict[str, Any]:
        """Record a positive local or native admission decision."""

        return self.record_admission_result(
            task_id,
            request_id,
            accepted=True,
            reason=reason,
            endpoint_boot_id=endpoint_boot_id,
        )

    def record_rejected(
        self,
        task_id: str,
        request_id: str,
        reason: str,
    ) -> dict[str, Any]:
        """Record a fail-closed admission rejection."""

        return self.record_admission_result(
            task_id,
            request_id,
            accepted=False,
            reason=reason,
        )

    def _resolve_request_attempt(
        self,
        task_id: str,
        native_request_id: str,
    ) -> str:
        rows = self._conn.execute(
            """
            SELECT request_id, native_request_id
            FROM navigation_task_attempts
            WHERE task_id = ?
            """,
            (task_id,),
        ).fetchall()
        exact = [
            str(row["request_id"])
            for row in rows
            if row["request_id"] == native_request_id or row["native_request_id"] == native_request_id
        ]
        if len(set(exact)) == 1:
            return exact[0]
        if len(set(exact)) > 1:
            raise TaskLedgerConflict("native request identity maps to multiple logical attempts")
        correlated = [
            str(row["request_id"]) for row in rows if native_request_id.startswith(f"{row['request_id']}-clock-retry-")
        ]
        if correlated:
            longest = max(len(item) for item in correlated)
            matches = {item for item in correlated if len(item) == longest}
            if len(matches) == 1:
                return matches.pop()
        raise KeyError(f"unknown navigation attempt {task_id}/{native_request_id}")

    def _resolve_or_create_observed_attempt(
        self,
        task_id: str,
        native_request_id: str,
        *,
        boot_id: str,
        timestamp: float,
    ) -> str:
        try:
            return self._resolve_request_attempt(task_id, native_request_id)
        except KeyError:
            task = self._conn.execute(
                "SELECT observed_only FROM navigation_tasks WHERE task_id = ?",
                (task_id,),
            ).fetchone()
            if task is None or not bool(task["observed_only"]):
                raise
            self._conn.execute(
                """
                INSERT INTO navigation_task_attempts (
                    task_id, request_id, native_request_id, kind, payload_json,
                    state, accepted, reason, endpoint_boot_id, created_at, updated_at
                ) VALUES (?, ?, ?, 'goal', '{}', 'accepted', 1, ?, ?, ?, ?)
                """,
                (
                    task_id,
                    native_request_id,
                    native_request_id,
                    "observed_from_endpoint",
                    boot_id,
                    timestamp,
                    timestamp,
                ),
            )
            self._insert_event(
                task_id,
                native_request_id,
                event_type="attempt_observed",
                state="accepted",
                reason="observed_from_endpoint",
                evidence={
                    "native_request_id": native_request_id,
                    "endpoint_boot_id": boot_id,
                },
                created_at=timestamp,
                dedupe_key=f"observed_attempt:{native_request_id}",
            )
            return native_request_id

    def record_native_ack(
        self,
        receipt: Any,
        *,
        endpoint_boot_id: str = "",
    ) -> dict[str, Any]:
        """Persist a typed native admission receipt and its exact evidence."""

        ack = _evidence_dict(receipt, "native_ack")
        task = _required_identity(ack.get("task_id"), "native_ack.task_id")
        native_request = _required_identity(
            ack.get("request_id"),
            "native_ack.request_id",
        )
        kind = _command_kind(ack.get("kind"))
        accepted = ack.get("accepted")
        if not isinstance(accepted, bool):
            raise ValueError("native_ack.accepted must be a boolean")
        reason = str(ack.get("reason") or "").strip()
        boot_id = str(endpoint_boot_id or ack.get("endpoint_boot_id") or "").strip()
        try:
            endpoint_timestamp = float(ack.get("endpoint_timestamp_s"))
        except (TypeError, ValueError) as exc:
            raise ValueError("native_ack.endpoint_timestamp_s must be finite and non-negative") from exc
        if not math.isfinite(endpoint_timestamp) or endpoint_timestamp < 0.0:
            raise ValueError("native_ack.endpoint_timestamp_s must be finite and non-negative")
        normalized = dict(ack)
        normalized["accepted"] = accepted
        normalized["kind"] = kind
        normalized["task_id"] = task
        normalized["request_id"] = native_request
        normalized["endpoint_timestamp_s"] = endpoint_timestamp
        normalized["reason"] = reason
        if boot_id:
            normalized["endpoint_boot_id"] = boot_id
        ack_json = _canonical_json(normalized, label="native_ack")

        with self._lock:
            self._require_open()
            request = self._resolve_request_attempt(task, native_request)
            attempt = self._conn.execute(
                """
                SELECT kind, native_request_id, native_ack_json
                FROM navigation_task_attempts
                WHERE task_id = ? AND request_id = ?
                """,
                (task, request),
            ).fetchone()
            assert attempt is not None
            if attempt["kind"] != kind:
                raise TaskLedgerConflict("native ACK kind does not match the admitted attempt")
            existing_native_request = str(attempt["native_request_id"] or "")
            if existing_native_request and existing_native_request != native_request:
                raise TaskLedgerConflict("logical attempt already has a different native request identity")
            existing_ack = attempt["native_ack_json"]
            if existing_ack is not None and existing_ack != ack_json:
                raise TaskLedgerConflict("native ACK conflicts with the recorded receipt")

            self.record_admission_result(
                task,
                request,
                accepted=accepted,
                reason=reason,
                endpoint_boot_id=boot_id,
            )
            with self._conn:
                self._conn.execute(
                    """
                    UPDATE navigation_task_attempts
                    SET native_request_id = ?, native_ack_json = ?, updated_at = ?
                    WHERE task_id = ? AND request_id = ?
                    """,
                    (
                        native_request,
                        ack_json,
                        float(self._clock()),
                        task,
                        request,
                    ),
                )
            record = self._task_record(task)
            assert record is not None
            return record

    def record_goal_status(self, evidence: Any) -> dict[str, Any]:
        """Apply one authoritative native task-lifecycle event."""

        status = _evidence_dict(evidence, "goal_status")
        task = _required_identity(status.get("task_id"), "goal_status.task_id")
        native_request = _required_identity(
            status.get("request_id"),
            "goal_status.request_id",
        )
        boot_id = _required_identity(status.get("boot_id"), "goal_status.boot_id")
        try:
            sequence = int(status.get("sequence"))
        except (TypeError, ValueError) as exc:
            raise ValueError("goal_status.sequence must be positive") from exc
        if sequence <= 0:
            raise ValueError("goal_status.sequence must be positive")
        timestamp = _positive_timestamp(
            status.get("timestamp_s", status.get("ts")),
            "goal_status timestamp",
        )
        state = _goal_status_state(status.get("state"))
        reason = str(status.get("reason") or "").strip()
        normalized = dict(status)
        normalized["timestamp_s"] = timestamp
        normalized.pop("ts", None)
        normalized["boot_id"] = boot_id
        normalized["sequence"] = sequence
        normalized["task_id"] = task
        normalized["request_id"] = native_request
        normalized["state"] = state
        normalized["reason"] = reason
        status_json = _canonical_json(normalized, label="goal_status")
        now = float(self._clock())

        with self._lock, self._conn:
            self._require_open()
            task_row = self._conn.execute(
                """
                SELECT state, terminal, endpoint_boot_id, last_goal_status_json
                FROM navigation_tasks WHERE task_id = ?
                """,
                (task,),
            ).fetchone()
            if task_row is None:
                raise KeyError(f"unknown navigation task {task}")
            request = self._resolve_or_create_observed_attempt(
                task,
                native_request,
                boot_id=boot_id,
                timestamp=timestamp,
            )

            previous_json = task_row["last_goal_status_json"]
            previous = _decode_json(previous_json) if previous_json else None
            if bool(task_row["terminal"]):
                if task_row["state"] == state:
                    record = self._task_record(task)
                    assert record is not None
                    return record
                raise TaskLedgerConflict(f"terminal task is already {task_row['state']}; cannot become {state}")

            if previous is not None and previous.get("boot_id") == boot_id:
                previous_sequence = int(previous.get("sequence", 0))
                if sequence < previous_sequence:
                    record = self._task_record(task)
                    assert record is not None
                    return record
                if sequence == previous_sequence:
                    if previous_json != status_json:
                        raise TaskLedgerConflict("goal status sequence was reused with different content")
                    record = self._task_record(task)
                    assert record is not None
                    return record
            previous_boot = str(task_row["endpoint_boot_id"] or "")
            if previous_boot and previous_boot != boot_id and state not in _TERMINAL_STATES:
                raise TaskLedgerConflict("endpoint boot changed; reconcile the open task before applying active status")

            current_state = str(task_row["state"])
            if (
                state not in _TERMINAL_STATES
                and current_state in _NONTERMINAL_PROGRESS
                and _NONTERMINAL_PROGRESS[state] < _NONTERMINAL_PROGRESS[current_state]
                and not (current_state == "recovering" and state == "executing")
            ):
                raise TaskLedgerConflict(f"goal status would regress {current_state} to {state}")

            terminal = state in _TERMINAL_STATES
            self._conn.execute(
                """
                UPDATE navigation_tasks
                SET state = ?, terminal = ?, reason = ?, endpoint_boot_id = ?,
                    active_request_id = ?, updated_at = ?, terminal_at = ?,
                    can_resume = 0, last_goal_status_json = ?
                WHERE task_id = ?
                """,
                (
                    state,
                    int(terminal),
                    reason,
                    boot_id,
                    request,
                    now,
                    timestamp if terminal else None,
                    status_json,
                    task,
                ),
            )
            self._conn.execute(
                """
                UPDATE navigation_task_attempts
                SET state = ?, reason = ?, endpoint_boot_id = ?, updated_at = ?
                WHERE task_id = ? AND request_id = ?
                """,
                (state, reason, boot_id, now, task, request),
            )
            self._insert_event(
                task,
                request,
                event_type="goal_status",
                state=state,
                reason=reason,
                evidence=normalized,
                created_at=timestamp,
                dedupe_key=f"goal_status:{boot_id}:{sequence}",
            )
            record = self._task_record(task)
            assert record is not None
            return record

    def record_navigation_state(self, evidence: Any) -> dict[str, Any] | None:
        """Apply one complete authoritative endpoint state snapshot."""

        snapshot = _evidence_dict(evidence, "navigation_state")
        boot_id = _required_identity(snapshot.get("boot_id"), "navigation_state.boot_id")
        try:
            sequence = int(snapshot.get("sequence"))
        except (TypeError, ValueError) as exc:
            raise ValueError("navigation_state.sequence must be positive") from exc
        if sequence <= 0:
            raise ValueError("navigation_state.sequence must be positive")
        timestamp = _positive_timestamp(
            snapshot.get("timestamp_s", snapshot.get("ts")),
            "navigation_state timestamp",
        )
        lifecycle = _navigation_lifecycle_state(snapshot.get("lifecycle_state"))
        active_task = str(snapshot.get("active_task_id") or "").strip()
        native_request = str(snapshot.get("active_request_id") or "").strip()
        if bool(active_task) != bool(native_request):
            raise ValueError("navigation_state active task and request identities must be present together")
        normalized = dict(snapshot)
        normalized["timestamp_s"] = timestamp
        normalized.pop("ts", None)
        normalized["boot_id"] = boot_id
        normalized["sequence"] = sequence
        normalized["lifecycle_state"] = lifecycle
        normalized["active_task_id"] = active_task
        normalized["active_request_id"] = native_request
        state_json = _canonical_json(normalized, label="navigation_state")
        if not active_task:
            return None

        with self._lock, self._conn:
            self._require_open()
            task_row = self._conn.execute(
                """
                SELECT state, terminal, endpoint_boot_id,
                       last_navigation_state_json
                FROM navigation_tasks WHERE task_id = ?
                """,
                (active_task,),
            ).fetchone()
            if task_row is None:
                return None
            request = self._resolve_or_create_observed_attempt(
                active_task,
                native_request,
                boot_id=boot_id,
                timestamp=timestamp,
            )
            previous_json = task_row["last_navigation_state_json"]
            previous = _decode_json(previous_json) if previous_json else None
            if previous is not None and previous.get("boot_id") == boot_id:
                previous_sequence = int(previous.get("sequence", 0))
                if sequence < previous_sequence:
                    return self._task_record(active_task)
                if sequence == previous_sequence:
                    if previous_json != state_json:
                        raise TaskLedgerConflict("navigation state sequence was reused with different content")
                    return self._task_record(active_task)

            previous_boot = str(task_row["endpoint_boot_id"] or "")
            if previous_boot and previous_boot != boot_id:
                raise TaskLedgerConflict("endpoint boot changed; reconcile the open task before applying active state")

            current_state = str(task_row["state"])
            target_state = lifecycle
            reason = str(snapshot.get("failure_code") or snapshot.get("hold_reason") or lifecycle).strip()
            if lifecycle == "paused":
                target_state = current_state
            elif lifecycle == "idle":
                target_state = "unknown"
                reason = "endpoint_state_inconsistent"

            if bool(task_row["terminal"]):
                if current_state == target_state:
                    record = self._task_record(active_task)
                    assert record is not None
                    return record
                raise TaskLedgerConflict(f"terminal task is already {current_state}; cannot become {target_state}")
            if (
                target_state not in _TERMINAL_STATES
                and target_state in _NONTERMINAL_PROGRESS
                and current_state in _NONTERMINAL_PROGRESS
                and _NONTERMINAL_PROGRESS[target_state] < _NONTERMINAL_PROGRESS[current_state]
                and not (current_state == "recovering" and target_state == "executing")
            ):
                raise TaskLedgerConflict(f"navigation state would regress {current_state} to {target_state}")

            terminal = target_state in _TERMINAL_STATES
            now = float(self._clock())
            self._conn.execute(
                """
                UPDATE navigation_tasks
                SET state = ?, terminal = ?, reason = ?, endpoint_boot_id = ?,
                    active_request_id = ?, updated_at = ?, terminal_at = ?,
                    can_resume = 0, last_navigation_state_json = ?
                WHERE task_id = ?
                """,
                (
                    target_state,
                    int(terminal),
                    reason,
                    boot_id,
                    request,
                    now,
                    timestamp if terminal else None,
                    state_json,
                    active_task,
                ),
            )
            self._conn.execute(
                """
                UPDATE navigation_task_attempts
                SET state = ?, reason = ?, endpoint_boot_id = ?, updated_at = ?
                WHERE task_id = ? AND request_id = ?
                """,
                (target_state, reason, boot_id, now, active_task, request),
            )
            self._insert_event(
                active_task,
                request,
                event_type="navigation_state",
                state=target_state,
                reason=reason,
                evidence=normalized,
                created_at=timestamp,
                dedupe_key=f"navigation_state:{boot_id}:{sequence}",
            )
            record = self._task_record(active_task)
            assert record is not None
            return record

    def reconcile_endpoint(
        self,
        evidence: Any | None,
        *,
        goal_statuses: Any | None = None,
    ) -> list[dict[str, Any]]:
        """Reconcile open tasks from endpoint evidence without dispatching motion."""

        open_ids = [item["task_id"] for item in self.list_open()]
        retained_items = None if goal_statuses is None else list(goal_statuses)
        retained_task_ids: set[str] = set()
        if retained_items is not None:
            for status in retained_items:
                status_dict = _evidence_dict(status, "goal_status")
                status_task = str(status_dict.get("task_id") or "").strip()
                if status_task:
                    retained_task_ids.add(status_task)
                try:
                    self.record_goal_status(status_dict)
                except KeyError:
                    continue
                except TaskLedgerConflict as exc:
                    if "endpoint boot changed" not in str(exc):
                        raise

        if evidence is None:
            for task_id in open_ids:
                self._mark_unknown(
                    task_id,
                    "endpoint_status_unavailable",
                    evidence={},
                )
            return [record for task_id in open_ids if (record := self.get_task(task_id)) is not None]

        snapshot = _evidence_dict(evidence, "navigation_state")
        boot_id = _required_identity(snapshot.get("boot_id"), "navigation_state.boot_id")
        active_task = str(snapshot.get("active_task_id") or "").strip()
        active_request = str(snapshot.get("active_request_id") or "").strip()
        if bool(active_task) != bool(active_request):
            raise ValueError("navigation_state active task and request identities must be present together")
        timestamp_value = snapshot.get("timestamp_s", snapshot.get("ts"))
        timestamp = (
            _positive_timestamp(timestamp_value, "navigation_state timestamp")
            if timestamp_value is not None
            else float(self._clock())
        )
        complete_state = (
            timestamp_value is not None
            and snapshot.get("sequence") is not None
            and snapshot.get("lifecycle_state") is not None
        )

        for task_id in open_ids:
            task = self.get_task(task_id)
            if task is None or task["terminal"]:
                continue
            previous_boot = str(task["endpoint_boot_id"] or "")
            if previous_boot and previous_boot != boot_id:
                self._interrupt_task(
                    task_id,
                    "endpoint_restarted",
                    evidence=snapshot,
                    terminal_at=timestamp,
                )
                continue

            if active_task == task_id:
                if complete_state:
                    self.record_navigation_state(snapshot)
                else:
                    with self._lock, self._conn:
                        request = self._resolve_or_create_observed_attempt(
                            task_id,
                            active_request,
                            boot_id=boot_id,
                            timestamp=timestamp,
                        )
                        self._conn.execute(
                            """
                            UPDATE navigation_tasks
                            SET endpoint_boot_id = ?, active_request_id = ?,
                                can_resume = 0, updated_at = ?,
                                last_navigation_state_json = ?
                            WHERE task_id = ? AND terminal = 0
                            """,
                            (
                                boot_id,
                                request,
                                float(self._clock()),
                                _canonical_json(snapshot, label="navigation_state"),
                                task_id,
                            ),
                        )
                continue

            if retained_items is None:
                self._mark_unknown(
                    task_id,
                    "endpoint_task_not_confirmed",
                    evidence=snapshot,
                )
            elif task_id in retained_task_ids:
                self._mark_unknown(
                    task_id,
                    "endpoint_evidence_conflict",
                    evidence=snapshot,
                )
            else:
                self._interrupt_task(
                    task_id,
                    "endpoint_task_not_active",
                    evidence=snapshot,
                    terminal_at=timestamp,
                )

        return [record for task_id in open_ids if (record := self.get_task(task_id)) is not None]

    def _mark_unknown(
        self,
        task_id: str,
        reason: str,
        *,
        evidence: Any,
    ) -> dict[str, Any] | None:
        now = float(self._clock())
        evidence_json = _canonical_json(evidence, label="reconcile evidence")
        evidence_dict = _decode_json(evidence_json)
        boot_id = str(evidence_dict.get("boot_id") or "")
        sequence = str(evidence_dict.get("sequence") or "")
        dedupe_key = f"reconcile:{reason}:{boot_id}:{sequence}"
        with self._lock, self._conn:
            row = self._conn.execute(
                """
                SELECT state, terminal, reason
                FROM navigation_tasks WHERE task_id = ?
                """,
                (task_id,),
            ).fetchone()
            if row is None:
                return None
            if bool(row["terminal"]):
                return self._task_record(task_id)
            duplicate = self._conn.execute(
                """
                SELECT 1 FROM navigation_task_events
                WHERE task_id = ? AND dedupe_key = ?
                """,
                (task_id, dedupe_key),
            ).fetchone()
            if row["state"] == "unknown" and row["reason"] == reason and duplicate:
                return self._task_record(task_id)

            self._conn.execute(
                """
                UPDATE navigation_tasks
                SET state = 'unknown', reason = ?, can_resume = 0,
                    updated_at = ?,
                    last_navigation_state_json = COALESCE(?, last_navigation_state_json)
                WHERE task_id = ?
                """,
                (reason, now, evidence_json if evidence_dict else None, task_id),
            )
            self._insert_event(
                task_id,
                "",
                event_type="reconcile",
                state="unknown",
                reason=reason,
                evidence=evidence_dict,
                created_at=now,
                dedupe_key=dedupe_key,
            )
            return self._task_record(task_id)

    def _interrupt_task(
        self,
        task_id: str,
        reason: str,
        *,
        evidence: Any,
        terminal_at: float,
    ) -> dict[str, Any] | None:
        now = float(self._clock())
        evidence_json = _canonical_json(evidence, label="reconcile evidence")
        evidence_dict = _decode_json(evidence_json)
        with self._lock, self._conn:
            row = self._conn.execute(
                """
                SELECT state, terminal, active_request_id
                FROM navigation_tasks WHERE task_id = ?
                """,
                (task_id,),
            ).fetchone()
            if row is None:
                return None
            if bool(row["terminal"]):
                if row["state"] != "interrupted":
                    raise TaskLedgerConflict(f"terminal task is already {row['state']}; cannot become interrupted")
                return self._task_record(task_id)
            active_request = str(row["active_request_id"] or "")
            self._conn.execute(
                """
                UPDATE navigation_tasks
                SET state = 'interrupted', terminal = 1, reason = ?,
                    can_resume = 0, updated_at = ?, terminal_at = ?,
                    last_navigation_state_json = ?
                WHERE task_id = ?
                """,
                (reason, now, terminal_at, evidence_json, task_id),
            )
            if active_request:
                self._conn.execute(
                    """
                    UPDATE navigation_task_attempts
                    SET state = 'interrupted', reason = ?, updated_at = ?
                    WHERE task_id = ? AND request_id = ?
                    """,
                    (reason, now, task_id, active_request),
                )

            self._insert_event(
                task_id,
                active_request,
                event_type="reconcile",
                state="interrupted",
                reason=reason,
                evidence=evidence_dict,
                created_at=terminal_at,
                dedupe_key=f"terminal:interrupted:{reason}",
            )
            return self._task_record(task_id)

    def list_recent(self, limit: int = 50) -> list[dict[str, Any]]:
        """Return the most recently updated task records."""

        if isinstance(limit, bool) or not isinstance(limit, int) or limit <= 0:
            raise ValueError("limit must be a positive integer")
        if limit > 1000:
            raise ValueError("limit cannot exceed 1000")
        with self._lock:
            self._require_open()
            rows = self._conn.execute(
                """
                SELECT task_id FROM navigation_tasks
                ORDER BY updated_at DESC, task_id DESC LIMIT ?
                """,
                (limit,),
            ).fetchall()
            return [record for row in rows if (record := self._task_record(str(row["task_id"]))) is not None]

    def list_open(self) -> list[dict[str, Any]]:
        """Return every nonterminal task without a recent-list limit."""

        with self._lock:
            self._require_open()
            rows = self._conn.execute(
                """
                SELECT task_id FROM navigation_tasks
                WHERE terminal = 0 ORDER BY updated_at DESC, task_id DESC
                """
            ).fetchall()
            return [record for row in rows if (record := self._task_record(str(row["task_id"]))) is not None]

    def get_task(self, task_id: str) -> dict[str, Any] | None:
        task = _required_identity(task_id, "task_id")
        with self._lock:
            self._require_open()
            return self._task_record(task)

    def _task_record(self, task_id: str) -> dict[str, Any] | None:
        row = self._conn.execute(
            "SELECT * FROM navigation_tasks WHERE task_id = ?",
            (task_id,),
        ).fetchone()
        if row is None:
            return None
        attempts = self._conn.execute(
            """
            SELECT request_id, native_request_id, kind, payload_json, state,
                   accepted, reason, endpoint_boot_id, native_ack_json, created_at, updated_at
            FROM navigation_task_attempts
            WHERE task_id = ? ORDER BY created_at, request_id
            """,
            (task_id,),
        ).fetchall()
        events = self._conn.execute(
            """
            SELECT id, request_id, event_type, state, reason, evidence_json, created_at
            FROM navigation_task_events
            WHERE task_id = ? ORDER BY id
            """,
            (task_id,),
        ).fetchall()
        return {
            "task_id": row["task_id"],
            "state": row["state"],
            "terminal": bool(row["terminal"]),
            "reason": row["reason"],
            "observed_only": bool(row["observed_only"]),
            "source": row["source"],
            "target": _decode_json(row["target_json"]),
            "product_fingerprint": row["product_fingerprint"],
            "map_identity": _decode_json(row["map_identity_json"]),
            "created_at": float(row["created_at"]),
            "updated_at": float(row["updated_at"]),
            "terminal_at": (float(row["terminal_at"]) if row["terminal_at"] is not None else None),
            "endpoint_boot_id": row["endpoint_boot_id"],
            "active_request_id": row["active_request_id"],
            "cancel_requested": row["cancel_requested_at"] is not None,
            "cancel_requested_at": (
                float(row["cancel_requested_at"]) if row["cancel_requested_at"] is not None else None
            ),
            "cancel_request_id": row["cancel_request_id"],
            "cancel_reason": row["cancel_reason"],
            "can_resume": bool(row["can_resume"]),
            "last_goal_status": (_decode_json(row["last_goal_status_json"]) if row["last_goal_status_json"] else None),
            "last_navigation_state": (
                _decode_json(row["last_navigation_state_json"]) if row["last_navigation_state_json"] else None
            ),
            "attempts": [
                {
                    "request_id": item["request_id"],
                    "native_request_id": item["native_request_id"],
                    "kind": item["kind"],
                    "payload": _decode_json(item["payload_json"]),
                    "state": item["state"],
                    "accepted": (bool(item["accepted"]) if item["accepted"] is not None else None),
                    "reason": item["reason"],
                    "endpoint_boot_id": item["endpoint_boot_id"],
                    "native_ack": (_decode_json(item["native_ack_json"]) if item["native_ack_json"] else None),
                    "created_at": float(item["created_at"]),
                    "updated_at": float(item["updated_at"]),
                }
                for item in attempts
            ],
            "events": [
                {
                    "id": int(item["id"]),
                    "request_id": item["request_id"],
                    "type": item["event_type"],
                    "state": item["state"],
                    "reason": item["reason"],
                    "evidence": _decode_json(item["evidence_json"]),
                    "created_at": float(item["created_at"]),
                }
                for item in events
            ],
        }

    def _insert_event(
        self,
        task_id: str,
        request_id: str,
        *,
        event_type: str,
        state: str,
        reason: str,
        evidence: Any,
        created_at: float,
        dedupe_key: str,
    ) -> None:
        self._conn.execute(
            """
            INSERT OR IGNORE INTO navigation_task_events (
                task_id, request_id, event_type, state, reason,
                evidence_json, created_at, dedupe_key
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                task_id,
                request_id,
                event_type,
                state,
                reason,
                _canonical_json(evidence, label="evidence"),
                created_at,
                dedupe_key,
            ),
        )

    def close(self) -> None:
        with self._lock:
            if not self._closed:
                self._conn.close()
                self._closed = True

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("navigation task ledger is closed")

    def __enter__(self) -> "NavigationTaskLedger":
        return self

    def __exit__(self, *args: object) -> None:
        self.close()
