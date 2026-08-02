"""Durable projection of native Explore run facts.

The native Explore endpoint owns execution state.  Gateway reserves one
``exploration_run_id`` before dispatch and persists the ordered native facts so
an operator can query the run after an HTTP or Host restart.  Command ACKs only
record admission; they never manufacture execution or terminal state.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import stat
import threading
import time
from collections import OrderedDict
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

_SCHEMA = "lingtu.explore.run.journal.v1"
_PUBLIC_SCHEMA = "lingtu.explore.run.v1"
_MAX_JOURNAL_BYTES = 4 * 1024 * 1024
_DEFAULT_RETENTION = 128
_ULID_ALPHABET = "0123456789ABCDEFGHJKMNPQRSTVWXYZ"
_ULID_PATTERN = re.compile(r"^[0-7][0-9A-HJKMNP-TV-Z]{25}$")

_STATE_BY_VALUE = {
    1: "admitted",
    2: "running",
    3: "pausing",
    4: "paused",
    5: "cancelling",
    6: "completed",
    7: "cancelled",
    8: "failed",
}
_KNOWN_STATES = frozenset(_STATE_BY_VALUE.values())
_TERMINAL_STATES = frozenset({"completed", "cancelled", "failed"})
_STOP_CONFIRMED_STATES = _TERMINAL_STATES | {"paused"}
_EVENT_KINDS = frozenset({1, 2, 3})


class ExploreRunConflict(ValueError):
    """An identity or idempotency key conflicts with persisted history."""


class ExploreRunJournalUnavailable(RuntimeError):
    """The configured durable projection cannot be trusted or updated."""


def _required_text(value: object, label: str, *, max_length: int = 256) -> str:
    text = str(value or "").strip()
    if not text:
        raise ValueError(f"{label} is required")
    if len(text) > max_length or any(ord(ch) < 0x20 for ch in text):
        raise ValueError(f"{label} is invalid")
    return text


def _new_ulid() -> str:
    """Return a dependency-free canonical 26-character uppercase ULID."""

    timestamp_ms = int(time.time_ns() // 1_000_000) & ((1 << 48) - 1)
    value = (timestamp_ms << 80) | int.from_bytes(os.urandom(10), "big")
    encoded = ["0"] * 26
    for index in range(25, -1, -1):
        encoded[index] = _ULID_ALPHABET[value & 0x1F]
        value >>= 5
    return "".join(encoded)


def new_request_id() -> str:
    """Generate the canonical server-side ID for a bodyless HTTP request."""

    return _new_ulid()


def _canonical_ulid(value: object, label: str) -> str:
    text = str(value or "").strip()
    if not _ULID_PATTERN.fullmatch(text):
        raise ValueError(f"{label} must be a canonical 26-character uppercase ULID")
    return text


def _json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def _json_object(value: object, label: str) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be an object")
    decoded = json.loads(_json_bytes(dict(value)))
    if not isinstance(decoded, dict):
        raise ValueError(f"{label} must be an object")
    return decoded


def _event_object(value: object) -> dict[str, Any]:
    if isinstance(value, Mapping):
        event = _json_object(value, "native event")
    else:
        converter = getattr(value, "to_dict", None)
        if not callable(converter):
            raise ValueError("native event must be an object or expose to_dict()")
        event = _json_object(converter(), "native event")
    if "ts" in event:
        alias = event.pop("ts")
        if "timestamp_s" in event:
            try:
                same_timestamp = float(event["timestamp_s"]) == float(alias)
            except (TypeError, ValueError):
                same_timestamp = False
            if not same_timestamp:
                raise ValueError("native event has conflicting timestamp fields")
        else:
            event["timestamp_s"] = alias
    return event


def _state_name(value: object) -> str:
    if isinstance(value, bool):
        raise ValueError("native exploration state is invalid")
    if isinstance(value, int):
        try:
            return _STATE_BY_VALUE[value]
        except KeyError as exc:
            raise ValueError(f"unsupported native exploration state {value!r}") from exc
    state = str(value or "").strip().lower()
    if state not in _KNOWN_STATES:
        raise ValueError(f"unsupported native exploration state {state!r}")
    return state


def _event_kind(value: object, *, state: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value not in _EVENT_KINDS:
        raise ValueError("native exploration event kind is invalid")
    if (value == 1) != (state == "admitted"):
        raise ValueError("native exploration event kind/state pair is invalid")
    if value == 3 and state not in {"pausing", "cancelling"}:
        raise ValueError("native exploration event kind/state pair is invalid")
    return value


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(
        f"{path.name}.tmp-{os.getpid()}-{threading.get_ident()}"
    )
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        if os.name != "nt":
            directory = os.open(path.parent, os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
    except BaseException:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise


class ExploreRuns:
    """Bounded run history whose lifecycle facts only come from native events."""

    def __init__(
        self,
        *,
        journal_path: str | os.PathLike[str] | None = None,
        retention: int = _DEFAULT_RETENTION,
        id_factory: Callable[[], str] = _new_ulid,
    ) -> None:
        self._lock = threading.RLock()
        self._retention = max(1, int(retention))
        self._id_factory = id_factory
        self._journal_path = Path(journal_path).expanduser() if journal_path else None
        self._journal_status = "disabled" if self._journal_path is None else "ready"
        self._journal_error = ""
        self._runs: OrderedDict[str, dict[str, Any]] = OrderedDict()
        self._requests: dict[str, str] = {}
        self._boot_sequences: dict[str, int] = {}
        self._event_gap = ""
        if self._journal_path is not None:
            self._restore()

    def reserve_start(
        self,
        request_id: str,
        *,
        product_session_id: str,
        route: str,
        map: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Persist one start admission before any native side effect."""
        request = _required_text(request_id, "request_id", max_length=128)
        session = _required_text(product_session_id, "product_session_id", max_length=128)
        normalized_route = str(route or "").strip().lower()
        if normalized_route not in {"live", "map"}:
            raise ValueError("route must be live or map")
        map_identity = _json_object(map, "map") if map is not None else None
        if normalized_route == "map" and not map_identity:
            raise ValueError("map route requires map identity")
        if normalized_route == "live" and map_identity:
            raise ValueError("live route cannot bind a saved map")
        identity = {
            "product_session_id": session,
            "route": normalized_route,
            "map": map_identity,
        }
        with self._lock:
            self._require_available()
            existing_run_id = self._requests.get(request)
            if existing_run_id is not None:
                record = self._runs[existing_run_id]
                if record["identity"] != identity:
                    raise ExploreRunConflict(
                        "request_id was already used for a different exploration context"
                    )
                self._runs.move_to_end(existing_run_id)
                return {"run": self._snapshot(record), "replay": True}

            exploration_run_id = ""
            for _ in range(8):
                candidate = _canonical_ulid(
                    self._id_factory(), "generated exploration_run_id"
                )
                if candidate not in self._runs:
                    exploration_run_id = candidate
                    break
            if not exploration_run_id:
                raise ExploreRunJournalUnavailable(
                    "could not allocate a unique exploration_run_id"
                )

            now = time.time()
            record = {
                "exploration_run_id": exploration_run_id,
                "start_request_id": request,
                "identity": identity,
                "admission": "pending",
                "state": "submitted",
                "state_source": "gateway_submission",
                "reason": "awaiting_native_ack",
                "terminal": False,
                "terminal_at": None,
                "can_resume": False,
                "motion_stop": {"confirmed": False, "reason": "not_observed"},
                "native_cursor": None,
                "last_native_event": None,
                "created_at": now,
                "updated_at": now,
            }
            self._runs[exploration_run_id] = record
            self._requests[request] = exploration_run_id
            self._trim()
            self._persist()
            return {"run": self._snapshot(record), "replay": False}

    def record_admission(
        self,
        exploration_run_id: str,
        *,
        accepted: bool,
        reason: str,
    ) -> dict[str, Any]:
        """Record only the native command ACK, not execution state."""
        run_id = _canonical_ulid(exploration_run_id, "exploration_run_id")
        if not isinstance(accepted, bool):
            raise ValueError("accepted must be a boolean")
        with self._lock:
            self._require_available()
            record = self._required_record(run_id)
            expected = "accepted" if accepted else "rejected"
            if record["admission"] not in {"pending", expected}:
                raise ExploreRunConflict("exploration admission conflicts with history")
            record["admission"] = expected
            record["reason"] = str(reason or expected)
            record["updated_at"] = time.time()
            if not accepted:
                record["state"] = "rejected"
                record["state_source"] = "native_exploration_ack"
                record["terminal"] = True
                record["terminal_at"] = record["updated_at"]
                record["motion_stop"] = {
                    "confirmed": True,
                    "reason": "motion_not_admitted",
                }
            self._persist()
            return self._snapshot(record)

    def observe_event(self, value: object) -> dict[str, Any]:
        """Project one ordered native lifecycle fact into durable query state."""
        event = _event_object(value)
        run_id = _canonical_ulid(
            event.get("exploration_run_id"), "native exploration_run_id"
        )
        boot_id = _required_text(event.get("boot_id"), "native boot_id", max_length=128)
        sequence = event.get("event_sequence")
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence <= 0:
            raise ValueError("native event_sequence must be a positive integer")
        state = _state_name(event.get("state"))
        kind = _event_kind(event.get("kind"), state=state)
        try:
            timestamp_s = float(event.get("timestamp_s"))
        except (TypeError, ValueError) as exc:
            raise ValueError("native exploration timestamp_s is invalid") from exc
        if not math.isfinite(timestamp_s) or timestamp_s <= 0.0:
            raise ValueError("native exploration timestamp_s is invalid")
        if _required_text(event.get("frame_id"), "native frame_id") != "map":
            raise ValueError("native exploration frame_id must be map")
        _required_text(event.get("reason"), "native reason", max_length=512)
        stop_confirmed = event.get("motion_stop_confirmed")
        if not isinstance(stop_confirmed, bool):
            raise ValueError("native motion_stop_confirmed must be a boolean")
        stop_reason = str(event.get("motion_stop_reason") or "").strip()
        if state in _STOP_CONFIRMED_STATES and not stop_confirmed:
            raise ValueError(
                f"native {state} event lacks confirmed motion-stop evidence"
            )
        if state in {"pausing", "cancelling"} and stop_confirmed:
            raise ValueError(f"native {state} event cannot claim confirmed motion stop")
        if stop_confirmed and not stop_reason:
            raise ValueError("native confirmed motion stop has no reason")
        if kind == 3:
            if stop_confirmed or not stop_reason:
                raise ValueError("native stop-confirmation failure event is invalid")
        elif not stop_confirmed and stop_reason:
            raise ValueError("native unconfirmed motion event has a stop reason")

        with self._lock:
            self._require_available()
            record = self._required_record(run_id)
            self._validate_event_binding(record, event)
            previous_sequence = self._boot_sequences.get(boot_id)
            if previous_sequence is not None and sequence <= previous_sequence:
                return self._snapshot(record)
            if record["terminal"]:
                raise ExploreRunConflict(
                    "exploration run is already terminal and cannot accept another event"
                )
            if previous_sequence is not None and sequence != previous_sequence + 1:
                self._event_gap = (
                    f"exploration_run_event_gap:{boot_id}:"
                    f"{previous_sequence}->{sequence}"
                )
            cursor = record.get("native_cursor")
            if isinstance(cursor, Mapping):
                prior_boot = str(cursor.get("boot_id") or "")
                if prior_boot and prior_boot != boot_id and not record["terminal"]:
                    raise ExploreRunConflict(
                        "exploration endpoint boot changed without an explicit safe terminal"
                    )

            self._boot_sequences[boot_id] = sequence
            now = time.time()
            record["native_cursor"] = {
                "boot_id": boot_id,
                "event_sequence": sequence,
            }
            record["last_native_event"] = event
            record["state"] = state
            record["state_source"] = "native_exploration_run_event"
            record["reason"] = str(event.get("reason") or state)
            record["can_resume"] = state == "paused"
            record["motion_stop"] = {
                "confirmed": stop_confirmed,
                "reason": stop_reason,
            }
            record["updated_at"] = now
            if state in _TERMINAL_STATES:
                record["terminal"] = True
                record["terminal_at"] = now
                record["can_resume"] = False
            self._persist()
            return self._snapshot(record)

    def reconcile_runtime(
        self,
        *,
        product_session_id: str,
        observed_boot_id: str,
    ) -> dict[str, Any] | None:
        """Expose a continuity loss without inventing a terminal outcome."""

        session = _required_text(product_session_id, "product_session_id")
        boot_id = _required_text(observed_boot_id, "observed_boot_id")
        with self._lock:
            self._require_available()
            for record in reversed(self._runs.values()):
                if record["terminal"] or record["identity"]["product_session_id"] != session:
                    continue
                cursor = record.get("native_cursor")
                known_boot = str(cursor.get("boot_id") or "") if isinstance(cursor, Mapping) else ""
                if not known_boot or known_boot == boot_id:
                    return None
                if (
                    record.get("state_source") == "runtime_reconciliation"
                    and str(record.get("replacement_boot_id") or "") == boot_id
                ):
                    return None
                now = time.time()
                record["state"] = "interrupted"
                record["state_source"] = "runtime_reconciliation"
                record["reason"] = "endpoint_restarted_stop_pending"
                record["terminal"] = False
                record["terminal_at"] = None
                record["can_resume"] = False
                record["motion_stop"] = {
                    "confirmed": False,
                    "reason": "awaiting_fail_safe_stop_evidence",
                }
                record["replacement_boot_id"] = boot_id
                record["updated_at"] = now
                self._persist()
                return self._snapshot(record)
            return None

    def query(self, exploration_run_id: str) -> dict[str, Any]:
        """Return one run projection without guessing an unknown outcome."""
        run_id = _canonical_ulid(exploration_run_id, "exploration_run_id")
        with self._lock:
            record = self._runs.get(run_id)
            if record is None:
                return {
                    "schema_version": _PUBLIC_SCHEMA,
                    "found": False,
                    "exploration_run_id": run_id,
                    "state": "unknown",
                    "terminal": False,
                    "reason": "exploration_run_not_found",
                }
            self._runs.move_to_end(run_id)
            return self._snapshot(record)

    def query_start_request(self, request_id: str) -> dict[str, Any] | None:
        """Return the run already bound to an idempotent start request."""

        request = _required_text(request_id, "request_id", max_length=128)
        with self._lock:
            run_id = self._requests.get(request)
            if run_id is None:
                return None
            return self._snapshot(self._runs[run_id])

    def list_recent(self, *, limit: int = 20) -> list[dict[str, Any]]:
        """Return newest runs first, bounded by the configured retention."""
        bounded = max(1, min(int(limit), self._retention))
        with self._lock:
            return [
                self._snapshot(record)
                for record in list(reversed(self._runs.values()))[:bounded]
            ]

    def latest_for_session(self, product_session_id: str) -> dict[str, Any] | None:
        """Return the newest run bound to one Product session, if retained."""

        session = _required_text(product_session_id, "product_session_id")
        with self._lock:
            for record in reversed(self._runs.values()):
                if record["identity"]["product_session_id"] == session:
                    return self._snapshot(record)
            return None

    def health(self) -> dict[str, Any]:
        """Return journal and native event-stream projection health."""
        with self._lock:
            return {
                "tracked_runs": len(self._runs),
                "retention": self._retention,
                "event_gap": self._event_gap,
                "journal": {
                    "status": self._journal_status,
                    "path": str(self._journal_path) if self._journal_path else "",
                    "error": self._journal_error,
                },
            }

    def _validate_event_binding(
        self, record: Mapping[str, Any], event: Mapping[str, Any]
    ) -> None:
        identity = record["identity"]
        checks = {
            "start_request_id": record["start_request_id"],
            "product_session_id": identity["product_session_id"],
            "route": identity["route"],
        }
        for field, expected in checks.items():
            if _required_text(event.get(field), f"native {field}") != expected:
                raise ExploreRunConflict(f"native exploration event {field} mismatch")
        expected_map = identity.get("map")
        if expected_map:
            if str(event.get("map_id") or "") != str(expected_map.get("map_id") or ""):
                raise ExploreRunConflict("native exploration event map_id mismatch")
            if int(event.get("map_version") or 0) != int(expected_map.get("map_version") or 0):
                raise ExploreRunConflict("native exploration event map_version mismatch")
            if str(event.get("artifact_hash") or "") != str(
                expected_map.get("artifact_hash") or ""
            ):
                raise ExploreRunConflict(
                    "native exploration event artifact_hash mismatch"
                )
        elif (
            str(event.get("map_id") or "")
            or int(event.get("map_version") or 0) != 0
            or str(event.get("artifact_hash") or "")
        ):
            raise ExploreRunConflict("native live exploration event carries saved-map identity")

    def _required_record(self, exploration_run_id: str) -> dict[str, Any]:
        try:
            return self._runs[exploration_run_id]
        except KeyError as exc:
            raise KeyError(f"unknown exploration run {exploration_run_id}") from exc

    def _snapshot(self, record: Mapping[str, Any]) -> dict[str, Any]:
        snapshot = json.loads(_json_bytes(dict(record)))
        snapshot["schema_version"] = _PUBLIC_SCHEMA
        snapshot["found"] = True
        return snapshot

    def _trim(self) -> None:
        while len(self._runs) > self._retention:
            _run_id, removed = self._runs.popitem(last=False)
            self._requests.pop(str(removed["start_request_id"]), None)

    def _require_available(self) -> None:
        if self._journal_status in {"corrupt", "write_failed"}:
            raise ExploreRunJournalUnavailable(
                self._journal_error or "exploration run journal is unavailable"
            )

    def _persist(self) -> None:
        if self._journal_path is None:
            return
        body = {
            "schema_version": _SCHEMA,
            "saved_at": time.time(),
            "retention": self._retention,
            "boot_sequences": self._boot_sequences,
            "runs": list(self._runs.values()),
        }
        body_bytes = _json_bytes(body)
        envelope = {"body": body, "sha256": hashlib.sha256(body_bytes).hexdigest()}
        payload = _json_bytes(envelope) + b"\n"
        if len(payload) > _MAX_JOURNAL_BYTES:
            self._journal_status = "write_failed"
            self._journal_error = "exploration run journal exceeds its size limit"
            raise ExploreRunJournalUnavailable(self._journal_error)
        try:
            _atomic_write(self._journal_path, payload)
        except OSError as exc:
            self._journal_status = "write_failed"
            self._journal_error = f"exploration run journal write failed: {exc}"
            raise ExploreRunJournalUnavailable(self._journal_error) from exc
        self._journal_status = "ready"
        self._journal_error = ""

    def _restore(self) -> None:
        if self._journal_path is None:
            raise RuntimeError("exploration run journal path is not configured")
        try:
            metadata = self._journal_path.lstat()
        except FileNotFoundError:
            return
        except OSError as exc:
            self._mark_corrupt(f"exploration run journal unreadable: {exc}")
            return
        try:
            if not stat.S_ISREG(metadata.st_mode):
                raise ValueError("exploration run journal is not a regular file")
            if metadata.st_size > _MAX_JOURNAL_BYTES:
                raise ValueError("exploration run journal exceeds its size limit")
            envelope = json.loads(self._journal_path.read_text(encoding="utf-8"))
            if not isinstance(envelope, Mapping):
                raise ValueError("exploration run journal envelope is invalid")
            body = envelope.get("body")
            if not isinstance(body, Mapping) or body.get("schema_version") != _SCHEMA:
                raise ValueError("exploration run journal schema is invalid")
            expected = str(envelope.get("sha256") or "")
            if hashlib.sha256(_json_bytes(body)).hexdigest() != expected:
                raise ValueError("exploration run journal integrity mismatch")
            values = body.get("runs")
            if not isinstance(values, list) or len(values) > self._retention:
                raise ValueError("exploration run journal retention is invalid")
            restored: OrderedDict[str, dict[str, Any]] = OrderedDict()
            requests: dict[str, str] = {}
            for value in values:
                record = _json_object(value, "run record")
                run_id = _canonical_ulid(
                    record.get("exploration_run_id"), "exploration_run_id"
                )
                request_id = _required_text(
                    record.get("start_request_id"), "start_request_id", max_length=128
                )
                if run_id in restored or request_id in requests:
                    raise ValueError("exploration run journal contains duplicate identity")
                restored[run_id] = record
                requests[request_id] = run_id
            raw_sequences = body.get("boot_sequences") or {}
            if not isinstance(raw_sequences, Mapping):
                raise ValueError("exploration run journal boot cursors are invalid")
            boot_sequences = {
                _required_text(key, "boot_id", max_length=128): int(value)
                for key, value in raw_sequences.items()
            }
            if any(value <= 0 for value in boot_sequences.values()):
                raise ValueError("exploration run journal boot cursor is invalid")
            self._runs = restored
            self._requests = requests
            self._boot_sequences = boot_sequences
            self._journal_status = "ready"
            self._journal_error = ""
        except (OSError, UnicodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
            self._mark_corrupt(str(exc))

    def _mark_corrupt(self, reason: str) -> None:
        self._runs.clear()
        self._requests.clear()
        self._boot_sequences.clear()
        self._journal_status = "corrupt"
        self._journal_error = reason


def create_explore_runs() -> ExploreRuns:
    """Create the process-local projection from deployment configuration."""
    configured = str(os.environ.get("LINGTU_EXPLORE_RUN_JOURNAL") or "").strip()
    return ExploreRuns(journal_path=configured or None)


def ensure_explore_runs(gateway: Any) -> ExploreRuns:
    """Attach exactly one Explore run projection to a Gateway instance."""
    runs = getattr(gateway, "_explore_runs", None)
    if isinstance(runs, ExploreRuns):
        return runs
    runs = create_explore_runs()
    gateway._explore_runs = runs
    return runs


__all__ = [
    "ExploreRunConflict",
    "ExploreRunJournalUnavailable",
    "ExploreRuns",
    "create_explore_runs",
    "ensure_explore_runs",
    "new_request_id",
]
