"""Command idempotency and client policy helpers for Gateway control APIs."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

COMMAND_IDEMPOTENCY_RETENTION_S = 120.0
COMMAND_JOURNAL_MAX_ENTRIES = 512

COMMAND_RATE_POLICY_HZ: dict[str, float] = {
    "goal": 2.0,
    "navigate_click": 2.0,
    "navigation_cancel": 2.0,
    "cmd_vel": 20.0,
    "stop": 5.0,
    "instruction": 1.0,
    "visual_servo": 2.0,
    "mode": 1.0,
    "lease": 1.0,
}


class ControlLease:
    """Small mutex-protected control lease shared by command routes."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._holder: str | None = None
        self._expiry: float = 0.0

    def acquire(self, client_id: str, ttl: float) -> bool:
        with self._lock:
            now = time.monotonic()
            if self._holder and self._holder != client_id and now < self._expiry:
                return False
            self._holder = client_id
            self._expiry = now + ttl
            return True

    def release(self, client_id: str) -> bool:
        with self._lock:
            if self._holder == client_id:
                self._holder = None
                self._expiry = 0.0
                return True
            return False

    def renew(self, client_id: str, ttl: float) -> bool:
        with self._lock:
            now = time.monotonic()
            if self._holder is not None and now >= self._expiry:
                self._holder = None
                self._expiry = 0.0
                return False
            if self._holder == client_id:
                self._expiry = now + ttl
                return True
            return False

    def check(self, client_id: str) -> bool:
        with self._lock:
            now = time.monotonic()
            if self._holder is not None and now >= self._expiry:
                self._holder = None
                self._expiry = 0.0
                return True
            if self._holder is None:
                return True
            return self._holder == client_id

    def to_dict(self) -> dict[str, Any]:
        with self._lock:
            now = time.monotonic()
            return {
                "holder": self._holder,
                "active": self._holder is not None and now < self._expiry,
                "expires_in": max(0.0, self._expiry - now) if self._holder else 0.0,
            }


class CommandJournal:
    """Small in-memory idempotency journal for retry-safe control requests."""

    def __init__(
        self,
        retention_s: float = COMMAND_IDEMPOTENCY_RETENTION_S,
        max_entries: int = COMMAND_JOURNAL_MAX_ENTRIES,
    ) -> None:
        self._retention_s = retention_s
        self._max_entries = max_entries
        self._records: dict[tuple[str, str], dict[str, Any]] = {}
        self._lock = threading.Lock()
        self._accepted_commands = 0
        self._replayed_commands = 0

    def replay(self, command: str, request_id: str | None) -> dict[str, Any] | None:
        request_id = _clean_request_id(request_id)
        if request_id is None:
            return None
        with self._lock:
            self._purge_locked(time.time())
            record = self._records.get((command, request_id))
            if record is None:
                return None
            self._replayed_commands += 1
            return self._with_receipt(
                command=command,
                request_id=request_id,
                client_id=record["client_id"],
                response=record["response"],
                accepted=True,
                replay=True,
                ts=record["ts"],
            )

    def accept(
        self,
        command: str,
        request_id: str | None,
        client_id: str | None,
        response: dict[str, Any],
    ) -> dict[str, Any]:
        request_id = _clean_request_id(request_id)
        client_id = _clean_client_id(client_id)
        now = time.time()
        with self._lock:
            self._accepted_commands += 1
            if request_id is not None:
                self._purge_locked(now)
                self._records[(command, request_id)] = {
                    "client_id": client_id,
                    "response": dict(response),
                    "ts": now,
                }
                self._trim_locked()
            return self._with_receipt(
                command=command,
                request_id=request_id,
                client_id=client_id,
                response=response,
                accepted=True,
                replay=False,
                ts=now,
            )

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            self._purge_locked(time.time())
            return {
                "idempotency_supported": True,
                "request_id_field": "request_id",
                "client_id_field": "client_id",
                "retention_s": self._retention_s,
                "max_entries": self._max_entries,
                "stored_requests": len(self._records),
                "accepted_commands": self._accepted_commands,
                "replayed_commands": self._replayed_commands,
                "rate_policy_hz": dict(COMMAND_RATE_POLICY_HZ),
                "rate_policy_enforcement": "advisory",
            }

    def _purge_locked(self, now: float) -> None:
        expired = [key for key, record in self._records.items() if now - float(record["ts"]) > self._retention_s]
        for key in expired:
            self._records.pop(key, None)

    def _trim_locked(self) -> None:
        overflow = len(self._records) - self._max_entries
        if overflow <= 0:
            return
        oldest = sorted(self._records.items(), key=lambda item: item[1]["ts"])
        for key, _record in oldest[:overflow]:
            self._records.pop(key, None)

    @staticmethod
    def _with_receipt(
        *,
        command: str,
        request_id: str | None,
        client_id: str,
        response: dict[str, Any],
        accepted: bool,
        replay: bool,
        ts: float,
    ) -> dict[str, Any]:
        payload = dict(response)
        payload.setdefault("schema_version", 1)
        payload.setdefault("ok", accepted)
        payload["command"] = {
            "name": command,
            "request_id": request_id,
            "client_id": client_id,
            "accepted": accepted,
            "replay": replay,
            "ts": ts,
        }
        return payload


def _clean_request_id(value: str | None) -> str | None:
    if value is None:
        return None
    cleaned = str(value).strip()
    return cleaned or None


def _clean_client_id(value: str | None) -> str:
    if value is None:
        return "unknown"
    cleaned = str(value).strip()
    return cleaned or "unknown"


def publish_command_ack(
    gw: Any,
    payload: dict[str, Any],
    *,
    status_code: int | None = None,
) -> None:
    """Publish a lightweight command acknowledgement for App/Web clients."""
    if not isinstance(payload, dict):
        return
    command = payload.get("command")
    if not isinstance(command, dict):
        return
    data = {
        "schema_version": 1,
        "ok": bool(payload.get("ok", False)),
        "status": payload.get("status"),
        "error": payload.get("error"),
        "message": payload.get("message"),
        "command": command,
        "detail": payload.get("detail"),
        "status_code": status_code,
        "ts": time.time(),
    }
    gw.push_event({"type": "command_ack", "data": data})


def run_control_command(
    gw: Any,
    command: str,
    body: Any,
    action: Callable[[], dict[str, Any]],
) -> dict[str, Any]:
    request_id = getattr(body, "request_id", None) if body is not None else None
    client_id = getattr(body, "client_id", None) if body is not None else None
    replay = gw._command_journal.replay(command, request_id)
    if replay is not None:
        publish_command_ack(gw, replay, status_code=200)
        return replay
    response = gw._command_journal.accept(
        command,
        request_id,
        client_id,
        action(),
    )
    publish_command_ack(gw, response, status_code=200)
    return response
