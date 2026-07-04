"""Command idempotency and client policy helpers for Gateway control APIs."""

from __future__ import annotations

import threading
import time
from typing import Any


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
        expired = [
            key for key, record in self._records.items()
            if now - float(record["ts"]) > self._retention_s
        ]
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
