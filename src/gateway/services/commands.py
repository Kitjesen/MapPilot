"""Command idempotency and client policy helpers for Gateway control APIs."""

from __future__ import annotations

import threading
import time
from collections.abc import Mapping
from typing import Any, Callable

from fastapi.responses import JSONResponse

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


class IdempotencyConflict(RuntimeError):
    """A request id was reused with a different command payload."""

    def __init__(self, command: str, request_id: str, client_id: str) -> None:
        self.command = command
        self.request_id = request_id
        self.client_id = client_id
        super().__init__(
            f"request_id {request_id!r} was already used for a different "
            f"{command!r} payload by client {client_id!r}"
        )


_NO_PENDING_RESULT = object()


class _PendingCommand:
    """One in-flight command shared by concurrent exact retries."""

    def __init__(self, request_payload: Any) -> None:
        self.request_payload = request_payload
        self.event = threading.Event()
        self.result: Any = _NO_PENDING_RESULT
        self.record: dict[str, Any] | None = None
        self.error: BaseException | None = None


class CommandJournal:
    """Concurrent idempotency boundary for retry-safe control requests."""

    def __init__(
        self,
        retention_s: float = COMMAND_IDEMPOTENCY_RETENTION_S,
        max_entries: int = COMMAND_JOURNAL_MAX_ENTRIES,
    ) -> None:
        self._retention_s = retention_s
        self._max_entries = max_entries
        self._records: dict[tuple[str, str, str], dict[str, Any]] = {}
        self._pending: dict[tuple[str, str, str], _PendingCommand] = {}
        self._lock = threading.Lock()
        self._accepted_commands = 0
        self._replayed_commands = 0
        self._conflicting_commands = 0

    def execute(
        self,
        command: str,
        request_id: str | None,
        client_id: str | None,
        request_payload: Any,
        action: Callable[[], Mapping[str, Any] | JSONResponse],
    ) -> dict[str, Any] | JSONResponse:
        """Execute once, replay exact retries, and reject payload conflicts.

        Commands without a request id or a distinct client identity deliberately
        bypass deduplication.  Otherwise, the first caller owns execution while
        concurrent exact retries wait without holding the journal lock.
        """

        request_id = _clean_request_id(request_id)
        client_id = _clean_client_id(client_id)
        if request_id is None or client_id == "unknown":
            response = action()
            if isinstance(response, JSONResponse):
                return response
            if not isinstance(response, Mapping):
                return self._invalid_action_receipt(
                    command,
                    request_id,
                    client_id,
                    response,
                )
            acceptance = self._acceptance_signal(response)
            if acceptance is None:
                return self._invalid_action_receipt(
                    command,
                    request_id,
                    client_id,
                    response,
                    reason=(
                        "command action mapping must include an explicit positive "
                        "acceptance signal"
                    ),
                )
            now = time.time()
            with self._lock:
                if acceptance:
                    self._accepted_commands += 1
            return self._with_receipt(
                command=command,
                request_id=request_id,
                client_id=client_id,
                response=dict(response),
                accepted=acceptance,
                replay=False,
                ts=now,
            )

        identity = (client_id, command, request_id)
        owner = False
        with self._lock:
            self._purge_locked(time.time())
            record = self._records.get(identity)
            if record is not None:
                self._assert_same_payload_locked(
                    identity,
                    request_payload,
                    record["request_payload"],
                )
                self._replayed_commands += 1
                return self._replay_receipt(command, request_id, record)

            pending = self._pending.get(identity)
            if pending is None:
                pending = _PendingCommand(request_payload)
                self._pending[identity] = pending
                owner = True
            else:
                self._assert_same_payload_locked(
                    identity,
                    request_payload,
                    pending.request_payload,
                )

        if not owner:
            pending.event.wait()
            if pending.error is not None:
                self._raise_pending_error(pending.error)
            if pending.result is not _NO_PENDING_RESULT:
                return pending.result
            with self._lock:
                record = pending.record or self._records.get(identity)
                if record is None:
                    raise RuntimeError("completed idempotent command has no result")
                self._replayed_commands += 1
                return self._replay_receipt(command, request_id, record)

        try:
            response = action()
        except BaseException as exc:
            with self._lock:
                self._pending.pop(identity, None)
                pending.error = exc
                pending.event.set()
            raise

        if isinstance(response, JSONResponse):
            with self._lock:
                self._pending.pop(identity, None)
                pending.result = response
                pending.event.set()
            return response

        if not isinstance(response, Mapping):
            rejected = self._invalid_action_receipt(
                command,
                request_id,
                client_id,
                response,
            )
            with self._lock:
                self._pending.pop(identity, None)
                pending.result = rejected
                pending.event.set()
            return rejected

        now = time.time()
        acceptance = self._acceptance_signal(response)
        if acceptance is None:
            rejected = self._invalid_action_receipt(
                command,
                request_id,
                client_id,
                response,
                reason=(
                    "command action mapping must include an explicit positive "
                    "acceptance signal"
                ),
            )
            with self._lock:
                self._pending.pop(identity, None)
                pending.result = rejected
                pending.event.set()
            return rejected

        if not acceptance:
            rejected = self._with_receipt(
                command=command,
                request_id=request_id,
                client_id=client_id,
                response=dict(response),
                accepted=False,
                replay=False,
                ts=now,
            )
            with self._lock:
                self._pending.pop(identity, None)
                pending.result = rejected
                pending.event.set()
            return rejected

        record = {
            "client_id": client_id,
            "request_payload": request_payload,
            "response": dict(response),
            "ts": now,
        }
        with self._lock:
            self._accepted_commands += 1
            self._records[identity] = record
            self._trim_locked()
            self._pending.pop(identity, None)
            pending.record = record
            pending.event.set()
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
                "idempotency_scope": "client_command_request_id",
                "payload_conflict_policy": "reject",
                "acceptance_signal_policy": "explicit_positive_required",
                "acceptance_signal_fields": [
                    "ok",
                    "accepted",
                    "success",
                    "command.accepted",
                ],
                "anonymous_client_policy": "execute_every_time",
                "client_identity_source": "caller_asserted",
                "retention_s": self._retention_s,
                "max_entries": self._max_entries,
                "stored_requests": len(self._records),
                "pending_requests": len(self._pending),
                "accepted_commands": self._accepted_commands,
                "replayed_commands": self._replayed_commands,
                "conflicting_commands": self._conflicting_commands,
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

    def _assert_same_payload_locked(
        self,
        identity: tuple[str, str, str],
        request_payload: Any,
        existing_payload: Any,
    ) -> None:
        if request_payload == existing_payload:
            return
        self._conflicting_commands += 1
        client_id, command, request_id = identity
        raise IdempotencyConflict(command, request_id, client_id)

    @staticmethod
    def _acceptance_signal(response: Mapping[str, Any]) -> bool | None:
        signals = [response.get(field) for field in ("ok", "accepted", "success")]
        command = response.get("command")
        if isinstance(command, Mapping):
            signals.append(command.get("accepted"))
        if any(signal is False for signal in signals):
            return False
        if any(signal is True for signal in signals):
            return True
        return None

    @classmethod
    def _invalid_action_receipt(
        cls,
        command: str,
        request_id: str | None,
        client_id: str,
        response: Any,
        *,
        reason: str | None = None,
    ) -> dict[str, Any]:
        return cls._with_receipt(
            command=command,
            request_id=request_id,
            client_id=client_id,
            response={
                "ok": False,
                "status": "rejected",
                "error": "invalid_command_response",
                "message": reason
                or (
                    "command action must return a mapping or JSONResponse; "
                    f"received {type(response).__name__}"
                ),
            },
            accepted=False,
            replay=False,
            ts=time.time(),
        )

    @classmethod
    def _replay_receipt(
        cls,
        command: str,
        request_id: str,
        record: dict[str, Any],
    ) -> dict[str, Any]:
        return cls._with_receipt(
            command=command,
            request_id=request_id,
            client_id=record["client_id"],
            response=record["response"],
            accepted=True,
            replay=True,
            ts=record["ts"],
        )

    @staticmethod
    def _raise_pending_error(error: BaseException) -> None:
        try:
            cloned = type(error)(*error.args)
        except Exception:
            cloned = RuntimeError(str(error))
        raise cloned from error

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
        if accepted:
            payload.setdefault("ok", True)
        else:
            payload["ok"] = False
        payload["command"] = {
            "name": command,
            "task_id": payload.get("task_id"),
            "request_id": request_id,
            "native_request_id": payload.get("native_request_id"),
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


def command_request_payload(body: Any) -> Any:
    """Return the command payload used for retry comparison.

    A request id is only retry-safe inside the same client, command, and exact
    payload.  Including the payload prevents a changed motion request from
    receiving a stale receipt for an earlier command.
    """

    value = body
    model_dump = getattr(body, "model_dump", None)
    if callable(model_dump):
        value = model_dump(mode="json")
    elif hasattr(body, "dict") and callable(body.dict):
        value = body.dict()
    return dict(value) if isinstance(value, Mapping) else value


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
    success = payload.get("success")
    data = {
        "schema_version": 1,
        "ok": payload.get("ok") is True,
        "accepted": command.get("accepted") is True,
        "replay": command.get("replay") is True,
        "status": payload.get("status"),
        "stage": payload.get("stage"),
        "success": success if isinstance(success, bool) else None,
        "execution_confirmed": payload.get("execution_confirmed"),
        "final_output_confirmed": payload.get("final_output_confirmed"),
        "error": payload.get("error"),
        "message": payload.get("message"),
        "command": command,
        "detail": payload.get("detail"),
        "status_code": status_code,
        "ts": time.time(),
    }
    gw.push_event({"type": "command_ack", "data": data})


def idempotency_conflict_response(
    gw: Any,
    command: str,
    conflict: IdempotencyConflict,
) -> JSONResponse:
    """Map a journal identity conflict to the Gateway error contract."""

    content = {
        "schema_version": 1,
        "ok": False,
        "error": "idempotency_conflict",
        "message": "request_id was already used with a different payload",
        "command": {
            "name": command,
            "request_id": conflict.request_id,
            "client_id": conflict.client_id,
            "accepted": False,
            "replay": False,
            "ts": time.time(),
        },
        "detail": {
            "reason_code": "idempotency_conflict",
            "reason": str(conflict),
            "source": "command_journal",
            "path": None,
            "blockers": ["idempotency_conflict"],
            "advisories": [],
        },
    }
    publish_command_ack(gw, content, status_code=409)
    return JSONResponse(status_code=409, content=content)


def run_control_command(
    gw: Any,
    command: str,
    body: Any,
    action: Callable[[], Mapping[str, Any] | JSONResponse],
    *,
    success_status_code: int = 200,
) -> dict[str, Any] | JSONResponse:
    request_id = getattr(body, "request_id", None) if body is not None else None
    client_id = getattr(body, "client_id", None) if body is not None else None
    request_payload = command_request_payload(body)
    try:
        response = gw._command_journal.execute(
            command,
            request_id,
            client_id,
            request_payload,
            action,
        )
    except IdempotencyConflict as conflict:
        return idempotency_conflict_response(gw, command, conflict)
    if not isinstance(response, dict):
        return response
    command_receipt = response.get("command")
    accepted = (
        command_receipt.get("accepted") is True
        if isinstance(command_receipt, dict)
        else False
    )
    if not accepted:
        response["ok"] = False
        response.setdefault("error", "command_rejected")
        response.setdefault("message", "command action rejected the request")
        response.setdefault(
            "detail",
            {
                "reason_code": str(response["error"]),
                "reason": str(response["message"]),
                "source": "command_action",
                "path": None,
                "blockers": [str(response["error"])],
                "advisories": [],
            },
        )
        publish_command_ack(gw, response, status_code=409)
        return JSONResponse(status_code=409, content=response)
    publish_command_ack(gw, response, status_code=success_status_code)
    return response
