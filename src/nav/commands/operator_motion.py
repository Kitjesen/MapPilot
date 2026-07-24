"""Operator motion intent ingress with one explicit ownership seam."""

from __future__ import annotations

import math
import threading
import time
import uuid
from dataclasses import dataclass, replace
from typing import Any, Callable

from runtime import Module, rpc
from runtime.registry import register


@dataclass(frozen=True)
class _OperatorSource:
    source_id: str
    adapter: str
    lease_ttl_s: float = 1.0


@dataclass(frozen=True)
class _OperatorMotionIntent:
    session_id: str
    sequence: int
    request_id: str
    vx_mps: float
    vy_mps: float
    wz_radps: float
    deadman: bool
    ttl_ms: int = 350
    source_stamp_ns: int | None = None


@dataclass(frozen=True)
class _Session:
    session_id: str
    source: _OperatorSource
    expires_at: float
    last_sequence: int = 0


@dataclass(frozen=True)
class _PendingIntent:
    intent: _OperatorMotionIntent
    received_at: float


@register("operator_motion", "native", description="Operator motion session and intent ingress")
class OperatorMotion(Module, layer=6):
    """Own operator sessions and streaming semantics, never final velocity."""

    runtime_id = "operator.motion"
    SOFT_DEPENDS = ["nav.commands"]

    def __init__(
        self,
        command_module: str = "nav.commands",
        *,
        clock: Callable[[], float] = time.monotonic,
        **config: Any,
    ) -> None:
        super().__init__(**config)
        self._command_module = str(command_module or "nav.commands").strip()
        self._clock = clock
        self._condition = threading.Condition(threading.RLock())
        self._commands: Any | None = None
        self._session: _Session | None = None
        self._pending: _PendingIntent | None = None
        self._inflight = False
        self._accepting = True
        self._worker_stop = False
        self._worker: threading.Thread | None = None
        self._last_error: str | None = None
        self._last_native_request_id: str | None = None

    def on_system_modules(self, modules: dict[str, Module]) -> None:
        """Resolve the native navigation command boundary from the product graph."""
        self._commands = modules.get(self._command_module)

    def start(self) -> None:
        """Start the latest-only intent worker."""
        with self._condition:
            if self._running:
                return
            self._worker_stop = False
            self._worker = threading.Thread(
                target=self._run,
                name="lingtu-operator-motion",
                daemon=True,
            )
            super().start()
            self._worker.start()

    def stop(self) -> None:
        """Close the active session with a best-effort confirmed-zero barrier."""
        with self._condition:
            active_session_id = self._session.session_id if self._running and self._session is not None else None
        if active_session_id is not None:
            try:
                self.release(
                    active_session_id,
                    disposition="close",
                    reason="module_stop",
                )
            except Exception as exc:
                with self._condition:
                    self._last_error = str(exc) or type(exc).__name__
        with self._condition:
            self._worker_stop = True
            self._accepting = False
            self._pending = None
            self._condition.notify_all()
            worker = self._worker
        if worker is not None:
            worker.join(timeout=2.0)
        super().stop()

    @rpc
    def open(
        self,
        source_id: str,
        adapter: str,
        lease_ttl_s: float = 1.0,
    ) -> dict[str, Any]:
        """Acquire the single operator session without producing motion."""

        source_id = str(source_id or "").strip()
        adapter = str(adapter or "").strip()
        try:
            ttl_s = float(lease_ttl_s)
        except (TypeError, ValueError, OverflowError):
            ttl_s = 0.0
        if not source_id or not adapter or not math.isfinite(ttl_s) or ttl_s <= 0.0:
            return self._receipt(False, "rejected", "invalid_source")
        while True:
            expired_session_id: str | None = None
            with self._condition:
                if not self._running:
                    return self._receipt(False, "rejected", "not_running")
                if not self._accepting:
                    return self._receipt(False, "faulted", self._last_error or "motion_not_accepting")
                now = self._clock()
                if self._session is not None:
                    if now >= self._session.expires_at:
                        expired_session_id = self._session.session_id
                    elif self._session.source.source_id == source_id:
                        return self._receipt(
                            True,
                            "opened",
                            "already_owned",
                            session_id=self._session.session_id,
                        )
                    else:
                        return self._receipt(False, "rejected", "lease_conflict")
                else:
                    session_id = uuid.uuid4().hex
                    normalized = _OperatorSource(source_id=source_id, adapter=adapter, lease_ttl_s=ttl_s)
                    self._session = _Session(
                        session_id=session_id,
                        source=normalized,
                        expires_at=now + ttl_s,
                    )
                    return self._receipt(True, "opened", "control_acquired", session_id=session_id)
            if expired_session_id is None:
                return self._receipt(False, "faulted", "session_state_inconsistent")
            released = self.release(
                expired_session_id,
                disposition="close",
                reason="lease_expired",
            )
            if released.get("accepted") is not True:
                return self._receipt(
                    False,
                    "faulted",
                    str(released.get("reason") or "lease_expiry_zero_unconfirmed"),
                    session_id=expired_session_id,
                    zero_confirmed=False,
                )

    @rpc
    def submit(
        self,
        session_id: str,
        vx_mps: float,
        vy_mps: float,
        wz_radps: float,
        *,
        deadman: bool,
        sequence: int | None = None,
        request_id: str | None = None,
        ttl_ms: int = 350,
        source_stamp_ns: int | None = None,
    ) -> dict[str, Any]:
        """Queue the latest valid intent; native admission remains asynchronous."""

        request_id = str(request_id or "").strip() or f"operator-motion-{uuid.uuid4().hex}"
        session_id = str(session_id or "").strip()
        if not deadman:
            release_receipt = self.release(
                session_id,
                disposition="hold",
                reason="deadman_released",
                request_id=request_id,
            )
            if isinstance(release_receipt, dict):
                return release_receipt
            return self._receipt(
                False,
                "faulted",
                "invalid_release_receipt",
                session_id=session_id,
                request_id=request_id,
                zero_confirmed=False,
            )
        try:
            normalized_sequence = int(sequence) if sequence is not None else None
            ttl_ms = int(ttl_ms)
            velocities = (
                float(vx_mps),
                float(vy_mps),
                float(wz_radps),
            )
        except (TypeError, ValueError, OverflowError):
            return self._receipt(False, "rejected", "invalid_intent", session_id=session_id, request_id=request_id)
        if (
            not session_id
            or not request_id
            or (normalized_sequence is not None and normalized_sequence <= 0)
            or ttl_ms <= 0
            or not all(math.isfinite(value) for value in velocities)
        ):
            return self._receipt(False, "rejected", "invalid_intent", session_id=session_id, request_id=request_id)
        now = self._clock()
        with self._condition:
            session = self._session
            if not self._running:
                return self._receipt(False, "rejected", "not_running", session_id=session_id, request_id=request_id)
            if self._commands is None:
                return self._receipt(
                    False,
                    "rejected",
                    "command_module_unavailable",
                    session_id=session_id,
                    request_id=request_id,
                )
            if session is None or session.session_id != session_id:
                return self._receipt(False, "rejected", "session_not_owned", session_id=session_id, request_id=request_id)
            if now >= session.expires_at:
                return self._receipt(False, "rejected", "lease_expired", session_id=session_id, request_id=request_id)
            if not self._accepting:
                reason = self._last_error or "motion_not_accepting"
                return self._receipt(False, "faulted", reason, session_id=session_id, request_id=request_id)
            if normalized_sequence is None:
                normalized_sequence = session.last_sequence + 1
            if normalized_sequence <= session.last_sequence:
                return self._receipt(
                    False,
                    "rejected",
                    "sequence_not_increasing",
                    session_id=session_id,
                    request_id=request_id,
                )
            self._session = replace(
                session,
                expires_at=now + session.source.lease_ttl_s,
                last_sequence=normalized_sequence,
            )
            normalized = _OperatorMotionIntent(
                session_id=session_id,
                sequence=normalized_sequence,
                request_id=request_id,
                vx_mps=velocities[0],
                vy_mps=velocities[1],
                wz_radps=velocities[2],
                deadman=True,
                ttl_ms=ttl_ms,
                source_stamp_ns=source_stamp_ns,
            )
            self._pending = _PendingIntent(normalized, now)
            self._condition.notify()
        return self._receipt(True, "queued", "queued", session_id=session_id, request_id=request_id)

    @rpc
    def release(
        self,
        session_id: str,
        *,
        disposition: str = "close",
        reason: str = "operator_release",
        request_id: str | None = None,
        timeout_s: float = 2.0,
    ) -> dict[str, Any]:
        """Quiesce streaming work, synchronously confirm zero, then hold or close."""

        session_id = str(session_id or "").strip()
        disposition = str(disposition or "").strip().lower()
        reason = str(reason or "operator_release").strip() or "operator_release"
        request_id = str(request_id or "").strip() or f"operator-release-{uuid.uuid4().hex}"
        try:
            timeout_s = float(timeout_s)
        except (TypeError, ValueError, OverflowError):
            timeout_s = 0.0
        if not session_id or disposition not in {"hold", "close"} or not math.isfinite(timeout_s) or timeout_s <= 0.0:
            return self._receipt(
                False,
                "rejected",
                "invalid_release",
                session_id=session_id,
                request_id=request_id,
                zero_confirmed=False,
            )

        deadline = self._clock() + timeout_s
        with self._condition:
            session = self._session
            if not self._running:
                return self._receipt(
                    False,
                    "rejected",
                    "not_running",
                    session_id=session_id,
                    request_id=request_id,
                    zero_confirmed=False,
                )
            if session is None or session.session_id != session_id:
                return self._receipt(
                    False,
                    "rejected",
                    "session_not_owned",
                    session_id=session_id,
                    request_id=request_id,
                    zero_confirmed=False,
                )
            commands = self._commands
            if commands is None:
                return self._receipt(
                    False,
                    "faulted",
                    "command_module_unavailable",
                    session_id=session_id,
                    request_id=request_id,
                    zero_confirmed=False,
                )
            self._accepting = False
            self._pending = None
            while self._inflight:
                remaining = deadline - self._clock()
                if remaining <= 0.0:
                    self._last_error = "release_quiesce_timeout"
                    return self._receipt(
                        False,
                        "faulted",
                        self._last_error,
                        session_id=session_id,
                        request_id=request_id,
                        zero_confirmed=False,
                    )
                self._condition.wait(remaining)

        error: str | None = None
        try:
            accepted = commands.send_teleop(0.0, 0.0, 0.0, request_id=request_id)
            if accepted is False:
                raise RuntimeError("native_zero_rejected")
        except Exception as exc:
            error = str(exc) or type(exc).__name__

        with self._condition:
            self._last_error = error
            if error is not None:
                self._accepting = False
                return self._receipt(
                    False,
                    "faulted",
                    error,
                    session_id=session_id,
                    request_id=request_id,
                    zero_confirmed=False,
                )
            self._last_native_request_id = request_id
            if disposition == "close":
                self._session = None
                stage = "closed"
            else:
                current = self._session
                if current is not None and current.session_id == session_id:
                    self._session = replace(
                        current,
                        expires_at=self._clock() + current.source.lease_ttl_s,
                    )
                stage = "held"
            self._accepting = True
            self._condition.notify_all()
        return self._receipt(
            True,
            stage,
            reason,
            session_id=session_id,
            request_id=request_id,
            zero_confirmed=True,
        )

    @rpc
    def health(self) -> dict[str, Any]:
        """Report ingress/session truth without claiming safe output or execution."""

        with self._condition:
            if not self._running:
                state = "stopped"
            elif self._last_error is not None or not self._accepting:
                state = "faulted"
            elif self._commands is None:
                state = "unavailable"
            else:
                state = "ready"
            session = self._session
            if session is None:
                session_payload: dict[str, Any] = {
                    "active": False,
                    "session_id": None,
                    "source_id": None,
                    "adapter": None,
                    "lease_remaining_s": 0.0,
                    "last_sequence": 0,
                }
            else:
                session_payload = {
                    "active": True,
                    "session_id": session.session_id,
                    "source_id": session.source.source_id,
                    "adapter": session.source.adapter,
                    "lease_remaining_s": max(0.0, session.expires_at - self._clock()),
                    "last_sequence": session.last_sequence,
                }
            return {
                "state": state,
                "command_module_ready": self._commands is not None,
                "accepting": self._accepting,
                "session": session_payload,
                "stream": {
                    "pending": self._pending is not None,
                    "inflight": self._inflight,
                    "last_native_request_id": self._last_native_request_id,
                    "last_error": self._last_error,
                },
            }

    @staticmethod
    def _receipt(
        accepted: bool,
        stage: str,
        reason: str,
        *,
        session_id: str | None = None,
        request_id: str | None = None,
        zero_confirmed: bool | None = None,
    ) -> dict[str, Any]:
        return {
            "accepted": bool(accepted),
            "stage": str(stage),
            "reason": str(reason),
            "session_id": session_id,
            "request_id": request_id,
            "zero_confirmed": zero_confirmed,
        }

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._worker_stop:
                    self._condition.wait()
                if self._worker_stop:
                    return
                pending = self._pending
                self._pending = None
                self._inflight = True
            if pending is None:
                with self._condition:
                    self._inflight = False
                    self._condition.notify_all()
                continue
            if self._clock() - pending.received_at > pending.intent.ttl_ms / 1000.0:
                with self._condition:
                    self._inflight = False
                    self._condition.notify_all()
                continue
            error: str | None = None
            try:
                commands = self._commands
                if commands is None:
                    raise RuntimeError("command_module_unavailable")
                accepted = commands.send_teleop(
                    pending.intent.vx_mps,
                    pending.intent.vy_mps,
                    pending.intent.wz_radps,
                    request_id=pending.intent.request_id,
                )
                if accepted is False:
                    raise RuntimeError("native_command_rejected")
            except Exception as exc:
                error = str(exc) or type(exc).__name__
            finally:
                with self._condition:
                    self._last_error = error
                    if error is None:
                        self._last_native_request_id = pending.intent.request_id
                    else:
                        self._accepting = False
                        self._pending = None
                    self._inflight = False
                    self._condition.notify_all()


__all__ = [
    "OperatorMotion",
]
