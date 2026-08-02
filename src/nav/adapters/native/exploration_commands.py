"""Exploration-session commands over the process-wide native C++ DDS client."""

from __future__ import annotations

import math
import os
import re
from typing import Any

from nav.adapters.native.abi import (
    NativeCommandClientError,
    NativeCommandSession,
    get_native_command_session,
)

ExplorationCommandClientError = NativeCommandClientError


DIRECTED_TARGET_MAX_ABS_COORDINATE_M = 1_000_000.0
DIRECTED_TARGET_MAX_TTL_S = 3_600.0
_EXPLORATION_RUN_ID_PATTERN = re.compile(r"^[0-7][0-9A-HJKMNP-TV-Z]{25}$")


def normalize_exploration_identity(
    exploration_run_id: Any,
    session_id: Any,
) -> tuple[str, str]:
    """Validate caller-owned run and Product-session identities."""

    run_id = str(exploration_run_id or "")
    if not _EXPLORATION_RUN_ID_PATTERN.fullmatch(run_id):
        raise ValueError(
            "exploration_run_id must be a canonical uppercase 26-character ULID"
        )
    product_session_id = str(session_id or "").strip()
    if not product_session_id:
        raise ValueError("exploration session_id is required")
    return run_id, product_session_id


def _finite_directed_target_value(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"directed target {label} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"directed target {label} must be a finite number")
    return number


def normalize_directed_target(x: Any, y: Any, ttl_s: Any) -> tuple[float, float, float]:
    """Validate a map-frame target before it crosses the native C ABI."""

    target_x = _finite_directed_target_value(x, "x")
    target_y = _finite_directed_target_value(y, "y")
    target_ttl_s = _finite_directed_target_value(ttl_s, "ttl_s")
    for value, label in ((target_x, "x"), (target_y, "y")):
        if abs(value) > DIRECTED_TARGET_MAX_ABS_COORDINATE_M:
            raise ValueError(
                f"directed target {label} must be within "
                f"+/-{DIRECTED_TARGET_MAX_ABS_COORDINATE_M:g} m"
            )
    if not 0.0 < target_ttl_s <= DIRECTED_TARGET_MAX_TTL_S:
        raise ValueError(
            f"directed target ttl_s must be in (0, {DIRECTED_TARGET_MAX_TTL_S:g}]"
        )
    return target_x, target_y, target_ttl_s


class NativeExplorationCommandClient:
    """Narrow interface for the native exploration lifecycle FSM."""

    def __init__(
        self,
        library_path: str | os.PathLike[str],
        *,
        domain_id: int = 0,
        timeout_ms: int = 1000,
        library: Any | None = None,
    ) -> None:
        self._session = NativeCommandSession(
            library_path,
            domain_id=domain_id,
            timeout_ms=timeout_ms,
            goal_timeout_ms=timeout_ms,
            cancel_timeout_ms=timeout_ms,
            teleop_timeout_ms=timeout_ms,
            library=library,
        )
        self._owns_session = True
        try:
            self._session.ensure_exploration_abi()
        except Exception:
            self._session.close()
            raise

    @classmethod
    def _from_session(
        cls,
        session: NativeCommandSession,
    ) -> NativeExplorationCommandClient:
        instance = cls.__new__(cls)
        instance._session = session
        instance._owns_session = False
        session.ensure_exploration_abi()
        return instance

    def start(
        self,
        exploration_run_id: str,
        session_id: str,
        *,
        reason: str = "operator_start",
        request_id: str | None = None,
    ) -> dict[str, object]:
        """Start a native exploration session after endpoint readiness gating."""

        run_id, product_session_id = normalize_exploration_identity(
            exploration_run_id,
            session_id,
        )
        return self._command(
            "lingtu_nav_client_start_exploration_with_receipt_v1",
            run_id,
            request_id,
            str(request_id or "").encode("utf-8"),
            run_id.encode("utf-8"),
            product_session_id.encode("utf-8"),
            str(reason or "operator_start").encode("utf-8"),
            self._session.goal_timeout_ms,
        )

    def pause(
        self,
        exploration_run_id: str,
        session_id: str,
        reason: str = "operator_pause",
        *,
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._reason_command(
            "lingtu_nav_client_pause_exploration_with_receipt_v1",
            exploration_run_id,
            session_id,
            reason,
            request_id,
        )

    def resume(
        self,
        exploration_run_id: str,
        session_id: str,
        reason: str = "operator_resume",
        *,
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._reason_command(
            "lingtu_nav_client_resume_exploration_with_receipt_v1",
            exploration_run_id,
            session_id,
            reason,
            request_id,
        )

    def stop(
        self,
        exploration_run_id: str,
        session_id: str,
        reason: str = "operator_stop",
        *,
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._reason_command(
            "lingtu_nav_client_stop_exploration_with_receipt_v1",
            exploration_run_id,
            session_id,
            reason,
            request_id,
        )

    def set_directed_target(
        self,
        x: float,
        y: float,
        ttl_s: float,
        exploration_run_id: str,
        session_id: str,
        reason: str = "operator_directed_explore",
        *,
        request_id: str | None = None,
    ) -> dict[str, object]:
        """Set one typed map-frame exploration intent and wait for its ACK."""

        target_x, target_y, target_ttl_s = normalize_directed_target(x, y, ttl_s)
        run_id, product_session_id = normalize_exploration_identity(
            exploration_run_id,
            session_id,
        )
        self._session.ensure_directed_exploration_abi()
        return self._command(
            "lingtu_nav_client_set_directed_exploration_target_with_receipt_v1",
            run_id,
            request_id,
            str(request_id or "").encode("utf-8"),
            run_id.encode("utf-8"),
            target_x,
            target_y,
            target_ttl_s,
            product_session_id.encode("utf-8"),
            str(reason or "operator_directed_explore").encode("utf-8"),
            self._session.goal_timeout_ms,
        )

    def clear_directed_target(
        self,
        exploration_run_id: str,
        session_id: str,
        reason: str = "operator_clear_directed_explore",
        *,
        request_id: str | None = None,
    ) -> dict[str, object]:
        """Clear the active directed exploration intent and wait for its ACK."""

        run_id, product_session_id = normalize_exploration_identity(
            exploration_run_id,
            session_id,
        )

        self._session.ensure_directed_exploration_abi()
        return self._command(
            "lingtu_nav_client_clear_directed_exploration_target_with_receipt_v1",
            run_id,
            request_id,
            str(request_id or "").encode("utf-8"),
            run_id.encode("utf-8"),
            product_session_id.encode("utf-8"),
            str(reason or "operator_clear_directed_explore").encode("utf-8"),
            self._session.cancel_timeout_ms,
        )

    def _reason_command(
        self,
        function_name: str,
        exploration_run_id: str,
        session_id: str,
        reason: str,
        request_id: str | None,
    ) -> dict[str, object]:
        run_id, product_session_id = normalize_exploration_identity(
            exploration_run_id,
            session_id,
        )
        return self._command(
            function_name,
            run_id,
            request_id,
            str(request_id or "").encode("utf-8"),
            run_id.encode("utf-8"),
            product_session_id.encode("utf-8"),
            str(reason).encode("utf-8"),
            self._session.cancel_timeout_ms,
        )

    def _command(
        self,
        function_name: str,
        exploration_run_id: str,
        request_id: str | None,
        *arguments: object,
    ) -> dict[str, object]:
        receipt = self._session.call_with_exploration_receipt(
            function_name,
            *arguments,
        )
        if receipt.get("exploration_run_id") != exploration_run_id:
            raise NativeCommandClientError(
                "native exploration command returned the wrong exploration_run_id"
            )
        expected_request_id = str(request_id or "")
        if expected_request_id and receipt.get("request_id") != expected_request_id:
            raise NativeCommandClientError(
                "native exploration command returned the wrong request_id"
            )
        return receipt

    def close(self) -> None:
        if self._owns_session:
            self._session.close()


def get_native_exploration_command_client(
    *,
    required: bool = False,
) -> NativeExplorationCommandClient | None:
    """Return the exploration view of the process-wide native command session."""

    session = get_native_command_session(required=required)
    if session is None:
        return None
    return NativeExplorationCommandClient._from_session(session)
