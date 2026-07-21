"""Exploration-session commands over the process-wide native C++ DDS client."""

from __future__ import annotations

import os
from typing import Any

from nav.adapters.native.abi import (
    NativeCommandClientError,
    NativeCommandSession,
    get_native_command_session,
)

ExplorationCommandClientError = NativeCommandClientError


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
        *,
        session_id: str = "",
        reason: str = "operator_start",
        request_id: str | None = None,
    ) -> None:
        """Start a native exploration session after endpoint readiness gating."""

        self._session.call(
            "lingtu_nav_client_start_exploration",
            str(request_id or "").encode("utf-8"),
            str(session_id or "").encode("utf-8"),
            str(reason or "operator_start").encode("utf-8"),
            self._session.goal_timeout_ms,
        )

    def pause(self, reason: str = "operator_pause", *, request_id: str | None = None) -> None:
        self._reason_command("lingtu_nav_client_pause_exploration", reason, request_id)

    def resume(self, reason: str = "operator_resume", *, request_id: str | None = None) -> None:
        self._reason_command("lingtu_nav_client_resume_exploration", reason, request_id)

    def stop(self, reason: str = "operator_stop", *, request_id: str | None = None) -> None:
        self._reason_command("lingtu_nav_client_stop_exploration", reason, request_id)

    def _reason_command(
        self,
        function_name: str,
        reason: str,
        request_id: str | None,
    ) -> None:
        self._session.call(
            function_name,
            str(request_id or "").encode("utf-8"),
            str(reason).encode("utf-8"),
            self._session.cancel_timeout_ms,
        )

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
