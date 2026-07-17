"""Inspection-command interface over the process-wide native C++ DDS session."""

from __future__ import annotations

import os
from typing import Any

from runtime.adapters.native.navigation_abi import (
    NativeCommandClientError,
    NativeCommandSession,
    get_native_command_session,
)

InspectionCommandClientError = NativeCommandClientError
UINT64_MAX = (1 << 64) - 1


def normalize_route_revision(value: Any, *, label: str = "route revision") -> int:
    """Return a revision that is safe to marshal through the uint64 C ABI."""

    try:
        revision = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be an integer") from exc
    if not 0 <= revision <= UINT64_MAX:
        raise ValueError(f"{label} must be between 0 and UINT64_MAX")
    return revision


class NativeInspectionCommandClient:
    """Narrow Python interface for inspection execution lifecycle commands."""

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
            self._session.ensure_inspection_abi()
        except Exception:
            self._session.close()
            raise

    @classmethod
    def _from_session(cls, session: NativeCommandSession) -> NativeInspectionCommandClient:
        instance = cls.__new__(cls)
        instance._session = session
        instance._owns_session = False
        session.ensure_inspection_abi()
        return instance

    def start(
        self,
        route_id: str,
        *,
        revision: int = 0,
        request_id: str | None = None,
    ) -> None:
        """Start one stored inspection route and wait for admission ACK."""

        route_revision = normalize_route_revision(revision)
        self._session.call(
            "lingtu_nav_client_start_inspection",
            str(request_id or "").encode("utf-8"),
            str(route_id).encode("utf-8"),
            route_revision,
            self._session.goal_timeout_ms,
        )

    def pause(self, reason: str = "operator_pause", *, request_id: str | None = None) -> None:
        """Pause the active inspection run."""

        self._reason_command("lingtu_nav_client_pause_inspection", reason, request_id)

    def resume(self, reason: str = "operator_resume", *, request_id: str | None = None) -> None:
        """Resume the active inspection run."""

        self._reason_command("lingtu_nav_client_resume_inspection", reason, request_id)

    def cancel(self, reason: str = "operator_cancel", *, request_id: str | None = None) -> None:
        """Cancel the active inspection run."""

        self._reason_command("lingtu_nav_client_cancel_inspection", reason, request_id)

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
        """Close a directly owned session; factory views leave it shared."""

        if self._owns_session:
            self._session.close()


def get_native_inspection_command_client(
    *,
    required: bool = False,
) -> NativeInspectionCommandClient | None:
    """Return the inspection view of the process-wide native command session."""

    session = get_native_command_session(required=required)
    if session is None:
        return None
    return NativeInspectionCommandClient._from_session(session)
