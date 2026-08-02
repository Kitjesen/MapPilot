"""Inspection-command interface over the process-wide native C++ DDS session."""

from __future__ import annotations

import os
from typing import Any

from nav.adapters.native.abi import (
    NativeCommandClientError,
    NativeCommandSession,
    get_native_command_session,
)

InspectionTaskClientError = NativeCommandClientError
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


class NativeInspectionTaskClient:
    """Task-addressed inspection client with no legacy command fallback."""

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
            self._session.ensure_inspection_task_abi()
        except Exception:
            self._session.close()
            raise

    @classmethod
    def _from_session(cls, session: NativeCommandSession) -> NativeInspectionTaskClient:
        instance = cls.__new__(cls)
        instance._session = session
        instance._owns_session = False
        session.ensure_inspection_task_abi()
        return instance

    def start(
        self,
        task_id: str,
        route_id: str,
        *,
        revision: int = 0,
        request_id: str | None = None,
    ) -> None:
        """Submit a stored route under the caller's stable task identity."""

        clean_task_id = self._require_task_id(task_id)
        clean_route_id = str(route_id or "").strip()
        if not clean_route_id:
            raise ValueError("inspection route id is required")
        route_revision = normalize_route_revision(revision)
        self._session.call(
            "lingtu_nav_client_start_inspection_task",
            clean_task_id.encode("utf-8"),
            str(request_id or "").encode("utf-8"),
            clean_route_id.encode("utf-8"),
            route_revision,
            self._session.goal_timeout_ms,
        )

    def pause(
        self,
        task_id: str,
        reason: str = "operator_pause",
        *,
        request_id: str | None = None,
    ) -> None:
        """Request pause for exactly the supplied inspection task."""

        self._task_reason_command(
            "lingtu_nav_client_pause_inspection_task", task_id, reason, request_id
        )

    def resume(
        self,
        task_id: str,
        reason: str = "operator_resume",
        *,
        request_id: str | None = None,
    ) -> None:
        """Request resume for exactly the supplied inspection task."""

        self._task_reason_command(
            "lingtu_nav_client_resume_inspection_task", task_id, reason, request_id
        )

    def cancel(
        self,
        task_id: str,
        reason: str = "operator_cancel",
        *,
        request_id: str | None = None,
    ) -> None:
        """Request cancellation for exactly the supplied inspection task."""

        self._task_reason_command(
            "lingtu_nav_client_cancel_inspection_task", task_id, reason, request_id
        )

    def _task_reason_command(
        self,
        function_name: str,
        task_id: str,
        reason: str,
        request_id: str | None,
    ) -> None:
        clean_task_id = self._require_task_id(task_id)
        self._session.call(
            function_name,
            clean_task_id.encode("utf-8"),
            str(request_id or "").encode("utf-8"),
            str(reason).encode("utf-8"),
            self._session.cancel_timeout_ms,
        )

    @staticmethod
    def _require_task_id(task_id: str) -> str:
        clean_task_id = str(task_id or "").strip()
        if not clean_task_id:
            raise ValueError("inspection task id is required")
        return clean_task_id

    def close(self) -> None:
        """Close a directly owned session; factory views leave it shared."""

        if self._owns_session:
            self._session.close()


def get_native_inspection_task_client(
    *,
    required: bool = False,
) -> NativeInspectionTaskClient | None:
    """Return the task-addressed inspection view of the shared native session."""

    session = get_native_command_session(required=required)
    if session is None:
        return None
    return NativeInspectionTaskClient._from_session(session)
