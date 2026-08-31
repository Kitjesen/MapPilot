"""Navigation-command interface over the process-wide native C++ DDS session."""

from __future__ import annotations

import os
import time
from typing import Any

from nav.adapters.native.abi import (
    NativeCommandClientError,
    NativeCommandSession,
    get_native_command_session,
)
from runtime.msgs import NavigationCommandReceipt

NavigationClientError = NativeCommandClientError


class NativeNavigationClient:
    """Narrow Python interface for task-oriented navigation and safety-control commands."""

    def __init__(
        self,
        library_path: str | os.PathLike[str],
        *,
        domain_id: int = 0,
        timeout_ms: int = 1000,
        goal_timeout_ms: int | None = None,
        cancel_timeout_ms: int | None = None,
        teleop_timeout_ms: int | None = None,
        library: Any | None = None,
    ) -> None:
        self._session = NativeCommandSession(
            library_path,
            domain_id=domain_id,
            timeout_ms=timeout_ms,
            goal_timeout_ms=goal_timeout_ms,
            cancel_timeout_ms=cancel_timeout_ms,
            teleop_timeout_ms=teleop_timeout_ms,
            library=library,
        )
        self._owns_session = True
        try:
            self._session.ensure_navigation_abi()
        except Exception:
            self._session.close()
            raise

    @classmethod
    def _from_session(cls, session: NativeCommandSession) -> NativeNavigationClient:
        instance = cls.__new__(cls)
        instance._session = session
        instance._owns_session = False
        session.ensure_navigation_abi()
        return instance

    def start_task(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float | None,
        *,
        task_id: str,
        request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        """Submit one product navigation task and return its business ACK."""

        receipt = self._session.start_navigation_task(
            str(task_id or ""),
            str(request_id or ""),
            float(x),
            float(y),
            float(z),
            None if yaw is None else float(yaw),
        )
        return NavigationCommandReceipt(**receipt)

    def preview_plan(self, x: float, y: float, z: float) -> dict[str, object]:
        """Return a read-only plan from the native endpoint planner."""

        request_id = f"plan-{os.getpid()}-{time.time_ns()}"
        return self._session.preview_plan(
            request_id,
            float(x),
            float(y),
            float(z),
        )

    def cancel_task(
        self,
        task_id: str,
        reason: str = "cancel",
        *,
        request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        """Cancel one native navigation task."""

        receipt = self._session.cancel_navigation_task(
            str(task_id or ""),
            str(request_id or ""),
            str(reason or "cancel"),
        )
        return NavigationCommandReceipt(**receipt)

    def pause_task(
        self,
        task_id: str,
        reason: str = "operator_pause",
        *,
        request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        """Request a stop-confirmed pause for one native navigation task."""

        receipt = self._session.pause_navigation_task(
            str(task_id or ""),
            str(request_id or ""),
            str(reason or "operator_pause"),
        )
        return NavigationCommandReceipt(**receipt)

    def resume_task(
        self,
        task_id: str,
        reason: str = "operator_resume",
        *,
        request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        """Request continuation of the same paused native navigation task."""

        receipt = self._session.resume_navigation_task(
            str(task_id or ""),
            str(request_id or ""),
            str(reason or "operator_resume"),
        )
        return NavigationCommandReceipt(**receipt)

    def stop(self, reason: str = "stop", *, request_id: str | None = None) -> None:
        """Immediately clear active motion without latching estop."""

        self._send_reason_command("lingtu_nav_client_stop_with_id", reason or "stop", request_id)

    def estop(self, reason: str = "estop", *, request_id: str | None = None) -> None:
        """Latch the native endpoint in a software emergency-stop state."""

        self._send_reason_command("lingtu_nav_client_estop_with_id", reason or "estop", request_id)

    def clear_estop(
        self,
        reason: str = "clear_estop",
        *,
        request_id: str | None = None,
    ) -> None:
        """Release the estop latch without restoring prior motion."""

        self._send_reason_command(
            "lingtu_nav_client_clear_estop_with_id",
            reason or "clear_estop",
            request_id,
        )

    def resume_autonomy(
        self,
        reason: str = "resume_autonomy",
        *,
        request_id: str | None = None,
    ) -> None:
        """Release manual takeover without restoring an old path."""

        self._send_reason_command(
            "lingtu_nav_client_resume_autonomy_with_id",
            reason or "resume_autonomy",
            request_id,
        )

    def resume_autonomy_with_receipt(
        self,
        reason: str = "resume_autonomy",
        *,
        request_id: str | None = None,
    ) -> dict[str, object]:
        """Release manual takeover and return the correlated endpoint ACK."""

        return self._session.resume_autonomy_with_receipt(
            str(request_id or ""),
            str(reason or "resume_autonomy"),
        )

    def _send_reason_command(
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


def get_native_navigation_client(*, required: bool = False) -> NativeNavigationClient | None:
    """Return the navigation view of the process-wide native command session."""

    session = get_native_command_session(required=required)
    if session is None:
        return None
    return NativeNavigationClient._from_session(session)
