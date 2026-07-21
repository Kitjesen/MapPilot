"""Navigation-command interface over the process-wide native C++ DDS session."""

from __future__ import annotations

import os
from typing import Any

from nav.adapters.native.abi import (
    NativeCommandClientError,
    NativeCommandSession,
    get_native_command_session,
)

NavigationClientError = NativeCommandClientError


class NativeNavigationClient:
    """Narrow Python interface for goal, teleop, and motion-control commands."""

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

    def send_goal(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        *,
        request_id: str | None = None,
    ) -> None:
        """Submit one map-frame navigation goal and wait for its business ACK."""

        self._session.call(
            "lingtu_nav_client_send_goal_with_id",
            str(request_id or "").encode("utf-8"),
            float(x),
            float(y),
            float(z),
            float(yaw),
            self._session.goal_timeout_ms,
        )

    def cancel(self, reason: str = "cancel", *, request_id: str | None = None) -> None:
        """Cancel the active native navigation task."""

        self._session.call(
            "lingtu_nav_client_cancel_with_id",
            str(request_id or "").encode("utf-8"),
            str(reason or "cancel").encode("utf-8"),
            self._session.cancel_timeout_ms,
        )

    def send_teleop(
        self,
        vx: float,
        vy: float,
        wz: float,
        *,
        request_id: str | None = None,
    ) -> None:
        """Submit one body-frame operator velocity request."""

        self._session.call(
            "lingtu_nav_client_send_teleop_with_id",
            str(request_id or "").encode("utf-8"),
            float(vx),
            float(vy),
            float(wz),
            self._session.teleop_timeout_ms,
        )

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
