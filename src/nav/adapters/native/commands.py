"""Navigation-command interface over the process-wide native C++ DDS session."""

from __future__ import annotations

import os
from typing import Any

from nav.adapters.native.abi import (
    NativeCommandClientError,
    NativeCommandSession,
    get_native_command_session,
)
from runtime.msgs import NavigationCommandKind, NavigationCommandReceipt

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

    def start_task(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        *,
        task_id: str,
        request_id: str,
    ) -> NavigationCommandReceipt:
        """Submit one product task and return its correlated business ACK."""

        task, request = self._task_identity(task_id, request_id)
        receipt = self._session.start_navigation_task(
            task,
            request,
            float(x),
            float(y),
            float(z),
            float(yaw),
        )
        return self._task_receipt(
            receipt,
            action="start_task",
            expected_kind=NavigationCommandKind.GOAL,
            task_id=task,
            request_id=request,
        )

    def cancel_task(
        self,
        task_id: str,
        reason: str = "cancel",
        *,
        request_id: str,
    ) -> NavigationCommandReceipt:
        """Cancel exactly one product task and return its business ACK."""

        task, request = self._task_identity(task_id, request_id)
        receipt = self._session.cancel_navigation_task(
            task,
            request,
            str(reason or "cancel"),
        )
        return self._task_receipt(
            receipt,
            action="cancel_task",
            expected_kind=NavigationCommandKind.CANCEL,
            task_id=task,
            request_id=request,
        )

    @staticmethod
    def _task_identity(task_id: str, request_id: str) -> tuple[str, str]:
        task = str(task_id or "").strip()
        request = str(request_id or "").strip()
        if not task:
            raise ValueError("navigation task_id is required")
        if not request:
            raise ValueError("navigation request_id is required")
        if task == request:
            raise ValueError("navigation task_id and request_id must be distinct")
        return task, request

    @staticmethod
    def _task_receipt(
        receipt: object,
        *,
        action: str,
        expected_kind: NavigationCommandKind,
        task_id: str,
        request_id: str,
    ) -> NavigationCommandReceipt:
        if not isinstance(receipt, NavigationCommandReceipt):
            raise RuntimeError(f"native navigation {action} returned an invalid receipt")
        if not isinstance(receipt.accepted, bool):
            raise RuntimeError(f"native navigation {action} returned an invalid accepted value")
        if int(receipt.kind) != int(expected_kind):
            raise RuntimeError(f"native navigation {action} returned the wrong command kind")
        if receipt.task_id != task_id:
            raise RuntimeError(f"native navigation {action} returned the wrong task_id")
        if not NativeNavigationClient._request_identity_matches(
            receipt.request_id,
            request_id,
        ):
            raise RuntimeError(f"native navigation {action} returned the wrong request_id")
        if not receipt.reason.strip():
            raise RuntimeError(f"native navigation {action} returned an empty reason")
        return receipt

    @staticmethod
    def _request_identity_matches(actual: str, requested: str) -> bool:
        return actual == requested or actual.startswith(f"{requested}-clock-retry-")

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
