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
from runtime.msgs import NavigationCommandKind, NavigationCommandReceipt

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
        max_speed_mps: float | None = None,
        acceptance_radius_m: float | None = None,
    ) -> NavigationCommandReceipt:
        """Submit one product navigation task and return its business ACK."""

        receipt = self._session.start_navigation_task(
            task_id, request_id or "", x, y, z, yaw,
            max_speed_mps=max_speed_mps, acceptance_radius_m=acceptance_radius_m,
        )
        return self._receipt(receipt, NavigationCommandKind.GOAL, task_id, request_id)

    def preview_plan(
        self,
        x: float,
        y: float,
        z: float,
        *,
        acceptance_radius_m: float | None = None,
    ) -> dict[str, object]:
        """Return a read-only plan from the native endpoint planner."""

        request_id = f"plan-{os.getpid()}-{time.time_ns()}"
        return self._session.preview_plan(
            request_id,
            x, y, z,
            acceptance_radius_m=acceptance_radius_m,
        )

    def query_semantic_views(
        self,
        *,
        boot_id: str,
        map_id: str,
        map_content_epoch: int,
        frame_id: str = "map",
        frame_epoch: int = 0,
        reference_z: float = 0.0,
        camera_range_m: float,
        camera_horizontal_fov_rad: float,
        views: list[dict[str, object]] | None = None,
        timeout_ms: int = 3000,
    ) -> dict[str, object]:
        """Return native 3D semantic observation candidates without moving."""

        return self._session.query_semantic_views(
            boot_id=boot_id,
            map_id=map_id,
            map_content_epoch=map_content_epoch,
            frame_id=frame_id,
            frame_epoch=frame_epoch,
            reference_z=reference_z,
            camera_range_m=camera_range_m,
            camera_horizontal_fov_rad=camera_horizontal_fov_rad,
            views=views,
            timeout_ms=timeout_ms,
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
        return self._receipt(receipt, NavigationCommandKind.TASK_CANCEL, task_id, request_id)

    def select_planner(self, name: str, *, request_id: str | None = None) -> dict[str, object]:
        return self._session.select_navigation_planner(name, request_id or "")

    def set_task_speed_limit(
        self, task_id: str, max_speed_mps: float, *, request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        receipt = self._session.set_navigation_task_speed_limit(
            task_id, str(request_id or ""), float(max_speed_mps),
        )
        return self._receipt(receipt, NavigationCommandKind.TASK_SPEED_LIMIT, task_id, request_id)

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
        return self._receipt(receipt, NavigationCommandKind.PAUSE_TASK, task_id, request_id)

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
        return self._receipt(receipt, NavigationCommandKind.RESUME_TASK, task_id, request_id)

    @staticmethod
    def _receipt(raw, kind, task_id, request_id) -> NavigationCommandReceipt:
        receipt = NavigationCommandReceipt(**raw)
        if receipt.kind != int(kind) or receipt.task_id != task_id:
            raise NavigationClientError("native navigation returned mismatched command identity")
        if request_id and not (receipt.request_id == request_id or
                               receipt.request_id.startswith(f"{request_id}-clock-retry-")):
            raise NavigationClientError("native navigation returned the wrong request_id")
        return receipt

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

    def stand(self, reason: str = "stand", *, request_id: str | None = None) -> dict[str, object]:
        return self._session.posture("stand", str(request_id or ""), str(reason or "stand"))

    def sit(self, reason: str = "sit", *, request_id: str | None = None) -> dict[str, object]:
        return self._session.posture("sit", str(request_id or ""), str(reason or "sit"))

    def external_enter(self, reason="external_enter", *, request_id=None):
        return self._session.external_control(16, str(request_id or ""), reason, request_id)

    def external_enable(self, session_id, reason="external_enable", *, request_id=None):
        return self._session.external_control(18, session_id, reason, request_id)

    def external_disable(self, session_id, reason="external_disable", *, request_id=None):
        return self._session.external_control(14, session_id, reason, request_id)

    def external_exit(self, session_id, reason="external_exit", *, request_id=None):
        return self._session.external_control(17, session_id, reason, request_id)

    def external_frame(self, session_id, sequence, joints, input_window):
        return self._session.external_frame(session_id, sequence, joints, input_window)

    def external_heartbeat(self, session_id):
        return self._session.external_heartbeat(session_id)

    def external_status(self):
        return self._session.external_status()

    def enable_motors(
        self, reason: str = "enable_motors", *, request_id: str | None = None,
    ) -> dict[str, object]:
        return self._session.posture("enable_motors", str(request_id or ""), reason)

    def disable_motors(
        self, reason: str = "disable_motors", *, request_id: str | None = None,
    ) -> dict[str, object]:
        return self._session.posture("disable_motors", str(request_id or ""), reason)

    def switch_profile(
        self, name: str, reason: str = "switch_profile", *, request_id: str | None = None,
    ) -> dict[str, object]:
        return self._session.switch_profile(name, str(request_id or ""), reason)

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
