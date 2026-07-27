"""Native navigation command boundary exposed as one runtime capability."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Callable

from nav.adapters.native.commands import get_native_navigation_client
from nav.adapters.native.exploration_commands import (
    get_native_exploration_command_client,
)
from nav.adapters.native.inspection_commands import (
    get_native_inspection_command_client,
    normalize_route_revision,
)
from runtime import Module, rpc
from runtime.msgs import NavigationCommandKind, NavigationCommandReceipt
from runtime.registry import register


@register("nav_commands", "native", description="Typed native navigation command boundary")
class Commands(Module, layer=3):
    """Own the process-wide C++ DDS command client.

    Gateway and domain services discover this capability by its stable
    ``nav.commands`` runtime ID. Native client details stay behind this
    boundary, and every operation waits for the endpoint business ACK.
    """

    runtime_id = "nav.commands"

    @rpc
    def send_goal(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        *,
        task_id: str,
        request_id: str,
    ) -> NavigationCommandReceipt:
        """Submit a task-identified navigation goal and return its native ACK."""
        return self._navigation_receipt(
            lambda client: client.start_task(
                float(x),
                float(y),
                float(z),
                float(yaw),
                task_id=task_id,
                request_id=request_id,
            ),
            action="send_goal",
            expected_kind=NavigationCommandKind.GOAL,
            task_id=task_id,
            request_id=request_id,
        )

    @rpc
    def cancel_task(
        self,
        task_id: str,
        reason: str = "cancel",
        request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        """Request cancellation of exactly one task and return its native ACK."""
        return self._navigation_receipt(
            lambda client: client.cancel_task(
                task_id,
                str(reason or "cancel"),
                request_id=request_id,
            ),
            action="cancel_task",
            expected_kind=NavigationCommandKind.CANCEL,
            task_id=task_id,
            request_id=request_id,
        )

    @rpc
    def read_navigation_state(self) -> dict[str, object] | None:
        """Read the latest authoritative native navigation snapshot."""

        client = get_native_navigation_client(required=True)
        if client is None:
            raise RuntimeError("native navigation command boundary is unavailable")
        snapshot = client.read_navigation_state()
        if snapshot is None:
            return None
        if not isinstance(snapshot, Mapping):
            raise RuntimeError("native navigation returned an invalid state mapping")
        active_task = str(snapshot.get("active_task_id") or "").strip()
        active_request = str(snapshot.get("active_request_id") or "").strip()
        if bool(active_task) != bool(active_request):
            raise RuntimeError("native navigation active task_id and request_id must be present together")
        if active_task and active_task == active_request:
            raise RuntimeError("native navigation active task_id and request_id must be distinct")
        return dict(snapshot)

    @rpc
    def get_navigation_task_status(
        self,
        task_id: str,
    ) -> dict[str, object] | None:
        """Read retained lifecycle state for one stable navigation task."""

        task = str(task_id or "").strip()
        if not task:
            raise ValueError("native navigation task_id is required")
        client = get_native_navigation_client(required=True)
        if client is None:
            raise RuntimeError("native navigation command boundary is unavailable")
        status = client.get_navigation_task_status(task)
        if status is None:
            return None
        if not isinstance(status, Mapping):
            raise RuntimeError("native navigation returned an invalid status mapping")
        returned_task = str(status.get("task_id") or "").strip()
        if returned_task != task:
            raise RuntimeError("native navigation returned status for the wrong task_id")
        returned_request = str(status.get("request_id") or "").strip()
        if not returned_request:
            raise RuntimeError("native navigation status request_id is required")
        if returned_request == returned_task:
            raise RuntimeError("native navigation status task_id and request_id must be distinct")
        return dict(status)

    @rpc
    def cancel(self, reason: str = "cancel", request_id: str | None = None) -> bool:
        return self._navigation(lambda client: client.cancel(str(reason or "cancel"), request_id=request_id))

    @rpc
    def send_teleop(
        self,
        vx: float,
        vy: float,
        wz: float,
        request_id: str | None = None,
    ) -> bool:
        return self._navigation(
            lambda client: client.send_teleop(
                float(vx),
                float(vy),
                float(wz),
                request_id=request_id,
            )
        )

    @rpc
    def stop_motion(self, reason: str = "stop", request_id: str | None = None) -> bool:
        return self._navigation(lambda client: client.stop(str(reason or "stop"), request_id=request_id))

    @rpc
    def estop(self, reason: str = "estop", request_id: str | None = None) -> bool:
        return self._navigation(lambda client: client.estop(str(reason or "estop"), request_id=request_id))

    @rpc
    def clear_estop(
        self,
        reason: str = "clear_estop",
        request_id: str | None = None,
    ) -> bool:
        return self._navigation(
            lambda client: client.clear_estop(
                str(reason or "clear_estop"),
                request_id=request_id,
            )
        )

    @rpc
    def resume_autonomy(
        self,
        reason: str = "resume_autonomy",
        request_id: str | None = None,
    ) -> bool:
        return self._navigation(
            lambda client: client.resume_autonomy(
                str(reason or "resume_autonomy"),
                request_id=request_id,
            )
        )

    @rpc
    def start_exploration(
        self,
        session_id: str = "",
        reason: str = "operator_start",
        request_id: str | None = None,
    ) -> bool:
        return self._exploration(
            lambda client: client.start(
                session_id=str(session_id or ""),
                reason=str(reason or "operator_start"),
                request_id=request_id,
            )
        )

    @rpc
    def pause_exploration(
        self,
        reason: str = "operator_pause",
        request_id: str | None = None,
    ) -> bool:
        return self._exploration(
            lambda client: client.pause(
                str(reason or "operator_pause"),
                request_id=request_id,
            )
        )

    @rpc
    def resume_exploration(
        self,
        reason: str = "operator_resume",
        request_id: str | None = None,
    ) -> bool:
        return self._exploration(
            lambda client: client.resume(
                str(reason or "operator_resume"),
                request_id=request_id,
            )
        )

    @rpc
    def stop_exploration(
        self,
        reason: str = "operator_stop",
        request_id: str | None = None,
    ) -> bool:
        return self._exploration(
            lambda client: client.stop(
                str(reason or "operator_stop"),
                request_id=request_id,
            )
        )

    @rpc
    def start_inspection(
        self,
        route_id: str,
        revision: int = 0,
        request_id: str | None = None,
    ) -> bool:
        clean_route_id = str(route_id or "").strip()
        if not clean_route_id:
            raise RuntimeError("inspection route_id is required")
        route_revision = normalize_route_revision(revision)
        return self._inspection(
            lambda client: client.start(
                clean_route_id,
                revision=route_revision,
                request_id=request_id,
            )
        )

    @rpc
    def pause_inspection(
        self,
        reason: str = "operator_pause",
        request_id: str | None = None,
    ) -> bool:
        return self._inspection(
            lambda client: client.pause(
                str(reason or "operator_pause"),
                request_id=request_id,
            )
        )

    @rpc
    def resume_inspection(
        self,
        reason: str = "operator_resume",
        request_id: str | None = None,
    ) -> bool:
        return self._inspection(
            lambda client: client.resume(
                str(reason or "operator_resume"),
                request_id=request_id,
            )
        )

    @rpc
    def cancel_inspection(
        self,
        reason: str = "operator_cancel",
        request_id: str | None = None,
    ) -> bool:
        return self._inspection(
            lambda client: client.cancel(
                str(reason or "operator_cancel"),
                request_id=request_id,
            )
        )

    @staticmethod
    def _navigation_receipt(
        operation: Callable[[Any], Any],
        *,
        action: str,
        expected_kind: NavigationCommandKind,
        task_id: str,
        request_id: str | None,
    ) -> NavigationCommandReceipt:
        task = str(task_id or "").strip()
        request = str(request_id or "").strip()
        if not task:
            raise ValueError(f"native navigation {action} task_id is required")
        if not request:
            raise ValueError(f"native navigation {action} request_id is required")
        if task == request:
            raise ValueError(f"native navigation {action} task_id and request_id must be distinct")

        client = get_native_navigation_client(required=True)
        if client is None:
            raise RuntimeError("native navigation command boundary is unavailable")
        receipt = operation(client)
        if not isinstance(receipt, NavigationCommandReceipt):
            raise RuntimeError(f"native navigation {action} returned an invalid receipt")
        if not isinstance(receipt.accepted, bool):
            raise RuntimeError(f"native navigation {action} returned an invalid accepted value")
        if int(receipt.kind) != int(expected_kind):
            raise RuntimeError(f"native navigation {action} returned the wrong command kind")
        if receipt.task_id != task:
            raise RuntimeError(f"native navigation {action} returned the wrong task_id")
        if not Commands._request_identity_matches(receipt.request_id, request):
            raise RuntimeError(f"native navigation {action} returned the wrong request_id")
        if not receipt.reason.strip():
            raise RuntimeError(f"native navigation {action} returned an empty reason")
        return receipt

    @staticmethod
    def _request_identity_matches(actual: str, requested: str) -> bool:
        return actual == requested or actual.startswith(f"{requested}-clock-retry-")

    @staticmethod
    def _navigation(operation: Callable[[Any], None]) -> bool:
        client = get_native_navigation_client(required=True)
        if client is None:
            raise RuntimeError("native navigation command boundary is unavailable")
        operation(client)
        return True

    @staticmethod
    def _exploration(operation: Callable[[Any], None]) -> bool:
        client = get_native_exploration_command_client(required=True)
        if client is None:
            raise RuntimeError("native exploration command boundary is unavailable")
        operation(client)
        return True

    @staticmethod
    def _inspection(operation: Callable[[Any], None]) -> bool:
        client = get_native_inspection_command_client(required=True)
        if client is None:
            raise RuntimeError("native inspection command boundary is unavailable")
        operation(client)
        return True


__all__ = ["Commands"]
