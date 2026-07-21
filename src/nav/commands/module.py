"""Native navigation command boundary exposed as one runtime capability."""

from __future__ import annotations

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
        request_id: str | None = None,
    ) -> bool:
        return self._navigation(
            lambda client: client.send_goal(
                float(x),
                float(y),
                float(z),
                float(yaw),
                request_id=request_id,
            )
        )

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
