"""Inspection route service backed exclusively by the native C++ store."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from nav.adapters.native.inspection_store import NativeInspectionStore
from runtime import Module, rpc
from runtime.registry import register


def _inspection_dir(value: str | None) -> Path:
    configured = value or os.environ.get("LINGTU_INSPECTION_DIR", "").strip()
    if configured:
        return Path(configured).expanduser()
    session_root = os.environ.get("LINGTU_SESSION_ROOT", "").strip()
    if session_root:
        return Path(session_root).expanduser() / "inspection"
    return Path.home() / ".lingtu" / "inspection"


@register("inspection_service", "native", description="Native inspection routes and execution")
class Inspection(Module, layer=4):
    """Own inspection route persistence and delegate execution to ``nav.commands``."""

    runtime_id = "nav.inspection"
    SOFT_DEPENDS = ["nav.commands"]

    def __init__(self, data_dir: str | None = None, **config: Any) -> None:
        super().__init__(**config)
        self._data_dir = _inspection_dir(data_dir)
        self._commands: Any | None = None

    def on_system_modules(self, modules: dict[str, Module]) -> None:
        self._commands = modules.get("nav.commands")

    @rpc
    def list_routes(self, map_id: str) -> dict[str, Any]:
        return self._store_call("list", map_id)

    @rpc
    def put_route(self, route: dict[str, Any]) -> dict[str, Any]:
        return self._store_call("put", route)

    @rpc
    def get_route(self, map_id: str, route_id: str) -> dict[str, Any]:
        return self._store_call("get", map_id, route_id)

    @rpc
    def delete_route(self, map_id: str, route_id: str) -> bool:
        self._store_call("delete", map_id, route_id)
        return True

    @rpc
    def status(self) -> dict[str, Any]:
        return self._store_call("status")

    @rpc
    def start_task(
        self,
        task_id: str,
        route_id: str,
        revision: int = 0,
        request_id: str | None = None,
    ) -> bool:
        return self._command(
            "start_inspection_task",
            task_id=task_id,
            route_id=route_id,
            revision=revision,
            request_id=request_id,
        )

    @rpc
    def pause_task(
        self,
        task_id: str,
        reason: str,
        request_id: str | None = None,
    ) -> bool:
        return self._command(
            "pause_inspection_task",
            task_id=task_id,
            reason=reason,
            request_id=request_id,
        )

    @rpc
    def resume_task(
        self,
        task_id: str,
        reason: str,
        request_id: str | None = None,
    ) -> bool:
        return self._command(
            "resume_inspection_task",
            task_id=task_id,
            reason=reason,
            request_id=request_id,
        )

    @rpc
    def cancel_task(
        self,
        task_id: str,
        reason: str,
        request_id: str | None = None,
    ) -> bool:
        return self._command(
            "cancel_inspection_task",
            task_id=task_id,
            reason=reason,
            request_id=request_id,
        )

    def _store_call(self, method: str, *args: Any) -> Any:
        with NativeInspectionStore(self._data_dir) as store:
            return getattr(store, method)(*args)

    def _command(self, method: str, **kwargs: Any) -> bool:
        if self._commands is None:
            raise RuntimeError("native navigation command capability is unavailable")
        operation = getattr(self._commands, method, None)
        if not callable(operation):
            raise RuntimeError(f"native navigation command capability does not implement {method}")
        accepted = operation(**kwargs)
        if accepted is False:
            # A negative native acknowledgement is a normal domain outcome.
            # Preserve it for the HTTP boundary so it can distinguish an
            # invalid task transition from a broken endpoint.
            return False
        if accepted is not True:
            raise RuntimeError(
                f"native navigation command {method} returned an invalid acknowledgement"
            )
        return True


__all__ = ["Inspection"]
