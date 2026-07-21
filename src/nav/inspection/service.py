"""Inspection route service backed exclusively by the native C++ store."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from maps.paths import nav_map_root
from nav.adapters.native.inspection_store import NativeInspectionStore
from runtime import Module, rpc
from runtime.registry import register


@register("inspection_service", "native", description="Native inspection routes and execution")
class Inspection(Module, layer=4):
    """Own inspection route persistence and delegate execution to ``nav.commands``."""

    runtime_id = "nav.inspection"
    SOFT_DEPENDS = ["nav.commands"]

    def __init__(self, map_root: str | None = None, **config: Any) -> None:
        super().__init__(**config)
        self._map_root = Path(map_root).expanduser() if map_root else nav_map_root()
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
    def start_route(
        self,
        route_id: str,
        revision: int = 0,
        request_id: str | None = None,
    ) -> bool:
        return self._command(
            "start_inspection",
            route_id=route_id,
            revision=revision,
            request_id=request_id,
        )

    @rpc
    def pause(self, reason: str, request_id: str | None = None) -> bool:
        return self._command(
            "pause_inspection",
            reason=reason,
            request_id=request_id,
        )

    @rpc
    def resume(self, reason: str, request_id: str | None = None) -> bool:
        return self._command(
            "resume_inspection",
            reason=reason,
            request_id=request_id,
        )

    @rpc
    def cancel(self, reason: str, request_id: str | None = None) -> bool:
        return self._command(
            "cancel_inspection",
            reason=reason,
            request_id=request_id,
        )

    def _store_call(self, method: str, *args: Any) -> Any:
        with NativeInspectionStore(self._map_root) as store:
            return getattr(store, method)(*args)

    def _command(self, method: str, **kwargs: Any) -> bool:
        if self._commands is None:
            raise RuntimeError("native navigation command capability is unavailable")
        operation = getattr(self._commands, method, None)
        if not callable(operation):
            raise RuntimeError(f"native navigation command capability does not implement {method}")
        return bool(operation(**kwargs))


__all__ = ["Inspection"]
