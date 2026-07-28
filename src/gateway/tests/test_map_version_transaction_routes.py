from __future__ import annotations

import asyncio
import json

import pytest

pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


class ActiveMapService:
    def __init__(self) -> None:
        self.commands: list[dict[str, object]] = []

    def execute(self, request):
        command = request.to_mapping()
        self.commands.append(command)
        if command["action"] == "get_active":
            return {"success": True, "active": "warehouse"}
        raise AssertionError(f"active map must not be mutated directly: {command}")


def test_active_map_version_cannot_change_outside_runtime_transaction() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    maps = ActiveMapService()
    gateway._map_mgr = maps

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/maps/{name}/versions/{version}/rollback")(
            "warehouse",
            1,
        )
    )
    payload = json.loads(response.body)

    assert response.status_code == 409
    assert payload["reason_code"] == "active_map_version_transaction_required"
    assert maps.commands == [{"action": "get_active"}]
