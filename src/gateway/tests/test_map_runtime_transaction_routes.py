from __future__ import annotations

import asyncio
import json

import pytest

pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _payload(response_or_payload):
    if hasattr(response_or_payload, "body"):
        return json.loads(response_or_payload.body)
    if hasattr(response_or_payload, "model_dump"):
        return response_or_payload.model_dump()
    return response_or_payload


class FakeMaps:
    def __init__(self) -> None:
        self.active = "warehouse"
        self.commands: list[dict[str, object]] = []

    def execute(self, request):
        command = request.to_mapping()
        self.commands.append(command)
        if command["action"] == "get_active":
            return {"success": True, "active": self.active}
        if command["action"] == "set_active":
            self.active = str(command["name"])
            return {
                "success": True,
                "active": self.active,
                "octomap": f"/maps/{self.active}/octomap.ot",
            }
        raise AssertionError(command)


def test_compatibility_map_use_cannot_bypass_active_session_transaction() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapRequest

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMaps()
    gateway._map_mgr = maps
    gateway._session_mode = "navigating"
    gateway._session_map = "warehouse"

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/maps")(
            MapRequest(action="use", name="office"),
        )
    )
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["detail"]["reason_code"] == "active_session_map_conflict"
    assert maps.commands == []


def test_external_navigation_map_use_fails_closed_without_runtime_ack() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapRequest

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMaps()
    gateway._map_mgr = maps

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/maps")(
            MapRequest(action="set_active", name="office"),
        )
    )
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["detail"]["reason_code"] == "runtime_binding_unconfirmed"
    assert payload["detail"]["transaction"]["state"] == "rollback_failed"
    assert maps.active == "warehouse"
