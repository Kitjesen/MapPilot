from __future__ import annotations

import asyncio
import json

import pytest

pytest.importorskip("fastapi")


class FakeMaps:
    def __init__(self) -> None:
        self.commands: list[dict[str, object]] = []

    def execute(self, request):
        self.commands.append(request.to_mapping())
        raise AssertionError("maps service must not run during a reserved transition")


def test_normal_activation_observes_session_reservation_inside_shared_lock() -> None:
    from gateway.services.map_service import activate_runtime_map

    class Gateway:
        _session_mode = "idle"
        _session_map = None
        _session_pending = True
        _map_mgr = FakeMaps()

    result = activate_runtime_map(Gateway(), "office", lambda _path: {"ok": True})

    assert result["success"] is False
    assert result["reason_code"] == "session_transition_in_progress"
    assert Gateway._map_mgr.commands == []


def test_runtime_staging_observes_session_reservation_inside_shared_lock() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapNameRequest

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMaps()
    gateway._map_mgr = maps
    gateway._session_mode = "idle"
    gateway._session_pending = True
    endpoint = next(
        route.endpoint
        for route in gateway._app.routes
        if route.path == "/api/v1/map/stage-for-runtime-switch"
    )

    response = asyncio.run(endpoint(MapNameRequest(name="office")))
    payload = json.loads(response.body)

    assert response.status_code == 409
    assert payload["reason_code"] == "session_transition_in_progress"
    assert maps.commands == []
