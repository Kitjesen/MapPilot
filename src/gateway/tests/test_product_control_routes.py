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
    return response_or_payload


def test_native_product_rejects_gateway_hot_switch() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/switch")({"profile": "localizer"})
    )

    assert response.status_code == 409
    assert "active Profile/Endpoint" in _payload(response)["message"]


def test_native_product_restart_delegates_to_runtime_plan(monkeypatch) -> None:
    import lingtu.control as product_control
    from gateway.gateway_module import GatewayModule

    calls: list[str] = []

    class Report:
        ok = True

        @staticmethod
        def as_dict():
            return {"schema_version": "lingtu.launch_report.v1", "ok": True}

    class FakeProductControl:
        def restart(self, process_name: str):
            calls.append(process_name)
            return Report()

    monkeypatch.setattr(product_control, "ProductControl", FakeProductControl)
    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()

    response = asyncio.run(_endpoint(gateway, "/api/v1/slam/restart")())
    payload = _payload(response)

    assert calls == ["slam"]
    assert payload["ok"] is True
    assert payload["details"]["schema_version"] == "lingtu.launch_report.v1"
