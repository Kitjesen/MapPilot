"""Body HTTP calls preserve the existing native owner and admission evidence."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.gateway_module import GatewayModule
from gateway.routes.body import register_body_routes


@pytest.fixture
def body_api():
    gateway = GatewayModule()
    gateway.setup()

    def receipt(reason, *, request_id, **kwargs):
        return {"accepted": True, "reason": "native_accepted", "request_id": request_id}

    controls = SimpleNamespace(**{
        action: Mock(side_effect=receipt)
        for action in ("stand", "sit", "enable_motors", "disable_motors", "switch_profile")
    })
    gateway.on_system_modules({"nav.commands": controls})
    app = FastAPI()
    register_body_routes(app, gateway)
    with TestClient(app) as http:
        yield http, gateway, controls


@pytest.mark.parametrize(("path", "action", "extra"), [
    ("stand", "stand", {}),
    ("sit", "sit", {}),
    ("motors/enable", "enable_motors", {}),
    ("motors/disable", "disable_motors", {}),
    ("profile", "switch_profile", {"name": "walking"}),
])
def test_body_commands_use_existing_owner_once(body_api, path, action, extra):
    http, _, controls = body_api
    body = {"request_id": "body-1", "client_id": "sdk", **extra}
    first = http.post(f"/api/v1/motion/{path}", json=body)
    repeated = http.post(f"/api/v1/motion/{path}", json=body)
    assert first.status_code == repeated.status_code == 200
    payload = first.json()
    assert payload["command"]["accepted"] is True
    assert payload["command"]["request_id"] == "body-1"
    assert payload["execution_confirmed"] is False
    assert payload["native_ack"]["reason"] == "native_accepted"
    getattr(controls, action).assert_called_once_with(
        reason=f"operator_{action}", request_id="body-1", **extra,
    )


@pytest.mark.parametrize(("blocker", "status"), [("owner", 403), ("estop", 409), ("missing", 409)])
def test_body_commands_keep_admission(body_api, blocker, status):
    http, gateway, controls = body_api
    if blocker == "owner":
        assert gateway._lease.acquire("other", 30)
    elif blocker == "estop":
        gateway._navigation_state = {"authority": "estop"}
    else:
        gateway._nav_commands = None
        gateway._all_modules = {}
    response = http.post("/api/v1/motion/stand", json={"request_id": "body-1", "client_id": "sdk"})
    assert response.status_code == status
    assert response.json()["command"]["accepted"] is False
    controls.stand.assert_not_called()


def test_unknown_native_result_keeps_failure_reason(body_api):
    http, _, controls = body_api
    controls.stand.side_effect = TimeoutError("native ACK timeout")
    response = http.post("/api/v1/motion/stand", json={"request_id": "body-1", "client_id": "sdk"})
    assert response.status_code == 503
    assert response.json()["detail"]["reason_code"] == "posture_ack_unknown"


def test_default_app_registers_body_routes():
    from gateway.app_factory import build_gateway_app

    app = build_gateway_app(GatewayModule())
    assert "/api/v1/motion/stand" in app.openapi()["paths"]
