"""HTTP contracts against a fake Host boundary, without drivers or simulation."""

from __future__ import annotations

import json
import threading
import time
from dataclasses import asdict, replace
from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes import sensor_control
from message.generated.enums import SensorControlKind
from runtime.contracts.sensor_control import SensorControlRequest, SensorControlResult

URL = "/api/v1/sensors/front/control"
BINDING = {
    "product_session_id": "product-current",
    "session_id": "source-session",
    "sensor_id": "front",
    "model_generation": 7,
    "reset_generation": 3,
}


class FakeSourceModule:
    def __init__(self):
        self.commands: list[SensorControlRequest] = []
        self.binding_thread = None
        self.execute_thread = None
        self.result = SensorControlResult(
            request_id="operator-1",
            **BINDING,
            applied_sim_time_ns=123456,
            applied=True,
            reason="owner_ack",
            settings={"recording": False, "rate_hz": 8.0},
        )

    def binding(self, sensor_id):
        self.binding_thread = threading.get_ident()
        if sensor_id != "front":
            raise KeyError(sensor_id)
        return dict(BINDING)

    def execute(self, command):
        assert isinstance(command, SensorControlRequest)
        self.execute_thread = threading.get_ident()
        self.commands.append(command)
        return self.result


def body(operation="query", **params):
    return {
        "request_id": "operator-1",
        "expected_product_session_id": BINDING["product_session_id"],
        "operation": operation,
        "params": params,
    }


@pytest.fixture
def boundary():
    source = FakeSourceModule()
    gateway = SimpleNamespace(_sensor_control_module=source, settings={"recording": True})
    app = FastAPI()
    sensor_control.register_sensor_control_routes(app, gateway)
    with TestClient(app) as client:
        yield source, gateway, client


@pytest.mark.parametrize(
    "operation,params,kind,expected",
    [
        ("query", {}, SensorControlKind.QUERY, {}),
        ("enabled", {"enabled": False}, SensorControlKind.SET_ENABLED, {"enabled": False}),
        ("rate", {"rate_hz": 12}, SensorControlKind.SET_RATE, {"rate_hz": 12.0}),
        (
            "noise",
            {"enabled": True, "unit": "K", "stddev": 0.25, "seed": 2**64 - 1},
            SensorControlKind.CONFIGURE_NOISE,
            {"enabled": True, "noise_unit": "K", "noise_stddev": 0.25, "noise_seed": 2**64 - 1},
        ),
        ("noise_enabled", {"enabled": True}, SensorControlKind.SET_NOISE_ENABLED, {"enabled": True}),
        ("recording", {"enabled": False}, SensorControlKind.SET_RECORDING, {"enabled": False}),
        (
            "gimbal",
            {"pan_deg": -12.5, "tilt_deg": 15},
            SensorControlKind.SET_GIMBAL,
            {"pan_deg": -12.5, "tilt_deg": 15.0},
        ),
    ],
)
def test_typed_commands_use_current_binding_and_return_actual_ack(boundary, operation, params, kind, expected):
    source, gateway, client = boundary
    before = time.time()
    response = client.post(URL, json=body(operation, **params))
    after = time.time()

    assert response.status_code == 200
    assert response.json() == asdict(source.result)
    json.dumps(response.json(), allow_nan=False)
    (command,) = source.commands
    assert command.request_id == "operator-1"
    for key, value in BINDING.items():
        assert getattr(command, key) == value
    assert before + 2 <= command.deadline_s <= after + 2
    assert command.operation is kind
    for key, value in expected.items():
        assert getattr(command, key) == value
    assert source.execute_thread != source.binding_thread
    assert gateway.settings == {"recording": True}


@pytest.mark.parametrize("operation", ["enabled", "noise_enabled", "recording"])
@pytest.mark.parametrize("value", ["false", "true", 0, 1, None, [], {}])
def test_boolean_params_are_never_coerced(boundary, operation, value):
    source, _, client = boundary
    assert client.post(URL, json=body(operation, enabled=value)).status_code == 422
    assert source.commands == []


@pytest.mark.parametrize(
    "operation,params",
    [
        ("query", {"enabled": True}),
        ("enabled", {}),
        ("enabled", {"enabled": True, "rate_hz": 20}),
        ("rate", {}),
        ("rate", {"rate_hz": "20"}),
        ("rate", {"rate_hz": True}),
        ("rate", {"rate_hz": 0}),
        ("rate", {"rate_hz": -1}),
        ("rate", {"rate_hz": 20, "settings": {"unrestricted": True}}),
        ("noise", {"enabled": "false", "unit": "K", "stddev": 1, "seed": 0}),
        ("noise", {"enabled": False, "unit": 1, "stddev": 1, "seed": 0}),
        ("noise", {"enabled": False, "unit": "", "stddev": 1, "seed": 0}),
        ("noise", {"enabled": False, "unit": "K", "stddev": True, "seed": 0}),
        ("noise", {"enabled": False, "unit": "K", "stddev": "1", "seed": 0}),
        ("noise", {"enabled": False, "unit": "K", "stddev": -1, "seed": 0}),
        ("noise", {"enabled": False, "unit": "K", "stddev": 1, "seed": True}),
        ("noise", {"enabled": False, "unit": "K", "stddev": 1, "seed": "1"}),
        ("noise", {"enabled": False, "unit": "K", "stddev": 1, "seed": 1.5}),
        ("noise", {"enabled": False, "unit": "K", "stddev": 1, "seed": -1}),
        ("noise", {"enabled": False, "unit": "K", "stddev": 1, "seed": 2**64}),
        ("noise", {"enabled": False, "unit": "K", "stddev": 1}),
        ("noise_enabled", {}),
        ("recording", {"enabled": False, "recording": True}),
        ("gimbal", {"pan_deg": 0}),
        ("gimbal", {"pan_deg": 0, "tilt_deg": "1"}),
        ("gimbal", {"pan_deg": True, "tilt_deg": 0}),
    ],
)
def test_operation_parameters_are_required_typed_and_whitelisted(boundary, operation, params):
    source, _, client = boundary
    assert client.post(URL, json=body(operation, **params)).status_code == 422
    assert source.commands == []


@pytest.mark.parametrize(
    "updates",
    [
        {"operation": "settings"},
        {"operation": "QUERY"},
        {"operation": 1},
        {"params": None},
        {"params": []},
        {"settings": {"enabled": True}},
        {"model_generation": 999},
        {"reset_generation": 999},
        {"session_id": "forged"},
        {"sensor_id": "other"},
        {"deadline_s": 99999999999},
        {"product_session_id": "forged"},
        {"request_id": ""},
        {"request_id": 12},
        {"request_id": " invalid "},
        {"request_id": "has\0nul"},
        {"request_id": "a" * 256},
        {"request_id": "界" * 100},
        {"expected_product_session_id": ""},
        {"expected_product_session_id": 12},
    ],
)
def test_invalid_envelopes_and_client_identity_overrides_never_execute(boundary, updates):
    source, _, client = boundary
    assert client.post(URL, json={**body(), **updates}).status_code == 422
    assert source.commands == []


@pytest.mark.parametrize("field", ["request_id", "expected_product_session_id", "operation"])
def test_envelope_fields_are_explicit(boundary, field):
    source, _, client = boundary
    payload = body()
    del payload[field]
    assert client.post(URL, json=payload).status_code == 422
    assert source.commands == []


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
@pytest.mark.parametrize(
    "operation,params,field",
    [
        ("rate", {}, "rate_hz"),
        ("noise", {"enabled": False, "unit": "K", "seed": 0}, "stddev"),
        ("gimbal", {"tilt_deg": 0}, "pan_deg"),
        ("gimbal", {"pan_deg": 0}, "tilt_deg"),
    ],
)
def test_nonfinite_request_numbers_are_rejected(boundary, value, operation, params, field):
    source, _, client = boundary
    response = client.post(
        URL,
        content=json.dumps(body(operation, **{**params, field: value})),
        headers={"Content-Type": "application/json"},
    )
    assert response.status_code == 422
    assert source.commands == []


def test_expected_product_session_must_match_binding(boundary):
    source, _, client = boundary
    response = client.post(URL, json={**body(), "expected_product_session_id": "old-product"})
    assert response.status_code == 409
    assert response.json()["error"] == "product_session_mismatch"
    assert response.json()["request_id"] == "operator-1"
    assert source.commands == []


def test_unknown_sensor_is_not_dispatched(boundary):
    source, _, client = boundary
    response = client.post("/api/v1/sensors/unknown/control", json=body())
    assert response.status_code == 404
    assert response.json()["sensor_id"] == "unknown"
    assert source.commands == []


@pytest.mark.parametrize("gateway", [None, SimpleNamespace(), SimpleNamespace(_sensor_control_module=None)])
def test_unconfigured_module_is_explicit_503(gateway):
    app = FastAPI()
    sensor_control.register_sensor_control_routes(app, gateway)
    with TestClient(app) as client:
        response = client.post(URL, json=body())
    assert response.status_code == 503
    assert response.json() == {"error": "sensor_control_unavailable", "request_id": "operator-1"}


@pytest.mark.parametrize("reason", ["invalid_configuration", "unsupported_operation", "generation_mismatch"])
def test_invalid_unit_or_owner_rejection_is_not_reported_as_applied(boundary, reason):
    source, gateway, client = boundary
    source.result = replace(source.result, applied=False, reason=reason, settings={"error": "unit must be K"})
    response = client.post(URL, json=body("noise", enabled=True, unit="Pa", stddev=1, seed=0))
    assert response.status_code == 409
    assert response.json() == asdict(source.result)
    assert response.json()["applied"] is False
    assert source.commands[0].noise_unit == "Pa"
    assert gateway.settings == {"recording": True}


def test_module_timeout_preserves_request_id(boundary, monkeypatch):
    source, _, client = boundary

    def execute(command):
        raise TimeoutError("source owner did not acknowledge")

    monkeypatch.setattr(source, "execute", execute)
    response = client.post(URL, json=body())
    assert response.status_code == 504
    assert response.json() == {
        "error": "sensor_control_timeout",
        "request_id": "operator-1",
        "sensor_id": "front",
    }


def test_http_wait_is_bounded_even_if_execute_is_still_waiting(boundary, monkeypatch):
    source, gateway, client = boundary
    entered = threading.Event()
    release = threading.Event()
    finished = threading.Event()

    def execute(command):
        entered.set()
        try:
            assert release.wait(1.0)
            return source.result
        finally:
            finished.set()

    monkeypatch.setattr(source, "execute", execute)
    monkeypatch.setattr(sensor_control, "CONTROL_TIMEOUT_S", 0.05)
    try:
        response = client.post(URL, json=body("recording", enabled=False))
        assert entered.is_set() and not finished.is_set()
        assert response.status_code == 504
        assert response.json()["request_id"] == "operator-1"
        assert "applied" not in response.json()
        assert gateway.settings == {"recording": True}
    finally:
        release.set()
        assert finished.wait(1.0)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_nonfinite_ack_is_not_serialized_as_success(boundary, value):
    source, _, client = boundary
    source.result = replace(source.result, settings={"noise": {"stddev": value}})
    response = client.post(URL, json=body())
    assert response.status_code == 502
    assert response.json()["error"] == "invalid_sensor_control_ack"
    json.dumps(response.json(), allow_nan=False)


@pytest.mark.parametrize("configured", [False, True])
def test_app_factory_registers_the_bound_control_route(monkeypatch, configured):
    from gateway.app_factory import build_gateway_app

    # Video registration requires unrelated Gateway ports, not this boundary.
    monkeypatch.setattr("gateway.routes.operations.register_video_routes", lambda app, gw: None)
    source = FakeSourceModule()
    gateway = SimpleNamespace(_sensor_control_module=source) if configured else None
    with TestClient(build_gateway_app(gateway)) as client:
        response = client.post(URL, json=body())
    assert response.status_code == (200 if configured else 503)
    assert len(source.commands) == int(configured)
