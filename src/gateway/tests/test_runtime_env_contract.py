from __future__ import annotations

from types import SimpleNamespace

import pytest
from pydantic import ValidationError

from gateway.auth import gateway_api_key_required
from gateway.schemas import (
    NavigationRuntimeBoundary,
    ReadinessProductContract,
    ReadinessRuntimeBoundary,
    SessionResponse,
)
from gateway.services.runtime_dataflow import (
    _motion_path,
    _transport_layers,
    build_runtime_dataflow_snapshot,
)
from gateway.services.runtime_status import _runtime_boundary_status


def test_gateway_schema_rejects_unknown_runtime_graph_product() -> None:
    with pytest.raises(ValidationError):
        ReadinessProductContract(product="unknown_product")


def test_runtime_status_reads_bound_gateway_identity(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("LINGTU_ENV", "real")
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "1" * 32)
    monkeypatch.setenv("LINGTU_RUN_PLAN", "/run/lingtu/plan.json")
    monkeypatch.setenv("LINGTU_PROFILE", "legacy_field_profile")
    gateway = SimpleNamespace(
        _compiled_run_plan=SimpleNamespace(
            product="nav",
            env="sim",
        ),
        _compiled_env="sim",
        _compiled_product="nav",
        _compiled_product_session_id="1" * 32,
        _compiled_run_plan_path="/run/lingtu/plan.json",
    )

    monkeypatch.setenv("LINGTU_ENV", "real-after-binding")
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "2" * 32)
    monkeypatch.setenv("LINGTU_RUN_PLAN", "/run/lingtu/other-plan.json")

    status = _runtime_boundary_status(gateway)

    assert status["env"] == "sim"
    assert status["product"] == "nav"
    assert status["state"] == "active"
    assert status["product_session_id"] == "1" * 32
    assert "run_plan_path" not in status
    assert "identity_source" not in status
    assert status["simulation_only"] is True
    assert "endpoint" not in status
    assert "profile" not in status
    assert "product_profile" not in status


def test_runtime_boundary_schemas_expose_env_and_product_only() -> None:
    readiness = ReadinessRuntimeBoundary(env="sim", product="nav").model_dump()
    navigation = NavigationRuntimeBoundary(env="real", product="map").model_dump()
    product_contract = ReadinessProductContract(product="teleop").model_dump()

    for payload in (readiness, navigation):
        assert "env" in payload
        assert "product" in payload
        assert "state" in payload
        assert "product_session_id" in payload
        assert "run_plan_path" not in payload
        assert "identity_source" not in payload
        assert "endpoint" not in payload
        assert "profile" not in payload
    assert product_contract["product"] == "teleop"
    assert "profile" not in product_contract


def test_runtime_dataflow_uses_env() -> None:
    sim_boundary = {"env": "sim"}
    real_boundary = {"env": "real"}

    assert _transport_layers(sim_boundary)["native_dds"]["primary"] is False
    assert _motion_path(sim_boundary)["kind"] == "host_or_simulation"
    assert _transport_layers(real_boundary)["native_dds"]["primary"] is True
    assert _motion_path(real_boundary)["kind"] == "native_field"


def test_runtime_dataflow_requires_active_run_plan() -> None:
    snapshot = build_runtime_dataflow_snapshot(
        SimpleNamespace(_compiled_run_plan=None, _modules={})
    )

    assert snapshot["authoritative"] is False
    assert snapshot["available"] is False
    assert snapshot["error"] == "run_plan_missing"
    assert snapshot["topics"] == []


class _FakeRunPlan:
    def __init__(
        self,
        product: str,
        env: str,
        label: str,
        *,
        product_variant: str | None = None,
        requires_map: bool = False,
    ) -> None:
        self.product = product
        self.product_variant = product_variant
        self.env = env
        self.robot = "doso/thunder_v4"
        del label
        self.process_control = "systemd"
        self.native_nav = {"control_mode": "autonomy"}
        self.host_config: dict[str, object] = {}
        self.lifecycle: dict[str, object] = {
            "product": product,
            "product_variant": product_variant,
            "requires_map": requires_map,
        }

    def as_dict(self) -> dict[str, object]:
        return {
            "identity": {
                "schema": "lingtu.run_plan.v8",
                "robot": self.robot,
                "product": self.product,
                "product_variant": self.product_variant,
                "env": self.env,
            },
            "launch": {
                "controller": self.process_control,
                "process_catalog": {"selected": [], "available": []},
                "stop_before_start": [],
                "native_process_environment": {},
                "session": dict(self.lifecycle),
                "parameters": {},
            },
            "host": {
                "config": dict(self.host_config),
                "expected_modules": [],
                "route_contract": None,
            },
            "checks": {"contracts": [], "critical_modules": []},
        }


def _plan(
    product: str,
    env: str,
    label: str,
    *,
    product_variant: str | None = None,
    requires_map: bool = False,
) -> _FakeRunPlan:
    return _FakeRunPlan(
        product,
        env,
        label,
        product_variant=product_variant,
        requires_map=requires_map,
    )


def test_gateway_auth_uses_fixed_env(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("LINGTU_GATEWAY_REQUIRE_API_KEY", raising=False)
    monkeypatch.setenv("LINGTU_ENV", "real")
    assert gateway_api_key_required() is True

    monkeypatch.setenv("LINGTU_ENV", "sim")
    assert gateway_api_key_required() is False


def test_session_contract_names_field_identity_env_and_product() -> None:
    payload = SessionResponse(
        mode="idle",
        env="real",
        product="teleop",
        product_session_id="product-session-1234",
    ).model_dump()

    assert payload["env"] == "real"
    assert payload["product"] == "teleop"
    assert payload["product_session_id"] == "product-session-1234"
    assert "product_profile" not in payload


def test_gateway_consumes_run_plan_identity() -> None:
    from gateway.gateway_module import GatewayModule

    plan = _plan("nav", "sim", "runtime-plan")
    plan.host_config = {
        "command_output_mode": "endpoint_only",
        "hardware_control_boundary": "native_endpoint",
    }

    gateway = GatewayModule(
        product="nav",
        run_plan=plan,
        product_session_id="1" * 32,
    )
    snapshot = gateway._session_snapshot()

    assert gateway._compiled_run_plan is plan
    assert gateway._compiled_product == "nav"
    assert gateway._compiled_env == "sim"
    assert gateway._compiled_product_session_id == "1" * 32
    assert not hasattr(gateway, "_compiled_run_plan_path")
    assert gateway._session_product == "nav"
    assert snapshot["env"] == "sim"
    assert snapshot["product"] == "nav"
    assert "product_profile" not in snapshot


def test_gateway_rejects_unknown_configuration_fields() -> None:
    from gateway.gateway_module import GatewayModule

    with pytest.raises(TypeError, match="unsupported GatewayModule configuration"):
        GatewayModule(unexpected_identity="unexpected")
