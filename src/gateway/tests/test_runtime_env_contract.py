from __future__ import annotations

from types import SimpleNamespace

import pytest
from pydantic import ValidationError

from gateway.auth import gateway_api_key_required
from gateway.schemas import (
    NavigationRuntimeBoundary,
    ReadinessProductContract,
    ReadinessRuntimeBoundary,
    RuntimeSwitchPlanRequest,
    SessionResponse,
    SessionStartRequest,
)
from gateway.services.runtime_dataflow import (
    _motion_path,
    _transport_layers,
    build_runtime_dataflow_snapshot,
)
from gateway.services.runtime_status import _runtime_boundary_status
from gateway.services.runtime_switch_plan import build_runtime_switch_plan


def test_session_product_identity_accepts_profile_alias() -> None:
    from gateway.routes.session import _normalize_product_identity

    product, session = _normalize_product_identity(
        SimpleNamespace(_compiled_run_plan=None),
        {"profile": "tracking"},
        "navigating",
    )

    assert product == "tracking"
    assert session == "tracking"


def test_product_switch_requests_do_not_select_the_outer_env() -> None:
    plan = RuntimeSwitchPlanRequest(target_product="nav").model_dump()
    assert "env" not in plan
    assert "current_env" not in plan
    assert "target_env" not in plan

    with pytest.raises(ValidationError):
        RuntimeSwitchPlanRequest()

    for policy_field in ("strategy", "allow_restart"):
        with pytest.raises(ValidationError):
            RuntimeSwitchPlanRequest(
                target_product="nav",
                **{policy_field: "cold" if policy_field == "strategy" else True},
            )


@pytest.mark.parametrize(
    "selector",
    ["env", "current_env", "target_env"],
)
def test_product_switch_requests_reject_outer_env_selectors(
    selector: str,
) -> None:
    with pytest.raises(ValidationError):
        RuntimeSwitchPlanRequest(target_product="nav", **{selector: "sim"})


def test_runtime_status_prefers_run_plan_identity(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("LINGTU_ENV", "real")
    monkeypatch.setenv("LINGTU_PROFILE", "legacy_field_profile")
    gateway = SimpleNamespace(
        _compiled_run_plan=SimpleNamespace(
            product="nav",
            env="sim",
            fingerprint="runtime-fingerprint",
        )
    )

    status = _runtime_boundary_status(gateway)

    assert status["env"] == "sim"
    assert status["product"] == "nav"
    assert status["run_plan_fingerprint"] == "runtime-fingerprint"
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


def test_runtime_switch_plan_resolves_target_in_gateway_env() -> None:
    current_plan = _plan("map", "sim", "current-fingerprint")
    target_plan = _plan("nav", "sim", "target-fingerprint")
    gateway = SimpleNamespace(_compiled_run_plan=current_plan)
    calls: list[tuple[str, str, str | None]] = []

    class FakeControl:
        env = "sim"

        def resolve(self, product: str, *, product_variant: str | None = None):
            calls.append((product, self.env, product_variant))
            return target_plan

    plan = build_runtime_switch_plan(
        RuntimeSwitchPlanRequest(current_product="map", target_product="nav"),
        gw=gateway,
        control=FakeControl(),
    )

    assert plan["ok"] is True
    assert plan["inputs"] == {
        "env": "sim",
        "current_product": "map",
        "target_product": "nav",
        "current_product_source": "request",
        "identity_source": "run_plan",
        "map_name": None,
        "relocalize": True,
        "initial_pose": None,
    }
    assert calls == [("nav", "sim", None)]
    assert plan["run_plan"] == target_plan.as_dict()
    assert plan["control_report"] is None
    assert plan["from"]["identity"]["product"] == "map"
    assert plan["to"]["identity"]["product"] == "nav"
    assert all("endpoint" not in key for key in plan["inputs"])


def test_runtime_switch_plan_resolver_uses_active_sim_env() -> None:
    current_plan = _plan("map", "sim", "current-fingerprint")
    target_plan = _plan("nav", "sim", "target-fingerprint")
    captured: dict[str, str | None] = {}

    class FakeControl:
        env = "sim"

        def resolve(self, product: str, *, product_variant: str | None = None):
            captured["product"] = product
            captured["env"] = self.env
            captured["product_variant"] = product_variant
            return target_plan

    plan = build_runtime_switch_plan(
        RuntimeSwitchPlanRequest(current_product="map", target_product="nav"),
        gw=SimpleNamespace(_compiled_run_plan=current_plan),
        control=FakeControl(),
    )

    assert plan["ok"] is True
    assert captured == {"product": "nav", "env": "sim", "product_variant": None}


@pytest.mark.parametrize(
    ("map_name", "expected_variant", "requires_map"),
    [(None, "live", False), ("survey-map", "map", True)],
)
def test_runtime_switch_plan_previews_exact_explore_variant(
    map_name: str | None,
    expected_variant: str,
    requires_map: bool,
) -> None:
    current_plan = _plan("map", "sim", "current-fingerprint")
    target_plan = _plan(
        "explore",
        "sim",
        f"explore-{expected_variant}-fingerprint",
        product_variant=expected_variant,
        requires_map=requires_map,
    )
    calls: list[tuple[str, str | None]] = []

    class FakeControl:
        env = "sim"

        def resolve(self, product: str, *, product_variant: str | None = None):
            calls.append((product, product_variant))
            return target_plan

    request = RuntimeSwitchPlanRequest(
        current_product="map",
        target_product="explore",
        map_name=map_name,
    )
    preview = build_runtime_switch_plan(
        request,
        gw=SimpleNamespace(_compiled_run_plan=current_plan),
        control=FakeControl(),
    )

    assert preview["ok"] is True
    assert calls == [("explore", expected_variant)]
    assert preview["to"]["identity"]["product_variant"] == expected_variant
    assert ("--map survey-map" in preview["operator_command"]) is requires_map


class _FakeRunPlan:
    def __init__(
        self,
        product: str,
        env: str,
        fingerprint: str,
        *,
        product_variant: str | None = None,
        requires_map: bool = False,
    ) -> None:
        self.product = product
        self.product_variant = product_variant
        self.env = env
        self.fingerprint = fingerprint
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
                "schema": "lingtu.run_plan.v1",
                "product": self.product,
                "product_variant": self.product_variant,
                "env": self.env,
                "fingerprint": self.fingerprint,
                "compiled_against": {},
            },
            "launch": {
                "controller": self.process_control,
                "process_catalog": {"selected": [], "available": []},
                "stop_plan": [],
                "native_process_environment": {},
                "session": dict(self.lifecycle),
                "parameter_profile": None,
                "parameter_overrides": {},
            },
            "host": {
                "config": dict(self.host_config),
                "expected_modules": [],
                "route_contract": None,
                "module_transport": "local",
            },
            "checks": {"contracts": [], "critical_modules": []},
        }


def _plan(
    product: str,
    env: str,
    fingerprint: str,
    *,
    product_variant: str | None = None,
    requires_map: bool = False,
) -> _FakeRunPlan:
    return _FakeRunPlan(
        product,
        env,
        fingerprint,
        product_variant=product_variant,
        requires_map=requires_map,
    )


def test_runtime_switch_requires_active_run_plan() -> None:
    plan = build_runtime_switch_plan(
        RuntimeSwitchPlanRequest(target_product="nav"),
        gw=SimpleNamespace(_compiled_run_plan=None),
    )

    assert plan["ok"] is False
    assert "run_plan_missing" in plan["blockers"]


def test_gateway_auth_uses_fixed_env(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("LINGTU_GATEWAY_REQUIRE_API_KEY", raising=False)
    monkeypatch.setenv("LINGTU_ENV", "real")
    assert gateway_api_key_required() is True

    monkeypatch.setenv("LINGTU_ENV", "sim")
    assert gateway_api_key_required() is False


def test_session_contract_names_field_identity_env_and_product() -> None:
    payload = SessionResponse(mode="idle", env="real", product="teleop").model_dump()

    assert payload["env"] == "real"
    assert payload["product"] == "teleop"
    assert "product_profile" not in payload

    with pytest.raises(ValidationError):
        SessionStartRequest(mode="mapping", product_profile="map")


def test_gateway_consumes_run_plan_identity() -> None:
    from gateway.gateway_module import GatewayModule

    plan = _plan("nav", "sim", "runtime-fingerprint")
    plan.host_config = {
        "command_output_mode": "endpoint_only",
        "hardware_control_boundary": "native_endpoint",
    }

    gateway = GatewayModule(product="nav", run_plan=plan)
    snapshot = gateway._session_snapshot()

    assert gateway._compiled_run_plan is plan
    assert gateway._compiled_run_plan_fingerprint == "runtime-fingerprint"
    assert gateway._compiled_product == "nav"
    assert gateway._session_product == "nav"
    assert snapshot["env"] == "sim"
    assert snapshot["product"] == "nav"
    assert "product_profile" not in snapshot


def test_gateway_rejects_unknown_configuration_fields() -> None:
    from gateway.gateway_module import GatewayModule

    with pytest.raises(TypeError, match="unsupported GatewayModule configuration"):
        GatewayModule(unexpected_identity="unexpected")
