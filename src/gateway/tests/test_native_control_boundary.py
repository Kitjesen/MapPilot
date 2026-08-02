from __future__ import annotations

from types import SimpleNamespace

from gateway.services import native_control
from gateway.services.command_boundary import CommandBoundaryError


class _Client:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str, str | None]] = []

    def stop_motion(self, reason: str, *, request_id: str | None = None) -> bool:
        self.calls.append(("stop", reason, request_id))
        return True

    def estop(self, reason: str, *, request_id: str | None = None) -> bool:
        self.calls.append(("estop", reason, request_id))
        return True

    def clear_estop(self, reason: str, *, request_id: str | None = None) -> bool:
        self.calls.append(("clear_estop", reason, request_id))
        return True


def test_endpoint_only_control_commands_share_one_native_boundary(monkeypatch):
    client = _Client()
    owner = SimpleNamespace(_nav_commands=client)
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")

    assert native_control.stop(owner, "rest_stop", request_id="stop-1") is True
    assert native_control.estop(owner, "mcp_estop", request_id="estop-1") is True
    assert native_control.clear_estop(owner, "operator_reset", request_id="reset-1") is True
    assert client.calls == [
        ("stop", "rest_stop", "stop-1"),
        ("estop", "mcp_estop", "estop-1"),
        ("clear_estop", "operator_reset", "reset-1"),
    ]


def test_control_without_native_client_reports_no_native_delivery(monkeypatch):
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    owner = SimpleNamespace(_nav_commands=None, _all_modules={})
    assert native_control.stop(owner, "dev_stop") is False


def test_native_control_rejects_ambiguous_command_acknowledgement(monkeypatch):
    class _AmbiguousClient:
        def stop_motion(self, reason: str, *, request_id: str | None = None):
            return None

    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    owner = SimpleNamespace(_nav_commands=_AmbiguousClient())

    try:
        native_control.stop(owner, "must_confirm")
    except CommandBoundaryError as exc:
        assert "invalid acknowledgement" in str(exc)
    else:
        raise AssertionError("ambiguous native command acknowledgement must fail closed")


def test_invalid_status_age_configuration_fails_closed(monkeypatch):
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "not-a-number")

    assert native_control.status_is_fresh({"stamp_s": 10.0}, now_s=10.1) is False


def test_gateway_compiled_product_contract_takes_precedence_over_environment(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "local_driver")
    monkeypatch.setenv("LINGTU_HARDWARE_CONTROL_BOUNDARY", "host")
    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.setenv("LINGTU_PRODUCT", "map")

    manifest = SimpleNamespace(
        env="real",
        product="teleop_avoid",
        fingerprint="compiled-fingerprint",
        host_config={
            "command_output_mode": "endpoint_only",
            "hardware_control_boundary": "driver",
        },
    )
    gateway = GatewayModule(run_plan=manifest)

    assert gateway._teleop_dds_enabled is True
    assert gateway._compiled_run_plan is manifest
    assert gateway._compiled_command_output_mode == "endpoint_only"
    assert gateway._compiled_hardware_control_boundary == "driver"
    assert gateway._compiled_product == "teleop_avoid"
    assert gateway._compiled_run_plan_fingerprint == "compiled-fingerprint"


def test_gateway_does_not_treat_local_host_profile_as_product_identity(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_PROFILE", "sim_nav")
    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.delenv("LINGTU_PRODUCT", raising=False)

    gateway = GatewayModule()

    assert not gateway._compiled_product


def test_gateway_rejects_contract_fragments_that_conflict_with_manifest():
    from gateway.gateway_module import GatewayModule

    manifest = SimpleNamespace(
        env="real",
        product="nav",
        fingerprint="manifest-fingerprint",
        host_config={
            "command_output_mode": "endpoint_only",
            "hardware_control_boundary": "driver",
        },
    )

    try:
        GatewayModule(run_plan=manifest, product="map")
    except ValueError as exc:
        assert "product conflicts" in str(exc)
    else:
        raise AssertionError("conflicting Product identity must fail closed")


def test_native_control_uses_owner_compiled_boundary_before_environment(monkeypatch):
    client = _Client()
    owner = SimpleNamespace(
        _nav_commands=client,
        _compiled_command_output_mode="endpoint_only",
    )
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "local_driver")

    assert native_control.endpoint_only_enabled(owner) is True
    assert native_control.stop(owner, "compiled_boundary", request_id="stop-compiled") is True
    assert client.calls == [("stop", "compiled_boundary", "stop-compiled")]


def test_session_product_uses_run_plan_before_environment(monkeypatch):
    from gateway.routes.session import _current_runtime_product

    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.setenv("LINGTU_PRODUCT", "map")
    monkeypatch.setenv("LINGTU_PROFILE", "stub")
    owner = SimpleNamespace(
        _compiled_run_plan=SimpleNamespace(
            env="real",
            product="teleop_avoid",
        ),
        _compiled_product="nav",
        _session_product="map",
    )

    assert _current_runtime_product(owner) == "teleop_avoid"
