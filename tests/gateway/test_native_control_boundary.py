from __future__ import annotations

from types import SimpleNamespace

from gateway.services import native_control
from gateway.services.command_boundary import CommandBoundaryError
from runtime.msgs import NavigationCommandKind


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


def test_motion_resume_context_reports_mode_specific_fresh_command(monkeypatch):
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "1.0")

    autonomy = native_control.motion_resume_context(
        {
            "stamp_s": 10.0,
            "control_mode": "autonomy",
            "control_authority": {"resume_required": True},
        },
        now_s=10.1,
    )
    teleop_avoid = native_control.motion_resume_context(
        {
            "stamp_s": 10.0,
            "control_mode": "teleop_avoid",
            "control_authority": {"resume_required": True},
        },
        now_s=10.1,
    )

    assert autonomy == {
        "status_fresh": True,
        "observed_control_mode": "autonomy",
        "resume_was_required": True,
        "goal_reissue_required": True,
        "fresh_operator_command_required": False,
    }
    assert teleop_avoid == {
        "status_fresh": True,
        "observed_control_mode": "teleop_avoid",
        "resume_was_required": True,
        "goal_reissue_required": False,
        "fresh_operator_command_required": True,
    }


def test_motion_resume_context_does_not_guess_from_stale_status(monkeypatch):
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "1.0")

    assert native_control.motion_resume_context(
        {
            "stamp_s": 8.0,
            "control_mode": "autonomy",
            "control_authority": {"resume_required": True},
        },
        now_s=10.1,
    ) == {
        "status_fresh": False,
        "observed_control_mode": None,
        "resume_was_required": None,
        "goal_reissue_required": None,
        "fresh_operator_command_required": None,
    }


def test_resume_control_prefers_correlated_native_receipt(monkeypatch):
    class _ReceiptClient:
        def resume_autonomy_with_receipt(
            self,
            reason: str,
            *,
            request_id: str | None = None,
        ) -> dict[str, object]:
            return {
                "accepted": True,
                "kind": int(NavigationCommandKind.RESUME_AUTONOMY),
                "task_id": "",
                "request_id": str(request_id or "native-generated"),
                "reason": "teleop_resume_ready_reassert_command",
                "endpoint_timestamp_s": 123.5,
            }

        def resume_autonomy(self, *_args, **_kwargs):
            raise AssertionError("receipt-capable clients must not use the legacy bool path")

    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    receipt = native_control.resume_control(
        SimpleNamespace(_nav_commands=_ReceiptClient()),
        "operator_resume",
        request_id="resume-1",
    )

    assert receipt["request_id"] == "resume-1"
    assert receipt["reason"] == "teleop_resume_ready_reassert_command"


def test_motion_resume_result_uses_native_reason_over_precommand_snapshot():
    result = native_control.motion_resume_result(
        {
            "accepted": True,
            "kind": int(NavigationCommandKind.RESUME_AUTONOMY),
            "task_id": "",
            "request_id": "resume-1",
            "reason": "teleop_resume_ready_reassert_command",
            "endpoint_timestamp_s": 123.5,
        },
        {
            "status_fresh": True,
            "observed_control_mode": "teleop_avoid",
            "resume_was_required": False,
            "goal_reissue_required": True,
            "fresh_operator_command_required": False,
        },
    )

    assert result == {
        "status_fresh": True,
        "observed_control_mode": "teleop_avoid",
        "resume_was_required": True,
        "goal_reissue_required": False,
        "fresh_operator_command_required": True,
        "native_reason": "teleop_resume_ready_reassert_command",
        "native_request_id": "resume-1",
        "native_endpoint_timestamp_s": 123.5,
    }


def test_gateway_compiled_product_contract_takes_precedence_over_environment(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "local_driver")
    monkeypatch.setenv("LINGTU_HARDWARE_CONTROL_BOUNDARY", "host")
    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.setenv("LINGTU_PRODUCT", "map")

    manifest = SimpleNamespace(
        env="real",
        product="teleop_avoid",
        host_config={
            "command_output_mode": "endpoint_only",
            "hardware_control_boundary": "driver",
        },
    )
    gateway = GatewayModule(run_plan=manifest)

    assert not hasattr(gateway, "_teleop_dds_enabled")
    assert gateway._compiled_run_plan is manifest
    assert gateway._compiled_command_output_mode == "endpoint_only"
    assert gateway._compiled_hardware_control_boundary == "driver"
    assert gateway._compiled_product == "teleop_avoid"


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
