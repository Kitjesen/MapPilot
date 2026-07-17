from __future__ import annotations

from gateway.services import native_control


class _Client:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str, str | None]] = []

    def stop(self, reason: str, *, request_id: str | None = None) -> None:
        self.calls.append(("stop", reason, request_id))

    def estop(self, reason: str, *, request_id: str | None = None) -> None:
        self.calls.append(("estop", reason, request_id))

    def clear_estop(self, reason: str, *, request_id: str | None = None) -> None:
        self.calls.append(("clear_estop", reason, request_id))


def test_endpoint_only_control_commands_share_one_native_boundary(monkeypatch):
    client = _Client()
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setattr(
        native_control,
        "get_native_navigation_client",
        lambda *, required=False: client,
    )

    assert native_control.stop("rest_stop", request_id="stop-1") is True
    assert native_control.estop("mcp_estop", request_id="estop-1") is True
    assert native_control.clear_estop("operator_reset", request_id="reset-1") is True
    assert client.calls == [
        ("stop", "rest_stop", "stop-1"),
        ("estop", "mcp_estop", "estop-1"),
        ("clear_estop", "operator_reset", "reset-1"),
    ]


def test_non_endpoint_profile_reports_no_native_delivery_without_client(monkeypatch):
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    monkeypatch.delenv("LINGTU_TELEOP_CMD_DDS", raising=False)
    monkeypatch.setattr(
        native_control,
        "get_native_navigation_client",
        lambda *, required=False: None,
    )

    assert native_control.stop("dev_stop") is False


def test_legacy_native_boundary_is_resolved_by_the_same_policy(monkeypatch):
    client = _Client()
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    monkeypatch.setenv("LINGTU_TELEOP_CMD_DDS", "1")
    required_values: list[bool] = []

    def get_client(*, required: bool = False):
        required_values.append(required)
        return client

    monkeypatch.setattr(native_control, "get_native_navigation_client", get_client)

    assert native_control.stop("legacy_stop") is True
    assert required_values == [True]


def test_conflicting_native_boundary_configuration_fails_closed(monkeypatch):
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_TELEOP_CMD_DDS", "0")

    try:
        native_control.stop("must_not_fallback")
    except ValueError as exc:
        assert "conflicts" in str(exc)
    else:
        raise AssertionError("conflicting control-boundary configuration must fail")


def test_invalid_status_age_configuration_fails_closed(monkeypatch):
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "not-a-number")

    assert native_control.status_is_fresh({"stamp_s": 10.0}, now_s=10.1) is False
