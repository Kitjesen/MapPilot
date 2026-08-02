from __future__ import annotations

from typing import Any

import pytest


class _FakeBlueprint:
    def __init__(self) -> None:
        self.build_transport = "not-called"
        self.route_contract_name = None
        self._module_names: list[str] = []

    @property
    def module_names(self) -> tuple[str, ...]:
        return tuple(self._module_names)

    def build(self, transport=None) -> str:
        self.build_transport = transport
        return "system"

    def route_contract(self, name: str):
        self.route_contract_name = name
        return self

    def require_modules(self, *names: str):
        self._module_names.extend(name for name in names if name not in self._module_names)
        return self


class _FakeBackend:
    name = "fake-ipc"

    def create_publisher(self, topic):  # pragma: no cover - not needed here
        raise AssertionError(topic)

    def create_subscriber(self, topic, callback):  # pragma: no cover
        raise AssertionError((topic, callback))

    def close(self) -> None:  # pragma: no cover
        pass


def test_local_module_transport_uses_blueprint_default_transport() -> None:
    from lingtu.assembly.profile_builder import module_transport_for_resolved_config

    assert module_transport_for_resolved_config({"_module_transport": "local"}) is None
    assert module_transport_for_resolved_config({}) is None


def test_nonlocal_module_transport_is_wrapped_for_module_ports(monkeypatch) -> None:
    import runtime.transport.factory as factory_mod
    from lingtu.assembly.profile_builder import module_transport_for_resolved_config
    from runtime.transport.abc import TransportStrategy

    calls: list[str] = []

    def fake_create_transport(strategy: str, ros_node=None, **kwargs) -> _FakeBackend:
        del ros_node, kwargs
        calls.append(strategy)
        return _FakeBackend()

    monkeypatch.setattr(factory_mod, "create_transport", fake_create_transport)

    transport = module_transport_for_resolved_config({"module_transport": "shm"})

    assert calls == [TransportStrategy.SHM]
    assert transport.backend_name == "fake-ipc"


def test_lcm_module_transport_is_not_supported() -> None:
    from lingtu.assembly.profile_builder import module_transport_for_resolved_config

    try:
        module_transport_for_resolved_config({"module_transport": "zmq"})
    except ValueError as exc:
        assert "Unknown strategy: zmq" in str(exc)
    else:  # pragma: no cover - defensive assertion
        raise AssertionError("unsupported module_transport must stay outside ModulePort")


def test_resolved_endpoint_route_contract_is_boundary_metadata() -> None:
    from lingtu.assembly.profile_builder import (
        route_contract_name_for_resolved_config,
        validate_route_contract_for_resolved_config,
    )

    config = {
        "_profile_adapter": "thunder_dds",
        "_endpoint_transport": "dds",
        "_endpoint_contract": "thunder_dds_v1",
    }

    assert route_contract_name_for_resolved_config(config) == "robot"
    validate_route_contract_for_resolved_config(config)


def test_route_contract_failure_reports_issue_scope_without_masking_error(monkeypatch) -> None:
    import runtime.route_contract as route_contract_mod

    from lingtu.assembly.profile_builder import validate_route_contract_for_resolved_config
    from runtime.route_contract import RouteIssue

    monkeypatch.setattr(route_contract_mod, "load_route_contract", lambda _name: object())
    monkeypatch.setattr(
        route_contract_mod,
        "validate_route_contract",
        lambda _contract: [
            RouteIssue(
                code="topic_consumers_missing",
                scope="/nav/example/status",
                message="topic /nav/example/status has no consumers",
            )
        ],
    )

    with pytest.raises(ValueError) as exc_info:
        validate_route_contract_for_resolved_config(
            {
                "_profile_adapter": "thunder_dds",
                "_endpoint_transport": "dds",
                "_endpoint_contract": "thunder_dds_v1",
            }
        )

    message = str(exc_info.value)
    assert "topic_consumers_missing:/nav/example/status" in message
    assert "has no consumers" in message


def test_build_system_from_resolved_profile_attaches_route_contract(monkeypatch) -> None:
    import lingtu.assembly.profile_builder as builder_mod

    fake_bp = _FakeBlueprint()
    monkeypatch.setattr(
        builder_mod,
        "blueprint_for_resolved_profile",
        lambda profile, config: fake_bp,
    )
    monkeypatch.setattr(
        builder_mod,
        "module_transport_for_resolved_config",
        lambda config: None,
    )

    from lingtu.assembly.products import resolve_product_host_config

    config = resolve_product_host_config("nav")
    config["_endpoint_transport"] = "dds"
    config["_endpoint_contract"] = "thunder_dds_v1"
    builder_mod.build_system_from_resolved_profile("nav", config)

    assert fake_bp.route_contract_name == "robot"


def test_build_system_from_resolved_profile_honors_module_transport(monkeypatch) -> None:
    import lingtu.assembly.profile_builder as builder_mod

    fake_bp = _FakeBlueprint()
    sentinel_transport: Any = object()

    monkeypatch.setattr(
        builder_mod,
        "blueprint_for_resolved_profile",
        lambda profile, config: fake_bp,
    )
    monkeypatch.setattr(
        builder_mod,
        "module_transport_for_resolved_config",
        lambda config: sentinel_transport,
    )

    system = builder_mod.build_system_from_resolved_profile(
        "lite",
        {"robot": "thunder", "module_transport": "shm"},
    )

    assert system == "system"
    assert fake_bp.build_transport is sentinel_transport


def test_build_system_from_resolved_profile_keeps_local_default_fresh(monkeypatch) -> None:
    import lingtu.assembly.profile_builder as builder_mod

    fake_bp = _FakeBlueprint()
    monkeypatch.setattr(
        builder_mod,
        "blueprint_for_resolved_profile",
        lambda profile, config: fake_bp,
    )
    monkeypatch.setattr(
        builder_mod,
        "module_transport_for_resolved_config",
        lambda config: None,
    )

    system = builder_mod.build_system_from_resolved_profile("lite", {"robot": "thunder"})

    assert system == "system"
    assert fake_bp.build_transport is None
