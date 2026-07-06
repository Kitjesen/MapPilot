from __future__ import annotations

from typing import Any


class _FakeBlueprint:
    def __init__(self) -> None:
        self.build_transport = "not-called"

    def build(self, transport=None) -> str:
        self.build_transport = transport
        return "system"


class _FakeBackend:
    name = "fake-ipc"

    def create_publisher(self, topic):  # pragma: no cover - not needed here
        raise AssertionError(topic)

    def create_subscriber(self, topic, callback):  # pragma: no cover
        raise AssertionError((topic, callback))

    def close(self) -> None:  # pragma: no cover
        pass


def test_local_module_transport_uses_blueprint_default_transport() -> None:
    from runtime.blueprints.profile_builder import module_transport_for_resolved_config

    assert module_transport_for_resolved_config({"_module_transport": "local"}) is None
    assert module_transport_for_resolved_config({}) is None


def test_nonlocal_module_transport_is_wrapped_for_module_ports(monkeypatch) -> None:
    import runtime.transport.factory as factory_mod
    from runtime.blueprints.profile_builder import module_transport_for_resolved_config
    from runtime.transport.abc import TransportStrategy

    calls: list[str] = []

    def fake_create_transport(strategy: str, ros_node=None) -> _FakeBackend:
        del ros_node
        calls.append(strategy)
        return _FakeBackend()

    monkeypatch.setattr(factory_mod, "create_transport", fake_create_transport)

    transport = module_transport_for_resolved_config({"module_transport": "shm"})

    assert calls == [TransportStrategy.SHM]
    assert transport.backend_name == "fake-ipc"


def test_lcm_module_transport_is_not_supported() -> None:
    from runtime.blueprints.profile_builder import module_transport_for_resolved_config

    try:
        module_transport_for_resolved_config({"module_transport": "lcm"})
    except ValueError as exc:
        assert "Unknown strategy: lcm" in str(exc)
    else:  # pragma: no cover - defensive assertion
        raise AssertionError("module_transport=lcm must stay outside ModulePort")


def test_build_system_from_resolved_profile_honors_module_transport(monkeypatch) -> None:
    import runtime.blueprints.profile_builder as builder_mod

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
        "nav",
        {"robot": "thunder", "module_transport": "shm"},
    )

    assert system == "system"
    assert fake_bp.build_transport is sentinel_transport


def test_build_system_from_resolved_profile_keeps_local_default_fresh(monkeypatch) -> None:
    import runtime.blueprints.profile_builder as builder_mod

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
