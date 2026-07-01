from __future__ import annotations


def test_full_stack_blueprint_construction_does_not_touch_service_manager(monkeypatch):
    import core.service_manager as service_manager
    from core.blueprints.full_stack import full_stack_blueprint

    def fail_get_service_manager():
        raise AssertionError("service manager should not be touched while building Blueprint")

    monkeypatch.setattr(service_manager, "get_service_manager", fail_get_service_manager)

    bp = full_stack_blueprint(
        robot="stub",
        slam_profile="none",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=False,
        enable_map_modules=False,
        run_startup_checks=False,
        manage_external_services=True,
    )

    assert bp is not None


class _FakeServiceManager:
    def __init__(self):
        self.calls: list[tuple[str, tuple[str, ...], float | None]] = []

    def stop(self, *services: str) -> None:
        self.calls.append(("stop", tuple(services), None))

    def ensure(self, *services: str) -> None:
        self.calls.append(("ensure", tuple(services), None))

    def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
        self.calls.append(("wait_ready", tuple(services), timeout))
        return True


def test_external_service_plan_runs_during_module_setup(monkeypatch):
    import core.service_manager as service_manager
    from core.blueprints.full_stack import _external_services_blueprint

    fake = _FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)

    bp = _external_services_blueprint(
        enabled=True,
        driver_module="ThunderDriver",
        slam_profile="fastlio2",
        enable_semantic=True,
        config={},
    )
    assert [entry.name for entry in bp._entries] == ["ExternalServiceManagerModule"]

    entry = bp._entries[0]
    module = entry.module_cls(**entry.config)
    module.setup()

    assert fake.calls == [
        ("stop", ("localizer", "super_lio", "super_lio_relocation"), None),
        ("ensure", ("slam", "slam_pgo", "camera"), None),
        ("wait_ready", ("slam", "slam_pgo"), 10.0),
    ]


def test_external_service_stack_uses_runtime_policy_for_slam_profiles():
    from core.blueprints.stacks.system import external_services
    from core.runtime_policy import slam_switch_plan

    bp = external_services(
        enabled=True,
        driver_module="ThunderDriver",
        slam_profile="super-lio-reloc",
        enable_semantic=False,
        config={},
    )

    plan = slam_switch_plan("super_lio_relocation")
    assert [entry.name for entry in bp._entries] == ["ExternalServiceManagerModule"]
    assert bp._entries[0].config["stop_services"] == plan.stop
    assert bp._entries[0].config["ensure_services"] == plan.ensure
    assert bp._entries[0].config["wait_ready_services"] == plan.wait_ready


def test_bridge_external_service_plan_only_starts_needed_camera():
    from core.blueprints.stacks.system import external_services

    bp = external_services(
        enabled=True,
        driver_module="ThunderDriver",
        slam_profile="bridge",
        enable_semantic=True,
        config={},
    )

    assert [entry.name for entry in bp._entries] == ["ExternalServiceManagerModule"]
    assert bp._entries[0].config["stop_services"] == ()
    assert bp._entries[0].config["ensure_services"] == ("camera",)
    assert bp._entries[0].config["wait_ready_services"] == ()
