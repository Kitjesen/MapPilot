from __future__ import annotations


def test_product_blueprint_construction_does_not_touch_service_manager(monkeypatch):
    import runtime.service_manager as service_manager
    from lingtu.assembly.products.thunder import thunder_blueprint

    def fail_get_service_manager():
        raise AssertionError("service manager should not be touched while building Blueprint")

    monkeypatch.setattr(service_manager, "get_service_manager", fail_get_service_manager)

    bp = thunder_blueprint(
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
    def __init__(self, *, ready: bool = True):
        self.calls: list[tuple[str, tuple[str, ...], float | None]] = []
        self.ready = ready

    def stop(self, *services: str) -> None:
        self.calls.append(("stop", tuple(services), None))

    def ensure(self, *services: str) -> None:
        self.calls.append(("ensure", tuple(services), None))

    def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
        self.calls.append(("wait_ready", tuple(services), timeout))
        return self.ready

    def status(self, *services: str) -> dict[str, str]:
        self.calls.append(("status", tuple(services), None))
        return {service: "running" if self.ready else "stopped" for service in services}


def test_external_service_plan_runs_during_module_setup(monkeypatch):
    import runtime.service_manager as service_manager
    from lingtu.assembly.stacks.system import external_services

    fake = _FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)

    bp = external_services(
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
        (
            "stop",
            (
                "slam_pgo",
                "localizer",
                "hba",
                "genz_icp",
                "super_lio",
                "super_lio_relocation",
            ),
            None,
        ),
        ("ensure", ("slam", "camera"), None),
        ("wait_ready", ("slam",), 10.0),
        ("status", ("slam", "camera"), None),
    ]


def test_external_service_setup_fails_when_wait_ready_fails(monkeypatch):
    import pytest
    import runtime.service_manager as service_manager
    from runtime.external_service_module import ExternalServiceManagerModule

    fake = _FakeServiceManager(ready=False)
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)

    module = ExternalServiceManagerModule(
        ensure_services=("lidar",),
        wait_ready_services=("lidar",),
    )

    with pytest.raises(RuntimeError, match="services not ready"):
        module.setup()

    assert module.health()["services"] == {"lidar": "stopped"}
    assert module.health()["ready"] is False


def test_external_service_stack_uses_runtime_policy_for_slam_profiles():
    from lingtu.assembly.stacks.system import external_services
    from runtime.runtime_policy import slam_switch_plan

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


def test_lidar_start_driver_starts_official_lidar_service_before_slam():
    from lingtu.assembly.stacks.system import external_services

    bp = external_services(
        enabled=True,
        driver_module="ThunderDriver",
        slam_profile="fastlio2",
        enable_semantic=False,
        config={"lidar_start_driver": True},
    )

    assert [entry.name for entry in bp._entries] == ["ExternalServiceManagerModule"]
    assert bp._entries[0].config["ensure_services"] == (
        "lidar",
        "slam",
    )
    assert bp._entries[0].config["wait_ready_services"] == (
        "lidar",
        "slam",
    )


def test_default_lidar_does_not_start_official_lidar_service():
    from lingtu.assembly.stacks.system import external_services

    bp = external_services(
        enabled=True,
        driver_module="ThunderDriver",
        slam_profile="fastlio2",
        enable_semantic=False,
        config={},
    )

    assert [entry.name for entry in bp._entries] == ["ExternalServiceManagerModule"]
    assert bp._entries[0].config["ensure_services"] == ("slam",)
    assert bp._entries[0].config["wait_ready_services"] == ("slam",)


def test_official_lidar_service_can_be_disabled_explicitly():
    from lingtu.assembly.stacks.system import external_services

    bp = external_services(
        enabled=True,
        driver_module="ThunderDriver",
        slam_profile="fastlio2",
        enable_semantic=False,
        config={
            "lidar_start_driver": True,
            "manage_livox_driver": False,
        },
    )

    assert bp._entries[0].config["ensure_services"] == ("slam",)
    assert bp._entries[0].config["wait_ready_services"] == ("slam",)


def test_bridge_external_service_plan_only_starts_needed_camera():
    from lingtu.assembly.stacks.system import external_services

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
