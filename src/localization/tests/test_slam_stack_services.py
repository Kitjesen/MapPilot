from __future__ import annotations


class _FakeServiceManager:
    def __init__(self):
        self.calls: list[tuple[str, tuple[str, ...]]] = []

    def stop(self, *services: str) -> None:
        self.calls.append(("stop", services))

    def ensure(self, *services: str) -> None:
        self.calls.append(("ensure", services))

    def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
        self.calls.append(("wait_ready", services))
        return True


def _run_external_service_plan(monkeypatch, slam_profile: str):
    import runtime.service_manager as service_manager
    from runtime.blueprints.stacks.system import external_services

    fake = _FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)

    bp = external_services(
        enabled=True,
        driver_module="ThunderDriver",
        slam_profile=slam_profile,
        enable_semantic=False,
        config={},
    )
    assert [entry.name for entry in bp._entries] == ["ExternalServiceManagerModule"]

    entry = bp._entries[0]
    module = entry.module_cls(**entry.config)
    module.setup()

    return fake.calls


def test_slam_stack_factory_does_not_touch_service_manager(monkeypatch):
    import runtime.service_manager as service_manager
    from runtime.blueprints.stacks.slam import slam

    def fail_get_service_manager():
        raise AssertionError("stack factory should not touch service manager")

    monkeypatch.setattr(service_manager, "get_service_manager", fail_get_service_manager)

    slam("super_lio", enable_visual_backup=False, manage_services=True)
    slam("fastlio2", enable_visual_backup=False)


def test_super_lio_profile_starts_experimental_service(monkeypatch):
    fake_calls = _run_external_service_plan(monkeypatch, "super_lio")

    assert ("stop", ("slam", "slam_pgo", "localizer", "super_lio_relocation")) in (
        fake_calls
    )
    assert ("ensure", ("lidar", "super_lio")) in fake_calls
    assert ("wait_ready", ("lidar", "super_lio")) in fake_calls


def test_super_lio_relocation_profile_starts_experimental_service(monkeypatch):
    fake_calls = _run_external_service_plan(monkeypatch, "super_lio_relocation")

    assert ("stop", ("slam", "slam_pgo", "localizer", "super_lio")) in fake_calls
    assert ("ensure", ("lidar", "super_lio_relocation")) in fake_calls
    assert ("wait_ready", ("lidar", "super_lio_relocation")) in fake_calls


def test_fastlio2_profile_stops_super_lio_before_mapping(monkeypatch):
    fake_calls = _run_external_service_plan(monkeypatch, "fastlio2")

    assert ("stop", ("localizer", "super_lio", "super_lio_relocation")) in fake_calls
    assert ("ensure", ("slam", "slam_pgo")) in fake_calls
    assert ("wait_ready", ("slam", "slam_pgo")) in fake_calls


def test_full_stack_treats_explicit_super_lio_relocation_as_external_lidar_owner():
    from runtime.blueprints.full_stack import full_stack_blueprint

    bp = full_stack_blueprint(
        robot="stub",
        slam_profile="super-lio-reloc",
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        localization_adapter="ros2_slam_bridge",
        manage_external_services=False,
        run_startup_checks=False,
    )

    modules = repr(bp)
    assert "SlamBridgeModule" in modules
    assert "LidarModule" not in modules
