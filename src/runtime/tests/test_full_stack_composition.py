from __future__ import annotations

from runtime.blueprints.full_stack import full_stack_blueprint
from runtime.blueprints.stacks.composition import compose_full_stack_modules
from runtime.blueprints.stacks.system import gnss


def _entry_names(bp) -> set[str]:
    return {entry.name for entry in bp._entries}


def _wire_set(bp) -> set[str]:
    return {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }


def _entry_config(bp, name: str) -> dict:
    return next(entry.config for entry in bp._entries if entry.name == name)


def test_compose_full_stack_modules_builds_minimal_stub_graph() -> None:
    bp = compose_full_stack_modules(
        robot="stub",
        driver_module="StubDogModule",
        slam_profile="none",
        detector="yoloe",
        encoder="mobileclip",
        llm="mock",
        planner_backend="astar",
        tomogram="",
        gateway_port=5050,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        manage_external_services=False,
        config={},
    )
    names = _entry_names(bp)

    assert "StubDogModule" in names
    assert "nav.mission" in names
    assert "nav.goals" in names
    assert "PatrolManagerModule" in names
    assert "TaskSchedulerModule" not in names
    assert "nav.safety" in names
    assert "nav.velocity_mux" in names
    assert "ExternalServiceManagerModule" not in names
    assert "PerceptionModule" not in names
    assert "GatewayModule" not in names


def test_compose_full_stack_modules_can_disable_support_services() -> None:
    bp = compose_full_stack_modules(
        robot="stub",
        driver_module="StubDogModule",
        slam_profile="none",
        detector="yoloe",
        encoder="mobileclip",
        llm="mock",
        planner_backend="astar",
        tomogram="",
        gateway_port=5050,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        manage_external_services=False,
        config={
            "enable_goals": False,
            "enable_patrol_routes": False,
            "enable_scheduler": True,
        },
    )
    names = _entry_names(bp)

    assert "nav.goals" not in names
    assert "PatrolManagerModule" not in names
    assert "TaskSchedulerModule" in names


def test_compose_full_stack_modules_can_disable_endpoint_host_services() -> None:
    bp = compose_full_stack_modules(
        robot="stub",
        driver_module="StubDogModule",
        slam_profile="bridge",
        detector="yoloe",
        encoder="mobileclip",
        llm="mock",
        planner_backend="astar",
        tomogram="",
        gateway_port=5050,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        enable_map_modules=False,
        manage_external_services=False,
        config={
            "enable_device_manager": False,
            "enable_robot_driver": False,
            "localization_adapter": "lcm_endpoint",
            "manage_session_services": False,
        },
    )
    names = _entry_names(bp)
    gateway_entry = next(entry for entry in bp._entries if entry.name == "GatewayModule")

    assert "DeviceManager" not in names
    assert "ExternalServiceManagerModule" not in names
    assert "StubDogModule" not in names
    assert gateway_entry.config["manage_session_services"] is False


def test_compose_full_stack_modules_can_disable_local_lidar_driver() -> None:
    bp = compose_full_stack_modules(
        robot="thunder",
        driver_module="ThunderDriver",
        slam_profile="fastlio2",
        detector="yoloe",
        encoder="mobileclip",
        llm="mock",
        planner_backend="astar",
        tomogram="",
        gateway_port=5050,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        manage_external_services=False,
        config={
            "enable_device_manager": False,
            "enable_robot_driver": False,
            "enable_lidar": False,
            "localization_adapter": "lcm_endpoint",
        },
    )
    names = _entry_names(bp)

    assert "LidarModule" not in names
    assert "ThunderDriver" not in names
    assert "SlamBridgeModule" in names


def test_compose_full_stack_modules_keeps_lidar_driver_start_opt_in() -> None:
    default_bp = compose_full_stack_modules(
        robot="thunder",
        driver_module="ThunderDriver",
        slam_profile="none",
        detector="yoloe",
        encoder="mobileclip",
        llm="mock",
        planner_backend="astar",
        tomogram="",
        gateway_port=5050,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        manage_external_services=False,
        config={
            "enable_device_manager": False,
            "enable_robot_driver": False,
            "enable_lidar": True,
            "lidar_ip": "192.0.2.10",
        },
    )
    explicit_bp = compose_full_stack_modules(
        robot="thunder",
        driver_module="ThunderDriver",
        slam_profile="none",
        detector="yoloe",
        encoder="mobileclip",
        llm="mock",
        planner_backend="astar",
        tomogram="",
        gateway_port=5050,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        manage_external_services=False,
        config={
            "enable_device_manager": False,
            "enable_robot_driver": False,
            "enable_lidar": True,
            "lidar_ip": "192.0.2.10",
            "lidar_start_driver": True,
        },
    )

    assert _entry_config(default_bp, "LidarModule") == {"ip": "192.0.2.10"}
    assert _entry_config(explicit_bp, "LidarModule") == {
        "ip": "192.0.2.10",
        "start_driver": True,
    }


def test_compose_full_stack_modules_does_not_pass_lidar_transport() -> None:
    bp = compose_full_stack_modules(
        robot="thunder",
        driver_module="ThunderDriver",
        slam_profile="none",
        detector="yoloe",
        encoder="mobileclip",
        llm="mock",
        planner_backend="astar",
        tomogram="",
        gateway_port=5050,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        manage_external_services=False,
        config={
            "enable_device_manager": False,
            "enable_robot_driver": False,
            "enable_lidar": True,
        },
    )

    assert "transport" not in _entry_config(bp, "LidarModule")


def test_gnss_stack_prefers_configured_serial_device(monkeypatch) -> None:
    class FakeConfig:
        raw = {
            "gnss": {
                "enabled": True,
                "model": "WTRTK-980",
                "device": "/dev/wtrtk980",
                "baud": 460800,
                "topic_fix": "/compat/gps/fix",
                "quality": {
                    "min_sat_used": 10,
                    "max_hdop": 1.8,
                },
            }
        }

    monkeypatch.setattr("runtime.config.get_config", lambda: FakeConfig())

    bp = gnss()
    names = _entry_names(bp)
    gnss_config = _entry_config(bp, "GnssModule")

    assert "GnssModule" in names
    assert "GnssBridgeModule" not in names
    assert gnss_config["serial_port"] == "/dev/wtrtk980"
    assert gnss_config["serial_baud"] == 460800
    assert gnss_config["fix_topic"] == "/compat/gps/fix"
    assert gnss_config["min_sat_used"] == 10
    assert gnss_config["max_hdop"] == 1.8


def test_gnss_stack_can_use_device_manager_bridge_explicitly(monkeypatch) -> None:
    class FakeConfig:
        raw = {
            "gnss": {
                "enabled": True,
                "model": "WTRTK-980",
                "device": "/dev/wtrtk980",
                "baud": 115200,
                "device_manager_bridge": True,
            }
        }

    monkeypatch.setattr("runtime.config.get_config", lambda: FakeConfig())

    bp = gnss()
    names = _entry_names(bp)
    gnss_config = _entry_config(bp, "GnssModule")
    bridge_config = _entry_config(bp, "GnssBridgeModule")

    assert "GnssModule" in names
    assert "GnssBridgeModule" in names
    assert gnss_config["serial_port"] is None
    assert bridge_config["device_id"] == "wtrtk980_main"


def test_full_stack_blueprint_keeps_wiring_outside_stack_composition() -> None:
    bp = full_stack_blueprint(
        robot="stub",
        slam_profile="none",
        detector="yoloe",
        encoder="mobileclip",
        llm="mock",
        planner_backend="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        manage_external_services=False,
        run_startup_checks=False,
    )
    names = _entry_names(bp)
    wires = _wire_set(bp)

    assert "StubDogModule" in names
    assert "nav.mission" in names
    assert "nav.safety" in names
    assert "nav.safety.stop_cmd->StubDogModule.stop_signal" in wires
    assert "nav.safety.stop_cmd->nav.mission.stop_signal" in wires
