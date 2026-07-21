from __future__ import annotations

import pytest

from lingtu.assembly.products.thunder import thunder_blueprint
from lingtu.assembly.stacks.composition import compose_full_stack_modules
from lingtu.assembly.stacks.system import gnss


def _entry_names(bp) -> set[str]:
    return {entry.name for entry in bp._entries}


def _wire_set(bp) -> set[str]:
    return {f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}" for wire in bp._wires}


def _entry_config(bp, name: str) -> dict:
    return next(entry.config for entry in bp._entries if entry.name == name)


class _RawConfig:
    def __init__(self, raw: dict):
        self.raw = raw


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
    assert "PatrolManagerModule" not in names
    assert "TaskSchedulerModule" not in names
    assert "nav.safety" in names
    assert "nav.velocity_mux" in names
    assert "ExternalServiceManagerModule" not in names
    assert "PerceptionModule" not in names
    assert "GatewayModule" not in names


def test_compose_endpoint_only_stack_uses_native_safety_and_omits_python_control() -> None:
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
        config={"command_output_mode": "endpoint_only"},
    )
    names = _entry_names(bp)

    assert "nav.safety" not in names
    assert "GeofenceManagerModule" not in names
    assert "nav.velocity_mux" not in names


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
            "enable_hw": False,
            "enable_robot_driver": False,
            "localization_adapter": "dds_endpoint",
            "manage_session_services": False,
        },
    )
    names = _entry_names(bp)
    gateway_entry = next(entry for entry in bp._entries if entry.name == "GatewayModule")

    assert "hw" not in names
    assert "ExternalServiceManagerModule" not in names
    assert "StubDogModule" not in names
    assert gateway_entry.config["manage_session_services"] is False


def test_compose_full_stack_modules_does_not_add_legacy_sim_lidar_by_scene_xml_only() -> None:
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
        scene_xml="sim/worlds/mujoco/building_scene.xml",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        manage_external_services=False,
        config={},
    )

    assert "SimPointCloudProvider" not in _entry_names(bp)


def test_compose_full_stack_modules_keeps_legacy_sim_lidar_opt_in() -> None:
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
        scene_xml="sim/worlds/mujoco/building_scene.xml",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        manage_external_services=False,
        config={"enable_legacy_sim_lidar": True},
    )

    assert "SimPointCloudProvider" in _entry_names(bp)


def test_compose_full_stack_modules_honors_session_service_env_override(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_MANAGE_SESSION_SERVICES", "0")

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
            "enable_hw": False,
            "enable_robot_driver": False,
            "localization_adapter": "dds_endpoint",
        },
    )
    gateway_entry = next(entry for entry in bp._entries if entry.name == "GatewayModule")

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
            "enable_hw": False,
            "enable_robot_driver": False,
            "enable_lidar": False,
            "localization_adapter": "dds_endpoint",
        },
    )
    names = _entry_names(bp)

    assert "LidarModule" not in names
    assert "ThunderDriver" not in names
    assert "SlamAdapterModule" in names
    assert "SlamBridgeModule" not in names


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
            "enable_hw": False,
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
            "enable_hw": False,
            "enable_robot_driver": False,
            "enable_lidar": True,
            "lidar_ip": "192.0.2.10",
            "lidar_start_driver": True,
        },
    )

    assert _entry_config(default_bp, "lidar") == {"ip": "192.0.2.10"}
    assert _entry_config(explicit_bp, "lidar") == {
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
            "enable_hw": False,
            "enable_robot_driver": False,
            "enable_lidar": True,
        },
    )

    assert "transport" not in _entry_config(bp, "lidar")


def test_compose_full_stack_modules_uses_mujoco_lidar_backend_for_mujoco_driver() -> None:
    bp = compose_full_stack_modules(
        robot="sim_mujoco",
        driver_module="MujocoDriverModule",
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
            "enable_hw": False,
            "enable_lidar": True,
        },
    )

    assert "lidar" in _entry_names(bp)
    assert _entry_config(bp, "MujocoDriverModule")["publish_lidar"] is False


def test_compose_full_stack_modules_keeps_mujoco_driver_sensor_publish_off_by_default() -> None:
    bp = compose_full_stack_modules(
        robot="sim_mujoco",
        driver_module="MujocoDriverModule",
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
        config={"enable_hw": False},
    )

    driver_config = _entry_config(bp, "MujocoDriverModule")
    assert driver_config["publish_lidar"] is False
    assert driver_config["publish_imu"] is False


def test_compose_full_stack_modules_adds_imu_role_when_enabled() -> None:
    bp = compose_full_stack_modules(
        robot="sim_mujoco",
        driver_module="MujocoDriverModule",
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
            "enable_hw": False,
            "enable_imu": True,
        },
    )

    assert "imu" in _entry_names(bp)
    assert _entry_config(bp, "MujocoDriverModule")["publish_imu"] is False


def test_gnss_stack_prefers_native_service_for_configured_serial_device(monkeypatch) -> None:
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

    assert "gnss" not in names
    assert "GnssModule" not in names
    assert "GnssBridgeModule" not in names


def test_gnss_stack_does_not_use_devices_yaml_inventory_as_dataflow(monkeypatch) -> None:
    class FakeConfig:
        raw = {
            "gnss": {
                "enabled": True,
                "model": "WTRTK-980",
                "device": "/dev/wtrtk980",
                "baud": 115200,
            },
            "devices": {
                "wtrtk980_main": {
                    "type": "gnss",
                    "enabled": True,
                    "driver": "serial_nmea0183",
                }
            },
        }

    monkeypatch.setattr("runtime.config.get_config", lambda: FakeConfig())

    bp = gnss()
    names = _entry_names(bp)

    assert "gnss" not in names
    assert "GnssModule" not in names
    assert "GnssBridgeModule" not in names


def test_gnss_stack_can_use_hw_bridge_explicitly(monkeypatch) -> None:
    class FakeConfig:
        raw = {
            "gnss": {
                "enabled": True,
                "model": "WTRTK-980",
                "device": "/dev/wtrtk980",
                "baud": 115200,
                "hw_bridge": True,
            }
        }

    monkeypatch.setattr("runtime.config.get_config", lambda: FakeConfig())

    bp = gnss()
    names = _entry_names(bp)
    gnss_config = _entry_config(bp, "gnss")
    bridge_config = _entry_config(bp, "GnssBridgeModule")

    assert "gnss" in names
    assert "GnssModule" not in names
    assert "GnssBridgeModule" in names
    assert gnss_config["serial_port"] is None
    assert gnss_config["source_backend"] == "hw"
    assert bridge_config["device_id"] == "wtrtk980_main"
    assert bridge_config["gnss_module_name"] == "gnss"


def test_gnss_stack_can_use_backend_key_for_dds_without_serial_or_hw(monkeypatch) -> None:
    class FakeConfig:
        raw = {
            "gnss": {
                "enabled": True,
                "model": "WTRTK-980",
                "device": "/dev/wtrtk980",
                "baud": 115200,
                "gnss_backend": "dds",
            }
        }

    monkeypatch.setattr("runtime.config.get_config", lambda: FakeConfig())

    bp = gnss()
    names = _entry_names(bp)
    gnss_config = _entry_config(bp, "gnss")

    assert "gnss" in names
    assert "GnssBridgeModule" not in names
    assert gnss_config["serial_port"] is None
    assert gnss_config["source_backend"] == "dds"


def test_gnss_stack_can_use_replay_backend_without_serial_or_hw(monkeypatch) -> None:
    class FakeConfig:
        raw = {
            "gnss": {
                "enabled": True,
                "model": "WTRTK-980",
                "device": "/dev/wtrtk980",
                "baud": 115200,
                "gnss_backend": "replay",
            }
        }

    monkeypatch.setattr("runtime.config.get_config", lambda: FakeConfig())

    bp = gnss()
    names = _entry_names(bp)
    gnss_config = _entry_config(bp, "gnss")

    assert "gnss" in names
    assert "GnssBridgeModule" not in names
    assert gnss_config["serial_port"] is None
    assert gnss_config["source_backend"] == "replay"


def test_gnss_stack_rejects_unknown_explicit_backend(monkeypatch) -> None:
    monkeypatch.setattr(
        "runtime.config.get_config",
        lambda: _RawConfig(
            {
                "gnss": {
                    "enabled": True,
                    "model": "WTRTK-980",
                    "device": "/dev/wtrtk980",
                    "gnss_backend": "device_manager",
                }
            }
        ),
    )

    with pytest.raises(ValueError, match="Unsupported gnss backend"):
        gnss()

    with pytest.raises(ValueError, match="Unsupported gnss backend"):
        gnss(backend="serial")


@pytest.mark.parametrize("backend", ["dds", "replay"])
def test_gnss_stack_enabled_argument_can_create_non_serial_role_without_config(
    monkeypatch,
    backend: str,
) -> None:
    monkeypatch.setattr("runtime.config.get_config", lambda: _RawConfig({}))

    bp = gnss(enabled=True, backend=backend)
    gnss_config = _entry_config(bp, "gnss")

    assert "gnss" in _entry_names(bp)
    assert "GnssBridgeModule" not in _entry_names(bp)
    assert gnss_config["serial_port"] is None
    assert gnss_config["source_backend"] == backend


def test_gnss_stack_enabled_argument_can_create_hw_bridge_without_config(monkeypatch) -> None:
    monkeypatch.setattr("runtime.config.get_config", lambda: _RawConfig({}))

    bp = gnss(enabled=True, backend="hw")
    gnss_config = _entry_config(bp, "gnss")
    bridge_config = _entry_config(bp, "GnssBridgeModule")

    assert "gnss" in _entry_names(bp)
    assert "GnssBridgeModule" in _entry_names(bp)
    assert gnss_config["serial_port"] is None
    assert gnss_config["source_backend"] == "hw"
    assert bridge_config["device_id"] == "wtrtk980_main"
    assert bridge_config["gnss_module_name"] == "gnss"


def test_gnss_stack_backend_argument_overrides_config_heuristic(monkeypatch) -> None:
    class FakeConfig:
        raw = {
            "gnss": {
                "enabled": True,
                "model": "WTRTK-980",
                "device": "/dev/wtrtk980",
                "baud": 115200,
            }
        }

    monkeypatch.setattr("runtime.config.get_config", lambda: FakeConfig())

    bp = gnss(backend="hw")
    names = _entry_names(bp)
    gnss_config = _entry_config(bp, "gnss")

    assert "gnss" in names
    assert "GnssBridgeModule" in names
    assert gnss_config["serial_port"] is None
    assert gnss_config["source_backend"] == "hw"


def test_product_blueprint_keeps_wiring_outside_stack_composition() -> None:
    bp = thunder_blueprint(
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
