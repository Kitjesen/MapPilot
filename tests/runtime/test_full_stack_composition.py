from __future__ import annotations

from lingtu.assembly.products.host import host_blueprint
from lingtu.assembly.stacks.composition import compose_full_stack_modules


def _entry_names(bp) -> set[str]:
    return {entry.name for entry in bp._entries}


def _wire_set(bp) -> set[str]:
    return {f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}" for wire in bp._wires}


def _entry_config(bp, name: str) -> dict:
    return next(entry.config for entry in bp._entries if entry.name == name)



def test_compose_full_stack_modules_builds_minimal_stub_graph() -> None:
    bp = compose_full_stack_modules(
        robot="stub",
        driver_module="StubDogModule",
        slam_profile="none",
        detector="yoloe",
        llm="mock",
        gateway_port=5050,
        enable_semantic=False,
        enable_gateway=False,
        config={},
    )
    names = _entry_names(bp)

    assert "StubDogModule" in names
    assert "nav.skills" in names
    assert "nav.goals" not in names
    assert "PatrolManagerModule" not in names
    assert "TaskSchedulerModule" not in names
    assert "PerceptionModule" not in names
    assert "GatewayModule" not in names


def test_compose_endpoint_only_gateway_uses_camera_relay_without_python_teleop() -> None:
    bp = compose_full_stack_modules(
        robot="stub",
        driver_module="StubDogModule",
        slam_profile="none",
        detector="yoloe",
        llm="mock",
        gateway_port=5050,
        enable_semantic=False,
        enable_gateway=True,
        enable_teleop=True,
        config={"command_output_mode": "endpoint_only", "enable_camera": True},
    )
    names = _entry_names(bp)
    media_entry = next(entry for entry in bp._entries if entry.name == "CameraJpegRelayModule")

    assert "CameraJpegRelayModule" in names
    assert "TeleopModule" not in names
    assert not hasattr(media_entry.module_cls, "velocity_input")
    assert not hasattr(media_entry.module_cls, "cmd_vel")
    assert not hasattr(media_entry.module_cls, "teleop_active")


def test_compose_full_stack_modules_can_disable_goal_service() -> None:
    bp = compose_full_stack_modules(
        robot="stub",
        driver_module="StubDogModule",
        slam_profile="none",
        detector="yoloe",
        llm="mock",
        gateway_port=5050,
        enable_semantic=False,
        enable_gateway=False,
        config={"enable_goals": False},
    )
    names = _entry_names(bp)

    assert "nav.goals" not in names


def test_compose_full_stack_modules_can_disable_endpoint_host_services() -> None:
    bp = compose_full_stack_modules(
        robot="stub",
        driver_module="StubDogModule",
        slam_profile="native_dds",
        detector="yoloe",
        llm="mock",
        gateway_port=5050,
        enable_semantic=False,
        enable_gateway=True,
        config={
            "enable_robot_driver": False,
            "localization_adapter": "cpp_slam_status",
        },
    )
    names = _entry_names(bp)

    assert "hw" not in names
    assert "StubDogModule" not in names


def test_compose_full_stack_modules_can_disable_local_lidar_driver() -> None:
    bp = compose_full_stack_modules(
        robot="thunder",
        driver_module="ThunderDriver",
        slam_profile="native_dds",
        detector="yoloe",
        llm="mock",
        gateway_port=5050,
        enable_semantic=False,
        enable_gateway=False,
        config={
            "enable_robot_driver": False,
            "enable_lidar": False,
            "localization_adapter": "cpp_slam_status",
        },
    )
    names = _entry_names(bp)

    assert "LidarModule" not in names
    assert "ThunderDriver" not in names
    assert "SlamAdapterModule" in names
    assert "SlamBridgeModule" not in names


def test_compose_full_stack_modules_does_not_pass_lidar_transport() -> None:
    bp = compose_full_stack_modules(
        robot="thunder",
        driver_module="ThunderDriver",
        slam_profile="none",
        detector="yoloe",
        llm="mock",
        gateway_port=5050,
        enable_semantic=False,
        enable_gateway=False,
        config={
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
        llm="mock",
        gateway_port=5050,
        enable_semantic=False,
        enable_gateway=False,
        config={
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
        llm="mock",
        gateway_port=5050,
        enable_semantic=False,
        enable_gateway=False,
        config={},
    )

    driver_config = _entry_config(bp, "MujocoDriverModule")
    assert driver_config["publish_lidar"] is False


def test_product_blueprint_keeps_wiring_outside_stack_composition() -> None:
    bp = host_blueprint(
        robot="stub",
        slam_profile="none",
        detector="yoloe",
        llm="mock",
        enable_semantic=False,
        enable_gateway=False,
        run_startup_checks=False,
    )
    names = _entry_names(bp)
    wires = _wire_set(bp)

    assert "StubDogModule" in names
    assert {"host.bus", "nav.commands", "nav.goals", "nav.skills"} <= names
    assert "nav.skills.goal_command->nav.goals.goal_command" in wires
