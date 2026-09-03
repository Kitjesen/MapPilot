from __future__ import annotations

import importlib
import sys
from pathlib import Path

import pytest

from runtime.module import Module
from runtime.registry import clear, register, restore, snapshot


def _entry_classes(bp) -> list[type]:
    return [entry.module_cls for entry in bp._entries]


def _entry_names(bp) -> list[str]:
    return [entry.name for entry in bp._entries]


def test_driver_stack_keeps_runtime_compat_resolution_in_adapter():
    driver_stack_source = Path("src/lingtu/assembly/stacks/driver.py").read_text(encoding="utf-8")
    driver_runtime_source = Path("src/runtime/adapters/driver_runtime.py").read_text(encoding="utf-8")

    removed_driver_adapter = "drivers" + ".adapters"

    assert "import_module" not in driver_stack_source
    assert removed_driver_adapter not in driver_stack_source
    assert removed_driver_adapter not in driver_runtime_source


@pytest.mark.parametrize(
    ("robot", "class_name"),
    [
        ("stub", "StubDogModule"),
        ("sim_mujoco", "MujocoDriverModule"),
        ("sim_endpoint", "SimEndpointDriverModule"),

    ],
)
def test_driver_stack_resolves_runtime_driver_keys(robot, class_name):
    from lingtu.assembly.stacks.driver import driver_name

    saved = snapshot()
    try:
        clear()

        assert driver_name(robot) == class_name
    finally:
        restore(saved)


@pytest.mark.parametrize("robot", ["s100p", "navigate", "sim_gazebo"])
def test_driver_stack_rejects_removed_backend_aliases(robot):
    from lingtu.assembly.stacks.driver import driver_name

    saved = snapshot()
    try:
        clear()

        with pytest.raises(KeyError):
            driver_name(robot)
    finally:
        restore(saved)
@pytest.mark.parametrize("robot", ["thunder", "thunder_remote", "grpc_brainstem"])
def test_driver_stack_rejects_retired_python_hardware_drivers(robot):
    from lingtu.assembly.stacks.driver import driver_name

    saved = snapshot()
    try:
        clear()

        with pytest.raises(
            RuntimeError,
            match=r"real Product.*native lingtu-driver",
        ):
            driver_name(robot)
    finally:
        restore(saved)
@pytest.mark.parametrize(
    ("category", "key"),
    (
        ("driver", "thunder"),
        ("driver", "thunder_remote"),
        ("driver", "grpc_brainstem"),
        ("driver_protocol", "grpc_brainstem"),
    ),
)
def test_driver_runtime_rejects_retired_python_hardware_keys(
    category: str,
    key: str,
) -> None:
    from runtime.adapters.driver_runtime import ensure_driver_runtime_registered

    saved = snapshot()
    try:
        clear()

        with pytest.raises(
            RuntimeError,
            match=r"real Product.*native lingtu-driver",
        ):
            ensure_driver_runtime_registered(category, key)
    finally:
        restore(saved)


def test_driver_stack_rejects_retired_driver_backend_parameter():
    from lingtu.assembly.stacks.driver import driver

    saved = snapshot()
    try:
        clear()

        with pytest.raises(TypeError, match="driver_backend= was removed"):
            driver(driver_backend="ros2")
    finally:
        restore(saved)


def test_lidar_stack_prefers_registered_mid360_module():
    from lingtu.assembly.stacks.lidar import lidar

    saved = snapshot()
    try:
        clear()

        @register("lidar", "mid360")
        class FakeLidar(Module, layer=1):
            pass

        bp = lidar(ip="192.0.2.10")

        assert _entry_classes(bp) == [FakeLidar]
        assert _entry_names(bp) == ["lidar"]
        assert bp._entries[0].config == {"ip": "192.0.2.10"}
    finally:
        restore(saved)


def test_lidar_stack_uses_default_source_without_transport_config():
    from lingtu.assembly.stacks.lidar import lidar

    saved = snapshot()
    try:
        clear()

        @register("lidar", "mid360")
        class FakeLidar(Module, layer=1):
            pass

        bp = lidar()

        assert bp._entries[0].config == {}
    finally:
        restore(saved)


def test_lidar_stack_can_select_mujoco_backend():
    from lingtu.assembly.stacks.lidar import lidar

    saved = snapshot()
    try:
        clear()

        @register("lidar", "mujoco")
        class FakeMujocoLidar(Module, layer=1):
            pass

        bp = lidar(enabled=True, backend="mujoco")

        assert _entry_classes(bp) == [FakeMujocoLidar]
        assert _entry_names(bp) == ["lidar"]
        assert bp._entries[0].config == {}
    finally:
        restore(saved)


def test_lidar_stack_rejects_unknown_backend_without_mid360_fallback():
    from lingtu.assembly.stacks.lidar import lidar

    saved = snapshot()
    try:
        clear()

        @register("lidar", "mid360")
        class FakeLidar(Module, layer=1):
            pass

        with pytest.raises(ValueError, match="Unsupported lidar backend"):
            lidar(enabled=True, backend="ros2")
    finally:
        restore(saved)


@pytest.mark.parametrize("backend", ["dds", "replay"])
def test_lidar_stack_rejects_declared_unimplemented_backends(backend):
    from lingtu.assembly.stacks.lidar import lidar

    with pytest.raises(ValueError, match="declared but not implemented"):
        lidar(enabled=True, backend=backend)


def test_gateway_stack_prefers_registered_interface_modules():
    from lingtu.assembly.stacks.gateway import gateway

    saved = snapshot()
    try:
        clear()

        @register("gateway", "fastapi")
        class FakeGateway(Module, layer=6):
            pass

        @register("mcp", "server")
        class FakeMcp(Module, layer=6):
            pass

        @register("media", "jpeg_relay")
        class FakeCameraRelay(Module, layer=6):
            pass

        bp = gateway(
            port=5051,
            mcp_port=8091,
            enable_teleop=True,
            enable_camera=True,
        )

        assert _entry_classes(bp) == [
            FakeGateway,
            FakeMcp,
            FakeCameraRelay,
        ]
        assert _entry_names(bp) == [
            "GatewayModule",
            "MCPServerModule",
            "CameraJpegRelayModule",
        ]
        assert bp._entries[0].config == {
            "port": 5051,
        }
        assert bp._entries[1].config == {"port": 8091}
        assert bp._entries[2].config == {}
    finally:
        restore(saved)


def test_navigation_stack_uses_registered_skills_with_canonical_alias():
    from lingtu.assembly.stacks.navigation import navigation

    saved = snapshot()
    try:
        clear()

        @register("navigation_skills", "default")
        class FakeNavSkills(Module, layer=6):
            pass

        bp = navigation()

        assert _entry_classes(bp) == [FakeNavSkills]
        assert _entry_names(bp) == ["nav.skills"]
    finally:
        restore(saved)


def test_external_tare_stack_prefers_registered_modules_with_canonical_aliases():
    from lingtu.assembly.stacks.exploration import exploration

    saved = snapshot()
    try:
        clear()

        @register("exploration", "tare")
        class FakeTareExplorer(Module, layer=5):
            pass

        @register("exploration", "supervisor")
        class FakeExplorationSupervisor(Module, layer=5):
            pass

        bp = exploration(
            backend="tare_external",
            tare_warn_timeout_s=1.5,
            tare_supervisor_hz=2.0,
        )

        assert _entry_classes(bp) == [FakeTareExplorer, FakeExplorationSupervisor]
        assert _entry_names(bp) == [
            "TAREExplorerModule",
            "ExplorationSupervisorModule",
        ]
        assert bp._entries[1].config == {"warn_timeout_s": 1.5, "poll_hz": 2.0}
    finally:
        restore(saved)


def test_tare_stack_prefers_registered_bridge_modules_with_canonical_aliases():
    from lingtu.assembly.stacks.exploration import exploration

    saved = snapshot()
    try:
        clear()

        @register("exploration", "tare")
        class FakeTareExplorer(Module, layer=5):
            pass

        @register("exploration", "supervisor")
        class FakeExplorationSupervisor(Module, layer=5):
            pass

        bp = exploration(
            backend="tare",
            tare_warn_timeout_s=1.5,
            tare_supervisor_hz=2.0,
        )

        assert _entry_classes(bp) == [
            FakeTareExplorer,
            FakeExplorationSupervisor,
        ]
        assert _entry_names(bp) == [
            "TAREExplorerModule",
            "ExplorationSupervisorModule",
        ]
        assert bp._entries[0].config["configured_backend"] == "tare"
        assert bp._entries[1].config == {"warn_timeout_s": 1.5, "poll_hz": 2.0}
    finally:
        restore(saved)


def test_perception_stack_prefers_registered_scene_and_camera_modules():
    from lingtu.assembly.stacks.perception import perception

    saved = snapshot()
    try:
        clear()

        @register("camera", "orbbec")
        class FakeCamera(Module, layer=1):
            pass

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        bp = perception(
            detector="bpu",
            force_camera=True,
            camera_rotate=90,
        )

        assert _entry_classes(bp)[:2] == [FakeCamera, FakePerception]
        assert _entry_names(bp)[:2] == ["camera", "PerceptionModule"]
        assert bp._entries[0].config == {"rotate": 90}
        assert bp._entries[1].config["detector"].name == "bpu"
    finally:
        restore(saved)


def test_perception_stack_defaults_external_camera_to_canonical_backend():
    from lingtu.assembly.stacks.perception import perception

    saved = snapshot()
    try:
        clear()

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        bp = perception(
            detector="bpu",
            force_camera=True,
        )

        assert "camera" in _entry_names(bp)
        camera_cls = bp._entries[_entry_names(bp).index("camera")].module_cls
        assert camera_cls.__module__ == "drivers.real.camera.module"
        assert "PerceptionModule" in _entry_names(bp)
    finally:
        restore(saved)


@pytest.mark.parametrize(
    "config",
    [
        {"_driver_cls_name": "ROS2SimDriverModule", "use_driver_camera": True},
    ],
)
def test_perception_stack_skips_camera_resolution_for_driver_camera(
    monkeypatch,
    config,
):
    perception_stack = importlib.import_module("lingtu.assembly.stacks.perception")

    saved = snapshot()
    try:
        clear()

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        monkeypatch.setattr(perception_stack, "optional_stack_module", lambda *a, **kw: None)
        monkeypatch.setattr(
            perception_stack,
            "optional_fallback_module",
            lambda *a, **kw: None,
        )

        bp = perception_stack.perception(
            detector="bpu",
            **config,
        )

        assert _entry_classes(bp) == [FakePerception]
        assert _entry_names(bp) == ["PerceptionModule"]
    finally:
        restore(saved)


def test_perception_stack_resolves_camera_for_mujoco_role(monkeypatch):
    perception_stack = importlib.import_module("lingtu.assembly.stacks.perception")

    saved = snapshot()
    try:
        clear()

        @register("camera", "sim")
        class FakeCamera(Module, layer=1):
            pass

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        monkeypatch.setattr(perception_stack, "optional_stack_module", lambda *a, **kw: None)
        monkeypatch.setattr(
            perception_stack,
            "optional_fallback_module",
            lambda *a, **kw: None,
        )

        bp = perception_stack.perception(
            detector="bpu",
            _driver_cls_name="MujocoDriverModule",
        )

        assert _entry_classes(bp) == [FakeCamera, FakePerception]
        assert _entry_names(bp) == ["camera", "PerceptionModule"]
    finally:
        restore(saved)


def test_perception_stack_resolves_camera_for_external_camera(monkeypatch):
    perception_stack = importlib.import_module("lingtu.assembly.stacks.perception")

    saved = snapshot()
    try:
        clear()

        @register("camera", "orbbec")
        class FakeCamera(Module, layer=1):
            pass

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        monkeypatch.setattr(perception_stack, "optional_stack_module", lambda *a, **kw: None)
        monkeypatch.setattr(
            perception_stack,
            "optional_fallback_module",
            lambda *a, **kw: None,
        )

        bp = perception_stack.perception(
            detector="bpu",
            _driver_cls_name="ROS2SimDriverModule",
            use_driver_camera=False,
        )

        assert _entry_classes(bp) == [FakeCamera, FakePerception]
        assert _entry_names(bp) == ["camera", "PerceptionModule"]
    finally:
        restore(saved)


def test_wiring_context_accepts_canonical_camera_name():
    from lingtu.assembly.wires.context import camera_source

    assert camera_source({"camera"}, driver_module="Driver") == (
        "camera",
        "color_image",
    )
    assert camera_source(set(), driver_module="Driver") == ("Driver", "camera_image")


def test_perception_stack_prefers_registered_optional_tool_modules():
    from lingtu.assembly.stacks.perception import perception

    saved = snapshot()
    try:
        clear()

        @register("camera", "orbbec")
        class FakeCamera(Module, layer=1):
            pass

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        @register("reconstruction", "default")
        class FakeReconstruction(Module, layer=3):
            pass

        @register("reconstruction", "dataset_recorder")
        class FakeDatasetRecorder(Module, layer=3):
            pass

        @register("reconstruction", "keyframe_exporter")
        class FakeKeyframeExporter(Module, layer=3):
            pass

        bp = perception(
            detector="bpu",
            force_camera=True,
            recon_save_dir="/tmp/lingtu-recon",
            recon_server_url="http://127.0.0.1:7890",
        )

        assert _entry_classes(bp) == [
            FakeCamera,
            FakePerception,
            FakeReconstruction,
            FakeDatasetRecorder,
            FakeKeyframeExporter,
        ]
        assert _entry_names(bp) == [
            "camera",
            "PerceptionModule",
            "ReconstructionModule",
            "DatasetRecorderModule",
            "ReconKeyframeExporterModule",
        ]
        assert bp._entries[3].config["save_dir"] == "/tmp/lingtu-recon"
        assert bp._entries[4].config["server_url"] == "http://127.0.0.1:7890"
    finally:
        restore(saved)


def test_planner_stack_prefers_registered_modules_with_canonical_aliases():
    from decision.modules.visual_servo import VisualServoModule
    from lingtu.assembly.stacks.planner import planner

    saved = snapshot()
    try:
        clear()

        @register("semantic_planner", "default")
        class FakeSemanticPlanner(Module, layer=4):
            pass

        @register("llm", "pluggable")
        class FakeLlm(Module, layer=4):
            pass

        bp = planner(llm="mock", save_dir="/tmp/lingtu-semantic")

        assert _entry_classes(bp) == [
            FakeSemanticPlanner,
            FakeLlm,
            VisualServoModule,
        ]
        assert _entry_names(bp) == [
            "SemanticPlannerModule",
            "LLMModule",
            "VisualServoModule",
        ]
        assert bp._entries[0].config == {
            "save_dir": "/tmp/lingtu-semantic",
            "llm_backend": "mock",
        }
        assert bp._entries[1].config == {"backend": "mock"}
    finally:
        restore(saved)


def test_planner_stack_only_adds_vla_when_requested():
    from decision.modules.visual_servo import VisualServoModule
    from lingtu.assembly.stacks.planner import planner

    saved = snapshot()
    try:
        clear()

        @register("semantic_planner", "default")
        class FakeSemanticPlanner(Module, layer=4):
            pass

        @register("llm", "pluggable")
        class FakeLlm(Module, layer=4):
            pass

        @register("vla", "default")
        class FakeVla(Module, layer=4):
            pass

        bp = planner(llm="mock", vla_backend="mock-vla")

        assert _entry_classes(bp) == [
            FakeSemanticPlanner,
            FakeLlm,
            VisualServoModule,
            FakeVla,
        ]
        assert _entry_names(bp) == [
            "SemanticPlannerModule",
            "LLMModule",
            "VisualServoModule",
            "VLAModule",
        ]
        assert bp._entries[3].config == {"backend": "mock-vla"}
    finally:
        restore(saved)


def test_memory_stack_prefers_registered_modules_with_canonical_aliases():
    from lingtu.assembly.stacks.memory import memory

    saved = snapshot()
    try:
        clear()

        @register("semantic", "mapper")
        class FakeSemanticMapper(Module, layer=3):
            pass

        @register("memory", "episodic")
        class FakeEpisodicMemory(Module, layer=3):
            pass

        @register("memory", "tagged_locations")
        class FakeTaggedLocations(Module, layer=3):
            pass

        @register("vector_memory", "default")
        class FakeVectorMemory(Module, layer=3):
            pass

        @register("memory", "temporal")
        class FakeTemporalMemory(Module, layer=3):
            pass

        @register("memory", "mission_logger")
        class FakeMissionLogger(Module, layer=3):
            pass

        bp = memory(save_dir="/tmp/lingtu-semantic")

        assert _entry_classes(bp) == [
            FakeSemanticMapper,
            FakeEpisodicMemory,
            FakeTaggedLocations,
            FakeVectorMemory,
            FakeTemporalMemory,
            FakeMissionLogger,
        ]
        assert _entry_names(bp) == [
            "SemanticMapperModule",
            "EpisodicMemoryModule",
            "TaggedLocationsModule",
            "VectorMemoryModule",
            "TemporalMemoryModule",
            "MissionLoggerModule",
        ]
        assert bp._entries[0].config == {"save_dir": "/tmp/lingtu-semantic"}
        assert bp._entries[3].config == {"persist_dir": "/tmp/lingtu-semantic"}
        assert bp._entries[4].config == {"save_dir": "/tmp/lingtu-semantic"}
    finally:
        restore(saved)


def test_slam_stack_rejects_removed_ros2_adapter_even_if_registered(monkeypatch):
    from lingtu.assembly.stacks.slam import slam

    monkeypatch.setenv("LINGTU_ENABLE_ROS2_COMPAT", "1")
    saved = snapshot()
    try:
        clear()

        @register("localization_adapter", "ros2_slam_bridge")
        class FakeLocalizationAdapter(Module, layer=1):
            pass

        with pytest.raises(ImportError, match="Unsupported localization adapter"):
            slam(
                "native_dds",
                localization_adapter="ros2_slam_bridge",
            )
    finally:
        restore(saved)


def test_slam_stack_rejects_removed_legacy_bridge_registry(monkeypatch):
    from localization.adapters.resolver import localization_adapter_module

    monkeypatch.setenv("LINGTU_ENABLE_ROS2_COMPAT", "1")
    saved = snapshot()
    try:
        clear()

        @register("slam_bridge", "default")
        class FakeSlamBridge(Module, layer=1):
            pass

        with pytest.raises(ImportError, match="Unsupported localization adapter"):
            localization_adapter_module("ros2_slam_bridge")
    finally:
        restore(saved)


def test_native_slam_stack_requires_explicit_adapter_without_importing_ros2_bridge():
    from lingtu.assembly.stacks.slam import slam

    saved = snapshot()
    ros2_bridge_before = sys.modules.get("localization.adapters.ros2.slam_bridge")
    had_ros2_bridge = "localization.adapters.ros2.slam_bridge" in sys.modules
    try:
        clear()
        sys.modules.pop("localization.adapters.ros2.slam_bridge", None)

        with pytest.raises(ValueError, match="requires an explicit localization_adapter"):
            slam("native_dds")

        assert "localization.adapters.ros2.slam_bridge" not in sys.modules
    finally:
        if had_ros2_bridge:
            sys.modules["localization.adapters.ros2.slam_bridge"] = ros2_bridge_before
        else:
            sys.modules.pop("localization.adapters.ros2.slam_bridge", None)
        restore(saved)
