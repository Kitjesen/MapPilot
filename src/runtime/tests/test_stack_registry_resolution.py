from __future__ import annotations

import importlib
import sys
from pathlib import Path

import pytest

from runtime.module import Module
from runtime.registry import clear, register, restore, snapshot
from runtime.stream import In, Out


def _entry_classes(bp) -> list[type]:
    return [entry.module_cls for entry in bp._entries]


def _entry_names(bp) -> list[str]:
    return [entry.name for entry in bp._entries]


def test_driver_stack_keeps_runtime_compat_resolution_in_adapter():
    driver_stack_source = Path("src/runtime/blueprints/stacks/driver.py").read_text(encoding="utf-8")
    driver_runtime_source = Path("src/runtime/adapters/driver_runtime.py").read_text(encoding="utf-8")

    removed_driver_adapter = "drivers" + ".adapters"

    assert "import_module" not in driver_stack_source
    assert removed_driver_adapter not in driver_stack_source
    assert removed_driver_adapter not in driver_runtime_source


@pytest.mark.parametrize(
    ("robot", "class_name"),
    [
        ("stub", "StubDogModule"),
        ("thunder", "ThunderDriver"),
        ("sim_mujoco", "MujocoDriverModule"),
        ("sim_endpoint", "SimEndpointDriverModule"),
        ("sim_gazebo", "SimEndpointDriverModule"),
    ],
)
def test_driver_stack_resolves_runtime_driver_keys(robot, class_name):
    from runtime.blueprints.stacks.driver import driver_name

    saved = snapshot()
    try:
        clear()

        assert driver_name(robot) == class_name
    finally:
        restore(saved)


def test_driver_stack_recovers_runtime_driver_after_registry_clear_with_loaded_module():
    from runtime.blueprints.stacks.driver import driver_name

    saved = snapshot()
    try:
        importlib.import_module("drivers.real.thunder.han_dog_module")
        clear()

        assert driver_name("thunder") == "ThunderDriver"
    finally:
        restore(saved)


def test_driver_stack_does_not_resolve_removed_ros2_bridge_profile():
    from runtime.blueprints.stacks.driver import driver

    saved = snapshot()
    try:
        clear()

        with pytest.raises(KeyError):
            driver(profile="ros2")
    finally:
        restore(saved)


def test_safety_stack_prefers_registered_modules():
    from runtime.blueprints.stacks.safety import safety

    saved = snapshot()
    try:
        clear()

        @register("safety", "ring")
        class FakeSafetyRing(Module, layer=0):
            pass

        @register("safety", "cmd_vel_mux")
        class FakeVelocityMux(Module, layer=0):
            pass

        @register("safety", "geofence")
        class FakeGeofence(Module, layer=0):
            pass

        bp = safety()
        classes = _entry_classes(bp)

        assert classes == [FakeSafetyRing, FakeVelocityMux, FakeGeofence]
        assert _entry_names(bp) == [
            "nav.safety",
            "nav.velocity_mux",
            "GeofenceManagerModule",
        ]
    finally:
        restore(saved)


def test_lidar_stack_prefers_registered_mid360_module():
    from runtime.blueprints.stacks.lidar import lidar

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


def test_lidar_stack_only_starts_legacy_driver_when_explicit():
    from runtime.blueprints.stacks.lidar import lidar

    saved = snapshot()
    try:
        clear()

        @register("lidar", "mid360")
        class FakeLidar(Module, layer=1):
            pass

        default_bp = lidar(ip="192.0.2.10")
        explicit_bp = lidar(ip="192.0.2.10", start_driver=True)

        assert default_bp._entries[0].config == {"ip": "192.0.2.10"}
        assert explicit_bp._entries[0].config == {
            "ip": "192.0.2.10",
            "start_driver": True,
        }
    finally:
        restore(saved)


def test_lidar_stack_uses_default_source_without_transport_config():
    from runtime.blueprints.stacks.lidar import lidar

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
    from runtime.blueprints.stacks.lidar import lidar

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
    from runtime.blueprints.stacks.lidar import lidar

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
    from runtime.blueprints.stacks.lidar import lidar

    with pytest.raises(ValueError, match="declared but not implemented"):
        lidar(enabled=True, backend=backend)


def test_imu_stack_can_select_mujoco_backend():
    from runtime.blueprints.stacks.imu import imu

    saved = snapshot()
    try:
        clear()

        @register("imu", "mujoco")
        class FakeMujocoImu(Module, layer=1):
            pass

        bp = imu(enabled=True, backend="mujoco")

        assert _entry_classes(bp) == [FakeMujocoImu]
        assert _entry_names(bp) == ["imu"]
        assert bp._entries[0].config == {}
    finally:
        restore(saved)


def test_imu_stack_rejects_unknown_backend_without_livox_fallback():
    from runtime.blueprints.stacks.imu import imu

    saved = snapshot()
    try:
        clear()

        @register("imu", "livox")
        class FakeImu(Module, layer=1):
            pass

        with pytest.raises(ValueError, match="Unsupported imu backend"):
            imu(enabled=True, backend="serial")
    finally:
        restore(saved)


def test_imu_stack_rejects_declared_unimplemented_replay_backend():
    from runtime.blueprints.stacks.imu import imu

    with pytest.raises(ValueError, match="declared but not implemented"):
        imu(enabled=True, backend="replay")


def test_gateway_stack_prefers_registered_interface_modules():
    from runtime.blueprints.stacks.gateway import gateway

    saved = snapshot()
    try:
        clear()

        @register("gateway", "fastapi")
        class FakeGateway(Module, layer=6):
            pass

        @register("mcp", "server")
        class FakeMcp(Module, layer=6):
            pass

        @register("teleop", "default")
        class FakeTeleop(Module, layer=6):
            pass

        @register("visualization", "rerun")
        class FakeRerun(Module, layer=6):
            pass

        bp = gateway(port=5051, mcp_port=8091, enable_teleop=True, enable_rerun=True)

        assert _entry_classes(bp) == [
            FakeGateway,
            FakeMcp,
            FakeTeleop,
            FakeRerun,
        ]
        assert _entry_names(bp) == [
            "GatewayModule",
            "MCPServerModule",
            "TeleopModule",
            "RerunBridgeModule",
        ]
        assert bp._entries[0].config == {
            "port": 5051,
            "manage_session_services": True,
        }
        assert bp._entries[1].config == {"port": 8091}
        assert bp._entries[2].config == {"port": 5051}
        assert bp._entries[3].config == {"web_port": 9090}
    finally:
        restore(saved)


def test_gateway_stack_defaults_rerun_to_core_module_without_ros2_fallback():
    from runtime.blueprints.stacks.gateway import gateway

    saved = snapshot()
    previous_rerun_bridge = sys.modules.pop("gateway.visualization.rerun_bridge", None)
    try:
        clear()

        @register("gateway", "fastapi")
        class FakeGateway(Module, layer=6):
            pass

        @register("mcp", "server")
        class FakeMcp(Module, layer=6):
            pass

        bp = gateway(enable_teleop=False, enable_rerun=True)

        rerun_entry = next(entry for entry in bp._entries if entry.name == "RerunBridgeModule")
        assert rerun_entry.module_cls.__module__ == "runtime.rerun_module"
        assert "gateway.visualization.rerun_bridge" not in sys.modules
    finally:
        if previous_rerun_bridge is not None:
            sys.modules["gateway.visualization.rerun_bridge"] = previous_rerun_bridge
        restore(saved)


def test_gateway_stack_ignores_registered_ros2_rerun_without_flag():
    from runtime.blueprints.stacks.gateway import gateway

    saved = snapshot()
    previous_rerun_bridge = sys.modules.pop("gateway.visualization.rerun_bridge", None)
    try:
        clear()
        importlib.import_module("gateway.visualization.rerun_bridge")

        @register("gateway", "fastapi")
        class FakeGateway(Module, layer=6):
            pass

        @register("mcp", "server")
        class FakeMcp(Module, layer=6):
            pass

        bp = gateway(enable_teleop=False, enable_rerun=True)

        rerun_entry = next(entry for entry in bp._entries if entry.name == "RerunBridgeModule")
        assert rerun_entry.module_cls.__module__ == "runtime.rerun_module"
    finally:
        if previous_rerun_bridge is not None:
            sys.modules["gateway.visualization.rerun_bridge"] = previous_rerun_bridge
        else:
            sys.modules.pop("gateway.visualization.rerun_bridge", None)
        restore(saved)


def test_gateway_stack_allows_explicit_ros2_rerun_bridge():
    from runtime.blueprints.stacks.gateway import gateway

    saved = snapshot()
    previous_rerun_bridge = sys.modules.pop("gateway.visualization.rerun_bridge", None)
    try:
        clear()

        @register("gateway", "fastapi")
        class FakeGateway(Module, layer=6):
            pass

        @register("mcp", "server")
        class FakeMcp(Module, layer=6):
            pass

        bp = gateway(
            enable_teleop=False,
            enable_rerun=True,
            enable_ros2_rerun_bridge=True,
        )

        rerun_entry = next(entry for entry in bp._entries if entry.name == "RerunBridgeModule")
        assert rerun_entry.module_cls.__module__ == "gateway.visualization.rerun_bridge"
    finally:
        if previous_rerun_bridge is not None:
            sys.modules["gateway.visualization.rerun_bridge"] = previous_rerun_bridge
        else:
            sys.modules.pop("gateway.visualization.rerun_bridge", None)
        restore(saved)


def test_navigation_stack_prefers_registered_modules_with_canonical_aliases():
    from runtime.blueprints.stacks.navigation import navigation

    saved = snapshot()
    try:
        clear()

        @register("navigation", "default")
        class FakeNavigation(Module, layer=5):
            pass

        @register("exploration", "wavefront_frontier")
        class FakeWavefront(Module, layer=5):
            pass

        @register("navigation", "traversable_frontier")
        class FakeTraversableFrontier(Module, layer=5):
            pass

        bp = navigation(
            enable_frontier=True,
            enable_traversable_frontier=True,
            enable_native=False,
        )

        assert _entry_classes(bp)[:3] == [
            FakeNavigation,
            FakeWavefront,
            FakeTraversableFrontier,
        ]
        assert _entry_names(bp)[:3] == [
            "nav.mission",
            "WavefrontFrontierExplorer",
            "TraversableFrontierModule",
        ]
        assert any(
            wire.out_module == "WavefrontFrontierExplorer" and wire.in_module == "nav.mission" for wire in bp._wires
        )
    finally:
        restore(saved)


def test_external_tare_stack_prefers_registered_modules_with_canonical_aliases():
    from runtime.blueprints.stacks.exploration import exploration

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
    from runtime.blueprints.stacks.exploration import exploration

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
    from runtime.blueprints.stacks.perception import perception

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
            encoder="mobileclip",
            manage_services=False,
            force_camera=True,
            camera_rotate=90,
        )

        assert _entry_classes(bp)[:2] == [FakeCamera, FakePerception]
        assert _entry_names(bp)[:2] == ["camera", "PerceptionModule"]
        assert bp._entries[0].config == {"rotate": 90}
        assert bp._entries[1].config["detector_type"] == "bpu"
        assert bp._entries[1].config["encoder_type"] == "mobileclip"
    finally:
        restore(saved)


def test_perception_stack_does_not_default_external_camera_to_ros2_bridge():
    from runtime.blueprints.stacks.perception import perception

    saved = snapshot()
    try:
        clear()

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        bp = perception(
            detector="bpu",
            encoder="mobileclip",
            manage_services=False,
            force_camera=True,
        )

        assert FakePerception in _entry_classes(bp)
        if "camera" in _entry_names(bp):
            camera_cls = bp._entries[_entry_names(bp).index("camera")].module_cls
            assert ".adapters.ros2." not in camera_cls.__module__
        assert "PerceptionModule" in _entry_names(bp)
    finally:
        restore(saved)


def test_perception_stack_ignores_removed_explicit_ros2_camera_bridge():
    from runtime.blueprints.stacks.perception import perception

    saved = snapshot()
    try:
        clear()

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        bp = perception(
            detector="bpu",
            encoder="mobileclip",
            manage_services=False,
            force_camera=True,
            enable_ros2_camera_bridge=True,
        )

        assert "camera" not in _entry_names(bp)
        assert "PerceptionModule" in _entry_names(bp)
    finally:
        restore(saved)


def test_perception_stack_ignores_registered_ros2_camera_bridge_without_flag():
    from runtime.blueprints.stacks.perception import perception

    saved = snapshot()
    try:
        clear()

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        bp = perception(
            detector="bpu",
            encoder="mobileclip",
            manage_services=False,
            force_camera=True,
        )

        if "camera" in _entry_names(bp):
            camera_cls = bp._entries[_entry_names(bp).index("camera")].module_cls
            assert ".adapters.ros2." not in camera_cls.__module__
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
    perception_stack = importlib.import_module("runtime.blueprints.stacks.perception")

    saved = snapshot()
    try:
        clear()

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        calls: list[str] = []

        def recording_camera_module():
            calls.append("camera")
            raise AssertionError("camera should not be resolved")

        monkeypatch.setattr(
            perception_stack,
            "camera_module",
            recording_camera_module,
        )
        monkeypatch.setattr(perception_stack, "optional_stack_module", lambda *a, **kw: None)
        monkeypatch.setattr(
            perception_stack,
            "optional_fallback_module",
            lambda *a, **kw: None,
        )

        bp = perception_stack.perception(
            detector="bpu",
            encoder="mobileclip",
            manage_services=False,
            **config,
        )

        assert calls == []
        assert _entry_classes(bp) == [FakePerception]
        assert _entry_names(bp) == ["PerceptionModule"]
    finally:
        restore(saved)


def test_perception_stack_resolves_camera_for_mujoco_role(monkeypatch):
    perception_stack = importlib.import_module("runtime.blueprints.stacks.perception")

    saved = snapshot()
    try:
        clear()

        @register("camera", "sim")
        class FakeCamera(Module, layer=1):
            pass

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        calls: list[dict] = []
        original_camera_module = perception_stack.camera_module

        def recording_camera_module(**kwargs):
            calls.append(dict(kwargs))
            return original_camera_module(**kwargs)

        monkeypatch.setattr(
            perception_stack,
            "camera_module",
            recording_camera_module,
        )
        monkeypatch.setattr(perception_stack, "optional_stack_module", lambda *a, **kw: None)
        monkeypatch.setattr(
            perception_stack,
            "optional_fallback_module",
            lambda *a, **kw: None,
        )

        bp = perception_stack.perception(
            detector="bpu",
            encoder="mobileclip",
            manage_services=False,
            _driver_cls_name="MujocoDriverModule",
        )

        assert calls == [{"enable_ros2": False, "backend": "sim"}]
        assert _entry_classes(bp) == [FakeCamera, FakePerception]
        assert _entry_names(bp) == ["camera", "PerceptionModule"]
    finally:
        restore(saved)


def test_perception_stack_resolves_camera_for_external_camera(monkeypatch):
    perception_stack = importlib.import_module("runtime.blueprints.stacks.perception")

    saved = snapshot()
    try:
        clear()

        @register("camera", "orbbec")
        class FakeCamera(Module, layer=1):
            pass

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        original_camera_module = perception_stack.camera_module
        calls: list[str] = []

        def recording_camera_module(**kwargs):
            calls.append("camera")
            return original_camera_module(**kwargs)

        monkeypatch.setattr(
            perception_stack,
            "camera_module",
            recording_camera_module,
        )
        monkeypatch.setattr(perception_stack, "optional_stack_module", lambda *a, **kw: None)
        monkeypatch.setattr(
            perception_stack,
            "optional_fallback_module",
            lambda *a, **kw: None,
        )

        bp = perception_stack.perception(
            detector="bpu",
            encoder="mobileclip",
            manage_services=False,
            _driver_cls_name="ROS2SimDriverModule",
            use_driver_camera=False,
        )

        assert calls == ["camera"]
        assert _entry_classes(bp) == [FakeCamera, FakePerception]
        assert _entry_names(bp) == ["camera", "PerceptionModule"]
    finally:
        restore(saved)


def test_wiring_context_accepts_canonical_camera_name():
    from runtime.blueprints.wires.context import camera_source
    from runtime.contracts import CAMERA_COMPAT_ALIAS

    assert camera_source({"camera"}, driver_module="Driver") == (
        "camera",
        "color_image",
    )
    assert camera_source({CAMERA_COMPAT_ALIAS}, driver_module="Driver") == (
        CAMERA_COMPAT_ALIAS,
        "color_image",
    )
    assert camera_source(set(), driver_module="Driver") == ("Driver", "camera_image")


def test_perception_stack_prefers_registered_optional_tool_modules():
    from runtime.blueprints.stacks.perception import perception

    saved = snapshot()
    try:
        clear()

        @register("camera", "orbbec")
        class FakeCamera(Module, layer=1):
            pass

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        @register("encoder", "pluggable")
        class FakeEncoder(Module, layer=3):
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
            encoder="mobileclip",
            manage_services=False,
            force_camera=True,
            enable_standalone_encoder=True,
            recon_save_dir="/tmp/lingtu-recon",
            recon_server_url="http://127.0.0.1:7890",
        )

        assert _entry_classes(bp) == [
            FakeCamera,
            FakePerception,
            FakeEncoder,
            FakeReconstruction,
            FakeDatasetRecorder,
            FakeKeyframeExporter,
        ]
        assert _entry_names(bp) == [
            "camera",
            "PerceptionModule",
            "EncoderModule",
            "ReconstructionModule",
            "DatasetRecorderModule",
            "ReconKeyframeExporterModule",
        ]
        assert bp._entries[2].config == {"encoder": "mobileclip"}
        assert bp._entries[4].config["save_dir"] == "/tmp/lingtu-recon"
        assert bp._entries[5].config["server_url"] == "http://127.0.0.1:7890"
    finally:
        restore(saved)


def test_maps_stack_does_not_default_map_output_to_ros2():
    from maps.adapters.resolver import map_output_adapter_module
    from runtime.blueprints.stacks.maps import maps

    saved = snapshot()
    try:
        clear()

        @register("map", "occupancy_grid")
        class FakeOccupancyGrid(Module, layer=2):
            exploration_grid: Out[dict]

        @register("map", "voxel")
        class FakeVoxelGrid(Module, layer=2):
            pass

        @register("map", "semantic")
        class FakeSemanticMap(Module, layer=2):
            pass

        @register("map", "esdf")
        class FakeEsdf(Module, layer=2):
            pass

        @register("map", "elevation")
        class FakeElevationMap(Module, layer=2):
            pass

        @register("map", "traversability_cost")
        class FakeTraversabilityCost(Module, layer=2):
            pass

        @register("map", "ros2_map_output")
        class FakeMapOut(Module, layer=2):
            exploration_grid: In[dict]

        @register("map", "service")
        class FakeMapsModule(Module, layer=2):
            pass

        assert map_output_adapter_module() is None

        bp = maps(
            grid_resolution=0.25,
            grid_radius=8.0,
            voxel_backend="cpp",
            map_dir="/tmp/lingtu-test-maps",
            octomap_resolution=0.1,
            octomap_free_layers_above=6,
            octomap_free_dilation_cells=1,
            enable_map_out=True,
        )

        assert _entry_classes(bp) == [
            FakeOccupancyGrid,
            FakeVoxelGrid,
            FakeSemanticMap,
            FakeEsdf,
            FakeElevationMap,
            FakeTraversabilityCost,
            FakeMapsModule,
        ]
        assert _entry_names(bp) == [
            "OccupancyGridModule",
            "VoxelGridModule",
            "SemanticMapModule",
            "ESDFModule",
            "ElevationMapModule",
            "TraversabilityCostModule",
            "maps.service",
        ]
        assert bp._entries[0].config["resolution"] == 0.25
        assert bp._entries[1].config["backend"] == "cpp"
        assert bp._entries[-1].config == {
            "map_dir": "/tmp/lingtu-test-maps",
            "octomap_resolution": 0.1,
            "octomap_free_layers_above": 6,
            "octomap_free_dilation_cells": 1,
        }
        assert {f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}" for wire in bp._wires} == set()
    finally:
        restore(saved)


def test_maps_stack_prefers_registered_explicit_dds_map_output_module():
    from runtime.blueprints.stacks.maps import maps

    saved = snapshot()
    try:
        clear()

        @register("map", "occupancy_grid")
        class FakeOccupancyGrid(Module, layer=2):
            exploration_grid: Out[dict]

        @register("map", "voxel")
        class FakeVoxelGrid(Module, layer=2):
            pass

        @register("map", "semantic")
        class FakeSemanticMap(Module, layer=2):
            pass

        @register("map", "esdf")
        class FakeEsdf(Module, layer=2):
            pass

        @register("map", "elevation")
        class FakeElevationMap(Module, layer=2):
            pass

        @register("map", "traversability_cost")
        class FakeTraversabilityCost(Module, layer=2):
            pass

        @register("map", "dds_map_output")
        class FakeMapOut(Module, layer=2):
            exploration_grid: In[dict]

        @register("map", "service")
        class FakeMapManager(Module, layer=2):
            pass

        bp = maps(
            grid_resolution=0.25,
            grid_radius=8.0,
            map_dir="/tmp/lingtu-test-maps",
            enable_map_out=True,
            map_out_adapter="dds_map_output",
        )

        assert _entry_classes(bp) == [
            FakeOccupancyGrid,
            FakeMapOut,
            FakeVoxelGrid,
            FakeSemanticMap,
            FakeEsdf,
            FakeElevationMap,
            FakeTraversabilityCost,
            FakeMapManager,
        ]
        assert _entry_names(bp)[1] == "map.out"
        assert {f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}" for wire in bp._wires} == {
            "OccupancyGridModule.exploration_grid->map.out.exploration_grid",
        }
        assert {wire.topic for wire in bp._wires} == {"/nav/exploration_grid"}
    finally:
        restore(saved)


def test_planner_stack_prefers_registered_modules_with_canonical_aliases():
    from runtime.blueprints.stacks.planner import planner

    saved = snapshot()
    try:
        clear()

        @register("semantic_planner", "default")
        class FakeSemanticPlanner(Module, layer=4):
            pass

        @register("llm", "pluggable")
        class FakeLlm(Module, layer=4):
            pass

        @register("agent_planner", "default")
        class FakeAgentPlanner(Module, layer=4):
            pass

        @register("visual_servo", "default")
        class FakeVisualServo(Module, layer=4):
            pass

        bp = planner(llm="mock", save_dir="/tmp/lingtu-semantic")

        assert _entry_classes(bp) == [
            FakeSemanticPlanner,
            FakeLlm,
            FakeAgentPlanner,
            FakeVisualServo,
        ]
        assert _entry_names(bp) == [
            "SemanticPlannerModule",
            "LLMModule",
            "AgentPlannerModule",
            "VisualServoModule",
        ]
        assert bp._entries[0].config == {
            "save_dir": "/tmp/lingtu-semantic",
            "llm_backend": "mock",
        }
        assert bp._entries[1].config == {"backend": "mock"}
        assert bp._entries[2].config == {"llm_backend": "mock"}
    finally:
        restore(saved)


def test_memory_stack_prefers_registered_modules_with_canonical_aliases():
    from runtime.blueprints.stacks.memory import memory

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


def test_navigation_stack_prefers_registered_autonomy_modules_with_canonical_aliases():
    from runtime.blueprints.stacks.navigation import navigation

    saved = snapshot()
    try:
        clear()

        @register("navigation", "default")
        class FakeNavigation(Module, layer=5):
            pass

        @register("terrain", "nanobind")
        class FakeTerrain(Module, layer=2):
            pass

        @register("local_planner", "cmu_py")
        class FakeLocalPlanner(Module, layer=2):
            pass

        @register("path_follower", "nav_kernel")
        class FakePathFollower(Module, layer=2):
            pass

        bp = navigation(
            planner_backend="pct",
            enable_native=True,
            local_planner_backend="cmu_py",
            path_follower_backend="nav_kernel",
            path_follower_max_speed=0.4,
        )

        assert _entry_classes(bp)[:4] == [
            FakeNavigation,
            FakeTerrain,
            FakeLocalPlanner,
            FakePathFollower,
        ]
        assert _entry_names(bp)[:4] == [
            "nav.mission",
            "nav.terrain",
            "nav.local_planner",
            "nav.path_follower",
        ]
        assert bp._entries[1].config == {"backend": "nanobind"}
        assert bp._entries[2].config == {"backend": "cmu_py"}
        assert bp._entries[3].config == {"backend": "nav_kernel", "max_speed": 0.4}
    finally:
        restore(saved)


def test_lidar_stack_keeps_legacy_driver_start_opt_in():
    from runtime.blueprints.stacks.lidar import lidar

    saved = snapshot()
    try:
        clear()

        @register("lidar", "mid360")
        class FakeLidar(Module, layer=1):
            pass

        default_bp = lidar(enabled=True, backend="mid360", ip="192.0.2.30")
        legacy_bp = lidar(
            enabled=True,
            backend="mid360",
            ip="192.0.2.30",
            start_driver=True,
        )

        assert _entry_classes(default_bp) == [FakeLidar]
        assert _entry_names(default_bp) == ["lidar"]
        assert default_bp._entries[0].config == {"ip": "192.0.2.30"}
        assert legacy_bp._entries[0].config == {
            "ip": "192.0.2.30",
            "start_driver": True,
        }
    finally:
        restore(saved)


def test_slam_stack_rejects_removed_ros2_adapter_even_if_registered(monkeypatch):
    from runtime.blueprints.stacks.slam import slam

    monkeypatch.setenv("LINGTU_ENABLE_ROS2_COMPAT", "1")
    saved = snapshot()
    try:
        clear()

        @register("localization_adapter", "ros2_slam_bridge")
        class FakeLocalizationAdapter(Module, layer=1):
            pass

        bp = slam(
            "bridge",
            enable_visual_backup=True,
            manage_services=False,
            localization_adapter="ros2_slam_bridge",
        )

        assert _entry_classes(bp) == []
        assert _entry_names(bp) == []
    finally:
        restore(saved)


def test_slam_stack_rejects_removed_legacy_bridge_registry(monkeypatch):
    from runtime.adapters.localization import localization_adapter_module

    monkeypatch.setenv("LINGTU_ENABLE_ROS2_COMPAT", "1")
    saved = snapshot()
    try:
        clear()

        @register("slam_bridge", "default")
        class FakeSlamBridge(Module, layer=1):
            pass

        with pytest.raises(ImportError, match="were removed"):
            localization_adapter_module("ros2_slam_bridge")
    finally:
        restore(saved)


def test_slam_stack_visual_backup_does_not_import_cv2_at_build_time():
    from runtime.blueprints.stacks.slam import slam

    saved = snapshot()
    cv2_before = sys.modules.get("cv2")
    had_cv2 = "cv2" in sys.modules
    try:
        clear()
        sys.modules.pop("cv2", None)

        @register("localization_adapter", "test_lcm_adapter")
        class FakeLocalizationAdapter(Module, layer=1):
            pass

        bp = slam(
            "bridge",
            enable_visual_backup=True,
            manage_services=False,
            localization_adapter="test_lcm_adapter",
        )

        assert "cv2" not in sys.modules
        assert _entry_names(bp) == ["SlamAdapterModule", "DepthVisualOdomModule"]
    finally:
        if had_cv2:
            sys.modules["cv2"] = cv2_before
        else:
            sys.modules.pop("cv2", None)
        restore(saved)


def test_slam_stack_without_explicit_adapter_does_not_import_ros2_bridge():
    from runtime.blueprints.stacks.slam import slam

    saved = snapshot()
    ros2_bridge_before = sys.modules.get("localization.adapters.ros2.slam_bridge")
    had_ros2_bridge = "localization.adapters.ros2.slam_bridge" in sys.modules
    try:
        clear()
        sys.modules.pop("localization.adapters.ros2.slam_bridge", None)

        bp = slam("bridge", enable_visual_backup=True, manage_services=False)

        assert _entry_names(bp) == []
        assert "localization.adapters.ros2.slam_bridge" not in sys.modules
    finally:
        if had_ros2_bridge:
            sys.modules["localization.adapters.ros2.slam_bridge"] = ros2_bridge_before
        else:
            sys.modules.pop("localization.adapters.ros2.slam_bridge", None)
        restore(saved)


def test_sim_lidar_stack_prefers_registered_pointcloud_provider():
    from runtime.blueprints.stacks.sim_lidar import sim_lidar

    saved = snapshot()
    try:
        clear()

        @register("sim_lidar", "pointcloud")
        class FakeSimPointCloudProvider(Module, layer=1):
            pass

        bp = sim_lidar(
            scene_xml="sim/worlds/test_scene.xml",
            sample_spacing=0.25,
        )

        assert _entry_classes(bp) == [FakeSimPointCloudProvider]
        assert _entry_names(bp) == ["SimPointCloudProvider"]
        assert bp._entries[0].config == {
            "scene_xml": "sim/worlds/test_scene.xml",
            "sample_spacing": 0.25,
        }
    finally:
        restore(saved)
