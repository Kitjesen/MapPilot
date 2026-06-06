from __future__ import annotations

import importlib
import sys

import pytest

from core.module import Module
from core.registry import clear, register, restore, snapshot


def _entry_classes(bp) -> list[type]:
    return [entry.module_cls for entry in bp._entries]


def _entry_names(bp) -> list[str]:
    return [entry.name for entry in bp._entries]


def test_safety_stack_prefers_registered_modules():
    from core.blueprints.stacks.safety import safety

    saved = snapshot()
    try:
        clear()

        @register("safety", "ring")
        class FakeSafetyRing(Module, layer=0):
            pass

        @register("safety", "cmd_vel_mux")
        class FakeCmdVelMux(Module, layer=0):
            pass

        @register("safety", "geofence")
        class FakeGeofence(Module, layer=0):
            pass

        bp = safety()
        classes = _entry_classes(bp)

        assert classes == [FakeSafetyRing, FakeCmdVelMux, FakeGeofence]
        assert _entry_names(bp) == [
            "SafetyRingModule",
            "CmdVelMux",
            "GeofenceManagerModule",
        ]
    finally:
        restore(saved)


def test_lidar_stack_prefers_registered_mid360_module():
    from core.blueprints.stacks.lidar import lidar

    saved = snapshot()
    try:
        clear()

        @register("lidar", "mid360")
        class FakeLidar(Module, layer=1):
            pass

        bp = lidar(ip="192.0.2.10")

        assert _entry_classes(bp) == [FakeLidar]
        assert _entry_names(bp) == ["LidarModule"]
        assert bp._entries[0].config == {"ip": "192.0.2.10"}
    finally:
        restore(saved)


def test_gateway_stack_prefers_registered_interface_modules():
    from core.blueprints.stacks.gateway import gateway

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

        @register("webrtc", "aiortc")
        class FakeWebRtc(Module, layer=6):
            pass

        @register("visualization", "rerun")
        class FakeRerun(Module, layer=6):
            pass

        bp = gateway(port=5051, mcp_port=8091, enable_teleop=True, enable_rerun=True)

        assert _entry_classes(bp) == [
            FakeGateway,
            FakeMcp,
            FakeTeleop,
            FakeWebRtc,
            FakeRerun,
        ]
        assert _entry_names(bp) == [
            "GatewayModule",
            "MCPServerModule",
            "TeleopModule",
            "WebRTCStreamModule",
            "RerunBridgeModule",
        ]
        assert bp._entries[0].config == {"port": 5051}
        assert bp._entries[1].config == {"port": 8091}
        assert bp._entries[2].config == {"port": 5051}
        assert bp._entries[4].config == {"web_port": 9090}
    finally:
        restore(saved)


def test_navigation_stack_prefers_registered_modules_with_canonical_aliases():
    from core.blueprints.stacks.navigation import navigation

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
            "NavigationModule",
            "WavefrontFrontierExplorer",
            "TraversableFrontierModule",
        ]
        assert any(
            wire.out_module == "WavefrontFrontierExplorer"
            and wire.in_module == "NavigationModule"
            for wire in bp._wires
        )
    finally:
        restore(saved)


def test_external_tare_stack_prefers_registered_modules_with_canonical_aliases():
    from core.blueprints.stacks.exploration import exploration

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


def test_local_tare_stack_prefers_registered_modules_with_canonical_aliases(monkeypatch):
    from core.blueprints.stacks.exploration import exploration

    class FakeConfig:
        raw = {"exploration": {"tare_scenario": "garage"}}

    saved = snapshot()
    captured: dict[str, object] = {}
    try:
        clear()

        @register("exploration", "tare")
        class FakeTareExplorer(Module, layer=5):
            pass

        @register("exploration", "supervisor")
        class FakeExplorationSupervisor(Module, layer=5):
            pass

        class FakeNativeTare(Module, layer=5):
            pass

        def fake_tare_explorer(cfg=None, scenario="forest"):
            captured["cfg"] = cfg
            captured["scenario"] = scenario
            return FakeNativeTare()

        monkeypatch.setattr("core.config.get_config", lambda: FakeConfig())
        monkeypatch.setattr(
            "core.native_install.exe",
            lambda cfg, package, executable: "/tmp/tare_planner_node",
        )
        monkeypatch.setattr("os.path.exists", lambda path: True)
        monkeypatch.setattr(
            "exploration.native_factories.tare_explorer",
            fake_tare_explorer,
        )

        bp = exploration(
            backend="tare",
            tare_warn_timeout_s=1.5,
            tare_supervisor_hz=2.0,
        )

        assert _entry_classes(bp) == [
            FakeNativeTare,
            FakeTareExplorer,
            FakeExplorationSupervisor,
        ]
        assert _entry_names(bp) == [
            "TAREPlannerNativeModule",
            "TAREExplorerModule",
            "ExplorationSupervisorModule",
        ]
        assert captured["scenario"] == "garage"
        assert bp._entries[2].config == {"warn_timeout_s": 1.5, "poll_hz": 2.0}
    finally:
        restore(saved)


def test_perception_stack_prefers_registered_scene_and_camera_bridge_modules():
    from core.blueprints.stacks.perception import perception

    saved = snapshot()
    try:
        clear()

        @register("camera_bridge", "default")
        class FakeCameraBridge(Module, layer=1):
            pass

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        bp = perception(
            detector="bpu",
            encoder="mobileclip",
            manage_services=False,
            force_camera_bridge=True,
            camera_rotate=90,
        )

        assert _entry_classes(bp)[:2] == [FakeCameraBridge, FakePerception]
        assert _entry_names(bp)[:2] == ["CameraBridgeModule", "PerceptionModule"]
        assert bp._entries[0].config == {"rotate": 90}
        assert bp._entries[1].config["detector_type"] == "bpu"
        assert bp._entries[1].config["encoder_type"] == "mobileclip"
    finally:
        restore(saved)


@pytest.mark.parametrize(
    "config",
    [
        {"_driver_cls_name": "MujocoDriverModule"},
        {"_driver_cls_name": "ROS2SimDriverModule", "use_driver_camera": True},
    ],
)
def test_perception_stack_skips_camera_bridge_resolution_for_driver_camera(
    monkeypatch,
    config,
):
    perception_stack = importlib.import_module("core.blueprints.stacks.perception")

    saved = snapshot()
    try:
        clear()

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        original_stack_module = perception_stack.stack_module
        calls: list[tuple[str, str]] = []

        def recording_stack_module(group, name, *args, **kwargs):
            calls.append((group, name))
            return original_stack_module(group, name, *args, **kwargs)

        monkeypatch.setattr(perception_stack, "stack_module", recording_stack_module)
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

        assert ("camera_bridge", "default") not in calls
        assert _entry_classes(bp) == [FakePerception]
        assert _entry_names(bp) == ["PerceptionModule"]
    finally:
        restore(saved)


def test_perception_stack_resolves_camera_bridge_for_external_camera(monkeypatch):
    perception_stack = importlib.import_module("core.blueprints.stacks.perception")

    saved = snapshot()
    try:
        clear()

        @register("camera_bridge", "default")
        class FakeCameraBridge(Module, layer=1):
            pass

        @register("perception", "scene")
        class FakePerception(Module, layer=3):
            pass

        original_stack_module = perception_stack.stack_module
        calls: list[tuple[str, str]] = []

        def recording_stack_module(group, name, *args, **kwargs):
            calls.append((group, name))
            return original_stack_module(group, name, *args, **kwargs)

        monkeypatch.setattr(perception_stack, "stack_module", recording_stack_module)
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

        assert ("camera_bridge", "default") in calls
        assert _entry_classes(bp) == [FakeCameraBridge, FakePerception]
        assert _entry_names(bp) == ["CameraBridgeModule", "PerceptionModule"]
    finally:
        restore(saved)


def test_perception_stack_prefers_registered_optional_tool_modules():
    from core.blueprints.stacks.perception import perception

    saved = snapshot()
    try:
        clear()

        @register("camera_bridge", "default")
        class FakeCameraBridge(Module, layer=1):
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
            force_camera_bridge=True,
            enable_standalone_encoder=True,
            recon_save_dir="/tmp/lingtu-recon",
            recon_server_url="http://127.0.0.1:7890",
        )

        assert _entry_classes(bp) == [
            FakeCameraBridge,
            FakePerception,
            FakeEncoder,
            FakeReconstruction,
            FakeDatasetRecorder,
            FakeKeyframeExporter,
        ]
        assert _entry_names(bp) == [
            "CameraBridgeModule",
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


def test_maps_stack_prefers_registered_modules_with_canonical_aliases():
    from core.blueprints.stacks.maps import maps

    saved = snapshot()
    try:
        clear()

        @register("map", "occupancy_grid")
        class FakeOccupancyGrid(Module, layer=2):
            pass

        @register("map", "voxel")
        class FakeVoxelGrid(Module, layer=2):
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

        @register("map", "ros2_grid_bridge")
        class FakeRos2GridBridge(Module, layer=2):
            pass

        @register("map", "manager")
        class FakeMapManager(Module, layer=2):
            pass

        bp = maps(
            grid_resolution=0.25,
            grid_radius=8.0,
            map_dir="/tmp/lingtu-test-maps",
            enable_ros2_grid_bridge=True,
        )

        assert _entry_classes(bp) == [
            FakeOccupancyGrid,
            FakeRos2GridBridge,
            FakeVoxelGrid,
            FakeEsdf,
            FakeElevationMap,
            FakeTraversabilityCost,
            FakeMapManager,
        ]
        assert _entry_names(bp) == [
            "OccupancyGridModule",
            "ROS2GridBridgeModule",
            "VoxelGridModule",
            "ESDFModule",
            "ElevationMapModule",
            "TraversabilityCostModule",
            "MapManagerModule",
        ]
        assert bp._entries[0].config["resolution"] == 0.25
        assert bp._entries[-1].config == {"map_dir": "/tmp/lingtu-test-maps"}
    finally:
        restore(saved)


def test_navigation_stack_prefers_registered_ros2_path_bridge_module():
    from core.blueprints.stacks.navigation import navigation

    saved = snapshot()
    try:
        clear()

        @register("navigation", "default")
        class FakeNavigation(Module, layer=5):
            pass

        @register("navigation", "ros2_path_bridge")
        class FakeRos2PathBridge(Module, layer=5):
            pass

        bp = navigation(
            enable_ros2_path_bridge=True,
            enable_native=False,
            planning_frame_id="map",
        )

        assert _entry_classes(bp)[:2] == [FakeNavigation, FakeRos2PathBridge]
        assert _entry_names(bp)[:2] == ["NavigationModule", "ROS2PathBridgeModule"]
        assert bp._entries[1].config == {"default_frame_id": "map"}
    finally:
        restore(saved)


def test_planner_stack_prefers_registered_modules_with_canonical_aliases():
    from core.blueprints.stacks.planner import planner

    saved = snapshot()
    try:
        clear()

        @register("semantic_planner", "default")
        class FakeSemanticPlanner(Module, layer=4):
            pass

        @register("llm", "pluggable")
        class FakeLlm(Module, layer=4):
            pass

        @register("visual_servo", "default")
        class FakeVisualServo(Module, layer=4):
            pass

        bp = planner(llm="mock", save_dir="/tmp/lingtu-semantic")

        assert _entry_classes(bp) == [FakeSemanticPlanner, FakeLlm, FakeVisualServo]
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


def test_memory_stack_prefers_registered_modules_with_canonical_aliases():
    from core.blueprints.stacks.memory import memory

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
    from core.blueprints.stacks.navigation import navigation

    saved = snapshot()
    try:
        clear()

        @register("navigation", "default")
        class FakeNavigation(Module, layer=5):
            pass

        @register("terrain", "cmu")
        class FakeTerrain(Module, layer=2):
            pass

        @register("local_planner", "cmu")
        class FakeLocalPlanner(Module, layer=2):
            pass

        @register("path_follower", "nav_core")
        class FakePathFollower(Module, layer=2):
            pass

        bp = navigation(
            planner_backend="pct",
            enable_native=True,
            local_planner_backend="cmu",
            path_follower_backend="nav_core",
            path_follower_max_speed=0.4,
        )

        assert _entry_classes(bp)[:4] == [
            FakeNavigation,
            FakeTerrain,
            FakeLocalPlanner,
            FakePathFollower,
        ]
        assert _entry_names(bp)[:4] == [
            "NavigationModule",
            "TerrainModule",
            "LocalPlannerModule",
            "PathFollowerModule",
        ]
        assert bp._entries[1].config == {"backend": "cmu"}
        assert bp._entries[2].config == {"backend": "cmu"}
        assert bp._entries[3].config == {"backend": "nav_core", "max_speed": 0.4}
    finally:
        restore(saved)


def test_slam_stack_prefers_registered_bridge_and_visual_odom_modules():
    from core.blueprints.stacks.slam import slam

    saved = snapshot()
    try:
        clear()

        @register("slam_bridge", "default")
        class FakeSlamBridge(Module, layer=1):
            pass

        @register("visual_odom", "depth")
        class FakeDepthVisualOdom(Module, layer=1):
            pass

        bp = slam("bridge", enable_visual_backup=True, manage_services=False)

        assert _entry_classes(bp) == [FakeSlamBridge, FakeDepthVisualOdom]
        assert _entry_names(bp) == ["SlamBridgeModule", "DepthVisualOdomModule"]
        assert bp._entries[0].config["backend_profile"] == "bridge"
    finally:
        restore(saved)


def test_slam_stack_visual_backup_does_not_import_cv2_at_build_time():
    from core.blueprints.stacks.slam import slam

    saved = snapshot()
    cv2_before = sys.modules.get("cv2")
    had_cv2 = "cv2" in sys.modules
    try:
        clear()
        sys.modules.pop("cv2", None)

        bp = slam("bridge", enable_visual_backup=True, manage_services=False)

        assert "cv2" not in sys.modules
        assert _entry_names(bp) == ["SlamBridgeModule", "DepthVisualOdomModule"]
    finally:
        if had_cv2:
            sys.modules["cv2"] = cv2_before
        else:
            sys.modules.pop("cv2", None)
        restore(saved)


def test_sim_lidar_stack_prefers_registered_pointcloud_provider():
    from core.blueprints.stacks.sim_lidar import sim_lidar

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
