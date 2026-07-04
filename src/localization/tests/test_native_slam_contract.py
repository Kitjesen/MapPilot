from __future__ import annotations

import sys
import types
from pathlib import Path

from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu, PointCloud2


def test_slam_module_exposes_pose_map_health_only() -> None:
    from localization.slam.module import SlamModule

    module = SlamModule()

    required_outputs = {
        "odometry",
        "registered_cloud",
        "map_cloud",
        "saved_map",
        "localization_status",
        "localization_quality",
        "alive",
        "map_odom_tf",
        "map_frame_jump_event",
        "gnss_fusion_health",
        "scene_mode",
        "lidar_scan",
        "imu",
        "state_estimation_at_scan",
    }
    forbidden_outputs = {
        "global_path",
        "local_path",
        "nav_way_point",
        "cmd_vel",
        "path",
    }

    assert required_outputs.issubset(module.ports_out)
    assert forbidden_outputs.isdisjoint(module.ports_out)
    assert {"visual_odom", "gnss_odom", "lidar_scan_in", "lidar_imu"}.issubset(
        module.ports_in
    )
    assert "lidar_raw_scan" in module.ports_in


def test_slam_module_consumes_lidar_inputs() -> None:
    from localization.slam.module import SlamModule

    module = SlamModule()
    module.setup()
    module.start()
    try:
        module.lidar_scan_in._deliver(
            PointCloud2.from_numpy(
                np.array([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32),
                frame_id="lidar_link",
            )
        )
        module.lidar_imu._deliver(Imu())
        module._drain_inputs()
        outputs = module._runner.outputs()
    finally:
        module.stop()

    assert outputs["lidar_buffer"] == 1
    assert outputs["imu_buffer"] == 1


def test_slam_imu_contract_preserves_covariance() -> None:
    from localization.slam.module import _imu_to_dict

    sample = _imu_to_dict(
        Imu(
            orientation_covariance=[0.1] * 9,
            angular_velocity_covariance=[0.2] * 9,
            linear_acceleration_covariance=[0.3] * 9,
            ts=1.0,
            frame_id="imu_link",
        )
    )

    assert sample["orientation_covariance"] == [0.1] * 9
    assert sample["angular_velocity_covariance"] == [0.2] * 9
    assert sample["linear_acceleration_covariance"] == [0.3] * 9


def test_slam_module_consumes_raw_livox_frame() -> None:
    from drivers.real.lidar.frames import POINT_DTYPE, LivoxPointFrame
    from localization.slam.module import SlamModule

    points = np.zeros(1, dtype=POINT_DTYPE)
    points["x"] = [1.0]
    points["y"] = [2.0]
    points["z"] = [3.0]
    points["intensity"] = [4.0]
    points["offset_time_ns"] = [123]
    points["line"] = [2]
    points["tag"] = [7]

    module = SlamModule()
    module.setup()
    module.start()
    try:
        module.lidar_raw_scan._deliver(
            LivoxPointFrame(points=points, timestamp_ns=1_000_000_000)
        )
        module._drain_inputs()
        outputs = module._runner.outputs()
    finally:
        module.stop()

    assert outputs["lidar_buffer"] == 1


def test_pointlio_profile_accepts_raw_lidar_but_reports_pending_algorithm() -> None:
    from drivers.real.lidar.frames import POINT_DTYPE, LivoxPointFrame
    from localization.slam.module import SlamModule

    points = np.zeros(1, dtype=POINT_DTYPE)
    points["x"] = [1.0]
    points["y"] = [2.0]
    points["z"] = [3.0]
    points["intensity"] = [4.0]
    points["offset_time_ns"] = [123]
    points["line"] = [2]
    points["tag"] = [7]

    module = SlamModule(backend_profile="pointlio")
    module.setup()
    module.start()
    try:
        module.lidar_raw_scan._deliver(
            LivoxPointFrame(points=points, timestamp_ns=1_000_000_000)
        )
        module._drain_inputs()
        module._runner.tick()
        outputs = module._runner.outputs()
    finally:
        module.stop()

    assert outputs["lidar_buffer"] == 1
    assert outputs["state"] == "DEGRADED"
    assert outputs["reason"] == "pointlio_algorithm_pending_ros_node_extraction"


def test_slam_module_save_map_writes_contract_artifacts(tmp_path) -> None:
    from localization.slam.module import SlamModule

    module = SlamModule()
    module.setup()
    module.start()
    try:
        module.lidar_scan_in._deliver(
            PointCloud2.from_numpy(
                np.array([[1.0, 0.0, 0.5, 10.0]], dtype=np.float32),
                frame_id="body",
            )
        )
        module.visual_odom._deliver(Odometry())
        module._drain_inputs()
        result = module.save_map(str(tmp_path))
    finally:
        module.stop()

    assert result["ok"] is True
    assert (tmp_path / "map.pcd").exists()
    assert (tmp_path / "poses.txt").exists()
    assert (tmp_path / "trajectory.txt").exists()
    assert (tmp_path / "patches").is_dir()
    assert (tmp_path / "patches" / "latest_scan.pcd").exists()


def test_slam_stack_defaults_to_native_slam_module() -> None:
    from runtime.blueprints.stacks.slam import slam, slam_module_name

    bp = slam("fastlio2", enable_visual_backup=False)

    assert [entry.name for entry in bp._entries] == ["SlamModule"]
    assert slam_module_name("fastlio2") == "SlamModule"


def test_slam_stack_keeps_explicit_ros2_bridge_compatibility() -> None:
    from runtime.blueprints.stacks.slam import slam

    bp = slam(
        "fastlio2",
        enable_visual_backup=False,
        localization_adapter="ros2_slam_bridge",
    )

    assert [entry.name for entry in bp._entries] == ["SlamBridgeModule"]


def test_native_slam_wiring_covers_old_bridge_consumers() -> None:
    from runtime.blueprints.full_stack_wiring import full_stack_wire_specs
    from runtime.runtime_interface import TOPICS

    modules = {
        "SlamModule",
        "DepthVisualOdomModule",
        "GatewayModule",
        "nav.safety",
        "nav.mission",
        "nav.local_planner",
        "nav.path_follower",
        "nav.terrain",
        "nav.maps",
        "OccupancyGridModule",
        "ElevationMapModule",
        "VoxelGridModule",
        "ThunderDriver",
        "LidarModule",
    }
    specs = full_stack_wire_specs(
        modules,
        robot="thunder",
        driver_module="ThunderDriver",
        slam_profile="fastlio2",
        enable_semantic=False,
    )
    wires = {
        f"{spec.out_module}.{spec.out_port}->{spec.in_module}.{spec.in_port}"
        for spec in specs
    }
    spec_by_wire = {
        f"{spec.out_module}.{spec.out_port}->{spec.in_module}.{spec.in_port}": spec
        for spec in specs
    }

    assert "SlamModule.saved_map->GatewayModule.saved_map" in wires
    assert "SlamModule.map_cloud_frame->nav.maps.map_cloud_frame" in wires
    assert "SlamModule.map_cloud_frame->nav.terrain.map_cloud_frame" in wires
    assert "SlamModule.map_cloud_frame->OccupancyGridModule.map_cloud_frame" in wires
    assert "SlamModule.map_cloud_frame->ElevationMapModule.map_cloud_frame" in wires
    assert "SlamModule.map_cloud_frame->VoxelGridModule.map_cloud_frame" in wires
    assert "SlamModule.map_cloud->nav.terrain.map_cloud" not in wires
    assert "SlamModule.map_cloud->OccupancyGridModule.map_cloud" not in wires
    assert "SlamModule.localization_status->GatewayModule.localization_status" in wires
    assert "SlamModule.localization_status->nav.safety.localization_status" in wires
    assert "SlamModule.localization_status->nav.mission.localization_status" in wires
    assert "SlamModule.localization_status->DepthVisualOdomModule.localization_status" in wires
    assert "SlamModule.gnss_fusion_health->nav.safety.gnss_fusion_health" in wires
    assert "SlamModule.map_frame_jump_event->nav.mission.map_frame_jump_event" in wires
    assert "DepthVisualOdomModule.visual_odometry->SlamModule.visual_odom" in wires
    assert "LidarModule.raw_scan->SlamModule.lidar_raw_scan" in wires
    assert "LidarModule.imu->SlamModule.lidar_imu" in wires
    assert spec_by_wire["LidarModule.raw_scan->SlamModule.lidar_raw_scan"].transport == "dds"
    assert spec_by_wire["LidarModule.raw_scan->SlamModule.lidar_raw_scan"].topic == TOPICS.raw_lidar_points
    assert spec_by_wire["LidarModule.imu->SlamModule.lidar_imu"].transport == "dds"
    assert spec_by_wire["LidarModule.imu->SlamModule.lidar_imu"].topic == TOPICS.raw_imu


def test_mujoco_slam_wiring_uses_raw_lidar_imu_contract() -> None:
    from runtime.blueprints.full_stack_wiring import full_stack_wire_specs
    from runtime.runtime_interface import TOPICS

    specs = full_stack_wire_specs(
        {
            "MujocoDriverModule",
            "SlamModule",
        },
        robot="sim_mujoco",
        driver_module="MujocoDriverModule",
        slam_profile="fastlio2",
        enable_semantic=False,
    )
    spec_by_wire = {
        f"{spec.out_module}.{spec.out_port}->{spec.in_module}.{spec.in_port}": spec
        for spec in specs
    }

    raw = spec_by_wire["MujocoDriverModule.raw_scan->SlamModule.lidar_raw_scan"]
    imu = spec_by_wire["MujocoDriverModule.imu->SlamModule.lidar_imu"]

    assert raw.transport == "dds"
    assert raw.topic == TOPICS.raw_lidar_points
    assert imu.transport == "dds"
    assert imu.topic == TOPICS.raw_imu


def test_slam_cpp_build_declares_python_native_binding() -> None:
    cmake = Path("src/localization/slam/cpp/CMakeLists.txt").read_text(encoding="utf-8")
    module = Path("src/localization/slam/module.py").read_text(encoding="utf-8")
    binding = Path("src/localization/slam/cpp/bind.cpp").read_text(encoding="utf-8")
    ros2_dds_runtime = Path(
        "src/localization/adapters/ros2/cpp/ros2_dds_runtime.cpp"
    ).read_text(encoding="utf-8")
    cyclone_runtime = Path("src/localization/slam/cpp/cyclone_runtime.cpp").read_text(encoding="utf-8")
    fastlio = Path("src/localization/slam/cpp/fastlio.cpp").read_text(encoding="utf-8")
    sdk2_stream = Path("src/drivers/real/lidar/sdk2_stream/main.cpp").read_text(encoding="utf-8")
    build_script = Path("scripts/build/build_slam_core.sh").read_text(encoding="utf-8")

    assert "LINGTU_SLAM_BUILD_PYTHON_BINDINGS" in cmake
    assert "nanobind_add_module(_native bind.cpp)" in cmake
    assert "LINGTU_SLAM_BUILD_DDS_RUNTIME" in cmake
    assert "add_executable(lingtu_slam_dds_runtime cyclone_runtime.cpp)" in cmake
    assert "OUTPUT_NAME lingtu_slam_cyclone_runtime" in cmake
    assert "LINGTU_SLAM_BUILD_ROS2_DDS_RUNTIME" in cmake
    assert "../../adapters/ros2/cpp/ros2_dds_runtime.cpp" in cmake
    assert "find_package(iceoryx_binding_c QUIET)" in cmake
    assert "find_program(CYCLONEDDS_IDLC_EXECUTABLE NAMES idlc REQUIRED)" in cmake
    assert "CycloneDDS::ddsc" in cmake
    assert "CycloneDDS-CXX" not in cmake
    assert "add_executable(lingtu_slam_cyclone_runtime ALIAS lingtu_slam_dds_runtime)" in cmake
    assert "LINGTU_SLAM_BUILD_DDS_RUNTIME" in build_script
    assert "LINGTU_SLAM_BUILD_ROS2_DDS_RUNTIME" in build_script
    assert "CPU_BBS3D_ROOT" in build_script
    assert "LINGTU_REQUIRE_BBS3D" in build_script
    assert "localization.slam._native" in module
    assert "NATIVE_SLAM_BINDING_SCHEMA" in module
    assert "NB_MODULE(_native, m)" in binding
    assert "SlamRunner" in binding
    assert "#include <livox_ros_driver2/msg/custom_msg.hpp>" in ros2_dds_runtime
    assert "create_subscription<livox_ros_driver2::msg::CustomMsg>" in ros2_dds_runtime
    assert "backend_->feedLidar" in ros2_dds_runtime
    assert "backend_->feedImu" in ros2_dds_runtime
    assert "#include \"dds/dds.h\"" in cyclone_runtime
    assert "rclcpp" not in cyclone_runtime
    assert "sensor_msgs" not in cyclone_runtime
    assert "livox_ros_driver2" not in cyclone_runtime
    assert "dds_create_reader" in cyclone_runtime
    assert "lingtu_dds_LivoxFrame_desc" in cyclone_runtime
    assert "lingtu_dds_Imu_desc" in cyclone_runtime
    assert "lingtu_dds_TFMessage_desc" in cyclone_runtime
    assert "lingtu_dds_RelocalizationRequest_desc" in cyclone_runtime
    assert "lingtu_dds_RelocalizationResponse_desc" in cyclone_runtime
    assert "drainRelocalizationRequests" in cyclone_runtime
    assert "dds.writeRelocalizationResponse" in cyclone_runtime
    assert "track_against_map_started" in cyclone_runtime
    assert "track_against_map_enabled" in cyclone_runtime
    assert "track_against_map_failures" in cyclone_runtime
    assert 'action == "track-against-map"' in cyclone_runtime
    assert "--track-against-map-period-s" in cyclone_runtime
    assert "track_against_map_period_s" in cyclone_runtime
    assert "kTrackAgainstMapMaxFailures" in cyclone_runtime
    assert "registered_cloud_stale" in cyclone_runtime
    assert "last_track_against_map_scan_s" in cyclone_runtime
    assert "lingtu::message::kTf.dds_topic.data()" in cyclone_runtime
    assert "lingtu::message::kTfStatic.dds_topic.data()" in cyclone_runtime
    assert "dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL)" in cyclone_runtime
    assert "dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT" in cyclone_runtime
    assert "ReaderQos::SensorStream" in cyclone_runtime
    assert "dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 256)" in cyclone_runtime
    assert "dds_qset_history(qos.get(), DDS_HISTORY_KEEP_LAST, 256)" in sdk2_stream
    assert "resetTrackingCoreAtPose(*odometry_odom_body_)" in fastlio
    assert "map_odom_pose_ = result.map_odom" in fastlio
    assert "composePoses(map_odom_pose_, *odometry_odom_body_)" in fastlio
    assert "effective_guess" in fastlio
    assert "relocalization_max_fitness" in fastlio
    assert "relocalization_fitness_rejected" in fastlio
    assert "relocalization_map_bounds_margin_m" in fastlio
    assert "relocalization_outside_map_bounds" in fastlio
    assert "updateMapBounds" in fastlio
    assert "poseInsideMapBounds" in fastlio
    assert "relocalization_refine_backend" in cyclone_runtime
    assert "relocalization_map_body" in cyclone_runtime
    assert "dds.writeTf(msg.msg)" in cyclone_runtime
    assert "backend->feedLidar" in cyclone_runtime
    assert "backend->feedImu" in cyclone_runtime
    assert "--log-status-s" in cyclone_runtime


def test_slam_relocalization_has_typed_dds_request_reply_contract() -> None:
    idl = Path("src/message/idl/lingtu_slam.idl").read_text(encoding="utf-8")
    topics = Path("src/message/cpp/dds_topics.hpp").read_text(encoding="utf-8")
    runtime_topics = Path("src/runtime/runtime_interface.py").read_text(encoding="utf-8")
    cyclone_runtime = Path("src/localization/slam/cpp/cyclone_runtime.cpp").read_text(
        encoding="utf-8"
    )
    slam_control = Path("src/localization/slam/cpp/slam_control.cpp").read_text(
        encoding="utf-8"
    )

    assert "struct RelocalizationRequest" in idl
    assert "struct RelocalizationResponse" in idl
    assert "string action" in idl
    assert "string engine" in idl
    assert "boolean has_initial_pose" in idl
    assert "boolean track_against_map_supported" in idl
    assert "boolean track_against_map_enabled" in idl
    assert "long track_against_map_failures" in idl

    assert "kSlamRelocalizationRequest" in topics
    assert "kSlamRelocalizationResponse" in topics
    assert "/slam/relocalization/request" in topics
    assert "/slam/relocalization/response" in topics
    assert "slam_relocalization_request" in runtime_topics
    assert "slam_relocalization_response" in runtime_topics

    assert "reader<lingtu_dds_RelocalizationRequest>" in cyclone_runtime
    assert "writer<lingtu_dds_RelocalizationResponse>" in cyclone_runtime
    assert "normalizedRelocalizationAction" in cyclone_runtime
    assert "seeded_relocalize" in cyclone_runtime
    assert "global_relocalize" in cyclone_runtime
    assert "query_status" in cyclone_runtime
    assert "track_against_map_started" in cyclone_runtime
    assert "track_against_map_enabled" in cyclone_runtime
    assert "track_against_map_failures" in cyclone_runtime
    assert "kTrackAgainstMapMaxFailures" in cyclone_runtime
    assert "--track-against-map-period-s" in cyclone_runtime
    assert "registered_cloud_stale" in cyclone_runtime
    assert "backend->relocalize(track_against_map_seed)" in cyclone_runtime

    assert "usesTypedRelocalizationService" in slam_control
    assert "runTypedRelocalizationService" in slam_control
    assert "kSlamRelocalizationRequest.dds_topic.data()" in slam_control
    assert "kSlamRelocalizationResponse.dds_topic.data()" in slam_control
    assert "lingtu.slam.relocalization_response.v1" in slam_control
    assert "track_against_map_enabled" in slam_control
    assert "track_against_map_failures" in slam_control
    assert "cpp_typed_dds" in slam_control
    assert "kSlamMapCommand.dds_topic.data()" in slam_control


def test_cpp_message_topic_contract_stays_ros_free() -> None:
    header = Path("src/message/cpp/dds_topics.hpp").read_text(encoding="utf-8")

    forbidden = (
        "rclcpp",
        "sensor_msgs",
        "nav_msgs",
        "geometry_msgs",
        "std_msgs",
        "livox_ros_driver2",
    )
    for marker in forbidden:
        assert marker not in header


def test_slam_loader_prefers_schema_checked_native_binding(monkeypatch) -> None:
    from localization.slam import module as slam_module

    class FakeRunner:
        def __init__(self, backend: str, mode: str, map_path: str) -> None:
            self.args = (backend, mode, map_path)

    fake_native = types.ModuleType("localization.slam._native")
    fake_native.NATIVE_SLAM_BINDING_SCHEMA = slam_module.NATIVE_SLAM_BINDING_SCHEMA
    fake_native.SlamRunner = FakeRunner
    monkeypatch.setitem(sys.modules, "localization.slam._native", fake_native)

    runner = slam_module._load_runner("fastlio2", "mapping", "")

    assert isinstance(runner, FakeRunner)
    assert runner.args == ("fastlio2", "mapping", "")


def test_slam_loader_ignores_unversioned_native_binding(monkeypatch) -> None:
    from localization.slam import module as slam_module

    stale_native = types.ModuleType("localization.slam._native")
    stale_native.SlamRunner = object
    monkeypatch.setitem(sys.modules, "localization.slam._native", stale_native)

    runner = slam_module._load_runner("fastlio2", "mapping", "")

    assert isinstance(runner, slam_module._PythonSlamRunner)


def test_fastlio_feed_lidar_only_queues_raw_frame() -> None:
    text = Path("src/localization/slam/cpp/fastlio.cpp").read_text(encoding="utf-8")
    feed = text.split("Status feedLidar(const LidarFrame& frame) override {", 1)[1]
    feed = feed.split("Status feedGnss", 1)[0]
    sync = text.split("bool prepareFastLioPackage() {", 1)[1]
    sync = sync.split("void updateWaitingReason()", 1)[0]

    assert "toPclCloud" not in feed
    assert "livox_scan_window" not in text
    assert "pending_lidar_scan_" not in text
    assert "pushLidarFrame(frame)" in feed
    assert "toPclCloud(lidar_buffer_.front(), builder_config_)" in sync


def test_slam_sensor_callbacks_are_enqueue_only() -> None:
    from drivers.real.lidar.frames import POINT_DTYPE, LivoxPointFrame
    from localization.slam.module import SlamModule

    points = np.zeros(1, dtype=POINT_DTYPE)
    points["x"] = [1.0]
    points["y"] = [2.0]
    points["z"] = [3.0]
    points["intensity"] = [4.0]

    module = SlamModule()
    module.setup()
    result = module.feed_raw_lidar(
        LivoxPointFrame(points=points, timestamp_ns=1_000_000_000)
    )

    assert result["message"] == "lidar_queued"
    assert module._runner.outputs()["lidar_buffer"] == 0
    assert module.slam_status()["input_queue"]["lidar"] == 1

    module._drain_inputs()

    assert module._runner.outputs()["lidar_buffer"] == 1
    assert module.slam_status()["input_queue"]["lidar"] == 0


def test_slam_status_reports_lidar_imu_sync_window() -> None:
    from drivers.real.lidar.frames import POINT_DTYPE, LivoxPointFrame
    from localization.slam.module import SlamModule

    points = np.zeros(2, dtype=POINT_DTYPE)
    points["x"] = [1.0, 2.0]
    points["offset_time_ns"] = [0, 100_000_000]

    module = SlamModule()
    module.setup()
    try:
        module.feed_imu(Imu(ts=1.02))
        module.feed_imu(Imu(ts=1.06))
        module.feed_raw_lidar(
            LivoxPointFrame(points=points, timestamp_ns=1_000_000_000)
        )
        module._drain_inputs()
        sync = module.slam_status()["sync"]
    finally:
        module.stop()

    assert abs(sync["scan_start_s"] - 1.0) < 1e-9
    assert abs(sync["scan_end_s"] - 1.1) < 1e-9
    assert abs(sync["last_imu_s"] - 1.06) < 1e-9
    assert sync["imu_batch"] == 2
    assert sync["wait_count"] == 1
