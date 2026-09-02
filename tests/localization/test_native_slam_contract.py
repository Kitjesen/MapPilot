from __future__ import annotations

import importlib
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

from runtime.msgs.map import MapCloudFrame
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2

ROOT = Path(__file__).resolve().parents[2]
LOCALIZATION_ROOT = ROOT / "src" / "localization"


def _configure_native_slam(
    *cmake_definitions: str,
) -> subprocess.CompletedProcess[str]:
    source_dir = LOCALIZATION_ROOT / "slam" / "cpp"
    cmake_executable = shutil.which("cmake")
    assert cmake_executable is not None, "cmake is required for native SLAM tests"
    with tempfile.TemporaryDirectory(prefix="lingtu-slam-cmake-") as build_dir:
        return subprocess.run(  # noqa: S603 - arguments are controlled test inputs
            [
                cmake_executable,
                "-S",
                str(source_dir),
                "-B",
                build_dir,
                "-DLINGTU_SLAM_BUILD_TESTS=OFF",
                *cmake_definitions,
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=60,
        )


def test_native_slam_dds_runtime_requires_real_fastlio_backend() -> None:
    result = _configure_native_slam(
        "-DLINGTU_SLAM_BUILD_DDS_RUNTIME=ON",
        "-DLINGTU_SLAM_FASTLIO2_BACKEND=OFF",
    )

    assert result.returncode != 0
    assert "requires LINGTU_SLAM_FASTLIO2_BACKEND=ON" in (
        result.stdout + result.stderr
    )


@pytest.mark.parametrize(
    ("deprecated_option", "replacement_option"),
    [
        ("LINGTU_SLAM_BUILD_CPP_DDS_RUNTIME", "LINGTU_SLAM_BUILD_DDS_RUNTIME"),
        ("LINGTU_SLAM_BUILD_CYCLONE_DDS_RUNTIME", "LINGTU_SLAM_BUILD_DDS_RUNTIME"),
        ("LINGTU_SLAM_WITH_FASTLIO2", "LINGTU_SLAM_FASTLIO2_BACKEND"),
    ],
)
def test_native_slam_rejects_removed_cmake_options(
    deprecated_option: str,
    replacement_option: str,
) -> None:
    result = _configure_native_slam(f"-D{deprecated_option}=OFF")

    assert result.returncode != 0
    output = " ".join((result.stdout + result.stderr).split())
    assert f"{deprecated_option} was removed" in output
    assert f"use {replacement_option} instead" in output


def test_native_slam_contract_only_configure_remains_supported() -> None:
    result = _configure_native_slam(
        "-DLINGTU_SLAM_BUILD_DDS_RUNTIME=OFF",
        "-DLINGTU_SLAM_FASTLIO2_BACKEND=OFF",
    )

    assert result.returncode == 0, result.stdout + result.stderr


def test_native_slam_cmake_registers_every_built_test_with_ctest() -> None:
    cmake = Path("src/localization/slam/cpp/CMakeLists.txt").read_text(encoding="utf-8")

    for name, target in (
        ("messages_contract", "test_slam_contract"),
        ("messages_map_tracking_health", "test_map_tracking_health"),
        ("messages_initial_body_origin", "test_initial_body_origin"),
        ("messages_imu_acceleration_scale", "test_imu_acceleration_scale"),
    ):
        assert f"NAME {name}" in cmake
        assert f"COMMAND {target}" in cmake


def test_native_slam_accepts_legacy_and_namespaced_yaml_cpp_targets() -> None:
    cmake = Path("src/localization/slam/cpp/CMakeLists.txt").read_text(encoding="utf-8")

    assert "if(TARGET yaml-cpp::yaml-cpp)" in cmake
    assert "elseif(TARGET yaml-cpp)" in cmake
    assert cmake.count("${LINGTU_YAML_CPP_TARGET}") == 2


def test_native_cloud_allocations_support_pcl_1_10_pointer_alias() -> None:
    localization = Path("src/localization")
    offenders = [
        str(path)
        for path in localization.rglob("*.cpp")
        if any(
            allocation in path.read_text(encoding="utf-8")
            for allocation in (
                "std::make_shared<CloudType>",
                "std::make_shared<LocalizerCloud>",
            )
        )
    ]

    assert offenders == []


def test_slam_control_dds_topics_have_runtime_contracts_and_command_qos() -> None:
    from runtime.graph import load_runtime_graph

    graph = load_runtime_graph()
    expected_qos = {
        "/slam/map_command": "reliable_volatile_keep_last_32",
        "/slam/map_event": "reliable_transient_local_keep_last_64",
        "/slam/relocalization/request": "reliable_volatile_keep_last_32",
        "/slam/relocalization/response": "reliable_transient_local_keep_last_64",
        "/slam/state_at_scan": "best_effort_volatile_keep_last_5_deadline_20ms",
        "/slam/localization_quality": "best_effort_volatile_keep_last_5_deadline_20ms",
        "/slam/localization_health": "reliable_volatile_keep_last_10",
    }
    for topic, qos in expected_qos.items():
        assert graph.topic_contracts[topic]["qos"] == qos

    native_topics = Path("src/message/cpp/topics.hpp").read_text(encoding="utf-8")
    for topic in ("rt/slam/map_command", "rt/slam/relocalization/request"):
        assert topic in native_topics
    for topic in ("rt/slam/map_event", "rt/slam/relocalization/response"):
        assert topic in native_topics

    input_topics = {"/slam/map_command", "/slam/relocalization/request"}
    output_topics = {
        "/slam/map_event",
        "/slam/relocalization/response",
        "/slam/state_at_scan",
    }
    real_endpoint = graph.envs["real"]["endpoints"]["contract"]
    sim_endpoint = graph.envs["sim"]["backends"]["mujoco"]["endpoints"]["contract"]
    for endpoint in (real_endpoint, sim_endpoint):
        assert input_topics <= set(endpoint["source_topics"])
        assert output_topics <= set(endpoint["exposed_topics"])

    runtime_source = Path("src/localization/slam/cpp/cyclone_runtime.cpp").read_text(
        encoding="utf-8"
    )
    for label, profile in (
        ("map_snapshot_request", "CommandRequest"),
        ("relocalization_request", "CommandRequest"),
        ("map_snapshot_ack", "CommandAck"),
        ("relocalization_response", "CommandAck"),
    ):
        start = runtime_source.index(f'"{label}"')
        assert f"QosProfile::{profile}" in runtime_source[start : start + 120]

    control_source = Path("src/localization/slam/cpp/slam_control.cpp").read_text(
        encoding="utf-8"
    )
    for fragment in (
        "auto request_qos = make_qos(QosProfile::CommandRequest);",
        "auto response_qos = make_qos(QosProfile::CommandAck);",
        "dds_create_writer(publisher, request_topic, request_qos.get(), nullptr)",
        "dds_create_reader(subscriber, response_topic, response_qos.get(), nullptr)",
    ):
        assert fragment in control_source


def test_slam_map_snapshot_uses_typed_request_and_ack() -> None:
    idl = Path("src/message/idl/messages.idl").read_text(encoding="utf-8")
    topics = Path("src/message/cpp/topics.hpp").read_text(encoding="utf-8")
    runtime = Path("src/localization/slam/cpp/cyclone_runtime.cpp").read_text(
        encoding="utf-8"
    )
    control = Path("src/localization/slam/cpp/slam_control.cpp").read_text(
        encoding="utf-8"
    )

    assert "struct SlamMapSnapshotRequest" in idl
    assert "struct SlamMapSnapshotAck" in idl
    request_struct = idl.split("struct SlamMapSnapshotRequest {", 1)[1].split(
        "};", 1
    )[0]
    ack_struct = idl.split("struct SlamMapSnapshotAck {", 1)[1].split("};", 1)[0]
    for declaration in (
        "string request_id;",
        "string map_id;",
        "string product_session_id;",
        "string output_path;",
        "boolean save_patches;",
    ):
        assert declaration in request_struct
    for declaration in (
        "string request_id;",
        "string map_id;",
        "boolean success;",
        "string message;",
        "string output_path;",
        "string runtime_instance_id;",
        "string product_session_id;",
        "unsigned long long reset_epoch;",
        "unsigned long long observation_sequence;",
        "unsigned long long captured_at_ns;",
        "string frame_id;",
        "unsigned long long point_count;",
        "string state;",
        "boolean healthy;",
        "string health_message;",
    ):
        assert declaration in ack_struct
    assert '"lingtu.dds.SlamMapSnapshotRequest"' in topics
    assert '"lingtu.dds.SlamMapSnapshotAck"' in topics
    assert "reader<lingtu_dds_SlamMapSnapshotRequest>" in runtime
    assert "writer<lingtu_dds_SlamMapSnapshotAck>" in runtime
    assert "drainMapSnapshotRequests" in runtime
    assert "writeMapSnapshotAck" in runtime
    assert "drainMapCommands" not in runtime
    assert "writeMapEvent" not in runtime
    assert "parseMapCommand" not in runtime
    assert "lingtu_dds_Text_desc" not in control
    assert "lingtu_dds_SlamMapSnapshotRequest" not in control
    assert "lingtu_dds_SlamMapSnapshotAck" not in control
    assert "save-map" not in control
    assert 'runtime_product != "map"' in runtime
    assert 'reject("map_product_required")' in runtime
    assert "runtime_session.empty()" in runtime
    assert 'reject("product_session_required")' in runtime
    assert "requested_session.empty()" in runtime
    assert 'reject("missing_product_session_id")' in runtime
    assert "requested_session != runtime_session" in runtime
    assert 'reject("product_session_mismatch")' in runtime


def _cyclone_runtime_source() -> str:
    return (
        LOCALIZATION_ROOT
        / "slam"
        / "cpp"
        / "cyclone_runtime.cpp"
    ).read_text(encoding="utf-8")


def _imu_frame_contract_source() -> str:
    return (
        LOCALIZATION_ROOT
        / "slam"
        / "cpp"
        / "imu_frame_contract.hpp"
    ).read_text(encoding="utf-8")


def test_native_backend_factories_reject_unavailable_profiles() -> None:
    path = Path("src/localization/slam/cpp/cyclone_runtime.cpp")
    source = path.read_text(encoding="utf-8")
    start = source.index("std::unique_ptr<ISlamBackend> createBackend")
    end = source.index("\n}", start)
    factory = source[start:end]

    assert 'throw std::invalid_argument("unsupported SLAM backend: " + backend);' in factory
    assert "makeContractBackend(normalized)" not in factory


def test_native_cyclone_runtime_catches_up_imu_sensor_stream_batches() -> None:
    source = _cyclone_runtime_source()

    assert "constexpr std::size_t kSensorStreamCatchupBatches = 16;" in source
    assert "void drainImu(Handler&& handler)" in source
    assert (
        "imu_reader_,\n"
        "        lingtu_dds_Imu_desc,\n"
        "        std::forward<Handler>(handler),\n"
        "        kSensorStreamCatchupBatches"
    ) in source


def test_native_cyclone_runtime_binds_sim_identity_and_env_defaults() -> None:
    source = _cyclone_runtime_source()
    parse_start = source.index("CliConfig parseArgs(int argc, char** argv)")
    parse_end = source.index("dds_entity_t checked", parse_start)
    parser = source[parse_start:parse_end]
    main_start = source.index("int main(int argc, char** argv)")
    main_setup_end = source.index("auto backend = createBackend", main_start)
    main_setup = source[main_start:main_setup_end]
    status_start = source.index("std::string statusSnapshotJson(")
    status_end = source.index("void writeTextAtomic", status_start)
    status = source[status_start:status_end]

    assert 'envOrEmpty("LINGTU_SLAM_MODE")' in parser
    assert 'envOrEmpty("LINGTU_SLAM_MAP")' in parser
    assert 'envOrEmpty("LINGTU_SLAM_CONFIG")' in parser
    assert 'envOrEmpty("LINGTU_SESSION_ROOT")' in parser
    assert '"slam.status.json"' in parser
    assert parser.index('envOrEmpty("LINGTU_SLAM_MODE")') < parser.index(
        'if (arg == "--backend")'
    )
    assert parser.index('envOrEmpty("LINGTU_SLAM_CONFIG")') < parser.index(
        'if (arg == "--backend")'
    )
    assert parser.index('arg == "--mode"') < parser.index("cfg.mode = next();")
    assert parser.index('arg == "--map"') < parser.index("cfg.map_path = next();")
    assert parser.index('arg == "--status-json"') < parser.index(
        "cfg.status_json_path = next();"
    )

    assert 'envOrEmpty("LINGTU_ENV") == "sim"' in main_setup
    assert 'envOrEmpty("LINGTU_PRODUCT")' in main_setup
    assert "productSessionId().empty()" in main_setup
    assert 'std::getenv("LINGTU_PRODUCT_SESSION_ID")' in source
    assert "simulationIdentityJson() +" in status
    assert '\\"native_product\\":' in source
    assert '\\"product\\":' in source
    assert '\\"product_session_id\\":' in source


def test_native_cyclone_runtime_rejects_wrong_imu_frames_before_acceptance() -> None:
    source = _cyclone_runtime_source()
    start = source.index("dds.drainImu([&](const lingtu_dds_Imu& msg)")
    end = source.index("dds.drainLidar", start)
    imu_drain = source[start:end]

    assert "validateImuFrame(msg.header.frame_id)" in imu_drain
    frame_contract = _imu_frame_contract_source()
    assert "imu_frame_missing" in frame_contract
    assert "imu_frame_mismatch" in frame_contract
    assert imu_drain.index("validateImuFrame(msg.header.frame_id)") < imu_drain.index(
        "imu_input_rate.mark"
    )
    assert imu_drain.index("validateImuFrame(msg.header.frame_id)") < imu_drain.index(
        "backend->feedImu"
    )


def test_native_cyclone_runtime_drains_lidar_latest_only_with_descriptor_free() -> None:
    source = _cyclone_runtime_source()
    drain_lidar_start = source.index("void drainLidar(Handler&& handler)")
    drain_lidar_end = source.index("template <typename Handler>", drain_lidar_start + 1)
    drain_lidar = source[drain_lidar_start:drain_lidar_end]
    latest_start = source.index("void drainLatestReader(")
    latest_end = source.index("using lingtu::dds::QosProfile;", latest_start)
    latest_reader = source[latest_start:latest_end]

    assert "drainLatestReader<lingtu_dds_LivoxFrame>" in drain_lidar
    assert "drainReader<lingtu_dds_LivoxFrame>" not in drain_lidar
    assert "handler(*static_cast<T*>(latest_batch->samples()[latest_index]));" in latest_reader
    assert latest_reader.count("handler(*static_cast<T*>") == 1
    assert latest_reader.index("for (std::size_t batch = 0;") < latest_reader.index(
        "handler(*static_cast<T*>(latest_batch->samples()[latest_index]));"
    )
    assert "latest_batch = std::move(current_batch);" in latest_reader
    assert "std::unique_ptr<DdsSampleBatch<T>> latest_batch;" in latest_reader
    assert "std::fprintf" not in latest_reader
    assert "dds_sample_free(sample, &descriptor_, DDS_FREE_ALL);" in source


def test_native_cyclone_runtime_dedupes_odom_and_scan_state_by_output_stamp() -> None:
    source = _cyclone_runtime_source()

    assert "double last_odometry_stamp_s = -1.0;" in source
    assert "double last_state_estimation_stamp_s = -1.0;" in source

    odom_start = source.index("if (out.odometry_odom_body.has_value() &&")
    odom_end = source.index("if (out.state_estimation_at_scan.has_value() &&", odom_start)
    odom_block = source[odom_start:odom_end]
    assert "std::abs(out.stamp_s - last_odometry_stamp_s) > 1e-6" in odom_block
    assert odom_block.index("last_odometry_stamp_s = out.stamp_s;") < odom_block.index(
        "dds.writeOdom(msg);"
    )
    assert "registered_cloud_body->stamp_s" not in odom_block
    assert "map_cloud_map->stamp_s" not in odom_block

    state_start = source.index("if (out.state_estimation_at_scan.has_value() &&")
    state_end = source.index("if (out.registered_cloud_body.has_value() &&", state_start)
    state_block = source[state_start:state_end]
    assert "std::abs(out.stamp_s - last_state_estimation_stamp_s) > 1e-6" in state_block
    assert state_block.index("last_state_estimation_stamp_s = out.stamp_s;") < state_block.index(
        "dds.writeState(msg);"
    )
    assert "registered_cloud_body->stamp_s" not in state_block
    assert "map_cloud_map->stamp_s" not in state_block


def test_native_cyclone_runtime_maps_fastlio_velocity_into_odom_twist_directly() -> None:
    source = _cyclone_runtime_source()
    odom_converter = source[
        source.index("lingtu_dds_Odometry toDdsOdom(") : source.index("struct TfMessage")
    ]

    assert "double vx" in odom_converter
    assert "double vy" in odom_converter
    assert "double vz" in odom_converter
    assert "out.twist.twist.linear.x = vx;" in odom_converter
    assert "out.twist.twist.linear.y = vy;" in odom_converter
    assert "out.twist.twist.linear.z = vz;" in odom_converter
    assert "std::isfinite" not in odom_converter
    assert "0.0" not in odom_converter

    odom_start = source.index("const auto msg = toDdsOdom(\n            *out.odometry_odom_body")
    odom_end = source.index("dds.writeOdom(msg);", odom_start)
    odom_call = source[odom_start:odom_end]
    state_start = source.index(
        "const auto msg = toDdsOdom(\n            *out.state_estimation_at_scan"
    )
    state_end = source.index("dds.writeState(msg);", state_start)
    state_call = source[state_start:state_end]
    for call in (odom_call, state_call):
        assert "out.fastlio_velocity_x" in call
        assert "out.fastlio_velocity_y" in call
        assert "out.fastlio_velocity_z" in call


def test_native_slam_forwards_structured_fastlio_lidar_update_diagnostics() -> None:
    root = LOCALIZATION_ROOT
    ieskf_header = (root / "fastlio2" / "src" / "map_builder" / "ieskf.h").read_text(
        encoding="utf-8"
    )
    ieskf_source = (root / "fastlio2" / "src" / "map_builder" / "ieskf.cpp").read_text(
        encoding="utf-8"
    )
    fastlio_source = (root / "slam" / "cpp" / "fastlio.cpp").read_text(encoding="utf-8")
    cyclone_source = _cyclone_runtime_source()

    for reason in (
        "no_valid_measurement",
        "pathological_degeneracy",
        "candidate_translation_limit_exceeded",
        "candidate_rotation_limit_exceeded",
        "candidate_velocity_limit_exceeded",
        "candidate_velocity_delta_limit_exceeded",
        "nonconverged_update",
        "degenerate_nonconverged_update",
        "information_ldlt_decomposition_failed",
        "information_ldlt_not_positive",
        "candidate_covariance_nonfinite",
        "candidate_covariance_nonpositive_diagonal",
        "posterior_covariance_nonfinite",
        "posterior_covariance_nonpositive_diagonal",
    ):
        assert f'"{reason}"' in ieskf_header

    assert "LidarUpdateDiagnostics m_lidar_update_diagnostics" in ieskf_header
    assert "previous_rejection_reason" in ieskf_header
    assert "candidate_translation_m" in ieskf_header
    assert "candidate_rotation_rad" in ieskf_header
    assert "candidate_velocity_mps" in ieskf_header
    assert "candidate_velocity_delta_mps" in ieskf_header
    assert "information_ldlt_evaluated" in ieskf_header
    assert "candidate_covariance_evaluated" in ieskf_header
    assert "posterior_covariance_evaluated" in ieskf_header
    update_source = ieskf_source[ieskf_source.index("bool IESKF::update()") :]
    assert update_source.count("recordLidarUpdateRejection(") == update_source.count(
        "return false;"
    )
    assert "builder_->lastLidarUpdateDiagnostics()" in fastlio_source

    json_start = cyclone_source.index("std::string fastLioLidarUpdateJson(")
    json_end = cyclone_source.index("std::string statusSnapshotJson(", json_start)
    diagnostics_json = cyclone_source[json_start:json_end]
    assert '"\\\"fastlio_lidar_update\\\":"' in cyclone_source
    assert "fastLioLidarUpdateJson(out.fastlio_lidar_update)" in cyclone_source
    for field in (
        "attempt_sequence",
        "rejection_reason",
        "previous_rejection_reason",
        "consecutive_rejections",
        "downsampled_points",
        "effective_points",
        "candidate",
        "thresholds",
        "information_ldlt",
        "candidate_covariance",
        "posterior_covariance",
    ):
        assert f'\\\"{field}\\\"' in diagnostics_json


def test_cpp_slam_adapter_converts_native_map_cloud_at_host_boundary(tmp_path) -> None:
    from localization.adapters.status import CppSlamStatusAdapterModule

    cloud = PointCloud2(
        points=np.array([[1.0, 2.0, 3.0]], dtype=np.float32),
        frame_id="map",
        ts=12.5,
    )
    (tmp_path / "map_cloud.bin").write_bytes(cloud.encode())
    adapter = CppSlamStatusAdapterModule(cloud_snapshot_dir=str(tmp_path))
    frames = []
    adapter.map_cloud_frame._add_callback(frames.append)

    adapter._poll_cloud_snapshots()

    assert "map_cloud" not in adapter.ports_out
    assert len(frames) == 1
    assert isinstance(frames[0], MapCloudFrame)
    assert frames[0].mode == "FULL"
    assert frames[0].source == "cpp_slam_status:map_cloud"
    assert frames[0].frame_id == "map"
    assert frames[0].points.tolist() == [[1.0, 2.0, 3.0]]


def test_cpp_slam_adapter_reads_the_direct_child_session_status(
    tmp_path,
    monkeypatch,
) -> None:
    from localization.adapters.status import CppSlamStatusAdapterModule

    monkeypatch.delenv("LINGTU_SLAM_STATUS_JSON", raising=False)
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))

    adapter = CppSlamStatusAdapterModule()

    assert adapter.health()["status_snapshot_path"] == str(
        tmp_path / "slam.status.json"
    )


def test_slam_stack_requires_external_native_adapter() -> None:
    from lingtu.assembly.stacks.slam import slam

    with pytest.raises(ValueError, match="localization_adapter"):
        slam("native_dds")

    bp = slam("native_dds", localization_adapter="cpp_slam_status")
    assert [entry.name for entry in bp._entries] == ["SlamAdapterModule"]


@pytest.mark.parametrize("profile", ("pointlio", "genz"))
def test_slam_stack_rejects_placeholder_profiles(profile: str) -> None:
    from lingtu.assembly.stacks.slam import slam

    with pytest.raises(ValueError, match="unsupported native SLAM profile"):
        slam(profile, localization_adapter="cpp_slam_status")


def test_ros2_bridge_compatibility_is_removed(monkeypatch) -> None:
    from localization.adapters.resolver import localization_adapter_module

    monkeypatch.delenv("LINGTU_ENABLE_ROS2_COMPAT", raising=False)
    monkeypatch.delenv("LINGTU_ENABLE_LEGACY_ROS2_SERVICES", raising=False)

    with pytest.raises(ImportError, match="Unsupported localization adapter"):
        localization_adapter_module("ros2_slam_bridge")


def test_slam_stack_does_not_restore_removed_ros2_bridge(monkeypatch) -> None:
    import lingtu.assembly.plugins as plugin_seed
    from lingtu.assembly.stacks.slam import slam

    monkeypatch.setenv("LINGTU_ENABLE_ROS2_COMPAT", "1")
    plugin_seed = importlib.reload(plugin_seed)
    try:
        with pytest.raises(ImportError, match="Unsupported localization adapter"):
            slam(
                "native_dds",
                localization_adapter="ros2_slam_bridge",
            )
    finally:
        monkeypatch.delenv("LINGTU_ENABLE_ROS2_COMPAT", raising=False)
        importlib.reload(plugin_seed)


def test_native_slam_adapter_wiring_covers_host_consumers() -> None:
    from lingtu.assembly.wires.full_stack import full_stack_wire_specs

    modules = {
        "SlamAdapterModule",
        "GatewayModule",
        "maps.service",
        "ThunderDriver",
        "lidar",
    }
    specs = full_stack_wire_specs(
        modules,
        driver_module="ThunderDriver",
        slam_profile="native_dds",
        enable_semantic=False,
    )
    wires = {f"{spec.out_module}.{spec.out_port}->{spec.in_module}.{spec.in_port}" for spec in specs}

    assert not any("saved_map" in wire for wire in wires)
    assert not any("maps.service" in wire for wire in wires)
    assert "SlamAdapterModule.localization_status->GatewayModule.localization_status" in wires
    assert not any(wire.endswith("->SlamAdapterModule.lidar_raw_scan") for wire in wires)
    assert not any(wire.endswith("->SlamAdapterModule.lidar_imu") for wire in wires)


def test_native_mapping_save_path_writes_only_real_map_artifacts() -> None:
    fastlio = Path("src/localization/slam/cpp/fastlio.cpp").read_text(encoding="utf-8")
    header = Path("src/localization/slam/cpp/slam.hpp").read_text(encoding="utf-8")
    cyclone_runtime = Path("src/localization/slam/cpp/cyclone_runtime.cpp").read_text(encoding="utf-8")

    assert "Status saveMap(const std::string& pcd_path) override" in fastlio
    assert "builder_->saveMap(pcd.string())" in fastlio
    assert "writeTrajectory(pcd.parent_path(), pose_history_)" in fastlio
    assert (
        "writePatchBundle(pcd.parent_path(), patches, patch_history_dropped_count_)"
        in fastlio
    )
    assert "max_patch_snapshots" in fastlio
    assert "patch_min_translation_m" in fastlio
    assert "patch_min_rotation_rad" in fastlio
    assert "patch_history_.size() > max_snapshots" in fastlio
    assert "patch_history_.size() > 300" not in fastlio
    assert "map.raw.pcd" not in fastlio
    assert "map_optimization" not in fastlio
    assert "MapOptimizationReport" not in fastlio
    assert "OptimizedMapResult" not in fastlio

    assert 'action == "track_against_map"' in cyclone_runtime
    assert "if (runtime_mode != SlamMode::Localization)" in cyclone_runtime
    assert '"localization_mode_required"' in cyclone_runtime
    assert "map_optimization" not in header
    assert "map_optimization" not in cyclone_runtime


def test_slam_cpp_build_declares_native_dds_runtime() -> None:
    cmake = Path("src/localization/slam/cpp/CMakeLists.txt").read_text(encoding="utf-8")
    cyclone_runtime = Path("src/localization/slam/cpp/cyclone_runtime.cpp").read_text(encoding="utf-8")
    fastlio = Path("src/localization/slam/cpp/fastlio.cpp").read_text(encoding="utf-8")
    sdk2_dds = Path("src/drivers/real/lidar/native/dds_module.cpp").read_text(encoding="utf-8")
    build_script = Path("scripts/build/build_slam_core.sh").read_text(encoding="utf-8")

    assert "LINGTU_SLAM_BUILD_DDS_RUNTIME" in cmake
    assert "add_executable(slamd cyclone_runtime.cpp)" in cmake
    assert "LINGTU_SLAM_BUILD_ROS2_DDS_RUNTIME" not in cmake
    assert "adapters/ros2" not in cmake
    assert "find_package(iceoryx_binding_c QUIET)" in cmake
    assert "lingtu_add_dds_c_messages(messages_cyclone_idl" in cmake
    assert "CYCLONEDDS_IDLC_EXECUTABLE" not in cmake
    assert "CycloneDDS::ddsc" in cmake
    assert "CycloneDDS-CXX" not in cmake
    assert "add_executable(slamctl slam_control.cpp)" in cmake
    assert "LINGTU_SLAM_BUILD_DDS_RUNTIME" in build_script
    assert "LINGTU_SLAM_BUILD_CPP_DDS_RUNTIME" in build_script
    assert "LINGTU_SLAM_BUILD_CYCLONE_DDS_RUNTIME" in build_script
    assert "LINGTU_SLAM_WITH_FASTLIO2" in build_script
    assert "was removed; use" in build_script
    assert "LINGTU_SLAM_BUILD_ROS2_DDS_RUNTIME" not in build_script
    assert "CPU_BBS3D_ROOT" in build_script
    assert "LINGTU_REQUIRE_BBS3D" in build_script
    assert not Path("src/localization/slam/module.py").exists()
    assert not Path("src/localization/adapters/ros2/cpp/ros2_dds_runtime.cpp").exists()
    assert '#include "dds/dds.h"' in cyclone_runtime
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
    assert "kTrackAgainstMapDegradedFailureCount" in cyclone_runtime
    assert "restart_track_against_map" in cyclone_runtime
    assert 'action == "seeded_relocalize" || action == "global_relocalize"' in cyclone_runtime
    assert "registered_cloud_stale" in cyclone_runtime
    assert "last_track_against_map_scan_s" in cyclone_runtime
    assert "lingtu::message::kTf.dds_topic.data()" in cyclone_runtime
    assert "lingtu::message::kTfStatic.dds_topic.data()" not in cyclone_runtime
    assert '#include "message/cpp/qos.hpp"' in cyclone_runtime
    assert "using lingtu::dds::QosProfile" in cyclone_runtime
    assert "using lingtu::dds::make_qos" in cyclone_runtime
    assert "QosProfile::RawLidarStream" in cyclone_runtime
    assert "QosProfile::SensorStream" in cyclone_runtime
    assert "QosProfile::TfDynamic" in cyclone_runtime
    assert "QosProfile::HighFreqState" in cyclone_runtime
    assert "QosProfile::LocalizationHealth" in cyclone_runtime
    assert "QosProfile::LidarPointcloud" in cyclone_runtime
    assert "lingtu_dds_contracts" in cmake
    assert '#include "message/cpp/qos.hpp"' in sdk2_dds
    assert "qos_for_topic(lingtu::message::kLidarRawFrame.dds_topic)" in sdk2_dds
    assert "qos_for_topic(lingtu::message::kLidarRawPacket.dds_topic)" in sdk2_dds
    assert "make_qos(lingtu::dds::QosProfile::SensorStream)" in sdk2_dds
    assert "state_estimation_at_scan_ = odometry_odom_body_" in fastlio
    assert "map_odom_pose_ = result.map_odom" in fastlio
    assert "composePoses(map_odom_pose_, *odometry_odom_body_)" in fastlio
    assert "effective_guess" in fastlio
    assert "relocalization_max_fitness" in fastlio
    assert "EvaluateRelocalizationGate" in fastlio
    assert "relocalization_map_bounds_margin_m" in fastlio
    assert "relocalization_outside_map_bounds" in fastlio
    assert "preserve_tracking_on_relocalization_failure" in fastlio
    assert 'relocalization_state_ = preserve_tracking ? "tracking" : relocalization_state' in fastlio
    assert "failRelocalization" in fastlio
    assert "map_alignment_update" in fastlio
    assert "state_estimation_at_scan_ = odometry_odom_body_" in fastlio
    assert "updateMapBounds" in fastlio
    assert "poseInsideMapBounds" in fastlio
    assert "relocalization_refine_backend" in cyclone_runtime
    assert "relocalization_map_body" in cyclone_runtime
    assert "dds.writeTf(msg.msg)" in cyclone_runtime
    assert "backend->feedLidar" in cyclone_runtime
    assert "backend->feedImu" in cyclone_runtime
    assert "--log-status-s" in cyclone_runtime


def test_native_slam_product_binary_names_hide_transport_details() -> None:
    product_surfaces = (
        Path("src/localization/slam/cpp/CMakeLists.txt"),
        Path("scripts/build/build_slam_core.sh"),
        Path("scripts/deploy/package_native_release.sh"),
        Path("scripts/deploy/thunder/run_slam_dds.sh"),
        Path("scripts/deploy/thunder/lt-slam.service"),
        Path("src/runtime/service_catalogs/thunder.py"),
        Path("config/runtime_graph/acceptance/mujoco_native_navigation_acceptance.json"),
        Path("config/runtime_graph/acceptance/mujoco_teleop_avoid_native_acceptance.json"),
    )
    combined = "\n".join(path.read_text(encoding="utf-8") for path in product_surfaces)

    assert "build/slam_core/slamd" in combined
    assert "build/slam_core/slamctl" in combined
    assert "messages_cyclone_runtime" not in combined
    assert "messages_control" not in combined


def test_slam_relocalization_has_typed_dds_request_reply_contract() -> None:
    idl = Path("src/message/idl/messages.idl").read_text(encoding="utf-8")
    topics = Path("src/message/cpp/topics.hpp").read_text(encoding="utf-8")
    runtime_topics = Path("src/runtime/runtime_interface.py").read_text(encoding="utf-8")
    cyclone_runtime = Path("src/localization/slam/cpp/cyclone_runtime.cpp").read_text(encoding="utf-8")
    slam_control = Path("src/localization/slam/cpp/slam_control.cpp").read_text(encoding="utf-8")

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
    assert "kTrackAgainstMapDegradedFailureCount" in cyclone_runtime
    assert "restart_track_against_map" in cyclone_runtime
    assert "--track-against-map-period-s" in cyclone_runtime
    assert "registered_cloud_stale" in cyclone_runtime
    assert "backend->startRelocalizeAsync(track_against_map_seed)" in cyclone_runtime
    assert "backend->pollRelocalizeAsync()" in cyclone_runtime
    assert "backend->relocalize(track_against_map_seed)" not in cyclone_runtime

    assert "runTypedRelocalizationService" in slam_control
    assert "kSlamRelocalizationRequest.dds_topic.data()" in slam_control
    assert "kSlamRelocalizationResponse.dds_topic.data()" in slam_control
    assert "lingtu.slam.relocalization_response.v1" in slam_control
    assert "track_against_map_enabled" in slam_control
    assert "track_against_map_failures" in slam_control
    assert "cpp_typed_dds" in slam_control
    assert "kSlamMapSnapshotRequest.dds_topic.data()" not in slam_control
    assert "map paths are selected by ProductControl" in slam_control
    assert "product_control_map_switch_required" in cyclone_runtime
    assert "lingtu_dds_SlamMapSnapshotRequest" in cyclone_runtime
    assert "lingtu_dds_SlamMapSnapshotAck" in cyclone_runtime

def test_relocalization_response_preserves_overlap_evidence_across_boundaries() -> None:
    evidence_fields = [
        "refine_input_points",
        "refine_evaluated_points",
        "refine_support_ratio",
        "refine_overlap_inlier_ratio",
    ]
    expected_declarations = [
        "long refine_input_points;",
        "long refine_evaluated_points;",
        "double refine_support_ratio;",
        "double refine_overlap_inlier_ratio;",
    ]
    idl = Path("src/message/idl/messages.idl").read_text(encoding="utf-8")
    response_struct = idl.split("struct RelocalizationResponse {", 1)[1].split(
        "};", 1
    )[0]
    assert all(item in response_struct for item in expected_declarations)

    cyclone_runtime = Path(
        "src/localization/slam/cpp/cyclone_runtime.cpp"
    ).read_text(encoding="utf-8")
    slam_control = Path("src/localization/slam/cpp/slam_control.cpp").read_text(
        encoding="utf-8"
    )
    for field_name in evidence_fields:
        assert f"response.msg.{field_name}" in cyclone_runtime
        assert f"response.{field_name}" in slam_control


def test_native_relocalization_uses_map_icp_with_generation_guard() -> None:
    native_relocalizer = Path("src/localization/slam/cpp/native_relocalizer.cpp").read_text(encoding="utf-8")
    map_icp_header = Path("src/localization/slam/cpp/map_icp.hpp").read_text(encoding="utf-8")
    map_icp_source = Path("src/localization/slam/cpp/map_icp.cpp").read_text(encoding="utf-8")
    cmake = Path("src/localization/slam/cpp/CMakeLists.txt").read_text(encoding="utf-8")
    fastlio = Path("src/localization/slam/cpp/fastlio.cpp").read_text(encoding="utf-8")
    gate = Path("src/localization/slam/cpp/relocalization_gate.hpp").read_text(encoding="utf-8")

    assert '#include "map_icp.hpp"' in native_relocalizer
    assert "MapIcp map_icp" in native_relocalizer
    assert "map_icp.verifySeed(scan, guess, map_generation)" in native_relocalizer
    seeded_body = native_relocalizer.split("NativeRelocalizationResult NativeRelocalizer::relocalize(", 1)[1].split(
        "NativeRelocalizationResult NativeRelocalizer::globalRelocalize(", 1
    )[0]
    assert "map_icp.refine(" not in seeded_body
    assert "map_icp.refine(scan, coarse.pose, map_generation)" in native_relocalizer
    assert "native_relocalizer_map_generation_mismatch" in native_relocalizer
    assert "native_global_relocalizer_map_generation_mismatch" in native_relocalizer

    assert "class MapIcp" in map_icp_header
    assert "std::uint64_t map_generation_" in map_icp_header
    assert "map_icp_generation_mismatch" in map_icp_source
    assert "map_icp_failed" in map_icp_source
    assert "alignPlanar" in map_icp_source
    assert "fixed_seed_planar_icp" in map_icp_source
    assert "map_icp.cpp" in cmake

    success_gate = fastlio.index("if (!result.success)")
    commit_gate = fastlio.index("EvaluateRelocalizationGate")
    bounds_gate = fastlio.index("relocalization_outside_map_bounds")
    mutation = fastlio.index("map_odom_pose_ = result.map_odom")
    assert success_gate < mutation
    assert commit_gate < mutation
    assert bounds_gate < mutation
    assert "relocalization_fitness_rejected" in gate
    assert "relocalization_degeneracy_metrics_unavailable" in gate
    assert "relocalization_translation_jump_rejected" in gate
    assert "relocalization_yaw_jump_rejected" in gate


def test_cpp_message_topic_contract_stays_ros_free() -> None:
    header = Path("src/message/cpp/topics.hpp").read_text(encoding="utf-8")

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
