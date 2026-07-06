import pytest

pytestmark = [pytest.mark.sim]

import json
from pathlib import Path
from types import SimpleNamespace

from runtime.tests.numpy_guard import import_numpy_or_skip

np = import_numpy_or_skip()

from sim.engine.mujoco.lidar import MuJoCoLidar

ROOT = Path(__file__).resolve().parents[2]
MID360_PATTERN = ROOT / "sim/assets/livox/mid360.npy"
MID360_SHA256 = "448821576a658673e8f7929992c8c0d687eb052657d7b584d038729a83da1bfb"


def _sha256(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def test_repo_mid360_pattern_asset_is_official_converted_scan_mode() -> None:
    assert MID360_PATTERN.is_file()
    assert _sha256(MID360_PATTERN) == MID360_SHA256

    angles = np.load(MID360_PATTERN, mmap_mode="r")

    assert angles.shape == (800000, 2)
    assert angles.dtype == np.float32
    assert 0.0 <= float(np.min(angles[:, 0])) < 1e-4
    assert np.isclose(float(np.max(angles[:, 0])), 2.0 * np.pi, atol=1e-5)
    assert np.isclose(float(np.min(angles[:, 1])), -0.1258784, atol=1e-5)
    assert np.isclose(float(np.max(angles[:, 1])), 0.9104336, atol=1e-5)


def test_mujoco_gates_default_to_forced_mid360_pattern() -> None:
    from sim.scripts.mujoco.live_gate import _build_parser as build_fastlio_parser
    from sim.scripts.mujoco.native_pct_gate import _build_parser as build_pct_parser

    fastlio_args = build_fastlio_parser().parse_args([])
    pct_args = build_pct_parser().parse_args([])

    assert fastlio_args.mid360_pattern == MID360_PATTERN
    assert pct_args.mid360_pattern == MID360_PATTERN
    assert fastlio_args.mid360_samples_per_frame == 15000
    assert pct_args.mid360_samples_per_frame == 24000
    assert fastlio_args.cmd_vel_angular_limit == 0.45
    assert fastlio_args.nav_max_angular_z == 0.45
    assert not fastlio_args.run_lingtu_tare
    assert fastlio_args.tare_min_goals == 2
    assert fastlio_args.tare_scenario == "indoor"
    assert fastlio_args.min_exploration_coverage_growth_ratio > 0.0
    assert not fastlio_args.allow_golden_spiral_lidar
    assert not pct_args.allow_golden_spiral_lidar
    assert fastlio_args.scan_time_profile == "physical_rolling"
    assert not fastlio_args.no_save_map_artifacts
    assert not fastlio_args.build_tomogram


def test_fastlio_live_gate_defaults_to_product_mujoco_lidar_backend() -> None:
    from sim.scripts.mujoco.live_gate import _build_parser

    args = _build_parser().parse_args([])

    assert args.lidar_backend == "mujoco_lidar"
    assert args.mujoco_lidar_backend == "cpu"
    assert args.allow_legacy_lidar_fallback is False
    assert args.localization_backend == ""


def test_product_mujoco_runtime_enables_realistic_lidar_returns() -> None:
    text = Path("src/drivers/sim/mujoco/runtime.py").read_text(encoding="utf-8")

    assert "add_noise=True" in text
    assert "mid360_npy_path=str(pattern_path)" in text


def test_fastlio_live_gate_removes_portable_lio_backend() -> None:
    from sim.scripts.mujoco.live_gate import _build_parser

    choices = _build_parser()._option_string_actions["--localization-backend"].choices
    script = Path("sim/scripts/mujoco/live_gate.py").read_text(
        encoding="utf-8"
    )

    assert choices is None
    assert "from runtime.adapters.ros2.fastlio2_live_bridge import" not in script
    assert "FastLio2Process(" not in script


def test_fastlio_live_gate_reports_product_lidar_backend_contract() -> None:
    script = Path("sim/scripts/mujoco/live_gate.py").read_text(
        encoding="utf-8"
    )

    assert '"lidar_backend": lidar_backend_report' in script
    assert '"product_lidar_backend_verified": product_lidar_backend_verified' in script
    assert '"product_mujoco_lidar_backend": product_lidar_backend_verified' in script
    assert "require_product_lidar_backend=True" in script
    assert "_RosLike" not in script
    assert "_load_portable_ros_like_modules" not in script
    assert "_PortableFastLio2Process" not in script
    assert '"lio_odometry"' in script
    assert '"lio_cloud_registered"' in script
    assert '"lio_cloud_map"' in script
    assert '"uses_ros_message_shim": False' in script


def test_fastlio_live_gate_exception_report_keeps_product_lidar_contract() -> None:
    from sim.scripts.mujoco.live_gate import (
        _build_parser,
        _gate_exception_report,
    )

    args = _build_parser().parse_args([])
    report = _gate_exception_report(args, RuntimeError("probe"))

    assert report["schema_version"] == "lingtu.mujoco_fastlio2_live_gate.v2"
    assert report["algorithm"] == "removed"
    assert report["localization_backend"] == "removed"
    assert report["localization_runtime"]["requires_ros2"] is False
    assert report["localization_runtime"]["requires_fastlio2_package"] is False
    assert report["localization_runtime"]["requires_std_msgs"] is False
    assert report["localization_runtime"]["uses_ros_message_shim"] is False
    assert report["lidar_backend"]["backend"] == "uninitialized"
    assert report["lidar_backend"]["requested_backend"] == "mujoco_lidar"
    assert report["lidar_backend"]["mujoco_lidar_backend"] == "cpu"
    assert report["product_lidar_backend_verified"] is False
    assert report["deliverable_contract"]["checks"]["product_mujoco_lidar_backend"] is False
    assert report["deliverable_contract"]["remaining_gaps"] == report["remaining_gaps"]


def test_saved_map_relocalize_discovers_tare_same_source_map(
    tmp_path,
    monkeypatch,
) -> None:
    from sim.scripts import saved_map_relocalize_runtime_gate as gate

    map_pcd = (
        tmp_path
        / "artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live_full/tare-1/same_source_map/map.pcd"
    )
    map_pcd.parent.mkdir(parents=True)
    map_pcd.write_text(
        "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n",
        encoding="ascii",
    )

    monkeypatch.setattr(gate, "ROOT", tmp_path)

    assert gate._resolve_latest_map() == map_pcd


def test_pct_saved_map_navigation_discovers_tare_same_source_tomogram(
    tmp_path,
    monkeypatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    tomogram = (
        tmp_path
        / "artifacts/server_sim_closure/mujoco_tare_exploration/tare-1/same_source_map/tomogram.pickle"
    )
    tomogram.parent.mkdir(parents=True)
    tomogram.write_bytes(b"tomogram")

    monkeypatch.setattr(gate, "ROOT", tmp_path)

    assert gate._resolve_tomogram(None) == tomogram


def test_pct_saved_map_navigation_uses_relocalized_same_source_tomogram(
    tmp_path,
    monkeypatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    relocalized_map = (
        tmp_path
        / "artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live_fullchain/tare/same_source_map/map.pcd"
    )
    relocalized_tomogram = relocalized_map.parent / "tomogram.pickle"
    newer_unrelated = (
        tmp_path
        / "artifacts/server_sim_closure/mujoco_tare_exploration/newer/same_source_map/tomogram.pickle"
    )
    for path in (relocalized_map, relocalized_tomogram, newer_unrelated):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"asset")

    monkeypatch.setattr(gate, "ROOT", tmp_path)

    assert gate._resolve_tomogram(
        None,
        relocalize_report={"map_pcd": str(relocalized_map)},
    ) == relocalized_tomogram


def test_pct_saved_map_navigation_rejects_mismatched_explicit_tomogram(
    tmp_path,
    monkeypatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    relocalized_map = tmp_path / "same_source_map_a" / "map.pcd"
    relocalized_tomogram = relocalized_map.parent / "tomogram.pickle"
    mismatched_tomogram = tmp_path / "same_source_map_b" / "tomogram.pickle"
    for path in (relocalized_map, relocalized_tomogram, mismatched_tomogram):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"asset")

    monkeypatch.setattr(gate, "ROOT", tmp_path)

    with pytest.raises(ValueError, match="explicit tomogram does not match"):
        gate._resolve_tomogram(
            mismatched_tomogram,
            relocalize_report={"map_pcd": str(relocalized_map)},
        )


def test_pct_saved_map_navigation_passes_short_route_progress_threshold() -> None:
    from sim.scripts.pct_saved_map_navigation_gate import (
        DEFAULT_SAVED_MAP_GOAL,
        _build_parser,
    )

    args = _build_parser().parse_args([])
    script = Path("sim/scripts/pct_saved_map_navigation_gate.py").read_text(
        encoding="utf-8"
    )

    assert args.min_route_progress_ratio == 0.90
    assert args.goal == DEFAULT_SAVED_MAP_GOAL
    assert '"--min-route-progress-ratio"' in script
    assert '"--use-last-pose"' in script
    assert '"--map-root"' in script


def test_fastlio_live_gate_reports_exploration_coverage_growth() -> None:
    from sim.scripts.mujoco.live_gate import _coverage_growth

    growth = _coverage_growth([
        {"known_m2": 1.0, "unknown_m2": 9.0},
        {"known_m2": 2.5, "unknown_m2": 7.5},
        {"known_m2": 4.0, "unknown_m2": 6.0},
    ])

    assert growth["first_ratio"] == 0.1
    assert growth["last_ratio"] == 0.4
    assert growth["max_ratio"] == 0.4
    assert np.isclose(growth["growth_ratio"], 0.3)
    assert growth["first_known_m2"] == 1.0
    assert growth["last_known_m2"] == 4.0


def test_fastlio_live_gate_has_no_ros_shutdown_path() -> None:
    source = (ROOT / "sim/scripts/mujoco/live_gate.py").read_text(
        encoding="utf-8"
    )

    assert "rclpy.ok()" not in source
    assert "rclpy.shutdown()" not in source


def test_fastlio_live_gate_writes_same_source_map_artifacts(tmp_path) -> None:
    from runtime.same_source_map_artifacts import (
        validate_same_source_map_metadata,
        write_same_source_map_artifacts,
    )

    points = np.array(
        [
            [0.0, 0.0, 0.1],
            [0.05, 0.0, 0.1],
            [1.0, 0.5, 0.2],
            [2.0, 1.5, 0.3],
        ],
        dtype=np.float32,
    )

    report = write_same_source_map_artifacts(
        artifact_dir=tmp_path / "same_source_map",
        points=points,
        frame_id="odom",
        world=tmp_path / "industrial_demo_scene.xml",
        source_topics=("/slam/map_cloud",),
        mapping_input_path="/points_raw + /imu_raw -> fastlio2 -> /slam/map_cloud",
        build_tomogram=False,
        tomogram_resolution=0.2,
        tomogram_slice_dh=0.25,
        tomogram_ground_h=0.0,
        map_artifact_max_span_m=120.0,
        tomogram_max_cells=50_000_000,
        extra_metadata={"scan_time_profile": "synthetic_rolling"},
    )

    assert report["ok"] is True
    assert report["source_contract"]["same_source_pcd"] is True
    assert report["source_contract"]["same_source_tomogram"] is False
    pcd = Path(report["assets"]["map_pcd"]["path"])
    metadata = Path(report["assets"]["metadata"]["path"])
    assert pcd.is_file()
    assert metadata.is_file()
    assert report["assets"]["map_pcd"]["point_count"] == 4
    assert "DATA ascii" in pcd.read_text(encoding="ascii")
    payload = json.loads(metadata.read_text(encoding="utf-8"))
    assert payload["scan_time_profile"] == "synthetic_rolling"
    assert payload["source_profile"] == "same_source_map_artifact_writer"
    assert payload["data_source"] == "same_source_map_artifact_writer"
    assert payload["frame_id"] == "odom"
    assert payload["artifacts"]["map_pcd"]["sha256"] == report["assets"]["map_pcd"]["sha256"]
    validation = validate_same_source_map_metadata(payload)
    assert validation["ok"] is True, validation


def test_same_source_map_metadata_rejects_derived_artifact_sha_drift() -> None:
    from runtime.same_source_map_artifacts import (
        validate_same_source_map_metadata,
    )

    metadata = {
        "source_profile": "sim_mujoco_live",
        "data_source": "mujoco_fastlio2_live",
        "slam_source": "fastlio2",
        "localization_source": "fastlio2",
        "mapping_source": "/points_raw + /imu_raw -> fastlio2 -> /slam/map_cloud",
        "frame_id": "odom",
        "created_at": "2026-05-23T00:00:00+00:00",
        "artifacts": {
            "map_pcd": {
                "path": "/tmp/map.pcd",
                "sha256": "map-sha",
                "source_profile": "sim_mujoco_live",
                "data_source": "mujoco_fastlio2_live",
                "slam_source": "fastlio2",
                "frame_id": "odom",
                "point_count": 10,
            },
            "tomogram": {
                "path": "/tmp/tomogram.pickle",
                "sha256": "tomo-sha",
                "source_map_sha256": "different-map-sha",
                "source_profile": "sim_mujoco_live",
                "data_source": "mujoco_fastlio2_live",
                "frame_id": "odom",
                "shape": [5, 1, 4, 4],
            },
        },
    }

    validation = validate_same_source_map_metadata(metadata)

    assert validation["ok"] is False
    assert (
        "metadata.artifacts.tomogram.source_map_sha256 does not match map_pcd.sha256"
        in validation["blockers"]
    )


def test_fastlio_live_gate_rejects_diverged_map_before_tomogram(tmp_path) -> None:
    from runtime.same_source_map_artifacts import (
        write_same_source_map_artifacts,
    )

    report = write_same_source_map_artifacts(
        artifact_dir=tmp_path / "same_source_map",
        points=np.array([[0.0, 0.0, 0.0], [2000.0, 0.0, 0.0]], dtype=np.float32),
        frame_id="odom",
        world=tmp_path / "industrial_demo_scene.xml",
        source_topics=("/slam/map_cloud",),
        mapping_input_path="/points_raw + /imu_raw -> fastlio2 -> /slam/map_cloud",
        build_tomogram=True,
        tomogram_resolution=0.2,
        tomogram_slice_dh=0.25,
        tomogram_ground_h=0.0,
        map_artifact_max_span_m=120.0,
        tomogram_max_cells=50_000_000,
    )

    assert report["ok"] is False
    assert any("map bounds span exceeds limit" in item for item in report["blockers"])
    assert "tomogram" not in report["assets"]


def test_fastlio_live_gate_uses_trackable_path_defaults_for_live_explore() -> None:
    stack = Path("src/drivers/sim/mujoco/stack.py")
    text = stack.read_text(encoding="utf-8")

    assert "local_planner_direct_track_fallback_min_distance_m=0.3" in text
    assert "local_planner_min_trackable_local_path_m=0.3" in text
    assert "stuck_timeout=max(45.0, float(frontier_goal_timeout) * 0.5)" in text
    assert "stuck_dist_thre=0.05" in text


def test_mujoco_tare_stack_reframes_cmu_waypoints_to_live_odom_contract() -> None:
    stack = Path("src/drivers/sim/mujoco/stack.py")
    text = stack.read_text(encoding="utf-8")
    endpoint_catalog = Path("src/runtime/profiles/catalog/endpoints.py").read_text(
        encoding="utf-8"
    )
    exploration_stack = Path("src/runtime/blueprints/stacks/exploration.py").read_text(
        encoding="utf-8"
    )

    assert "goal_frame_id=MUJOCO_LIVE_GOAL_FRAME_ID" in text
    assert "hold_active_goal_until_terminal=True" in text
    assert '"goal_frame_id"' in endpoint_catalog
    assert '"goal_frame_id"' in exploration_stack
    assert '"hold_active_goal_until_terminal"' in endpoint_catalog
    assert '"hold_active_goal_until_terminal"' in exploration_stack


def test_mujoco_inspection_stack_keeps_static_tomogram_saved_map_frame_contract() -> None:
    stack = Path("src/drivers/sim/mujoco/stack.py")
    text = stack.read_text(encoding="utf-8")
    nav_stack = Path("src/runtime/blueprints/stacks/navigation_core.py").read_text(
        encoding="utf-8"
    )

    assert "MUJOCO_LIVE_PLANNING_FRAME_ID = topic_default_frame_id(TOPICS.odometry)" in text
    assert (
        "MUJOCO_LIVE_SAVED_MAP_FRAME_ID = "
        "topic_default_frame_id(TOPICS.saved_map_cloud)"
    ) in text
    assert "expected_saved_map_frame_id=MUJOCO_LIVE_SAVED_MAP_FRAME_ID" in text
    assert '"expected_saved_map_frame_id"' in nav_stack


def test_fastlio_live_gate_reports_sim_realtime_factor() -> None:
    script = Path("sim/scripts/mujoco/live_gate.py")
    text = script.read_text(encoding="utf-8")

    assert '"elapsed_wall_s": round(float(sim_wall_time_s), 3)' in text
    assert '"elapsed_sim_s": round(float(elapsed_sim_s), 3)' in text
    assert '"sim_realtime_factor": (' in text


def test_fastlio_live_gate_uses_simulator_world_frame_contract() -> None:
    script = Path("sim/scripts/mujoco/live_gate.py")
    text = script.read_text(encoding="utf-8")

    assert "simulator_world_frame_id" in text
    assert "SIM_WORLD_FRAME_ID = simulator_world_frame_id()" in text
    assert "FRAMES.world" not in text
    assert "FRAMES.simulator_world" not in text


def test_fastlio_live_gate_can_run_duration_by_sim_time() -> None:
    script = Path("sim/scripts/mujoco/live_gate.py")
    text = script.read_text(encoding="utf-8")
    launcher = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(
        encoding="utf-8"
    )

    assert "--duration-clock" in text
    assert 'choices=["wall", "sim"]' in text
    assert '"duration_clock": duration_clock' in text
    assert (
        '"duration_clock": args.duration_clock' in text
        or "duration_clock=args.duration_clock" in text
    )
    assert "LINGTU_MUJOCO_LIVE_DURATION_CLOCK" in launcher
    assert '"--duration-clock" "$duration_clock"' in launcher
    assert "LINGTU_MUJOCO_LIVE_WORLD=industrial_park" in launcher
    assert 'LINGTU_MUJOCO_LIVE_WORLD:-industrial_park' in launcher


def test_mujoco_live_gate_defaults_to_finite_difference_imu() -> None:
    script = Path("sim/scripts/mujoco/live_gate.py")
    text = script.read_text(encoding="utf-8")
    launcher = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(
        encoding="utf-8"
    )

    assert 'default="finite_difference"' in text
    assert 'LINGTU_MUJOCO_LIVE_IMU_ACC_MODE=finite_difference' in launcher
    assert 'LINGTU_MUJOCO_LIVE_IMU_ACC_MODE:-finite_difference' in launcher


def test_fastlio_live_gate_uses_src_mujoco_sensors() -> None:
    script = Path("sim/scripts/mujoco_live/sensors.py")
    text = script.read_text(encoding="utf-8")

    assert "from drivers.sim.mujoco import sensors" in text
    assert "_MUJOCO_SENSORS" in text
    assert "return _mujoco_sensors().angle_delta_rad" in text
    assert "return _mujoco_sensors().specific_force_body" in text
    assert "return _mujoco_sensors().world_xyzi_to_sensor_xyzi" in text


def test_fastlio_live_gate_uses_core_msg_outputs_without_ros_bridge() -> None:
    gate = Path("sim/scripts/mujoco/live_gate.py").read_text(encoding="utf-8")
    report = Path("sim/scripts/mujoco_live/report.py").read_text(encoding="utf-8")
    text = gate + report

    assert "FastLio2NavBridgeRuntime" not in text
    assert 'node.create_subscription(Odometry, "/Odometry"' not in text
    assert 'node.create_subscription(PointCloud2, "/cloud_map"' not in text
    assert "canonical_nav_outputs_verified" in text
    assert "TOPICS.odometry" in text
    assert "TOPICS.registered_cloud" in text
    assert "TOPICS.map_cloud" in text


def test_fastlio_live_gate_uses_src_mujoco_runtime() -> None:
    script = Path("sim/scripts/mujoco/live_gate.py")
    text = script.read_text(encoding="utf-8")

    assert "from drivers.sim.mujoco.runtime import" in text
    assert "def _resolve_world(" not in text
    assert "def _scene_start(" not in text
    assert "def _parse_start(" not in text
    assert "def _scene_with_memory(" not in text
    assert "def _build_engine(" not in text


def test_fastlio_live_gate_does_not_import_unused_stack_builders() -> None:
    script = Path("sim/scripts/mujoco/live_gate.py")
    text = script.read_text(encoding="utf-8")
    stack = Path("src/drivers/sim/mujoco/stack.py").read_text(
        encoding="utf-8"
    )

    assert "from drivers.sim.mujoco.stack import" not in text
    assert "build_fastlio2_inspection_stack" not in text
    assert "build_fastlio2_frontier_stack(" not in text
    assert "build_fastlio2_tare_stack(" not in text
    assert "build_system_for_profile" in stack
    assert "occupancy_raycast_free_space=True" in stack
    assert 'robot="sim_endpoint"' in stack
    assert 'robot="sim_ros2"' not in stack
    assert "enable_nav_out=False" in stack
    assert "enable_ros2_path_bridge=True" not in stack
    assert 'exploration_backend="tare"' in stack
    assert "plan_safety_policy=\"reject\"" in stack


def test_fastlio_live_gate_reports_motion_path_length_and_command_integrals() -> None:
    script = Path("sim/scripts/mujoco/live_gate.py")
    text = script.read_text(encoding="utf-8")

    assert '"sim_path_length_m": round(float(sim_path_length_m), 4)' in text
    assert '"localization_path_length_m": round(float(odom_path_length_m), 4)' in text
    assert '"odom_path_length_m": round(float(odom_path_length_m), 4)' in text
    assert '"applied_command": dict(applied_cmd_stats)' in text


def test_load_mid360_csv_pattern_converts_zenith_to_elevation(tmp_path) -> None:
    csv_path = tmp_path / "mid360.csv"
    csv_path.write_text(
        "Time/s,Azimuth/deg,Zenith/deg\n"
        "1,0,90\n"
        "2,90,0\n"
        "3,180,180\n",
        encoding="utf-8",
    )

    angles = MuJoCoLidar._load_scan_mode_angles(str(csv_path))

    assert angles.shape == (3, 2)
    np.testing.assert_allclose(angles[:, 0], [0.0, np.pi / 2, np.pi], atol=1e-6)
    np.testing.assert_allclose(angles[:, 1], [0.0, np.pi / 2, -np.pi / 2], atol=1e-6)


def test_mid360_pattern_sampler_advances_and_wraps() -> None:
    lidar = MuJoCoLidar.__new__(MuJoCoLidar)
    lidar._config = SimpleNamespace(samples_per_frame=3)
    lidar._ray_angles = np.array(
        [
            [0.0, 0.0],
            [np.pi / 2, 0.0],
            [np.pi, 0.0],
            [3 * np.pi / 2, 0.0],
            [0.0, np.pi / 2],
        ],
        dtype=np.float32,
    )
    lidar._ray_cursor = 0

    first = lidar._next_pattern_dirs_local()
    second = lidar._next_pattern_dirs_local()

    np.testing.assert_allclose(first[:, :2], [[1, 0], [0, 1], [-1, 0]], atol=1e-6)
    np.testing.assert_allclose(second[0], [0, -1, 0], atol=1e-6)
    np.testing.assert_allclose(second[1], [0, 0, 1], atol=1e-6)
    np.testing.assert_allclose(second[2], [1, 0, 0], atol=1e-6)
    assert lidar._ray_cursor == 1


def test_lidar_return_model_uses_distance_weighted_intensity() -> None:
    lidar = MuJoCoLidar.__new__(MuJoCoLidar)
    lidar._config = SimpleNamespace(
        add_noise=False,
        range_min=0.1,
        range_max=70.0,
        pixel_dropout_prob=0.0,
        distance_dropout_prob_at_max=0.0,
        intensity_base=180.0,
        intensity_range_scale_m=25.0,
        intensity_noise_std=0.0,
        intensity_min=1.0,
        intensity_max=255.0,
        site_name="lidar_site",
    )
    lidar._rng = np.random.default_rng(0)
    lidar._mujoco_lidar = None
    lidar._data = None
    lidar._body_id = 0

    cloud = lidar._points_with_return_model(
        np.array([[1.0, 0.0, 0.0], [40.0, 0.0, 0.0]], dtype=np.float32)
    )

    assert cloud.shape == (2, 4)
    assert cloud[0, 3] > cloud[1, 3]
    assert not np.allclose(cloud[:, 3], 100.0)


def test_mid360_pattern_angle_noise_keeps_theta_wrapped() -> None:
    lidar = MuJoCoLidar.__new__(MuJoCoLidar)
    lidar._config = SimpleNamespace(
        samples_per_frame=4,
        add_noise=True,
        angle_noise_std_rad=0.01,
    )
    lidar._ray_angles = np.array([[0.0, 0.1], [6.2, 0.2]], dtype=np.float32)
    lidar._ray_cursor = 0
    lidar._rng = np.random.default_rng(0)

    theta, phi = lidar._next_pattern_angles()

    assert theta.shape == (4,)
    assert phi.shape == (4,)
    assert np.all(theta >= 0.0)
    assert np.all(theta <= 2.0 * np.pi)
