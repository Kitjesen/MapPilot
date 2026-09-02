import importlib.util
import json
import math
import sys
import types
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import pytest

from drivers.sim.mujoco.driver import MujocoDriverModule
from runtime.msgs.geometry import Twist, Vector3
from runtime.tests.numpy_guard import import_numpy_or_skip
from sim.compat.engine.core.robot import RobotConfig

pytestmark = [pytest.mark.sim]

np = import_numpy_or_skip()

_ROS2_AVAILABLE = importlib.util.find_spec("rclpy") is not None


def test_default_thunder_v4_resolves_current_robot_and_controller():
    sim_root = Path(__file__).resolve().parents[2] / "sim"
    cfg = RobotConfig.default_thunder_v4().resolve_paths(base_dir=str(sim_root))

    assert Path(cfg.robot_xml).as_posix().endswith("robots/doso/thunder_v4/mjcf/thunderv4.xml")
    assert Path(cfg.robot_xml).exists()
    assert Path(cfg.policy_onnx).name == "policy_1119.onnx"
    assert cfg.base_body_name == "base_link"
    assert cfg.lidar_body_name == "lidar_link"
    assert cfg.leg_act_offset == 0
    assert cfg.leg_joint_names[0] == "FR_hip_joint"


def test_thunderv4_model_exposes_lidar_site_imu_package():
    model_path = (
        Path(__file__).resolve().parents[2]
        / "sim" / "packages" / "robots"
        / "doso"
        / "thunder_v4"
        / "mjcf"
        / "thunderv4.xml"
    )
    root = ET.fromstring(model_path.read_text(encoding="utf-8"))
    sensors = {node.attrib.get("name"): node.attrib for node in root.findall(".//sensor/*")}

    for name in (
        "lidar-orientation",
        "lidar-position",
        "lidar-angular-velocity",
        "lidar-linear-velocity",
        "lidar-linear-acceleration",
    ):
        assert name in sensors

    assert sensors["lidar-angular-velocity"]["site"] == "lidar_site"
    assert sensors["lidar-linear-acceleration"]["site"] == "lidar_site"
    assert sensors["lidar-orientation"]["objname"] == "lidar_site"


def test_mujoco_engine_prefers_lidar_site_imu_for_raw_slam_package():
    from sim.compat.engine.mujoco.engine import MuJoCoEngine

    class FakeSensor:
        def __init__(self, data):
            self.data = np.asarray(data, dtype=np.float64)

    class FakeData:
        def __init__(self):
            self._sensors = {
                "orientation": FakeSensor([1.0, 0.0, 0.0, 0.0]),
                "angular-velocity": FakeSensor([9.0, 9.0, 9.0]),
                "linear-acceleration": FakeSensor([7.0, 8.0, 9.0]),
                "lidar-orientation": FakeSensor([1.0, 0.0, 0.0, 0.0]),
                "lidar-angular-velocity": FakeSensor([0.1, 0.2, 0.3]),
                "lidar-linear-acceleration": FakeSensor([1.0, 2.0, 9.8]),
            }

        def sensor(self, name):
            return self._sensors[name]

    engine = MuJoCoEngine(drive_mode="kinematic")
    engine._data = FakeData()

    gyro, projected_gravity = engine._get_sensor_imu()
    accel = engine._get_imu_linear_acceleration(projected_gravity)

    np.testing.assert_allclose(gyro, [0.1, 0.2, 0.3])
    np.testing.assert_allclose(projected_gravity, [0.0, 0.0, -1.0])
    np.testing.assert_allclose(accel, [1.0, 2.0, 9.8])


def test_default_thunder_v4_resolves_paths_from_engine_core_default():
    sim_root = Path(__file__).resolve().parents[2] / "sim"
    cfg = RobotConfig.default_thunder_v4().resolve_paths()

    assert Path(cfg.robot_xml).exists()
    assert Path(cfg.robot_xml).is_relative_to(sim_root)
    assert Path(cfg.robot_xml).name == "thunderv4.xml"
    assert "sim/sim" not in Path(cfg.robot_xml).as_posix()


def test_mujoco_driver_splits_body_lidar_cloud_from_world_map_cloud():
    from drivers.sim.mujoco.sensors import world_points_to_body_frame
    from runtime.runtime_interface import FRAMES

    pts = np.array([[0.0, 1.0, 0.0, 0.5]], dtype=np.float32)
    yaw_90_xyzw = np.array([0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)])

    body_pts = world_points_to_body_frame(pts, np.zeros(3), yaw_90_xyzw)

    assert body_pts[0, :3] == pytest.approx([1.0, 0.0, 0.0], abs=1e-6)
    assert FRAMES.body == "body"
    assert FRAMES.odom == "odom"


def test_mujoco_driver_builds_livox_raw_scan_frame():
    from drivers.sim.mujoco.driver import _xyzi_to_livox_frame

    pts = np.array(
        [[1.0, 2.0, 3.0, 42.0], [4.0, 5.0, 6.0, 84.0]],
        dtype=np.float32,
    )

    frame = _xyzi_to_livox_frame(
        pts,
        timestamp_ns=1_000_000_000,
        sequence=7,
        frame_id="lidar_link",
        scan_duration_ns=100_000_000,
    )

    assert frame.frame_id == "lidar_link"
    assert frame.sequence == 7
    assert frame.timestamp_ns == 1_000_000_000
    assert frame.point_count == 2
    assert frame.points["x"].tolist() == pytest.approx([1.0, 4.0])
    assert frame.points["intensity"].tolist() == pytest.approx([42.0, 84.0])
    assert frame.points["offset_time_ns"][0] == 0
    assert frame.points["offset_time_ns"][1] > 0
    assert frame.points["tag"].tolist() == [0x00, 0x00]
    assert frame.points["line"].tolist() == [0, 0]
    assert frame.to_xyzi() == pytest.approx(pts)


def test_mujoco_livox_frame_offsets_cover_scan_window():
    from drivers.sim.mujoco.driver import _xyzi_to_livox_frame

    pts = np.array(
        [[1.0, 0.0, 0.0, 10.0], [2.0, 0.0, 0.0, 20.0], [3.0, 0.0, 0.0, 30.0]],
        dtype=np.float32,
    )

    frame = _xyzi_to_livox_frame(
        pts,
        timestamp_ns=2_000_000_000,
        sequence=1,
        frame_id="lidar_link",
        scan_duration_ns=100_000_000,
    )

    assert frame.points["offset_time_ns"].tolist() == [0, 49_999_999, 99_999_999]
    assert frame.timestamp_ns + int(frame.points["offset_time_ns"][-1]) < 2_100_000_000


def test_mujoco_livox_frame_accepts_physical_offsets():
    from drivers.sim.mujoco.driver import _xyzi_to_livox_frame

    pts = np.array(
        [
            [1.0, 0.0, 0.0, 10.0],
            [2.0, 0.0, 0.0, 20.0],
            [3.0, 0.0, 0.0, 30.0],
            [4.0, 0.0, 0.0, 40.0],
            [5.0, 0.0, 0.0, 50.0],
        ],
        dtype=np.float32,
    )

    frame = _xyzi_to_livox_frame(
        pts,
        timestamp_ns=2_000_000_000,
        sequence=1,
        frame_id="lidar_link",
        scan_duration_ns=100_000_000,
        offset_time_ns=np.array([0, 20_000_000, 40_000_000, 60_000_000, 80_000_000], dtype=np.uint64),
    )

    assert frame.points["offset_time_ns"].tolist() == [0, 20_000_000, 40_000_000, 60_000_000, 80_000_000]
    assert frame.points["line"].tolist() == [0] * 5
    assert frame.points["tag"].tolist() == [0x00] * 5


def test_mujoco_livox_frame_accepts_explicit_lines_and_tags():
    from drivers.sim.mujoco.driver import _xyzi_to_livox_frame

    pts = np.array(
        [[1.0, 0.0, 0.0, 10.0], [2.0, 0.0, 0.0, 20.0], [3.0, 0.0, 0.0, 30.0]],
        dtype=np.float32,
    )

    frame = _xyzi_to_livox_frame(
        pts,
        timestamp_ns=2_000_000_000,
        sequence=1,
        frame_id="lidar_link",
        scan_duration_ns=100_000_000,
        line_ids=np.array([3, 1, 2], dtype=np.uint8),
        tag_values=np.array([0x10, 0x00, 0x10], dtype=np.uint8),
    )

    assert frame.points["line"].tolist() == [3, 1, 2]
    assert frame.points["tag"].tolist() == [0x10, 0x00, 0x10]


def test_mujoco_module_instantaneous_cloud_does_not_fake_rolling_offsets():
    source = (Path("src/drivers/sim/mujoco/driver.py")).read_text(encoding="utf-8")

    assert "timestamp_ns=int(ts * 1_000_000_000)" in source
    assert "scan_duration_ns=0" in source
    assert "ts - scan_duration_s" not in source


def test_mujoco_native_dds_sensor_bridge_lidar_imu_timebase_contract():
    import io

    from drivers.sim.mujoco.driver import _xyzi_to_livox_frame
    from runtime.msgs.geometry import Quaternion, Vector3
    from runtime.msgs.sensor import Imu
    from sim.scripts.mujoco import native_dds_sensors as bridge

    lidar_ts_ns = 2_000_000_000
    scan_duration_ns = 100_000_000
    frame = _xyzi_to_livox_frame(
        np.array([[1.0, 0.0, 0.0, 10.0], [2.0, 0.0, 0.0, 20.0]], dtype=np.float32),
        timestamp_ns=lidar_ts_ns,
        sequence=3,
        frame_id=bridge.LIDAR_FRAME_ID,
        scan_duration_ns=scan_duration_ns,
    )
    imu = Imu(
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.1, 0.2, 0.3),
        linear_acceleration=Vector3(0.0, 0.0, bridge._MID360_ACCEL_MPS2_PER_G),
        ts=(lidar_ts_ns + scan_duration_ns // 2) / 1_000_000_000.0,
        frame_id=bridge.IMU_FRAME_ID,
    )
    stream = io.BytesIO()

    bridge._write_native_imu(stream, imu, sequence=4)
    _, record_type, imu_ts_ns, sequence, count, _ = bridge._HEADER.unpack(stream.getvalue()[: bridge._HEADER.size])

    assert record_type == bridge._RECORD_IMU
    assert sequence == 4
    assert count == 1
    assert frame.timestamp_ns <= imu_ts_ns < frame.timestamp_ns + scan_duration_ns


def test_mujoco_native_dds_sensor_bridge_defaults_to_fastlio_only():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    args = bridge._build_parser().parse_args([])

    assert args.publish_odom_prior is False
    assert args.imu_hz == pytest.approx(200.0)
    assert args.imu_acc_mode == "sensor"
    assert args.imu_acc_conditioning == "realistic"


def test_mujoco_native_dds_sensor_bridge_writes_odom_prior_record():
    import io

    from sim.scripts.mujoco import native_dds_sensors as bridge

    state = types.SimpleNamespace(
        position=np.array([1.0, 2.0, 0.3], dtype=np.float64),
        orientation=np.array([0.0, 0.0, 0.5, 0.8660254], dtype=np.float64),
        linear_velocity=np.array([0.4, 0.0, 0.0], dtype=np.float64),
    )
    stream = io.BytesIO()

    bridge._write_native_odom_prior(stream, state, timestamp_s=12.5, sequence=9)
    header = stream.getvalue()[: bridge._HEADER.size]
    payload = stream.getvalue()[bridge._HEADER.size :]
    _, record_type, timestamp_ns, sequence, count, payload_bytes = bridge._HEADER.unpack(header)
    values = bridge._ODOM_PRIOR_PAYLOAD.unpack(payload)

    assert record_type == bridge._RECORD_ODOM_PRIOR
    assert timestamp_ns == 12_500_000_000
    assert sequence == 9
    assert count == 1
    assert payload_bytes == bridge._ODOM_PRIOR_PAYLOAD.size
    assert values[:10] == pytest.approx((1.0, 2.0, 0.3, 0.0, 0.0, 0.5, 0.8660254, 0.4, 0.0, 0.0))
    assert values[10] == 1


def test_mujoco_native_dds_sensor_bridge_writes_body_registered_cloud_record():
    import io

    from sim.scripts.mujoco import native_dds_sensors as bridge

    points = np.array(
        [[1.0, 2.0, 0.3, 11.0], [-0.5, 0.25, 1.2, 42.0]],
        dtype=np.float32,
    )
    stream = io.BytesIO()

    bridge._write_native_registered_cloud(
        stream,
        points,
        timestamp_ns=12_500_000_000,
        sequence=10,
    )
    header = stream.getvalue()[: bridge._HEADER.size]
    payload = stream.getvalue()[bridge._HEADER.size :]
    _, record_type, timestamp_ns, sequence, count, payload_bytes = bridge._HEADER.unpack(header)
    records = np.frombuffer(payload, dtype=bridge.POINT_DTYPE)

    assert record_type == bridge._RECORD_REGISTERED_CLOUD
    assert timestamp_ns == 12_500_000_000
    assert sequence == 10
    assert count == 2
    assert payload_bytes == 2 * bridge.POINT_DTYPE.itemsize
    np.testing.assert_allclose(records["x"], [1.0, -0.5])
    np.testing.assert_allclose(records["y"], [2.0, 0.25])
    np.testing.assert_allclose(records["z"], [0.3, 1.2])
    np.testing.assert_allclose(records["intensity"], [11.0, 42.0])


def test_mujoco_navigation_fixture_cloud_uses_current_body_frame():
    from drivers.sim.mujoco import sensors as sensor_helpers

    yaw = math.pi / 2.0
    state = types.SimpleNamespace(
        position=np.array([10.0, 20.0, 0.5], dtype=np.float64),
        orientation=np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]),
    )
    world = np.array([[10.0, 21.0, 0.5, 100.0]], dtype=np.float32)

    body = sensor_helpers.world_xyzi_to_body_xyzi(state, world)

    np.testing.assert_allclose(body[0], [1.0, 0.0, 0.0, 100.0], atol=1e-6)


def test_navigation_fixture_ground_points_are_deterministic_body_frame_grid():
    from drivers.sim.mujoco import sensors as sensor_helpers

    first = sensor_helpers.navigation_fixture_ground_body_points(0.42)
    second = sensor_helpers.navigation_fixture_ground_body_points(0.42)

    resolution = 0.2
    expected_x = np.arange(-1.0, 4.0 + resolution * 0.5, resolution)
    expected_y = np.arange(
        -1.6,
        1.6 + resolution * 0.5,
        resolution,
    )

    np.testing.assert_array_equal(first, second)
    assert first.shape == (expected_x.size * expected_y.size, 4)
    assert float(first[:, 0].min()) == pytest.approx(-1.0)
    assert float(first[:, 0].max()) == pytest.approx(4.0)
    assert float(first[:, 1].min()) == pytest.approx(-1.6)
    assert float(first[:, 1].max()) == pytest.approx(1.6)
    assert np.all(np.diff(np.unique(first[:, 0])) <= resolution + 1e-6)
    assert np.all(np.diff(np.unique(first[:, 1])) <= resolution + 1e-6)
    np.testing.assert_allclose(first[:, 2], -0.42, atol=1e-6)


def test_navigation_fixture_ground_and_raw_world_ground_remain_coplanar_when_tilted():
    from drivers.sim.mujoco import sensors as sensor_helpers

    roll = math.radians(15.0)
    pitch = math.radians(-10.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    state = types.SimpleNamespace(
        position=np.array([3.0, -2.0, 0.42], dtype=np.float64),
        orientation=np.array([sr * cp, cr * sp, -sr * sp, cr * cp], dtype=np.float64),
    )
    raw_world = np.array(
        [[4.5, -1.25, 0.0, 77.0], [4.5, -1.25, 0.2, 88.0]],
        dtype=np.float32,
    )
    raw_body = sensor_helpers.world_xyzi_to_body_xyzi(
        state,
        raw_world,
    )
    ground_count = sensor_helpers.navigation_fixture_ground_body_points(0.42).shape[0]

    combined, stats = sensor_helpers.navigation_fixture_registered_body_points(
        raw_body,
        state,
        max_points=ground_count + 2,
    )

    rotation_body_to_world = sensor_helpers.quat_xyzw_to_matrix(state.orientation)
    synthetic_world = (
        combined[:ground_count, :3].astype(np.float64) @ rotation_body_to_world.T
        + state.position
    )
    np.testing.assert_allclose(synthetic_world[:, 2], 0.0, atol=1e-6)
    assert stats["raw_obstacle_overlay_points"] == 1
    assert stats["raw_context_overlay_points"] == 1
    np.testing.assert_allclose(combined[-2:], raw_body, atol=1e-6)


def test_navigation_fixture_ground_helper_rejects_too_coarse_resolution():
    from drivers.sim.mujoco import sensors as sensor_helpers

    with pytest.raises(ValueError, match="resolution"):
        sensor_helpers.navigation_fixture_ground_body_points(
            0.42,
            resolution_m=0.25,
        )


def test_navigation_fixture_ground_width_can_match_a_wider_scene():
    from drivers.sim.mujoco import sensors as sensor_helpers

    ground = sensor_helpers.navigation_fixture_ground_body_points(
        0.42,
        y_half_m=1.8,
    )

    assert float(ground[:, 1].min()) == pytest.approx(-1.8)
    assert float(ground[:, 1].max()) == pytest.approx(1.8)


def test_navigation_fixture_registered_cloud_preserves_ground_when_budgeted():
    from drivers.sim.mujoco import sensors as sensor_helpers

    state = types.SimpleNamespace(position=np.array([0.0, 0.0, 0.42], dtype=np.float64))
    raw = np.array([[5.0, 5.0, 0.1, 77.0]], dtype=np.float32)
    ground_count = sensor_helpers.navigation_fixture_ground_body_points(0.42).shape[0]

    combined, stats = sensor_helpers.navigation_fixture_registered_body_points(
        raw,
        state,
        max_points=ground_count,
    )

    assert combined.shape[0] == ground_count
    assert stats["synthetic_ground_points"] == ground_count
    assert stats["raw_overlay_points"] == 0
    np.testing.assert_allclose(combined[:, 2], -0.42, atol=1e-6)


def test_navigation_fixture_registered_cloud_preserves_raw_obstacle_overlay():
    from drivers.sim.mujoco import sensors as sensor_helpers

    state = types.SimpleNamespace(position=np.array([0.0, 0.0, 0.42], dtype=np.float64))
    ground_count = sensor_helpers.navigation_fixture_ground_body_points(0.42).shape[0]
    obstacle = np.array([[1.1, -0.2, 0.55, 99.0]], dtype=np.float32)
    low_context = np.array(
        [[float(index), 2.0, -0.42, 10.0] for index in range(8)],
        dtype=np.float32,
    )
    raw = np.vstack([low_context, obstacle])

    combined, stats = sensor_helpers.navigation_fixture_registered_body_points(
        raw,
        state,
        max_points=ground_count + 1,
    )

    assert combined.shape[0] == ground_count + 1
    assert stats["raw_obstacle_overlay_points"] == 1
    assert stats["raw_context_overlay_points"] == 0
    np.testing.assert_allclose(combined[-1], obstacle[0], atol=1e-6)


def test_navigation_fixture_registered_cloud_can_disable_raw_overlay_for_free_space():
    from drivers.sim.mujoco import sensors as sensor_helpers

    state = types.SimpleNamespace(position=np.array([0.0, 0.0, 0.42], dtype=np.float64))
    raw = np.array(
        [[0.2, 0.1, 0.55, 99.0], [0.4, -0.2, -0.42, 13.0]],
        dtype=np.float32,
    )
    ground_count = sensor_helpers.navigation_fixture_ground_body_points(0.42).shape[0]

    combined, stats = sensor_helpers.navigation_fixture_registered_body_points(
        raw,
        state,
        max_points=ground_count + len(raw),
        raw_overlay_enabled=False,
    )

    assert combined.shape[0] == ground_count
    assert stats["raw_body_points"] == len(raw)
    assert stats["raw_overlay_enabled"] is False
    assert stats["raw_overlay_points"] == 0


def test_navigation_fixture_registered_cloud_keeps_raw_points_with_full_budget():
    from drivers.sim.mujoco import sensors as sensor_helpers

    state = types.SimpleNamespace(position=np.array([0.0, 0.0, 0.42], dtype=np.float64))
    ground_count = sensor_helpers.navigation_fixture_ground_body_points(0.42).shape[0]
    raw = np.array(
        [[0.4, 0.0, -0.42, 13.0], [1.2, 0.3, 0.7, 88.0]],
        dtype=np.float32,
    )

    combined, stats = sensor_helpers.navigation_fixture_registered_body_points(
        raw,
        state,
        max_points=ground_count + len(raw),
    )

    assert stats["raw_overlay_points"] == len(raw)
    np.testing.assert_allclose(combined[-len(raw) :], raw, atol=1e-6)


def test_mujoco_native_dds_odom_prior_velocity_uses_pose_window_not_qvel_impulse():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    estimator = bridge.OdomPriorVelocityEstimator(window_s=0.10)
    velocity = None
    valid = False
    for index in range(21):
        stamp_s = index * 0.005
        position = np.array([0.4 * stamp_s, -0.1 * stamp_s, 0.4])
        velocity, valid = estimator.update(position, stamp_s)

    assert valid
    np.testing.assert_allclose(velocity, [0.4, -0.1, 0.0], atol=1e-9)


def test_mujoco_native_dds_odom_prior_velocity_rejects_single_pose_solver_bounce():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    estimator = bridge.OdomPriorVelocityEstimator(window_s=0.10)
    bounce_velocity = None
    for index in range(21):
        stamp_s = index * 0.005
        position = np.array([0.4 * stamp_s, -0.1 * stamp_s, 0.4])
        if index == 16:
            position[:2] += [0.5, -0.4]
        velocity, valid = estimator.update(position, stamp_s)
        if index == 16:
            assert valid
            bounce_velocity = velocity

    np.testing.assert_allclose(bounce_velocity, [0.4, -0.1, 0.0], atol=1e-9)


def test_mujoco_native_dds_odom_prior_velocity_preserves_real_pose_jump():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    estimator = bridge.OdomPriorVelocityEstimator(window_s=0.10)
    estimator.update(np.array([0.0, 0.0, 0.4]), 0.0)
    velocity, valid = estimator.update(np.array([0.5, 0.0, 0.4]), 0.10)

    assert valid
    assert velocity[0] == pytest.approx(5.0)


def test_mujoco_native_dds_runtime_stage_profiler_keeps_ranked_slow_evidence():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    profiler = bridge.RuntimeStageProfiler(slow_threshold_s=0.05, max_slow_events=2)
    profiler.record("physics", 0.01, 1.0)
    profiler.record("lidar_subscan", 0.20, 2.0)
    profiler.record("pacing_sleep", 0.08, 3.0)
    profiler.record("physics", 0.10, 4.0)

    stats = profiler.stats()
    assert stats["stages"]["physics"]["count"] == 2
    assert stats["stages"]["physics"]["max_s"] == pytest.approx(0.10)
    assert stats["stages"]["physics"]["max_sim_time_s"] == pytest.approx(4.0)
    assert [event["stage"] for event in stats["slow_events"]] == [
        "lidar_subscan",
        "physics",
    ]


def test_mujoco_native_dds_odom_prior_contract_is_wired():
    from sim.scripts.mujoco import native_dds_sensors as bridge
    from sim.scripts.mujoco import native_sensor_records

    bridge_source = Path("sim/scripts/mujoco/native_dds_sensors.py").read_text(encoding="utf-8")
    sdk_source = Path("src/drivers/real/lidar/sdk2_stream/main.cpp").read_text(encoding="utf-8")
    runtime_source = Path("src/localization/slam/cpp/cyclone_runtime.cpp").read_text(encoding="utf-8")
    fastlio_source = Path("src/localization/slam/cpp/fastlio.cpp").read_text(encoding="utf-8")
    config_source = Path("src/localization/fastlio2/config/sim_mid360_slam.yaml").read_text(encoding="utf-8")
    topics_source = Path("src/message/cpp/topics.hpp").read_text(encoding="utf-8")

    assert native_sensor_records.RECORD_ODOM_PRIOR == 3
    assert bridge._RECORD_ODOM_PRIOR == native_sensor_records.RECORD_ODOM_PRIOR
    assert "_RECORD_ODOM_PRIOR = _sensor_records.RECORD_ODOM_PRIOR" in bridge_source
    assert "sensor_counts[TOPICS.odom_prior]" in bridge_source
    assert "kRecordOdomPrior = 3" in sdk_source
    assert "publish_odom_prior" in sdk_source
    assert "drainOdomPrior" in runtime_source
    assert "tracking_with_odom_prior" in fastlio_source
    assert "odom_prior_enabled: false" in config_source
    assert "kSlamOdomPrior" in topics_source
    assert "rt/slam/odom_prior" in topics_source


def test_mujoco_native_dds_sensor_bridge_flags_slam_motion_mismatch():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    motion = {
        "available": True,
        "sim_xy_m": 4.0,
        "slam_odom_xy_m": 0.05,
    }

    gaps = bridge._slam_motion_gaps(
        motion,
        min_sim_motion_for_odom_check_m=0.5,
        min_slam_motion_ratio=0.20,
        max_slam_motion_ratio=3.0,
    )

    assert gaps == ["native_slam_motion_mismatch:sim_xy=4.000,slam_xy=0.050,min_slam_xy=0.800"]


def test_mujoco_native_dds_sensor_bridge_accepts_slam_motion_ratio():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    motion = {
        "available": True,
        "sim_xy_m": 4.0,
        "slam_odom_xy_m": 1.0,
    }

    assert (
        bridge._slam_motion_gaps(
            motion,
            min_sim_motion_for_odom_check_m=0.5,
            min_slam_motion_ratio=0.20,
            max_slam_motion_ratio=3.0,
        )
        == []
    )


def test_mujoco_native_dds_motion_report_uses_delta_for_odom_prior():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    motion = bridge._motion_report(
        sim_start_position=np.array([3.0, 4.0, 0.5]),
        sim_end_position=np.array([4.0, 4.0, 0.5]),
        sim_start_yaw=0.5,
        sim_end_yaw=0.7,
        slam_status={
            "odom_prior_enabled": True,
            "odometry": {
                "pose": {
                    "x": 4.0,
                    "y": 4.0,
                    "z": 0.5,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": math.sin(0.7 / 2.0),
                    "qw": math.cos(0.7 / 2.0),
                }
            },
        },
        min_sim_motion_for_odom_check_m=0.5,
        min_slam_motion_ratio=0.5,
        max_slam_motion_ratio=1.6,
        min_sim_yaw_for_odom_check_rad=0.1,
        max_slam_yaw_error_rad=0.15,
    )

    assert motion["sim_xy_m"] == pytest.approx(1.0)
    assert motion["slam_odom_xy_m"] == pytest.approx(1.0)
    assert motion["slam_to_sim_xy_ratio"] == pytest.approx(1.0)
    assert motion["slam_to_sim_yaw_error_rad"] == pytest.approx(0.0, abs=1e-6)


def test_mujoco_native_dds_sensor_bridge_flags_slam_motion_overshoot():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    motion = {
        "available": True,
        "sim_xy_m": 2.0,
        "slam_odom_xy_m": 9.0,
    }

    gaps = bridge._slam_motion_gaps(
        motion,
        min_sim_motion_for_odom_check_m=0.5,
        min_slam_motion_ratio=0.20,
        max_slam_motion_ratio=3.0,
    )

    assert gaps == ["native_slam_motion_overshoot:sim_xy=2.000,slam_xy=9.000,max_slam_xy=6.000"]


def test_mujoco_native_dds_sensor_bridge_uses_map_pose_when_tracking_is_valid():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    motion = {
        "available": True,
        "sim_xy_m": 2.0,
        "slam_odom_xy_m": 9.0,
        "slam_map_xy_error_m": 0.12,
        "track_against_map": {
            "enabled": True,
            "successes": 4,
            "consecutive_failures": 0,
        },
    }

    assert (
        bridge._slam_motion_gaps(
            motion,
            min_sim_motion_for_odom_check_m=0.5,
            min_slam_motion_ratio=0.2,
            max_slam_motion_ratio=3.0,
            max_slam_map_xy_error_m=0.35,
        )
        == []
    )


def test_mujoco_native_dds_sensor_bridge_rejects_inaccurate_map_pose():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    motion = {
        "available": True,
        "sim_xy_m": 2.0,
        "slam_odom_xy_m": 2.0,
        "slam_map_xy_error_m": 0.62,
        "track_against_map": {
            "enabled": True,
            "successes": 4,
            "consecutive_failures": 0,
        },
    }

    assert bridge._slam_motion_gaps(
        motion,
        min_sim_motion_for_odom_check_m=0.5,
        min_slam_motion_ratio=0.2,
        max_slam_motion_ratio=3.0,
        max_slam_map_xy_error_m=0.35,
    ) == ["native_slam_map_pose_mismatch:xy_error=0.620,max_error=0.350"]


def test_mujoco_native_dds_sensor_bridge_rejects_inaccurate_map_yaw():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    motion = {
        "available": True,
        "sim_xy_m": 2.0,
        "slam_odom_xy_m": 2.0,
        "slam_map_xy_error_m": 0.10,
        "slam_map_yaw_error_rad": 0.25,
        "track_against_map": {
            "enabled": True,
            "successes": 4,
            "consecutive_failures": 0,
        },
    }

    assert bridge._slam_motion_gaps(
        motion,
        min_sim_motion_for_odom_check_m=0.5,
        min_slam_motion_ratio=0.2,
        max_slam_motion_ratio=3.0,
        max_slam_yaw_error_rad=0.15,
        max_slam_map_xy_error_m=0.35,
    ) == ["native_slam_map_yaw_mismatch:yaw_error=0.250,max_error=0.150"]


def test_mujoco_native_dds_sensor_bridge_flags_slam_yaw_mismatch():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    motion = {
        "available": True,
        "sim_xy_m": 2.0,
        "slam_odom_xy_m": 2.1,
        "sim_yaw_delta_rad": 0.85,
        "slam_odom_yaw_rad": 1.10,
        "slam_to_sim_yaw_error_rad": 0.25,
    }

    gaps = bridge._slam_motion_gaps(
        motion,
        min_sim_motion_for_odom_check_m=0.5,
        min_slam_motion_ratio=0.5,
        max_slam_motion_ratio=1.6,
        min_sim_yaw_for_odom_check_rad=0.2,
        max_slam_yaw_error_rad=0.15,
    )

    assert gaps == ["native_slam_yaw_mismatch:sim_yaw=0.850,slam_yaw=1.100,yaw_error=0.250,max_error=0.150"]


def test_mujoco_native_dds_sensor_bridge_defaults_to_physical_rolling_scan_time():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    args = bridge._build_parser().parse_args([])

    assert args.scan_time_profile == "physical_rolling"
    assert args.physical_rolling_sample_mode == "subscan"
    assert args.timestamp_clock == "sim_hardware"
    assert args.imu_timestamp_clock == ""
    assert args.lidar_timestamp_clock == ""
    assert args.sim_hardware_realtime_factor == pytest.approx(1.0)
    assert args.settle_s == pytest.approx(3.0)
    assert args.warmup_s == pytest.approx(2.0)
    assert args.drive_ramp_s == pytest.approx(5.0)
    assert args.drive_mode == "policy"
    assert args.drive_profile == "arc"
    assert args.imu_acc_mode == "sensor"
    assert args.imu_acc_conditioning == "realistic"
    assert args.imu_acc_lowpass_hz == pytest.approx(5.0)
    assert args.imu_acc_max_dynamic_mps2 == pytest.approx(1.5)
    assert args.imu_acc_max_slew_mps3 == pytest.approx(30.0)
    assert args.imu_acc_axis_scale == "auto"
    assert args.imu_gyro_axis_scale == "1,1,1"
    assert args.allow_kinematic_fastlio_acceptance is False
    assert args.min_slam_motion_ratio == pytest.approx(0.5)
    assert args.max_slam_motion_ratio == pytest.approx(1.6)
    assert args.max_slam_map_xy_error_m == pytest.approx(0.35)
    assert args.min_sim_yaw_for_odom_check_rad == pytest.approx(0.2)
    assert args.max_slam_yaw_error_rad == pytest.approx(0.15)


def test_mujoco_native_dds_sensor_bridge_drive_profiles_are_reproducible():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    assert bridge._drive_command_for_profile(
        "arc",
        99.0,
        drive_vx=0.12,
        drive_vy=0.01,
        drive_wz=0.04,
    ) == pytest.approx((0.12, 0.01, 0.04))
    assert bridge._drive_command_for_profile(
        "box_explore",
        1.0,
        drive_vx=0.08,
        drive_vy=0.02,
        drive_wz=0.04,
    ) == pytest.approx((0.10, 0.0, 0.0))
    assert bridge._drive_command_for_profile(
        "box_explore",
        8.0,
        drive_vx=0.08,
        drive_vy=0.02,
        drive_wz=0.04,
    ) == pytest.approx((0.0, 0.0, 0.35))


def test_mujoco_native_dds_sensor_bridge_policy_default_prefers_onnx():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    policy_path = bridge._resolve_policy_path_for_drive("policy", "")

    assert policy_path is not None
    assert policy_path.name.endswith(".onnx")
    assert bridge._resolve_policy_path_for_drive("kinematic", "") is None


def test_mujoco_native_dds_sensor_bridge_accepts_split_timestamp_clocks():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    args = bridge._build_parser().parse_args(
        [
            "--timestamp-clock",
            "wall",
            "--imu-timestamp-clock",
            "sim",
            "--lidar-timestamp-clock",
            "wall",
        ]
    )

    assert args.timestamp_clock == "wall"
    assert args.imu_timestamp_clock == "sim"
    assert args.lidar_timestamp_clock == "wall"


def test_mujoco_native_dds_sensor_bridge_uses_unified_sim_hardware_clock():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    clock = bridge.SimulatedHardwareClock(
        sim_start_s=10.0,
        wall_epoch_s=1000.0,
        monotonic_start_s=2000.0,
        realtime_factor=2.0,
    )

    assert clock.timestamp_s(12.5) == pytest.approx(1002.5)
    assert clock.monotonic_deadline_s(12.5) == pytest.approx(2001.25)
    assert clock.expected_sim_time_s(2001.25) == pytest.approx(12.5)
    assert clock.lag_s(12.0, 2001.25) == pytest.approx(0.5)
    assert bridge._sensor_timestamp_s(
        clock="sim_hardware",
        sim_clock_epoch_s=clock.wall_epoch_s,
        sim_time_s=12.5,
        wall_time_s=3000.0,
    ) == pytest.approx(1002.5)
    assert bridge._scan_start_timestamp_s(
        clock="sim_hardware",
        sim_clock_epoch_s=clock.wall_epoch_s,
        scan_start_sim_s=12.4,
        wall_scan_start_s=2999.9,
    ) == pytest.approx(1002.4)
    assert bridge._uses_unified_sim_hardware_clock(
        timestamp_clock="sim_hardware",
        imu_clock="sim_hardware",
        lidar_clock="sim_hardware",
    )


def test_mujoco_native_dds_clock_alignment_selects_lowest_rtt_sample():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    second = 1_000_000_000
    result = bridge._select_native_wall_clock_alignment(
        [
            (100 * second, 105 * second + 40_000_000, 100 * second + 80_000_000),
            (200 * second, 205 * second + 2_000_000, 200 * second + 4_000_000),
            (300 * second, 305 * second + 10_000_000, 300 * second + 20_000_000),
        ]
    )

    assert result["sample_count"] == 3
    assert result["rtt_ms"] == pytest.approx(4.0)
    assert result["uncertainty_ms"] == pytest.approx(2.0)
    assert result["native_minus_local_s"] == pytest.approx(5.0)


def test_mujoco_native_dds_clock_alignment_rejects_high_uncertainty():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    with pytest.raises(RuntimeError, match="uncertainty too high"):
        bridge._select_native_wall_clock_alignment(
            [(1_000_000_000, 2_000_000_000, 1_250_000_000)]
        )


def test_mujoco_native_dds_clock_handshake_excludes_wsl_startup(monkeypatch):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    native_offset_ns = 5_000_000_000
    local_times = iter(
        value
        for sample in range(bridge.DEFAULT_NATIVE_CLOCK_SYNC_SAMPLES)
        for value in (1_000_000_000 + sample * 10_000, 1_000_001_000 + sample * 10_000)
    )
    native_samples = [
        1_000_000_500 + sample * 10_000 + native_offset_ns
        for sample in range(bridge.DEFAULT_NATIVE_CLOCK_SYNC_SAMPLES)
    ]
    stdout = types.SimpleNamespace()
    response_lines = iter(
        [b"LINGTU_CLOCK_READY\n", *[f"{value}\n".encode("ascii") for value in native_samples]]
    )
    stdout.readline = lambda: next(response_lines, b"")
    stdout.close = lambda: None
    stdin = types.SimpleNamespace(writes=[])
    stdin.write = lambda value: stdin.writes.append(value)
    stdin.flush = lambda: None
    process = types.SimpleNamespace(stdin=stdin, stdout=stdout)
    monkeypatch.setattr(bridge.time, "time_ns", lambda: next(local_times))

    alignment = bridge._synchronize_managed_native_clock(process)

    assert alignment["source"] == "managed_native_process_handshake"
    assert alignment["sample_count"] == bridge.DEFAULT_NATIVE_CLOCK_SYNC_SAMPLES
    assert alignment["rtt_ms"] == pytest.approx(0.001)
    assert alignment["native_minus_local_s"] == pytest.approx(5.0)
    assert stdin.writes == [
        *[b"LINGTU_CLOCK_SAMPLE\n"] * bridge.DEFAULT_NATIVE_CLOCK_SYNC_SAMPLES,
        b"LINGTU_CLOCK_START\n",
    ]


def test_mujoco_native_dds_managed_wsl_clock_handshake_is_opt_in(tmp_path):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    command = ["wsl.exe", "-e", "/tmp/livox_sdk2_stream", "--dds"]
    plain = bridge._managed_wsl_command(command, tmp_path / "plain.pid")
    synchronized = bridge._managed_wsl_command(
        command,
        tmp_path / "synchronized.pid",
        clock_handshake=True,
    )

    assert "LINGTU_CLOCK_READY" not in plain[4]
    assert "LINGTU_CLOCK_READY" in synchronized[4]
    assert "LINGTU_CLOCK_SAMPLE" in synchronized[4]
    assert "LINGTU_CLOCK_START" in synchronized[4]


def test_mujoco_native_dds_publisher_preserves_unified_sim_hardware_clock():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    unified = bridge._build_parser().parse_args([])
    wall = bridge._build_parser().parse_args(["--timestamp-clock", "wall"])
    split = bridge._build_parser().parse_args(
        ["--imu-timestamp-clock", "wall"]
    )
    fixture = bridge._build_parser().parse_args(
        ["--navigation-fixture", "--timestamp-clock", "wall"]
    )

    assert bridge._should_restamp_native_records(unified) is False
    assert bridge._should_restamp_native_records(wall) is True
    assert bridge._should_restamp_native_records(split) is True
    assert bridge._should_restamp_native_records(fixture) is False


def test_mujoco_native_dds_external_arm_cli_contract(tmp_path):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    arm_file = tmp_path / "sensor_arm.json"
    status_file = tmp_path / "sensor_arm_status.json"
    args = bridge._build_parser().parse_args(
        [
            "--domain-id",
            "83",
            "--external-arm-file",
            str(arm_file),
            "--external-arm-token",
            "run-token",
            "--external-arm-scenario",
            "free",
            "--external-arm-timeout-s",
            "12.5",
            "--external-arm-status-json",
            str(status_file),
        ]
    )

    config = bridge._external_arm_config_from_args(args)

    assert config is not None
    assert config["arm_file"] == arm_file.resolve()
    assert config["status_json"] == status_file.resolve()
    assert config["token"] == "run-token"
    assert config["domain_id"] == 83
    assert config["scenario"] == "free"
    assert config["timeout_s"] == pytest.approx(12.5)
    sibling = tmp_path / "unrelated.json"
    arm_file.write_text('{"stale":true}', encoding="utf-8")
    status_file.write_text('{"state":"armed"}', encoding="utf-8")
    sibling.write_text("keep", encoding="utf-8")
    bridge._prepare_external_arm_files(config)
    assert not arm_file.exists()
    assert not status_file.exists()
    assert sibling.read_text(encoding="utf-8") == "keep"

    assert bridge._external_arm_config_from_args(bridge._build_parser().parse_args([])) is None

    missing_token = bridge._build_parser().parse_args(
        ["--external-arm-file", str(arm_file), "--external-arm-scenario", "free"]
    )
    with pytest.raises(ValueError, match="external-arm-token"):
        bridge._external_arm_config_from_args(missing_token)


def test_mujoco_native_dds_atomic_status_write_retries_windows_sharing_violation(
    tmp_path,
    monkeypatch,
):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    target = tmp_path / "status.json"
    real_replace = bridge.os.replace
    attempts = 0

    def replace(source, destination):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            error = PermissionError("sharing violation")
            error.winerror = 32
            raise error
        real_replace(source, destination)

    monkeypatch.setattr(bridge.os, "replace", replace)
    monkeypatch.setattr(bridge.time, "sleep", lambda _seconds: None)

    bridge._write_atomic_json_object(target, {"state": "armed"})

    assert attempts == 2
    assert json.loads(target.read_text(encoding="utf-8")) == {"state": "armed"}
    assert not list(tmp_path.glob(".*.tmp"))


def test_mujoco_native_dds_external_arm_rejects_contract_mismatches():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    valid = {
        "schema": bridge.EXTERNAL_ARM_SCHEMA,
        "arm": True,
        "token": "run-token",
        "domain_id": 83,
        "scenario": "free",
    }
    assert bridge._validate_external_arm_payload(
        valid,
        expected_token="run-token",
        expected_domain_id=83,
        expected_scenario="free",
    ) == ""

    cases = []
    for field, value, error in (
        ("schema", "old", "external_arm_schema_invalid"),
        ("arm", False, "external_arm_value_invalid"),
        ("token", "wrong", "external_arm_token_mismatch"),
        ("domain_id", 84, "external_arm_domain_mismatch"),
        ("domain_id", True, "external_arm_domain_mismatch"),
        ("scenario", "obstacle_stop", "external_arm_scenario_mismatch"),
    ):
        payload = dict(valid)
        payload[field] = value
        cases.append((payload, error))
    payload_with_extra_key = dict(valid)
    payload_with_extra_key["unexpected"] = True
    cases.append((payload_with_extra_key, "external_arm_keys_invalid"))

    for payload, expected_error in cases:
        assert bridge._validate_external_arm_payload(
            payload,
            expected_token="run-token",
            expected_domain_id=83,
            expected_scenario="free",
        ) == expected_error


def test_mujoco_native_dds_external_arm_uses_strict_bounded_json():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    with pytest.raises(ValueError, match="duplicate_key"):
        bridge._strict_json_object(b'{"schema":"a","schema":"b"}')
    with pytest.raises(ValueError, match="constant_invalid"):
        bridge._strict_json_object(b'{"value":NaN}')
    with pytest.raises(ValueError, match="object_required"):
        bridge._strict_json_object(b"[]")
    with pytest.raises(ValueError, match="size_invalid"):
        bridge._strict_json_object(b"x" * (bridge._EXTERNAL_ARM_MAX_BYTES + 1))


def test_mujoco_native_dds_external_arm_ack_starts_sim_duration_and_releases_anchor(tmp_path):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    arm_file = tmp_path / "sensor_arm.json"
    status_file = tmp_path / "sensor_arm_status.json"
    gate = bridge.ExternalArmGate(
        arm_file=arm_file,
        token="run-token",
        domain_id=83,
        scenario="free",
        timeout_s=10.0,
        status_json=status_file,
        started_wall_s=100.0,
    )

    initial_status = json.loads(status_file.read_text(encoding="utf-8"))
    assert initial_status["state"] == "waiting"
    assert initial_status["acknowledged"] is False
    assert bridge._sensor_anchor_active(
        "off",
        motion_started=False,
        external_arm_gate=gate,
    ) is True
    assert gate.poll(sim_time_s=11.0, monotonic_now_s=100.1) == "waiting"
    assert bridge._external_arm_drive_elapsed_s(gate, sim_time_s=50.0) == 0.0

    bridge._write_atomic_json_object(
        arm_file,
        {
            "schema": bridge.EXTERNAL_ARM_SCHEMA,
            "arm": True,
            "token": "run-token",
            "domain_id": 83,
            "scenario": "free",
        },
    )
    assert gate.poll(sim_time_s=12.5, monotonic_now_s=100.2) == "armed"

    armed_status = json.loads(status_file.read_text(encoding="utf-8"))
    assert armed_status["schema"] == bridge.EXTERNAL_ARM_STATUS_SCHEMA
    assert armed_status["state"] == "armed"
    assert armed_status["acknowledged"] is True
    assert armed_status["domain_id"] == 83
    assert armed_status["scenario"] == "free"
    assert armed_status["duration_clock"] == "sim"
    assert armed_status["arm_observed_sim_time_s"] == pytest.approx(12.5)
    assert "token_sha256_12" not in armed_status
    assert "run-token" not in status_file.read_text(encoding="utf-8")
    assert "run-token" not in json.dumps(gate.snapshot(), sort_keys=True)
    assert bridge._external_arm_drive_elapsed_s(gate, sim_time_s=12.5) == 0.0
    assert bridge._external_arm_drive_elapsed_s(gate, sim_time_s=13.75) == pytest.approx(1.25)
    assert bridge._sensor_anchor_active(
        "off",
        motion_started=False,
        external_arm_gate=gate,
    ) is False
    assert not list(tmp_path.glob(".*.tmp"))

    # The gate is immutable after acknowledgement; replacing the file cannot revoke it.
    arm_file.write_text("{}", encoding="utf-8")
    assert gate.poll(sim_time_s=14.0, monotonic_now_s=100.3) == "armed"


def test_mujoco_native_dds_external_arm_is_fail_closed_on_invalid_or_late_file(tmp_path):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    invalid_file = tmp_path / "invalid_arm.json"
    invalid_file.write_text(
        json.dumps(
            {
                "schema": bridge.EXTERNAL_ARM_SCHEMA,
                "arm": True,
                "token": "wrong-token",
                "domain_id": 83,
                "scenario": "free",
            }
        ),
        encoding="utf-8",
    )
    invalid_gate = bridge.ExternalArmGate(
        arm_file=invalid_file,
        token="run-token",
        domain_id=83,
        scenario="free",
        timeout_s=10.0,
        started_wall_s=10.0,
    )
    assert invalid_gate.poll(sim_time_s=3.0, monotonic_now_s=10.1) == "invalid"
    assert invalid_gate.failure_gap == "external_arm_token_mismatch"
    assert invalid_gate.acknowledged is False

    late_file = tmp_path / "late_arm.json"
    timeout_gate = bridge.ExternalArmGate(
        arm_file=late_file,
        token="run-token",
        domain_id=83,
        scenario="free",
        timeout_s=1.0,
        started_wall_s=20.0,
    )
    assert timeout_gate.poll(sim_time_s=4.0, monotonic_now_s=21.0) == "timed_out"
    assert timeout_gate.failure_gap == "external_arm_timeout"
    bridge._write_atomic_json_object(
        late_file,
        {
            "schema": bridge.EXTERNAL_ARM_SCHEMA,
            "arm": True,
            "token": "run-token",
            "domain_id": 83,
            "scenario": "free",
        },
    )
    assert timeout_gate.poll(sim_time_s=5.0, monotonic_now_s=21.1) == "timed_out"
    assert timeout_gate.acknowledged is False


def test_mujoco_native_dds_external_arm_retries_transient_read_error(
    tmp_path, monkeypatch
):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    arm_file = tmp_path / "sensor_arm.json"
    bridge._write_atomic_json_object(
        arm_file,
        {
            "schema": bridge.EXTERNAL_ARM_SCHEMA,
            "arm": True,
            "token": "run-token",
            "domain_id": 83,
            "scenario": "free",
        },
    )
    gate = bridge.ExternalArmGate(
        arm_file=arm_file,
        token="run-token",
        domain_id=83,
        scenario="free",
        timeout_s=10.0,
        started_wall_s=10.0,
    )
    original_read_bytes = Path.read_bytes
    attempts = 0

    def transient_read(path: Path) -> bytes:
        nonlocal attempts
        if path == arm_file.resolve() and attempts == 0:
            attempts += 1
            raise PermissionError("transient sharing violation")
        return original_read_bytes(path)

    monkeypatch.setattr(Path, "read_bytes", transient_read)
    assert gate.poll(sim_time_s=3.0, monotonic_now_s=10.1) == "waiting"
    assert gate.acknowledged is False
    assert gate.snapshot()["read_failures"] == 1
    assert gate.poll(sim_time_s=3.1, monotonic_now_s=10.2) == "armed"
    assert gate.acknowledged is True


def test_mujoco_native_dds_external_arm_ignores_periodic_status_write_failure_but_requires_armed_ack(
    tmp_path, monkeypatch
):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    arm_file = tmp_path / "sensor_arm.json"
    status_file = tmp_path / "sensor_arm_status.json"
    gate = bridge.ExternalArmGate(
        arm_file=arm_file,
        token="run-token",
        domain_id=83,
        scenario="free",
        timeout_s=10.0,
        status_json=status_file,
        started_wall_s=10.0,
    )
    write_json = bridge._write_atomic_json_object

    def fail_write(path, payload):
        if path == status_file.resolve():
            raise PermissionError("transient sharing violation")
        write_json(path, payload)

    monkeypatch.setattr(bridge, "_write_atomic_json_object", fail_write)
    assert gate.poll(sim_time_s=3.0, monotonic_now_s=10.3) == "waiting"
    assert gate.snapshot()["status_write_failures"] == 1

    write_json(
        arm_file,
        {
            "schema": bridge.EXTERNAL_ARM_SCHEMA,
            "arm": True,
            "token": "run-token",
            "domain_id": 83,
            "scenario": "free",
        },
    )
    assert gate.poll(sim_time_s=3.1, monotonic_now_s=10.4) == "invalid"
    assert gate.failure_gap == "external_arm_status_write_failed"


def test_mujoco_native_dds_sensor_bridge_catches_up_by_dropping_observations(monkeypatch):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    clock = bridge.SimulatedHardwareClock(
        sim_start_s=10.0,
        wall_epoch_s=1000.0,
        monotonic_start_s=2000.0,
        realtime_factor=1.0,
    )
    controller = bridge.SimHardwareCatchUpController(
        clock=clock,
        max_lag_s=0.05,
        yield_every_steps=2,
    )
    sleeps = []
    monkeypatch.setattr(bridge.time, "sleep", sleeps.append)

    assert controller.should_drop_sensor_tick(10.0, monotonic_now_s=2000.04) is False
    assert controller.should_drop_sensor_tick(10.0, monotonic_now_s=2000.08) is True
    assert controller.should_drop_sensor_tick(10.005, monotonic_now_s=2000.085) is True
    controller.force_sensor_observation(reason="lidar_due")
    controller.record_lidar_subscan_drop()
    controller.record_lidar_frame_drops(2)
    controller.yield_if_due()
    assert sleeps == []

    # Catching up advances simulated dynamics; once lag is back inside the
    # budget, the next observation is published instead of being dropped.
    assert controller.should_drop_sensor_tick(10.07, monotonic_now_s=2000.10) is False
    assert controller.should_drop_sensor_tick(10.10, monotonic_now_s=2000.20) is True

    stats = controller.stats(10.15, monotonic_now_s=2000.20)
    assert stats["strategy"] == "small_step_dynamics_preserve_lidar_deadlines"
    assert stats["catch_up_events"] == 2
    assert stats["dropped_imu_ticks"] == 2
    assert stats["forced_sensor_observations"] == 1
    assert stats["forced_lidar_observations"] == 1
    assert stats["dropped_lidar_subscan_ticks"] == 1
    assert stats["dropped_lidar_frames"] == 2
    assert stats["max_consecutive_steps"] == 2
    assert stats["final_lag_s"] == pytest.approx(0.05)


def test_mujoco_native_dds_sensor_bridge_accounts_for_skipped_lidar_deadlines():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    next_due_s, dropped = bridge._advance_skipped_lidar_deadlines(
        next_lidar_sim_s=10.0,
        lidar_period_s=0.1,
        sim_time_s=10.205,
    )

    assert dropped == 3
    assert next_due_s == pytest.approx(10.3)


def test_mujoco_native_dds_sensor_bridge_exposes_bounded_catch_up_defaults():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    args = bridge._build_parser().parse_args([])

    assert args.sim_hardware_max_lag_s == pytest.approx(0.05)
    assert args.sim_hardware_catch_up_yield_steps == 40


def test_mujoco_native_dds_parent_diagnostics_are_opt_in():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    args = bridge._build_parser().parse_args([])

    assert args.parent_diagnostics_json == ""
    assert args.parent_diagnostics_period_s == pytest.approx(0.5)


def test_mujoco_native_dds_async_publisher_defaults_to_sync_with_bounded_limits():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    args = bridge._build_parser().parse_args([])

    assert args.publisher_write_mode == "sync"
    assert args.async_publisher_max_bytes == 1_048_576
    assert args.async_publisher_max_records == 512
    assert args.async_publisher_max_batches == 256
    assert args.async_publisher_oldest_s == pytest.approx(0.5)
    assert args.async_publisher_shutdown_s == pytest.approx(2.0)


def test_mujoco_native_dds_parent_diagnostics_write_atomic_rolling_counters(tmp_path):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    output = tmp_path / "parent_sensor_diagnostics.json"
    diagnostics = bridge.ParentSensorDiagnostics(output, period_s=0.5)

    startup = json.loads(output.read_text(encoding="utf-8"))
    assert startup["schema_version"] == "lingtu.mujoco.parent_sensor_diagnostics.v1"
    assert startup["reason"] == "startup"
    assert set(startup["record_types"]) == {
        "cloud",
        "imu",
        "odom_prior",
        "registered_cloud",
    }
    assert startup["record_types"]["imu"]["scheduled"] == 0
    assert startup["scheduler"]["pacing"] == {
        "catch_up_events": 0,
        "catch_up_yields": 0,
        "final_lag_s": 0.0,
        "forced_lidar_observations": 0,
        "forced_sensor_observations": 0,
        "max_consecutive_steps": 0,
        "max_lag_observed_s": 0.0,
    }

    diagnostics.record_scheduled("imu", 3)
    diagnostics.record_catchup_drop("imu", 2)
    diagnostics.record_generated("imu")
    diagnostics.record_deadline_skip("cloud", 4)
    assert diagnostics.force_publish("test") is True

    snapshot = json.loads(output.read_text(encoding="utf-8"))
    assert snapshot["reason"] == "test"
    assert snapshot["record_types"]["imu"]["scheduled"] == 3
    assert snapshot["record_types"]["imu"]["catchup_dropped_before_generation"] == 2
    assert snapshot["record_types"]["imu"]["generated"] == 1
    assert snapshot["scheduler"]["deadline_skip"]["cloud"] == 4
    assert snapshot["record_types"]["imu"]["pipe_write_duration_us"]["bounds_us"]
    assert not list(tmp_path.glob(".*.tmp"))


def test_mujoco_native_dds_parent_diagnostics_publish_scheduler_pacing(tmp_path):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    diagnostics = bridge.ParentSensorDiagnostics(
        tmp_path / "parent_sensor_diagnostics.json",
        period_s=0.5,
    )
    diagnostics.update_scheduler_pacing(
        {
            "final_lag_s": 0.012,
            "max_lag_observed_s": 0.25,
            "max_consecutive_steps": 8,
            "catch_up_events": 3,
            "catch_up_yields": 2,
            "forced_sensor_observations": 9,
            "forced_lidar_observations": 7,
        }
    )
    diagnostics._next_publish_monotonic_ns = 0

    assert diagnostics.maybe_publish() is True

    snapshot = json.loads(diagnostics.path.read_text(encoding="utf-8"))
    assert snapshot["reason"] == "periodic"
    assert snapshot["scheduler"]["pacing"] == {
        "catch_up_events": 3,
        "catch_up_yields": 2,
        "final_lag_s": 0.012,
        "forced_lidar_observations": 7,
        "forced_sensor_observations": 9,
        "max_consecutive_steps": 8,
        "max_lag_observed_s": 0.25,
    }


def test_mujoco_native_dds_parent_diagnostics_signal_handler_only_sets_stop_flag(tmp_path):
    import signal

    from sim.scripts.mujoco import native_dds_sensors as bridge

    diagnostics = bridge.ParentSensorDiagnostics(
        tmp_path / "parent_sensor_diagnostics.json",
        period_s=0.5,
    )
    startup_bytes = diagnostics.path.read_bytes()

    diagnostics.handle_stop_signal(signal.SIGTERM, None)

    assert diagnostics.stop_requested is True
    assert diagnostics.final_reason == "signal:SIGTERM"
    assert diagnostics.path.read_bytes() == startup_bytes
    diagnostics.force_publish(diagnostics.final_reason)
    snapshot = json.loads(diagnostics.path.read_text(encoding="utf-8"))
    assert snapshot["reason"] == "signal:SIGTERM"


def test_mujoco_native_dds_main_writes_final_parent_diagnostics_and_restores_signals(
    tmp_path,
    monkeypatch,
):
    import signal

    from sim.scripts.mujoco import native_dds_sensors as bridge

    output = tmp_path / "parent_sensor_diagnostics.json"
    previous_term_handler = signal.getsignal(signal.SIGTERM)
    previous_int_handler = signal.getsignal(signal.SIGINT)
    monkeypatch.setattr(bridge, "run", lambda _args: {"ok": True})

    result = bridge.main(
        [
            "--parent-diagnostics-json",
            str(output),
            "--parent-diagnostics-period-s",
            "0.25",
        ]
    )

    assert result == 0
    snapshot = json.loads(output.read_text(encoding="utf-8"))
    assert snapshot["reason"] == "final"
    assert snapshot["period_s"] == pytest.approx(0.25)
    assert signal.getsignal(signal.SIGTERM) == previous_term_handler
    assert signal.getsignal(signal.SIGINT) == previous_int_handler


def test_mujoco_native_dds_main_without_parent_diagnostics_does_not_touch_signals(
    monkeypatch,
):
    import signal

    from sim.scripts.mujoco import native_dds_sensors as bridge

    monkeypatch.setattr(bridge, "run", lambda _args: {"ok": True})
    monkeypatch.setattr(
        signal,
        "signal",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("signal handlers must remain untouched")
        ),
    )

    assert bridge.main([]) == 0


def test_mujoco_native_dds_sensor_bridge_writes_motion_complete_marker(tmp_path):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    marker = tmp_path / "motion_complete.json"
    bridge._write_motion_complete_marker(
        marker,
        sim_time_s=12.5,
        goal_reached=True,
    )

    payload = json.loads(marker.read_text(encoding="utf-8"))
    assert payload == {
        "complete": True,
        "goal_reached": True,
        "sim_time_s": 12.5,
    }


def test_mujoco_native_dds_sensor_bridge_timestamps_instantaneous_scan_at_sample_time():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    assert bridge._scan_stamp_sim_time_s(
        scan_time_profile="instantaneous",
        scan_start_sim_s=10.0,
        scan_end_sim_s=10.1,
    ) == pytest.approx(10.1)
    assert bridge._scan_stamp_wall_time_s(
        scan_time_profile="instantaneous",
        wall_scan_start_s=1000.0,
        wall_scan_end_s=1000.1,
    ) == pytest.approx(1000.1)
    assert bridge._scan_stamp_sim_time_s(
        scan_time_profile="physical_rolling",
        scan_start_sim_s=10.0,
        scan_end_sim_s=10.1,
    ) == pytest.approx(10.0)


def test_mujoco_native_dds_sensor_bridge_uses_one_timebase_for_sim_clock():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    assert bridge._sensor_timestamp_s(
        clock="sim",
        sim_clock_epoch_s=1000.0,
        sim_time_s=2.5,
        wall_time_s=2000.0,
    ) == pytest.approx(1002.5)
    assert bridge._scan_start_timestamp_s(
        clock="sim",
        sim_clock_epoch_s=1000.0,
        scan_start_sim_s=2.4,
        wall_scan_start_s=1999.9,
    ) == pytest.approx(1002.4)


def test_mujoco_native_dds_sensor_bridge_keeps_wall_clock_diagnostic_mode():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    assert bridge._sensor_timestamp_s(
        clock="wall",
        sim_clock_epoch_s=1000.0,
        sim_time_s=2.5,
        wall_time_s=2000.0,
    ) == pytest.approx(2000.0)
    assert bridge._scan_start_timestamp_s(
        clock="wall",
        sim_clock_epoch_s=1000.0,
        scan_start_sim_s=2.4,
        wall_scan_start_s=1999.9,
    ) == pytest.approx(1999.9)


def test_mujoco_native_dds_sensor_bridge_auto_scales_sim_hardware_kinematic_imu_acceleration():
    from runtime.msgs.geometry import Quaternion, Vector3
    from runtime.msgs.sensor import Imu
    from sim.scripts.mujoco import native_dds_sensors as bridge

    scale, source = bridge._resolve_imu_acc_axis_scale(
        "auto",
        drive_mode="kinematic",
        acc_mode="finite_difference",
        timestamp_clock="sim_hardware",
    )
    imu = Imu(
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.0, 0.0, 0.0),
        linear_acceleration=Vector3(10.0, 2.0, 9.80665),
        ts=1.0,
        frame_id=bridge.IMU_FRAME_ID,
    )

    bridge._apply_imu_acc_axis_scale(imu, scale)

    assert source == "auto_kinematic_sim_hardware"
    assert imu.linear_acceleration.x == pytest.approx(0.0)
    assert imu.linear_acceleration.y == pytest.approx(2.0)
    assert imu.linear_acceleration.z == pytest.approx(9.80665)


def test_mujoco_native_dds_sensor_bridge_sensor_imu_adds_missing_gravity_for_kinematic():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    class State:
        orientation = np.array([0.0, 0.0, 0.0, 1.0])
        imu_gyro = np.array([0.0, 0.0, 0.0])
        imu_projected_gravity = np.array([0.0, 0.0, -1.0])
        imu_linear_acceleration = np.array([0.0, 0.0, 0.0])
        linear_velocity = np.zeros(3)

    imu = bridge._runtime_imu_from_state(
        State(),
        1.0,
        None,
        0.0,
        "sensor",
    )

    assert imu.linear_acceleration.x == pytest.approx(0.0)
    assert imu.linear_acceleration.y == pytest.approx(0.0)
    assert imu.linear_acceleration.z == pytest.approx(bridge._MID360_ACCEL_MPS2_PER_G)


def test_mujoco_native_dds_sensor_bridge_sensor_imu_preserves_physical_accelerometer():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    class State:
        orientation = np.array([0.0, 0.0, 0.0, 1.0])
        imu_gyro = np.array([0.0, 0.0, 0.0])
        imu_projected_gravity = np.array([0.0, 0.0, -1.0])
        imu_linear_acceleration = np.array([0.3, -0.2, 10.1])
        linear_velocity = np.zeros(3)

    imu = bridge._runtime_imu_from_state(
        State(),
        1.0,
        None,
        0.0,
        "sensor",
    )

    assert imu.linear_acceleration.x == pytest.approx(0.3)
    assert imu.linear_acceleration.y == pytest.approx(-0.2)
    assert imu.linear_acceleration.z == pytest.approx(10.1)


def test_mujoco_native_dds_sensor_bridge_conditions_contact_impulses_before_fastlio():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    class State:
        orientation = np.array([0.0, 0.0, 0.0, 1.0])
        imu_gyro = np.array([0.0, 0.0, 0.0])
        imu_projected_gravity = np.array([0.0, 0.0, -1.0])
        imu_linear_acceleration = np.array([0.0, 32.0, bridge._MID360_ACCEL_MPS2_PER_G])
        linear_velocity = np.zeros(3)

    conditioner = bridge.SimImuSignalConditioner(
        lowpass_hz=30.0,
        max_dynamic_accel_mps2=6.0,
        max_slew_rate_mps3=400.0,
    )

    imu = bridge._runtime_imu_from_state(
        State(),
        1.0,
        None,
        0.005,
        "sensor",
        acc_conditioner=conditioner,
    )
    dynamic_y = imu.linear_acceleration.y

    assert abs(dynamic_y) <= 6.0
    assert conditioner.stats()["dynamic_clipped_count"] == 1


def test_mujoco_native_dds_sensor_bridge_rejects_kinematic_fastlio_acceptance_by_default():
    from runtime.runtime_interface import TOPICS
    from sim.scripts.mujoco import native_dds_sensors as bridge

    sensor_counts = Counter({TOPICS.lidar_scan: 3, TOPICS.imu: 40, TOPICS.odom_prior: 40})
    slam_counts = Counter({topic: 1 for topic in bridge.REQUIRED_SLAM_OUTPUT_TOPICS})

    gaps = bridge._remaining_gaps(
        sensor_counts=sensor_counts,
        slam_counts=slam_counts,
        require_slam_output=True,
        drive_mode="kinematic",
    )
    allowed = bridge._remaining_gaps(
        sensor_counts=sensor_counts,
        slam_counts=slam_counts,
        require_slam_output=True,
        drive_mode="kinematic",
        allow_kinematic_fastlio_acceptance=True,
    )

    assert gaps == ["kinematic_fastlio_acceptance_disabled"]
    assert allowed == []


def test_mujoco_native_dds_sensor_bridge_physical_rolling_uses_subscan_budget():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    assert (
        bridge._rolling_subscan_sample_count(
            frame_samples=15000,
            imu_period_s=0.01,
            lidar_period_s=0.10,
        )
        == 1500
    )
    assert (
        bridge._rolling_subscan_sample_count(
            frame_samples=15000,
            imu_period_s=0.02,
            lidar_period_s=0.10,
        )
        == 3000
    )


def test_mujoco_native_dds_sensor_bridge_builds_physical_rolling_scan():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    instantaneous = bridge._relative_times_for_scan(
        4,
        0.1,
        scan_time_profile="instantaneous",
    )
    rolling = bridge._relative_times_for_scan(
        4,
        0.1,
        scan_time_profile="synthetic_rolling",
    )
    sensor, world, times, moving_count, subscan_count = bridge._physical_rolling_scan_from_samples(
        [
            (
                1.02,
                np.array([[1.0, 0.0, 0.0, 10.0]], dtype=np.float32),
                np.array([[10.0, 0.0, 0.0, 10.0]], dtype=np.float32),
                0,
            ),
            (
                1.08,
                np.array(
                    [[2.0, 0.0, 0.0, 20.0], [3.0, 0.0, 0.0, 30.0]],
                    dtype=np.float32,
                ),
                np.array(
                    [[20.0, 0.0, 0.0, 20.0], [30.0, 0.0, 0.0, 30.0]],
                    dtype=np.float32,
                ),
                2,
            ),
        ],
        scan_start_s=1.0,
        scan_end_s=1.1,
    )

    assert np.all(instantaneous == 0.0)
    assert rolling.tolist() == pytest.approx([0.0, 0.025, 0.05, 0.075])
    assert sensor.shape == (3, 4)
    assert world.shape == (3, 4)
    assert times.tolist() == pytest.approx([0.02, 0.08, 0.08])
    assert moving_count == 2
    assert subscan_count == 2


def test_mujoco_native_dds_sensor_bridge_steps_engine_at_sensor_tick():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    class Engine:
        def __init__(self):
            self.calls = []

        def step_sensor_tick(self, cmd, dt_s):
            self.calls.append(("sensor", cmd, dt_s))
            return "sensor_state"

        def step(self, cmd):
            self.calls.append(("control", cmd))
            return "control_state"

    engine = Engine()
    cmd = object()

    state = bridge._step_engine_for_sensor_tick(engine, cmd, 0.005)

    assert state == "sensor_state"
    assert engine.calls[0][:2] == ("sensor", cmd)
    assert engine.calls[0][2] == pytest.approx(0.005)


def test_mujoco_engine_fast_static_clock_advance_skips_state_recompute():
    import threading

    from sim.compat.engine.mujoco.engine import MuJoCoEngine

    engine = object.__new__(MuJoCoEngine)
    engine._physics_dt = 0.001
    engine._sensor_tick_residual_s = 0.0
    engine._sim_time = 2.0
    engine._data_lock = threading.RLock()
    engine._data = types.SimpleNamespace(
        qvel=np.ones(3, dtype=np.float64),
        qacc=np.ones(3, dtype=np.float64),
        time=2.0,
    )

    result = engine.advance_static_sensor_clock(dt_s=0.005)

    assert result == pytest.approx(2.005)
    assert engine._data.time == pytest.approx(2.005)
    assert engine._data.qvel.tolist() == pytest.approx([0.0, 0.0, 0.0])
    assert engine._data.qacc.tolist() == pytest.approx([0.0, 0.0, 0.0])
    assert engine._sensor_tick_residual_s == pytest.approx(0.0)


def test_mujoco_lidar_pattern_cursor_supports_subscan_sample_count():
    from sim.compat.engine.mujoco.lidar import MuJoCoLidar

    lidar = object.__new__(MuJoCoLidar)
    lidar._ray_angles = np.asarray(
        [[0.0, 0.0], [1.0, 0.1], [2.0, 0.2], [3.0, 0.3]],
        dtype=np.float32,
    )
    lidar._ray_cursor = 0
    lidar._config = types.SimpleNamespace(samples_per_frame=4, add_noise=False)
    lidar._rng = np.random.default_rng(0)

    theta, phi = lidar._next_pattern_angles(sample_count=2)
    theta2, _phi2 = lidar._next_pattern_angles()

    assert theta.tolist() == pytest.approx([0.0, 1.0])
    assert phi.tolist() == pytest.approx([0.0, 0.1])
    assert theta2.tolist() == pytest.approx([2.0, 3.0, 0.0, 1.0])
    assert lidar._ray_cursor == 2


def test_mujoco_native_dds_sensor_bridge_auto_keeps_legacy_split_acceleration_scale():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    scale, source = bridge._resolve_imu_acc_axis_scale(
        "auto",
        drive_mode="kinematic",
        acc_mode="finite_difference",
        timestamp_clock="wall",
    )

    assert source == "auto_kinematic_legacy_split"
    assert scale == pytest.approx((-0.43, 1.0, 1.0))


def test_mujoco_native_dds_sensor_bridge_scales_gyro_when_requested():
    from runtime.msgs.geometry import Quaternion, Vector3
    from runtime.msgs.sensor import Imu
    from sim.scripts.mujoco import native_dds_sensors as bridge

    imu = Imu(
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(1.0, 2.0, 3.0),
        linear_acceleration=Vector3(0.0, 0.0, 9.80665),
        ts=1.0,
        frame_id=bridge.IMU_FRAME_ID,
    )

    bridge._apply_imu_gyro_axis_scale(imu, bridge._resolve_axis_scale("1,0.5,0.8", "--imu-gyro-axis-scale"))

    assert imu.angular_velocity.x == pytest.approx(1.0)
    assert imu.angular_velocity.y == pytest.approx(1.0)
    assert imu.angular_velocity.z == pytest.approx(2.4)


def test_mujoco_native_dds_sensor_bridge_auto_keeps_policy_imu_identity():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    scale, source = bridge._resolve_imu_acc_axis_scale(
        "auto",
        drive_mode="policy",
        acc_mode="finite_difference",
    )

    assert scale == (1.0, 1.0, 1.0)
    assert source == "auto_identity"


def test_mujoco_native_dds_fastlio_config_uses_sim_gravity_scale():
    cfg = Path("src/localization/fastlio2/config/sim_mid360_slam.yaml")
    text = cfg.read_text(encoding="utf-8")

    assert "acc_scale: 9.80665" in text
    assert "r_il: [0.7071067812, 0.0, 0.7071067812, 0.0, -1.0, 0.0, 0.7071067812, 0.0, -0.7071067812]" in text
    assert "t_il: [0.402876074867229, 0.0, 0.0582019450665819]" in text
    assert "navigation_body_from_imu_translation: [0.0, 0.0, 0.0]" in text
    assert "max_update_translation_m: 0.35" in text
    assert "max_update_velocity_mps: 3.0" in text
    assert "odom_prior_enabled: false" in text
    assert "vertical_velocity_constraint_enabled: true" in text
    assert "max_update_velocity_delta_mps: 0.35" in text
    assert "reject_nonconverged_update: false" in text
    assert "reject_degenerate_nonconverged_update: false" in text


def test_thunderv4_mid360_recording_world_is_dense(tmp_path):
    pytest.importorskip("cv2")
    pytest.importorskip("imageio")
    pytest.importorskip("mujoco")
    import xml.etree.ElementTree as ET

    from sim.scripts.mujoco.record_thunderv4_mid360_policy import _write_world

    world = tmp_path / "world.xml"
    _write_world(world)
    names = {
        geom.attrib.get("name", "") for geom in ET.fromstring(world.read_text(encoding="utf-8")).findall(".//geom")
    }

    assert "front_machine_left" in names
    assert "overhead_beam_a" in names
    assert "pipe_left" in names
    assert len(names) >= 14


def test_mujoco_driver_raw_scan_uses_lidar_frame_for_native_slam():
    import drivers.sim.mujoco.driver as driver
    from runtime.runtime_interface import TOPICS, topic_default_frame_id

    source = Path(driver.__file__).read_text(encoding="utf-8")

    assert driver.MUJOCO_MODULE_LIDAR_FRAME_ID == topic_default_frame_id(TOPICS.lidar_scan)
    assert "frame_id=MUJOCO_MODULE_LIDAR_FRAME_ID" in source


def test_thunder_v3_source_assets_are_resolvable():
    asset_root = Path(__file__).resolve().parents[2] / "sim" / "compat" / "assets" / "thunder_v3"
    urdf_path = asset_root / "urdf" / "thunder_v3.urdf"
    xml_path = asset_root / "xml" / "thunder_v3.xml"

    assert urdf_path.exists()
    assert xml_path.exists()
    assert xml_path.read_bytes() == urdf_path.read_bytes()

    root = ET.parse(urdf_path).getroot()
    assert root.attrib["name"] == "thunder_v3"

    masses = [float(m.attrib["value"]) for m in root.findall(".//mass")]
    assert sum(masses) == pytest.approx(48.79163, abs=1e-5)
    assert sum(1 for value in masses if value == pytest.approx(1.40377, abs=1e-5)) == 4

    joints = {j.attrib["name"]: j for j in root.findall("joint")}
    assert "FR_hip_joint" in joints
    assert "fr_hip_joint" not in joints
    assert joints["FR_hip_joint"].find("limit").attrib["effort"] == "120"
    assert joints["FR_hip_joint"].find("limit").attrib["velocity"] == "17.48"
    assert joints["FR_foot_joint"].attrib["type"] == "continuous"
    assert joints["FR_foot_joint"].find("limit") is None

    for mesh in root.findall(".//mesh"):
        mesh_path = (urdf_path.parent / mesh.attrib["filename"]).resolve()
        assert mesh_path.exists(), mesh.attrib["filename"]


def test_thunder_v3_mjcf_runtime_keeps_lingtu_sensor_and_control_contracts():
    asset_root = Path(__file__).resolve().parents[2] / "sim" / "compat" / "assets" / "thunder_v3"
    upstream_path = asset_root / "mjcf" / "thunder_v3_mujoco.xml"
    runtime_path = asset_root / "mjcf" / "thunder_v3_lingtu.xml"

    upstream = ET.parse(upstream_path).getroot()
    runtime = ET.parse(runtime_path).getroot()

    assert upstream.attrib["model"] == "thunder_v3_mujoco"
    assert runtime.attrib["model"] == "thunder_v3_mujoco"
    assert runtime.find(".//body[@name='base_link']") is not None
    assert runtime.find(".//body[@name='lidar_link']") is not None
    assert runtime.find(".//camera[@name='front_camera']") is not None

    upstream_motors = upstream.findall("./actuator/motor")
    runtime_positions = runtime.findall("./actuator/position")
    assert len(upstream_motors) == 16
    assert len(runtime_positions) == 16
    assert {a.attrib["joint"] for a in runtime_positions} >= {
        "FR_hip_joint",
        "FL_foot_joint",
        "RR_calf_joint",
        "RL_foot_joint",
    }


@pytest.mark.skipif(not _ROS2_AVAILABLE, reason="Needs ROS2 runtime")
def test_semantic_namespace_wrappers_expose_runtime_import_paths():
    assert importlib.util.find_spec("perception.tracking.instance_tracker") is not None
    assert importlib.util.find_spec("decision.llm.client") is not None

    # Canonical imports from runtime.utils
    from perception.tracking.instance_tracker import InstanceTracker
    from perception.tracking.tracked_objects import TrackedObject
    from runtime.msgs import scene as scene_msgs
    from runtime.utils.sanitize import sanitize_position

    assert callable(sanitize_position)
    assert InstanceTracker is not None
    assert scene_msgs.TrackedObject is TrackedObject


def test_fastlio2_cpp_applies_configured_ieskf_iteration_and_degeneracy_guard():
    root = Path(__file__).resolve().parents[2]
    fastlio2 = root / "src" / "localization" / "fastlio2"
    lidar_processor = (fastlio2 / "src" / "map_builder" / "lidar_processor.cpp").read_text(
        encoding="utf-8"
    )
    ieskf = (fastlio2 / "src" / "map_builder" / "ieskf.cpp").read_text(encoding="utf-8")
    native_backend = (
        root / "src" / "localization" / "slam" / "cpp" / "fastlio.cpp"
    ).read_text(encoding="utf-8")

    assert "setMaxIter(static_cast<size_t>(std::max(1, m_config.ieskf_max_iter)))" in lidar_processor
    assert "setDegeneracyGuard(" in lidar_processor
    assert "degeneracy_max_update_dof" in native_backend
    assert "max_update_translation_m" in native_backend
    assert "max_update_rotation_rad" in native_backend
    assert "reject_nonconverged_update" in native_backend
    assert "reject_degenerate_nonconverged_update" in native_backend
    assert "vertical_velocity_constraint_enabled" in native_backend
    assert "injectVerticalVelocityConstraint" in ieskf
    assert "injectVerticalVelocityConstraint" in (
        fastlio2 / "src" / "map_builder" / "imu_processor.cpp"
    ).read_text(encoding="utf-8")
    assert "too_many_degenerate_dofs" in ieskf
    assert "update_translation_too_large" in ieskf
    assert "update_rotation_too_large" in ieskf
    assert "m_reject_nonconverged_update" in ieskf
    assert "m_reject_degenerate_nonconverged_update" in ieskf
    assert "candidate_covariance_positive_diagonal" in ieskf
    assert "(P_candidate.diagonal().array() > 0.0).all()" in ieskf
    assert "P_MIN" in ieskf


def test_mujoco_sensor_helper_prefers_lidar_site_pose(monkeypatch):
    from drivers.sim.mujoco.sensors import world_xyzi_to_sensor_xyzi

    class FakeMjtObj:
        mjOBJ_SITE = object()

    fake_mujoco = types.SimpleNamespace(
        mjtObj=FakeMjtObj,
        mj_name2id=lambda _model, _obj, name: 2 if name == "lidar_site" else -1,
    )
    monkeypatch.setitem(sys.modules, "mujoco", fake_mujoco)

    class FakeData:
        site_xpos = [None, None, np.array([1.0, 2.0, 0.5])]
        site_xmat = [None, None, np.eye(3).reshape(-1)]
        xpos = [None, np.array([100.0, 200.0, 50.0])]
        xmat = [None, np.eye(3).reshape(-1)]

    class FakeEngine:
        _model = object()
        _data = FakeData()
        _lidar_body_id = 1
        _lidar_cfg = types.SimpleNamespace(site_name="lidar_site")

    world_cloud = np.array([[2.0, 4.0, 1.5, 42.0]], dtype=np.float32)

    sensor_cloud = world_xyzi_to_sensor_xyzi(FakeEngine(), world_cloud)

    np.testing.assert_allclose(sensor_cloud[:, :3], [[1.0, 2.0, 1.0]])
    np.testing.assert_allclose(sensor_cloud[:, 3], [42.0])


def test_mujoco_sensor_helper_converts_sensor_cloud_to_body_frame():
    from drivers.sim.mujoco.sensors import sensor_xyzi_to_body_xyzi
    from runtime.runtime_interface import lidar_extrinsic

    sensor_cloud = np.array(
        [
            [1.0, 2.0, 3.0, 42.0],
            [-0.5, -1.0, 0.0, 12.0],
        ],
        dtype=np.float32,
    )

    body_cloud = sensor_xyzi_to_body_xyzi(
        sensor_cloud,
        lidar_extrinsic("mujoco_thunder_v3"),
    )

    np.testing.assert_allclose(
        body_cloud[:, :3],
        [[0.69362, 2.0, 3.19417], [-0.80638, -1.0, 0.19417]],
        atol=1e-6,
    )
    np.testing.assert_allclose(body_cloud[:, 3], [42.0, 12.0])


def test_mujoco_sensor_helper_preserves_signed_imu_gyro_z():
    from drivers.sim.mujoco.sensors import make_imu_msg

    class FakeImu:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id="")
            self.angular_velocity = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.linear_acceleration = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    state = types.SimpleNamespace(
        imu_gyro=np.array([0.01, -0.02, -0.25]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        linear_velocity=np.zeros(3),
    )

    msg = make_imu_msg(
        state=state,
        prev_velocity=np.zeros(3),
        dt=0.02,
        stamp=types.SimpleNamespace(),
        imu_cls=FakeImu,
    )

    assert msg.angular_velocity.x == pytest.approx(0.01)
    assert msg.angular_velocity.y == pytest.approx(-0.02)
    assert msg.angular_velocity.z == pytest.approx(-0.25)


def test_navigation_runtime_dataflow_documents_no_python_slam_rule():
    architecture = Path("docs/architecture/NAVIGATION_RUNTIME_DATAFLOW.md").read_text(encoding="utf-8")
    recording = Path(
        "docs/07-testing/simulation/thunderv4_mujoco_lidar_recording_requirements.md"
    ).read_text(encoding="utf-8")

    assert "Do not write or ship Python SLAM" in architecture
    assert "Python code may adapt streams, status" in architecture
    assert "it must not become a SLAM backend" in architecture
    assert "Do not write or ship Python SLAM" in recording
    assert "Reports must keep `no_python_slam=true`" in recording


def test_mujoco_native_dds_sensor_bridge_declares_no_python_slam_contract():
    from collections import Counter

    from sim.scripts.mujoco import native_dds_sensors as bridge

    source = Path(bridge.__file__).read_text(encoding="utf-8")
    report = bridge._make_report(
        ok=False,
        duration_s=1.0,
        domain_id=83,
        sensor_counts=Counter({bridge.TOPICS.lidar_scan: 1, bridge.TOPICS.imu: 1}),
        slam_counts=Counter(),
        require_slam_output=True,
        remaining_gaps=["native_slam_output_missing:/slam/odometry"],
    )

    assert "not SLAM" in source
    assert "localization.slam._native" not in source
    assert "DDSTransport" not in source
    assert "runtime.transport.dds" not in source
    assert bridge.NATIVE_SLAM_RUNTIME == "slamd"
    assert bridge.NATIVE_SENSOR_PUBLISHER == "lingtu_mujoco_sensor_publisher --stdin-records --dds"
    assert bridge.REQUIRED_SLAM_OUTPUT_TOPICS == (
        bridge.TOPICS.odometry,
        bridge.TOPICS.map_cloud,
        bridge.TOPICS.localization_health,
    )
    assert report["no_python_slam"] is True
    assert report["native_sensor_publisher"] == ""
    assert report["python_role"] == "mujoco_sensor_dds_adapter_only"
    assert report["localization_runtime_expected"] == "slamd"
    assert report["sensor_topics"][bridge.TOPICS.lidar_scan]["dds_topic"] == "rt/lidar/raw_frame"


def test_mujoco_native_dds_sensor_bridge_accepts_policy_path_override():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    args = bridge._build_parser().parse_args(
        [
            "--drive-mode",
            "policy",
            "--policy-path",
            "sim/packages/controllers/doso/thunder_v4/locomotion/policy/pose_flat_low_kpkd_microterrain_model29600_policy.onnx",
        ]
    )
    source = Path(bridge.__file__).read_text(encoding="utf-8")

    assert args.policy_path.endswith("_policy.onnx")
    assert "policy_path = _resolve_policy_path_for_drive" in source
    assert "policy_path=policy_path" in source


def test_mujoco_native_dds_sensor_bridge_reads_native_slam_status(tmp_path):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    status_path = tmp_path / "status.json"
    status_path.write_text(
        json.dumps(
            {
                "has_odom": True,
                "registered_points": 7,
                "map_points": 11,
                "state": "TRACKING",
                "localization_quality": 0.42,
            }
        ),
        encoding="utf-8",
    )

    counts, status = bridge._slam_status_counts(str(status_path))

    assert counts[bridge.TOPICS.odometry] == 1
    assert counts[bridge.TOPICS.registered_cloud] == 1
    assert counts[bridge.TOPICS.map_cloud] == 1
    assert counts[bridge.TOPICS.localization_health] == 1
    assert counts[bridge.TOPICS.localization_quality] == 1
    assert status["state"] == "TRACKING"


def test_mujoco_native_dds_sensor_bridge_writes_livox_binary_record():
    import io

    from sim.scripts.mujoco import native_dds_sensors as bridge

    stream = io.BytesIO()
    bridge._write_record(stream, bridge._RECORD_IMU, 123, 4, b"abcdef", 1)

    raw = stream.getvalue()
    magic, record_type, timestamp_ns, sequence, count, payload_bytes = bridge._HEADER.unpack(raw[: bridge._HEADER.size])
    assert magic == bridge._MAGIC
    assert record_type == bridge._RECORD_IMU
    assert timestamp_ns == 123
    assert sequence == 4
    assert count == 1
    assert payload_bytes == 6
    assert raw[bridge._HEADER.size :] == b"abcdef"


def test_mujoco_native_dds_async_publisher_preserves_mixed_record_order_and_batch_flushes():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    class RecordingStream:
        def __init__(self):
            self.events = []

        def write(self, value):
            self.events.append(("write", bytes(value)))
            return len(value)

        def flush(self):
            self.events.append(("flush", b""))

        def close(self):
            self.events.append(("close", b""))

    stream = RecordingStream()
    publisher = bridge.AsyncFifoPublisher(stream)
    first = bridge.AsyncPublisherBatch()
    bridge._write_record(
        None,
        bridge._RECORD_IMU,
        100,
        1,
        b"imu-1",
        1,
        async_batch=first,
        diagnostic_record_type="imu",
    )
    bridge._write_record(
        None,
        bridge._RECORD_CLOUD,
        101,
        2,
        b"cloud-1",
        1,
        async_batch=first,
        diagnostic_record_type="cloud",
    )
    second = bridge.AsyncPublisherBatch()
    bridge._write_record(
        None,
        bridge._RECORD_ODOM_PRIOR,
        102,
        3,
        b"odom-1",
        1,
        async_batch=second,
        diagnostic_record_type="odom_prior",
    )
    bridge._write_record(
        None,
        bridge._RECORD_IMU,
        103,
        4,
        b"imu-2",
        1,
        async_batch=second,
        diagnostic_record_type="imu",
    )

    publisher.enqueue(first)
    publisher.enqueue(second)
    publisher.request_stop("test_complete")
    assert publisher.join(timeout_s=1.0) is True
    publisher.raise_if_failed()

    record_types = [
        bridge._HEADER.unpack(value)[1]
        for event, value in stream.events
        if event == "write" and len(value) == bridge._HEADER.size
    ]
    assert record_types == [
        bridge._RECORD_IMU,
        bridge._RECORD_CLOUD,
        bridge._RECORD_ODOM_PRIOR,
        bridge._RECORD_IMU,
    ]
    assert [event for event, _value in stream.events] == [
        "write",
        "write",
        "write",
        "write",
        "flush",
        "write",
        "write",
        "write",
        "write",
        "flush",
        "close",
    ]
    assert publisher.stats()["enqueued_batch_sequence"] == 2
    assert publisher.stats()["enqueued_record_sequence"] == 4
    assert publisher.stats()["written_batch_sequence"] == 2
    assert publisher.stats()["written_record_sequence"] == 4


def test_mujoco_native_dds_async_publisher_coalesces_backlog_without_reordering():
    import threading

    from sim.scripts.mujoco import native_dds_sensors as bridge

    class BackloggedStream:
        def __init__(self):
            self.first_write = threading.Event()
            self.release = threading.Event()
            self.payloads = []
            self.flushes = 0

        def write(self, value):
            self.first_write.set()
            assert self.release.wait(timeout=1.0)
            self.payloads.append(bytes(value))
            return len(value)

        def flush(self):
            self.flushes += 1

        def close(self):
            return None

    def one_record(sequence):
        batch = bridge.AsyncPublisherBatch()
        bridge._write_record(
            None,
            bridge._RECORD_IMU,
            100 + sequence,
            sequence,
            bytes([sequence]),
            1,
            async_batch=batch,
            diagnostic_record_type="imu",
        )
        return batch

    stream = BackloggedStream()
    publisher = bridge.AsyncFifoPublisher(stream)
    publisher.enqueue(one_record(1))
    assert stream.first_write.wait(timeout=1.0)
    for sequence in range(2, 18):
        publisher.enqueue(one_record(sequence))
    stream.release.set()
    publisher.request_stop("test_complete")
    assert publisher.join(timeout_s=1.0) is True
    publisher.raise_if_failed()

    payloads = [
        value[0]
        for index, value in enumerate(stream.payloads)
        if index % 2 == 1
    ]
    assert payloads == list(range(1, 18))
    assert stream.flushes == 2
    assert publisher.stats()["written_batch_sequence"] == 17
    assert publisher.stats()["written_record_sequence"] == 17


@pytest.mark.parametrize(
    ("limits", "expected_limit"),
    [
        ({"max_bytes": 1, "max_records": 10, "max_batches": 10}, "bytes"),
        ({"max_bytes": 10_000, "max_records": 1, "max_batches": 10}, "records"),
        ({"max_bytes": 10_000, "max_records": 10, "max_batches": 1}, "batches"),
    ],
)
def test_mujoco_native_dds_async_publisher_capacity_is_atomic_and_fatal(
    limits,
    expected_limit,
):
    import threading

    from sim.scripts.mujoco import native_dds_sensors as bridge

    class BlockingStream:
        def __init__(self):
            self.entered = threading.Event()
            self.release = threading.Event()
            self.closed = 0

        def write(self, value):
            self.entered.set()
            assert self.release.wait(timeout=1.0)
            return len(value)

        def flush(self):
            return None

        def close(self):
            self.closed += 1

    def one_record(payload):
        batch = bridge.AsyncPublisherBatch()
        bridge._write_record(
            None,
            bridge._RECORD_IMU,
            100,
            1,
            payload,
            1,
            async_batch=batch,
            diagnostic_record_type="imu",
        )
        return batch

    if expected_limit == "bytes":
        limits["max_bytes"] = bridge._HEADER.size + 1
    stream = BlockingStream()
    publisher = bridge.AsyncFifoPublisher(stream, **limits)
    publisher.enqueue(one_record(b"a"))
    assert stream.entered.wait(timeout=1.0)

    with pytest.raises(bridge.AsyncPublisherError, match=f"queue_full:{expected_limit}"):
        publisher.enqueue(one_record(b"b"))
    with pytest.raises(bridge.AsyncPublisherError, match=f"queue_full:{expected_limit}"):
        publisher.raise_if_failed()

    stats = publisher.stats()
    assert stats["enqueued_batch_sequence"] == 1
    assert stats["enqueued_record_sequence"] == 1
    assert stats["undrained_batches"] == 1
    assert stats["undrained_records"] == 1
    stream.release.set()
    assert publisher.join(timeout_s=1.0) is True
    assert stream.closed == 1


def test_mujoco_native_dds_async_publisher_oldest_age_is_fatal():
    import threading

    from sim.scripts.mujoco import native_dds_sensors as bridge

    class FakeClock:
        now_ns = 0

        def __call__(self):
            return self.now_ns

    class BlockingStream:
        def __init__(self):
            self.entered = threading.Event()
            self.release = threading.Event()

        def write(self, value):
            self.entered.set()
            assert self.release.wait(timeout=1.0)
            return len(value)

        def flush(self):
            return None

        def close(self):
            return None

    clock = FakeClock()
    stream = BlockingStream()
    publisher = bridge.AsyncFifoPublisher(
        stream,
        oldest_s=0.5,
        monotonic_ns=clock,
    )
    batch = bridge.AsyncPublisherBatch()
    bridge._write_record(
        None,
        bridge._RECORD_IMU,
        100,
        1,
        b"a",
        1,
        async_batch=batch,
        diagnostic_record_type="imu",
    )
    publisher.enqueue(batch)
    assert stream.entered.wait(timeout=1.0)

    clock.now_ns = 500_000_001
    with pytest.raises(bridge.AsyncPublisherError, match="queue_oldest_age"):
        publisher.raise_if_failed()

    assert publisher.stats()["oldest_age_s"] == pytest.approx(0.500000001)
    stream.release.set()
    assert publisher.join(timeout_s=1.0) is True


def test_mujoco_native_dds_async_publisher_reports_enqueue_queue_and_writer_io(tmp_path):
    import io

    from sim.scripts.mujoco import native_dds_sensors as bridge

    diagnostics = bridge.ParentSensorDiagnostics(
        tmp_path / "parent_sensor_diagnostics.json",
        period_s=0.5,
    )
    publisher = bridge.AsyncFifoPublisher(
        io.BytesIO(),
        parent_diagnostics=diagnostics,
    )
    batch = bridge.AsyncPublisherBatch()
    bridge._write_record(
        None,
        bridge._RECORD_IMU,
        100,
        1,
        b"imu",
        1,
        async_batch=batch,
        diagnostic_record_type="imu",
    )
    bridge._write_record(
        None,
        bridge._RECORD_CLOUD,
        101,
        2,
        b"cloud",
        1,
        async_batch=batch,
        diagnostic_record_type="cloud",
    )

    publisher.enqueue(batch)
    publisher.request_stop("test_complete")
    assert publisher.join(timeout_s=1.0) is True
    publisher.raise_if_failed()
    diagnostics.force_publish("test")

    snapshot = json.loads(diagnostics.path.read_text(encoding="utf-8"))
    for name, wire_bytes in (
        ("imu", bridge._HEADER.size + 3),
        ("cloud", bridge._HEADER.size + 5),
    ):
        record = snapshot["record_types"][name]
        assert record["enqueue_attempt"] == 1
        assert record["enqueue_success"] == 1
        assert record["enqueue_error"] == 0
        assert record["enqueue_full"] == 0
        assert record["enqueue_bytes"] == wire_bytes
        assert record["enqueue_duration_us"]["count"] == 1
        assert record["pipe_write_attempt"] == 1
        assert record["pipe_write_success"] == 1
    queue = snapshot["async_queue"]
    assert queue["enabled"] is True
    assert queue["current_batches"] == 0
    assert queue["max_batches"] == 1
    assert queue["enqueued_batch_sequence"] == 1
    assert queue["enqueued_record_sequence"] == 2
    assert queue["written_batch_sequence"] == 1
    assert queue["written_record_sequence"] == 2
    assert queue["batch_sequence_lag"] == 0
    assert queue["record_sequence_lag"] == 0
    assert queue["undrained_batches"] == 0
    assert queue["writer_alive"] is False
    assert queue["cleanup_reason"] == "test_complete"
    assert queue["fatal_reason"] == ""
    assert snapshot["flush"]["attempt"] == 1
    assert snapshot["flush"]["success"] == 1


@pytest.mark.parametrize("failure_point", ["write", "flush"])
def test_mujoco_native_dds_async_publisher_propagates_first_writer_error(
    tmp_path,
    failure_point,
):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    class FailingStream:
        def __init__(self):
            self.closed = 0

        def write(self, value):
            if failure_point == "write":
                raise OSError("write failed")
            return len(value)

        def flush(self):
            if failure_point == "flush":
                raise OSError("flush failed")

        def close(self):
            self.closed += 1
            raise OSError("later close failed")

    diagnostics = bridge.ParentSensorDiagnostics(
        tmp_path / f"{failure_point}.json",
        period_s=0.5,
    )
    stream = FailingStream()
    publisher = bridge.AsyncFifoPublisher(
        stream,
        parent_diagnostics=diagnostics,
    )
    batch = bridge.AsyncPublisherBatch()
    bridge._write_record(
        None,
        bridge._RECORD_IMU,
        100,
        1,
        b"imu",
        1,
        async_batch=batch,
        diagnostic_record_type="imu",
    )
    publisher.enqueue(batch)
    assert publisher.join(timeout_s=1.0) is True

    with pytest.raises(bridge.AsyncPublisherError, match=f"{failure_point} failed") as failure:
        publisher.raise_if_failed()
    assert isinstance(failure.value.__cause__, OSError)
    assert str(failure.value.__cause__) == f"{failure_point} failed"

    rejected = bridge.AsyncPublisherBatch()
    bridge._write_record(
        None,
        bridge._RECORD_CLOUD,
        101,
        2,
        b"cloud",
        1,
        async_batch=rejected,
        diagnostic_record_type="cloud",
    )
    with pytest.raises(bridge.AsyncPublisherError, match=f"{failure_point} failed"):
        publisher.enqueue(rejected)

    diagnostics.force_publish("test")
    snapshot = json.loads(diagnostics.path.read_text(encoding="utf-8"))
    assert snapshot["async_queue"]["fatal_reason"] == "writer_error"
    assert snapshot["async_queue"]["undrained_batches"] == 1
    assert snapshot["record_types"]["cloud"]["enqueue_error"] == 1
    assert stream.closed == 1
    if failure_point == "write":
        assert snapshot["record_types"]["imu"]["pipe_write_error"] == 1
        assert snapshot["flush"]["attempt"] == 0
    else:
        assert snapshot["record_types"]["imu"]["pipe_write_success"] == 1
        assert snapshot["flush"]["error"] == 1


def test_mujoco_native_dds_async_publisher_term_and_finally_stop_are_idempotent(
    monkeypatch,
):
    import threading

    from sim.scripts.mujoco import native_dds_sensors as bridge

    class RecordingStream:
        def __init__(self):
            self.events = []

        def write(self, value):
            self.events.append(("write", threading.current_thread().name, bytes(value)))
            return len(value)

        def flush(self):
            self.events.append(("flush", threading.current_thread().name, b""))

        def close(self):
            self.events.append(("close", threading.current_thread().name, b""))

    stream = RecordingStream()
    publisher = bridge.AsyncFifoPublisher(stream)
    batch = bridge.AsyncPublisherBatch()
    bridge._write_record(
        None,
        bridge._RECORD_IMU,
        100,
        1,
        b"imu",
        1,
        async_batch=batch,
        diagnostic_record_type="imu",
    )
    publisher.enqueue(batch)

    publisher.request_stop("signal:SIGTERM")
    publisher.request_stop("finally")
    monkeypatch.setattr(
        bridge,
        "_terminate_wsl_pid",
        lambda _pid: pytest.fail("normal drain must not terminate the child"),
    )
    process = types.SimpleNamespace(_lingtu_linux_pid=4321, stdin=stream)
    cleanup = bridge._shutdown_async_native_publisher(
        publisher,
        process,
        timeout_s=1.0,
    )
    assert cleanup["timed_out"] is False
    assert cleanup["termination"] is None
    publisher.request_stop("finally_again")
    publisher.raise_if_failed()

    assert publisher.stats()["cleanup_reason"] == "signal:SIGTERM"
    assert [event for event, _thread, _value in stream.events].count("flush") == 1
    close_events = [event for event in stream.events if event[0] == "close"]
    assert len(close_events) == 1
    assert close_events[0][1] == "mujoco-native-dds-writer"


@pytest.mark.parametrize("release_on_terminate", [True, False])
def test_mujoco_native_dds_async_publisher_timeout_terminates_child_before_second_join(
    monkeypatch,
    release_on_terminate,
):
    import threading

    from sim.scripts.mujoco import native_dds_sensors as bridge

    class BlockingStream:
        def __init__(self):
            self.entered = threading.Event()
            self.release = threading.Event()
            self.close_threads = []

        def write(self, value):
            self.entered.set()
            self.release.wait()
            return len(value)

        def flush(self):
            return None

        def close(self):
            self.close_threads.append(threading.current_thread().name)

    class Process:
        _lingtu_linux_pid = 4321

        def __init__(self, stream):
            self.stdin = stream

    stream = BlockingStream()
    process = Process(stream)
    publisher = bridge.AsyncFifoPublisher(stream)
    batch = bridge.AsyncPublisherBatch()
    bridge._write_record(
        None,
        bridge._RECORD_IMU,
        100,
        1,
        b"imu",
        1,
        async_batch=batch,
        diagnostic_record_type="imu",
    )
    publisher.enqueue(batch)
    assert stream.entered.wait(timeout=1.0)
    terminated = []

    def terminate_child(pid):
        terminated.append(pid)
        if release_on_terminate:
            stream.release.set()
        return {"linux_pid": pid, "clean": True, "errors": []}

    monkeypatch.setattr(bridge, "_terminate_wsl_pid", terminate_child)

    cleanup = bridge._shutdown_async_native_publisher(
        publisher,
        process,
        timeout_s=0.01,
    )

    assert cleanup["timed_out"] is True
    assert cleanup["joined_after_terminate"] is release_on_terminate
    assert cleanup["timeout_stats"]["writer_alive"] is True
    assert cleanup["timeout_stats"]["undrained_batches"] == 1
    assert cleanup["queue"]["cleanup_reason"] == "shutdown_timeout"
    assert cleanup["queue"]["fatal_reason"] == "shutdown_timeout"
    assert cleanup["queue"]["undrained_batches"] == 1
    assert terminated == [4321]
    if release_on_terminate:
        assert cleanup["queue"]["writer_alive"] is False
        assert stream.close_threads == ["mujoco-native-dds-writer"]
    else:
        assert cleanup["queue"]["writer_alive"] is True
        assert cleanup["queue"]["fatal_reason"] == "shutdown_timeout"
        stream.release.set()
        assert publisher.join(timeout_s=1.0) is True
        assert stream.close_threads == ["mujoco-native-dds-writer"]


def test_mujoco_native_dds_publish_loop_boundary_keeps_sync_write_then_flush_order():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    class RecordingStream:
        def __init__(self):
            self.events = []

        def write(self, value):
            self.events.append(("write", bytes(value)))
            return len(value)

        def flush(self):
            self.events.append(("flush", b""))

    stream = RecordingStream()
    batch = bridge._begin_native_publisher_batch(None)
    bridge._write_record(
        stream,
        bridge._RECORD_IMU,
        100,
        1,
        b"imu",
        1,
        async_batch=batch,
        diagnostic_record_type="imu",
    )
    bridge._commit_native_publisher_batch(
        stream,
        batch,
        async_publisher=None,
    )

    assert [event for event, _value in stream.events] == ["write", "write", "flush"]


@pytest.mark.parametrize(
    ("fail_on_check", "expected_enqueue_calls"),
    [(1, 0), (2, 0), (3, 1)],
)
def test_mujoco_native_dds_publish_loop_checks_fatal_at_top_before_and_after_enqueue(
    fail_on_check,
    expected_enqueue_calls,
):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    class Publisher:
        def __init__(self):
            self.checks = 0
            self.enqueue_calls = 0

        def raise_if_failed(self):
            self.checks += 1
            if self.checks == fail_on_check:
                raise bridge.AsyncPublisherError(f"failed_check:{self.checks}")

        def enqueue(self, _batch):
            self.enqueue_calls += 1

    publisher = Publisher()
    if fail_on_check == 1:
        with pytest.raises(bridge.AsyncPublisherError, match="failed_check:1"):
            bridge._begin_native_publisher_batch(publisher)
    else:
        batch = bridge._begin_native_publisher_batch(publisher)
        bridge._write_record(
            None,
            bridge._RECORD_IMU,
            100,
            1,
            b"imu",
            1,
            async_batch=batch,
            diagnostic_record_type="imu",
        )
        with pytest.raises(
            bridge.AsyncPublisherError,
            match=f"failed_check:{fail_on_check}",
        ):
            bridge._commit_native_publisher_batch(
                None,
                batch,
                async_publisher=publisher,
            )
    assert publisher.enqueue_calls == expected_enqueue_calls


def test_mujoco_native_dds_parent_diagnostics_measure_pipe_record_writes(tmp_path):
    import io

    from sim.scripts.mujoco import native_dds_sensors as bridge

    diagnostics = bridge.ParentSensorDiagnostics(
        tmp_path / "parent_sensor_diagnostics.json",
        period_s=0.5,
    )
    stream = io.BytesIO()

    bridge._write_record(
        stream,
        bridge._RECORD_IMU,
        123,
        4,
        b"abcdef",
        1,
        parent_diagnostics=diagnostics,
        diagnostic_record_type="imu",
    )
    diagnostics.force_publish("test")

    snapshot = json.loads(diagnostics.path.read_text(encoding="utf-8"))
    imu = snapshot["record_types"]["imu"]
    assert imu["pipe_write_attempt"] == 1
    assert imu["pipe_write_success"] == 1
    assert imu["pipe_write_error"] == 0
    assert imu["payload_bytes"] == 6
    assert imu["pipe_bytes"] == bridge._HEADER.size + 6
    assert imu["pipe_write_duration_us"]["count"] == 1


def test_mujoco_native_dds_parent_diagnostics_count_pipe_write_errors(tmp_path):
    from sim.scripts.mujoco import native_dds_sensors as bridge

    class BrokenStream:
        def write(self, _value):
            raise OSError("pipe closed")

    diagnostics = bridge.ParentSensorDiagnostics(
        tmp_path / "parent_sensor_diagnostics.json",
        period_s=0.5,
    )

    with pytest.raises(OSError, match="pipe closed"):
        bridge._write_record(
            BrokenStream(),
            bridge._RECORD_IMU,
            123,
            4,
            b"abcdef",
            1,
            parent_diagnostics=diagnostics,
            diagnostic_record_type="imu",
        )

    diagnostics.force_publish("test")
    snapshot = json.loads(diagnostics.path.read_text(encoding="utf-8"))
    imu = snapshot["record_types"]["imu"]
    assert imu["pipe_write_attempt"] == 1
    assert imu["pipe_write_success"] == 0
    assert imu["pipe_write_error"] == 1
    assert imu["pipe_write_duration_us"]["count"] == 1


def test_mujoco_native_dds_parent_diagnostics_measure_flush_by_record_type(tmp_path):
    import io

    from sim.scripts.mujoco import native_dds_sensors as bridge

    diagnostics = bridge.ParentSensorDiagnostics(
        tmp_path / "parent_sensor_diagnostics.json",
        period_s=0.5,
    )
    stream = io.BytesIO()
    bridge._write_record(
        stream,
        bridge._RECORD_IMU,
        123,
        4,
        b"abcdef",
        1,
        parent_diagnostics=diagnostics,
        diagnostic_record_type="imu",
    )
    bridge._write_record(
        stream,
        bridge._RECORD_CLOUD,
        124,
        5,
        b"xyz",
        1,
        parent_diagnostics=diagnostics,
        diagnostic_record_type="cloud",
    )

    diagnostics.force_publish("pending")
    pending_snapshot = json.loads(diagnostics.path.read_text(encoding="utf-8"))
    assert pending_snapshot["flush"]["records_since_flush"]["current"] == 2
    assert pending_snapshot["flush"]["record_types"]["imu"]["bytes_since_flush"][
        "current"
    ] == bridge._HEADER.size + 6

    bridge._flush_native_publisher(stream, parent_diagnostics=diagnostics)
    diagnostics.force_publish("test")

    snapshot = json.loads(diagnostics.path.read_text(encoding="utf-8"))
    flush = snapshot["flush"]
    assert flush["attempt"] == 1
    assert flush["success"] == 1
    assert flush["error"] == 0
    assert flush["duration_us"]["count"] == 1
    assert flush["records_since_flush"]["last"] == 2
    assert flush["bytes_since_flush"]["last"] == 2 * bridge._HEADER.size + 9
    assert flush["record_types"]["imu"]["records_since_flush"]["last"] == 1
    assert flush["record_types"]["imu"]["duration_us"]["count"] == 1
    assert flush["record_types"]["cloud"]["records_since_flush"]["last"] == 1
    assert flush["record_types"]["cloud"]["duration_us"]["count"] == 1
    assert flush["record_types"]["odom_prior"]["duration_us"]["count"] == 0
    assert flush["records_since_flush"]["current"] == 0


def test_mujoco_native_dds_parent_diagnostics_retain_pending_records_on_flush_error(
    tmp_path,
):
    import io

    from sim.scripts.mujoco import native_dds_sensors as bridge

    class BrokenFlush(io.BytesIO):
        def flush(self):
            raise OSError("flush failed")

    diagnostics = bridge.ParentSensorDiagnostics(
        tmp_path / "parent_sensor_diagnostics.json",
        period_s=0.5,
    )
    stream = BrokenFlush()
    bridge._write_record(
        stream,
        bridge._RECORD_IMU,
        123,
        4,
        b"abcdef",
        1,
        parent_diagnostics=diagnostics,
        diagnostic_record_type="imu",
    )

    with pytest.raises(OSError, match="flush failed"):
        bridge._flush_native_publisher(stream, parent_diagnostics=diagnostics)

    diagnostics.force_publish("test")
    snapshot = json.loads(diagnostics.path.read_text(encoding="utf-8"))
    flush = snapshot["flush"]
    assert flush["attempt"] == 1
    assert flush["success"] == 0
    assert flush["error"] == 1
    assert flush["records_since_flush"]["current"] == 1
    assert flush["records_since_flush"]["total"] == 0
    assert flush["record_types"]["imu"]["error"] == 1
    assert flush["record_types"]["imu"]["duration_us"]["count"] == 1


def test_mujoco_native_dds_sensor_bridge_writes_mid360_imu_acceleration_in_g_units():
    import io

    from runtime.msgs.geometry import Quaternion, Vector3
    from runtime.msgs.sensor import Imu
    from sim.scripts.mujoco import native_dds_sensors as bridge

    imu = Imu(
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.1, 0.2, 0.3),
        linear_acceleration=Vector3(0.0, 0.0, bridge._MID360_ACCEL_MPS2_PER_G),
        ts=1.0,
        frame_id=bridge.IMU_FRAME_ID,
    )
    stream = io.BytesIO()

    bridge._write_native_imu(stream, imu, sequence=5)
    payload = stream.getvalue()[bridge._HEADER.size :]
    gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z = bridge._IMU_PAYLOAD.unpack(payload)

    assert gyro_x == pytest.approx(0.1)
    assert gyro_y == pytest.approx(0.2)
    assert gyro_z == pytest.approx(0.3)
    assert acc_x == pytest.approx(0.0)
    assert acc_y == pytest.approx(0.0)
    assert acc_z == pytest.approx(1.0)


def test_mujoco_native_dds_typed_writer_attributes_diagnostics_to_imu(tmp_path):
    import io

    from runtime.msgs.geometry import Quaternion, Vector3
    from runtime.msgs.sensor import Imu
    from sim.scripts.mujoco import native_dds_sensors as bridge

    diagnostics = bridge.ParentSensorDiagnostics(
        tmp_path / "parent_sensor_diagnostics.json",
        period_s=0.5,
    )
    imu = Imu(
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.1, 0.2, 0.3),
        linear_acceleration=Vector3(0.0, 0.0, bridge._MID360_ACCEL_MPS2_PER_G),
        ts=1.0,
        frame_id=bridge.IMU_FRAME_ID,
    )

    bridge._write_native_imu(
        io.BytesIO(),
        imu,
        sequence=5,
        parent_diagnostics=diagnostics,
    )
    diagnostics.force_publish("test")

    snapshot = json.loads(diagnostics.path.read_text(encoding="utf-8"))
    assert snapshot["record_types"]["imu"]["pipe_write_success"] == 1
    assert snapshot["record_types"]["cloud"]["pipe_write_success"] == 0


def test_mujoco_native_dds_sensor_bridge_uses_specific_force_for_raw_imu():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    class State:
        orientation = np.array([0.0, 0.0, 0.0, 1.0])
        imu_gyro = np.array([0.1, 0.2, 0.3])
        imu_linear_acceleration = np.array([0.0, 0.0, -123.0])
        linear_velocity = np.zeros(3)

    imu = bridge._runtime_imu_from_state(State(), 1.0, None, 0.0)

    assert imu.angular_velocity.z == pytest.approx(0.3)
    assert imu.linear_acceleration.x == pytest.approx(0.0)
    assert imu.linear_acceleration.y == pytest.approx(0.0)
    assert imu.linear_acceleration.z == pytest.approx(9.81)


def test_mujoco_native_dds_sensor_bridge_flags_unhealthy_native_slam_status():
    from sim.scripts.mujoco import native_dds_sensors as bridge

    gaps = bridge._slam_health_gaps(
        {
            "state": "TRACKING",
            "localization_quality": 0.2,
            "odometry": {"pose": {"x": 0.0, "y": 0.0, "z": -56.0}},
        },
        min_quality=0.5,
        max_odom_abs_m=100.0,
        max_odom_z_abs_m=20.0,
    )

    assert "native_slam_quality_low:0.200" in gaps
    assert "native_slam_odom_z_out_of_bounds:-56.000" in gaps


def _native_slam_status_payload(**overrides):
    from localization.adapters.status import STATUS_SNAPSHOT_SCHEMA

    payload = {
        "schema_version": STATUS_SNAPSHOT_SCHEMA,
        "runtime_instance_id": "native-slam-test",
        "source_epoch": 1,
        "source": "cpp_cyclone_slam",
        "backend": "fastlio2",
        "mode": "mapping",
        "state": "TRACKING",
        "reason": "tracking",
        "alive": True,
        "has_odom": True,
        "stamp_s": 12.34,
        "confidence": 0.8,
        "localization_quality": 0.7,
        "observation_sequence": 1,
        "odometry": {
            "pose": {
                "x": 1.0,
                "y": 2.0,
                "z": 3.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            },
        },
    }
    payload.update(overrides)
    return payload


def test_native_slam_status_adapter_records_odom_snapshot_stamp():
    from localization.adapters.status import CppSlamStatusAdapterModule

    adapter = CppSlamStatusAdapterModule()
    odometry_seen = []
    status_seen = []
    adapter.odometry.subscribe(odometry_seen.append)
    adapter.localization_status.subscribe(status_seen.append)

    adapter._publish_status_snapshot(_native_slam_status_payload())

    assert odometry_seen[-1].ts == pytest.approx(12.34)
    assert status_seen[-1]["ts"] == pytest.approx(12.34)


def test_native_slam_status_adapter_uses_runtime_default_frames():
    from localization.adapters.status import CppSlamStatusAdapterModule
    from runtime.runtime_interface import TOPICS, topic_default_frame_id

    adapter = CppSlamStatusAdapterModule()
    odometry_seen = []
    adapter.odometry.subscribe(odometry_seen.append)

    adapter._publish_status_snapshot(_native_slam_status_payload())

    assert odometry_seen[-1].frame_id == topic_default_frame_id(TOPICS.odometry)
    assert odometry_seen[-1].child_frame_id == "body"


def test_native_slam_status_adapter_feeds_module_ports_directly(tmp_path):
    from localization.adapters.status import CppSlamStatusAdapterModule
    from runtime.msgs.sensor import PointCloud2
    from runtime.runtime_interface import TOPICS, topic_default_frame_id

    cloud_dir = tmp_path / "clouds"
    cloud_dir.mkdir()
    (cloud_dir / "registered_cloud.bin").write_bytes(
        PointCloud2(
            points=[[1.0, 2.0, 3.0, 0.5]],
            ts=12.34,
            frame_id=topic_default_frame_id(TOPICS.registered_cloud),
        ).encode()
    )
    (cloud_dir / "map_cloud.bin").write_bytes(
        PointCloud2(points=[[4.0, 5.0, 6.0, 0.7]], ts=12.35, frame_id=topic_default_frame_id(TOPICS.map_cloud)).encode()
    )
    adapter = CppSlamStatusAdapterModule(cloud_snapshot_dir=str(cloud_dir))
    odometry_seen = []
    registered_seen = []
    map_seen = []
    adapter.odometry.subscribe(odometry_seen.append)
    adapter.registered_cloud.subscribe(registered_seen.append)
    adapter.map_cloud_frame.subscribe(map_seen.append)

    adapter._publish_status_snapshot(_native_slam_status_payload())
    adapter._poll_cloud_snapshots()

    assert odometry_seen[-1].frame_id == topic_default_frame_id(TOPICS.odometry)
    assert registered_seen[-1].frame_id == topic_default_frame_id(TOPICS.registered_cloud)
    assert map_seen[-1].frame_id == topic_default_frame_id(TOPICS.map_cloud)
    assert map_seen[-1].mode == "FULL"


def test_native_slam_status_adapter_ignores_status_without_odom_pose():
    from localization.adapters.status import CppSlamStatusAdapterModule

    adapter = CppSlamStatusAdapterModule()
    odometry_seen = []
    status_seen = []
    adapter.odometry.subscribe(odometry_seen.append)
    adapter.localization_status.subscribe(status_seen.append)
    payload = _native_slam_status_payload(odometry={"frame_id": "odom"})

    adapter._publish_status_snapshot(payload)

    assert odometry_seen == []
    assert status_seen[-1]["has_odom"] is True


def test_mujoco_world_registry_includes_industrial_demo_scene():
    from drivers.sim.mujoco.driver import WORLDS

    world_file = WORLDS["industrial_demo"]

    assert world_file.name == "industrial_demo_scene.xml"
    assert world_file.is_file()
    assert "robot_placeholder" in world_file.read_text(encoding="utf-8")


def test_mujoco_world_registry_includes_product_industrial_park_scene():
    from drivers.sim.mujoco.driver import WORLDS

    world_file = WORLDS["industrial_park"]
    text = world_file.read_text(encoding="utf-8")

    assert world_file.name == "industrial_park_scene.xml"
    assert world_file.is_file()
    assert text.isascii()
    assert "robot_placeholder" in text


class _FakeEngine:
    def __init__(
        self,
        robot_config,
        world_config,
        lidar_config,
        camera_configs,
        headless,
        drive_mode="policy",
        discrete_ray_config=None,
        **_kwargs,
    ):
        self.robot_config = robot_config
        self.world_config = world_config
        self.lidar_config = lidar_config
        self.camera_configs = camera_configs
        self.headless = headless
        self.drive_mode = drive_mode
        self.discrete_ray_config = discrete_ray_config
        self.loaded_xml_path = ""
        self.reset_called = False

    def load(self, xml_path: str = "", **kwargs):
        self.loaded_xml_path = xml_path

    def reset(self):
        self.reset_called = True


def test_mujoco_driver_setup_uses_selected_scene_and_real_robot(monkeypatch):
    import sim.compat.engine.mujoco.engine as mujoco_engine

    monkeypatch.setitem(sys.modules, "mujoco", types.SimpleNamespace(__version__="test"))
    monkeypatch.setattr(mujoco_engine, "MuJoCoEngine", _FakeEngine)

    driver = MujocoDriverModule(
        world="open_field",
        render=False,
        enable_camera=True,
    )
    driver.setup()

    expected_world = (
        Path(__file__).resolve().parents[2]
        / "sim"
        / "packages"
        / "worlds"
        / "open_field"
        / "physics"
        / "open_field.xml"
    )

    assert driver._engine is not None
    assert Path(driver._engine.loaded_xml_path) == expected_world
    assert Path(driver._engine.world_config.scene_xml) == expected_world
    assert Path(driver._engine.robot_config.robot_xml).name == "thunderv4.xml"
    assert Path(driver._engine.robot_config.robot_xml).exists()
    assert driver._engine.robot_config.base_body_name == "base_link"
    assert driver._engine.lidar_config.body_name == "lidar_link"
    assert driver._engine.drive_mode == "policy"
    assert driver._engine.reset_called is True
    assert len(driver._engine.camera_configs) == 1


def test_mujoco_driver_setup_accepts_absolute_world_path(monkeypatch, tmp_path):
    import sim.compat.engine.mujoco.engine as mujoco_engine

    world = tmp_path / "custom.xml"
    world.write_text("<mujoco><worldbody /></mujoco>\n", encoding="utf-8")
    monkeypatch.setitem(sys.modules, "mujoco", types.SimpleNamespace(__version__="test"))
    monkeypatch.setattr(mujoco_engine, "MuJoCoEngine", _FakeEngine)

    driver = MujocoDriverModule(
        world=str(world),
        render=False,
        enable_camera=False,
        drive_mode="kinematic",
    )
    driver.setup()

    assert driver._engine is not None
    assert Path(driver._engine.loaded_xml_path) == world.resolve()
    assert Path(driver._engine.world_config.scene_xml) == world.resolve()


def test_mujoco_driver_uses_scene_placeholder_start_pose(monkeypatch):
    import sim.compat.engine.mujoco.engine as mujoco_engine

    monkeypatch.setitem(sys.modules, "mujoco", types.SimpleNamespace(__version__="test"))
    monkeypatch.setattr(mujoco_engine, "MuJoCoEngine", _FakeEngine)

    driver = MujocoDriverModule(
        world="building_scene",
        render=False,
        enable_camera=False,
    )
    driver.setup()

    assert driver._engine is not None
    assert driver._engine.robot_config.init_position == [2.0, 3.0, 0.5]
    assert Path(driver._engine.robot_config.policy_onnx).name == "policy_1119.onnx"


def test_mujoco_driver_defaults_to_thunderv4_policy():
    import drivers.sim.mujoco.driver as driver_mod

    assert len(driver_mod._POLICY_CANDIDATES) == 1
    assert driver_mod._POLICY_CANDIDATES[0].name == "policy_1119.onnx"
    assert driver_mod._EXPLICIT_POLICY_CANDIDATES[0].name == "policy_251119.onnx"


def test_root_operation_entrypoint_is_unique_and_uses_current_release_paths():
    repo_root = Path(__file__).resolve().parents[2]
    canonical_entry = repo_root / "scripts" / "lingtu"
    retired_entry = repo_root / "scripts" / ("lingtu" + ".sh")
    installer = (repo_root / "scripts" / "deploy" / "install_native_release.sh").read_text(
        encoding="utf-8"
    )
    scripts_index = (repo_root / "scripts" / "README.md").read_text(encoding="utf-8")
    release_guide = (repo_root / "docs" / "04-deployment" / "OTA_GUIDE.md").read_text(
        encoding="utf-8"
    )

    assert canonical_entry.is_file()
    assert not retired_entry.exists()
    assert not (repo_root / "scripts" / "ota").exists()
    assert "navigation_run.launch.py" not in installer
    assert "navigation_bringup.launch.py" not in installer
    assert "launch/subsystems/planning.launch.py" not in installer

    assert "`scripts/lingtu` only executes `python -m lingtu.control`" in scripts_index
    assert "scripts/ota/" not in scripts_index
    assert "scripts/deploy/package_native_release.sh" in release_guide
    assert "python3 -m lingtu.control" in installer

def test_sim_boundary_indexes_document_stable_contracts():
    repo_root = Path(__file__).resolve().parents[2]
    indexes = {
        "simulation": repo_root / "sim" / "README.md",
        "scripts": repo_root / "sim" / "scripts" / "README.md",
        "engine": repo_root / "sim" / "compat" / "engine" / "README.md",
        "repository": repo_root / "docs" / "REPO_LAYOUT.md",
    }
    texts = {name: path.read_text(encoding="utf-8") for name, path in indexes.items()}

    assert "Stable Root Contract" in texts["simulation"]
    assert "sim/scripts/mujoco/*" in texts["simulation"]
    assert "Native typed DDS sensors" in texts["simulation"]
    assert "prove field readiness by itself." in texts["simulation"]
    assert "## Product acceptance" in texts["scripts"]
    assert "## Sensors, mapping, and evidence" in texts["scripts"]
    assert "not the canonical generic simulation Runtime" in texts["engine"]
    assert "remains hardware-free" in texts["engine"]
    assert "| `sim/` | Simulation engines" in texts["repository"]

    boundary_markers = {
        "evaluation/data": ("offline replay inputs", "generated validation evidence", "artifacts/"),
        "packages/sensors": (
            "Package assets stay with their owning SensorPackage",
            "sim/packages/sensors/livox/mid360/assets/mid360.npy",
            "startup side effects",
        ),
    }
    for folder, markers in boundary_markers.items():
        text = (repo_root / "sim" / folder / "README.md").read_text(encoding="utf-8")
        assert all(marker in text for marker in markers)


def test_mujoco_driver_resolves_repo_relative_paths(monkeypatch, tmp_path):
    import drivers.sim.mujoco.driver as driver_mod

    sim_root = tmp_path / "sim"
    policy = sim_root / "packages" / "controllers" / "doso" / "thunder_v4" / "locomotion" / "policy.onnx"
    policy.parent.mkdir(parents=True)
    policy.write_bytes(b"policy")

    monkeypatch.setattr(driver_mod, "_SIM_ROOT", sim_root)
    explicit = MujocoDriverModule(
        policy_path="packages/controllers/doso/thunder_v4/locomotion/policy.onnx",
        drive_mode="policy",
    )
    assert Path(explicit._policy_path) == policy.resolve()

    missing = driver_mod._resolve_sim_path("sim/packages/controllers/doso/thunder_v4/missing.onnx")
    assert Path(missing) == (
        sim_root / "packages" / "controllers" / "doso" / "thunder_v4" / "missing.onnx"
    ).resolve()


def test_mujoco_driver_applies_explicit_cmd_velocity_limits(monkeypatch):
    import sim.compat.engine.mujoco.engine as mujoco_engine

    monkeypatch.setitem(sys.modules, "mujoco", types.SimpleNamespace(__version__="test"))
    monkeypatch.setattr(mujoco_engine, "MuJoCoEngine", _FakeEngine)

    driver = MujocoDriverModule(
        world="open_field",
        render=False,
        enable_camera=False,
        max_linear_vel=0.3,
        max_angular_vel=0.2,
    )
    driver.setup()

    assert driver._engine is not None
    assert driver._engine.robot_config.max_linear_vel == pytest.approx(0.3)
    assert driver._engine.robot_config.max_angular_vel == pytest.approx(0.2)


def test_mujoco_driver_kinematic_mode_disables_policy(monkeypatch):
    import sim.compat.engine.mujoco.engine as mujoco_engine

    monkeypatch.setitem(sys.modules, "mujoco", types.SimpleNamespace(__version__="test"))
    monkeypatch.setattr(mujoco_engine, "MuJoCoEngine", _FakeEngine)

    driver = MujocoDriverModule(
        world="open_field",
        render=False,
        enable_camera=False,
        drive_mode="kinematic",
    )
    driver.setup()

    assert driver._engine is not None
    assert driver._engine.drive_mode == "kinematic"
    assert driver._engine.robot_config.policy_onnx == ""


def test_mujoco_driver_stop_signal_zero_clears_soft_stop_latch():
    driver = MujocoDriverModule(world="open_field", render=False, enable_camera=False)
    driver.cmd_vel.subscribe(driver._on_cmd_vel)
    driver.stop_signal.subscribe(driver._on_stop)

    driver.stop_signal._deliver(1)
    driver.cmd_vel._deliver(Twist(linear=Vector3(0.4, 0.0, 0.0)))
    assert driver._stopped is True
    assert driver._cmd_vx == 0.0

    driver.stop_signal._deliver(0)
    driver.cmd_vel._deliver(Twist(linear=Vector3(0.4, 0.0, 0.0)))
    assert driver._stopped is False
    assert driver._cmd_vx == pytest.approx(0.4)


def test_mujoco_policy_runner_resolves_legacy_history_contracts():
    from sim.compat.engine.mujoco.robot_controller import OBS_DIM, PolicyRunner

    assert PolicyRunner._resolve_history_len(OBS_DIM) == 1
    assert PolicyRunner._resolve_history_len(OBS_DIM * 5) == 5


def test_mujoco_policy_runner_rejects_unknown_obs_contract():
    from sim.compat.engine.mujoco.robot_controller import PolicyRunner, UnsupportedPolicyInputError

    with pytest.raises(UnsupportedPolicyInputError, match="76-D input"):
        PolicyRunner._resolve_history_len(76)


def test_mujoco_policy_runner_does_not_pad_or_truncate_obs():
    from sim.compat.engine.mujoco.robot_controller import OBS_DIM, PolicyRunner

    runner = object.__new__(PolicyRunner)
    runner._history_len = 1

    obs = np.arange(OBS_DIM, dtype=np.float32)
    adapted = runner._adapt_obs_to_policy_input(obs)

    assert adapted.shape == (OBS_DIM,)
    assert np.allclose(adapted, obs)

    with pytest.raises(ValueError, match="expected one 57-D observation"):
        runner._adapt_obs_to_policy_input(np.arange(76, dtype=np.float32))


def test_mujoco_policy_runner_clamp_matches_brainstem_noop():
    from sim.compat.engine.mujoco.robot_controller import PolicyRunner

    action = np.array(
        [2.0, -3.0, 4.0, -2.0, 3.0, -4.0, 1.8, -2.8, 3.8, -1.8, 2.8, -3.8, 0.8, -0.9, 1.1, -1.2],
        dtype=np.float64,
    )

    clamped = PolicyRunner.clamp_action(action)

    assert np.allclose(clamped, action)
    assert clamped is not action


def test_mujoco_thunderv4_policy_identifier_accepts_canonical_layout():
    from sim.compat.engine.mujoco.robot_controller import _is_thunderv4_policy

    assert _is_thunderv4_policy(
        Path("sim/packages/controllers/doso/thunder_v4/locomotion/policy/policy_1119.onnx")
    )
    assert _is_thunderv4_policy(
        Path("sim/packages/controllers/doso/thunder_v4/locomotion/policy/pose_flat_low_kpkd_microterrain_model29600_policy.onnx")
    )
    assert _is_thunderv4_policy(Path("pose_flat_low_kpkd_microterrain_model29600_policy.onnx"))
    assert not _is_thunderv4_policy(Path("model/policy_251119.onnx"))


def test_mujoco_thunderv4_policy_1119_uses_history_runner():
    pytest.importorskip("onnxruntime")
    from sim.compat.engine.mujoco.robot_controller import (
        OBS_DIM,
        THUNDERV4_STANDING_POSE,
        PolicyRunner,
        load_policy_runner,
    )

    policy = (
        Path(__file__).resolve().parents[2]
        / "sim"
        / "packages"
        / "controllers"
        / "doso"
        / "thunder_v4"
        / "locomotion"
        / "policy"
        / "policy_1119.onnx"
    )
    if not policy.exists():
        pytest.skip(f"ThunderV4 ONNX policy missing: {policy}")

    runner = load_policy_runner(str(policy))
    action = runner.infer(np.zeros(OBS_DIM, dtype=np.float32))

    assert type(runner) is PolicyRunner
    assert runner._history_len == 5
    assert action.shape == (16,)
    assert np.allclose(action, THUNDERV4_STANDING_POSE)


def test_mujoco_policy_idle_command_detection():
    from sim.compat.engine.mujoco.engine import MuJoCoEngine

    engine = MuJoCoEngine()

    assert engine._is_idle_policy_command(np.array([0.0, 0.0, 0.0]))
    assert engine._is_idle_policy_command(np.array([1e-5, 0.0, 0.0]))
    assert not engine._is_idle_policy_command(np.array([1e-3, 0.0, 0.0]))


def test_mujoco_policy_resume_does_not_restart_startup_hold(monkeypatch):
    from sim.compat.engine.mujoco.engine import MuJoCoEngine

    class FakePolicy:
        run_at_idle = False
        zero_wheels_at_idle = True

        def __init__(self):
            self.reset_calls = 0
            self.warm_up_calls = 0

        def reset(self):
            self.reset_calls += 1

        def warm_up(self, gyro, projected_gravity, joint_position, joint_velocity):
            self.warm_up_calls += 1

        def build_obs(self, gyro, projected_gravity, direction, joint_position, joint_velocity):
            return np.zeros(1, dtype=np.float32)

        def infer(self, obs):
            return np.zeros(16, dtype=np.float32)

    engine = MuJoCoEngine()
    policy = FakePolicy()
    engine._policy = policy
    engine._policy_idle_hold = True
    engine._cmd_vel[:] = [0.2, 0.0, 0.0]
    monkeypatch.setattr(engine, "_get_imu", lambda: (np.zeros(3), np.zeros(3)))
    monkeypatch.setattr(engine, "_get_joint_state", lambda: (np.zeros(16), np.zeros(16)))
    writes = []
    monkeypatch.setattr(engine, "_write_leg_ctrl", lambda action: writes.append(action.copy()))

    engine._step_policy()

    assert policy.reset_calls == 0
    assert policy.warm_up_calls == 1
    assert engine._policy_idle_hold is False
    assert len(writes) == 1


def test_mujoco_camera_preserves_metric_depth_output():
    from sim.compat.engine.mujoco.camera import MuJoCoCamera

    raw = np.array([[0.4, 2.5, 25.0]], dtype=np.float32)
    depth = MuJoCoCamera._coerce_depth_meters(raw, near=0.1, far=10.0)

    assert np.allclose(depth, np.array([[0.4, 2.5, 10.0]], dtype=np.float32))


def test_mujoco_driver_default_pose_emits_lidar_points():
    pytest.importorskip("mujoco")

    driver = MujocoDriverModule(
        world="building_scene",
        render=False,
        enable_camera=False,
        drive_mode="kinematic",
    )
    driver.setup()
    try:
        assert driver._engine is not None
        pts = driver._engine.get_lidar_points()
        assert pts is not None
        assert len(pts) > 0
    finally:
        if driver._engine is not None:
            driver._engine.close()
            driver._engine = None


def test_mujoco_driver_kinematic_cmd_vel_moves_free_base():
    pytest.importorskip("mujoco")
    from sim.compat.engine.core.engine import VelocityCommand

    driver = MujocoDriverModule(
        world="open_field",
        render=False,
        enable_camera=False,
        drive_mode="kinematic",
    )
    driver.setup()
    try:
        assert driver._engine is not None
        assert driver._engine.drive_mode == "kinematic"
        assert driver._engine.has_policy is False

        start = driver._engine.get_robot_state().position.copy()
        for _ in range(100):
            state = driver._engine.step(VelocityCommand(linear_x=0.5))

        moved = math.hypot(state.position[0] - start[0], state.position[1] - start[1])
        expected_distance = 0.5 * 100 * driver._engine.control_dt
        assert moved == pytest.approx(expected_distance, abs=0.05)
    finally:
        if driver._engine is not None:
            driver._engine.close()
            driver._engine = None


def test_mujoco_kinematic_step_sanitizes_post_physics_base_jitter(monkeypatch):
    import sys
    from types import SimpleNamespace

    from sim.compat.engine.core.engine import VelocityCommand
    from sim.compat.engine.mujoco.engine import MuJoCoEngine

    engine = MuJoCoEngine(drive_mode="kinematic")
    engine._model = SimpleNamespace()
    engine._data = SimpleNamespace(
        qpos=np.array([0.0, 0.0, 0.42, 1.0, 0.0, 0.0, 0.0], dtype=float),
        qvel=np.zeros(6, dtype=float),
        xmat=np.array([[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]], dtype=float),
        ctrl=np.zeros(0, dtype=float),
    )
    engine._root_qposadr = 0
    engine._root_dofadr = 0
    engine._base_body_id = 0
    engine._leg_joint_ids = []
    engine._leg_actuator_ids = []
    engine._physics_dt = 0.01
    engine._control_dt = 0.02
    engine._step_policy = lambda: None

    def _fake_mj_step(_model, data):
        data.qpos[0] += data.qvel[0] * engine._physics_dt
        data.qpos[1] += data.qvel[1] * engine._physics_dt
        data.qpos[2] -= 0.03
        data.qvel[3] = 0.4
        data.qvel[4] = -0.3

    monkeypatch.setitem(sys.modules, "mujoco", SimpleNamespace(mj_step=_fake_mj_step))

    state = engine.step(VelocityCommand(linear_x=0.5, angular_z=0.2))

    assert state.position[0] > 0.0
    assert state.position[2] == pytest.approx(engine._robot_cfg.init_position[2])
    assert state.angular_velocity.tolist() == pytest.approx([0.0, 0.0, 0.2])
    assert state.imu_gyro.tolist() == pytest.approx([0.0, 0.0, 0.2])


def test_mujoco_kinematic_step_integrates_yaw_from_cmd_vel(monkeypatch):
    import sys
    from types import SimpleNamespace

    from sim.compat.engine.core.engine import VelocityCommand
    from sim.compat.engine.mujoco.engine import MuJoCoEngine

    engine = MuJoCoEngine(drive_mode="kinematic")
    engine._model = SimpleNamespace()
    engine._data = SimpleNamespace(
        qpos=np.array([0.0, 0.0, 0.42, 1.0, 0.0, 0.0, 0.0], dtype=float),
        qvel=np.zeros(6, dtype=float),
        xmat=np.array([[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]], dtype=float),
        ctrl=np.zeros(0, dtype=float),
    )
    engine._root_qposadr = 0
    engine._root_dofadr = 0
    engine._base_body_id = 0
    engine._leg_joint_ids = []
    engine._leg_actuator_ids = []
    engine._physics_dt = 0.01
    engine._control_dt = 0.02
    engine._step_policy = lambda: None

    def _fake_mj_step(_model, data):
        # Reproduce the failure mode: physics/contact dynamics do not advance
        # yaw in proportion to the commanded free-base angular velocity.
        data.qvel[5] = 0.0

    monkeypatch.setitem(sys.modules, "mujoco", SimpleNamespace(mj_step=_fake_mj_step))

    state = engine.step(VelocityCommand(angular_z=0.2))
    _, _, yaw = _rpy_from_xyzw(state.orientation)

    assert yaw == pytest.approx(0.2 * engine._control_dt)
    assert state.angular_velocity.tolist() == pytest.approx([0.0, 0.0, 0.2])
    assert state.imu_gyro.tolist() == pytest.approx([0.0, 0.0, 0.2])


def _real_policy_path_or_skip() -> Path:
    policy_path = (
        Path(__file__).resolve().parents[2]
        / "sim" / "packages" / "controllers"
        / "doso"
        / "thunder_v4"
        / "locomotion"
        / "policy"
        / "policy_1119.onnx"
    )
    if not policy_path.is_file():
        pytest.skip("default MuJoCo policy is not available in this checkout")
    return policy_path


def _rpy_from_xyzw(q) -> tuple[float, float, float]:
    x, y, z, w = [float(v) for v in q]
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def test_mujoco_policy_cmd_vel_produces_stable_motion_when_real_policy_available():
    pytest.importorskip("mujoco")
    pytest.importorskip("onnxruntime")
    from sim.compat.engine.core.engine import VelocityCommand

    policy_path = _real_policy_path_or_skip()
    driver = MujocoDriverModule(
        world="open_field",
        render=False,
        enable_camera=False,
        drive_mode="policy",
        policy_path=str(policy_path),
    )
    driver.setup()
    try:
        assert driver._engine is not None
        assert driver._engine.drive_mode == "policy"
        assert driver._engine.has_policy is True

        start = driver._engine.get_robot_state()
        start_xy = np.array(start.position[:2], dtype=float)
        z_values = []
        roll_values = []
        pitch_values = []
        for _ in range(300):
            state = driver._engine.step(VelocityCommand(linear_x=0.2))
            roll, pitch, _ = _rpy_from_xyzw(state.orientation)
            z_values.append(float(state.position[2]))
            roll_values.append(abs(roll))
            pitch_values.append(abs(pitch))
            assert np.isfinite(state.position).all()
            assert np.isfinite(state.orientation).all()

        end = driver._engine.get_robot_state()
        moved = float(np.linalg.norm(np.array(end.position[:2], dtype=float) - start_xy))

        assert moved > 0.20
        assert min(z_values) > 0.35
        assert max(z_values) < 0.50
        assert max(roll_values) < 0.20
        assert max(pitch_values) < 0.20
    finally:
        if driver._engine is not None:
            driver._engine.close()
            driver._engine = None


@pytest.mark.parametrize(("command_vy", "expected_sign"), [(0.2, 1.0), (-0.2, -1.0)])
def test_mujoco_policy_lateral_command_matches_body_left_convention(
    command_vy: float,
    expected_sign: float,
):
    pytest.importorskip("mujoco")
    pytest.importorskip("onnxruntime")
    from sim.compat.engine.core.engine import VelocityCommand

    driver = MujocoDriverModule(
        world="open_field",
        render=False,
        enable_camera=False,
        drive_mode="policy",
        policy_path=str(_real_policy_path_or_skip()),
    )
    driver.setup()
    try:
        assert driver._engine is not None
        start = driver._engine.get_robot_state()
        start_xy = np.asarray(start.position[:2], dtype=float)
        _, _, start_yaw = _rpy_from_xyzw(start.orientation)
        for _ in range(200):
            end = driver._engine.step(VelocityCommand(linear_y=command_vy))

        displacement = np.asarray(end.position[:2], dtype=float) - start_xy
        body_left = (
            -float(displacement[0]) * math.sin(start_yaw)
            + float(displacement[1]) * math.cos(start_yaw)
        )
        body_forward = (
            float(displacement[0]) * math.cos(start_yaw)
            + float(displacement[1]) * math.sin(start_yaw)
        )

        assert expected_sign * body_left > 0.20
        assert abs(body_forward) < abs(body_left) * 0.25
    finally:
        if driver._engine is not None:
            driver._engine.close()
            driver._engine = None


def test_sim_scene_observer_emits_building_scene_stairs():
    from perception.detection.sim_scene_observer import SimSceneObserver

    class _Intrinsics:
        fx = 415.7
        fy = 415.7
        cx = 320.0
        cy = 240.0
        width = 640
        height = 480

    tf = np.eye(4, dtype=np.float32)
    tf[:3, :3] = np.array(
        [
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    tf[:3, 3] = [2.0, 3.0, 0.5]
    observer = SimSceneObserver(world="building_scene")

    detections = observer.observe(tf, _Intrinsics(), text_prompt="stairs . goal")

    assert any(det.label == "stairs" for det in detections)


def test_sim_scene_observer_respects_live_forward_axis_convention():
    from perception.detection.sim_scene_observer import SimSceneObserver

    class _Intrinsics:
        fx = 415.7
        fy = 415.7
        cx = 320.0
        cy = 240.0
        width = 640
        height = 480

    tf = np.eye(4, dtype=np.float32)
    tf[:3, :3] = np.eye(3, dtype=np.float32)
    tf[:3, 3] = [2.0, 3.0, 0.5]
    observer = SimSceneObserver(world="building_scene")

    detections = observer.observe(tf, _Intrinsics(), text_prompt="stairs . goal")

    assert any(det.label == "stairs" for det in detections)
