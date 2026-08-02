import importlib.util
import json
import math
import os
import subprocess
import sys
import time
import types
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import pytest

from drivers.sim.mujoco.driver import MujocoDriverModule
from lingtu.assembly.products.thunder import thunder_blueprint
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.tests.numpy_guard import import_numpy_or_skip
from sim.engine.core.robot import RobotConfig

pytestmark = [pytest.mark.sim]

np = import_numpy_or_skip()

_ROS2_AVAILABLE = importlib.util.find_spec("rclpy") is not None


def test_default_nova_dog_resolves_real_robot_model():
    sim_root = Path(__file__).resolve().parents[2] / "sim"
    cfg = RobotConfig.default_nova_dog().resolve_paths(base_dir=str(sim_root))

    assert Path(cfg.robot_xml).as_posix().endswith("robots/thunderv4/mjcf/thunderv4.xml")
    assert Path(cfg.robot_xml).exists()
    assert Path(cfg.policy_onnx).name == "pose_flat_low_kpkd_microterrain_model29600_policy.pt"
    assert cfg.base_body_name == "base_link"
    assert cfg.lidar_body_name == "lidar_link"
    assert cfg.leg_act_offset == 0
    assert cfg.leg_joint_names[0] == "FR_hip_joint"


def test_thunderv4_model_exposes_lidar_site_imu_package():
    model_path = Path(__file__).resolve().parents[2] / "sim" / "robots" / "thunderv4" / "mjcf" / "thunderv4.xml"
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
    from sim.engine.mujoco.engine import MuJoCoEngine

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


def test_default_nova_dog_resolves_paths_from_engine_core_default():
    sim_root = Path(__file__).resolve().parents[2] / "sim"
    cfg = RobotConfig.default_nova_dog().resolve_paths()

    assert Path(cfg.robot_xml).exists()
    assert Path(cfg.robot_xml).is_relative_to(sim_root)
    assert Path(cfg.robot_xml).name == "thunderv4.xml"
    assert "sim/sim" not in Path(cfg.robot_xml).as_posix()


def test_mujoco_driver_splits_body_lidar_cloud_from_world_map_cloud():
    from drivers.sim.mujoco.driver import _world_points_to_body_frame
    from runtime.runtime_interface import FRAMES

    pts = np.array([[0.0, 1.0, 0.0, 0.5]], dtype=np.float32)
    yaw_90_xyzw = np.array([0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)])

    body_pts = _world_points_to_body_frame(pts, np.zeros(3), yaw_90_xyzw)

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
    from sim.scripts.mujoco import native_dds_sensors as bridge

    yaw = math.pi / 2.0
    state = types.SimpleNamespace(
        position=np.array([10.0, 20.0, 0.5], dtype=np.float64),
        orientation=np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]),
    )
    world = np.array([[10.0, 21.0, 0.5, 100.0]], dtype=np.float32)

    body = bridge._world_xyzi_to_body_xyzi(world, state)

    np.testing.assert_allclose(body[0], [1.0, 0.0, 0.0, 100.0], atol=1e-6)


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
    bridge_source = Path("sim/scripts/mujoco/native_dds_sensors.py").read_text(encoding="utf-8")
    sdk_source = Path("src/drivers/real/lidar/sdk2_stream/main.cpp").read_text(encoding="utf-8")
    runtime_source = Path("src/localization/slam/cpp/cyclone_runtime.cpp").read_text(encoding="utf-8")
    fastlio_source = Path("src/localization/slam/cpp/fastlio.cpp").read_text(encoding="utf-8")
    config_source = Path("src/localization/fastlio2/config/mid360_mujoco_native_dds.yaml").read_text(encoding="utf-8")
    topics_source = Path("src/message/cpp/dds_topics.hpp").read_text(encoding="utf-8")

    assert "_RECORD_ODOM_PRIOR = 3" in bridge_source
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
    controller.record_lidar_subscan_drop()
    controller.record_lidar_frame_drops(2)
    controller.yield_if_due()
    assert sleeps == [0.0]

    # Catching up advances simulated dynamics; once lag is back inside the
    # budget, the next observation is published instead of being dropped.
    assert controller.should_drop_sensor_tick(10.07, monotonic_now_s=2000.10) is False
    assert controller.should_drop_sensor_tick(10.10, monotonic_now_s=2000.20) is True

    stats = controller.stats(10.15, monotonic_now_s=2000.20)
    assert stats["strategy"] == "small_step_dynamics_drop_intermediate_sensor_ticks"
    assert stats["catch_up_events"] == 2
    assert stats["dropped_imu_ticks"] == 3
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

    from sim.engine.mujoco.engine import MuJoCoEngine

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
    from sim.engine.mujoco.lidar import MuJoCoLidar

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
    cfg = Path("src/localization/fastlio2/config/mid360_mujoco_native_dds.yaml")
    text = cfg.read_text(encoding="utf-8")

    assert "acc_scale: 9.80665" in text
    assert "t_il: [0.0, 0.0, 0.0]" in text
    assert "navigation_body_from_imu_translation: [-0.30638, 0.0, 0.19417]" in text
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


def test_mujoco_portable_imu_uses_runtime_frame_and_acceleration():
    from drivers.sim.mujoco.adapter import MujocoPortableAdapter
    from sim.engine.core.engine import RobotState

    state = RobotState(
        position=np.zeros(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        linear_velocity=np.zeros(3),
        angular_velocity=np.zeros(3),
        joint_positions=np.zeros(16),
        joint_velocities=np.zeros(16),
        imu_gyro=np.array([0.1, -0.2, 0.3]),
        imu_projected_gravity=np.array([0.0, 0.0, -1.0]),
        imu_linear_acceleration=np.array([1.0, 2.0, 9.8]),
    )

    imu = MujocoPortableAdapter._imu_from_state(state, 12.5)

    assert imu.frame_id == "lidar_link"
    assert imu.angular_velocity.z == pytest.approx(0.3)
    assert imu.linear_acceleration.x == pytest.approx(1.0)
    assert imu.linear_acceleration.y == pytest.approx(2.0)
    assert imu.linear_acceleration.z == pytest.approx(9.8)


def test_thunder_v3_urdf_xml_assets_are_current_and_resolvable():
    sim_root = Path(__file__).resolve().parents[2] / "sim"
    urdf_path = sim_root / "assets" / "urdf" / "thunder_v3.urdf"
    xml_path = sim_root / "assets" / "xml" / "thunder_v3.xml"
    compat_urdf_path = sim_root / "robots" / "thunder.urdf"

    assert urdf_path.exists()
    assert xml_path.exists()
    assert compat_urdf_path.exists()
    assert xml_path.read_bytes() == urdf_path.read_bytes()

    root = ET.parse(urdf_path).getroot()
    compat_root = ET.parse(compat_urdf_path).getroot()
    assert root.attrib["name"] == "thunder_v3"
    assert compat_root.attrib["name"] == "thunder_v3"

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
    for mesh in compat_root.findall(".//mesh"):
        mesh_path = (compat_urdf_path.parent / mesh.attrib["filename"]).resolve()
        assert mesh_path.exists(), mesh.attrib["filename"]


def test_thunder_v3_mjcf_runtime_keeps_lingtu_sensor_and_control_contracts():
    sim_root = Path(__file__).resolve().parents[2] / "sim"
    upstream_path = sim_root / "assets" / "mjcf" / "thunder_v3_mujoco.xml"
    runtime_path = sim_root / "assets" / "mjcf" / "thunder_v3_lingtu.xml"

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
    lidar_processor = (root / "src" / "slam" / "fastlio2" / "src" / "map_builder" / "lidar_processor.cpp").read_text(
        encoding="utf-8"
    )
    ieskf = (root / "src" / "slam" / "fastlio2" / "src" / "map_builder" / "ieskf.cpp").read_text(encoding="utf-8")
    lio_node = (root / "src" / "slam" / "fastlio2" / "src" / "lio_node.cpp").read_text(encoding="utf-8")

    assert "setMaxIter(static_cast<size_t>(std::max(1, m_config.ieskf_max_iter)))" in lidar_processor
    assert "setDegeneracyGuard(" in lidar_processor
    assert "degeneracy_max_update_dof" in lio_node
    assert "max_update_translation_m" in lio_node
    assert "max_update_rotation_rad" in lio_node
    assert "reject_nonconverged_update" in lio_node
    assert "reject_degenerate_nonconverged_update" in lio_node
    assert "vertical_velocity_constraint_enabled" in lio_node
    assert "injectVerticalVelocityConstraint" in ieskf
    assert "injectVerticalVelocityConstraint" in (
        root / "src" / "slam" / "fastlio2" / "src" / "map_builder" / "imu_processor.cpp"
    ).read_text(encoding="utf-8")
    assert "too_many_degenerate_dofs" in ieskf
    assert "update_translation_too_large" in ieskf
    assert "update_rotation_too_large" in ieskf
    assert "m_reject_nonconverged_update" in ieskf
    assert "m_reject_degenerate_nonconverged_update" in ieskf
    assert "P_candidate.diagonal().array() <= 0.0" in ieskf
    assert "P_MIN" in ieskf


def test_mujoco_fastlio2_live_gate_converts_world_cloud_to_sensor_frame():
    from sim.scripts.mujoco.live_gate import _world_xyzi_to_sensor_xyzi

    class FakeData:
        xpos = [None, np.array([1.0, 2.0, 0.5])]
        xmat = [None, np.eye(3).reshape(-1)]

    class FakeEngine:
        _data = FakeData()
        _lidar_body_id = 1

    world_cloud = np.array(
        [
            [2.0, 4.0, 1.5, 42.0],
            [0.5, 1.0, 0.0, 12.0],
        ],
        dtype=np.float32,
    )

    sensor_cloud = _world_xyzi_to_sensor_xyzi(FakeEngine(), world_cloud)

    np.testing.assert_allclose(sensor_cloud[:, :3], [[1.0, 2.0, 1.0], [-0.5, -1.0, -0.5]])
    np.testing.assert_allclose(sensor_cloud[:, 3], [42.0, 12.0])


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


def test_mujoco_fastlio2_live_gate_converts_sensor_cloud_to_body_frame():
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


def test_mujoco_fastlio2_live_gate_stationary_imu_specific_force_points_up():
    from sim.scripts.mujoco.live_gate import _specific_force_body

    state = types.SimpleNamespace(
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        linear_velocity=np.zeros(3),
    )

    acc = _specific_force_body(state, np.zeros(3), 0.02)

    np.testing.assert_allclose(acc, [0.0, 0.0, 9.81], atol=1e-6)


def test_mujoco_fastlio2_live_gate_gravity_only_imu_ignores_kinematic_velocity_step():
    from sim.scripts.mujoco.live_gate import _specific_force_body

    state = types.SimpleNamespace(
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        linear_velocity=np.array([1.0, -0.5, 0.0]),
    )

    acc = _specific_force_body(
        state,
        np.zeros(3),
        0.02,
        mode="gravity_only",
    )

    np.testing.assert_allclose(acc, [0.0, 0.0, 9.81], atol=1e-6)


def test_mujoco_fastlio2_live_gate_preserves_signed_imu_gyro_z():
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


def test_mujoco_fastlio2_live_gate_defaults_to_kinematic_safe_imu_mode():
    from sim.scripts.mujoco.live_gate import _build_parser

    args = _build_parser().parse_args([])

    assert args.imu_acc_mode == "finite_difference"
    assert args.max_fastlio_z_drift_m == 1.0
    assert args.max_fastlio_yaw_drift_rad == 0.5


def test_mujoco_fastlio2_live_gate_exposes_wall_timeout_guard():
    from sim.scripts.mujoco.live_gate import (
        _build_parser,
        _wall_timeout_status,
    )

    args = _build_parser().parse_args(["--max-wall-time-s", "12.5"])

    assert args.max_wall_time_s == pytest.approx(12.5)
    assert _wall_timeout_status(4.0, 0.0)["enabled"] is False

    status = _wall_timeout_status(13.0, 12.5)

    assert status["enabled"] is True
    assert status["triggered"] is True
    assert status["elapsed_wall_s"] == pytest.approx(13.0)
    assert status["max_wall_time_s"] == pytest.approx(12.5)
    assert "gate wall timeout" in status["fault"]


def test_mujoco_launcher_passes_wall_timeout_guard():
    text = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "--max-wall-time-s" in text
    assert "LINGTU_MUJOCO_LIVE_MAX_WALL_TIME_S" in text
    assert '"--partial-json-out" "$run_dir/report.partial.json"' in text


def test_mujoco_launcher_exposes_native_dds_sensor_gate():
    text = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(encoding="utf-8")

    assert 'cd "$script_dir/../../.." && pwd' in text
    assert "native-dds-sensors" in text
    assert "native-dds-gate" in text
    assert "mujoco/native_dds_sensors.py" in text
    assert "--imu-hz" in text
    assert "--imu-acc-mode" in text
    assert "--imu-acc-axis-scale" in text
    assert "--imu-gyro-axis-scale" in text
    assert "--settle-s" in text
    assert "--warmup-s" in text
    assert "--drive-ramp-s" in text
    assert "--scan-time-profile" in text
    assert "--physical-rolling-sample-mode" in text
    assert "--timestamp-clock" in text
    assert "--sim-hardware-realtime-factor" in text
    assert "--imu-timestamp-clock" in text
    assert "--lidar-timestamp-clock" in text
    assert "--min-slam-motion-ratio" in text
    assert "--max-slam-motion-ratio" in text
    assert "--min-sim-yaw-for-odom-check-rad" in text
    assert "--max-slam-yaw-error-rad" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_MIN_SLAM_MOTION_RATIO:-0.5" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_MAX_SLAM_MOTION_RATIO:-1.6" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_MIN_SIM_YAW_FOR_ODOM_CHECK_RAD:-0.2" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_MAX_SLAM_YAW_ERROR_RAD:-0.15" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_TIMESTAMP_CLOCK:-sim_hardware" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_PHYSICAL_ROLLING_SAMPLE_MODE:-subscan" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_SIM_HARDWARE_REALTIME_FACTOR:-1.0" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_SETTLE_S:-3.0" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_WARMUP_S:-2.0" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_IMU_TIMESTAMP_CLOCK" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_LIDAR_TIMESTAMP_CLOCK" in text
    assert "--publisher-bin" in text
    assert "--slam-status-json" in text
    assert "--drive-profile" in text
    assert "--allow-kinematic-fastlio-acceptance" in text
    assert "--require-slam-output" in text
    assert "--policy-path" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_PUBLISHER_BIN" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_SLAM_STATUS_JSON" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_MODE=sensor" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_CONDITIONING=realistic" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_DRIVE_MODE=policy" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_MODE:-sensor" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_CONDITIONING:-realistic" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_LOWPASS_HZ:-30" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_MAX_DYNAMIC_MPS2:-6" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_MAX_SLEW_MPS3:-400" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_DRIVE_MODE:-policy" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_DRIVE_PROFILE:-arc" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_ALLOW_KINEMATIC_FASTLIO_ACCEPTANCE" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_POLICY_PATH" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_AXIS_SCALE" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_IMU_GYRO_AXIS_SCALE:-1,1,1" in text
    assert "LINGTU_MUJOCO_NATIVE_DDS_DRIVE_RAMP_S" in text
    assert "prepare_native_dds_env" in text


def test_navigation_runtime_dataflow_documents_no_python_slam_rule():
    architecture = Path("docs/architecture/NAVIGATION_RUNTIME_DATAFLOW.md").read_text(encoding="utf-8")
    recording = Path("docs/07-testing/thunderv4_mujoco_lidar_recording_requirements.md").read_text(encoding="utf-8")

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
    assert bridge.NATIVE_SENSOR_PUBLISHER == "livox_sdk2_stream --stdin-records --dds"
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
            "sim/robots/thunderv4/policy/pose_flat_low_kpkd_microterrain_model29600_policy.onnx",
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


def test_mujoco_fastlio2_live_gate_accepts_partial_report_path():
    from sim.scripts.mujoco.live_gate import _build_parser

    args = _build_parser().parse_args(["--partial-json-out", "artifacts/live/report.partial.json"])

    assert args.partial_json_out == "artifacts/live/report.partial.json"


def test_mujoco_fastlio2_live_gate_writes_json_before_stdout_print():
    text = Path("sim/scripts/mujoco/live_gate.py").read_text(encoding="utf-8")

    json_write = text.index("if args.json_out:")
    stdout_print = text.index("print(text)")

    assert json_write < stdout_print
    assert "except BrokenPipeError:" in text


def test_mujoco_launcher_defaults_mid360_lidar_to_rolling_scan_time():
    text = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "--scan-time-profile" in text
    assert "${LINGTU_MUJOCO_LIVE_SCAN_TIME_PROFILE:-physical_rolling}" in text
    assert "${LINGTU_MUJOCO_LIVE_MID360_SAMPLES_PER_FRAME:-20000}" in text


def test_mujoco_launcher_uses_sim_clock_inspection_timeout_default():
    text = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(encoding="utf-8")

    assert 'inspection_default_goal_timeout="${LINGTU_MUJOCO_LIVE_INSPECTION_GOAL_TIMEOUT:-900}"' in text
    assert 'if [[ "$duration_clock" != "sim"' in text
    assert '"--inspection-goal-timeout" "$inspection_default_goal_timeout"' in text


def test_mujoco_launcher_disables_pct_optimizer_by_default():
    text = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(encoding="utf-8")

    assert 'LINGTU_PCT_OPTIMIZE_TRAJECTORY="${LINGTU_PCT_OPTIMIZE_TRAJECTORY:-0}"' in text
    assert "explicitly opts into the GPMP optimizer" in text


def test_mujoco_fastlio2_live_gate_disables_pct_optimizer_by_default():
    text = Path("sim/scripts/mujoco/live_gate.py").read_text(encoding="utf-8")

    assert 'PCT_OPTIMIZE_TRAJECTORY_ENV = "LINGTU_PCT_OPTIMIZE_TRAJECTORY"' in text
    assert 'os.environ.setdefault(PCT_OPTIMIZE_TRAJECTORY_ENV, "0")' in text


def test_mujoco_fastlio2_live_gate_samples_video_on_duration_clock():
    from sim.scripts.mujoco.live_gate import _video_sample_elapsed_s

    assert _video_sample_elapsed_s(
        "sim",
        elapsed_sim_s=12.5,
        elapsed_wall_s=90.0,
    ) == pytest.approx(12.5)
    assert _video_sample_elapsed_s(
        "wall",
        elapsed_sim_s=12.5,
        elapsed_wall_s=90.0,
    ) == pytest.approx(90.0)
    assert _video_sample_elapsed_s(
        "",
        elapsed_sim_s=12.5,
        elapsed_wall_s=90.0,
    ) == pytest.approx(90.0)


def test_mujoco_launcher_cleans_stale_fastlio_processes():
    text = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "cleanup_stale_fastlio2" in text
    assert "pkill -f" in text
    assert "mujoco_fastlio2_live" in text
    assert "PIPESTATUS[0]" in text


def test_mujoco_fastlio2_live_gate_accepts_fastlio_tuning_args():
    from sim.scripts.mujoco.live_gate import _build_parser

    args = _build_parser().parse_args(
        [
            "--fastlio-lidar-filter-num",
            "2",
            "--fastlio-scan-resolution",
            "0.1",
            "--fastlio-map-resolution",
            "0.2",
            "--fastlio-near-search-num",
            "8",
            "--fastlio-ieskf-max-iter",
            "8",
            "--fastlio-lidar-cov-inv",
            "500",
        ]
    )

    assert args.fastlio_lidar_filter_num == 2
    assert args.fastlio_scan_resolution == pytest.approx(0.1)
    assert args.fastlio_map_resolution == pytest.approx(0.2)
    assert args.fastlio_near_search_num == 8
    assert args.fastlio_ieskf_max_iter == 8
    assert args.fastlio_lidar_cov_inv == pytest.approx(500.0)


def test_mujoco_fastlio2_live_gate_accepts_fastlio_time_diff_arg():
    from sim.scripts.mujoco.live_gate import _build_parser

    args = _build_parser().parse_args(["--fastlio-time-diff-lidar-to-imu", "-0.0075"])

    assert args.fastlio_time_diff_lidar_to_imu == pytest.approx(-0.0075)


def test_mujoco_fastlio2_live_gate_accepts_vertical_velocity_constraint_arg():
    from sim.scripts.mujoco.live_gate import _build_parser

    default_args = _build_parser().parse_args([])
    explicit_args = _build_parser().parse_args(["--fastlio-vertical-velocity-constraint", "off"])

    assert default_args.fastlio_vertical_velocity_constraint == "off"
    assert explicit_args.fastlio_vertical_velocity_constraint == "off"


def test_mujoco_fastlio2_live_gate_accepts_turn_speed_coupling_args():
    from sim.scripts.mujoco.live_gate import _build_parser

    args = _build_parser().parse_args(
        [
            "--nav-turn-speed-yaw-rate-start",
            "0.12",
            "--nav-turn-speed-min-scale",
            "0.45",
        ]
    )

    assert args.nav_turn_speed_yaw_rate_start == pytest.approx(0.12)
    assert args.nav_turn_speed_min_scale == pytest.approx(0.45)


def test_mujoco_fastlio2_live_gate_accepts_inspection_tracking_args():
    from sim.scripts.mujoco.live_gate import _build_parser

    default_args = _build_parser().parse_args([])
    args = _build_parser().parse_args(
        [
            "--inspection-waypoint-threshold",
            "1.2",
            "--inspection-downsample-dist",
            "1.8",
            "--inspection-final-waypoint-threshold",
            "0.65",
            "--inspection-complete-path-on-goal-proximity",
            "--inspection-goal-proximity-completion-threshold",
            "1.2",
            "--inspection-path-goal-tolerance",
            "0.11",
            "--inspection-path-lookahead",
            "2.0",
            "--inspection-path-min-speed",
            "0.10",
            "--inspection-path-yaw-rate-gain",
            "0.9",
            "--inspection-path-stop-yaw-rate-gain",
            "0.9",
            "--inspection-path-dir-diff-thre",
            "1.8",
        ]
    )

    assert default_args.inspection_waypoint_threshold == pytest.approx(0.50)
    assert default_args.inspection_downsample_dist == pytest.approx(0.35)
    assert default_args.inspection_final_waypoint_threshold == pytest.approx(0.50)
    assert default_args.inspection_complete_path_on_goal_proximity is False
    assert default_args.inspection_goal_proximity_completion_threshold is None
    assert default_args.inspection_path_goal_tolerance == pytest.approx(0.12)
    assert default_args.inspection_path_lookahead == pytest.approx(1.5)
    assert default_args.inspection_path_min_speed == pytest.approx(0.15)
    assert default_args.inspection_path_yaw_rate_gain == pytest.approx(7.5)
    assert default_args.inspection_path_stop_yaw_rate_gain == pytest.approx(7.5)
    assert default_args.inspection_path_dir_diff_thre == pytest.approx(0.1)
    assert args.inspection_waypoint_threshold == pytest.approx(1.2)
    assert args.inspection_downsample_dist == pytest.approx(1.8)
    assert args.inspection_final_waypoint_threshold == pytest.approx(0.65)
    assert args.inspection_complete_path_on_goal_proximity is True
    assert args.inspection_goal_proximity_completion_threshold == pytest.approx(1.2)
    assert args.inspection_path_goal_tolerance == pytest.approx(0.11)
    assert args.inspection_path_lookahead == pytest.approx(2.0)
    assert args.inspection_path_min_speed == pytest.approx(0.10)
    assert args.inspection_path_yaw_rate_gain == pytest.approx(0.9)
    assert args.inspection_path_stop_yaw_rate_gain == pytest.approx(0.9)
    assert args.inspection_path_dir_diff_thre == pytest.approx(1.8)


def test_mujoco_launcher_passes_fastlio_time_diff_control():
    text = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "--fastlio-time-diff-lidar-to-imu" in text
    assert "LINGTU_MUJOCO_LIVE_FASTLIO_TIME_DIFF_LIDAR_TO_IMU" in text
    assert "--fastlio-vertical-velocity-constraint" in text
    assert "LINGTU_MUJOCO_LIVE_FASTLIO_VERTICAL_VELOCITY_CONSTRAINT" in text


def test_mujoco_launcher_passes_turn_speed_coupling_controls():
    text = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "--nav-turn-speed-yaw-rate-start" in text
    assert "LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_YAW_RATE_START" in text
    assert "--nav-turn-speed-min-scale" in text
    assert "LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_MIN_SCALE" in text


def test_mujoco_launcher_passes_inspection_tracking_controls():
    text = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "--inspection-waypoint-threshold" in text
    assert "--inspection-downsample-dist" in text
    assert "LINGTU_MUJOCO_LIVE_INSPECTION_DOWNSAMPLE_DIST" in text
    assert "LINGTU_MUJOCO_LIVE_INSPECTION_WAYPOINT_THRESHOLD" in text
    assert "--inspection-final-waypoint-threshold" in text
    assert "LINGTU_MUJOCO_LIVE_INSPECTION_FINAL_WAYPOINT_THRESHOLD" in text
    assert "--inspection-complete-path-on-goal-proximity" in text
    assert "LINGTU_MUJOCO_LIVE_INSPECTION_COMPLETE_PATH_ON_GOAL_PROXIMITY" in text
    assert "--inspection-goal-proximity-completion-threshold" in text
    assert "LINGTU_MUJOCO_LIVE_INSPECTION_GOAL_PROXIMITY_COMPLETION_THRESHOLD" in text
    assert "--inspection-path-goal-tolerance" in text
    assert "LINGTU_MUJOCO_LIVE_INSPECTION_PATH_GOAL_TOLERANCE" in text
    assert "--inspection-path-lookahead" in text
    assert "LINGTU_MUJOCO_LIVE_INSPECTION_PATH_LOOKAHEAD" in text
    assert "--inspection-path-min-speed" in text
    assert "LINGTU_MUJOCO_LIVE_INSPECTION_PATH_MIN_SPEED" in text
    assert "--inspection-path-yaw-rate-gain" in text
    assert "LINGTU_MUJOCO_LIVE_INSPECTION_PATH_YAW_RATE_GAIN" in text
    assert "--inspection-path-stop-yaw-rate-gain" in text
    assert "LINGTU_MUJOCO_LIVE_INSPECTION_PATH_STOP_YAW_RATE_GAIN" in text
    assert "--inspection-path-dir-diff-thre" in text
    assert "LINGTU_MUJOCO_LIVE_INSPECTION_PATH_DIR_DIFF_THRE" in text
    assert 'inspection_downsample_dist_default="1.0"' in text
    assert 'cmd_vel_angular_limit_default="0.25"' in text
    assert 'nav_max_angular_z_default="0.25"' in text
    assert 'inspection_waypoint_threshold_default="0.65"' in text
    assert 'inspection_final_waypoint_threshold_default="0.65"' in text
    assert 'inspection_complete_path_on_goal_proximity_default="0"' in text
    assert 'inspection_complete_path_on_goal_proximity_default="1"' in text
    assert 'inspection_goal_proximity_completion_threshold_default="0.65"' in text
    assert 'inspection_path_lookahead_default="2.0"' in text
    assert 'inspection_path_min_speed_default="0.15"' in text
    assert 'cmd_vel_linear_limit_default="0.45"' in text
    assert 'nav_max_linear_speed_default="0.45"' in text
    assert 'inspection_path_yaw_rate_gain_default="2.5"' in text
    assert 'inspection_path_stop_yaw_rate_gain_default="2.5"' in text
    assert 'inspection_path_dir_diff_thre_default="1.8"' in text
    assert 'nav_turn_speed_yaw_rate_start_default="0.0"' in text
    assert 'nav_turn_speed_min_scale_default="1.0"' in text


def test_fastlio_inspection_stack_passes_sim_tracking_params(monkeypatch):
    import lingtu.assembly.profile_builder as profile_builder_module
    from drivers.sim.mujoco import stack as mujoco_stack

    captured = {}
    modules = {
        "SimEndpointDriverModule": object(),
        "OccupancyGridModule": object(),
        "nav.mission": object(),
        "nav.local_planner": object(),
        "nav.path_follower": object(),
        "nav.velocity_mux": object(),
    }

    class FakeSystem:
        def get_module(self, name):
            return modules[name]

    def fake_build_system_for_profile(profile, overrides=None, **kwargs):
        captured["profile"] = profile
        captured.update(dict(overrides or {}))
        captured.update(kwargs)
        return FakeSystem()

    monkeypatch.setattr(
        profile_builder_module,
        "build_system_for_profile",
        fake_build_system_for_profile,
    )

    stack = mujoco_stack.build_fastlio2_inspection_stack(
        planner_backend="pct",
        downsample_dist=1.0,
        waypoint_threshold=1.2,
        final_waypoint_threshold=0.65,
        complete_path_on_goal_proximity=True,
        goal_proximity_completion_threshold=1.2,
        path_follower_goal_tolerance=0.11,
        path_follower_lookahead=2.0,
        path_follower_min_speed=0.10,
        path_follower_yaw_rate_gain=2.5,
        path_follower_stop_yaw_rate_gain=2.25,
        path_follower_dir_diff_thre=0.35,
    )

    assert stack.navigation is modules["nav.mission"]
    assert stack.driver is modules["SimEndpointDriverModule"]
    assert captured["profile"] == "sim_mujoco_live"
    assert captured["robot"] == "sim_endpoint"
    assert captured["enable_nav_out"] is False
    assert "enable_ros2_path_bridge" not in captured
    assert "enable_ros2_grid_bridge" not in captured
    assert captured["planner_backend"] == "pct"
    assert captured["downsample_dist"] == pytest.approx(1.0)
    assert captured["waypoint_threshold"] == pytest.approx(1.2)
    assert captured["final_waypoint_threshold"] == pytest.approx(0.65)
    assert captured["complete_path_on_goal_proximity"] is True
    assert captured["goal_proximity_completion_threshold"] == pytest.approx(1.2)
    assert captured["path_follower_goal_tolerance"] == pytest.approx(0.11)
    assert captured["path_follower_lookahead"] == pytest.approx(2.0)
    assert captured["path_follower_min_speed"] == pytest.approx(0.10)
    assert captured["path_follower_yaw_rate_gain"] == pytest.approx(2.5)
    assert captured["path_follower_stop_yaw_rate_gain"] == pytest.approx(2.25)
    assert captured["path_follower_dir_diff_thre"] == pytest.approx(0.35)


def test_mujoco_fastlio2_live_gate_reports_fastlio_observability_warnings(tmp_path: Path):
    from sim.scripts.mujoco.live_gate import _fastlio2_log_diagnostics

    log = tmp_path / "fastlio2_node.log"
    log.write_text(
        "\n".join(
            [
                "[WARN] [1.0] [lio_node]: DEGENERACY DETECTED: 1/6 DOFs degenerate, cond=1196.2, eff_ratio=0.83",
                "[WARN] [1.1] [lio_node]: IEKF did not converge: iter_num=10/5, cond=5.2e+03",
                "[WARN] [1.2] [lio_node]: DEGENERACY DETECTED: 2/6 DOFs degenerate, cond=279.6, eff_ratio=0.67",
            ]
        ),
        encoding="utf-8",
    )

    diagnostics = _fastlio2_log_diagnostics(log)

    assert diagnostics["log_exists"] is True
    assert diagnostics["degeneracy_warning_count"] == 2
    assert diagnostics["iekf_nonconverged_count"] == 1
    assert diagnostics["max_condition_number"] == pytest.approx(5200.0)
    assert diagnostics["max_degenerate_dof_count"] == 2
    assert diagnostics["min_effective_ratio"] == pytest.approx(0.67)
    assert len(diagnostics["latest_warnings"]) == 3


def test_mujoco_fastlio2_live_gate_summarizes_degeneracy_detail_samples():
    from sim.scripts.mujoco.live_gate import _summarize_degeneracy_detail_samples

    summary = _summarize_degeneracy_detail_samples(
        [
            {
                "condition_number": 88.0,
                "effective_ratio": 1.0,
                "degenerate_dof_count": 0,
                "dof_mask": [1, 1, 1, 1, 1, 1],
                "pos_cov_trace": 0.12,
                "ieskf_iter_num": 3,
                "ieskf_converged": True,
            },
            {
                "condition_number": 310.4,
                "effective_ratio": 0.83,
                "degenerate_dof_count": 1,
                "dof_mask": [1, 1, 1, 1, 1, 0],
                "pos_cov_trace": 2.5,
                "ieskf_iter_num": 5,
                "ieskf_converged": False,
            },
        ]
    )

    assert summary["sample_count"] == 2
    assert summary["max_condition_number"] == pytest.approx(310.4)
    assert summary["min_effective_ratio"] == pytest.approx(0.83)
    assert summary["max_degenerate_dof_count"] == 1
    assert summary["tz_degenerate_count"] == 1
    assert summary["iekf_nonconverged_count"] == 1
    assert summary["max_pos_cov_trace"] == pytest.approx(2.5)
    assert summary["last_sample"]["dof_mask"] == [1, 1, 1, 1, 1, 0]


def test_mujoco_fastlio2_live_gate_confirms_runtime_faults_by_streak():
    from sim.scripts.mujoco.live_gate import _update_runtime_fault_streak

    streaks = {"motion": 0, "z": 0, "yaw": 0}

    first = _update_runtime_fault_streak(
        streaks,
        kind="yaw",
        confirm_samples=2,
    )
    assert first == {"kind": "yaw", "streak": 1, "confirmed": False}
    assert streaks == {"motion": 0, "z": 0, "yaw": 1}

    second = _update_runtime_fault_streak(
        streaks,
        kind="yaw",
        confirm_samples=2,
    )
    assert second == {"kind": "yaw", "streak": 2, "confirmed": True}

    cleared = _update_runtime_fault_streak(
        streaks,
        kind="",
        confirm_samples=2,
    )
    assert cleared == {"kind": "", "streak": 0, "confirmed": False}
    assert streaks == {"motion": 0, "z": 0, "yaw": 0}


def test_mujoco_fastlio2_live_gate_finds_nearest_time_aligned_sim_pose():
    from sim.scripts.mujoco.live_gate import _nearest_sim_pose_sample

    samples = [
        (1.0, 10.0, 0.0, 0.1, 0.2),
        (1.2, 12.0, 0.0, 0.1, 0.4),
    ]

    nearest = _nearest_sim_pose_sample(samples, target_sim_time_s=1.08, max_dt_s=0.12)

    assert nearest == {
        "sim_time_s": pytest.approx(1.0),
        "xyz": [10.0, 0.0, 0.1],
        "yaw": pytest.approx(0.2),
        "dt_s": pytest.approx(0.08),
    }
    assert _nearest_sim_pose_sample(samples, target_sim_time_s=2.0, max_dt_s=0.12) is None


def test_mujoco_fastlio2_live_gate_builds_fastlio_large_loop_diagnostic_report():
    from sim.scripts.mujoco.live_gate import _fastlio_large_loop_diagnostic_report

    report = _fastlio_large_loop_diagnostic_report(
        segment_consistency=[
            {
                "segment": "start_to_goal_1",
                "sim_delta_m": 6.0,
                "fastlio_delta_m": 5.7,
                "z_delta_error_m": 0.08,
            },
            {
                "segment": "goal_3_to_start",
                "sim_delta_m": 6.1,
                "fastlio_delta_m": 2.0,
                "z_delta_error_m": 10.3896,
            },
        ],
        imu_samples=[
            {"dt_s": 0.02, "acc_norm": 9.81, "gyro_norm": 0.01, "gyro_z_radps": 0.01},
            {"dt_s": 0.02, "acc_norm": 10.4, "gyro_norm": 0.22, "gyro_z_radps": 0.22},
        ],
        scan_relative_times_s=[0.0, 0.025, 0.05, 0.075],
        scan_time_profile="synthetic_rolling",
        command_samples=[
            {
                "sim_time_s": 1.0,
                "linear_x": 0.25,
                "angular_z": 0.0,
                "source": "nav_cmd_vel",
            },
            {
                "sim_time_s": 2.0,
                "linear_x": 0.0,
                "angular_z": 0.2,
                "source": "nav_cmd_vel",
            },
            {
                "sim_time_s": 3.0,
                "linear_x": 0.0,
                "angular_z": 0.2,
                "source": "nav_cmd_vel",
            },
        ],
    )

    assert report["segment_consistency"]["sample_count"] == 2
    assert report["segment_consistency"]["max_z_delta_error_m"] == pytest.approx(10.3896)
    assert report["segment_consistency"]["worst_segment"]["segment"] == "goal_3_to_start"
    assert report["imu_statistics"]["sample_count"] == 2
    assert report["imu_statistics"]["mean_dt_s"] == pytest.approx(0.02)
    assert report["imu_statistics"]["max_acc_norm"] == pytest.approx(10.4)
    assert report["imu_statistics"]["max_gyro_norm"] == pytest.approx(0.22)
    assert report["imu_statistics"]["max_gyro_z_radps"] == pytest.approx(0.22)
    assert report["imu_statistics"]["min_gyro_z_radps"] == pytest.approx(0.01)
    assert report["imu_statistics"]["gyro_z_signed_integral_rad"] == pytest.approx(0.0046)
    assert report["scan_timing_statistics"]["profile"] == "synthetic_rolling"
    assert report["scan_timing_statistics"]["point_count"] == 4
    assert report["scan_timing_statistics"]["span_s"] == pytest.approx(0.075)
    assert report["command_trajectory_summary"]["sample_count"] == 3
    assert report["command_trajectory_summary"]["source"] == "nav_cmd_vel"
    assert report["command_trajectory_summary"]["max_linear_x"] == pytest.approx(0.25)
    assert report["command_trajectory_summary"]["angular_signed_integral_from_samples_rad"] == pytest.approx(0.2)
    assert report["yaw_input_consistency"]["checked"] is False


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


def test_mujoco_fastlio2_motion_window_aligns_sim_samples_to_odom_stamps():
    from sim.scripts.mujoco.live_gate import _aligned_motion_window

    window = _aligned_motion_window(
        [
            (0.0, 0.0, 0.0, 0.0, 0.0),
            (1.0, 0.9, 0.1, 0.0, 0.1),
            (2.0, 1.8, 0.2, 0.0, 0.2),
            (4.0, 3.5, 0.5, 0.0, 0.4),
        ],
        ros_time_origin_s=10.0,
        first_odom_stamp_s=11.02,
        last_odom_stamp_s=13.98,
        fallback_first_sim_xyz=[-1.0, 0.0, 0.0],
        fallback_last_sim_xyz=[9.0, 0.0, 0.0],
        fallback_first_sim_yaw=-0.5,
        fallback_last_sim_yaw=0.5,
    )

    assert window["time_aligned"] is True
    assert window["first_source"] == "fastlio2_stamp_aligned"
    assert window["last_source"] == "fastlio2_stamp_aligned"
    assert window["first_sim_xyz"] == pytest.approx([0.9, 0.1, 0.0])
    assert window["last_sim_xyz"] == pytest.approx([3.5, 0.5, 0.0])
    assert window["first_dt_s"] == pytest.approx(0.02)
    assert window["last_dt_s"] == pytest.approx(0.02)


def test_mujoco_fastlio2_motion_window_reports_fallback_when_stamps_are_missing():
    from sim.scripts.mujoco.live_gate import _aligned_motion_window

    window = _aligned_motion_window(
        [],
        ros_time_origin_s=10.0,
        first_odom_stamp_s=None,
        last_odom_stamp_s=None,
        fallback_first_sim_xyz=[-1.0, 0.0, 0.2],
        fallback_last_sim_xyz=[2.0, 0.4, 0.3],
        fallback_first_sim_yaw=-0.1,
        fallback_last_sim_yaw=0.2,
    )

    assert window["time_aligned"] is False
    assert window["first_source"] == "gate_first_sim_pose"
    assert window["last_source"] == "gate_last_sim_pose"
    assert window["first_sim_xyz"] == pytest.approx([-1.0, 0.0, 0.2])
    assert window["last_sim_xyz"] == pytest.approx([2.0, 0.4, 0.3])
    assert window["first_sim_yaw_rad"] == pytest.approx(-0.1)
    assert window["last_sim_yaw_rad"] == pytest.approx(0.2)


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
    adapter.map_cloud.subscribe(map_seen.append)

    adapter._publish_status_snapshot(_native_slam_status_payload())
    adapter._poll_cloud_snapshots()

    assert odometry_seen[-1].frame_id == topic_default_frame_id(TOPICS.odometry)
    assert registered_seen[-1].frame_id == topic_default_frame_id(TOPICS.registered_cloud)
    assert map_seen[-1].frame_id == topic_default_frame_id(TOPICS.map_cloud)


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


def test_mujoco_fastlio2_live_gate_exposes_scan_time_profiles():
    from sim.scripts.mujoco.live_gate import (
        _build_parser,
        _physical_rolling_scan_from_samples,
        _relative_times_for_scan,
    )

    args = _build_parser().parse_args([])
    times = _relative_times_for_scan(
        4,
        0.1,
        scan_time_profile="instantaneous",
    )
    rolling = _relative_times_for_scan(
        4,
        0.1,
        scan_time_profile="synthetic_rolling",
    )
    physical_sensor, physical_world, physical_times, moving_count, subscan_count = _physical_rolling_scan_from_samples(
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

    assert args.scan_time_profile == "physical_rolling"
    assert np.all(times == 0.0)
    assert rolling.tolist() == pytest.approx([0.0, 0.025, 0.05, 0.075])
    assert physical_sensor.shape == (3, 4)
    assert physical_world.shape == (3, 4)
    assert physical_times.tolist() == pytest.approx([0.02, 0.08, 0.08])
    assert moving_count == 2
    assert subscan_count == 2


def test_mujoco_fastlio2_live_gate_defaults_use_raw_fastlio2_topics():
    from sim.scripts.mujoco.live_gate import (
        _build_parser,
        _nav_planner_has_live_map,
        _parse_inspection_goals,
        _parse_start,
    )

    args = _build_parser().parse_args([])

    assert args.world == "building_scene"
    assert args.drive_mode == "kinematic"
    assert args.drive_source == "fixed"
    assert args.nav_data_source == "fastlio2"
    assert args.run_lingtu_frontier is False
    assert args.run_lingtu_inspection is False
    assert args.frontier_min_goals == 3
    assert args.inspection_min_checkpoints == 3
    assert args.fastlio_lidar_input == "livox_custom_msg"
    assert args.cmd_vel_linear_limit == pytest.approx(0.25)
    assert args.cmd_vel_angular_limit == pytest.approx(0.45)
    assert args.nav_max_linear_speed == pytest.approx(0.25)
    assert args.nav_max_angular_z == pytest.approx(0.45)
    assert args.runtime_motion_fault_min_sim_m == pytest.approx(0.25)
    assert args.drive_vx > 0.0
    assert args.mujoco_memory == "64M"
    assert args.work_dir.endswith("mujoco_fastlio2_live")
    assert _parse_start("5,3,-2") == [5.0, 3.0, -2.0]
    assert _parse_inspection_goals("1,2;3,4,0.5") == [
        [1.0, 2.0, 0.0],
        [3.0, 4.0, 0.5],
    ]
    assert _parse_inspection_goals('[{"x": 1, "y": 2, "frame_id": "odom"}]') == [
        [1.0, 2.0, 0.0],
    ]

    class Planner:
        has_map = True

    class NavModule:
        _planner_svc = Planner()

    assert _nav_planner_has_live_map(NavModule()) is True
    assert _nav_planner_has_live_map(object()) is False


def test_mujoco_fastlio2_live_gate_builds_navigation_diagnostic_sample():
    from sim.scripts.mujoco.live_gate import _navigation_diagnostic_sample

    sample = _navigation_diagnostic_sample(
        sim_time_s=12.345,
        wall_time_s=15.2,
        first_sim_xyz=[-9.5, -5.6, 0.5],
        current_sim_xyz=[-7.5, -5.1, 0.5],
        first_sim_yaw=0.1,
        current_sim_yaw=0.35,
        first_odom_xyz=[0.0, 0.0, 0.0],
        current_odom_xyz=[1.8, 0.4, 1.02],
        first_odom_yaw=0.0,
        current_odom_yaw=0.26,
        latest_nav_cmd={"vx": 0.2, "vy": 0.0, "wz": 0.05, "stamp": 100.0},
        now_s=100.2,
        cmd_vel_timeout_s=0.75,
        command_fresh=True,
        global_path_counts=[10, 12],
        local_path_counts=[20, 24, 18],
        waypoint_count=3,
        navigation_health={
            "state": "RUNNING",
            "patrol_index": 1,
            "patrol_total": 4,
            "wp_index": 2,
            "wp_total": 8,
            "failure_reason": "",
            "goal": [4.8, 5.7, 0.0],
            "current_waypoint": [3.5, 5.5, 0.0],
            "distance_to_goal_m": 2.25,
            "active_waypoint_distance_m": 0.85,
            "complete_path_on_goal_proximity": True,
            "goal_proximity_completion_threshold": 0.75,
            "last_plan_report": {
                "primary_planner": "pct",
                "selected_planner": "pct",
                "fallback_reason": "",
            },
        },
        local_planner_health={
            "local_planner": {
                "last_local_path_points": 0,
                "last_local_path_span_m": 0.0,
                "last_control_hint": {
                    "reason": "untrackable_local_path",
                    "safety_stop": True,
                    "near_field_stop": True,
                    "path_found": False,
                    "recovery_state": 0,
                },
                "last_result": {
                    "path_point_count": 1,
                    "path_length_m": 0.02,
                    "path_span_m": 0.02,
                    "path_found": False,
                    "near_field_stop": True,
                    "recovery_state": 0,
                },
            }
        },
        path_follower_health={
            "path_follower": {
                "has_path": False,
                "vehicle_speed": 0.0,
                "control_hint": {"reason": "untrackable_local_path"},
            }
        },
        runtime_faults=["runtime Fast-LIO Z drift (error=1.02m, allowed=1.0m)"],
    )

    assert sample["sim_time_s"] == 12.345
    assert sample["sim_xyz"] == [-7.5, -5.1, 0.5]
    assert sample["fastlio2_xyz"] == [1.8, 0.4, 1.02]
    assert sample["fastlio2_z_delta_error_m"] == 1.02
    assert sample["fastlio2_yaw_delta_error_rad"] == pytest.approx(0.01)
    assert sample["nav_cmd"]["fresh"] is True
    assert sample["nav_cmd"]["age_s"] == pytest.approx(0.2)
    assert sample["navigation"]["state"] == "RUNNING"
    assert sample["navigation"]["patrol_index"] == 1
    assert sample["navigation"]["wp_index"] == 2
    assert sample["navigation"]["wp_total"] == 8
    assert sample["navigation"]["distance_to_goal_m"] == pytest.approx(2.25)
    assert sample["navigation"]["active_waypoint_distance_m"] == pytest.approx(0.85)
    assert sample["navigation"]["complete_path_on_goal_proximity"] is True
    assert sample["navigation"]["goal_proximity_completion_threshold"] == pytest.approx(0.75)
    assert sample["navigation"]["selected_planner"] == "pct"
    assert sample["paths"]["global_path_count"] == 2
    assert sample["paths"]["local_path_points_latest"] == 18
    assert sample["local_planner"]["last_control_hint"]["reason"] == "untrackable_local_path"
    assert sample["local_planner"]["last_control_hint"]["safety_stop"] is True
    assert sample["local_planner"]["last_result"]["path_point_count"] == 1
    assert sample["path_follower"]["has_path"] is False
    assert sample["runtime_fault_count"] == 1


def test_mujoco_fastlio2_live_gate_can_stop_when_inspection_evidence_is_complete():
    from sim.scripts.mujoco.live_gate import _inspection_gate_evidence_complete

    ready = _inspection_gate_evidence_complete(
        run_lingtu_inspection=True,
        navigation_health={"state": "SUCCESS", "patrol_index": 3, "patrol_total": 3},
        inspection_goal_count=3,
        inspection_min_checkpoints=3,
        algorithm_verified=True,
        canonical_nav_outputs_verified=True,
        global_path_counts=[3],
        local_path_counts=[91],
        nav_cmd_nonzero=12,
        moving_obstacle_enabled=True,
        moving_obstacle_published_update_count=4,
        moving_obstacle_published_point_count_max=288,
        video_required=True,
        video_sample_count=1,
    )

    assert ready["ok"] is True
    assert ready["reason"] == "inspection evidence complete"
    assert ready["missing"] == []

    not_ready = _inspection_gate_evidence_complete(
        run_lingtu_inspection=True,
        navigation_health={"state": "SUCCESS", "patrol_index": 3, "patrol_total": 3},
        inspection_goal_count=3,
        inspection_min_checkpoints=3,
        algorithm_verified=True,
        canonical_nav_outputs_verified=True,
        global_path_counts=[3],
        local_path_counts=[91],
        nav_cmd_nonzero=12,
        moving_obstacle_enabled=True,
        moving_obstacle_published_update_count=0,
        moving_obstacle_published_point_count_max=0,
        video_required=True,
        video_sample_count=0,
    )

    assert not_ready["ok"] is False
    assert not_ready["missing"] == ["moving_obstacles", "video_samples"]

    latest_path_empty = _inspection_gate_evidence_complete(
        run_lingtu_inspection=True,
        navigation_health={"state": "SUCCESS", "patrol_index": 3, "patrol_total": 3},
        inspection_goal_count=3,
        inspection_min_checkpoints=3,
        algorithm_verified=True,
        canonical_nav_outputs_verified=True,
        global_path_counts=[3],
        local_path_counts=[91, 0],
        nav_cmd_nonzero=12,
        moving_obstacle_enabled=False,
        moving_obstacle_published_update_count=0,
        moving_obstacle_published_point_count_max=0,
        video_required=False,
        video_sample_count=0,
    )

    assert latest_path_empty["ok"] is True
    assert latest_path_empty["missing"] == []
    assert latest_path_empty["terminal_success"] is True

    latest_path_empty_before_success = _inspection_gate_evidence_complete(
        run_lingtu_inspection=True,
        navigation_health={"state": "PATROLLING", "patrol_index": 1, "patrol_total": 3},
        inspection_goal_count=3,
        inspection_min_checkpoints=3,
        algorithm_verified=True,
        canonical_nav_outputs_verified=True,
        global_path_counts=[3],
        local_path_counts=[91, 0],
        nav_cmd_nonzero=12,
        moving_obstacle_enabled=False,
        moving_obstacle_published_update_count=0,
        moving_obstacle_published_point_count_max=0,
        video_required=False,
        video_sample_count=0,
    )

    assert latest_path_empty_before_success["ok"] is False
    assert latest_path_empty_before_success["missing"] == [
        "patrol_success",
        "checkpoint_count",
        "local_path",
    ]
    assert latest_path_empty_before_success["terminal_success"] is False

    latest_path_never_trackable = _inspection_gate_evidence_complete(
        run_lingtu_inspection=True,
        navigation_health={"state": "SUCCESS", "patrol_index": 3, "patrol_total": 3},
        inspection_goal_count=3,
        inspection_min_checkpoints=3,
        algorithm_verified=True,
        canonical_nav_outputs_verified=True,
        global_path_counts=[3],
        local_path_counts=[1, 0],
        nav_cmd_nonzero=12,
        moving_obstacle_enabled=False,
        moving_obstacle_published_update_count=0,
        moving_obstacle_published_point_count_max=0,
        video_required=False,
        video_sample_count=0,
    )

    assert latest_path_never_trackable["ok"] is False
    assert latest_path_never_trackable["missing"] == ["local_path"]


def test_mujoco_fastlio2_live_gate_summarizes_path_geometry():
    from sim.scripts.mujoco.live_gate import _path_summary

    path = types.SimpleNamespace(
        poses=[
            types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0))),
            types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=3.0, y=4.0, z=0.0))),
        ]
    )

    summary = _path_summary(path)

    assert summary["point_count"] == 2
    assert summary["finite_point_count"] == 2
    assert summary["path_length_m"] == pytest.approx(5.0)
    assert summary["first_xyz"] == pytest.approx([0.0, 0.0, 0.0])
    assert summary["last_xyz"] == pytest.approx([3.0, 4.0, 0.0])
    assert summary["bounds_xy"] == pytest.approx([0.0, 0.0, 3.0, 4.0])


def test_mujoco_fastlio2_live_gate_summarizes_numpy_path_points():
    from sim.scripts.mujoco.live_gate import _path_summary

    summary = _path_summary(
        [
            np.array([-9.5, -5.6, 0.0], dtype=np.float32),
            np.array([-4.7, -5.6, 0.2], dtype=np.float32),
        ]
    )

    assert summary["point_count"] == 2
    assert summary["finite_point_count"] == 2
    assert summary["path_length_m"] > 4.8
    assert summary["first_xyz"] == pytest.approx([-9.5, -5.6, 0.0])
    assert summary["last_xyz"] == pytest.approx([-4.7, -5.6, 0.2])
    assert summary["bounds_xy"] == pytest.approx([-9.5, -5.6, -4.7, -5.6])


def test_mujoco_truth_nav_pose_aligns_to_tomogram_map_frame(tmp_path: Path):
    from types import SimpleNamespace

    from sim.scripts.mujoco.live_gate import (
        _map_frame_origin_world_xy_from_tomogram,
        _state_in_map_frame,
    )

    tomogram = tmp_path / "same_source_map" / "tomogram.pickle"
    tomogram.parent.mkdir()
    tomogram.write_bytes(b"not-used")
    (tomogram.parent / "metadata.json").write_text(
        json.dumps({"map_frame_origin_world_xy": [-9.5, -5.6]}),
        encoding="utf-8",
    )
    state = SimpleNamespace(
        position=[-8.5, -5.1, 0.25],
        orientation=[0.0, 0.0, 0.0, 1.0],
        linear_velocity=[0.1, 0.0, 0.0],
        angular_velocity=[0.0, 0.0, 0.02],
    )

    origin = _map_frame_origin_world_xy_from_tomogram(tomogram)
    mapped = _state_in_map_frame(state, origin)

    assert origin == pytest.approx((-9.5, -5.6))
    assert mapped.position.tolist() == pytest.approx([1.0, 0.5, 0.25])
    assert mapped.linear_velocity.tolist() == pytest.approx([0.1, 0.0, 0.0])


def test_mujoco_live_gate_limits_command_acceleration_for_imu_consistency():
    from sim.scripts.mujoco.live_gate import _limit_command_delta

    limited = _limit_command_delta(
        target=(0.25, -0.1, 0.4),
        previous=(0.0, 0.0, 0.0),
        dt_s=0.02,
        linear_accel_limit=0.5,
        angular_accel_limit=1.0,
    )

    assert limited == pytest.approx((0.01, -0.01, 0.02))


def test_mujoco_fastlio2_live_gate_rejects_large_translation_scale_error():
    from sim.scripts.mujoco.live_gate import _motion_consistency_report

    report = _motion_consistency_report(
        fastlio2_moved_m=1.7019,
        fastlio2_path_length_m=1.7256,
        sim_moved_m=0.1369,
        sim_path_length_m=0.1649,
    )

    assert report["checked"] is True
    assert report["ok"] is False
    assert report["motion_delta_error_m"] == pytest.approx(1.565)
    assert report["max_allowed_motion_error_m"] < 1.0

    under_response = _motion_consistency_report(
        fastlio2_moved_m=0.1778,
        fastlio2_path_length_m=0.18,
        sim_moved_m=0.4288,
        sim_path_length_m=0.4288,
    )

    assert under_response["ok"] is False
    assert under_response["max_allowed_motion_error_m"] == pytest.approx(0.15008)


def test_mujoco_fastlio2_live_gate_can_hold_latest_nav_cmd_for_slow_sim_clock():
    from sim.scripts.mujoco.live_gate import _select_nav_cmd_for_step

    selected = _select_nav_cmd_for_step(
        latest_nav_cmd={"vx": 0.2, "vy": -0.03, "wz": 0.1, "stamp": 100.0},
        now_s=105.0,
        cmd_vel_timeout_s=0.0,
    )

    assert selected["fresh"] is True
    assert selected["vx"] == pytest.approx(0.2)
    assert selected["vy"] == pytest.approx(-0.03)
    assert selected["wz"] == pytest.approx(0.1)


def test_mujoco_fastlio2_live_gate_summarizes_dynamic_obstacle_sweep_quality():
    from sim.scripts.mujoco.live_gate import _dynamic_obstacle_sweep_quality

    report = _dynamic_obstacle_sweep_quality(
        cases=[
            {"density": 2, "speed_mps": 0.2, "collision": False, "runtime_evidence_ok": True},
            {"density": 6, "speed_mps": 0.5, "collision": False, "runtime_evidence_ok": True},
            {"density": 10, "speed_mps": 0.8, "collision": False, "runtime_evidence_ok": True},
        ],
        required_densities=(2, 6, 10),
        required_speeds=(0.2, 0.5, 0.8),
    )

    assert report["ok"] is True
    assert report["covered_density_count"] == 3
    assert report["covered_speed_count"] == 3


def test_mujoco_launcher_exposes_cmd_vel_timeout_override():
    text = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "--cmd-vel-timeout" in text
    assert "--cmd-vel-mux-source-timeout" in text
    assert "LINGTU_MUJOCO_LIVE_CMD_VEL_TIMEOUT" in text
    assert "LINGTU_MUJOCO_LIVE_CMD_VEL_MUX_SOURCE_TIMEOUT" in text
    assert 'cmd_vel_timeout_default="0"' in text
    assert 'cmd_vel_mux_source_timeout_default="5.0"' in text
    assert 'drive_source" == "nav_cmd_vel"' in text
    assert 'moving_obstacle_default_start="2"' in text
    assert 'cmd_vel_angular_limit_default="0.25"' in text
    assert 'nav_max_angular_z_default="0.20"' in text
    assert 'runtime_fault_confirm_samples_default="6"' in text
    assert 'runtime_motion_fault_min_sim_m_default="1.0"' in text
    assert 'max_angular_saturation_ratio_default="0.40"' in text
    assert "--runtime-motion-fault-min-sim-m" in text
    assert "--max-angular-saturation-ratio" in text
    assert "cmd_vel_angular_limit=" in text
    assert "runtime_fault_confirm_samples=" in text
    assert "runtime_motion_fault_min_sim_m=" in text
    assert "max_angular_saturation_ratio=" in text


def test_safety_stack_allows_sim_specific_cmd_vel_mux_source_timeout():
    from lingtu.assembly.stacks.safety import safety

    system = safety(cmd_vel_mux_source_timeout=5.0).build()
    mux = system.get_module("nav.velocity_mux")

    assert mux.health()["source_timeout_s"] == pytest.approx(5.0)


def test_mujoco_fastlio2_live_gate_robot_crossing_obstacles_scale_density_and_speed():
    from sim.scripts.mujoco.live_gate import (
        _live_moving_obstacle_boxes_from_pose,
        _live_moving_obstacle_points,
        _live_moving_obstacle_speed_bounds,
        _live_moving_obstacle_trail_clearance,
    )

    boxes = _live_moving_obstacle_boxes_from_pose(
        position_xy=(0.0, 0.0),
        yaw_rad=0.0,
        elapsed_s=2.0,
        mode="robot_crossing",
        count=3,
        start_s=0.0,
        duration_s=10.0,
        period_s=8.0,
        forward_m=2.0,
        forward_step_m=0.8,
        lateral_phase_step_rad=math.pi / 2.0,
        lateral_amplitude_m=0.9,
        along_amplitude_m=0.2,
        radius_m=0.16,
        height_m=0.6,
    )
    points = _live_moving_obstacle_points(boxes, spacing=0.10, intensity=220.0)
    slow = _live_moving_obstacle_speed_bounds(
        period_s=8.0,
        lateral_amplitude_m=0.9,
        along_amplitude_m=0.2,
    )
    fast = _live_moving_obstacle_speed_bounds(
        period_s=4.0,
        lateral_amplitude_m=0.9,
        along_amplitude_m=0.2,
    )
    clearance = _live_moving_obstacle_trail_clearance(
        timed_trail=[(2.0, 0.0, 0.0, 0.0)],
        robot_radius_m=0.28,
        mode="robot_crossing",
        count=3,
        start_s=0.0,
        duration_s=10.0,
        period_s=8.0,
        forward_m=2.0,
        forward_step_m=0.8,
        lateral_phase_step_rad=math.pi / 2.0,
        lateral_amplitude_m=0.9,
        along_amplitude_m=0.2,
        radius_m=0.16,
        height_m=0.6,
    )

    assert len(boxes) == 3
    assert len(points) > 0
    assert len({round(box["position"][0], 3) for box in boxes}) > 1
    assert fast["peak_planar_speed_bound_mps"] > slow["peak_planar_speed_bound_mps"]
    assert clearance["checked"] is True
    assert clearance["collision"] is False
    assert clearance["min_clearance_minus_robot_radius_m"] > 0.0


def test_mujoco_world_registry_includes_industrial_demo_scene():
    from drivers.sim.mujoco.driver import _WORLDS_DIR, WORLDS

    world_file = WORLDS["industrial_demo"]

    assert world_file == "industrial_demo_scene.xml"
    assert (_WORLDS_DIR / world_file).is_file()
    assert "robot_placeholder" in (_WORLDS_DIR / world_file).read_text(encoding="utf-8")


def test_mujoco_world_registry_includes_product_industrial_park_scene():
    from drivers.sim.mujoco.driver import _WORLDS_DIR, WORLDS

    world_file = WORLDS["industrial_park"]
    text = (_WORLDS_DIR / world_file).read_text(encoding="utf-8")

    assert world_file == "industrial_park_scene.xml"
    assert (_WORLDS_DIR / world_file).is_file()
    assert text.isascii()
    assert "robot_placeholder" in text


def test_mujoco_fastlio2_live_gate_relays_fastlio_outputs_to_nav_topics():
    from pathlib import Path

    from runtime.runtime_interface import TOPICS
    from sim.scripts.mujoco import live_gate as mujoco_live_gate

    source = Path(mujoco_live_gate.__file__).read_text(encoding="utf-8")
    report_source = Path("sim/scripts/mujoco_live/report.py").read_text(encoding="utf-8")
    diagnostics_source = Path("sim/scripts/mujoco_live/diag.py").read_text(encoding="utf-8")
    motion_source = Path("sim/scripts/mujoco_live/motion.py").read_text(encoding="utf-8")
    combined_source = source + report_source + diagnostics_source + motion_source
    stack_source = Path("src/drivers/sim/mujoco/stack.py").read_text(encoding="utf-8")
    launcher_source = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "resolved_runtime_data_flow" in combined_source
    assert "FastLio2NavBridgeRuntime" not in combined_source
    assert 'node.create_subscription(Odometry, "/Odometry"' not in combined_source
    assert "canonical_nav_outputs_verified" in combined_source
    assert "TOPICS.odometry" in combined_source
    assert "TOPICS.registered_cloud" in combined_source
    assert "TOPICS.map_cloud" in combined_source
    assert "FRAMES.odom" not in combined_source
    assert "FRAMES.body" not in combined_source
    assert "FRAMES.lidar" not in combined_source
    assert "FRAMES.map" not in combined_source
    assert TOPICS.raw_lidar_points == "/points_raw"
    assert TOPICS.raw_imu == "/imu_raw"
    assert "--drive-source" in source
    assert '"nav_cmd_vel"' in source
    assert "motion_consistency" in combined_source
    assert "remaining_gaps" in combined_source
    assert "canonical_nav_outputs_verified" in combined_source
    assert "--nav-data-source" in source
    assert 'plan_safety_policy="reject"' in stack_source
    assert "relative_times_s" in combined_source
    assert "livox_custom_msg" in source
    assert "timed_pointcloud2" in source
    assert "elapsed_sim_s" in combined_source
    assert "SIM_LIDAR_FRAME_ID" in source
    assert "--drive-vx" in launcher_source
    assert "--drive-vy" in launcher_source
    assert "LINGTU_MUJOCO_LIVE_DRIVE_VY" in launcher_source
    assert "LINGTU_MUJOCO_LIVE_DRIVE_WZ" in launcher_source


def test_mujoco_fastlio2_live_gate_exception_report_keeps_runtime_contract():
    from sim.scripts.mujoco.live_gate import _gate_exception_report

    args = types.SimpleNamespace(
        duration_clock="wall",
        fastlio_lidar_input="livox_custom_msg",
        mid360_pattern=str(Path("sim/assets/livox/mid360.npy").resolve()),
        mid360_samples_per_frame=1200,
        n_rays=6400,
        json_out="artifacts/live/report.json",
        partial_json_out="artifacts/live/report.partial.json",
        scan_time_profile="physical_rolling",
    )
    partial_report = {
        "schema_version": "lingtu.mujoco_fastlio2_live_gate.partial.v1",
        "ok": False,
        "partial_report": True,
        "world": "artifacts/server_sim_closure/large_terrain/large_terrain_scene.xml",
        "nav_data_source": "fastlio2",
        "elapsed_wall_s": 1140.5,
        "elapsed_sim_s": 63.68,
        "counts": {"sim_steps": 120},
        "outputs": {"nav_cmd_vel": 240, "nav_cmd_vel_nonzero": 120},
        "lingtu_inspection": {
            "enabled": True,
            "start_status": "patrolling",
            "patrol_index": 0,
            "patrol_total": 4,
            "global_path_points_max": 7,
            "local_path_points_max": 8,
        },
        "simulation_path": {
            "first_sim_xyz": [-9.5, -5.6, 0.0],
            "last_sim_xyz": [-4.98, -4.83, 0.0],
            "first_sim_yaw_rad": 0.0,
            "last_sim_yaw_rad": 0.3,
            "sim_path_length_m": 9.43,
        },
        "runtime_faults": ["gate wall timeout after 1140.5s"],
        "gate_wall_timeout": {"enabled": True, "triggered": True},
    }
    report = _gate_exception_report(
        args,
        RuntimeError("ROS2 Python modules are unavailable"),
        partial_report=partial_report,
        partial_report_path=Path("artifacts/live/report.partial.json"),
    )

    assert report["ok"] is False
    assert report["partial_report_available"] is True
    assert report["partial_report_path"] == "artifacts/live/report.partial.json"
    assert report["partial_report"] == partial_report
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["world"] == partial_report["world"]
    assert report["elapsed_sim_s"] == pytest.approx(63.68)
    assert report["outputs"]["nav_cmd_vel_nonzero"] == 120
    assert report["lingtu_inspection"]["patrol_total"] == 4
    assert report["last_sim_xyz"] == pytest.approx([-4.98, -4.83, 0.0])
    assert report["sim_path_length_m"] == pytest.approx(9.43)
    assert "gate wall timeout after 1140.5s" in report["runtime_faults"]
    assert report["runtime_contract"]["name"] == "mujoco_fastlio2_live"
    assert report["runtime_contract"]["ok"] is False
    assert sorted(report["runtime_contract"]["frame_evidence"]) == [
        "body_to_camera",
        "body_to_lidar",
        "map_to_odom",
        "odom_to_body",
    ]
    assert sorted(report["runtime_contract"]["data_flow_evidence"]) == [
        "command_boundary",
        "dynamic_obstacle_gate",
        "endpoint_adapter",
        "global_planning",
        "local_planning_and_following",
        "map_layers_and_exploration",
        "slam_or_relayed_localization_map",
    ]
    assert report["runtime_evidence"]["ok"] is False
    assert report["runtime_evidence"]["frame_links_required"] is True
    assert report["runtime_evidence"]["data_flow_required"] is True
    assert report["lidar_source"]["forced_pattern"] is True
    assert report["lidar_source"]["samples_per_frame"] == 1200
    gaps = "\n".join(report["remaining_gaps"])
    assert "gate_exception: RuntimeError: ROS2 Python modules are unavailable" in gaps
    assert "partial_runtime_fault: gate wall timeout after 1140.5s" in gaps
    assert "runtime_contract.ok is not true" in gaps
    assert "frame evidence missing or failed for map_to_odom" in gaps
    assert "data-flow evidence missing or failed for endpoint_adapter" in gaps


def test_mujoco_data_flow_marks_unrequired_navigation_stages_not_run():
    from runtime.runtime_interface import TOPICS
    from sim.scripts.mujoco.live_gate import _mujoco_data_flow_evidence

    evidence = _mujoco_data_flow_evidence(
        topic_evidence={
            TOPICS.raw_lidar_points: {"ok": True},
            TOPICS.raw_imu: {"ok": True},
            TOPICS.odometry: {"ok": True},
            TOPICS.map_cloud: {"ok": True},
        },
        navigation_required=False,
    )

    assert evidence["endpoint_adapter"]["ok"] is True
    assert evidence["endpoint_adapter"]["inputs"] == [TOPICS.raw_lidar_points, TOPICS.raw_imu]
    assert evidence["endpoint_adapter"]["outputs"] == [TOPICS.raw_lidar_points, TOPICS.raw_imu]
    assert evidence["slam_or_relayed_localization_map"]["ok"] is True
    assert evidence["slam_or_relayed_localization_map"]["outputs"] == [
        TOPICS.odometry,
        TOPICS.registered_cloud,
        TOPICS.map_cloud,
    ]
    assert evidence["global_planning"]["required"] is False
    assert evidence["global_planning"]["ok"] is False
    assert evidence["global_planning"]["owner"] == "lingtu_navigation_or_planner_backend"
    assert evidence["global_planning"]["frame_role"] == "map"
    assert evidence["global_planning"]["map_dependency"] == (
        "octoplanner3d_uses_headless_octomap_or_point_cloud;"
        "astar_compat_or_frontier_may_use_live_occupancy_grid;"
        "pct_compat_uses_same_source_tomogram"
    )
    assert evidence["global_planning"]["reason"] == "not_required_for_basic_slam_gate"
    assert evidence["command_boundary"]["required"] is False
    assert evidence["command_boundary"]["ok"] is False
    assert evidence["command_boundary"]["outputs"] == ["mujoco_velocity_adapter"]
    assert evidence["command_boundary"]["owner"] == "command_arbiter_to_driver"
    assert evidence["command_boundary"]["frame_role"] == "body_twist"
    assert evidence["command_boundary"]["map_dependency"] == "none"


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
    import sim.engine.mujoco.engine as mujoco_engine

    monkeypatch.setitem(sys.modules, "mujoco", types.SimpleNamespace(__version__="test"))
    monkeypatch.setattr(mujoco_engine, "MuJoCoEngine", _FakeEngine)

    driver = MujocoDriverModule(
        world="open_field",
        render=False,
        enable_camera=True,
    )
    driver.setup()

    expected_world = Path(__file__).resolve().parents[2] / "sim" / "worlds" / "mujoco" / "open_field.xml"

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


def test_mujoco_driver_uses_scene_placeholder_start_pose(monkeypatch):
    import sim.engine.mujoco.engine as mujoco_engine

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
    assert Path(driver._engine.robot_config.policy_onnx).name == "pose_flat_low_kpkd_microterrain_model29600_policy.pt"


def test_mujoco_driver_defaults_to_thunderv4_policy():
    import drivers.sim.mujoco.driver as driver_mod

    assert len(driver_mod._POLICY_CANDIDATES) == 1
    assert driver_mod._POLICY_CANDIDATES[0].name == "pose_flat_low_kpkd_microterrain_model29600_policy.pt"
    assert driver_mod._EXPLICIT_POLICY_CANDIDATES[0].name == "policy_251119.onnx"


def test_optional_go1_asset_contract_has_placeholder_readme():
    repo_root = Path(__file__).resolve().parents[2]
    indoor_office = (repo_root / "sim" / "worlds" / "mujoco" / "indoor_office.xml").read_text(encoding="utf-8")
    readme = repo_root / "sim" / "robots" / "go1_playground" / "README.md"

    assert "../robots/go1_playground/xmls/go1_mjx_feetonly.xml" in indoor_office
    assert readme.exists()
    text = readme.read_text(encoding="utf-8")
    assert "optional external assets" in text
    assert "not part of the G4 server closure" in text


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

    assert "Sole canonical field thin adapter" in scripts_index
    assert "scripts/ota/" not in scripts_index
    assert "scripts/deploy/package_native_release.sh" in release_guide
    assert "ProductControl" in installer

def test_sim_boundary_indexes_document_stable_contracts():
    repo_root = Path(__file__).resolve().parents[2]
    indexes = {
        "simulation": repo_root / "sim" / "README.md",
        "scripts": repo_root / "sim" / "scripts" / "README.md",
        "engine": repo_root / "sim" / "engine" / "README.md",
        "repository": repo_root / "docs" / "REPO_LAYOUT.md",
    }
    texts = {name: path.read_text(encoding="utf-8") for name, path in indexes.items()}

    assert "Stable Root Contract" in texts["simulation"]
    assert "sim/scripts/mujoco/*" in texts["simulation"]
    assert "MuJoCo Native DDS Gate" in texts["simulation"]
    assert "Simulation cannot prove" in texts["simulation"]
    assert "Canonical MuJoCo Entrypoints" in texts["scripts"]
    assert "Safety Classes" in texts["scripts"]
    assert "canonical simulation runtime" in texts["engine"]
    assert "real_robot_motion=false" in texts["engine"]
    assert "| `sim/` | Simulation engines" in texts["repository"]

    boundary_markers = {
        "bridge": ("Legacy ROS2 redirect entrypoints were removed", "src/drivers/sim/"),
        "datasets": ("offline replay inputs", "generated validation evidence", "artifacts/"),
        "sensors": ("hardware-free", "sim/assets/livox/", "startup side effects"),
    }
    for folder, markers in boundary_markers.items():
        text = (repo_root / "sim" / folder / "README.md").read_text(encoding="utf-8")
        assert all(marker in text for marker in markers)


def test_mujoco_driver_resolves_explicit_policy_and_repo_relative_paths(monkeypatch, tmp_path):
    import drivers.sim.mujoco.driver as driver_mod

    sim_root = tmp_path / "sim"
    policy_dir = sim_root / "robots" / "nova_dog"
    brainstem_policy = policy_dir / "model" / "policy_251119.onnx"
    thunder_policy = policy_dir / "thunder_policy.onnx"
    legacy_policy = policy_dir / "policy.onnx"
    brainstem_policy.parent.mkdir(parents=True)
    policy_dir.mkdir(parents=True, exist_ok=True)
    brainstem_policy.write_bytes(b"brainstem")
    thunder_policy.write_bytes(b"thunder")
    legacy_policy.write_bytes(b"legacy")

    monkeypatch.setattr(driver_mod, "_SIM_ROOT", sim_root)
    monkeypatch.setattr(driver_mod, "_POLICY_CANDIDATES", ())

    driver = MujocoDriverModule(policy_path="")
    assert driver._policy_path == ""

    explicit = MujocoDriverModule(policy_path="model/policy_251119.onnx")
    assert Path(explicit._policy_path) == brainstem_policy.resolve()

    missing_explicit = driver_mod._resolve_sim_path("sim/robots/nova_dog/missing.onnx")
    assert Path(missing_explicit) == (sim_root / "robots" / "nova_dog" / "missing.onnx").resolve()

    missing_sim_relative = driver_mod._resolve_sim_path("robots/nova_dog/missing.onnx")
    assert Path(missing_sim_relative) == (sim_root / "robots" / "nova_dog" / "missing.onnx").resolve()


def test_mujoco_driver_applies_explicit_cmd_velocity_limits(monkeypatch):
    import sim.engine.mujoco.engine as mujoco_engine

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
    import sim.engine.mujoco.engine as mujoco_engine

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
    from sim.engine.mujoco.robot_controller import OBS_DIM, PolicyRunner

    assert PolicyRunner._resolve_history_len(OBS_DIM) == 1
    assert PolicyRunner._resolve_history_len(OBS_DIM * 5) == 5


def test_mujoco_policy_runner_rejects_unknown_obs_contract():
    from sim.engine.mujoco.robot_controller import PolicyRunner, UnsupportedPolicyInputError

    with pytest.raises(UnsupportedPolicyInputError, match="76-D input"):
        PolicyRunner._resolve_history_len(76)


def test_mujoco_policy_runner_does_not_pad_or_truncate_obs():
    from sim.engine.mujoco.robot_controller import OBS_DIM, PolicyRunner

    runner = object.__new__(PolicyRunner)
    runner._history_len = 1

    obs = np.arange(OBS_DIM, dtype=np.float32)
    adapted = runner._adapt_obs_to_policy_input(obs)

    assert adapted.shape == (OBS_DIM,)
    assert np.allclose(adapted, obs)

    with pytest.raises(ValueError, match="expected one 57-D observation"):
        runner._adapt_obs_to_policy_input(np.arange(76, dtype=np.float32))


def test_mujoco_policy_runner_clamp_matches_brainstem_noop():
    from sim.engine.mujoco.robot_controller import PolicyRunner

    action = np.array(
        [2.0, -3.0, 4.0, -2.0, 3.0, -4.0, 1.8, -2.8, 3.8, -1.8, 2.8, -3.8, 0.8, -0.9, 1.1, -1.2],
        dtype=np.float64,
    )

    clamped = PolicyRunner.clamp_action(action)

    assert np.allclose(clamped, action)
    assert clamped is not action


def test_mujoco_thunderv4_onnx_policy_uses_thunderv4_runner():
    from sim.engine.mujoco.robot_controller import _is_thunderv4_policy

    assert _is_thunderv4_policy(
        Path("sim/robots/thunderv4/policy/pose_flat_low_kpkd_microterrain_model29600_policy.onnx")
    )
    assert _is_thunderv4_policy(Path("pose_flat_low_kpkd_microterrain_model29600_policy.onnx"))
    assert not _is_thunderv4_policy(Path("model/policy_251119.onnx"))


def test_mujoco_thunderv4_onnx_policy_infers_when_available():
    pytest.importorskip("onnxruntime")
    from sim.engine.mujoco.robot_controller import (
        OBS_DIM,
        THUNDERV4_STANDING_POSE,
        ThunderV4OnnxPolicyRunner,
        load_policy_runner,
    )

    policy = (
        Path(__file__).resolve().parents[2]
        / "sim"
        / "robots"
        / "thunderv4"
        / "policy"
        / "pose_flat_low_kpkd_microterrain_model29600_policy.onnx"
    )
    if not policy.exists():
        pytest.skip(f"ThunderV4 ONNX policy missing: {policy}")

    runner = load_policy_runner(str(policy))
    action = runner.infer(np.zeros(OBS_DIM, dtype=np.float32))

    assert isinstance(runner, ThunderV4OnnxPolicyRunner)
    assert action.shape == (16,)
    assert np.allclose(action, THUNDERV4_STANDING_POSE)


def _stable_policy_contact_summary():
    return {
        "sample_count": 10,
        "available_sample_count": 10,
        "contact_sample_count": 10,
        "foot_contact_sample_count": 10,
        "unique_feet": ["FL_foot", "FR_foot", "RL_foot", "RR_foot"],
        "unique_feet_count": 4,
        "per_foot_contact_samples": {
            "FL_foot": 6,
            "FR_foot": 6,
            "RL_foot": 6,
            "RR_foot": 6,
        },
        "support_count": {"min": 2.0, "max": 4.0, "avg": 3.0},
        "max_support_count": 4,
        "non_foot_ground_contacts": 0,
        "max_normal_force": 85.0,
    }


def test_policy_nav_smoke_pass_fail_gates_are_conservative():
    from sim.scripts import policy_nav_smoke

    direct = {
        "drive_mode": "policy",
        "policy_loaded": True,
        "finite": True,
        "moved_m": 0.25,
        "yaw_delta_abs_rad": 0.0,
        "z": {"min": 0.40, "max": 0.45},
        "roll_abs": {"max": 0.05},
        "pitch_abs": {"max": 0.04},
        "contacts": _stable_policy_contact_summary(),
    }
    nav = {
        "drive_mode": "policy",
        "policy_loaded": True,
        "finite": True,
        "global_planner_backend_status": {"configured_backend": "octoplanner3d"},
        "local_planner_backend_actual": "nanobind",
        "path_follower_backend_actual": "nav_kernel",
        "success_seen": True,
        "moved_m": 0.25,
        "z": {"min": 0.40, "max": 0.45},
        "seen": {
            "costmap": 1,
            "waypoints": 1,
            "local_path": 1,
            "path_follower_cmd": 4,
            "mux_cmd": 4,
            "direct_fallback": 0,
        },
        "dist_to_goal_m": 0.30,
        "dist_at_success_m": 0.05,
        "roll_abs": {"max": 0.05},
        "pitch_abs": {"max": 0.04},
        "contacts": _stable_policy_contact_summary(),
    }

    assert policy_nav_smoke._passes_direct(direct, min_motion=0.20) is True
    assert policy_nav_smoke._passes_nav(nav, min_motion=0.20, max_dist_to_goal=0.10) is True

    direct["moved_m"] = 0.05
    nav["seen"]["direct_fallback"] = 1
    assert policy_nav_smoke._passes_direct(direct, min_motion=0.20) is False
    assert policy_nav_smoke._passes_nav(nav, min_motion=0.20, max_dist_to_goal=0.10) is False


def test_policy_nav_smoke_requires_real_policy_drive_mode():
    from sim.scripts import policy_nav_smoke

    direct = {
        "drive_mode": "kinematic",
        "policy_loaded": True,
        "finite": True,
        "moved_m": 0.30,
        "z": {"min": 0.40, "max": 0.45},
        "roll_abs": {"max": 0.05},
        "pitch_abs": {"max": 0.04},
        "contacts": _stable_policy_contact_summary(),
    }
    nav = {
        "drive_mode": "kinematic",
        "policy_loaded": True,
        "finite": True,
        "global_planner_backend_status": {"configured_backend": "octoplanner3d"},
        "local_planner_backend_actual": "nanobind",
        "path_follower_backend_actual": "nav_kernel",
        "success_seen": True,
        "moved_m": 0.30,
        "z": {"min": 0.40, "max": 0.45},
        "seen": {
            "costmap": 1,
            "waypoints": 1,
            "local_path": 1,
            "path_follower_cmd": 4,
            "mux_cmd": 4,
            "direct_fallback": 0,
        },
        "dist_at_success_m": 0.05,
        "roll_abs": {"max": 0.05},
        "pitch_abs": {"max": 0.04},
        "contacts": _stable_policy_contact_summary(),
    }

    assert policy_nav_smoke._passes_direct(direct, min_motion=0.20) is False
    assert policy_nav_smoke._passes_nav(nav, min_motion=0.20, max_dist_to_goal=0.10) is False

    direct["drive_mode"] = "policy"
    direct["policy_loaded"] = False
    nav["drive_mode"] = "policy"
    nav["policy_loaded"] = False
    assert policy_nav_smoke._passes_direct(direct, min_motion=0.20) is False
    assert policy_nav_smoke._passes_nav(nav, min_motion=0.20, max_dist_to_goal=0.10) is False


def test_policy_nav_smoke_accepts_close_goal_without_terminal_success_state():
    from sim.scripts import policy_nav_smoke

    nav = {
        "drive_mode": "policy",
        "policy_loaded": True,
        "finite": True,
        "global_planner_backend_status": {"configured_backend": "octoplanner3d"},
        "local_planner_backend_actual": "nanobind",
        "path_follower_backend_actual": "nav_kernel",
        "success_seen": False,
        "moved_m": 0.30,
        "dist_to_goal_m": 0.05,
        "z": {"min": 0.40, "max": 0.45},
        "roll_abs": {"max": 0.05},
        "pitch_abs": {"max": 0.04},
        "seen": {
            "costmap": 0,
            "waypoints": 1,
            "local_path": 1,
            "path_follower_cmd": 4,
            "mux_cmd": 4,
            "direct_fallback": 0,
        },
        "costmap_readiness": {"planner_has_map": True},
        "contacts": _stable_policy_contact_summary(),
    }

    assert policy_nav_smoke._passes_nav(nav, min_motion=0.20, max_dist_to_goal=0.10) is True
    nav["dist_to_goal_m"] = 0.30
    assert policy_nav_smoke._passes_nav(nav, min_motion=0.20, max_dist_to_goal=0.10) is False
    nav["success_seen"] = True
    assert policy_nav_smoke._passes_nav(nav, min_motion=0.20, max_dist_to_goal=0.10) is True

    nav["global_planner_backend_status"] = {"configured_backend": "astar"}
    assert policy_nav_smoke._passes_nav(nav, min_motion=0.20, max_dist_to_goal=0.10) is False
    nav["global_planner_backend_status"] = {"configured_backend": "octoplanner3d"}
    nav["local_planner_backend_actual"] = "cmu_py"
    assert policy_nav_smoke._passes_nav(nav, min_motion=0.20, max_dist_to_goal=0.10) is False
    nav["local_planner_backend_actual"] = "nanobind"
    nav["path_follower_backend_actual"] = "pid"
    assert policy_nav_smoke._passes_nav(nav, min_motion=0.20, max_dist_to_goal=0.10) is False


def test_policy_nav_smoke_direct_stand_and_turn_gates():
    from sim.scripts import policy_nav_smoke

    base = {
        "drive_mode": "policy",
        "policy_loaded": True,
        "finite": True,
        "z": {"min": 0.40, "max": 0.45},
        "roll_abs": {"max": 0.05},
        "pitch_abs": {"max": 0.04},
        "contacts": _stable_policy_contact_summary(),
    }

    stand = {**base, "moved_m": 0.03, "yaw_delta_abs_rad": 0.0}
    assert (
        policy_nav_smoke._passes_direct(
            stand,
            min_motion=0.20,
            direct_mode="stand",
            max_stand_drift=0.05,
        )
        is True
    )
    stand["moved_m"] = 0.08
    assert (
        policy_nav_smoke._passes_direct(
            stand,
            min_motion=0.20,
            direct_mode="stand",
            max_stand_drift=0.05,
        )
        is False
    )

    turn = {**base, "moved_m": 0.04, "yaw_delta_abs_rad": 0.45}
    assert (
        policy_nav_smoke._passes_direct(
            turn,
            min_motion=0.20,
            direct_mode="turn",
            min_turn_yaw=0.35,
            max_turn_drift=0.10,
        )
        is True
    )
    turn["yaw_delta_abs_rad"] = 0.20
    assert (
        policy_nav_smoke._passes_direct(
            turn,
            min_motion=0.20,
            direct_mode="turn",
            min_turn_yaw=0.35,
            max_turn_drift=0.10,
        )
        is False
    )
    turn["yaw_delta_abs_rad"] = 0.45
    turn["moved_m"] = 0.15
    assert (
        policy_nav_smoke._passes_direct(
            turn,
            min_motion=0.20,
            direct_mode="turn",
            min_turn_yaw=0.35,
            max_turn_drift=0.10,
        )
        is False
    )


def test_policy_nav_smoke_accepts_explicit_policy_argument():
    from sim.scripts import policy_nav_smoke

    args = policy_nav_smoke._build_parser().parse_args(
        [
            "--policy",
            "/tmp/policy.onnx",
            "--nav-max-angular-z",
            "0.2",
            "--nav-planner-backend",
            "octoplanner3d",
            "--nav-local-planner-backend",
            "nanobind",
            "--nav-path-follower-backend",
            "nav_kernel",
            "--direct-mode",
            "turn",
            "--max-stand-drift",
            "0.04",
            "--min-turn-yaw",
            "0.25",
            "--max-turn-drift",
            "0.12",
            "--nav-costmap-wait",
            "1.5",
            "--nav-free-costmap-resolution",
            "0.2",
            "--nav-free-costmap-margin",
            "4.0",
            "--direct-only",
        ]
    )

    assert args.policy == "/tmp/policy.onnx"
    assert args.nav_planner_backend == "octoplanner3d"
    assert args.nav_local_planner_backend == "nanobind"
    assert args.nav_path_follower_backend == "nav_kernel"
    assert args.direct_mode == "turn"
    assert args.nav_max_angular_z == pytest.approx(0.2)
    assert args.nav_waypoint_threshold == pytest.approx(0.25)
    assert args.nav_final_waypoint_threshold == pytest.approx(0.06)
    assert args.nav_path_goal_tolerance == pytest.approx(0.08)
    assert args.nav_path_min_speed == pytest.approx(0.05)
    assert args.nav_post_success_settle == pytest.approx(0.0)
    assert args.nav_safe_goal_tolerance == pytest.approx(0.0)
    assert args.nav_costmap_wait == pytest.approx(1.5)
    assert args.no_nav_inject_free_costmap is False
    assert args.nav_free_costmap_resolution == pytest.approx(0.2)
    assert args.nav_free_costmap_margin == pytest.approx(4.0)
    assert args.max_nav_dist_to_goal == pytest.approx(0.10)
    assert args.max_stand_drift == pytest.approx(0.04)
    assert args.min_turn_yaw == pytest.approx(0.25)
    assert args.max_turn_drift == pytest.approx(0.12)
    assert args.direct_only is True


def test_policy_nav_smoke_defaults_to_product_backends():
    from sim.scripts import policy_nav_smoke

    args = policy_nav_smoke._build_parser().parse_args([])

    assert args.nav_planner_backend == "octoplanner3d"
    assert args.nav_local_planner_backend == "nanobind"
    assert args.nav_path_follower_backend == "nav_kernel"


def test_record_policy_nav_video_uses_product_backends():
    repo_root = Path(__file__).resolve().parents[2]
    source = (repo_root / "sim/scripts/mujoco/record_policy_nav_video.py").read_text(encoding="utf-8")

    assert "PRODUCTION_GLOBAL_PLANNER_BACKEND" in source
    assert "planner_backend=PRODUCTION_GLOBAL_PLANNER_BACKEND" in source
    assert "python_autonomy_backend=PRODUCTION_LOCAL_PLANNER_BACKEND" in source
    assert "python_path_follower_backend=PRODUCTION_PATH_FOLLOWER_BACKEND" in source
    assert '"global_planner_backend_requested": PRODUCTION_GLOBAL_PLANNER_BACKEND' in source
    assert '"local_planner_backend_requested": PRODUCTION_LOCAL_PLANNER_BACKEND' in source
    assert '"path_follower_backend_requested": PRODUCTION_PATH_FOLLOWER_BACKEND' in source


def test_policy_nav_smoke_free_costmap_covers_start_and_goal():
    from sim.scripts import policy_nav_smoke

    cm = policy_nav_smoke._free_costmap_for_goal(
        (1.0, -2.0, 0.0),
        (2.5, -1.0, 0.0),
        resolution=0.25,
        margin=1.0,
    )

    grid = cm["grid"]
    origin = cm["origin"]
    assert cm["source"] == "policy_nav_smoke_free_space"
    assert cm["frame_id"] == "map"
    assert grid.ndim == 2
    assert grid.shape[0] >= 32
    assert grid.shape[1] >= 32
    assert float(grid.max()) == 0.0
    assert origin[0] <= 0.0
    assert origin[1] <= -3.0
    assert origin[0] + grid.shape[1] * cm["resolution"] >= 3.5
    assert origin[1] + grid.shape[0] * cm["resolution"] >= 0.0


def test_nova_dog_policy_manifest_records_verified_contract():
    manifest = Path(__file__).resolve().parents[2] / "sim" / "robots" / "nova_dog" / "policy_manifest.json"
    data = json.loads(manifest.read_text(encoding="utf-8"))

    assert data["schema_version"] == "lingtu.sim_policy_manifest.v1"
    assert data["asset"] == "policy.onnx"
    assert data["sha256"] == "c672253ffb89ae4f0c766615e7028a9a676572c77fc1741474e552eae55b2672"
    assert data["onnx"]["input_shape"] == [1, 57]
    assert data["onnx"]["output_shape"] == [1, 16]
    assert data["contract"]["observation"] == "brainstem StandardObservationBuilder"
    assert data["simulation_only"] is True
    assert data["real_robot_motion"] is False


def test_mujoco_policy_idle_command_detection():
    from sim.engine.mujoco.engine import MuJoCoEngine

    engine = MuJoCoEngine()

    assert engine._is_idle_policy_command(np.array([0.0, 0.0, 0.0]))
    assert engine._is_idle_policy_command(np.array([1e-5, 0.0, 0.0]))
    assert not engine._is_idle_policy_command(np.array([1e-3, 0.0, 0.0]))


def test_navigation_stack_passes_path_follower_precision_params():
    system = thunder_blueprint(
        robot="sim_mujoco",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        planner_backend="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        final_waypoint_threshold=0.09,
        path_follower_goal_tolerance=0.07,
        path_follower_min_speed=0.04,
        path_follower_max_speed=0.22,
        path_follower_max_yaw_rate=0.18,
        path_follower_turn_speed_yaw_rate_start=0.12,
        path_follower_turn_speed_min_scale=0.45,
        run_startup_checks=False,
    ).build()

    follower = system.get_module("nav.path_follower")
    nav = system.get_module("nav.mission")

    assert nav._tracker._final_threshold == pytest.approx(0.09)
    assert follower._goal_tolerance == pytest.approx(0.07)
    assert follower._min_speed == pytest.approx(0.04)
    assert follower._max_speed == pytest.approx(0.22)
    assert follower._max_yaw_rate == pytest.approx(0.18)
    assert follower._turn_speed_yaw_rate_start == pytest.approx(0.12)
    assert follower._turn_speed_min_scale == pytest.approx(0.45)


def test_navigation_stack_passes_local_planner_trackable_path_threshold():
    system = thunder_blueprint(
        robot="sim_mujoco",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        planner_backend="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        local_planner_allow_direct_track_fallback=True,
        local_planner_direct_track_fallback_min_distance_m=0.75,
        local_planner_min_trackable_local_path_m=0.8,
        run_startup_checks=False,
    ).build()

    local_planner = system.get_module("nav.local_planner")

    assert local_planner._allow_direct_track_fallback is True
    assert local_planner._direct_track_fallback_min_distance_m == pytest.approx(0.75)
    assert local_planner._min_trackable_local_path_xy == pytest.approx(0.8)


def test_mujoco_camera_preserves_metric_depth_output():
    from sim.engine.mujoco.camera import MuJoCoCamera

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
    from sim.engine.core.engine import VelocityCommand

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
        assert moved > 0.75
    finally:
        if driver._engine is not None:
            driver._engine.close()
            driver._engine = None


def test_mujoco_kinematic_step_sanitizes_post_physics_base_jitter(monkeypatch):
    import sys
    from types import SimpleNamespace

    from sim.engine.core.engine import VelocityCommand
    from sim.engine.mujoco.engine import MuJoCoEngine

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

    from sim.engine.core.engine import VelocityCommand
    from sim.engine.mujoco.engine import MuJoCoEngine

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
    import drivers.sim.mujoco.driver as driver_mod

    policy_path = driver_mod._first_existing_path(driver_mod._POLICY_CANDIDATES)
    if not policy_path:
        pytest.skip("default MuJoCo policy is not available in this checkout")
    return Path(policy_path)


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
    pytest.importorskip("torch")
    from sim.engine.core.engine import VelocityCommand

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


def test_sim_mujoco_full_stack_emits_costmap_and_plans_local_goal():
    pytest.importorskip("mujoco")

    system = thunder_blueprint(
        robot="sim_mujoco",
        world="open_field",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        planner_backend="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        odom_frame_id="map",
        render=False,
        drive_mode="kinematic",
        waypoint_threshold=0.35,
        downsample_dist=0.5,
        safety_stop_wiring=False,
        lidar_publish_every=100000,
        height_ray_publish_every=100000,
        run_startup_checks=False,
    ).build()
    driver = system.get_module("MujocoDriverModule")
    ogm = system.get_module("OccupancyGridModule")
    nav = system.get_module("nav.mission")
    local_planner = system.get_module("nav.local_planner")
    path_follower = system.get_module("nav.path_follower")
    mux = system.get_module("nav.velocity_mux")

    seen = {
        "costmap": 0,
        "waypoints": 0,
        "local_path": 0,
        "path_follower_cmd": 0,
        "mux_cmd": 0,
        "direct_fallback": 0,
    }
    odom = []
    state_history = []
    ogm.costmap._add_callback(lambda _: seen.__setitem__("costmap", seen["costmap"] + 1))
    nav.waypoint._add_callback(lambda _: seen.__setitem__("waypoints", seen["waypoints"] + 1))
    nav.planner_status._add_callback(lambda state: state_history.append(state))
    nav.adapter_status._add_callback(
        lambda e: seen.__setitem__(
            "direct_fallback",
            seen["direct_fallback"] + (1 if e.get("event") == "direct_goal_fallback" else 0),
        )
    )
    local_planner.local_path._add_callback(lambda _: seen.__setitem__("local_path", seen["local_path"] + 1))
    path_follower.cmd_vel._add_callback(lambda _: seen.__setitem__("path_follower_cmd", seen["path_follower_cmd"] + 1))
    mux.driver_cmd_vel._add_callback(lambda _: seen.__setitem__("mux_cmd", seen["mux_cmd"] + 1))
    driver.odometry._add_callback(
        lambda m: odom.append((float(m.pose.position.x), float(m.pose.position.y), float(m.pose.position.z)))
    )

    system.start()
    try:
        deadline = time.time() + 6.0
        while time.time() < deadline and (seen["costmap"] == 0 or not odom):
            time.sleep(0.1)
        assert driver._engine is not None
        assert seen["costmap"] > 0
        assert nav._planner_svc.has_map is True

        start = odom[-1]
        x, y, _ = start
        goal_x = x + 2.5
        goal_y = y
        nav.goal_pose._deliver(
            PoseStamped(
                pose=Pose(
                    position=Vector3(goal_x, goal_y, 0.0),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
                frame_id="map",
                ts=time.time(),
            )
        )

        plan_deadline = time.time() + 16.0
        moved = 0.0
        dist_to_goal = math.hypot(goal_x - start[0], goal_y - start[1])
        while time.time() < plan_deadline:
            time.sleep(0.1)
            if odom:
                moved = math.hypot(odom[-1][0] - start[0], odom[-1][1] - start[1])
                dist_to_goal = math.hypot(goal_x - odom[-1][0], goal_y - odom[-1][1])
            if (
                seen["waypoints"] > 0
                and seen["local_path"] > 0
                and seen["path_follower_cmd"] >= 3
                and seen["mux_cmd"] >= 3
                and moved > 0.30
                and dist_to_goal < 2.20
            ):
                break

        state_at_navigation_evidence = nav._state
        state_deadline = time.time() + 2.5
        while time.time() < state_deadline and nav._state == "PLANNING":
            time.sleep(0.05)

        assert seen["waypoints"] > 0
        assert seen["local_path"] > 0
        assert seen["path_follower_cmd"] >= 3
        assert seen["mux_cmd"] >= 3
        assert seen["direct_fallback"] == 0
        assert moved > 0.30, {
            "seen": seen,
            "moved": moved,
            "dist_to_goal": dist_to_goal,
            "nav_state": nav._state,
            "state_at_navigation_evidence": state_at_navigation_evidence,
            "state_history_tail": state_history[-12:],
        }
        assert dist_to_goal < 2.20
        assert "EXECUTING" in state_history or nav._state in ("EXECUTING", "SUCCESS"), {
            "nav_state": nav._state,
            "state_at_navigation_evidence": state_at_navigation_evidence,
            "state_history_tail": state_history[-12:],
        }
        assert "FAILED" not in state_history
        assert "CANCELLED" not in state_history
    finally:
        system.stop()


def test_sim_mujoco_full_stack_policy_mode_moves_under_nav_cmds_when_real_policy_available():
    pytest.importorskip("mujoco")
    pytest.importorskip("torch")
    policy_path = _real_policy_path_or_skip()

    # MuJoCo + policy state is not repeat-isolated inside one Windows
    # Python process. Production validation launches this gate in a fresh
    # process, so keep the test boundary identical to the runtime boundary.
    script = r"""
import json
import math
import time

from lingtu.assembly.products.thunder import thunder_blueprint
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3

system = thunder_blueprint(
    robot="sim_mujoco",
    world="open_field",
    slam_profile="none",
    detector="sim_scene",
    llm="mock",
    planner_backend="direct",
    enable_native=False,
    enable_semantic=False,
    enable_gateway=False,
    render=False,
    python_autonomy_backend="simple",
    python_path_follower_backend="pid",
    drive_mode="policy",
    policy_path=r"__POLICY_PATH__",
    max_angular_vel=0.15,
    waypoint_threshold=0.35,
    final_waypoint_threshold=0.06,
    downsample_dist=0.25,
    path_follower_goal_tolerance=0.08,
    path_follower_min_speed=0.15,
    path_follower_max_speed=0.25,
    path_follower_max_yaw_rate=0.15,
    run_startup_checks=False,
).build()
driver = system.get_module("MujocoDriverModule")
ogm = system.get_module("OccupancyGridModule")
nav = system.get_module("nav.mission")
local_planner = system.get_module("nav.local_planner")
path_follower = system.get_module("nav.path_follower")
mux = system.get_module("nav.velocity_mux")

seen = {
    "costmap": 0,
    "waypoints": 0,
    "local_path": 0,
    "path_follower_cmd": 0,
    "mux_cmd": 0,
    "direct_fallback": 0,
}
odom = []
z_values = []
state_history = []
ogm.costmap._add_callback(lambda _: seen.__setitem__("costmap", seen["costmap"] + 1))
nav.waypoint._add_callback(lambda _: seen.__setitem__("waypoints", seen["waypoints"] + 1))
nav.planner_status._add_callback(lambda state: state_history.append(state))
nav.adapter_status._add_callback(
    lambda e: seen.__setitem__(
        "direct_fallback",
        seen["direct_fallback"] + (1 if e.get("event") == "direct_goal_fallback" else 0),
    )
)
local_planner.local_path._add_callback(
    lambda _: seen.__setitem__("local_path", seen["local_path"] + 1)
)
path_follower.cmd_vel._add_callback(
    lambda _: seen.__setitem__("path_follower_cmd", seen["path_follower_cmd"] + 1)
)
mux.driver_cmd_vel._add_callback(lambda _: seen.__setitem__("mux_cmd", seen["mux_cmd"] + 1))
driver.odometry._add_callback(
    lambda m: odom.append(
        (float(m.pose.position.x), float(m.pose.position.y), float(m.pose.position.z))
    )
)

system.start()
try:
    deadline = time.time() + 8.0
    while time.time() < deadline and (seen["costmap"] == 0 or not odom):
        time.sleep(0.1)
    if driver._engine is None:
        raise AssertionError("MuJoCo engine did not start")
    if driver._engine.drive_mode != "policy" or not driver._engine.has_policy:
        raise AssertionError("policy drive mode did not load")
    if seen["costmap"] <= 0:
        raise AssertionError("costmap was not produced")

    start = odom[-1]
    goal_x = start[0] + 1.0
    goal_y = start[1]
    nav.goal_pose._deliver(
        PoseStamped(
            pose=Pose(
                position=Vector3(goal_x, goal_y, 0.0),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
            frame_id="map",
            ts=time.time(),
        )
    )

    deadline = time.time() + 18.0
    moved = 0.0
    dist_to_goal = math.hypot(goal_x - start[0], goal_y - start[1])
    while time.time() < deadline:
        time.sleep(0.1)
        if odom:
            moved = math.hypot(odom[-1][0] - start[0], odom[-1][1] - start[1])
            dist_to_goal = math.hypot(goal_x - odom[-1][0], goal_y - odom[-1][1])
            z_values.append(odom[-1][2])
        if (
            seen["waypoints"] > 0
            and seen["local_path"] > 0
            and seen["path_follower_cmd"] > 3
            and seen["mux_cmd"] > 3
            and moved > 0.05
        ):
            break

    state_at_navigation_evidence = nav._state
    deadline = time.time() + 2.5
    while time.time() < deadline and nav._state == "PLANNING":
        time.sleep(0.05)

    print("__LINGTU_POLICY_NAV_RESULT__" + json.dumps({
        "seen": seen,
        "moved": moved,
        "dist_to_goal": dist_to_goal,
        "min_z": min(z_values) if z_values else None,
        "max_z": max(z_values) if z_values else None,
        "nav_state": nav._state,
        "state_at_navigation_evidence": state_at_navigation_evidence,
        "state_history_tail": state_history[-12:],
    }, sort_keys=True))
finally:
    system.stop()
""".replace("__POLICY_PATH__", str(policy_path).replace("\\", "\\\\"))

    env = os.environ.copy()
    repo_root = Path(__file__).resolve().parents[2]
    env["PYTHONPATH"] = os.pathsep.join([str(repo_root / "src"), str(repo_root), env.get("PYTHONPATH", "")])
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=repo_root,
        env=env,
        text=True,
        capture_output=True,
        timeout=45,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    marker = "__LINGTU_POLICY_NAV_RESULT__"
    payloads = [line[len(marker) :] for line in result.stdout.splitlines() if line.startswith(marker)]
    assert payloads, result.stdout + result.stderr
    report = json.loads(payloads[-1])

    seen = report["seen"]
    assert seen["waypoints"] > 0
    assert seen["local_path"] > 0
    assert seen["path_follower_cmd"] > 3
    assert seen["mux_cmd"] > 3
    assert seen["direct_fallback"] == 0
    assert report["moved"] > 0.05
    assert report["dist_to_goal"] < 0.95
    assert report["min_z"] > 0.35
    assert report["max_z"] < 0.50
    assert report["nav_state"] in ("EXECUTING", "SUCCESS"), report


def test_sim_mujoco_full_stack_routes_autonomy_cmds_through_mux():
    pytest.importorskip("mujoco")

    system = thunder_blueprint(
        robot="sim_mujoco",
        world="open_field",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        render=False,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        run_startup_checks=False,
    ).build()

    nav = system.get_module("nav.mission")
    assert system.get_module("nav.velocity_mux") is not None
    recovery_edge = (
        "nav.mission",
        "recovery_cmd_vel",
        "nav.velocity_mux",
        "recovery_cmd_vel",
    )
    if hasattr(nav, "recovery_cmd_vel"):
        assert recovery_edge in system.connections
    else:
        assert recovery_edge not in system.connections
    assert (
        "nav.path_follower",
        "cmd_vel",
        "nav.velocity_mux",
        "path_follower_cmd_vel",
    ) in system.connections
    assert (
        "nav.velocity_mux",
        "driver_cmd_vel",
        "MujocoDriverModule",
        "cmd_vel",
    ) in system.connections
    assert (
        "nav.velocity_mux",
        "driver_cmd_vel",
        "nav.safety",
        "cmd_vel",
    ) in system.connections
    assert (
        "nav.path_follower",
        "cmd_vel",
        "MujocoDriverModule",
        "cmd_vel",
    ) not in system.connections


def test_full_stack_mux_wiring_tolerates_legacy_nav_without_recovery_cmd():
    from lingtu.assembly.full_stack_wiring import apply_full_stack_wires
    from nav.safety.velocity_mux import VelocityMux
    from runtime.blueprint import Blueprint
    from runtime.module import Module
    from runtime.stream import In, Out

    class LegacyNavigation(Module, layer=5):
        stop_signal: In[int]

    class TestDriverModule(Module, layer=1):
        cmd_vel: In[Twist]
        stop_signal: In[int]

    class TestPathFollower(Module, layer=5):
        cmd_vel: Out[Twist]

    class TestSafetyRing(Module, layer=0):
        stop_cmd: Out[int]
        cmd_vel: In[Twist]

    bp = Blueprint()
    bp.add(LegacyNavigation, alias="nav.mission")
    bp.add(TestDriverModule, alias="MujocoDriverModule")
    bp.add(TestPathFollower, alias="nav.path_follower")
    bp.add(TestSafetyRing, alias="nav.safety")
    bp.add(VelocityMux)

    system = apply_full_stack_wires(
        bp,
        robot="sim_mujoco",
        driver_module="MujocoDriverModule",
        slam_profile="none",
        enable_semantic=False,
    ).build()

    assert (
        "nav.mission",
        "recovery_cmd_vel",
        "nav.velocity_mux",
        "recovery_cmd_vel",
    ) not in system.connections
    assert (
        "nav.path_follower",
        "cmd_vel",
        "nav.velocity_mux",
        "path_follower_cmd_vel",
    ) in system.connections
    assert (
        "nav.velocity_mux",
        "driver_cmd_vel",
        "MujocoDriverModule",
        "cmd_vel",
    ) in system.connections
    assert (
        "nav.path_follower",
        "cmd_vel",
        "MujocoDriverModule",
        "cmd_vel",
    ) not in system.connections


def test_full_stack_required_safety_stop_wire_reports_missing_contract():
    from lingtu.assembly.full_stack_wiring import apply_full_stack_wires
    from runtime.blueprint import Blueprint
    from runtime.module import Module
    from runtime.stream import In, Out

    class LegacyNavigation(Module, layer=5):
        pass

    class TestDriverModule(Module, layer=1):
        stop_signal: In[int]

    class TestSafetyRing(Module, layer=0):
        stop_cmd: Out[int]

    bp = Blueprint()
    bp.add(LegacyNavigation, alias="nav.mission")
    bp.add(TestDriverModule, alias="MujocoDriverModule")
    bp.add(TestSafetyRing, alias="nav.safety")

    with pytest.raises(ValueError) as exc:
        apply_full_stack_wires(
            bp,
            robot="sim_mujoco",
            driver_module="MujocoDriverModule",
            slam_profile="none",
            enable_semantic=False,
        )

    assert "Required full-stack wire unavailable" in str(exc.value)
    assert "nav.safety.stop_cmd->nav.mission.stop_signal" in str(exc.value)
    assert "missing destination port nav.mission.stop_signal" in str(exc.value)


def test_full_stack_wires_frontier_exploration_goal_to_navigation():
    system = thunder_blueprint(
        robot="stub",
        slam_profile="none",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        enable_frontier=True,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        run_startup_checks=False,
    ).build()

    assert (
        "WavefrontFrontierExplorer",
        "exploration_goal",
        "nav.mission",
        "goal_pose",
    ) in system.connections
    assert (
        "nav.mission",
        "mission_status",
        "WavefrontFrontierExplorer",
        "navigation_status",
    ) in system.connections


def test_frontier_exploration_goal_reaches_navigation_planner():
    system = thunder_blueprint(
        robot="stub",
        slam_profile="none",
        planner_backend="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        enable_frontier=True,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        frontier_min_size=1,
        frontier_safe_distance=0.0,
        frontier_goal_timeout=0.5,
        frontier_rate=10.0,
        run_startup_checks=False,
    ).build()
    explorer = system.get_module("WavefrontFrontierExplorer")
    nav = system.get_module("nav.mission")

    seen = {"exploration_goals": 0, "waypoints": 0, "paths": 0}
    state_history = []
    explorer.exploration_goal._add_callback(
        lambda _: seen.__setitem__("exploration_goals", seen["exploration_goals"] + 1)
    )
    nav.waypoint._add_callback(lambda _: seen.__setitem__("waypoints", seen["waypoints"] + 1))
    nav.global_path._add_callback(lambda _: seen.__setitem__("paths", seen["paths"] + 1))
    nav.planner_status._add_callback(lambda state: state_history.append(state))

    grid = np.full((50, 50), -1, dtype=np.int16)
    grid[18:33, 18:33] = 0
    costmap = {
        "grid": grid,
        "resolution": 0.2,
        "origin": np.array([-5.0, -5.0], dtype=np.float32),
        "origin_x": -5.0,
        "origin_y": -5.0,
        "width": 50,
        "height": 50,
    }
    odom = Odometry(pose=Pose(0.0, 0.0, 0.0), frame_id="map")

    system.start()
    try:
        explorer.odometry._deliver(odom)
        nav.odometry._deliver(odom)
        explorer.costmap._deliver(costmap)
        nav.costmap._deliver(costmap)

        assert explorer.begin_exploration() == "started"
        deadline = time.time() + 3.0
        while time.time() < deadline and (seen["exploration_goals"] == 0 or seen["waypoints"] == 0):
            time.sleep(0.05)

        assert seen["exploration_goals"] > 0
        assert seen["paths"] > 0
        assert seen["waypoints"] > 0
        assert nav._state in ("EXECUTING", "SUCCESS") or any(
            state in ("EXECUTING", "SUCCESS") for state in state_history
        ), {"nav_state": nav._state, "state_history_tail": state_history[-12:]}
    finally:
        explorer.end_exploration()
        system.stop()


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
