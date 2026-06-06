import importlib.util
import json
import math
import os
import subprocess
import struct
import sys
import time
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from core.tests.numpy_guard import import_numpy_or_skip

pytestmark = [pytest.mark.sim]

np = import_numpy_or_skip()

try:
    import rclpy  # noqa: F401
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False

from sim.engine.core.robot import RobotConfig

from core.blueprints.full_stack import full_stack_blueprint
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from drivers.sim.mujoco_driver_module import MujocoDriverModule


def test_default_nova_dog_resolves_real_robot_model():
    sim_root = Path(__file__).resolve().parents[2] / "sim"
    cfg = RobotConfig.default_nova_dog().resolve_paths(base_dir=str(sim_root))

    assert Path(cfg.robot_xml).name == "thunder_v3_lingtu.xml"
    assert Path(cfg.robot_xml).exists()
    assert cfg.policy_onnx == ""
    assert cfg.base_body_name == "base_link"
    assert cfg.lidar_body_name == "lidar_link"
    assert cfg.leg_act_offset == 0
    assert cfg.leg_joint_names[0] == "FR_hip_joint"


def test_default_nova_dog_resolves_paths_from_engine_core_default():
    sim_root = Path(__file__).resolve().parents[2] / "sim"
    cfg = RobotConfig.default_nova_dog().resolve_paths()

    assert Path(cfg.robot_xml).exists()
    assert Path(cfg.robot_xml).is_relative_to(sim_root)
    assert Path(cfg.robot_xml).name == "thunder_v3_lingtu.xml"
    assert "sim/sim" not in Path(cfg.robot_xml).as_posix()


def test_mujoco_driver_splits_body_lidar_cloud_from_world_map_cloud():
    from core.runtime_interface import FRAMES
    from drivers.sim.mujoco_driver_module import _world_points_to_body_frame

    pts = np.array([[0.0, 1.0, 0.0, 0.5]], dtype=np.float32)
    yaw_90_xyzw = np.array([0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)])

    body_pts = _world_points_to_body_frame(pts, np.zeros(3), yaw_90_xyzw)

    assert body_pts[0, :3] == pytest.approx([1.0, 0.0, 0.0], abs=1e-6)
    assert FRAMES.body == "body"
    assert FRAMES.odom == "odom"


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
    assert importlib.util.find_spec("semantic_perception.instance_tracker") is not None
    assert importlib.util.find_spec("semantic_planner.llm_client") is not None

    # Canonical imports from core.utils
    from core.msgs import scene as scene_msgs
    from core.utils.robustness import retry
    from core.utils.sanitize import sanitize_position
    from core.utils.validation import validate_bgr
    from semantic.perception.instance_tracker import InstanceTracker
    from semantic.perception.tracked_objects import TrackedObject

    assert callable(sanitize_position)
    assert InstanceTracker is not None
    assert scene_msgs.TrackedObject is TrackedObject


def test_rosbag_slam_bridge_replay_classifies_algorithm_vs_raw_topics():
    from sim.scripts.rosbag_slam_bridge_replay import _choose_topic, _topic_class

    slam = _topic_class("/Odometry", "/cloud_registered")
    assert slam["slam_algorithm_output_verified"] is True
    assert slam["raw_sensor_bag_only"] is False

    raw = _topic_class("/state_SDK", "/points_raw")
    assert raw["slam_algorithm_output_verified"] is False
    assert raw["raw_sensor_bag_only"] is True

    topics = {
        "/state_SDK": "nav_msgs/msg/Odometry",
        "/points_raw": "sensor_msgs/msg/PointCloud2",
    }
    assert _choose_topic(topics, ["/Odometry", "/state_SDK"], "") == "/state_SDK"
    assert _choose_topic(topics, ["/Odometry"], "/state_SDK") == "/state_SDK"


def test_fastlio2_rosbag_replay_config_targets_raw_bag_topics(tmp_path):
    from sim.scripts.fastlio2_rosbag_replay_gate import _write_fastlio2_config

    config = tmp_path / "fastlio2_replay.yaml"
    _write_fastlio2_config(config, imu_topic="/imu_raw", lidar_topic="/points_raw")
    text = config.read_text(encoding="utf-8")

    assert "imu_topic: /imu_raw" in text
    assert "lidar_topic: /points_raw" in text
    assert "lidar_type: 3" in text
    assert "acc_scale: 1.0" in text


def test_fastlio2_config_can_target_timed_pointcloud2(tmp_path):
    from sim.scripts.fastlio2_rosbag_replay_gate import _write_fastlio2_config

    config = tmp_path / "fastlio2_live.yaml"
    _write_fastlio2_config(
        config,
        imu_topic="/imu_raw",
        lidar_topic="/points_raw",
        lidar_type=2,
        scan_line=4,
        timestamp_unit=0,
    )
    text = config.read_text(encoding="utf-8")

    assert "lidar_type: 2" in text
    assert "scan_line: 4" in text
    assert "timestamp_unit: 0" in text


def test_fastlio2_config_can_target_livox_custom_msg(tmp_path):
    from sim.scripts.fastlio2_rosbag_replay_gate import _write_fastlio2_config

    config = tmp_path / "fastlio2_livox.yaml"
    _write_fastlio2_config(
        config,
        imu_topic="/imu_raw",
        lidar_topic="/points_raw",
        lidar_type=1,
        scan_line=4,
        timestamp_unit=3,
        livox_scan_window=0.0,
    )
    text = config.read_text(encoding="utf-8")

    assert "lidar_type: 1" in text
    assert "timestamp_unit: 3" in text
    assert "livox_scan_window: 0.000000" in text


def test_fastlio2_config_can_write_lidar_imu_extrinsic(tmp_path):
    from sim.scripts.fastlio2_rosbag_replay_gate import _write_fastlio2_config

    config = tmp_path / "fastlio2_extrinsic.yaml"
    _write_fastlio2_config(
        config,
        imu_topic="/imu_raw",
        lidar_topic="/points_raw",
        t_il=(0.0, 0.0, 0.28),
    )
    text = config.read_text(encoding="utf-8")

    assert "r_il: [1, 0, 0, 0, 1, 0, 0, 0, 1]" in text
    assert "t_il: [0, 0, 0.28]" in text


def test_fastlio2_config_can_disable_sim_zupt(tmp_path):
    from sim.scripts.fastlio2_rosbag_replay_gate import _write_fastlio2_config

    config = tmp_path / "fastlio2_no_zupt.yaml"
    _write_fastlio2_config(
        config,
        imu_topic="/imu_raw",
        lidar_topic="/points_raw",
        imu_static_acc_thresh=0.0,
        imu_static_gyro_thresh=0.0,
        zupt_min_static_frames=1_000_000,
    )
    text = config.read_text(encoding="utf-8")

    assert "imu_static_acc_thresh: 0" in text
    assert "imu_static_gyro_thresh: 0" in text
    assert "zupt_min_static_frames: 1000000" in text


def test_fastlio2_live_config_can_write_time_diff_control(tmp_path):
    from slam.fastlio2_live_bridge import write_fastlio2_config

    config = tmp_path / "fastlio2_live_time_diff.yaml"
    write_fastlio2_config(
        config,
        imu_topic="/imu_raw",
        lidar_topic="/points_raw",
        time_diff_lidar_to_imu=0.0125,
    )
    text = config.read_text(encoding="utf-8")

    assert "time_diff_lidar_to_imu: 0.0125" in text


def test_fastlio2_live_config_writes_degeneracy_guard(tmp_path):
    from slam.fastlio2_live_bridge import write_fastlio2_config

    config = tmp_path / "fastlio2_live_degeneracy_guard.yaml"
    write_fastlio2_config(
        config,
        imu_topic="/imu_raw",
        lidar_topic="/points_raw",
        degeneracy_max_update_dof=2,
        degeneracy_max_condition=50_000.0,
        max_update_translation_m=0.5,
        max_update_rotation_rad=0.35,
        reject_nonconverged_update=True,
        reject_degenerate_nonconverged_update=True,
        vertical_velocity_constraint_enabled=True,
    )
    text = config.read_text(encoding="utf-8")

    assert "degeneracy_max_update_dof: 2" in text
    assert "degeneracy_max_condition: 50000" in text
    assert "max_update_translation_m: 0.5" in text
    assert "max_update_rotation_rad: 0.35" in text
    assert "reject_nonconverged_update: true" in text
    assert "reject_degenerate_nonconverged_update: true" in text
    assert "vertical_velocity_constraint_enabled: true" in text
    assert "vertical_velocity_sigma_v: 0.05" in text


def test_fastlio2_cpp_applies_configured_ieskf_iteration_and_degeneracy_guard():
    root = Path(__file__).resolve().parents[2]
    lidar_processor = (
        root / "src" / "slam" / "fastlio2" / "src" / "map_builder" / "lidar_processor.cpp"
    ).read_text(encoding="utf-8")
    ieskf = (
        root / "src" / "slam" / "fastlio2" / "src" / "map_builder" / "ieskf.cpp"
    ).read_text(encoding="utf-8")
    lio_node = (root / "src" / "slam" / "fastlio2" / "src" / "lio_node.cpp").read_text(
        encoding="utf-8"
    )

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
    from sim.scripts.mujoco_fastlio2_live_gate import _world_xyzi_to_sensor_xyzi

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


def test_mujoco_fastlio2_live_gate_converts_sensor_cloud_to_body_frame():
    from core.runtime_interface import lidar_extrinsic
    from sim.scripts.mujoco_fastlio2_live_gate import _sensor_xyzi_to_body_xyzi

    sensor_cloud = np.array(
        [
            [1.0, 2.0, 3.0, 42.0],
            [-0.5, -1.0, 0.0, 12.0],
        ],
        dtype=np.float32,
    )

    body_cloud = _sensor_xyzi_to_body_xyzi(
        sensor_cloud,
        lidar_extrinsic("mujoco_thunder_v3"),
    )

    np.testing.assert_allclose(body_cloud[:, :3], [[1.0, 2.0, 3.28], [-0.5, -1.0, 0.28]])
    np.testing.assert_allclose(body_cloud[:, 3], [42.0, 12.0])


def test_mujoco_fastlio2_live_gate_stationary_imu_specific_force_points_up():
    from sim.scripts.mujoco_fastlio2_live_gate import _specific_force_body

    state = types.SimpleNamespace(
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        linear_velocity=np.zeros(3),
    )

    acc = _specific_force_body(state, np.zeros(3), 0.02)

    np.testing.assert_allclose(acc, [0.0, 0.0, 9.81], atol=1e-6)


def test_mujoco_fastlio2_live_gate_gravity_only_imu_ignores_kinematic_velocity_step():
    from sim.scripts.mujoco_fastlio2_live_gate import _specific_force_body

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
    from sim.scripts.mujoco_fastlio2_live_gate import _make_imu_msg

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

    msg = _make_imu_msg(
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
    from sim.scripts.mujoco_fastlio2_live_gate import _build_parser

    args = _build_parser().parse_args([])

    assert args.imu_acc_mode == "finite_difference"
    assert args.max_fastlio_z_drift_m == 1.0
    assert args.max_fastlio_yaw_drift_rad == 0.5


def test_mujoco_fastlio2_live_gate_exposes_wall_timeout_guard():
    from sim.scripts.mujoco_fastlio2_live_gate import (
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


def test_launch_mujoco_fastlio2_live_passes_wall_timeout_guard():
    text = Path("sim/scripts/launch_mujoco_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "--max-wall-time-s" in text
    assert "LINGTU_MUJOCO_LIVE_MAX_WALL_TIME_S" in text


def test_launch_mujoco_fastlio2_live_defaults_mid360_lidar_to_rolling_scan_time():
    text = Path("sim/scripts/launch_mujoco_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "--scan-time-profile" in text
    assert '${LINGTU_MUJOCO_LIVE_SCAN_TIME_PROFILE:-physical_rolling}' in text
    assert '${LINGTU_MUJOCO_LIVE_MID360_SAMPLES_PER_FRAME:-15000}' in text


def test_launch_mujoco_fastlio2_live_uses_sim_clock_inspection_timeout_default():
    text = Path("sim/scripts/launch_mujoco_fastlio2_live.sh").read_text(encoding="utf-8")

    assert 'inspection_default_goal_timeout="${LINGTU_MUJOCO_LIVE_INSPECTION_GOAL_TIMEOUT:-900}"' in text
    assert 'if [[ "$duration_clock" != "sim"' in text
    assert '"--inspection-goal-timeout" "$inspection_default_goal_timeout"' in text


def test_mujoco_fastlio2_live_gate_accepts_fastlio_tuning_args():
    from sim.scripts.mujoco_fastlio2_live_gate import _build_parser

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
    from sim.scripts.mujoco_fastlio2_live_gate import _build_parser

    args = _build_parser().parse_args(["--fastlio-time-diff-lidar-to-imu", "-0.0075"])

    assert args.fastlio_time_diff_lidar_to_imu == pytest.approx(-0.0075)


def test_mujoco_fastlio2_live_gate_accepts_vertical_velocity_constraint_arg():
    from sim.scripts.mujoco_fastlio2_live_gate import _build_parser

    default_args = _build_parser().parse_args([])
    explicit_args = _build_parser().parse_args(
        ["--fastlio-vertical-velocity-constraint", "off"]
    )

    assert default_args.fastlio_vertical_velocity_constraint == "off"
    assert explicit_args.fastlio_vertical_velocity_constraint == "off"


def test_mujoco_fastlio2_live_gate_accepts_turn_speed_coupling_args():
    from sim.scripts.mujoco_fastlio2_live_gate import _build_parser

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


def test_launch_mujoco_fastlio2_live_passes_fastlio_time_diff_control():
    text = Path("sim/scripts/launch_mujoco_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "--fastlio-time-diff-lidar-to-imu" in text
    assert "LINGTU_MUJOCO_LIVE_FASTLIO_TIME_DIFF_LIDAR_TO_IMU" in text
    assert "--fastlio-vertical-velocity-constraint" in text
    assert "LINGTU_MUJOCO_LIVE_FASTLIO_VERTICAL_VELOCITY_CONSTRAINT" in text


def test_launch_mujoco_fastlio2_live_passes_turn_speed_coupling_controls():
    text = Path("sim/scripts/launch_mujoco_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "--nav-turn-speed-yaw-rate-start" in text
    assert "LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_YAW_RATE_START" in text
    assert "--nav-turn-speed-min-scale" in text
    assert "LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_MIN_SCALE" in text


def test_mujoco_fastlio2_live_gate_reports_fastlio_observability_warnings(tmp_path: Path):
    from sim.scripts.mujoco_fastlio2_live_gate import _fastlio2_log_diagnostics

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
    from sim.scripts.mujoco_fastlio2_live_gate import _summarize_degeneracy_detail_samples

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
    from sim.scripts.mujoco_fastlio2_live_gate import _update_runtime_fault_streak

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
    from sim.scripts.mujoco_fastlio2_live_gate import _nearest_sim_pose_sample

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
    from sim.scripts.mujoco_fastlio2_live_gate import _fastlio_large_loop_diagnostic_report

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


def test_fastlio2_nav_bridge_records_odom_header_stamps():
    from slam.fastlio2_nav_bridge import FastLio2NavBridgeRuntime

    class FakePublisher:
        def publish(self, _msg):
            pass

    class FakeNode:
        def create_publisher(self, *_args, **_kwargs):
            return FakePublisher()

        def create_subscription(self, *_args, **_kwargs):
            return object()

    class FakeSlamBridge:
        _odom_worker_thread = None

        def _on_rclpy_odom(self, _msg):
            pass

        def _process_rclpy_cloud(self, _msg):
            pass

    msg = types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=12, nanosec=340_000_000),
            frame_id="odom",
        )
    )

    bridge = FastLio2NavBridgeRuntime(
        node=FakeNode(),
        slam_bridge=FakeSlamBridge(),
        odometry_cls=object,
        pointcloud2_cls=object,
        odom_xyz=lambda _msg: [1.0, 2.0, 3.0],
        odom_yaw=lambda _msg: 0.5,
    )
    bridge.on_odom(msg)

    assert bridge.first_odom_stamp_s == pytest.approx(12.34)
    assert bridge.last_odom_stamp_s == pytest.approx(12.34)


def test_fastlio2_nav_bridge_uses_runtime_default_frames():
    from core.runtime_interface import TOPICS, topic_default_frame_id
    from slam.fastlio2_nav_bridge import FastLio2NavBridgeRuntime

    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    class FakeNode:
        def __init__(self):
            self.publishers = {}

        def create_publisher(self, _msg_cls, topic, _qos):
            publisher = FakePublisher()
            self.publishers[topic] = publisher
            return publisher

        def create_subscription(self, *_args, **_kwargs):
            return object()

    class FakeSlamBridge:
        _odom_worker_thread = None

        def __init__(self):
            self.odom = []
            self.clouds = []

        def _on_rclpy_odom(self, msg):
            self.odom.append(msg)

        def _process_rclpy_cloud(self, msg):
            self.clouds.append(msg)

    def msg(frame_id):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=1, nanosec=0),
                frame_id=frame_id,
            )
        )

    node = FakeNode()
    slam_bridge = FakeSlamBridge()
    bridge = FastLio2NavBridgeRuntime(
        node=node,
        slam_bridge=slam_bridge,
        odometry_cls=object,
        pointcloud2_cls=object,
    )

    bridge.on_odom(msg("raw_odom"))
    bridge.on_registered_cloud(msg("raw_registered"))
    bridge.on_map_cloud(msg("raw_map"))

    assert node.publishers[TOPICS.odometry].messages[0].header.frame_id == (
        topic_default_frame_id(TOPICS.odometry)
    )
    assert node.publishers[TOPICS.registered_cloud].messages[0].header.frame_id == (
        topic_default_frame_id(TOPICS.registered_cloud)
    )
    assert node.publishers[TOPICS.map_cloud].messages[0].header.frame_id == (
        topic_default_frame_id(TOPICS.odometry)
    )
    assert slam_bridge.odom[0].header.frame_id == topic_default_frame_id(TOPICS.odometry)
    assert slam_bridge.clouds[0].header.frame_id == topic_default_frame_id(
        TOPICS.odometry)


def test_fastlio2_nav_bridge_rejects_messages_without_frame_header():
    from slam.fastlio2_nav_bridge import FastLio2NavBridgeRuntime

    msg = types.SimpleNamespace(header=types.SimpleNamespace())

    with pytest.raises(ValueError, match="header.frame_id"):
        FastLio2NavBridgeRuntime.nav_frame_msg(msg, "odom")


def test_fastlio2_live_bridge_writes_tunable_sim_config(tmp_path: Path):
    from slam.fastlio2_live_bridge import write_fastlio2_config

    config = tmp_path / "fastlio_live.yaml"

    write_fastlio2_config(
        config,
        imu_topic="/imu_raw",
        lidar_topic="/points_raw",
        lidar_filter_num=2,
        scan_resolution=0.1,
        map_resolution=0.2,
        near_search_num=8,
        ieskf_max_iter=8,
        lidar_cov_inv=500.0,
    )

    text = config.read_text(encoding="utf-8")
    assert "lidar_filter_num: 2" in text
    assert "scan_resolution: 0.1" in text
    assert "map_resolution: 0.2" in text
    assert "near_search_num: 8" in text
    assert "ieskf_max_iter: 8" in text
    assert "lidar_cov_inv: 500" in text


def test_mujoco_fastlio2_live_gate_pointcloud_includes_time_and_ring():
    from sim.scripts.mujoco_fastlio2_live_gate import _make_pointcloud2

    class FakeHeader:
        def __init__(self):
            self.stamp = None
            self.frame_id = ""

    class FakePointCloud2:
        def __init__(self):
            self.header = FakeHeader()

    class FakePointField:
        UINT16 = 4
        FLOAT32 = 7

        def __init__(self, *, name, offset, datatype, count):
            self.name = name
            self.offset = offset
            self.datatype = datatype
            self.count = count

    msg = _make_pointcloud2(
        points_xyzi=np.array([[1.0, 2.0, 3.0, 10.0], [4.0, 5.0, 6.0, 20.0]], dtype=np.float32),
        stamp=types.SimpleNamespace(),
        frame_id="body",
        pointcloud_cls=FakePointCloud2,
        pointfield_cls=FakePointField,
        relative_times_s=np.array([0.0, 0.05], dtype=np.float32),
        rings=np.array([0, 3], dtype=np.uint16),
    )

    assert msg.point_step == 24
    assert msg.row_step == 48
    assert {field.name: field.offset for field in msg.fields} == {
        "x": 0,
        "y": 4,
        "z": 8,
        "intensity": 12,
        "time": 16,
        "ring": 20,
    }
    assert struct.unpack_from("<fffffH", msg.data, 24) == pytest.approx(
        (4.0, 5.0, 6.0, 20.0, 0.05, 3)
    )


def test_mujoco_fastlio2_live_gate_livox_custom_msg_uses_offset_time():
    from sim.scripts.mujoco_fastlio2_live_gate import _make_livox_custom_msg

    class FakeHeader:
        def __init__(self):
            self.stamp = None
            self.frame_id = ""

    class FakeCustomMsg:
        def __init__(self):
            self.header = FakeHeader()
            self.timebase = 0
            self.lidar_id = 0
            self.rsvd = []
            self.points = []

    class FakeCustomPoint:
        def __init__(self):
            self.offset_time = 0
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.reflectivity = 0
            self.tag = 0
            self.line = 0

    stamp = types.SimpleNamespace(sec=10, nanosec=20)
    msg = _make_livox_custom_msg(
        points_xyzi=np.array([[1.0, 2.0, 3.0, 10.0], [4.0, 5.0, 6.0, 300.0]], dtype=np.float32),
        stamp=stamp,
        frame_id="body",
        custom_msg_cls=FakeCustomMsg,
        custom_point_cls=FakeCustomPoint,
        relative_times_s=np.array([0.0, 0.05], dtype=np.float32),
        rings=np.array([0, 3], dtype=np.uint16),
    )

    assert msg.header.stamp is stamp
    assert msg.header.frame_id == "body"
    assert msg.timebase == 10_000_000_020
    assert msg.point_num == 2
    assert msg.points[1].offset_time == 50_000_000
    assert msg.points[1].reflectivity == 255
    assert msg.points[1].tag == 0x10
    assert msg.points[1].line == 3


def test_mujoco_fastlio2_live_gate_exposes_scan_time_profiles():
    from sim.scripts.mujoco_fastlio2_live_gate import (
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
    physical_sensor, physical_world, physical_times, moving_count, subscan_count = (
        _physical_rolling_scan_from_samples(
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
    from sim.scripts.mujoco_fastlio2_live_gate import (
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
    from sim.scripts.mujoco_fastlio2_live_gate import _navigation_diagnostic_sample

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
            "failure_reason": "",
            "last_plan_report": {
                "primary_planner": "pct",
                "selected_planner": "pct",
                "fallback_reason": "",
            },
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
    assert sample["navigation"]["selected_planner"] == "pct"
    assert sample["paths"]["global_path_count"] == 2
    assert sample["paths"]["local_path_points_latest"] == 18
    assert sample["runtime_fault_count"] == 1


def test_mujoco_fastlio2_live_gate_emits_navigation_diagnostics_in_report():
    root = Path(__file__).resolve().parents[2]
    source = (root / "sim/scripts/mujoco_fastlio2_live_gate.py").read_text(encoding="utf-8")

    assert "navigation_diagnostic_samples.append(" in source
    assert '"navigation_diagnostics": navigation_diagnostics' in source
    assert '"stale_nav_cmd_samples"' in source
    assert '"path_diagnostics":' in source


def test_mujoco_fastlio2_live_gate_summarizes_path_geometry():
    from sim.scripts.mujoco_fastlio2_live_gate import _path_summary

    path = types.SimpleNamespace(
        poses=[
            types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
                )
            ),
            types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=3.0, y=4.0, z=0.0)
                )
            ),
        ]
    )

    summary = _path_summary(path)

    assert summary["point_count"] == 2
    assert summary["finite_point_count"] == 2
    assert summary["path_length_m"] == pytest.approx(5.0)
    assert summary["first_xyz"] == pytest.approx([0.0, 0.0, 0.0])
    assert summary["last_xyz"] == pytest.approx([3.0, 4.0, 0.0])
    assert summary["bounds_xy"] == pytest.approx([0.0, 0.0, 3.0, 4.0])


def test_mujoco_truth_nav_pose_aligns_to_tomogram_map_frame(tmp_path: Path):
    from types import SimpleNamespace

    from sim.scripts.mujoco_fastlio2_live_gate import (
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
    from sim.scripts.mujoco_fastlio2_live_gate import _limit_command_delta

    limited = _limit_command_delta(
        target=(0.25, -0.1, 0.4),
        previous=(0.0, 0.0, 0.0),
        dt_s=0.02,
        linear_accel_limit=0.5,
        angular_accel_limit=1.0,
    )

    assert limited == pytest.approx((0.01, -0.01, 0.02))


def test_mujoco_fastlio2_live_gate_rejects_large_translation_scale_error():
    from sim.scripts.mujoco_fastlio2_live_gate import _motion_consistency_report

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
    from sim.scripts.mujoco_fastlio2_live_gate import _select_nav_cmd_for_step

    selected = _select_nav_cmd_for_step(
        latest_nav_cmd={"vx": 0.2, "vy": -0.03, "wz": 0.1, "stamp": 100.0},
        now_s=105.0,
        cmd_vel_timeout_s=0.0,
    )

    assert selected["fresh"] is True
    assert selected["vx"] == pytest.approx(0.2)
    assert selected["vy"] == pytest.approx(-0.03)
    assert selected["wz"] == pytest.approx(0.1)


def test_mujoco_fastlio2_live_gate_rejects_spin_without_translation():
    from sim.scripts.mujoco_fastlio2_live_gate import _control_quality_report

    report = _control_quality_report(
        applied_cmd_stats={
            "linear_distance_integral_m": 1.1,
            "angular_abs_integral_rad": 3.2,
            "max_angular_abs": 0.45,
            "angular_saturation_samples": 80,
            "cmd_samples": 100,
        },
        max_yaw_per_meter=1.2,
        max_angular_saturation_ratio=0.35,
    )

    assert report["ok"] is False
    assert "yaw_per_meter too high" in report["blockers"]
    assert "angular saturation ratio too high" in report["blockers"]


def test_mujoco_fastlio2_live_gate_summarizes_dynamic_obstacle_sweep_quality():
    from sim.scripts.mujoco_fastlio2_live_gate import _dynamic_obstacle_sweep_quality

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


def test_launch_mujoco_fastlio2_live_exposes_cmd_vel_timeout_override():
    text = Path("sim/scripts/launch_mujoco_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "--cmd-vel-timeout" in text
    assert "LINGTU_MUJOCO_LIVE_CMD_VEL_TIMEOUT" in text
    assert 'cmd_vel_timeout_default="0"' in text
    assert 'drive_source" == "nav_cmd_vel"' in text


def test_mujoco_fastlio2_live_gate_robot_crossing_obstacles_scale_density_and_speed():
    from sim.scripts.mujoco_fastlio2_live_gate import (
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
    from pathlib import Path
    from drivers.sim.mujoco_driver_module import WORLDS, _WORLDS_DIR

    world_file = WORLDS["industrial_demo"]

    assert world_file == "industrial_demo_scene.xml"
    assert (_WORLDS_DIR / world_file).is_file()
    assert "robot_placeholder" in (_WORLDS_DIR / world_file).read_text(encoding="utf-8")


def test_mujoco_world_registry_includes_product_industrial_park_scene():
    from drivers.sim.mujoco_driver_module import WORLDS, _WORLDS_DIR

    world_file = WORLDS["industrial_park"]
    text = (_WORLDS_DIR / world_file).read_text(encoding="utf-8")

    assert world_file == "industrial_park_scene.xml"
    assert (_WORLDS_DIR / world_file).is_file()
    assert text.isascii()
    assert "robot_placeholder" in text


def test_mujoco_fastlio2_live_gate_relays_fastlio_outputs_to_nav_topics():
    from pathlib import Path
    from core.runtime_interface import TOPICS
    from sim.scripts import mujoco_fastlio2_live_gate

    source = Path(mujoco_fastlio2_live_gate.__file__).read_text(encoding="utf-8")
    bridge_source = Path("src/slam/fastlio2_nav_bridge.py").read_text(encoding="utf-8")
    stack_source = Path("src/drivers/sim/mujoco_lingtu_stack.py").read_text(encoding="utf-8")
    launcher_source = Path("sim/scripts/launch_mujoco_fastlio2_live.sh").read_text(encoding="utf-8")

    assert "resolved_runtime_data_flow" in source
    assert "FastLio2NavBridgeRuntime" in source
    assert "node.create_publisher(" in bridge_source
    assert "TOPICS.odometry" in bridge_source
    assert "TOPICS.registered_cloud" in bridge_source
    assert "TOPICS.map_cloud" in bridge_source
    assert "create_subscription(TwistStamped, TOPICS.cmd_vel" in source
    assert "topic_default_frame_id(TOPICS.odometry)" in bridge_source
    assert "topic_default_frame_id(TOPICS.registered_cloud)" in bridge_source
    assert "from core.runtime_interface import FRAMES" not in bridge_source
    assert '"nav_odometry": SIM_NAV_ODOMETRY_FRAME_ID' in source
    assert '"nav_registered_cloud": SIM_NAV_REGISTERED_CLOUD_FRAME_ID' in source
    assert '"nav_map_cloud": SIM_FASTLIO_LIVE_MAP_FRAME_ID' in source
    assert "FRAMES.odom" not in source
    assert "FRAMES.body" not in source
    assert "FRAMES.lidar" not in source
    assert "FRAMES.map" not in source
    assert TOPICS.raw_lidar_points == "/points_raw"
    assert TOPICS.raw_imu == "/imu_raw"
    assert '--run-lingtu-frontier' in source
    assert 'drive_source == "nav_cmd_vel"' in source
    assert 'frontier_blockers' in source
    assert "min_map_area_growth_m2" in source
    assert "exploration_known_area" in source
    assert "exploration_grid_growth_is_acceptance_metric" in source
    assert "_pointcloud_xy_stats" in source
    assert "motion_consistency" in source
    assert "base_blockers" in source
    assert "Fast-LIO odometry diverged from MuJoCo motion" in source
    assert "canonical_nav_outputs_verified" in source
    assert "--nav-data-source" in source
    assert "mujoco_ground_truth" in source
    assert "demo_visualization_mode" in source
    assert 'plan_safety_policy="reject"' in stack_source
    assert "frontier_started_after_slam_ready" in source
    assert "navigation_chain" in source
    assert "planner_fallback_used" in source
    assert "frontier_mission_events" in source
    assert "successful_navigation_goal_count" in source
    assert "frontier successful navigation count" in source
    assert "relative_times_s" in source
    assert "fastlio2_lidar_type" in source
    assert "livox_custom_msg" in source
    assert "_make_livox_custom_msg" in source
    assert "require_livox=fastlio_lidar_input == \"livox_custom_msg\"" in source
    assert "--fastlio-lidar-input timed_pointcloud2" in source
    assert "sensor_timestamp_source" in source
    assert "engine.sim_time" in source
    assert "frame_id=SIM_LIDAR_FRAME_ID" in source
    assert "body_to_lidar_m" in source
    assert "fastlio2_lidar_frame" in source
    assert "fastlio2_log_diagnostics" in source
    assert "--drive-vx" in launcher_source
    assert "--drive-vy" in launcher_source
    assert "LINGTU_MUJOCO_LIVE_DRIVE_VY" in launcher_source
    assert "LINGTU_MUJOCO_LIVE_DRIVE_WZ" in launcher_source


def test_mujoco_fastlio2_live_gate_exception_report_keeps_runtime_contract():
    from sim.scripts.mujoco_fastlio2_live_gate import _gate_exception_report

    args = types.SimpleNamespace(
        duration_clock="wall",
        fastlio_lidar_input="livox_custom_msg",
        mid360_pattern=str(
            Path("sim/assets/livox/mid360.npy").resolve()
        ),
        mid360_samples_per_frame=1200,
        n_rays=6400,
        scan_time_profile="physical_rolling",
    )
    report = _gate_exception_report(
        args,
        RuntimeError("ROS2 Python modules are unavailable"),
    )

    assert report["ok"] is False
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["runtime_contract"]["name"] == "mujoco_fastlio2_live"
    assert report["runtime_contract"]["ok"] is False
    assert sorted(report["runtime_contract"]["frame_evidence"]) == [
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
    assert "runtime_contract.ok is not true" in gaps
    assert "frame evidence missing or failed for map_to_odom" in gaps
    assert "data-flow evidence missing or failed for endpoint_adapter" in gaps


def test_mujoco_data_flow_marks_unrequired_navigation_stages_not_run():
    from core.runtime_interface import TOPICS
    from sim.scripts.mujoco_fastlio2_live_gate import _mujoco_data_flow_evidence

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
    assert evidence["global_planning"]["owner"] == "lingtu_navigation_or_pct"
    assert evidence["global_planning"]["frame_role"] == "map"
    assert evidence["global_planning"]["map_dependency"] == (
        "pct_uses_same_source_saved_tomogram;"
        "astar_or_frontier_may_use_live_occupancy_grid"
    )
    assert evidence["global_planning"]["reason"] == "not_required_for_basic_slam_gate"
    assert evidence["command_boundary"]["required"] is False
    assert evidence["command_boundary"]["ok"] is False
    assert evidence["command_boundary"]["outputs"] == ["mujoco_velocity_adapter"]
    assert evidence["command_boundary"]["owner"] == "cmd_vel_mux_to_endpoint_sink"
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
    ):
        self.robot_config = robot_config
        self.world_config = world_config
        self.lidar_config = lidar_config
        self.camera_configs = camera_configs
        self.headless = headless
        self.drive_mode = drive_mode
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
    assert Path(driver._engine.robot_config.robot_xml).name == "thunder_v3_lingtu.xml"
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


def test_mujoco_driver_policy_candidates_prioritize_brainstem_default():
    import drivers.sim.mujoco_driver_module as driver_mod

    first = driver_mod._POLICY_CANDIDATES[0]
    assert first.name == "policy_251119.onnx"
    assert first.parent.name == "model"
    assert first.parent.parent.name == "nova_dog"
    assert driver_mod._POLICY_CANDIDATES[-2].name == "policy.onnx"
    assert driver_mod._POLICY_CANDIDATES[-1].name == "thunder_policy.onnx"


def test_legacy_nova_nav_bridge_uses_current_robot_paths():
    bridge = Path(__file__).resolve().parents[2] / "src" / "drivers" / "sim" / "nova_nav_bridge.py"
    source = bridge.read_text(encoding="utf-8")

    assert 'SIM_DIR / "robot"' not in source
    assert 'SIM_DIR / "robots" / "nova_dog" / "robot_with_camera.xml"' in source
    assert 'SIM_DIR / "robots" / "nova_dog" / "policy.onnx"' in source
    assert '<include file="robot_with_camera.xml"/>' in source


def test_legacy_sim_launch_global_planner_entrypoint_exists_and_is_guarded():
    repo_root = Path(__file__).resolve().parents[2]
    launch = (repo_root / "sim" / "launch" / "sim.launch.py").read_text(
        encoding="utf-8"
    )
    wrapper = repo_root / "sim" / "scripts" / "run_global_planner.py"

    assert 'sim_dir / "scripts" / "run_global_planner.py"' in launch
    assert wrapper.exists()
    source = wrapper.read_text(encoding="utf-8")
    assert "ROS_DOMAIN_ID" in source
    assert "isolated simulation domain" in source
    assert "/nav/global_path" in source
    assert "cmd_vel" not in source


def test_legacy_manual_nova_scripts_default_to_current_robot_asset_paths():
    repo_root = Path(__file__).resolve().parents[2]

    for script_name in ("test_factory_nova.sh", "test_semantic_nav.sh"):
        source = (repo_root / "sim" / "scripts" / script_name).read_text(
            encoding="utf-8"
        )
        assert "/tmp/nova_sim" not in source
        assert "/robot/factory_nova_scene.xml" not in source
        assert "robots/nova_dog/robot_with_camera.xml" in source
        assert "LINGTU_NOVA_SCENE_XML" in source
        assert "LINGTU_SIM_DIR" in source


def test_optional_go1_asset_contract_has_placeholder_readme():
    repo_root = Path(__file__).resolve().parents[2]
    indoor_office = (repo_root / "sim" / "worlds" / "mujoco" / "indoor_office.xml").read_text(
        encoding="utf-8"
    )
    readme = repo_root / "sim" / "robots" / "go1_playground" / "README.md"

    assert "../robots/go1_playground/xmls/go1_mjx_feetonly.xml" in indoor_office
    assert readme.exists()
    text = readme.read_text(encoding="utf-8")
    assert "optional external assets" in text
    assert "not part of the G4 server closure" in text


def test_root_operation_scripts_do_not_point_at_deleted_navigation_launches():
    repo_root = Path(__file__).resolve().parents[2]
    shell_entry = (repo_root / "scripts" / "lingtu.sh").read_text(encoding="utf-8")
    ota_start = (repo_root / "scripts" / "ota" / "start_nav.sh").read_text(
        encoding="utf-8"
    )
    ota_install = (repo_root / "scripts" / "ota" / "install_nav.sh").read_text(
        encoding="utf-8"
    )
    pct_profile = (
        repo_root / "launch" / "profiles" / "planner_pct_py.launch.py"
    ).read_text(encoding="utf-8")
    scripts_index = (repo_root / "scripts" / "README.md").read_text(encoding="utf-8")

    for source in (shell_entry, ota_start, ota_install, pct_profile):
        assert "navigation_run.launch.py" not in source
        assert "navigation_bringup.launch.py" not in source
        assert "launch/subsystems/planning.launch.py" not in source

    assert 'WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"' in shell_entry
    assert 'LINGTU_CLI="$WORKSPACE_DIR/lingtu.py"' in shell_entry
    assert "_run_lingtu map" in shell_entry
    assert "_run_lingtu nav" in shell_entry
    assert "_run_lingtu status" in shell_entry
    assert '"$NAV_DIR/lingtu.py" nav' in ota_start
    assert "python3 \\$NAV_DIR/lingtu.py nav" in ota_install
    assert "Shell 兼容入口" in scripts_index
    assert "机器人端 `scripts/lingtu`" in scripts_index


def test_sim_boundary_indexes_document_stable_contracts():
    repo_root = Path(__file__).resolve().parents[2]
    readme_text = (repo_root / "sim" / "README.md").read_text(encoding="utf-8")
    repo_layout = (repo_root / "docs" / "REPO_LAYOUT.md").read_text(encoding="utf-8")
    plan_text = (
        repo_root
        / "docs"
        / "superpowers"
        / "plans"
        / "2026-05-31-sim-folder-modularization-goals.md"
    ).read_text(encoding="utf-8")
    scripts_index = (repo_root / "sim" / "scripts" / "README.md").read_text(
        encoding="utf-8"
    )
    launch_index = (repo_root / "sim" / "launch" / "README.md").read_text(
        encoding="utf-8"
    )
    engine_index = (repo_root / "sim" / "engine" / "README.md").read_text(
        encoding="utf-8"
    )
    closure_index = (
        repo_root / "artifacts" / "server_sim_closure" / "README.md"
    ).read_text(encoding="utf-8")
    sim_root = Path(__file__).resolve().parents[2] / "sim"
    expected = {
        "bridge": [
            "thin entrypoints",
            "do not re-add",
            "sim/robot/",
        ],
        "datasets": [
            "offline replay inputs",
            "artifacts/",
            "not `sim/datasets/`",
        ],
        "sensors": [
            "hardware-free",
            "sim/assets/livox/",
            "Do not add ROS 2 or robot service startup side effects",
        ],
    }

    for folder, markers in expected.items():
        readme = sim_root / folder / "README.md"
        assert readme.exists()
        text = readme.read_text(encoding="utf-8")
        for marker in markers:
            assert marker in text

    for directory in (
        "validation/",
        "evaluation/",
        "external_scenes/",
        "meshes/",
    ):
        assert f"`{directory}`" in readme_text
    assert "assets, scenes, scripts" not in repo_layout
    assert "worlds, assets, robots, scripts, validation/evaluation" in repo_layout
    assert "sim/README.md" in repo_layout
    assert "bridge/sensors/datasets" in repo_layout
    assert "artifacts/server_sim_closure/" in repo_layout
    assert "launch/gazebo_simulation.launch.py" in repo_layout
    assert "legacy Go1 demos" in scripts_index
    assert "sim/robots/go1_playground/" in scripts_index
    assert "Add boundary README later" not in plan_text
    assert "Boundary README added" in plan_text
    assert "default product runtime" in readme_text
    assert "enable_native=False" in readme_text
    assert "LocalPlannerModule" in readme_text
    assert "PathFollowerModule" in readme_text
    assert "CmdVelMux" in readme_text
    assert "native gate/legacy experiment" in readme_text
    assert "`maps/` | Reserved empty placeholder" in readme_text
    assert "`configs/` | Reserved empty placeholder" in readme_text
    assert "artifacts/server_sim_closure_summary_g4_current.json" in readme_text
    assert "aggregator does not launch missing gates" in readme_text
    assert "full_sim_validation.py" in scripts_index
    assert "cmu_unity_lingtu_stack.py" in scripts_index
    assert "run_global_planner.py" in scripts_index
    assert "isolated nonzero `ROS_DOMAIN_ID`" in scripts_index
    assert "not be used as the current G4 planner evidence source" in scripts_index
    assert "_run_legkilo_test.sh" in scripts_index
    assert "legacy/manual dataset helper" in scripts_index
    assert "Safety class" in scripts_index
    assert "summary-only unless --run-missing" in scripts_index
    assert "local non-motion" in scripts_index
    assert "simulated motion only" in scripts_index
    assert "ROS2 isolated simulation" in scripts_index
    assert "legacy manual" in scripts_index
    assert "canonical simulation runtime core" in engine_index
    assert "lingtu.py sim" in engine_index
    assert "real_robot_motion=false" in engine_index
    assert "generated evidence root" in closure_index
    assert "24h freshness" in closure_index
    assert "artifacts/server_sim_closure_summary_g4_current.json" in closure_index
    assert "host_requirements" in closure_index
    assert "Linux/ROS 2/MuJoCo/PCT-native checks" in closure_index
    assert "cmd_vel_sent_to_hardware=false" in closure_index
    assert "expected_report_path" in closure_index
    assert "accepted_patterns" in closure_index
    assert "Legacy ROS launch / smoke contract" in readme_text
    assert "Simulation endpoint required" in readme_text
    assert "Do not treat bare `nav`, `map`, or `explore` as simulation" in readme_text
    assert "isolated `ROS_DOMAIN_ID`" in readme_text
    assert "no hardware subscriber" in readme_text
    assert "Legacy ROS launch / smoke contract" in launch_index
    assert "isolated ROS_DOMAIN_ID" in launch_index
    assert "Do not run on a robot ROS domain" in launch_index
    assert "must not have hardware cmd_vel subscribers" in launch_index


def test_mujoco_driver_prefers_brainstem_policy_and_resolves_repo_relative_paths(monkeypatch, tmp_path):
    import drivers.sim.mujoco_driver_module as driver_mod

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
    monkeypatch.setattr(driver_mod, "_POLICY_CANDIDATES", (brainstem_policy, legacy_policy, thunder_policy))

    driver = MujocoDriverModule(policy_path="")
    assert Path(driver._policy_path) == brainstem_policy

    explicit = MujocoDriverModule(policy_path="model/policy_251119.onnx")
    assert Path(explicit._policy_path) == brainstem_policy.resolve()

    missing_explicit = driver_mod._resolve_sim_path("sim/robots/nova_dog/missing.onnx")
    assert Path(missing_explicit) == (sim_root / "robots" / "nova_dog" / "missing.onnx").resolve()

    missing_sim_relative = driver_mod._resolve_sim_path("robots/nova_dog/missing.onnx")
    assert Path(missing_sim_relative) == (
        sim_root / "robots" / "nova_dog" / "missing.onnx"
    ).resolve()


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
        [2.0, -3.0, 4.0, -2.0, 3.0, -4.0, 1.8, -2.8,
         3.8, -1.8, 2.8, -3.8, 0.8, -0.9, 1.1, -1.2],
        dtype=np.float64,
    )

    clamped = PolicyRunner.clamp_action(action)

    assert np.allclose(clamped, action)
    assert clamped is not action


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
    assert policy_nav_smoke._passes_direct(
        stand,
        min_motion=0.20,
        direct_mode="stand",
        max_stand_drift=0.05,
    ) is True
    stand["moved_m"] = 0.08
    assert policy_nav_smoke._passes_direct(
        stand,
        min_motion=0.20,
        direct_mode="stand",
        max_stand_drift=0.05,
    ) is False

    turn = {**base, "moved_m": 0.04, "yaw_delta_abs_rad": 0.45}
    assert policy_nav_smoke._passes_direct(
        turn,
        min_motion=0.20,
        direct_mode="turn",
        min_turn_yaw=0.35,
        max_turn_drift=0.10,
    ) is True
    turn["yaw_delta_abs_rad"] = 0.20
    assert policy_nav_smoke._passes_direct(
        turn,
        min_motion=0.20,
        direct_mode="turn",
        min_turn_yaw=0.35,
        max_turn_drift=0.10,
    ) is False
    turn["yaw_delta_abs_rad"] = 0.45
    turn["moved_m"] = 0.15
    assert policy_nav_smoke._passes_direct(
        turn,
        min_motion=0.20,
        direct_mode="turn",
        min_turn_yaw=0.35,
        max_turn_drift=0.10,
    ) is False


def test_policy_nav_smoke_accepts_explicit_policy_argument():
    from sim.scripts import policy_nav_smoke

    args = policy_nav_smoke._build_parser().parse_args(
        [
            "--policy",
            "/tmp/policy.onnx",
            "--nav-max-angular-z",
            "0.2",
            "--nav-local-planner-backend",
            "nanobind",
            "--nav-path-follower-backend",
            "nav_core",
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
    assert args.nav_local_planner_backend == "nanobind"
    assert args.nav_path_follower_backend == "nav_core"
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
    manifest = (
        Path(__file__).resolve().parents[2]
        / "sim"
        / "robots"
        / "nova_dog"
        / "policy_manifest.json"
    )
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
    system = full_stack_blueprint(
        robot="sim_mujoco",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
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

    follower = system.get_module("PathFollowerModule")
    nav = system.get_module("NavigationModule")

    assert nav._tracker._final_threshold == pytest.approx(0.09)
    assert follower._goal_tolerance == pytest.approx(0.07)
    assert follower._min_speed == pytest.approx(0.04)
    assert follower._max_speed == pytest.approx(0.22)
    assert follower._max_yaw_rate == pytest.approx(0.18)
    assert follower._turn_speed_yaw_rate_start == pytest.approx(0.12)
    assert follower._turn_speed_min_scale == pytest.approx(0.45)


def test_navigation_stack_passes_local_planner_trackable_path_threshold():
    system = full_stack_blueprint(
        robot="sim_mujoco",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        local_planner_allow_direct_track_fallback=True,
        local_planner_direct_track_fallback_min_distance_m=0.75,
        local_planner_min_trackable_local_path_m=0.8,
        run_startup_checks=False,
    ).build()

    local_planner = system.get_module("LocalPlannerModule")

    assert local_planner._allow_direct_track_fallback is True
    assert local_planner._direct_track_fallback_min_distance_m == pytest.approx(0.75)
    assert local_planner._min_trackable_local_path_xy == pytest.approx(0.8)


def test_mujoco_camera_preserves_metric_depth_output():
    from sim.engine.mujoco.camera import MuJoCoCamera

    raw = np.array([[0.4, 2.5, 25.0]], dtype=np.float32)
    depth = MuJoCoCamera._coerce_depth_meters(raw, near=0.1, far=10.0)

    assert np.allclose(depth, np.array([[0.4, 2.5, 10.0]], dtype=np.float32))



def test_mujoco_driver_default_robot_emits_lidar_points():
    pytest.importorskip("mujoco")

    driver = MujocoDriverModule(world="building_scene", render=False, enable_camera=False)
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
    import drivers.sim.mujoco_driver_module as driver_mod

    policy_path = driver_mod._first_existing_path(driver_mod._POLICY_CANDIDATES)
    if not policy_path:
        pytest.skip("policy_251119.onnx is not available in this checkout")
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
    pytest.importorskip("onnxruntime")
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

    system = full_stack_blueprint(
        robot="sim_mujoco",
        world="open_field",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
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
        run_startup_checks=False,
    ).build()
    driver = system.get_module("MujocoDriverModule")
    ogm = system.get_module("OccupancyGridModule")
    nav = system.get_module("NavigationModule")
    local_planner = system.get_module("LocalPlannerModule")
    path_follower = system.get_module("PathFollowerModule")
    mux = system.get_module("CmdVelMux")

    seen = {
        "costmap": 0,
        "waypoints": 0,
        "local_path": 0,
        "path_follower_cmd": 0,
        "mux_cmd": 0,
        "direct_fallback": 0,
    }
    odom = []
    ogm.costmap._add_callback(lambda _: seen.__setitem__("costmap", seen["costmap"] + 1))
    nav.waypoint._add_callback(lambda _: seen.__setitem__("waypoints", seen["waypoints"] + 1))
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

        plan_deadline = time.time() + 10.0
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
                and seen["path_follower_cmd"] > 0
                and seen["mux_cmd"] > 0
                and moved > 0.30
                and dist_to_goal < 2.20
            ):
                break

        assert seen["waypoints"] > 0
        assert seen["local_path"] > 0
        assert seen["path_follower_cmd"] > 0
        assert seen["mux_cmd"] > 0
        assert seen["direct_fallback"] == 0
        assert moved > 0.30
        assert dist_to_goal < 2.20
        assert nav._state in ("EXECUTING", "SUCCESS")
    finally:
        system.stop()


def test_sim_mujoco_full_stack_policy_mode_moves_under_nav_cmds_when_real_policy_available():
    pytest.importorskip("mujoco")
    pytest.importorskip("onnxruntime")
    policy_path = _real_policy_path_or_skip()

    # MuJoCo + ONNX policy state is not repeat-isolated inside one Windows
    # Python process. Production validation launches this gate in a fresh
    # process, so keep the test boundary identical to the runtime boundary.
    script = r'''
import json
import math
import time

from core.blueprints.full_stack import full_stack_blueprint
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3

system = full_stack_blueprint(
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
nav = system.get_module("NavigationModule")
local_planner = system.get_module("LocalPlannerModule")
path_follower = system.get_module("PathFollowerModule")
mux = system.get_module("CmdVelMux")

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
'''.replace("__POLICY_PATH__", str(policy_path).replace("\\", "\\\\"))

    env = os.environ.copy()
    repo_root = Path(__file__).resolve().parents[2]
    env["PYTHONPATH"] = os.pathsep.join(
        [str(repo_root / "src"), str(repo_root), env.get("PYTHONPATH", "")]
    )
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
    payloads = [line[len(marker):] for line in result.stdout.splitlines() if line.startswith(marker)]
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

    system = full_stack_blueprint(
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

    nav = system.get_module("NavigationModule")
    assert system.get_module("CmdVelMux") is not None
    recovery_edge = (
        "NavigationModule",
        "recovery_cmd_vel",
        "CmdVelMux",
        "recovery_cmd_vel",
    )
    if hasattr(nav, "recovery_cmd_vel"):
        assert recovery_edge in system.connections
    else:
        assert recovery_edge not in system.connections
    assert (
        "PathFollowerModule",
        "cmd_vel",
        "CmdVelMux",
        "path_follower_cmd_vel",
    ) in system.connections
    assert (
        "CmdVelMux",
        "driver_cmd_vel",
        "MujocoDriverModule",
        "cmd_vel",
    ) in system.connections
    assert (
        "CmdVelMux",
        "driver_cmd_vel",
        "SafetyRingModule",
        "cmd_vel",
    ) in system.connections
    assert (
        "PathFollowerModule",
        "cmd_vel",
        "MujocoDriverModule",
        "cmd_vel",
    ) not in system.connections


def test_full_stack_mux_wiring_tolerates_legacy_nav_without_recovery_cmd():
    from core.blueprint import Blueprint
    from core.blueprints.full_stack_wiring import apply_full_stack_wires
    from core.module import Module
    from core.stream import In, Out
    from nav.cmd_vel_mux_module import CmdVelMux

    class LegacyNavigationModule(Module, layer=5):
        stop_signal: In[int]

    class TestDriverModule(Module, layer=1):
        cmd_vel: In[Twist]
        stop_signal: In[int]

    class TestPathFollowerModule(Module, layer=5):
        cmd_vel: Out[Twist]

    class TestSafetyRingModule(Module, layer=0):
        stop_cmd: Out[int]
        cmd_vel: In[Twist]

    bp = Blueprint()
    bp.add(LegacyNavigationModule, alias="NavigationModule")
    bp.add(TestDriverModule, alias="MujocoDriverModule")
    bp.add(TestPathFollowerModule, alias="PathFollowerModule")
    bp.add(TestSafetyRingModule, alias="SafetyRingModule")
    bp.add(CmdVelMux)

    system = apply_full_stack_wires(
        bp,
        robot="sim_mujoco",
        driver_module="MujocoDriverModule",
        slam_profile="none",
        enable_semantic=False,
    ).build()

    assert (
        "NavigationModule",
        "recovery_cmd_vel",
        "CmdVelMux",
        "recovery_cmd_vel",
    ) not in system.connections
    assert (
        "PathFollowerModule",
        "cmd_vel",
        "CmdVelMux",
        "path_follower_cmd_vel",
    ) in system.connections
    assert (
        "CmdVelMux",
        "driver_cmd_vel",
        "MujocoDriverModule",
        "cmd_vel",
    ) in system.connections
    assert (
        "PathFollowerModule",
        "cmd_vel",
        "MujocoDriverModule",
        "cmd_vel",
    ) not in system.connections


def test_full_stack_required_safety_stop_wire_reports_missing_contract():
    from core.blueprint import Blueprint
    from core.blueprints.full_stack_wiring import apply_full_stack_wires
    from core.module import Module
    from core.stream import In, Out

    class LegacyNavigationModule(Module, layer=5):
        pass

    class TestDriverModule(Module, layer=1):
        stop_signal: In[int]

    class TestSafetyRingModule(Module, layer=0):
        stop_cmd: Out[int]

    bp = Blueprint()
    bp.add(LegacyNavigationModule, alias="NavigationModule")
    bp.add(TestDriverModule, alias="MujocoDriverModule")
    bp.add(TestSafetyRingModule, alias="SafetyRingModule")

    with pytest.raises(ValueError) as exc:
        apply_full_stack_wires(
            bp,
            robot="sim_mujoco",
            driver_module="MujocoDriverModule",
            slam_profile="none",
            enable_semantic=False,
        )

    assert "Required full-stack wire unavailable" in str(exc.value)
    assert "SafetyRingModule.stop_cmd->NavigationModule.stop_signal" in str(exc.value)
    assert "missing destination port NavigationModule.stop_signal" in str(exc.value)


def test_full_stack_wires_frontier_exploration_goal_to_navigation():
    system = full_stack_blueprint(
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
        "NavigationModule",
        "goal_pose",
    ) in system.connections
    assert (
        "NavigationModule",
        "mission_status",
        "WavefrontFrontierExplorer",
        "navigation_status",
    ) in system.connections


def test_frontier_exploration_goal_reaches_navigation_planner():
    system = full_stack_blueprint(
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
    nav = system.get_module("NavigationModule")

    seen = {"exploration_goals": 0, "waypoints": 0, "paths": 0}
    explorer.exploration_goal._add_callback(
        lambda _: seen.__setitem__("exploration_goals", seen["exploration_goals"] + 1)
    )
    nav.waypoint._add_callback(lambda _: seen.__setitem__("waypoints", seen["waypoints"] + 1))
    nav.global_path._add_callback(lambda _: seen.__setitem__("paths", seen["paths"] + 1))

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
        while (
            time.time() < deadline
            and (seen["exploration_goals"] == 0 or seen["waypoints"] == 0)
        ):
            time.sleep(0.05)

        assert seen["exploration_goals"] > 0
        assert seen["paths"] > 0
        assert seen["waypoints"] > 0
        assert nav._state in ("EXECUTING", "SUCCESS")
    finally:
        explorer.end_exploration()
        system.stop()


def test_sim_scene_observer_emits_building_scene_stairs():
    from semantic.perception.sim_scene_observer import SimSceneObserver

    class _Intrinsics:
        fx = 415.7
        fy = 415.7
        cx = 320.0
        cy = 240.0
        width = 640
        height = 480

    tf = np.eye(4, dtype=np.float32)
    tf[:3, :3] = np.array([
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)
    tf[:3, 3] = [2.0, 3.0, 0.5]
    observer = SimSceneObserver(world="building_scene")

    detections = observer.observe(tf, _Intrinsics(), text_prompt="stairs . goal")

    assert any(det.label == "stairs" for det in detections)


def test_sim_scene_observer_respects_live_forward_axis_convention():
    from semantic.perception.sim_scene_observer import SimSceneObserver

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
