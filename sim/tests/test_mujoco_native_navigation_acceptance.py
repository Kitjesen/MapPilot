from __future__ import annotations

import json
import math
import subprocess
import sys
import threading
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from lingtu.assembly.native_nav import mapd_environment
from sim.engine.core.engine import VelocityCommand
from sim.engine.mujoco import robot_controller
from sim.engine.mujoco.engine import MuJoCoEngine
from sim.scripts.mujoco import native_dds_sensors as sensors
from sim.scripts.mujoco import native_navigation_acceptance as acceptance
from sim.scripts.mujoco import native_navigation_video as navigation_video
from sim.scripts.mujoco import saved_map_plan_gate

ROOT = Path(__file__).resolve().parents[2]


def test_navigation_acceptance_uses_current_thunder_mjcf() -> None:
    assert acceptance.DEFAULT_THUNDERV4_MJCF == (
        ROOT / "sim" / "robots" / "doso" / "thunder_v4" / "mjcf" / "thunderv4.xml"
    )
    assert acceptance.DEFAULT_THUNDERV4_MJCF.is_file()


def test_native_map_identity_comes_from_selected_map_metadata(monkeypatch, tmp_path) -> None:
    map_dir = tmp_path / "building_map"
    map_dir.mkdir()
    session_root = tmp_path / "session"
    session_root.mkdir()
    metadata = map_dir / "metadata.json"
    metadata.write_text(
        json.dumps(
            {
                "map_name": "building",
                "frame_id": "map",
                "created_at": "12345",
            }
        ),
        encoding="utf-8",
    )
    for name in (
        "LINGTU_MAP_ID",
        "LINGTU_MAP_CONTENT_EPOCH",
        "LINGTU_MAP_FRAME",
        "LINGTU_PRODUCT_SESSION_ID",
    ):
        monkeypatch.delenv(name, raising=False)

    identity = acceptance._native_map_identity(
        paths={"map_dir": map_dir, "metadata": metadata},
        phase="motion",
        domain_id=228,
        session_root=session_root,
    )

    assert identity["LINGTU_MAP_ID"] == "building"
    assert identity["LINGTU_MAP_CONTENT_EPOCH"] == "12345"
    assert identity["LINGTU_MAP_FRAME"] == "map"
    assert identity["LINGTU_SESSION_ROOT"] == str(session_root.resolve())
    assert "LINGTU_EXPLORE_ROUTE" not in identity


def test_saved_map_build_does_not_activate(monkeypatch, tmp_path):
    commands = []
    monkeypatch.setattr(saved_map_plan_gate, "_mapctl_executable", lambda: Path("lingtu-mapctl"))
    monkeypatch.setattr(
        saved_map_plan_gate,
        "_run_json_process",
        lambda command, **_kwargs: commands.append(command) or {"success": True},
    )
    args = SimpleNamespace(
        converter="",
        converter_timeout=5.0,
        free_dilation_cells=1,
        free_layers_above=1,
        resolution=0.1,
        support_dilation_cells=1,
    )

    result = saved_map_plan_gate._build_octomap_native(
        map_root=tmp_path,
        map_id="yard",
        args=args,
        scene_preset="default",
        lidar_map_source=False,
    )

    assert result["ok"] is True
    assert "--activate" not in commands[0]


def test_asset_builder_locks_support_dilation(monkeypatch, tmp_path):
    monkeypatch.setattr(
        saved_map_plan_gate,
        "run_gate",
        lambda args: {"support_dilation_cells": args.support_dilation_cells},
    )

    result = acceptance._run_saved_map_asset_builder(
        {
            "scene_preset": "multifloor_stack_3",
            "map_source": "mujoco_lidar",
            "support_dilation_cells": 2,
        },
        tmp_path,
    )

    assert result["support_dilation_cells"] == 2


def test_multifloor_global_guide_uses_stair_compatible_support_model():
    manifest = acceptance._load_manifest(
        ROOT / "config/runtime_graph/acceptance/mujoco_multifloor_navigation_acceptance.json"
    )
    constraints = manifest["planner_constraints"]

    assert manifest["start"] == [7.0, -0.8, 0.55, 3.141592653589793]
    assert constraints["robot_radius_m"] == 0.35
    assert constraints["strict_ground_support"] is False
    assert constraints["obstacle_clearance_radius_cells"] == 4


def test_multifloor_ground_is_visual_only():
    root = ET.fromstring(saved_map_plan_gate._stairs3d_scene_xml("multifloor_stack_3"))
    ground = root.find(".//geom[@name='ground_reference']")

    assert ground is not None
    assert ground.get("contype") == "0"
    assert ground.get("conaffinity") == "0"
    assert ground.get("group") == "5"


def test_multifloor_perimeter_rails_are_not_step_over_support():
    root = ET.fromstring(saved_map_plan_gate._stairs3d_scene_xml("multifloor_stack_3"))
    rail = root.find(".//geom[@name='rail_level_1_south']")

    assert rail is not None
    half_height = float(rail.get("size", "0 0 0").split()[2])
    assert half_height >= 0.45


def test_saved_map_lidar_collector_uses_current_engine_interface(monkeypatch, tmp_path):
    class Engine:
        sim_time = 0.0

        @staticmethod
        def get_lidar_backend_report():
            return {"active": "mujoco_lidar"}

        @staticmethod
        def set_robot_pose(_position, _orientation):
            return None

        @staticmethod
        def get_robot_state():
            return SimpleNamespace(position=[0.0, 0.0, 0.5])

        @staticmethod
        def get_lidar_points():
            return [[1.0, 0.0, 0.5, 1.0]]

        @staticmethod
        def step(_command):
            return SimpleNamespace(position=[0.0, 0.0, 0.5])

        @staticmethod
        def close():
            return None

    def build_engine(
        *,
        world,
        drive_mode,
        start,
        mujoco_memory,
        mid360_pattern,
        mid360_samples_per_frame,
        lidar_backend,
        mujoco_lidar_backend,
        require_product_lidar_backend,
    ):
        return Engine()

    monkeypatch.setattr("drivers.sim.mujoco.runtime.build_engine", build_engine)
    points, report = saved_map_plan_gate.collect_mujoco_lidar_points(
        tmp_path / "scene.xml",
        start=[0.0, 0.0, 0.5, 0.0],
        scans=1,
        duration_s=0.1,
        timeout_s=1.0,
        vx=0.0,
        wz=0.0,
        publish_hz=10.0,
        mid360_pattern=tmp_path / "pattern.csv",
        mid360_samples_per_frame=100,
        lidar_backend="mujoco_lidar",
        mujoco_lidar_backend="cpu",
        allow_legacy_lidar_fallback=False,
        mujoco_memory="64M",
        mapping_trajectory=[[0.0, 0.0, 0.5, 0.0]],
    )

    assert report["ok"] is True
    assert points == [(1.0, 0.0, 0.5)]


class _FakeOnnxSession:
    def __init__(self, input_dim: int) -> None:
        self._input = SimpleNamespace(name="obs", shape=[1, input_dim])
        self._output = SimpleNamespace(name="actions", shape=[1, 16])

    def get_inputs(self):
        return [self._input]

    def get_outputs(self):
        return [self._output]


def test_motion_arm_is_acknowledged_before_navigation_commands(monkeypatch, tmp_path):
    arm_file = tmp_path / "motion_arm.json"
    status_file = tmp_path / "motion_arm_status.json"

    class Sensor:
        @staticmethod
        def poll():
            return None

    def acknowledge(_delay_s: float) -> None:
        acceptance._write_json(status_file, {"state": "armed"})

    monkeypatch.setattr(acceptance.time, "sleep", acknowledge)
    result = acceptance._arm_mujoco_motion(
        arm_file=arm_file,
        status_file=status_file,
        token="run-token",
        domain_id=227,
        scenario="scan:motion:227",
        sensor=Sensor(),
    )

    payload = json.loads(arm_file.read_text(encoding="utf-8"))
    assert result["state"] == "armed"
    assert payload == {
        "schema": "lingtu.mujoco.external_arm.v1",
        "arm": True,
        "token": "run-token",
        "domain_id": 227,
        "scenario": "scan:motion:227",
    }


def test_onnx_policy_runner_is_selected_from_input_contract_not_directory(monkeypatch, tmp_path):
    created_options = []

    class SessionOptions:
        def __init__(self):
            self.intra_op_num_threads = 0
            self.inter_op_num_threads = 0
            self.execution_mode = None

    def make_session(*_args, **kwargs):
        created_options.append(kwargs["sess_options"])
        return next(sessions)

    sessions = iter([_FakeOnnxSession(285), _FakeOnnxSession(57)])
    monkeypatch.setitem(
        sys.modules,
        "onnxruntime",
        SimpleNamespace(
            ExecutionMode=SimpleNamespace(ORT_SEQUENTIAL="sequential"),
            InferenceSession=make_session,
            SessionOptions=SessionOptions,
        ),
    )
    policy_dir = tmp_path / "thunderv4" / "policy"
    policy_dir.mkdir(parents=True)

    history_runner = robot_controller.load_policy_runner(
        str(policy_dir / "history.onnx"), cpu_threads=2
    )
    frame_runner = robot_controller.load_policy_runner(
        str(policy_dir / "frame.onnx"), cpu_threads=2
    )

    assert isinstance(history_runner, robot_controller.PolicyRunner)
    assert history_runner._history_len == 5
    assert history_runner.run_at_idle is False
    assert history_runner.zero_wheels_at_idle is True
    assert isinstance(frame_runner, robot_controller.ThunderV4OnnxPolicyRunner)
    assert len(created_options) == 2
    assert all(options.intra_op_num_threads == 2 for options in created_options)
    assert all(options.inter_op_num_threads == 1 for options in created_options)
    assert all(options.execution_mode == "sequential" for options in created_options)


def test_cpp_driver_bridge_consumes_canonical_typed_dds_topic():
    source = (ROOT / "sim" / "adapters" / "dds" / "mujoco_driver_bridge.cpp").read_text(
        encoding="utf-8"
    )
    protocol = (
        ROOT / "sim" / "adapters" / "dds" / "mujoco_driver_bridge_protocol.cpp"
    ).read_text(encoding="utf-8")
    cmake = (ROOT / "sim" / "adapters" / "dds" / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "lingtu::message::kNavCmdVel" in source
    assert "lingtu_dds_FinalVelocityCommand_desc" in source
    assert "lingtu_dds_TwistStamped_desc" not in source
    assert "static_cast<const lingtu_dds_FinalVelocityCommand *>" in source
    assert "qos_for_topic(command_contract.dds_topic)" in source
    assert "LT_DRIVER_COMMAND_V2" in protocol
    assert "src/message/idl/messages.idl" in cmake
    assert "add_executable(lingtu_mujoco_driver_bridge" in cmake


def test_cpp_driver_bridge_timestamps_command_after_dds_take():
    source = (ROOT / "sim" / "adapters" / "dds" / "mujoco_driver_bridge.cpp").read_text(
        encoding="utf-8"
    )

    take_latest = source.split("std::optional<NavCommand> takeLatest()", 1)[1].split(
        "void publishControlState", 1
    )[0]
    assert take_latest.index("dds_take(command_reader_") < take_latest.index(
        "const auto arrival_time = bridgeNow()"
    )
    assert "takeLatest(command_arrival)" not in source


def test_nav_runtime_preserves_goal_during_transient_driver_blocker():
    source = (ROOT / "src" / "nav" / "cpp" / "endpoint" / "runtime" / "loop.cpp").read_text(
        encoding="utf-8"
    )

    transition = source.split("const std::string driver_blocker", 1)[1].split(
        "if (staged_inspection_goal", 1
    )[0]
    assert "clearEndpointMotion" in transition
    assert "keepZeroFresh" in transition
    assert "navigation_runtime_controller.interrupt" not in transition


def test_cpp_driver_bridge_control_state_is_derived_from_physical_bridge_status():
    source = (ROOT / "sim" / "adapters" / "dds" / "mujoco_driver_bridge.cpp").read_text(
        encoding="utf-8"
    )

    assert "lingtu::message::kDriverControlState" in source
    assert "lingtu_dds_DriverControlState_desc" in source
    assert "const auto output_ack = status.output_ack" in source
    assert "message.ready = status.ready" in source
    assert "message.lease_valid = status.lease_valid" in source
    assert "message.accepted_sequence = status.accepted_sequence" in source
    assert "output_ack.producerBootId()" in source
    assert "output_ack.outputSequence()" in source
    assert "output_ack.accepted()" in source
    assert "checked(dds_write(state_writer_, &message)" in source
    assert "message.ready = true" not in source
    assert "message.last_command_accepted = true" not in source


def test_native_navigation_phase_timeout_includes_wsl_shutdown_grace():
    assert acceptance._phase_runtime_timeout_s(20.0, 120.0) == pytest.approx(140.0)
    assert acceptance._phase_runtime_timeout_s(20.0, 10.0) == pytest.approx(80.0)
    assert acceptance._phase_runtime_timeout_s(
        20.0,
        10.0,
        realtime_factor=0.5,
    ) == pytest.approx(100.0)


def test_native_navigation_resume_waits_for_control_loop_recovery(monkeypatch):
    replies = iter(
        [
            {
                "returncode": 1,
                "stdout": "",
                "stderr": "control_loop_recovery_pending",
            },
            {"returncode": 0, "stdout": "accepted", "stderr": ""},
        ]
    )
    monkeypatch.setattr(acceptance, "_resume_command", lambda *_args, **_kwargs: next(replies))
    monkeypatch.setattr(acceptance.time, "sleep", lambda _seconds: None)

    result = acceptance._resume_when_ready(Path("nav_control"), 220, 3.0)

    assert result["returncode"] == 0
    assert len(result["attempts"]) == 2


@pytest.mark.parametrize(("ok", "expected_exit"), ((True, 0), (False, 1)))
def test_native_navigation_runner_accepts_dispatcher_strict_argument(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    ok: bool,
    expected_exit: int,
) -> None:
    def run(args) -> dict[str, object]:
        assert args.strict is True
        assert args.preflight_only is True
        return {
            "ok": ok,
            "evidence_scope": "preflight",
            "product_acceptance_passed": False,
        }

    monkeypatch.setattr(acceptance, "run", run)

    assert acceptance.main(["--strict", "--preflight-only"]) == expected_exit
    report = json.loads(capsys.readouterr().out)
    assert report["product_acceptance_passed"] is False


def test_local_planner_manifests_share_one_truth_lidar_baseline() -> None:
    scan = acceptance._load_manifest(
        ROOT
        / "config"
        / "runtime_graph"
        / "acceptance"
        / "mujoco_local_scan.json"
    )
    cmu = acceptance._load_manifest(
        ROOT
        / "config"
        / "runtime_graph"
        / "acceptance"
        / "mujoco_local_cmu.json"
    )

    assert scan["product_contract"]["product"] == "nav"
    assert scan["navigation_runtime"]["local_planner"] == "scan"
    assert cmu["navigation_runtime"]["local_planner"] == "cmu"
    assert scan["windows_cpu_isolation"] is True
    assert scan["windows_navigation_physical_cores"] == 3
    for field in ("world", "start", "goal", "map_dir", "asset_builder", "slam_runtime"):
        assert scan[field] == cmu[field]
    assert scan["start"] == [3.0, 4.0, 0.6, 0.0]
    assert scan["navigation_runtime"]["path_follower_max_speed_mps"] >= 0.5
    assert scan["slam_runtime"]["provider"] == "mujoco_navigation_fixture"
    assert scan["paths"]["slam_config"] == cmu["paths"]["slam_config"]
    assert scan["paths"]["slam_config"] == (
        "src/localization/fastlio2/config/sim_mid360.yaml"
    )
    for field in (
        "publish_hz",
        "imu_hz",
        "publish_odom_prior",
        "scan_time_profile",
        "mid360_samples_per_frame",
        "physics_timestep_s",
        "physics_integrator",
        "publisher_write_mode",
    ):
        assert scan["sensor_runtime"][field] == cmu["sensor_runtime"][field]
    assert scan["sensor_runtime"]["publish_odom_prior"] is True
    assert scan["sensor_runtime"]["scan_time_profile"] == "instantaneous"
    assert scan["sensor_runtime"]["mid360_samples_per_frame"] == 4000
    assert scan["sensor_runtime"]["physics_timestep_s"] == 0.005
    assert scan["sensor_runtime"]["physics_integrator"] == "euler"
    assert scan["sensor_runtime"]["publisher_write_mode"] == "async_fifo"
    assert scan["thresholds"]["min_mujoco_truth_peak_xy_speed_mps"] >= 0.5
    assert scan["thresholds"]["require_traversability"] is False
    assert scan["navigation_runtime"]["use_traversability_cost"] is False
    assert scan["asset_builder"] == {
        "kind": "saved_map_plan_gate",
        "scene_preset": "industrial_park",
        "length": 60.0,
        "width": 40.0,
        "spacing": 0.1,
        "hits_per_cell": 2,
        "resolution": 0.2,
        "map_source": "synthetic_hits",
    }
    assert scan["paths"]["path_library"] is None
    assert cmu["paths"]["path_library"] == (
        "src/nav/cpp/planning/local/cmu/paths/thunder"
    )
    assert "rt/maps/local_collision" in scan["contracts"]["navigation_inputs"]
    assert "rt/nav/traversability" not in scan["contracts"]["navigation_inputs"]
    assert scan["contracts"]["terrain_output"] is None
    assert acceptance._path_library_args("scan", Path("navd"), {}) == ()

    sensor_args = acceptance._sensor_runtime_args(scan)
    assert "--publish-odom-prior" in sensor_args
    assert sensor_args[sensor_args.index("--scan-time-profile") + 1] == "instantaneous"
    assert sensor_args[sensor_args.index("--mid360-samples-per-frame") + 1] == "4000"
    assert sensor_args[sensor_args.index("--publisher-write-mode") + 1] == "async_fifo"

    follower_args = acceptance._path_follower_args(scan["navigation_runtime"])
    assert "--path-follower-goal-tolerance-m" not in follower_args
    assert follower_args[follower_args.index("--path-follower-max-speed-mps") + 1] == "0.6"
    assert follower_args[follower_args.index("--path-follower-max-accel-mps2") + 1] == "1.0"
    assert follower_args[follower_args.index("--path-follower-max-yaw-rate-rad-s") + 1] == "0.8"
    assert follower_args[follower_args.index("--path-follower-heading-align-enter-rad") + 1] == "1.0"
    assert follower_args[follower_args.index("--path-follower-heading-align-exit-rad") + 1] == "0.5"
    assert acceptance._mujoco_truth_speed_blocker(
        {"mujoco_truth_velocity": {"max_xy_speed_mps": 0.49}},
        scan["thresholds"],
    ) == "mujoco_truth_peak_xy_speed_below_threshold"
    assert acceptance._mujoco_truth_speed_blocker(
        {"mujoco_truth_velocity": {"max_xy_speed_mps": 0.51}},
        scan["thresholds"],
    ) is None


def test_scan_acceptance_gives_navd_three_whole_cores_and_keeps_support_disjoint() -> None:
    topology = acceptance.WindowsCpuTopology(
        cores=(
            acceptance.WindowsPhysicalCore(0, 0, 0b0000000011, 2),
            acceptance.WindowsPhysicalCore(1, 0, 0b0000001100, 2),
            acceptance.WindowsPhysicalCore(2, 0, 0b0000110000, 1),
            acceptance.WindowsPhysicalCore(3, 0, 0b0011000000, 1),
            acceptance.WindowsPhysicalCore(4, 0, 0b1100000000, 0),
        )
    )
    plan = acceptance.WindowsCpuIsolationPlan(
        processor_group=0,
        mujoco_core_id=0,
        owner_core_id=1,
        unreal_core_ids=(2, 3, 4),
        mujoco_affinity_mask=0b0000000011,
        owner_thread_affinity_mask=0b0000001100,
        unreal_affinity_mask=0b1111110000,
    )

    nav_mask, support_mask = acceptance._navigation_affinity_masks(plan, topology, 3)

    assert nav_mask == 0b0011111100
    assert support_mask == 0b1100000011
    assert nav_mask & support_mask == 0


def test_manifest_loader_reports_invalid_manifest(tmp_path: Path) -> None:
    manifest = tmp_path / "invalid.json"
    manifest.write_text("{", encoding="utf-8")

    with pytest.raises(ValueError, match="invalid acceptance manifest"):
        acceptance._load_manifest(manifest)


def test_native_navigation_cli_cannot_override_manifest_local_planner() -> None:
    with pytest.raises(SystemExit) as failure:
        acceptance.main(["--local-planner", "scan", "--preflight-only"])

    assert failure.value.code == 2


def test_binary_candidate_resolver_rejects_noncanonical_windows_exe():
    candidates = [
        "build/nav-cpp/linux-aarch64/navd",
        "build/nav-cpp/windows-x64-nav-endpoint-root-d/Release/navd.exe",
    ]

    assert acceptance._ordered_binary_candidates(candidates, platform_name="nt") == [
        "build/nav-cpp/linux-aarch64/navd",
    ]


def test_binary_candidate_resolver_windows_ignores_existing_retired_root_d_exe(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    retired = (
        tmp_path
        / "build/nav-cpp/windows-x64-nav-endpoint-root-d/Release/navd.exe"
    )
    canonical = tmp_path / "build/nav-cpp/windows-x64-nav-endpoint/Release/navd.exe"
    retired.parent.mkdir(parents=True)
    canonical.parent.mkdir(parents=True)
    retired.write_bytes(b"retired")
    canonical.write_bytes(b"canonical")
    monkeypatch.setattr(acceptance, "ROOT", tmp_path)
    monkeypatch.setattr(acceptance.os, "name", "nt")

    resolved = acceptance._resolve_binary(
        {
            "candidates": [
                retired.relative_to(tmp_path).as_posix(),
                canonical.relative_to(tmp_path).as_posix(),
            ]
        }
    )

    assert resolved == canonical.resolve()


def test_run_plan_binding_replaces_component_candidates_and_env_override(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    artifact = tmp_path / "build/root-d/navd.exe"
    artifact.parent.mkdir(parents=True)
    artifact.write_bytes(b"run-plan-navd")
    relative = artifact.relative_to(tmp_path).as_posix()
    command = SimpleNamespace(
        artifact=SimpleNamespace(
            path=relative,
        )
    )
    plan = SimpleNamespace(
        processes=(SimpleNamespace(name="nav_runtime", command=command),)
    )
    manifest = {
        "binaries": {
            "navigation": {
                "env": "LINGTU_MUJOCO_NAV_BIN",
                "candidates": ["build/old/navd.exe"],
            }
        }
    }
    monkeypatch.setattr(acceptance, "ROOT", tmp_path)

    bindings = acceptance._bind_manifest_binaries_to_run_plan(manifest, plan)

    assert manifest["binaries"]["navigation"] == {"candidates": [relative]}
    assert bindings == {
        "navigation": {
            "path": str(artifact.resolve()),
            "process": "nav_runtime",
        }
    }


def test_scan_mapd_launch_uses_run_plan_collision_profile(tmp_path: Path) -> None:
    environment = mapd_environment("scan")

    command, process_environment = acceptance._native_mapd_launch(
        binary=Path("mapd.exe"),
        domain_id=227,
        status_file=tmp_path / "mapd.json",
        map_root=tmp_path,
        runtime={},
        environment=environment,
    )

    cap_index = command.index("--max-collision-snapshot-points")
    assert command[cap_index + 1] == "4000000"
    assert process_environment["LINGTU_MAPD_OCCUPANCY_RESOLUTION_M"] == "0.05"
    assert process_environment["LINGTU_MAPD_OCCUPANCY_SIZE_Z"] == "100"
    assert mapd_environment("cmu") == {}


def test_binary_candidate_resolver_skips_exe_on_non_windows():
    candidates = [
        "build/nav-cpp/windows-x64-nav-endpoint/Release/navd.exe",
        "build/nav-cpp/linux-aarch64/navd",
    ]

    assert acceptance._ordered_binary_candidates(candidates, platform_name="posix") == [
        "build/nav-cpp/linux-aarch64/navd",
    ]


def test_driver_bridge_must_share_navigation_native_boot_clock():
    wsl_nav = Path("build/native/navd")
    wsl_bridge = Path("build/native/lingtu_mujoco_driver_bridge")
    windows_nav = Path("build/native/navd.exe")
    windows_bridge = Path("build/native/lingtu_mujoco_driver_bridge.exe")

    assert (
        acceptance._validated_native_clock_platform(
            wsl_nav,
            wsl_bridge,
            platform_name="nt",
        )
        == "wsl"
    )
    assert (
        acceptance._validated_native_clock_platform(
            windows_nav,
            windows_bridge,
            platform_name="nt",
        )
        == "windows"
    )
    assert (
        acceptance._validated_native_clock_platform(
            wsl_nav,
            wsl_bridge,
            platform_name="posix",
        )
        == "posix"
    )
    with pytest.raises(ValueError, match="same native boot clock"):
        acceptance._validated_native_clock_platform(
            windows_nav,
            wsl_bridge,
            platform_name="nt",
        )


def test_native_driver_runtime_uses_one_host_identity_and_exact_bridge_contract(tmp_path):
    host_boot_id = "a" * 32
    navigation_binary = Path("build/native/navd")
    driver_bridge_binary = Path("build/native/lingtu_mujoco_driver_bridge")
    driver_bridge_pid = tmp_path / "driver_bridge.pid"
    manifest = {
        "driver_runtime": {
            "max_linear_mps": 1.0,
            "max_angular_rps": 1.0,
            "command_timeout_ms": 200,
            "heartbeat_timeout_ms": 500,
            "apply_timeout_ms": 500,
        }
    }

    launch = acceptance._native_driver_runtime_launch(
        manifest=manifest,
        navigation_binary=navigation_binary,
        driver_bridge_binary=driver_bridge_binary,
        navigation_command=[str(navigation_binary), "--domain-id", "220"],
        driver_bridge_pid=driver_bridge_pid,
        host_boot_id=host_boot_id,
        platform_name="posix",
    )

    assert launch.host_boot_id == host_boot_id
    assert launch.clock_platform == "posix"
    assert (
        launch.navigation_env.get("LINGTU_HOST_BOOT_ID") == host_boot_id
        or f"LINGTU_HOST_BOOT_ID={host_boot_id}" in launch.navigation_command
    )
    assert launch.driver_bridge_args == (
        "--driver-bridge-bin",
        str(driver_bridge_binary),
        "--driver-bridge-pid-file",
        str(driver_bridge_pid),
        "--driver-expected-host-boot-id",
        host_boot_id,
        "--driver-max-linear-mps",
        "1",
        "--driver-max-angular-rps",
        "1",
        "--driver-command-timeout-ms",
        "200",
        "--driver-heartbeat-timeout-ms",
        "500",
        "--driver-apply-timeout-ms",
        "500",
    )


def test_terminal_driver_stop_evidence_requires_exact_zero_identity_and_cleanup():
    sensor_report = {
        "cmd_vel": {
            "bridge_boot_id": "bridge-boot",
            "controller_boot_id": "controller-boot",
            "driver_ready": False,
            "accepted_sequence": 0,
            "accepted_producer_boot_id": "",
            "accepted_output_sequence": 0,
            "observed_output_ack": {
                "accepted_sequence": 41,
                "producer_boot_id": "host-boot:123:456",
                "output_sequence": 17,
            },
            "stopped_evidence": {
                "bridge_boot_id": "bridge-boot",
                "controller_boot_id": "controller-boot",
                "bridge_command_seq": 42,
                "applied_step_seq": 99,
                "kind": "deactivate_zero",
            },
            "last_command": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            "process_returncode": 0,
            "process_cleanup": {"clean": True},
        },
        "native_sensor_publisher_process": {"clean": True},
    }
    process_cleanup = [
        {"name": "slam", "clean": True},
        {"name": "navigation", "clean": True},
        {"name": "sensor", "clean": True},
        {"name": "driver_bridge", "clean": True},
    ]

    report = acceptance._terminal_driver_stop_evidence(
        sensor_report=sensor_report,
        expected_host_boot_id="host-boot",
        process_cleanup=process_cleanup,
    )

    assert report["ok"] is True
    assert report["stop_ack"] == {
        "accepted": True,
        "bridge_boot_id": "bridge-boot",
        "controller_boot_id": "controller-boot",
        "bridge_command_seq": 42,
        "applied_step_seq": 99,
        "kind": "deactivate_zero",
    }
    assert report["logical_final_cmd_vel"] == {
        "exact_zero": True,
        "vx": 0.0,
        "vy": 0.0,
        "wz": 0.0,
    }
    assert report["physical_stop"]["exact_zero"] is True
    assert report["authority_cleared"] is True
    assert report["owned_processes_stopped"] is True
    assert report["blockers"] == []


def test_terminal_driver_stop_does_not_invent_prior_output_in_no_motion_phase():
    sensor_report = {
        "cmd_vel": {
            "bridge_boot_id": "bridge-boot",
            "controller_boot_id": "controller-boot",
            "driver_ready": False,
            "accepted_sequence": 0,
            "accepted_producer_boot_id": "",
            "accepted_output_sequence": 0,
            "observed_output_ack": {},
            "stopped_evidence": {
                "bridge_boot_id": "bridge-boot",
                "controller_boot_id": "controller-boot",
                "bridge_command_seq": 1,
                "applied_step_seq": 2,
                "kind": "deactivate_zero",
            },
            "last_command": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            "process_returncode": 0,
            "process_cleanup": {"clean": True},
        },
        "native_sensor_publisher_process": {"clean": True},
    }

    report = acceptance._terminal_driver_stop_evidence(
        sensor_report=sensor_report,
        expected_host_boot_id="host-boot",
        process_cleanup=[{"name": "sensor", "clean": True}],
        require_prior_output_ack=False,
    )

    assert report["ok"] is True
    assert report["prior_output_ack"]["required"] is False
    assert report["prior_output_ack"]["exact_identity_and_sequence"] is False


@pytest.mark.parametrize(
    ("mutate", "blocker"),
    [
        (
            lambda report: report["cmd_vel"]["stopped_evidence"].update(
                bridge_boot_id="wrong"
            ),
            "driver_terminal_stop_ack_identity_mismatch",
        ),
        (
            lambda report: report["cmd_vel"]["last_command"].update(vx=1e-12),
            "driver_terminal_logical_cmd_vel_not_exact_zero",
        ),
        (
            lambda report: report["cmd_vel"].update(accepted_sequence=1),
            "driver_terminal_authority_not_cleared",
        ),
        (
            lambda report: report["cmd_vel"]["process_cleanup"].update(clean=False),
            "driver_terminal_owned_process_cleanup_failed",
        ),
    ],
)
def test_terminal_driver_stop_evidence_fails_closed(mutate, blocker):
    sensor_report = {
        "cmd_vel": {
            "bridge_boot_id": "bridge-boot",
            "controller_boot_id": "controller-boot",
            "driver_ready": False,
            "accepted_sequence": 0,
            "accepted_producer_boot_id": "",
            "accepted_output_sequence": 0,
            "observed_output_ack": {
                "accepted_sequence": 10,
                "producer_boot_id": "host-boot:123:456",
                "output_sequence": 9,
            },
            "stopped_evidence": {
                "bridge_boot_id": "bridge-boot",
                "controller_boot_id": "controller-boot",
                "bridge_command_seq": 11,
                "applied_step_seq": 12,
                "kind": "deactivate_zero",
            },
            "last_command": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            "process_returncode": 0,
            "process_cleanup": {"clean": True},
        },
        "native_sensor_publisher_process": {"clean": True},
    }
    mutate(sensor_report)

    report = acceptance._terminal_driver_stop_evidence(
        sensor_report=sensor_report,
        expected_host_boot_id="host-boot",
        process_cleanup=[{"name": "sensor", "clean": True}],
    )

    assert report["ok"] is False
    assert blocker in report["blockers"]


def test_native_driver_identity_matches_nav_and_bridge_token_intersection(tmp_path):
    manifest = {
        "driver_runtime": {
            "max_linear_mps": 1.0,
            "max_angular_rps": 1.0,
            "command_timeout_ms": 200,
            "heartbeat_timeout_ms": 500,
            "apply_timeout_ms": 500,
        }
    }
    common = {
        "manifest": manifest,
        "navigation_binary": Path("build/native/navd"),
        "driver_bridge_binary": Path("build/native/lingtu_mujoco_driver_bridge"),
        "navigation_command": ["navd"],
        "driver_bridge_pid": tmp_path / "driver_bridge.pid",
        "platform_name": "posix",
    }

    launch = acceptance._native_driver_runtime_launch(
        **common,
        host_boot_id="host.acceptance:phase-1",
    )
    assert launch.host_boot_id == "host.acceptance:phase-1"

    for invalid in ("host@phase", "-leading-dash", "x" * 129, "non ascii 中文"):
        with pytest.raises(ValueError, match="safe ASCII token"):
            acceptance._native_driver_runtime_launch(
                **common,
                host_boot_id=invalid,
            )


def test_acceptance_run_reuses_one_host_identity_across_phases(monkeypatch, tmp_path):
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text('{"domain_id_base": 220}\n', encoding="utf-8")
    monkeypatch.setattr(acceptance, "_probe_wsl_runtime", lambda: (True, "ready"))
    monkeypatch.setattr(
        acceptance,
        "_artifact_storage_probe",
        lambda _path: (True, "native_linux"),
    )
    monkeypatch.setattr(
        acceptance,
        "_preflight",
        lambda _manifest: ({}, {}, [], {}),
    )
    monkeypatch.setattr(
        acceptance,
        "_mirror_native_runtime_inputs",
        lambda paths, _out_dir: (paths, {"enabled": False, "reason": "native_linux"}),
    )
    host_boot_ids: list[str] = []

    def run_phase(**kwargs):
        host_boot_ids.append(kwargs["host_boot_id"])
        return {"ok": True, "blockers": []}

    monkeypatch.setattr(acceptance, "_run_phase", run_phase)
    args = SimpleNamespace(
        manifest=str(manifest_path),
        world="",
        map_dir="",
        phase_duration_s=None,
        diagnostic_imu_acc_mode="",
        diagnostic_scan_time_profile="",
        diagnostic_imu_acc_lowpass_hz=None,
        diagnostic_imu_acc_max_dynamic_mps2=None,
        diagnostic_imu_acc_max_slew_mps3=None,
        record_video=False,
        video_width=1920,
        video_height=1080,
        video_fps=24.0,
        video_lidar_points=640,
        out_dir=str(tmp_path / "out"),
        prepare_assets=False,
        build_helper=False,
        preflight_only=False,
        mode="both",
        domain_id=220,
        allow_windows_9p_artifacts=False,
    )

    report = acceptance.run(args)

    assert report["ok"] is True
    assert len(host_boot_ids) == 2
    assert len(set(host_boot_ids)) == 1
    assert len(host_boot_ids[0]) == 32


def test_binary_candidate_resolver_env_override_is_authoritative(monkeypatch, tmp_path):
    override = tmp_path / "override-navd.exe"
    fallback = tmp_path / "navd"
    override.write_bytes(b"override")
    fallback.write_bytes(b"fallback")
    monkeypatch.setenv("LINGTU_TEST_NAVD", str(override))

    resolved = acceptance._resolve_binary(
        {"env": "LINGTU_TEST_NAVD", "candidates": [str(fallback)]}
    )

    assert resolved == override.resolve()


def test_native_runtime_requirement_follows_resolved_binary_platform(monkeypatch):
    monkeypatch.setattr(
        acceptance,
        "_resolve_binary",
        lambda spec: Path(str(spec["resolved"])),
    )
    windows_manifest = {
        "slam_runtime": {"provider": "mujoco_navigation_fixture"},
        "binaries": {
            "slam": {"resolved": "ignored-slam"},
            "navigation": {"resolved": "navd.exe"},
            "driver_bridge": {"resolved": "driver_bridge.exe"},
        },
    }
    mixed_manifest = {
        "binaries": {
            "navigation": {"resolved": "navd.exe"},
            "driver_bridge": {"resolved": "driver_bridge.exe"},
            "traversability": {"resolved": "lingtu_traversability_dds"},
        },
    }

    assert acceptance._requires_wsl_runtime(
        windows_manifest, platform_name="nt"
    ) is False
    assert acceptance._requires_wsl_runtime(
        mixed_manifest, platform_name="nt"
    ) is True
    assert acceptance._requires_wsl_runtime(
        mixed_manifest, platform_name="posix"
    ) is False


def test_native_path_argument_matches_worker_platform(monkeypatch, tmp_path):
    asset = tmp_path / "map" / "octomap.ot"
    monkeypatch.setattr(acceptance, "_wsl_path", lambda _path: "/mnt/d/map/octomap.ot")

    assert acceptance._native_path_arg(
        Path("navd.exe"), asset, platform_name="nt"
    ) == str(asset.resolve())
    assert acceptance._native_path_arg(
        Path("navd"), asset, platform_name="nt"
    ) == "/mnt/d/map/octomap.ot"


@pytest.mark.skipif(acceptance.os.name != "nt", reason="Windows candidate resolution")
def test_binary_candidate_resolver_windows_prefers_existing_exe_and_falls_back(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    exe_candidate = (
        tmp_path
        / "build/nav-cpp/windows-x64-nav-endpoint/Release/navd.exe"
    )
    linux_candidate = tmp_path / "build/mujoco_native_nav/navd"
    exe_candidate.parent.mkdir(parents=True)
    linux_candidate.parent.mkdir(parents=True)
    linux_candidate.write_bytes(b"linux")
    monkeypatch.setattr(acceptance, "ROOT", tmp_path)

    fallback = acceptance._resolve_binary(
        {
            "candidates": [
                exe_candidate.relative_to(tmp_path).as_posix(),
                linux_candidate.relative_to(tmp_path).as_posix(),
            ]
        }
    )

    exe_candidate.write_bytes(b"windows")
    preferred = acceptance._resolve_binary(
        {
            "candidates": [
                linux_candidate.relative_to(tmp_path).as_posix(),
                exe_candidate.relative_to(tmp_path).as_posix(),
            ]
        }
    )

    assert fallback == linux_candidate.resolve()
    assert preferred == exe_candidate.resolve()


def test_native_traversability_identity_is_injected_without_wslenv(monkeypatch, tmp_path):
    map_dir = tmp_path / "same_source_map"
    map_dir.mkdir()
    map_path = map_dir / "map.pcd"
    map_path.write_bytes(b"mujoco-map")
    for name in (
        "LINGTU_EXPLORE_ROUTE",
        "LINGTU_PRODUCT_SESSION_ID",
        "LINGTU_MAP_ID",
        "LINGTU_MAP_CONTENT_EPOCH",
    ):
        monkeypatch.delenv(name, raising=False)

    values = acceptance._native_traversability_env(
        paths={"map_dir": map_dir, "slam": map_path},
        phase="motion",
        domain_id=226,
    )
    command, env = acceptance._with_native_env(
        ["wsl.exe", "-e", "/tmp/lingtu_traversability_dds", "--domain-id", "226"],
        **values,
    )

    assert values["LINGTU_EXPLORE_ROUTE"] == "map"
    assert values["LINGTU_MAP_ID"] == "same_source_map"
    assert values["LINGTU_MAP_CONTENT_EPOCH"] == "1"
    assert values["LINGTU_PRODUCT_SESSION_ID"].startswith("mujoco-native-motion-226-")
    assert command[:3] == ["wsl.exe", "-e", "env"]
    assert env == {}
    assert "LINGTU_EXPLORE_ROUTE=map" in command
    assert "/tmp/lingtu_traversability_dds" in command


def test_native_traversability_identity_rejects_nonnumeric_epoch(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("LINGTU_MAP_CONTENT_EPOCH", "not-a-number")

    with pytest.raises(ValueError, match="must be a positive integer"):
        acceptance._native_traversability_env(
            paths={},
            phase="motion",
            domain_id=226,
        )


def test_native_traversability_identity_uses_popen_env_for_windows_exe(monkeypatch):
    monkeypatch.setattr(acceptance.os, "name", "nt")

    command, env = acceptance._with_native_env(
        ["D:\\build\\lingtu_traversability_dds.exe", "--domain-id", "226"],
        LINGTU_EXPLORE_ROUTE="live",
    )

    assert command == ["D:\\build\\lingtu_traversability_dds.exe", "--domain-id", "226"]
    assert env == {"LINGTU_EXPLORE_ROUTE": "live"}


@pytest.mark.skipif(sys.platform != "win32", reason="Windows UNC path contract")
def test_wsl_ext4_unc_paths_convert_to_linux_paths_without_resolving_share(monkeypatch):
    def unavailable_share(*_args, **_kwargs):
        raise OSError("WSL share is unavailable")

    monkeypatch.setattr(Path, "resolve", unavailable_share)
    assert acceptance._wsl_path(
        Path(r"\\wsl.localhost\Ubuntu\home\thunder\acceptance\nav_status.json")
    ) == "/home/thunder/acceptance/nav_status.json"
    assert acceptance._wsl_path(
        Path(r"\\wsl$\Ubuntu\home\thunder\acceptance\nav_status.json")
    ) == "/home/thunder/acceptance/nav_status.json"


def test_wsl_runtime_probe_is_bounded_and_reports_unavailable(monkeypatch):
    monkeypatch.setattr(acceptance.shutil, "which", lambda _name: "wsl.exe")

    def timeout_run(*_args, **kwargs):
        assert kwargs["timeout"] == acceptance.WSL_RUNTIME_PROBE_TIMEOUT_S
        raise subprocess.TimeoutExpired("wsl.exe", kwargs["timeout"])

    monkeypatch.setattr(acceptance.subprocess, "run", timeout_run)

    ok, detail = acceptance._probe_wsl_runtime()

    assert ok is False
    assert detail.startswith("TimeoutExpired:")


def test_native_probe_decodes_windows_wsl_utf16_diagnostics():
    assert acceptance._decode_native_probe_output("wsl: ready") == "wsl: ready"
    assert acceptance._decode_native_probe_output("wsl: ready".encode("utf-16")) == "wsl: ready"
    odd_utf16 = "wsl: ready\r\n".encode("utf-16-le") + b"\n"
    assert acceptance._decode_native_probe_output(odd_utf16) == "wsl: ready\r\n"
    mixed = "wsl: ready\r\n".encode("utf-16-le") + b"stdin records: clouds=0"
    assert acceptance._decode_native_probe_output(mixed) == "wsl: ready\r\nstdin records: clouds=0"


def test_native_acceptance_artifacts_require_wsl_ext4_for_windows_motion():
    ok, detail = acceptance._artifact_storage_probe(Path(r"D:\tmp\lingtu-native"))
    assert ok is False
    assert detail == "windows_9p_mount"

    ok, detail = acceptance._artifact_storage_probe(
        Path(r"\\wsl.localhost\Ubuntu\tmp\lingtu-native")
    )
    assert ok is True
    assert detail == "wsl_ext4_unc"


def test_native_runtime_inputs_are_mirrored_to_ext4_artifacts(monkeypatch, tmp_path):
    monkeypatch.setattr(acceptance.os, "name", "nt")
    monkeypatch.setattr(
        acceptance,
        "_artifact_storage_probe",
        lambda _path: (True, "wsl_ext4_unc"),
    )
    source = tmp_path / "source"
    map_dir = source / "map"
    map_dir.mkdir(parents=True)
    (map_dir / "map.pcd").write_bytes(b"map")
    (map_dir / "octomap.ot").write_bytes(b"octomap")
    (map_dir / "metadata.json").write_text("{}\n", encoding="utf-8")
    library = source / "paths"
    library.mkdir()
    (library / "default.path").write_text("path\n", encoding="utf-8")
    slam_config = source / "slam.yaml"
    slam_config.write_text("mode: localization\n", encoding="utf-8")
    paths = {
        "map_dir": map_dir,
        "slam": map_dir / "map.pcd",
        "planner": map_dir / "octomap.ot",
        "metadata": map_dir / "metadata.json",
        "path_library": library,
        "slam_config": slam_config,
    }

    mirrored, report = acceptance._mirror_native_runtime_inputs(
        paths, tmp_path / "run"
    )

    assert report["ok"] is True
    assert report["filesystem"] == "wsl_ext4_unc"
    assert mirrored["map_dir"].name == "map"
    assert mirrored["map_dir"] != map_dir
    assert mirrored["slam"].read_bytes() == b"map"
    assert mirrored["planner"].read_bytes() == b"octomap"
    assert mirrored["path_library"].joinpath("default.path").read_text(encoding="utf-8") == "path\n"
    assert mirrored["slam_config"].read_text(encoding="utf-8") == "mode: localization\n"


def test_native_acceptance_stops_before_workers_when_wsl_is_unavailable(
    monkeypatch, tmp_path
):
    manifest = tmp_path / "manifest.json"
    manifest.write_text('{"name": "test"}\n', encoding="utf-8")
    monkeypatch.setattr(
        acceptance,
        "_probe_wsl_runtime",
        lambda: (False, "TimeoutExpired:wsl.exe"),
    )
    monkeypatch.setattr(acceptance, "_requires_wsl_runtime", lambda _manifest: True)
    args = type(
        "Args",
        (),
        {
            "manifest": str(manifest),
            "world": "",
            "map_dir": "",
            "phase_duration_s": None,
            "diagnostic_imu_acc_mode": "",
            "diagnostic_scan_time_profile": "",
            "diagnostic_imu_acc_lowpass_hz": None,
            "diagnostic_imu_acc_max_dynamic_mps2": None,
            "diagnostic_imu_acc_max_slew_mps3": None,
            "record_video": False,
            "video_width": 1920,
            "video_height": 1080,
            "video_fps": 24.0,
            "video_lidar_points": 640,
            "out_dir": str(tmp_path / "report"),
        },
    )()

    report = acceptance.run(args)

    assert report["ok"] is False
    assert report["blockers"] == ["wsl_runtime_unavailable"]
    assert report["preflight"]["native_runtime_probe"]["detail"] == "TimeoutExpired:wsl.exe"
    assert (tmp_path / "report" / "report.json").is_file()


def test_native_acceptance_rejects_windows_9p_motion_artifacts(monkeypatch, tmp_path):
    manifest = tmp_path / "manifest.json"
    manifest.write_text('{"name": "test"}\n', encoding="utf-8")
    monkeypatch.setattr(acceptance, "_probe_wsl_runtime", lambda: (True, "ready"))
    monkeypatch.setattr(acceptance, "_requires_wsl_runtime", lambda _manifest: True)
    args = type(
        "Args",
        (),
        {
            "manifest": str(manifest),
            "world": "",
            "map_dir": "",
            "phase_duration_s": None,
            "diagnostic_imu_acc_mode": "",
            "diagnostic_scan_time_profile": "",
            "diagnostic_imu_acc_lowpass_hz": None,
            "diagnostic_imu_acc_max_dynamic_mps2": None,
            "diagnostic_imu_acc_max_slew_mps3": None,
            "record_video": False,
            "video_width": 1920,
            "video_height": 1080,
            "video_fps": 24.0,
            "video_lidar_points": 640,
            "mode": "motion",
            "allow_windows_9p_artifacts": False,
            "out_dir": str(tmp_path / "windows-artifacts"),
        },
    )()

    report = acceptance.run(args)

    assert report["ok"] is False
    assert report["blockers"] == ["native_acceptance_artifacts_on_windows_9p"]
    assert report["preflight"]["artifact_storage"]["detail"] == "windows_9p_mount"


def test_native_windows_chain_skips_wsl_and_ext4_gates(monkeypatch, tmp_path):
    manifest = tmp_path / "manifest.json"
    manifest.write_text('{"name": "test"}\n', encoding="utf-8")
    monkeypatch.setattr(acceptance, "_requires_wsl_runtime", lambda _manifest: False)

    def unexpected_probe(*_args, **_kwargs):
        raise AssertionError("native Windows workers must not probe WSL or ext4")

    monkeypatch.setattr(acceptance, "_probe_wsl_runtime", unexpected_probe)
    monkeypatch.setattr(acceptance, "_artifact_storage_probe", unexpected_probe)
    monkeypatch.setattr(
        acceptance,
        "_preflight",
        lambda _manifest: ({}, {}, ["expected_preflight_stop"], {}),
    )
    args = SimpleNamespace(
        manifest=str(manifest),
        world="",
        map_dir="",
        phase_duration_s=None,
        diagnostic_imu_acc_mode="",
        diagnostic_scan_time_profile="",
        diagnostic_imu_acc_lowpass_hz=None,
        diagnostic_imu_acc_max_dynamic_mps2=None,
        diagnostic_imu_acc_max_slew_mps3=None,
        record_video=False,
        video_width=1920,
        video_height=1080,
        video_fps=24.0,
        video_lidar_points=640,
        out_dir=str(tmp_path / "native-windows"),
        prepare_assets=False,
        build_helper=False,
        preflight_only=True,
        mode="motion",
        domain_id=220,
        allow_windows_9p_artifacts=False,
    )

    report = acceptance.run(args)

    assert report["blockers"] == ["expected_preflight_stop"]
    provenance = report["preflight"]["map_provenance"]
    assert provenance["native_runtime_probe"] == {
        "ok": True,
        "detail": "native_windows_pe",
    }
    assert provenance["artifact_storage"] == {
        "ok": True,
        "detail": "native_windows_filesystem",
        "path": str((tmp_path / "native-windows").resolve()),
    }
    assert provenance["native_asset_mirror"] == {
        "enabled": False,
        "ok": True,
        "reason": "native_windows_pe",
    }


def test_wsl_cleanup_control_calls_use_bounded_timeout(monkeypatch):
    calls: list[float] = []
    monkeypatch.setattr(sensors.os, "name", "nt")
    monkeypatch.setattr(sensors.shutil, "which", lambda _name: "wsl.exe")

    def timeout_run(*_args, **kwargs):
        calls.append(float(kwargs["timeout"]))
        raise subprocess.TimeoutExpired("wsl.exe", kwargs["timeout"])

    monkeypatch.setattr(sensors.subprocess, "run", timeout_run)

    assert sensors._wsl_pid_alive(123) is True
    assert sensors._signal_wsl_pid(123, "TERM") is False
    assert calls == [sensors.WSL_CONTROL_TIMEOUT_S, sensors.WSL_CONTROL_TIMEOUT_S]


def test_managed_wsl_process_fails_when_linux_pid_ownership_is_missing(
    monkeypatch,
    tmp_path,
):
    class Process:
        pid = 4242
        returncode = None

        def poll(self):
            return self.returncode

    process = Process()

    class ProcessOwner:
        terminated = False

        @staticmethod
        def popen_options():
            return {}

        @staticmethod
        def attach(_process):
            return None

        def terminate(self, _process, *, timeout_s):
            assert timeout_s == pytest.approx(2.0)
            self.terminated = True

        @staticmethod
        def close():
            return None

    owner = ProcessOwner()
    monkeypatch.setattr(acceptance.os, "name", "nt")
    monkeypatch.setattr(acceptance, "ProcessTreeOwner", lambda: owner)
    monkeypatch.setattr(acceptance.subprocess, "Popen", lambda *_args, **_kwargs: process)
    monkeypatch.setattr(
        acceptance,
        "_managed_wsl_command",
        lambda command, _pid_path: command,
    )
    monkeypatch.setattr(acceptance, "_read_linux_pid", lambda _path: None)
    managed = acceptance.ManagedProcess(
        "navd",
        ["wsl.exe", "-e", "/opt/lingtu/navd"],
        tmp_path / "navd.log",
    )

    with pytest.raises(RuntimeError, match="ownership handshake failed"):
        managed.start()

    assert owner.terminated is True
    assert managed.process is None
    assert managed._log is None


def test_parent_sensor_diagnostics_args_use_case_artifact_and_controlled_period(tmp_path):
    artifact = tmp_path / "parent_sensor_diagnostics.json"

    defaults = acceptance._parent_sensor_diagnostics_args(artifact, {})
    overridden = acceptance._parent_sensor_diagnostics_args(
        artifact,
        {"parent_diagnostics_period_s": 0.75},
    )

    assert defaults == [
        "--parent-diagnostics-json",
        str(artifact),
        "--parent-diagnostics-period-s",
        "0.5",
    ]
    assert overridden[-1] == "0.75"
    with pytest.raises(ValueError, match="parent_diagnostics_period_s"):
        acceptance._parent_sensor_diagnostics_args(
            artifact,
            {"parent_diagnostics_period_s": 0},
        )


def test_compiled_lidar_offset_uses_final_site_pose_not_local_site_pos(tmp_path):
    model_path = tmp_path / "nested_lidar.xml"
    model_path.write_text(
        """
<mujoco model="nested_lidar">
  <compiler angle="degree"/>
  <worldbody>
    <body name="base_link" pos="2 3 1" euler="0 0 30">
      <body name="lidar_mount" pos="1 0 0" euler="0 0 90">
        <site name="lidar_site" pos="0.25 0 0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
""".strip(),
        encoding="utf-8",
    )

    offset = acceptance._compiled_mujoco_site_offset_body(model_path)

    assert offset == pytest.approx((1.0, 0.25, 0.0), abs=1e-9)
    assert offset != pytest.approx((0.25, 0.0, 0.0), abs=1e-9)


def test_thunderv4_compiled_lidar_offset_matches_body_frame_extrinsics():
    offset = acceptance._compiled_mujoco_site_offset_body(
        ROOT / "sim" / "robots" / "thunderv4" / "mjcf" / "thunderv4.xml"
    )

    assert offset == pytest.approx((-0.30638, 0.0, 0.19417), abs=1e-6)
    assert acceptance._sensor_offset_args(offset) == [
        "--sensor-offset-x-m",
        "-0.30638",
        "--sensor-offset-y-m",
        "0",
        "--sensor-offset-z-m",
        "0.19417",
    ]


def test_driver_bridge_protocol_rejects_unknown_nan_and_identity_mismatch():
    parse = sensors.NativeDriverBridge.parse_stdout_line
    bridge_boot_id = "b" * 32
    controller_boot_id = "controller-1"

    parsed = parse(
        "LT_DRIVER_COMMAND_V2\t"
        f"{bridge_boot_id}\t{controller_boot_id}\t7\tnav\tproducer-a\t91\t0.25\t-0.5\t1",
        bridge_boot_id=bridge_boot_id,
        controller_boot_id=controller_boot_id,
    )

    assert parsed.bridge_command_seq == 7
    assert parsed.kind == "nav"
    assert parsed.producer_boot_id == "producer-a"
    assert parsed.output_sequence == 91
    assert parsed.walk_x == pytest.approx(0.25)
    assert parsed.walk_y == pytest.approx(-0.5)
    assert parsed.walk_z == pytest.approx(1.0)

    invalid_lines = [
        "LT_CMD_V1\t7\t123.25\t0.3\t-0.1\t0.4",
        "LT_DRIVER_COMMAND_V2\t"
        f"{bridge_boot_id}\t{controller_boot_id}\t8\tnav\tproducer-a\t92\tnan\t0\t0",
        "LT_DRIVER_COMMAND_V2\t"
        f"{'c' * 32}\t{controller_boot_id}\t9\tnav\tproducer-a\t93\t0\t0\t0",
        "LT_DRIVER_COMMAND_V2\t"
        f"{bridge_boot_id}\t{controller_boot_id}\t10\tnav\tproducer-a\t94\t1_0\t0\t0",
    ]
    for line in invalid_lines:
        with pytest.raises(ValueError):
            parse(line, bridge_boot_id=bridge_boot_id, controller_boot_id=controller_boot_id)


def test_sensor_tick_applies_driver_bridge_command_before_ack():
    events: list[tuple[str, object, int | None]] = []
    command = SimpleNamespace(
        velocity=VelocityCommand(linear_x=0.2, linear_y=-0.1, angular_z=0.3),
        bridge_command_seq=7,
    )

    class Engine:
        def step_sensor_tick(self, cmd, *, dt_s):
            events.append(("step", cmd, None))
            assert dt_s == pytest.approx(0.005)
            return {"state": "stepped"}

    class Bridge:
        def complete_step(self, applied, step_seq):
            events.append(("applied", applied, step_seq))

    state, next_step_seq = sensors._step_with_driver_bridge(
        Engine(),
        Bridge(),
        command,
        imu_period_s=0.005,
        step_seq=41,
    )

    assert state == {"state": "stepped"}
    assert next_step_seq == 42
    assert events == [
        ("step", command.velocity, None),
        ("applied", command, 42),
    ]


def test_anchored_sensor_tick_acks_zero_without_running_policy_dynamics():
    events: list[tuple[str, object, int | None]] = []
    command = sensors.PreparedDriverBridgeStep(
        velocity=VelocityCommand(),
        protocol=SimpleNamespace(kind="activation_zero"),
    )

    class Engine:
        def step_static_sensor_tick(self, *, dt_s):
            events.append(("static", dt_s, None))
            return {"state": "anchored"}

        def step_sensor_tick(self, cmd, *, dt_s):
            raise AssertionError("anchored ticks must not advance policy dynamics")

    class Bridge:
        def complete_step(self, applied, *, step_seq):
            events.append(("applied", applied, step_seq))

    state, next_step_seq = sensors._step_anchored_with_driver_bridge(
        Engine(),
        Bridge(),
        command,
        imu_period_s=0.005,
        step_seq=41,
    )

    assert state == {"state": "anchored"}
    assert next_step_seq == 42
    assert events == [
        ("static", 0.005, None),
        ("applied", command.protocol, 42),
    ]


def test_anchored_sensor_tick_rejects_nonzero_motion_before_ack():
    command = sensors.PreparedDriverBridgeStep(
        velocity=VelocityCommand(linear_x=0.2),
        protocol=SimpleNamespace(kind="nav"),
    )

    class Engine:
        def step_static_sensor_tick(self, *, dt_s):
            raise AssertionError("nonzero anchored motion must be rejected before stepping")

    class Bridge:
        def complete_step(self, applied, *, step_seq):
            raise AssertionError("nonzero anchored motion must not be acknowledged")

    with pytest.raises(RuntimeError, match="exact physical stop"):
        sensors._step_anchored_with_driver_bridge(
            Engine(),
            Bridge(),
            command,
            imu_period_s=0.005,
            step_seq=41,
        )


def test_sensor_tick_does_not_ack_driver_bridge_command_when_physics_step_fails():
    command = SimpleNamespace(
        velocity=VelocityCommand(linear_x=0.2, linear_y=0.0, angular_z=0.0),
        bridge_command_seq=9,
    )

    class Engine:
        def step_sensor_tick(self, cmd, *, dt_s):
            raise RuntimeError("physics failed")

    class Bridge:
        def complete_step(self, applied, step_seq):
            raise AssertionError("failed physics steps must not be ACKed")

    with pytest.raises(RuntimeError, match="physics failed"):
        sensors._step_with_driver_bridge(
            Engine(),
            Bridge(),
            command,
            imu_period_s=0.005,
            step_seq=41,
        )


def test_driver_heartbeat_refreshes_during_sensor_work_and_stops_cleanly():
    refreshed = threading.Event()

    class Bridge:
        heartbeat_timeout_ms = 60

        def refresh_heartbeat(self):
            refreshed.set()
            return True

    heartbeat = sensors.DriverHeartbeat(Bridge())
    heartbeat.start()
    assert refreshed.wait(timeout=0.5)
    heartbeat.stop()


def test_start_anchor_releases_once_real_motion_starts():
    assert sensors._start_anchor_active("run", motion_started=True) is True
    assert sensors._start_anchor_active("warmup", motion_started=False) is True
    assert sensors._start_anchor_active("warmup", motion_started=True) is False
    assert sensors._start_anchor_active("off", motion_started=False) is False


def test_static_sensor_tick_uses_engine_freeze_api():
    class FakeEngine:
        def step_static_sensor_tick(self, *, dt_s):
            return {"dt_s": dt_s}

    assert sensors._step_static_engine_for_sensor_tick(FakeEngine(), 0.005) == {"dt_s": 0.005}


def test_dropped_static_tick_uses_fast_clock_only_api():
    class FakeEngine:
        def __init__(self):
            self.calls = []

        def advance_static_sensor_clock(self, *, dt_s):
            self.calls.append(dt_s)
            return 4.25

        def step_static_sensor_tick(self, *, dt_s):
            raise AssertionError("dropped anchored ticks must not run mj_forward")

    engine = FakeEngine()

    assert sensors._advance_static_engine_clock_for_dropped_tick(engine, 0.005) == 4.25
    assert engine.calls == [0.005]


def test_acceptance_text_capture_tolerates_missing_subprocess_streams():
    assert acceptance._text_tail(None) == ""
    assert acceptance._text_tail("abcdef", 3) == "def"


def test_mujoco_sensor_record_clock_contract_separates_navigation_fixture_from_replay():
    publisher = (
        ROOT / "src" / "drivers" / "real" / "lidar" / "sdk2_stream" / "main.cpp"
    ).read_text(encoding="utf-8")
    sensor_bridge = (
        ROOT / "sim" / "scripts" / "mujoco" / "native_dds_sensors.py"
    ).read_text(encoding="utf-8")
    endpoint_loop = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "runtime" / "loop.cpp"
    ).read_text(encoding="utf-8")
    pose_input = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "input" / "pose.cpp"
    ).read_text(encoding="utf-8")
    map_input = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "input" / "map.cpp"
    ).read_text(encoding="utf-8")
    health_input = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "input" / "health.cpp"
    ).read_text(encoding="utf-8")
    nav_main = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "main.cpp"
    ).read_text(encoding="utf-8")
    traversability = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "traversability" / "traversability_dds.cpp"
    ).read_text(encoding="utf-8")

    assert "--restamp-stdin-records" in publisher
    assert "replay_restamper.stamp_ns(source_timestamp_ns, target_deadline)" in publisher
    assert "if (navigation_fixture)" in publisher
    navigation_fixture_branch = publisher.split("if (navigation_fixture)", 1)[1].split("}", 1)[0]
    assert "return source_timestamp_ns;" in navigation_fixture_branch
    assert "replay_restamper.stamp_monotonic_ns(" not in publisher
    assert "source_timestamp_ns, std::chrono::steady_clock::now()" not in publisher
    assert "replay_rate > 0.0 && !navigation_fixture" in publisher
    restamper = (
        ROOT
        / "src"
        / "drivers"
        / "real"
        / "lidar"
        / "sdk2_stream"
        / "replay_deadline_restamper.hpp"
    ).read_text(encoding="utf-8")
    assert "steady_now - target_deadline" in restamper
    assert "cached_source_ns_ == source_timestamp_ns" in restamper
    assert "*cached_output_ns_ <= realtime_now_ns" in restamper
    publisher_start = sensor_bridge.split("def _start_native_publisher", 1)[1].split(
        "def ", 1
    )[0]
    restamp_policy = sensor_bridge.split("def _should_restamp_native_records", 1)[1].split(
        "def _start_native_publisher", 1
    )[0]
    assert 'if bool(getattr(args, "navigation_fixture", False)):' in restamp_policy
    assert "return False" in restamp_policy
    assert "if _should_restamp_native_records(args):" in publisher_start
    assert 'publisher_args.append("--restamp-stdin-records")' in publisher_start
    assert "inputs.projectTf(msg, steadySeconds())" in endpoint_loop
    assert "inputs.projectOdometry(msg, steadySeconds())" in endpoint_loop
    assert "classifySourceOrder(state_.last_tf_s" in pose_input
    assert "classifySourceOrder(state_.last_odom_s" in pose_input
    assert 'resetEpoch(transform->stamp_s, "source_clock_rebase", false)' in pose_input
    tf_handler = pose_input.split("InputProjector::projectTf", 1)[1].split(
        "InputProjector::projectOdometry", 1
    )[0]
    assert tf_handler.index("const bool map_frame_jump") < tf_handler.index(
        "stamp_decision == SourceStampDecision::kClockRebase"
    )
    assert "classifySourceOrder(state_.last_cloud_s" in map_input
    assert "classifySourceOrder(state_.last_traversability_s" in map_input
    assert "state_.localization_health.stamp_s" in health_input
    assert "input.odom_receive_s = state_.last_odom_receive_s;" in health_input
    assert "input.tf_receive_s = state_.last_tf_receive_s;" in health_input
    assert "input.cloud_receive_s = state_.last_cloud_receive_s;" in health_input
    assert (
        "input.traversability_receive_s = state_.last_traversability_receive_s;"
        in health_input
    )
    assert (
        "input.localization_health_receive_s = state_.localization_health_receive_s;"
        in health_input
    )
    cloud_handler = map_input.split("InputProjector::projectCloud", 1)[1].split(
        "InputProjector::projectTerrainMap", 1
    )[0]
    traversability_handler = map_input.split(
        "InputProjector::projectTraversability", 1
    )[1].split("InputProjector::projectLocalTraversability", 1)[0]
    health_handler = health_input.split("InputProjector::projectLocalizationHealth", 1)[1]
    assert "resetEpoch" not in cloud_handler
    assert "resetEpoch" not in traversability_handler
    assert "resetEpoch" not in health_handler
    assert "navigator.tickIntent(" in nav_main
    assert "navigator.tick(" in nav_main
    assert nav_main.count("steadySeconds") >= 2
    assert "const double schedule_now = steadySeconds();" in traversability
    assert "last_publish = schedule_now;" in traversability
    assert "last_map_odom_receive_s = steadySeconds();" in traversability
    assert "schedule_now - last_map_odom_receive_s" in traversability
    assert "now - map_odom->stamp_s" not in traversability
    assert "const auto odom_stamp_decision" in traversability
    assert "const auto cloud_stamp_decision" in traversability
    assert traversability.count("classifySourceOrder(") >= 3
    assert traversability.count("SourceStampDecision::kClockRebase") >= 3


def test_motion_log_contract_captures_native_navigation_and_visual_state(tmp_path):
    status = tmp_path / "nav.json"
    status.write_text(
        json.dumps({"global_path": [[0.0, 0.0, 0.0]], "local_path": []}),
        encoding="utf-8",
    )
    assert sensors._read_json_object(status)["global_path"]
    assert sensors._read_json_object(tmp_path / "missing.json") == {}
    assert sensors._native_nav_goal_reached({"last_local": {"goal_reached": True}}) is True
    assert sensors._native_nav_goal_reached({}) is False

    sensor_source = (ROOT / "sim" / "scripts" / "mujoco" / "native_dds_sensors.py").read_text(encoding="utf-8")
    for required_field in (
        '"qpos"',
        '"global_path"',
        '"local_path"',
        '"planner_debug_id"',
        '"nav_status_stamp_s"',
        '"lidar_world"',
        '"cmd"',
    ):
        assert required_field in sensor_source
    assert "planner_debug_writer" in sensor_source
    assert "AsyncJsonlWriter" in sensor_source


def test_sensor_loop_records_the_exact_terminal_goal_snapshot_before_breaking():
    sensor_source = (ROOT / "sim" / "scripts" / "mujoco" / "native_dds_sensors.py").read_text(
        encoding="utf-8"
    )

    goal_check = sensor_source.index("goal_reached_this_tick = False")
    motion_log = sensor_source.index("motion_log_due =")
    terminal_break = sensor_source.index("if goal_reached_this_tick:", motion_log)
    assert goal_check < motion_log < terminal_break
    assert "motion_log_due or goal_reached_this_tick" in sensor_source
    assert "nav_status_snapshot" in sensor_source


def test_motion_log_joins_exact_planner_debug_id_across_clock_domains(tmp_path):
    motion_log = tmp_path / "motion.jsonl"
    debug_log = tmp_path / "motion_planner_debug.jsonl"
    motion_log.write_text(
        json.dumps(
            {
                "qpos": [0.0],
                "driving": True,
                "t": 12.5,
                "planner_debug_id": 8,
                "nav_status_stamp_s": 12.6,
                "global_path": [[9.0, 0.0, 0.0]],
                "input_gate": {"reason": "future"},
            }
        )
        + "\n",
        encoding="utf-8",
    )
    snapshots = [
        {
            "id": 7,
            "nav_status_stamp_s": 12.4,
            "local_planner_debug": {
                "valid": True,
                "timestamp_s": 12.3,
                "candidates": [{"selected": True, "path": [[0, 0, 0], [1, 0, 0]]}],
            },
            "local_map": {"enabled": True, "obstacle_points": [[1, 0, 0, 1]]},
            "global_path": [[0, 0, 0], [4, 0, 0]],
            "local_path": [[0, 0, 0], [1, 0, 0]],
            "last_local": {"reason": "control_ready", "near_field_stop": False},
            "input_gate": {"reason": "ready"},
            "dynamic_objects": [
                {"id": 3, "centroid": [1.0, 0.2, 0.3], "velocity": [0.2, 0.0, 0.0]}
            ],
        },
        {
            "id": 8,
            "nav_status_stamp_s": 12.6,
            "local_planner_debug": {
                "valid": True,
                "timestamp_s": 12.55,
                "candidates": [{"selected": True, "path": [[0, 0, 0], [2, 0, 0]]}],
            },
            "local_map": {"enabled": True, "obstacle_points": [[2, 0, 0, 1]]},
            "global_path": [[0, 0, 0], [8, 0, 0]],
            "local_path": [[0, 0, 0], [2, 0, 0]],
            "last_local": {"reason": "control_ready", "near_field_stop": False},
            "input_gate": {"reason": "ready"},
            "dynamic_objects": [
                {"id": 8, "centroid": [2.0, 0.2, 0.3], "velocity": [0.2, 0.0, 0.0]}
            ],
        },
    ]
    debug_log.write_text(
        "".join(json.dumps(snapshot) + "\n" for snapshot in snapshots),
        encoding="utf-8",
    )

    rows = navigation_video._load_jsonl(motion_log)

    assert len(rows) == 1
    assert rows[0]["planner_debug_id"] == 8
    assert rows[0]["nav_status_stamp_s"] == 12.6
    assert rows[0]["planner_debug_join"] == "exact_id"
    assert rows[0]["nav_status_hold_age_s"] == pytest.approx(0.0)
    assert rows[0]["local_candidates"][0]["selected"] is True
    assert rows[0]["local_map"]["enabled"] is True
    assert rows[0]["local_planner_debug"]["timestamp_s"] == 12.55
    assert rows[0]["local_reason"] == "control_ready"
    assert rows[0]["global_path"][-1] == [8, 0, 0]
    assert rows[0]["input_gate"]["reason"] == "ready"
    assert rows[0]["dynamic_objects"][0]["id"] == 8
    assert navigation_video._snapshot_age_s(rows[0]) == pytest.approx(0.0)


def test_exact_planner_debug_id_expires_on_motion_clock_when_status_stops(tmp_path):
    motion_log = tmp_path / "motion.jsonl"
    debug_log = tmp_path / "motion_planner_debug.jsonl"
    motion_log.write_text(
        "".join(
            json.dumps(
                {
                    "qpos": [0.0],
                    "driving": True,
                    "t": stamp,
                    "planner_debug_id": 5,
                }
            )
            + "\n"
            for stamp in (100.0, 100.9)
        ),
        encoding="utf-8",
    )
    debug_log.write_text(
        json.dumps(
            {
                "id": 5,
                "nav_status_stamp_s": 200.0,
                "local_planner_debug": {
                    "valid": True,
                    "timestamp_s": 50.0,
                    "candidates": [
                        {"selected": True, "path": [[0, 0, 0], [1, 0, 0]]}
                    ],
                },
                "local_map": {"enabled": True},
            }
        )
        + "\n",
        encoding="utf-8",
    )

    rows = navigation_video._load_jsonl(motion_log)

    assert navigation_video._snapshot_age_s(rows[0]) == pytest.approx(0.0)
    assert navigation_video._effective_local_candidates(rows[0])
    assert navigation_video._snapshot_age_s(rows[1]) == pytest.approx(0.9)
    assert navigation_video._effective_local_candidates(rows[1]) == []


def test_missing_exact_planner_debug_id_does_not_guess_by_cross_clock_stamp(tmp_path):
    motion_log = tmp_path / "motion.jsonl"
    debug_log = tmp_path / "motion_planner_debug.jsonl"
    motion_log.write_text(
        json.dumps(
            {
                "qpos": [0.0],
                "driving": True,
                "t": 12.5,
                "planner_debug_id": 99,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    debug_log.write_text(
        json.dumps(
            {
                "id": 7,
                "nav_status_stamp_s": 12.4,
                "local_planner_debug": {
                    "valid": True,
                    "candidates": [
                        {"selected": True, "path": [[0, 0, 0], [1, 0, 0]]}
                    ],
                },
                "local_map": {"enabled": True},
            }
        )
        + "\n",
        encoding="utf-8",
    )

    row = navigation_video._load_jsonl(motion_log)[0]

    assert row.get("planner_debug_join") is None
    assert navigation_video._effective_local_candidates(row) == []
    assert navigation_video._effective_local_map(row) == {}


def test_navigation_video_snapshot_age_uses_joined_status_clock_not_planner_steady_clock():
    row = {
        "t": 1_784_361_436.85,
        "nav_status_stamp_s": 1_784_361_436.75,
        "local_planner_debug": {
            "valid": True,
            "timestamp_s": 489.48,
        },
        "local_candidates": [
            {
                "selected": True,
                "path": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            }
        ],
    }

    assert navigation_video._snapshot_age_s(row) == pytest.approx(0.1)
    assert navigation_video._planner_snapshot_is_fresh(row) is True
    assert navigation_video._effective_local_candidates(row)


def test_native_navigation_video_preserves_source_timeline_with_cfr_holds():
    source_rows = [
        {"t": 10.0, "qpos": [0.0], "x": 0.0},
        {"t": 10.1, "qpos": [0.0], "x": 0.1},
        {"t": 11.1, "qpos": [0.0], "x": 1.1},
    ]

    rows, timeline = navigation_video._resample_rows_for_cfr(source_rows, 10.0)

    assert len(rows) == 12
    assert rows[2]["x"] == pytest.approx(0.1)
    assert rows[2]["t"] == pytest.approx(10.2)
    assert rows[-1]["x"] == pytest.approx(1.1)
    assert timeline["source_duration_s"] == pytest.approx(1.2)
    assert timeline["encoded_duration_s"] == pytest.approx(1.2)
    assert timeline["max_source_gap_s"] == pytest.approx(1.0)
    assert timeline["timeline_preserved"] is True


def test_native_navigation_video_hides_stale_and_overhead_lidar_only_from_presentation():
    fresh_row = {
        "t": 10.2,
        "nav_status_stamp_s": 10.1,
        "timeline_source_stamp_s": 10.1,
        "timeline_hold_age_s": 0.1,
        "z": 0.5,
        "lidar_world": [
            [1.0, 0.0, 0.2],
            [1.0, 0.0, 3.2],
        ],
        "local_map": {
            "enabled": True,
            "obstacle_points_fresh": True,
            "obstacle_points": [
                [1.0, 0.1, 0.3, 1.0],
                [1.0, 0.1, 3.2, 1.0],
            ],
        },
    }

    visible = navigation_video._presentation_lidar_points(fresh_row)
    planner_visible = navigation_video._presentation_planner_obstacle_points(fresh_row)

    assert [point.tolist() for point in visible] == [[1.0, 0.0, 0.2]]
    assert [point.tolist() for point in planner_visible] == [[1.0, 0.1, 0.3]]
    assert fresh_row["lidar_world"] == [
        [1.0, 0.0, 0.2],
        [1.0, 0.0, 3.2],
    ]
    assert fresh_row["local_map"]["obstacle_points"] == [
        [1.0, 0.1, 0.3, 1.0],
        [1.0, 0.1, 3.2, 1.0],
    ]

    scan_row = {
        **fresh_row,
        "local_map": {
            "enabled": True,
            "obstacle_points_fresh": True,
            "obstacle_points": [],
            "collision": {
                "live": True,
                "complete": True,
                "occupied_points": [[1.2, 0.0, 0.4], [1.2, 0.0, 3.2]],
            },
        },
    }
    scan_visible = navigation_video._presentation_planner_obstacle_points(scan_row)
    assert [point.tolist() for point in scan_visible] == [[1.2, 0.0, 0.4]]

    stale_row = dict(fresh_row, t=11.0, timeline_hold_age_s=0.9)
    assert navigation_video._presentation_lidar_points(stale_row) == []
    assert navigation_video._presentation_planner_obstacle_points(stale_row) == []


def test_motion_log_lidar_sampling_filters_overhead_before_spending_point_budget():
    points = np.asarray(
        [[float(index), 0.0, 3.0, 10.0] for index in range(20)]
        + [[0.2 * float(index), 1.0, 0.6, 20.0] for index in range(6)]
        + [[10.0 + float(index), 1.0, 0.6, 30.0] for index in range(12)],
        dtype=np.float32,
    )

    sampled = sensors._motion_log_lidar_sample(
        points,
        max_points=4,
        robot_xyz=[0.0, 0.0, 0.5],
    )

    assert sampled.shape == (4, 4)
    assert np.all(sampled[:, 2] <= 2.3)
    assert np.count_nonzero(sampled[:, 0] < 4.0) == 3
    assert np.count_nonzero(sampled[:, 0] >= 10.0) == 1


def test_local_planner_inset_draws_raw_lidar_and_planner_obstacles_as_separate_layers():
    pytest.importorskip("cv2", reason="local-planner rendering requires the optional vision extra")
    row = {
        "t": 10.2,
        "nav_status_stamp_s": 10.1,
        "timeline_hold_age_s": 0.0,
        "x": 0.0,
        "y": 0.0,
        "z": 0.5,
        "yaw": 0.0,
        "lidar_world": [[0.6, -0.4, 0.4]],
        "local_map": {
            "enabled": True,
            "obstacle_points_fresh": True,
            "obstacle_points": [[0.9, 0.4, 0.4, 1.0]],
        },
    }
    panel_size = 360
    frame = navigation_video._render_local_planner_inset(
        np.zeros((720, 1280, 3), dtype=np.uint8),
        row,
        panel_size=panel_size,
    )
    origin = (1280 - panel_size - 12, 12)
    center = (panel_size // 2, panel_size // 2 + 12)
    pixels_per_m = panel_size * 0.43 / 2.0

    def patch_has_color(world_xy, color):
        pixel = navigation_video._world_xy_to_panel(
            *world_xy,
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=0.0,
            center=center,
            pixels_per_m=pixels_per_m,
        )
        patch = frame[
            origin[1] + pixel[1] - 3 : origin[1] + pixel[1] + 4,
            origin[0] + pixel[0] - 3 : origin[0] + pixel[0] + 4,
        ]
        expected = np.asarray(color, dtype=np.float32) * 0.94
        color_error = np.linalg.norm(
            patch.astype(np.float32) - expected.reshape(1, 1, 3),
            axis=2,
        )
        return bool(np.any(color_error <= 3.0))

    assert patch_has_color((0.6, -0.4), navigation_video._RAW_LIDAR_BGR)
    assert patch_has_color((0.9, 0.4), navigation_video._PLANNER_OBSTACLE_BGR)


def test_navigation_video_reduces_all_scene_lights_before_rendering():
    headlight = SimpleNamespace(
        diffuse=np.asarray([0.6, 0.5, 0.4], dtype=np.float32),
        ambient=np.asarray([0.3, 0.2, 0.1], dtype=np.float32),
        specular=np.asarray([0.09, 0.06, 0.03], dtype=np.float32),
    )
    model = SimpleNamespace(
        light_diffuse=np.asarray([[0.8, 0.7, 0.6]], dtype=np.float32),
        light_ambient=np.asarray([[0.4, 0.3, 0.2]], dtype=np.float32),
        light_specular=np.asarray([[0.12, 0.08, 0.04]], dtype=np.float32),
        vis=SimpleNamespace(headlight=headlight),
    )
    before = {
        name: np.asarray(getattr(model, name)).copy()
        for name in ("light_diffuse", "light_ambient", "light_specular")
    }
    headlight_before = {
        name: np.asarray(getattr(headlight, name)).copy()
        for name in ("diffuse", "ambient", "specular")
    }

    navigation_video._apply_presentation_light_gain(model, 0.65)

    for name, values in before.items():
        assert np.allclose(getattr(model, name), values * 0.65)
    for name, values in headlight_before.items():
        assert np.allclose(getattr(headlight, name), values * 0.65)


def test_navigation_video_luma_metrics_detect_clipped_white_frames():
    frame = np.asarray([[[255, 255, 255], [20, 30, 40]]], dtype=np.uint8)

    metrics = navigation_video._frame_luma_metrics(frame)

    assert metrics["white_clip_fraction"] == pytest.approx(0.5)
    assert metrics["pure_white_fraction"] == pytest.approx(0.5)
    assert metrics["luma_p50"] < metrics["luma_p99"]


def test_native_navigation_video_labels_fused_cost_cells_and_unknown_provenance():
    sampled_grid = {
        "fresh": True,
        "complete": False,
        "risk_cells_total": 364,
        "risk_cells_returned": 320,
    }

    assert navigation_video._fused_cost_status_text(sampled_grid) == (
        "fused cost cells 320/364  |  occupancy+terrain/unknown  |  "
        "sampled/unknown  |  fresh"
    )
    assert navigation_video._fused_cost_provenance_label(
        {"provenance": {"source_layers": ["occupancy", "terrain"]}}
    ) == "occupancy+terrain"


def test_native_navigation_video_discloses_presentation_filter_and_separates_sensor_legends():
    assert navigation_video._presentation_filter_status_text() == (
        "presentation filter: overhead raw lidar + planner obstacles > robot+1.8m hidden"
    )

    sensor_legend = navigation_video._sensor_overlay_legend()
    candidate_legend = navigation_video._candidate_overlay_legend()
    assert [label for label, _color in sensor_legend] == [
        "raw lidar",
        "planner obstacles",
    ]
    assert navigation_video._RAW_LIDAR_BGR != navigation_video._PLANNER_OBSTACLE_BGR
    assert not {
        label for label, _color in sensor_legend
    }.intersection(label for label, _color in candidate_legend)


def test_native_navigation_video_shows_local_planner_decision_layers():
    points = navigation_video._path3([[1.0, 2.0, 3.0], [float("nan"), 0.0, 0.0], [4.0, 5.0]])
    assert len(points) == 1
    assert points[0].tolist() == [1.0, 2.0, 3.0]

    renderer_source = (ROOT / "sim" / "scripts" / "mujoco" / "native_navigation_video.py").read_text(encoding="utf-8")
    acceptance_source = (ROOT / "sim" / "scripts" / "mujoco" / "native_navigation_acceptance.py").read_text(
        encoding="utf-8"
    )
    assert '"text_overlay": True' in renderer_source
    assert '"inset_overlay": bool(show_debug_inset)' in renderer_source
    assert "active_only: bool = True" in renderer_source
    assert "show_debug_inset: bool = True" in renderer_source
    assert 'camera_preset: str = "overview"' in renderer_source
    assert "show_goal: bool = True" in renderer_source
    assert '"local_candidates"' in renderer_source
    assert '"planner_local_map"' in renderer_source
    assert '"--nav-status-json", str(nav_status)' in acceptance_source
    assert '"--local-planner-debug-candidates"' in acceptance_source
    assert '"--local-map-debug-points"' in acceptance_source
    assert "render_native_navigation_video(" in acceptance_source


def test_local_planner_visualization_uses_distinct_candidate_colors_and_selected_green():
    pytest.importorskip("cv2", reason="local-planner rendering requires the optional vision extra")
    assert navigation_video._candidate_bgr("feasible", selected=False) != navigation_video._candidate_bgr(
        "collision_blocked", selected=False
    )
    assert navigation_video._candidate_bgr("terrain_blocked", selected=False) != navigation_video._candidate_bgr(
        "collision_blocked", selected=False
    )
    assert navigation_video._candidate_bgr("terrain_cost", selected=False) != navigation_video._candidate_bgr(
        "terrain_blocked", selected=False
    )
    assert navigation_video._candidate_bgr("feasible", selected=True) == (48, 245, 72)
    assert navigation_video._local_path_bgr(
        {
            "t": 10.2,
            "nav_status_stamp_s": 10.1,
            "local_reason": "recovery_path",
            "local_candidates": [],
        }
    ) == navigation_video._RECOVERY_BGR
    goal_row = {
        "t": 10.2,
        "nav_status_stamp_s": 10.1,
        "local_reason": "goal_reached",
        "local_diagnostics": {"goal_reached": True},
    }
    assert navigation_video._local_goal_reached(goal_row) is True
    stopped_row = {
        "t": 10.2,
        "nav_status_stamp_s": 10.1,
        "local_reason": "control_ready",
        "local_diagnostics": {"final_safety": {"stopped": True}},
        "local_planner_debug": {"valid": True, "timestamp_s": 10.1},
        "local_candidates": [{"selected": True}],
    }
    assert navigation_video._local_path_bgr(stopped_row) == navigation_video._STOPPED_BGR
    stopped_row["local_diagnostics"] = {"near_field_stop": True}
    assert navigation_video._local_path_bgr(stopped_row) == navigation_video._STOPPED_BGR
    stale_row = {
        "t": 11.0,
        "local_planner_debug": {"valid": True, "timestamp_s": 10.0},
        "local_candidates": [{"selected": True}],
        "local_map": {"enabled": True},
    }
    assert navigation_video._planner_snapshot_is_fresh(stale_row) is False
    assert navigation_video._effective_local_candidates(stale_row) == []
    assert navigation_video._effective_local_map(stale_row) == {}
    fresh_local_map = {
        "t": 10.2,
        "nav_status_stamp_s": 10.1,
        "local_map": {"enabled": True, "obstacle_points": [[1.0, 0.0, 0.2]]},
    }
    assert navigation_video._effective_local_map(fresh_local_map) == fresh_local_map["local_map"]
    stale_safety_row = {
        "t": 11.0,
        "nav_status_stamp_s": 10.0,
        "local_reason": "recovery_path",
        "local_diagnostics": {
            "near_field_stop": True,
            "recovery_state": 2,
            "final_safety": {"stopped": True},
        },
    }
    assert navigation_video._local_safety_stopped(stale_safety_row) is False
    assert navigation_video._local_is_recovery(stale_safety_row) is False
    stale_map_row = {
        "t": 11.0,
        "nav_status_stamp_s": 10.0,
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0,
        "local_map": {
            "enabled": True,
            "obstacle_points": [[0.5, 0.0, 0.0, 1.0]],
            "traversability": {
                "fresh": True,
                "rows": 2,
                "cols": 2,
                "resolution_m": 0.2,
                "origin_xy": [0.0, 0.0],
                "risk_cells": [[0, 0, 100.0]],
            },
        },
    }
    no_map_row = dict(stale_map_row)
    no_map_row["local_map"] = {}
    blank = np.zeros((270, 480, 3), dtype=np.uint8)
    assert np.array_equal(
        navigation_video._render_local_planner_inset(blank, stale_map_row),
        navigation_video._render_local_planner_inset(blank, no_map_row),
    )
    assert navigation_video._traversability_is_complete({"complete": True}) is True
    assert navigation_video._traversability_is_complete(
        {"rows": 60, "cols": 60, "risk_cells": [[0, 0, 100.0]]}
    ) is False
    assert navigation_video._snapshot_age_s(
        {
            "nav_status_stamp_s": 100.0,
            "local_planner_debug": {"valid": False, "timestamp_s": 0.0},
        }
    ) is None

    non_contiguous_bgr = np.zeros((720, 1280, 3), dtype=np.uint8)[:, :, ::-1]
    assert not non_contiguous_bgr.flags.c_contiguous
    frame = navigation_video._render_local_planner_inset(
        non_contiguous_bgr,
            {
                "t": 10.2,
                "nav_status_stamp_s": 10.15,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "local_planner_debug": {"valid": True, "timestamp_s": 10.1},
            "local_candidates": [
                {
                    "state": "feasible",
                    "selected": False,
                    "path": [[0.0, 0.0, 0.0], [2.0, 0.5, 0.0]],
                },
                {
                    "state": "collision_blocked",
                    "selected": False,
                    "path": [[0.0, 0.0, 0.0], [1.5, -0.8, 0.0]],
                },
            ],
            "local_path": [[0.0, 0.0, 0.0], [2.5, 0.0, 0.0]],
            "local_map": {
                "enabled": True,
                "obstacle_points": [[1.0, 0.0, 0.0, 0.4]],
                "traversability": {
                    "rows": 4,
                    "cols": 4,
                    "resolution_m": 0.5,
                    "origin_xy": [-1.0, -1.0],
                    "complete": False,
                    "risk_cells_total": 8,
                    "risk_cells": [[2, 2, 90.0]],
                },
            },
        },
        panel_size=360,
    )
    assert frame.shape == (720, 1280, 3)
    assert np.count_nonzero(frame) > 0
    selected_pixel = navigation_video._world_xy_to_panel(
        1.0,
        0.0,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        center=(180, 192),
        pixels_per_m=360 * 0.43 / 2.0,
    )
    selected_bgr = frame[12 + selected_pixel[1], 1280 - 360 - 12 + selected_pixel[0]]
    assert int(selected_bgr[1]) > int(selected_bgr[2])


def test_native_navigation_video_hides_roof_without_hiding_robot_visuals():
    names = ["roof_deck_main", "roof_strip_east", "chassis", "FR_wheel"]
    fake_mujoco = SimpleNamespace(
        mjtObj=SimpleNamespace(mjOBJ_GEOM=1),
        mj_id2name=lambda _model, _kind, geom_id: names[geom_id],
    )
    model = SimpleNamespace(ngeom=len(names), geom_group=[0, 0, 5, 5])

    navigation_video._hide_observer_occluders(fake_mujoco, model)

    assert model.geom_group == [4, 4, 5, 5]


def test_native_acceptance_manifest_declares_exact_product_chain():
    manifest = json.loads(
        (ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_native_navigation_acceptance.json").read_text(
            encoding="utf-8"
        )
    )

    contracts = manifest["contracts"]
    assert contracts["sensor_inputs"] == ["rt/lidar/raw_frame", "rt/imu/raw"]
    assert contracts["terrain_output"] == "rt/nav/traversability"
    assert "rt/nav/command/request" in contracts["navigation_inputs"]
    assert "rt/nav/goal_pose" not in contracts["navigation_inputs"]
    assert contracts["navigation_command_ack"] == "rt/nav/command/ack"
    assert contracts["navigation_goal_status"] == "rt/nav/goal/status"
    assert contracts["navigation_command_transport"] == "typed_dds_request_ack"
    assert contracts["locomotion_input"] == "rt/nav/cmd_vel"
    assert contracts["local_path_role"] == "dds_telemetry_and_preview"
    assert contracts["path_follower_role"] == "embedded_before_cmd_vel_gate"
    assert "Python global planner" in contracts["forbidden_compute"]
    assert manifest["phases"]["no_motion"]["publish_cmd_vel"] is False
    assert manifest["phases"]["motion"]["publish_cmd_vel"] is True
    assert manifest["runtime_tolerances"]["sim_hardware_realtime_factor"] == 1.0
    assert manifest["runtime_tolerances"]["input_future_tolerance_s"] == 0.05
    assert manifest["runtime_tolerances"]["input_recovery_frames"] >= 1
    assert manifest["frame_contract"]["traversability_geometry_frame_current"] == "map"
    assert manifest["frame_contract"]["traversability_header_frame_current"] == "map"
    slam_config = (ROOT / manifest["paths"]["slam_config"]).read_text(encoding="utf-8")
    assert "r_il: [0.7071067812, 0.0, 0.7071067812, 0.0, -1.0, 0.0, 0.7071067812, 0.0, -0.7071067812]" in slam_config
    assert "t_il: [0.402876074867229, 0.0, 0.0582019450665819]" in slam_config
    assert "navigation_body_from_imu_translation: [0.0, 0.0, 0.0]" in slam_config
    assert manifest["runtime_tolerances"]["track_against_map_period_s"] == 5.0
    assert manifest["thresholds"]["min_track_against_map_successes"] == 1
    assert manifest["asset_builder"]["kind"] == "saved_map_plan_gate"
    assert manifest["asset_builder"]["map_source"] == "mujoco_lidar"
    assert manifest["world"] == ""
    assert manifest["map_dir"] == ""


@pytest.mark.parametrize(
    "manifest_name",
    [
        "mujoco_native_navigation_acceptance.json",
        "mujoco_multifloor_navigation_acceptance.json",
        "mujoco_industrial_park_60m_navigation_acceptance.json",
    ],
)
def test_native_navigation_manifest_uses_physical_driver_bridge(manifest_name):
    manifest = acceptance._load_manifest(
        ROOT / "config" / "runtime_graph" / "acceptance" / manifest_name
    )

    assert "cmd_vel_tap" not in manifest["binaries"]
    assert manifest["binaries"]["driver_bridge"] == {
        "env": "LINGTU_MUJOCO_DRIVER_BRIDGE_BIN",
        "candidates": [
            "build/windows-native-dds-adapter/Release/lingtu_mujoco_driver_bridge.exe",
            "build/mujoco_native_dds/lingtu_mujoco_driver_bridge",
        ],
    }
    assert manifest["driver_runtime"] == {
        "max_linear_mps": 1.0,
        "max_angular_rps": 1.0,
        "command_timeout_ms": 200,
        "heartbeat_timeout_ms": 500,
        "apply_timeout_ms": 500,
    }
    assert manifest["contracts"]["locomotion_input"] == "rt/nav/cmd_vel"
    assert manifest["contracts"]["locomotion_state"] == "rt/driver/control_state"
    assert manifest["contracts"]["locomotion_ack_source"] == "physical_mujoco_step"


def test_native_navigation_runner_has_no_diagnostic_tap_in_formal_path():
    source = (
        ROOT / "sim" / "scripts" / "mujoco" / "native_navigation_acceptance.py"
    ).read_text(encoding="utf-8")

    assert "cmd_vel_tap" not in source
    assert "--cmd-vel-tap-bin" not in source
    assert "--cmd-vel-timeout-s" not in source
    assert "_native_driver_runtime_launch(" in source
    assert "*driver_runtime.driver_bridge_args" in source
    assert '"ack_source": "physical_mujoco_step"' in source


def test_multifloor_merged_manifest_locks_runtime_octoplanner3d_constraints():
    manifest = acceptance._load_manifest(
        ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_multifloor_navigation_acceptance.json"
    )

    assert manifest["asset_builder"]["scene_preset"] == "multifloor_stack_3"
    assert manifest["asset_builder"]["resolution"] == 0.09
    assert manifest["asset_builder"]["support_dilation_cells"] == 2
    assert manifest["start"][:3] == [7.0, -1.2, 0.55]
    assert math.isclose(manifest["start"][3], math.pi)
    assert manifest["goal"][:3] == [7.4, 2.8, 4.15]
    start_args = acceptance._sensor_start_args(manifest["start"])
    assert start_args[start_args.index("--start-yaw-deg") + 1] == "180.0"
    constraints = manifest["planner_constraints"]
    assert constraints["robot_radius_m"] == 0.15
    assert constraints["obstacle_clearance_radius_cells"] == 2
    assert constraints["obstacle_clearance_weight"] == 2.0
    assert constraints["body_clearance_below_m"] == 0.15
    assert constraints["body_clearance_above_m"] == 0.30
    assert constraints["support_height_m"] == 0.55
    assert constraints["max_step_height_m"] == 0.23
    assert constraints["max_slope"] == 0.65
    args = acceptance._planner_constraint_args(manifest)
    assert args[args.index("--octo-max-step-height-m") + 1] == "0.23"
    assert args[args.index("--octo-support-height-m") + 1] == "0.55"
    sensor_args = acceptance._sensor_runtime_args(manifest)
    assert sensor_args[sensor_args.index("--publish-hz") + 1] == "10.0"
    assert sensor_args[sensor_args.index("--mid360-samples-per-frame") + 1] == "6000"
    assert "--publish-odom-prior" in sensor_args
    assert manifest["mapd_runtime"]["stale_ms"] == 3000
    assert manifest["slam_runtime"]["provider"] == "mujoco_navigation_fixture"
    assert manifest["navigation_runtime"]["local_planner"] == "scan"
    assert manifest["thresholds"]["require_traversability"] is False
    runtime_args = acceptance._path_follower_args(manifest["navigation_runtime"])
    assert runtime_args[runtime_args.index("--path-follower-goal-tolerance-m") + 1] == "0.05"
    assert runtime_args[runtime_args.index("--path-follower-max-accel-mps2") + 1] == "0.3"
    assert runtime_args[runtime_args.index("--goal-reached-m") + 1] == "0.1"
    assert runtime_args[runtime_args.index("--waypoint-reached-m") + 1] == "0.2"
    assert acceptance._scan_follower_environment(manifest["navigation_runtime"]) == {
        "LINGTU_NAV_SCAN_TIME_FORWARD_S": "0.55",
        "LINGTU_NAV_SCAN_HEADING_ERROR_RAD": "0.8",
        "LINGTU_NAV_SCAN_POSITION_GAIN": "0.9",
        "LINGTU_NAV_SCAN_YAW_GAIN": "1.2",
        "LINGTU_NAV_SCAN_MAX_VX_MPS": "0.5",
        "LINGTU_NAV_SCAN_MAX_VY_MPS": "0.25",
        "LINGTU_NAV_SCAN_MAX_YAW_RATE_RAD_S": "1",
    }
    assert manifest["driver_runtime"]["apply_timeout_ms"] == 1500


def test_industrial_park_navigation_matches_global_and_local_clearance_contracts():
    manifest = acceptance._load_manifest(
        ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_industrial_park_60m_navigation_acceptance.json"
    )

    assert manifest["planner_constraints"]["robot_radius_m"] == 0.95
    assert manifest["navigation_runtime"]["corridor_lookahead_m"] == 1.2
    assert manifest["sensor_runtime"].get("publish_odom_prior") is not True
    assert "odom_prior_velocity_window_s" not in manifest["sensor_runtime"]
    assert manifest["sensor_runtime"]["stop_on_nav_goal_reached"] is True
    assert manifest["sensor_runtime"]["mid360_samples_per_frame"] == 10000
    assert manifest["sensor_runtime"]["imu_hz"] == 200.0
    assert manifest["sensor_runtime"]["scan_time_profile"] == "physical_rolling"
    assert manifest["runtime_tolerances"]["sim_hardware_realtime_factor"] == 0.5
    assert manifest["sensor_runtime"]["physics_integrator"] == "euler"
    assert manifest["sensor_runtime"]["physical_rolling_sample_mode"] == "subscan"
    assert manifest["telemetry_log"]["hz"] == 10.0
    assert manifest["telemetry_log"]["lidar_points"] == 640
    assert manifest["slam_runtime"]["provider"] == "fastlio2"
    assert manifest["navigation_runtime"]["status_period_s"] == 0.5
    assert "debug_candidate_limit" not in manifest["navigation_runtime"]
    assert "debug_local_map_points" not in manifest["navigation_runtime"]
    assert manifest["phases"]["motion"]["duration_s"] == 600.0
    assert manifest["thresholds"]["startup_timeout_s"] == 120.0
    assert manifest["thresholds"]["startup_ready_stable_s"] == 3.0
    assert manifest["thresholds"]["min_input_gate_ready_fraction"] == 0.98
    assert manifest["thresholds"]["max_odom_tf_rejections"] == 5
    assert manifest["thresholds"]["max_cloud_pose_rejections"] == 5
    assert manifest["thresholds"]["max_consecutive_input_stale_s"] == 1.0
    assert manifest["thresholds"]["max_navigation_loop_overrun_p99_ms"] == 100.0
    assert manifest["thresholds"]["max_navigation_loop_overrun_peak_ms"] == 250.0
    assert "max_navigation_loop_overrun_ms" not in manifest["thresholds"]
    assert "max_sim_hardware_lag_observed_s" not in manifest["thresholds"]
    assert "max_sim_hardware_consecutive_catch_up_steps" not in manifest["thresholds"]
    assert manifest["thresholds"]["require_slam_tracking"] is True
    assert manifest["thresholds"]["require_slam_accuracy"] is True
    assert manifest["thresholds"]["require_navigation_safety_contracts"] is True
    assert manifest["thresholds"]["require_video_artifact"] is False
    assert acceptance.RUNTIME_EVIDENCE_SAMPLE_PERIOD_S == 0.20
    assert acceptance.SENSOR_PUBLISHER_PROBE_TIMEOUT_S == 60.0
    sensor_args = acceptance._sensor_runtime_args(manifest)
    assert "--stop-on-nav-goal-reached" in sensor_args
    assert "--publish-odom-prior" in sensor_args
    assert manifest["slam_runtime"]["provider"] == "mujoco_navigation_fixture"
    assert manifest["navigation_runtime"]["local_planner"] == "scan"
    assert manifest["navigation_runtime"]["use_traversability_cost"] is False
    assert manifest["thresholds"]["require_traversability"] is False
    assert manifest["thresholds"]["require_goal_reached"] is True
    assert manifest["thresholds"]["min_tracking_samples"] == 50
    assert manifest["acceptance_video"]["enabled"] is True
    assert "--odom-prior-velocity-window-s" not in sensor_args
    assert sensor_args[sensor_args.index("--scan-time-profile") + 1] == "physical_rolling"
    assert "--navigation-fixture-cloud-points" not in sensor_args
    assert sensor_args[sensor_args.index("--imu-hz") + 1] == "200.0"
    assert sensor_args[sensor_args.index("--physical-rolling-sample-mode") + 1] == "subscan"
    assert sensor_args[sensor_args.index("--physics-integrator") + 1] == "euler"
    source = (ROOT / "sim" / "scripts" / "mujoco" / "native_navigation_acceptance.py").read_text(encoding="utf-8")
    assert '"--corridor-lookahead-m"' in source
    endpoint_source = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "main.cpp"
    ).read_text(encoding="utf-8")
    config_source = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "config" / "build.cpp"
    ).read_text(encoding="utf-8")
    assert "buildNavLoopConfig(cfg, safety_config.obstacle_margin_m)" in endpoint_source
    assert "out.local_planner.footprintPadding = obstacle_margin_m;" in config_source
    assert endpoint_source.count("const auto safety_config = commandSafetyConfig(cfg);") == 1


def test_video_capture_uses_live_telemetry_rate_instead_of_encoder_fps():
    settings = acceptance._motion_capture_settings(
        record_video=True,
        record_telemetry=True,
        video_cfg={"fps": 24.0, "lidar_points": 640},
        telemetry_cfg={"hz": 10.0, "lidar_points": 320},
    )

    assert settings == (10.0, 640)


def test_video_only_capture_defaults_to_ten_hz_live_sampling():
    settings = acceptance._motion_capture_settings(
        record_video=True,
        record_telemetry=False,
        video_cfg={"fps": 30.0, "lidar_points": 512},
        telemetry_cfg={},
    )

    assert settings == (10.0, 512)


def test_mujoco_engine_exposes_explicit_integrator_override(monkeypatch):
    integrators = SimpleNamespace(
        mjINT_EULER=10,
        mjINT_RK4=11,
        mjINT_IMPLICIT=12,
        mjINT_IMPLICITFAST=13,
    )
    monkeypatch.setitem(sys.modules, "mujoco", SimpleNamespace(mjtIntegrator=integrators))
    engine = object.__new__(MuJoCoEngine)
    engine._model = SimpleNamespace(
        opt=SimpleNamespace(integrator=integrators.mjINT_RK4, timestep=0.001)
    )
    engine._physics_dt = 0.001
    engine._sensor_tick_residual_s = 0.0004

    assert engine.physics_integrator == "rk4"
    assert engine.set_physics_integrator("euler") == "euler"
    assert engine._model.opt.integrator == integrators.mjINT_EULER
    with pytest.raises(ValueError, match="unsupported MuJoCo integrator"):
        engine.set_physics_integrator("bogus")

    assert engine.set_physics_timestep(0.002) == pytest.approx(0.002)
    assert engine._model.opt.timestep == pytest.approx(0.002)
    assert engine._sensor_tick_residual_s == 0.0
    for invalid in (0.0, 0.00049, 0.00501, float("nan")):
        with pytest.raises(
            ValueError,
            match="physics timestep must be in",
        ):
            engine.set_physics_timestep(invalid)


def test_required_video_artifact_blocks_when_not_requested_or_rendering_fails():
    assert (
        acceptance._video_artifact_blocker(
            required=True,
            video_report={"requested": False, "ok": False, "reason": "not_requested"},
        )
        == "native_navigation_video_failed"
    )
    assert (
        acceptance._video_artifact_blocker(
            required=True,
            video_report={"requested": True, "ok": False, "reason": "render_failed"},
        )
        == "native_navigation_video_failed"
    )


def test_video_artifact_gate_accepts_success_and_optional_runs():
    assert (
        acceptance._video_artifact_blocker(
            required=True,
            video_report={
                "requested": True,
                "ok": True,
                "candidate_frames": 12,
                "selected_candidate_frames": 12,
                "local_map_frames": 12,
                "visible_local_map_frames": 8,
                "exact_planner_join_frames": 12,
                "presentation_lighting": {"brightness_ok": True},
            },
        )
        is None
    )
    assert (
        acceptance._video_artifact_blocker(
            required=False,
            video_report={"requested": False, "ok": False},
        )
        is None
    )
    assert (
        acceptance._video_artifact_blocker(
            required=True,
            require_candidates=False,
            video_report={
                "requested": True,
                "ok": True,
                "candidate_frames": 0,
                "selected_candidate_frames": 0,
                "local_map_frames": 12,
                "visible_local_map_frames": 8,
                "exact_planner_join_frames": 12,
                "presentation_lighting": {"brightness_ok": True},
            },
        )
        is None
    )


def test_required_video_artifact_blocks_missing_planner_presentation_evidence():
    base = {
        "requested": True,
        "ok": True,
        "candidate_frames": 12,
        "selected_candidate_frames": 12,
        "local_map_frames": 12,
        "visible_local_map_frames": 8,
        "exact_planner_join_frames": 12,
        "presentation_lighting": {"brightness_ok": True},
    }

    assert acceptance._video_artifact_blocker(
        required=True,
        video_report={**base, "candidate_frames": 0},
    ) == "native_navigation_video_candidates_missing"
    assert acceptance._video_artifact_blocker(
        required=True,
        video_report={**base, "selected_candidate_frames": 0},
    ) == "native_navigation_video_selected_path_missing"
    assert acceptance._video_artifact_blocker(
        required=True,
        video_report={**base, "local_map_frames": 0},
    ) == "native_navigation_video_local_map_missing"
    assert acceptance._video_artifact_blocker(
        required=True,
        video_report={**base, "visible_local_map_frames": 0},
    ) == "native_navigation_video_local_map_not_visible"
    assert acceptance._video_artifact_blocker(
        required=True,
        video_report={**base, "exact_planner_join_frames": 0},
    ) == "native_navigation_video_exact_join_missing"
    assert acceptance._video_artifact_blocker(
        required=True,
        video_report={
            **base,
            "presentation_lighting": {"brightness_ok": False},
        },
    ) is None


def test_video_decode_validation_rejects_ffmpeg_decoder_errors(monkeypatch, tmp_path):
    video_path = tmp_path / "navigation.mp4"
    video_path.write_bytes(b"not-a-real-video")
    monkeypatch.setattr(navigation_video.shutil, "which", lambda _name: "ffmpeg")
    monkeypatch.setattr(
        navigation_video.subprocess,
        "run",
        lambda *_args, **_kwargs: SimpleNamespace(
            returncode=1,
            stderr="Invalid NAL unit size",
        ),
    )

    result = navigation_video._validate_video_decode(video_path)

    assert result["ok"] is False
    assert result["method"] == "ffmpeg_full_decode"
    assert "Invalid NAL unit size" in result["error"]


def test_navigation_fixture_readiness_uses_native_nav_odom_without_slam_status():
    readiness = acceptance._slam_navigation_readiness(
        slam={},
        nav={"has_odom": True, "counters": {"odom": 5}},
        thresholds={"navigation_state_provider": "mujoco_navigation_fixture"},
    )

    assert readiness == "mujoco_navigation_fixture"


def _complete_startup_dependencies(evidence, tmp_path: Path) -> Path:
    evidence.last_nav["control_loop_health"] = {"ready": True, "healthy": True}
    evidence.last_nav["local_map"] = {"collision": {"generation": 1}}
    status = tmp_path / "mapd-status.json"
    status.write_text(
        json.dumps(
            {
                "ready": True,
                "live": True,
                "accepted_observations": 1,
                "map_layers_published_generation": 1,
            }
        ),
        encoding="utf-8",
    )
    return status


def test_navigation_fixture_startup_waits_for_complete_native_input_gate(tmp_path):
    sensor = SimpleNamespace(poll=lambda: None)
    evidence = acceptance.NativeEvidence(
        last_nav={
            "has_odom": True,
            "counters": {"odom": 5},
            "input_gate": {"ready": False},
        },
        last_traversability={"counters": {"published": 5}},
    )
    paths = {
        "nav_status": tmp_path / "missing-nav.json",
        "slam_status": tmp_path / "missing-slam.json",
        "traversability_status": tmp_path / "missing-traversability.json",
    }
    mapd_status = _complete_startup_dependencies(evidence, tmp_path)

    ok, _ = acceptance._wait_for_startup(
        sensor=sensor,
        evidence=evidence,
        nav_status=paths["nav_status"],
        slam_status=paths["slam_status"],
        traversability_status=paths["traversability_status"],
        mapd_status=mapd_status,
        timeout_s=0.01,
        thresholds={"navigation_state_provider": "mujoco_navigation_fixture"},
    )
    assert not ok

    evidence.last_nav["input_gate"]["ready"] = True
    ok, reason = acceptance._wait_for_startup(
        sensor=sensor,
        evidence=evidence,
        nav_status=paths["nav_status"],
        slam_status=paths["slam_status"],
        traversability_status=paths["traversability_status"],
        mapd_status=mapd_status,
        timeout_s=0.1,
        thresholds={"navigation_state_provider": "mujoco_navigation_fixture"},
    )
    assert ok
    assert reason == "ready_mujoco_navigation_fixture"


def test_navigation_fixture_startup_requires_configured_stable_ready_window(
    monkeypatch,
    tmp_path,
):
    now = [10.0]
    monkeypatch.setattr(acceptance.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(
        acceptance.time,
        "sleep",
        lambda duration: now.__setitem__(0, now[0] + float(duration)),
    )
    evidence = acceptance.NativeEvidence(
        last_nav={
            "has_odom": True,
            "counters": {"odom": 5},
            "input_gate": {"ready": True},
        },
        last_traversability={"counters": {"published": 5}},
    )
    mapd_status = _complete_startup_dependencies(evidence, tmp_path)

    ok, reason = acceptance._wait_for_startup(
        sensor=SimpleNamespace(poll=lambda: None),
        evidence=evidence,
        nav_status=tmp_path / "missing-nav.json",
        slam_status=tmp_path / "missing-slam.json",
        traversability_status=tmp_path / "missing-traversability.json",
        mapd_status=mapd_status,
        timeout_s=1.0,
        thresholds={
            "navigation_state_provider": "mujoco_navigation_fixture",
            "startup_ready_stable_s": 0.25,
        },
    )

    assert ok
    assert reason == "ready_mujoco_navigation_fixture"
    assert now[0] >= 10.25


def test_navigation_fixture_startup_stable_window_resets_after_ready_dropout(
    monkeypatch,
    tmp_path,
):
    now = [10.0]
    monkeypatch.setattr(acceptance.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(
        acceptance.time,
        "sleep",
        lambda duration: now.__setitem__(0, now[0] + float(duration)),
    )
    ready_sequence = iter((True, True, False, True, True, True, True))
    evidence = SimpleNamespace(
        last_slam={},
        last_nav={
            "has_odom": True,
            "counters": {"odom": 5},
            "input_gate": {"ready": True},
        },
        last_traversability={"counters": {"published": 5}},
    )
    mapd_status = _complete_startup_dependencies(evidence, tmp_path)

    def sample(**_):
        evidence.last_nav["input_gate"]["ready"] = next(ready_sequence, True)

    evidence.sample = sample

    ok, reason = acceptance._wait_for_startup(
        sensor=SimpleNamespace(poll=lambda: None),
        evidence=evidence,
        nav_status=tmp_path / "missing-nav.json",
        slam_status=tmp_path / "missing-slam.json",
        traversability_status=tmp_path / "missing-traversability.json",
        mapd_status=mapd_status,
        timeout_s=1.0,
        thresholds={
            "navigation_state_provider": "mujoco_navigation_fixture",
            "startup_ready_stable_s": 0.25,
        },
    )

    assert ok
    assert reason == "ready_mujoco_navigation_fixture"
    assert now[0] == pytest.approx(10.6)


def test_manifest_inheritance_merges_nested_runtime_sections(tmp_path):
    parent = tmp_path / "parent.json"
    child = tmp_path / "child.json"
    parent.write_text(
        json.dumps({"phases": {"motion": {"duration_s": 10, "publish_cmd_vel": True}}}),
        encoding="utf-8",
    )
    child.write_text(
        json.dumps({"extends": "parent.json", "phases": {"motion": {"duration_s": 90}}}),
        encoding="utf-8",
    )

    manifest = acceptance._load_manifest(child)

    assert manifest["phases"]["motion"] == {"duration_s": 90, "publish_cmd_vel": True}


def test_native_acceptance_rejects_unknown_planner_constraint():
    try:
        acceptance._planner_constraint_args({"planner_constraints": {"invented": 1}})
    except ValueError as exc:
        assert "invented" in str(exc)
    else:
        raise AssertionError("unknown planner constraint must fail closed")


def test_native_acceptance_rejects_unknown_sensor_runtime_option():
    try:
        acceptance._sensor_runtime_args({"sensor_runtime": {"invented": 1}})
    except ValueError as exc:
        assert "invented" in str(exc)
    else:
        raise AssertionError("unknown sensor runtime option must fail closed")


def test_navigation_sensor_assessment_keeps_transport_failures_blocking_but_not_slam_accuracy():
    accuracy_only = acceptance._navigation_sensor_assessment(
        {
            "ok": False,
            "remaining_gaps": [
                "native_slam_not_tracking:DEGRADED",
                "native_slam_quality_low:0.100",
                "native_slam_motion_mismatch:sim_xy=60.0,slam_xy=6.0",
                "native_slam_yaw_mismatch:sim_yaw=1.0,slam_yaw=0.0",
            ],
        },
        {"require_slam_accuracy": False},
    )

    assert accuracy_only["navigation_critical_ok"] is True
    assert accuracy_only["blocking_gaps"] == []
    assert len(accuracy_only["non_blocking_slam_accuracy_gaps"]) == 4

    missing_transport = acceptance._navigation_sensor_assessment(
        {
            "ok": False,
            "remaining_gaps": [
                "native_slam_motion_mismatch:sim_xy=60.0,slam_xy=6.0",
                "native_slam_output_missing:/slam/odometry",
            ],
        },
        {"require_slam_accuracy": False},
    )

    assert missing_transport["navigation_critical_ok"] is False
    assert missing_transport["blocking_gaps"] == ["native_slam_output_missing:/slam/odometry"]


def test_sensor_runtime_options_are_attached_only_to_sensor_process():
    source = (ROOT / "sim/scripts/mujoco/native_navigation_acceptance.py").read_text(encoding="utf-8")
    assert source.count("_sensor_runtime_args(manifest)") == 1
    assert "sensor_args.extend(_sensor_runtime_args(manifest))" in source
    assert source.count("_parent_sensor_diagnostics_args(") == 2
    assert "parent_sensor_diagnostics_path = phase_dir / \"parent_sensor_diagnostics.json\"" in source
    assert "parent_diagnostics_json" not in source


def test_native_endpoint_keeps_lidar_extrinsic_out_of_body_pose_local_planning():
    endpoint_source = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "main.cpp"
    ).read_text(encoding="utf-8")
    input_source = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "input" / "map.cpp"
    ).read_text(encoding="utf-8")
    config_source = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "config" / "build.cpp"
    ).read_text(encoding="utf-8")

    assert "sensorOriginFromBody" in input_source
    assert "inputs_config.sensor_offset" in endpoint_source
    assert "cfg.sensor_offset_x_m" in endpoint_source
    assert "cfg.sensor_offset_y_m" in endpoint_source
    assert "out.local_planner.sensorOffsetX = 0.0;" in config_source
    assert "out.local_planner.sensorOffsetY = 0.0;" in config_source
    assert (
        "out.local_planner.sensorOffsetX = cfg.sensor_offset_x_m;"
        not in config_source
    )
    assert (
        "out.local_planner.sensorOffsetY = cfg.sensor_offset_y_m;"
        not in config_source
    )


def test_cmd_vel_direction_diagnostics_use_body_forward_sign():
    assert sensors._linear_command_direction(0.4, 0.0) == "forward"
    assert sensors._linear_command_direction(-0.4, 0.0) == "reverse"
    assert sensors._linear_command_direction(0.0, 0.4) == "lateral"
    assert sensors._linear_command_direction(0.0, 0.0) == "idle"


def test_motion_phase_rejects_reverse_dominated_control_even_when_it_moves():
    evidence = acceptance.NativeEvidence(
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=8,
        max_local_path_points=8,
        max_cmd_vel_published=20,
        max_traversability_published=20,
        max_registered_clouds=20,
        max_path_follower_cmd_norm=0.4,
        max_computed_cmd_norm=0.4,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
    )
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "cmd_vel": {
            "nonzero_samples": 20,
            "forward_linear_samples": 1,
            "reverse_linear_samples": 19,
            "lateral_linear_samples": 0,
        },
        "motion": {
            "sim_start_xyz": [0.0, 0.0, 0.48],
            "sim_end_xyz": [1.0, 0.0, 0.48],
            "sim_path_length_xy_m": 1.0,
        },
    }

    _, blockers, metrics = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": True},
        thresholds={
            "min_forward_linear_command_fraction": 0.8,
            "max_reverse_linear_command_fraction": 0.1,
        },
        evidence=evidence,
        sensor_report=sensor_report,
        goal=[2.0, 0.0, 0.3, 0.0],
    )

    assert metrics["command_direction"]["forward_linear_fraction"] == pytest.approx(0.05)
    assert metrics["command_direction"]["reverse_linear_fraction"] == pytest.approx(0.95)
    assert "forward_linear_command_fraction_below_threshold" in blockers
    assert "reverse_linear_command_fraction_above_threshold" in blockers


def test_goal_command_timeout_covers_multifloor_plan(monkeypatch, tmp_path):
    captured = {}

    class Result:
        returncode = 0
        stdout = "accepted"
        stderr = ""

    def fake_run(command, **kwargs):
        captured["command"] = command
        captured["timeout"] = kwargs["timeout"]
        return Result()

    monkeypatch.setattr(acceptance.subprocess, "run", fake_run)
    monkeypatch.setattr(
        acceptance,
        "_native_command",
        lambda binary, *args: [str(binary), *args],
    )

    result = acceptance._goal_command(
        tmp_path / "lingtu_nav_control",
        [7.4, 2.8, 4.15, 0.0],
        220,
        timeout_s=50.0,
    )

    assert result["returncode"] == 0
    option = captured["command"].index("--timeout-ms")
    assert captured["command"][option + 1] == "50000"
    assert captured["timeout"] == 55.0


def test_deferred_wsl_goal_command_prestarts_wsl_relay_before_trigger(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setattr(acceptance.os, "name", "nt")
    monkeypatch.setattr(
        acceptance,
        "_native_command",
        lambda binary, *args: ["wsl.exe", "-e", "/tmp/lingtu_nav_control", *args],
    )

    trigger = tmp_path / "goal_control.trigger"
    command = acceptance._deferred_goal_command(
        tmp_path / "lingtu_nav_control",
        [60.0, 0.0, 0.0, 0.0],
        229,
        timeout_s=20.0,
        trigger_path=trigger,
    )

    assert command[:4] == ["wsl.exe", "-e", "bash", "-lc"]
    assert 'while [ ! -f "$trigger" ]' in command[4]
    assert command[6] == acceptance._wsl_path(trigger)
    assert command[7:10] == ["/tmp/lingtu_nav_control", "goal", "60.0"]
    assert command[-4:] == ["--domain-id", "229", "--timeout-ms", "20000"]


def test_multifloor_manifest_locks_runtime_octoplanner3d_constraints():
    manifest = json.loads(
        (ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_multifloor_navigation_acceptance.json").read_text(
            encoding="utf-8"
        )
    )

    assert manifest["asset_builder"]["scene_preset"] == "multifloor_stack_3"
    assert manifest["asset_builder"]["resolution"] == 0.09
    assert manifest["start"][:3] == [7.0, -1.2, 0.55]
    assert manifest["goal"][:3] == [7.4, 2.8, 4.15]
    constraints = manifest["planner_constraints"]
    assert constraints["body_clearance_below_m"] == 0.15
    assert constraints["body_clearance_above_m"] == 0.30
    assert constraints["support_height_m"] == 0.55
    assert constraints["max_step_height_m"] == 0.23
    assert constraints["max_slope"] == 0.65
    args = acceptance._planner_constraint_args(manifest)
    assert args[args.index("--octo-max-step-height-m") + 1] == "0.23"
    assert args[args.index("--octo-support-height-m") + 1] == "0.55"


def test_acceptance_prepares_same_source_assets_when_manifest_paths_are_empty(tmp_path, monkeypatch):
    scene_xml = tmp_path / "generated" / "scene.xml"
    map_dir = tmp_path / "generated" / "same_source_map"
    map_dir.mkdir(parents=True)
    scene_xml.write_text("<mujoco/>", encoding="utf-8")
    builder_report = {
        "scene_xml": str(scene_xml),
        "map_dir": str(map_dir),
        "build": {"ok": True},
        "artifact_gate": {"ok": True},
    }
    monkeypatch.delenv("LINGTU_MUJOCO_MAP_DIR", raising=False)
    monkeypatch.setattr(
        acceptance,
        "_run_saved_map_asset_builder",
        lambda _spec, _out_dir: builder_report,
    )
    manifest = {
        "world": "",
        "map_dir": "",
        "asset_builder": {"kind": "saved_map_plan_gate"},
    }

    result = acceptance._prepare_acceptance_assets(manifest, tmp_path / "run")

    assert result["ok"] is True
    assert result["attempted"] is True
    assert Path(manifest["world"]) == scene_xml
    assert Path(manifest["map_dir"]) == map_dir

def test_acceptance_preserves_asset_builder_failure_reason(tmp_path, monkeypatch):
    scene_xml = tmp_path / "generated" / "scene.xml"
    map_dir = tmp_path / "generated" / "same_source_map"
    map_dir.mkdir(parents=True)
    scene_xml.write_text("<mujoco/>", encoding="utf-8")
    monkeypatch.setattr(
        acceptance,
        "_run_saved_map_asset_builder",
        lambda _spec, _out_dir: {
            "scene_xml": str(scene_xml),
            "map_dir": str(map_dir),
            "build": {
                "ok": False,
                "octomap_result": {
                    "success": False,
                    "reason_code": "missing_converter",
                },
            },
            "artifact_gate": {"ok": False, "blockers": ["missing_octomap"]},
        },
    )
    manifest = {
        "world": "",
        "map_dir": "",
        "asset_builder": {"kind": "saved_map_plan_gate"},
    }

    result = acceptance._prepare_acceptance_assets(manifest, tmp_path / "run")

    assert result["ok"] is False
    assert result["reason"] == "asset_builder_incomplete"
    assert result["blockers"] == [
        "asset_octomap_build_failed:missing_converter",
        "asset_artifact_gate_failed",
    ]
    assert result["build"]["octomap_result"]["reason_code"] == "missing_converter"
    assert result["artifact_gate"]["blockers"] == ["missing_octomap"]



def test_preflight_validates_map_formats_and_metadata(tmp_path, monkeypatch):
    map_dir = tmp_path / "map"
    map_dir.mkdir()
    map_pcd = map_dir / "map.pcd"
    octomap = map_dir / "octomap.ot"
    map_pcd.write_bytes(b"pcd")
    octomap.write_bytes(b"octomap")
    (map_dir / "metadata.json").write_text(
        json.dumps(
            {
                "artifacts": {
                    "map_pcd": {"path": "map.pcd", "frame_id": "map"},
                    "octomap": {
                        "path": "octomap.ot",
                        "frame_id": "map",
                    },
                },
                "frame_id": "map",
                "schema_version": "lingtu.saved_map_artifacts.v1",
            }
        ),
        encoding="utf-8",
    )

    fake_bin = tmp_path / "native"
    fake_bin.write_bytes(b"native")
    fake_config = tmp_path / "slam.yaml"
    fake_config.write_text("backend: fastlio2\n", encoding="utf-8")
    fake_paths = tmp_path / "paths"
    fake_paths.mkdir()
    fake_sensor = tmp_path / "sensor.py"
    fake_sensor.write_text("pass\n", encoding="utf-8")
    fake_policy = tmp_path / "history_285.onnx"
    fake_policy.write_bytes(b"onnx")
    world = tmp_path / "world.xml"
    world.write_text("<mujoco/>", encoding="utf-8")
    manifest = {
        "world": str(world),
        "map_dir": str(map_dir),
        "map_files": {"slam": "map.pcd", "planner": "octomap.ot", "metadata": "metadata.json"},
        "source_identity": {
            "map_artifacts": {
                "metadata_schema": "lingtu.saved_map_artifacts.v1",
            }
        },
        "paths": {
            "slam_config": str(fake_config),
            "path_library": str(fake_paths),
            "sensor_runner": str(fake_sensor),
            "policy": str(fake_policy),
        },
        "binaries": {
            name: {"candidates": [str(fake_bin)]}
            for name in (
                "sensor_publisher",
                "slam",
                "traversability",
                "navigation",
                "navigation_control",
                "driver_bridge",
            )
        },
    }
    monkeypatch.delenv("LINGTU_MUJOCO_MAP_DIR", raising=False)
    monkeypatch.setattr(acceptance, "_probe_sensor_publisher", lambda _binary: (True, "dds_enabled"))

    binaries, paths, blockers, provenance = acceptance._preflight(manifest)

    assert not blockers
    assert set(binaries) == set(manifest["binaries"])
    assert paths["slam"] == map_pcd
    assert paths["policy"] == fake_policy
    assert provenance["map_format_ok"] is True
    assert provenance["planner_format_ok"] is True
    assert provenance["driver_bridge"] == {
        "path": str(fake_bin),
    }

    metadata = json.loads((map_dir / "metadata.json").read_text(encoding="utf-8"))
    metadata["artifacts"]["octomap"]["frame_id"] = "odom"
    (map_dir / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    _, _, blockers, _ = acceptance._preflight(manifest)

    assert "map_artifact_metadata_inconsistent" in blockers

def test_phase_evaluator_proves_local_path_is_telemetry_and_follower_is_embedded():
    evidence = acceptance.NativeEvidence(
        samples=5,
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=8,
        max_local_path_points=40,
        max_cmd_vel_published=12,
        max_traversability_published=20,
        max_registered_clouds=20,
        max_path_follower_cmd_norm=0.4,
        max_computed_cmd_norm=0.4,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
        last_slam={"state": "TRACKING", "track_against_map": {"successes": 1}},
    )
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "cmd_vel": {"nonzero_samples": 12},
        "motion": {
            "sim_start_xyz": [0.0, 0.0, 0.48],
            "sim_end_xyz": [0.5, 0.0, 0.48],
            "sim_path_length_xy_m": 0.5,
        },
    }

    ok, blockers, metrics = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": True},
        thresholds={
            "min_global_path_points": 2,
            "min_local_path_points": 2,
            "min_cmd_vel_samples": 3,
            "min_motion_m": 0.15,
            "min_goal_distance_reduction_m": 0.1,
        },
        evidence=evidence,
        sensor_report=sensor_report,
        goal=[1.0, 0.0, 0.3, 0.0],
    )

    assert ok
    assert not blockers
    assert metrics["distance_reduction_m"] == 0.5


def test_scan_baseline_does_not_require_traversability_process():
    evidence = acceptance.NativeEvidence(
        samples=5,
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=8,
        max_local_path_points=40,
        max_cmd_vel_published=12,
        max_path_follower_cmd_norm=0.4,
        max_computed_cmd_norm=0.4,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
        last_slam={"state": "TRACKING", "track_against_map": {"successes": 1}},
        last_nav={"has_odom": True, "counters": {"odom": 5}},
    )
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "cmd_vel": {"nonzero_samples": 12},
        "motion": {
            "sim_start_xyz": [0.0, 0.0, 0.48],
            "sim_end_xyz": [-0.5, 0.0, 0.48],
            "sim_path_length_xy_m": 0.5,
        },
    }

    ok, blockers, _ = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": True},
        thresholds={
            "require_traversability": False,
            "navigation_state_provider": "mujoco_navigation_fixture",
            "max_motion_slam_map_xy_error_m": 0.2,
        },
        evidence=evidence,
        sensor_report=sensor_report,
        goal=[1.0, 0.0, 0.48, 0.0],
    )

    assert ok, blockers
    assert "motion_slam_map_pose_error_too_large" not in blockers
    assert "goal_distance_did_not_decrease" not in blockers
    assert "registered_cloud_not_consumed_by_traversability" not in blockers
    assert "traversability_not_published" not in blockers


def test_multifloor_goal_metrics_report_xy_z_and_3d_error():
    metrics = acceptance._goal_metrics(
        {
            "motion": {
                "sim_start_xyz": [0.0, 0.0, 0.5],
                "sim_end_xyz": [3.0, 4.0, 2.5],
                "sim_path_length_xy_m": 6.0,
            }
        },
        [3.0, 4.0, 3.0, 0.0],
    )

    assert metrics["end_error_xy_m"] == pytest.approx(0.0)
    assert metrics["end_error_z_m"] == pytest.approx(0.5)
    assert metrics["end_error_3d_m"] == pytest.approx(0.5)


def test_truth_tracking_metrics_project_each_pose_to_current_local_path(tmp_path):
    motion_log = tmp_path / "motion.jsonl"
    rows = [
        {
            "sim_time_s": 10.0,
            "driving": True,
            "x": 0.5,
            "y": 0.2,
            "z": 0.0,
            "local_path": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        },
        {
            "sim_time_s": 11.0,
            "driving": True,
            "x": 1.5,
            "y": 0.4,
            "z": 0.3,
            "local_path": [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        },
    ]
    motion_log.write_text(
        "".join(json.dumps(row) + "\n" for row in rows),
        encoding="utf-8",
    )

    metrics = acceptance._trajectory_tracking_metrics(motion_log)

    assert metrics["available"] is True
    assert metrics["samples"] == 2
    assert metrics["elapsed_s"] == pytest.approx(1.0)
    assert metrics["max_xy_error_m"] == pytest.approx(0.4)
    assert metrics["max_3d_error_m"] == pytest.approx(0.5)


def test_motion_strict_arrival_rejects_progress_without_final_goal_arrival():
    evidence = acceptance.NativeEvidence(
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=120,
        max_local_path_points=40,
        max_cmd_vel_published=500,
        max_traversability_published=500,
        max_registered_clouds=500,
        max_path_follower_cmd_norm=0.4,
        max_computed_cmd_norm=0.4,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
        last_nav={"last_local": {"goal_reached": False}},
        last_slam={"state": "TRACKING", "track_against_map": {"successes": 10}},
    )
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "cmd_vel": {"nonzero_samples": 500},
        "motion": {
            "sim_start_xyz": [3.0, 4.0, 0.48],
            "sim_end_xyz": [50.0, 28.0, 0.48],
            "sim_path_length_xy_m": 53.0,
        },
    }

    ok, blockers, metrics = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": True},
        thresholds={
            "require_goal_reached": True,
            "max_goal_error_m": 0.5,
        },
        evidence=evidence,
        sensor_report=sensor_report,
        goal=[56.0, 32.0, 0.3, 0.0],
    )

    assert not ok
    assert "native_goal_not_reached" in blockers
    assert "final_goal_error_above_threshold" in blockers
    assert metrics["end_distance_m"] > 0.5


def test_motion_rejects_long_path_without_long_net_displacement():
    evidence = acceptance.NativeEvidence(
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=300,
        max_local_path_points=40,
        max_cmd_vel_published=500,
        max_traversability_published=500,
        max_registered_clouds=500,
        max_path_follower_cmd_norm=0.4,
        max_computed_cmd_norm=0.4,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
        goal_reached_observed=True,
        last_slam={"state": "TRACKING", "track_against_map": {"successes": 10}},
    )
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "cmd_vel": {"nonzero_samples": 500},
        "motion": {
            "sim_start_xyz": [3.0, 4.0, 0.48],
            "sim_end_xyz": [8.0, 4.0, 0.48],
            "sim_path_length_xy_m": 55.0,
        },
    }

    ok, blockers, metrics = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": True},
        thresholds={
            "min_motion_m": 50.0,
            "min_net_displacement_m": 50.0,
        },
        evidence=evidence,
        sensor_report=sensor_report,
        goal=[56.0, 32.0, 0.3, 0.0],
    )

    assert not ok
    assert metrics["sim_path_length_xy_m"] == 55.0
    assert metrics["net_displacement_xy_m"] == 5.0
    assert "net_displacement_below_threshold" in blockers


def test_motion_rejects_host_realtime_instability_even_if_navigation_arrives():
    evidence = acceptance.NativeEvidence(
        samples=100,
        input_gate_ready_samples=50,
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=120,
        max_local_path_points=40,
        max_cmd_vel_published=500,
        max_traversability_published=500,
        max_registered_clouds=500,
        max_path_follower_cmd_norm=0.4,
        max_computed_cmd_norm=0.4,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
        goal_reached_observed=True,
        last_slam={"state": "TRACKING", "track_against_map": {"successes": 10}},
    )
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "cmd_vel": {"nonzero_samples": 500},
        "sim_hardware_pacing": {
            "max_lag_observed_s": 9.6,
            "max_consecutive_steps": 4020,
        },
        "motion": {
            "sim_start_xyz": [3.0, 4.0, 0.48],
            "sim_end_xyz": [56.0, 32.0, 0.48],
            "sim_path_length_xy_m": 70.0,
        },
    }

    ok, blockers, metrics = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": True},
        thresholds={
            "require_goal_reached": True,
            "max_goal_error_m": 0.5,
            "min_input_gate_ready_fraction": 0.65,
            "max_sim_hardware_lag_observed_s": 2.0,
            "max_sim_hardware_consecutive_catch_up_steps": 1000,
        },
        evidence=evidence,
        sensor_report=sensor_report,
        goal=[56.0, 32.0, 0.3, 0.0],
    )

    assert not ok
    assert metrics["input_gate_ready_fraction"] == 0.5
    assert "input_gate_ready_fraction_below_threshold" in blockers
    assert "sim_hardware_lag_above_threshold" in blockers
    assert "sim_hardware_catch_up_streak_above_threshold" in blockers


def test_native_evidence_tracks_input_sync_rejections_stale_streak_and_loop_overrun(tmp_path):
    nav_status = tmp_path / "nav.json"
    missing_slam = tmp_path / "missing-slam.json"
    missing_traversability = tmp_path / "missing-traversability.json"
    evidence = acceptance.NativeEvidence()

    def sample(*, ready, odom_rejected, cloud_pose_rejected, overrun_ms):
        nav_status.write_text(
            json.dumps(
                    {
                        "input_gate": {"ready": ready},
                        "command_boundary": {"last_accepted": True},
                        "frame_gate": {"odom_rejected": odom_rejected},
                    "cloud_sync": {"pose_rejected": cloud_pose_rejected},
                    "timing_ms": {"overrun": overrun_ms},
                }
            ),
            encoding="utf-8",
        )
        evidence.sample(
            nav_path=nav_status,
            slam_path=missing_slam,
            traversability_path=missing_traversability,
        )

    sample(ready=False, odom_rejected=1, cloud_pose_rejected=1, overrun_ms=2.0)
    sample(ready=True, odom_rejected=2, cloud_pose_rejected=1, overrun_ms=5.0)
    sample(ready=False, odom_rejected=7, cloud_pose_rejected=3, overrun_ms=40.0)
    sample(ready=False, odom_rejected=9, cloud_pose_rejected=6, overrun_ms=110.0)
    sample(ready=True, odom_rejected=9, cloud_pose_rejected=6, overrun_ms=0.0)

    assert evidence.max_odom_tf_rejections == 9
    assert evidence.max_cloud_pose_rejections == 6
    assert evidence.max_consecutive_input_stale_s == pytest.approx(0.4)
    assert evidence.max_navigation_loop_overrun_ms == pytest.approx(110.0)
    assert evidence.to_dict()["max_consecutive_input_stale_s"] == pytest.approx(0.4)


def test_native_evidence_excludes_startup_samples_without_explicit_input_gate(tmp_path):
    nav_status = tmp_path / "nav.json"
    missing_slam = tmp_path / "missing-slam.json"
    missing_traversability = tmp_path / "missing-traversability.json"
    evidence = acceptance.NativeEvidence()

    nav_status.write_text(
        json.dumps(
            {
                "frame_gate": {"odom_rejected": 90},
                "cloud_sync": {"pose_rejected": 80},
                "timing_ms": {"overrun": 700.0},
            }
        ),
        encoding="utf-8",
    )
    evidence.sample(
        nav_path=nav_status,
        slam_path=missing_slam,
        traversability_path=missing_traversability,
    )

    nav_status.write_text(
        json.dumps(
                {
                    "input_gate": {"ready": True, "reason": "ready"},
                    "command_boundary": {"last_accepted": True},
                    "frame_gate": {"odom_rejected": 1},
                "cloud_sync": {"pose_rejected": 2},
                "timing_ms": {"overrun": 4.0},
            }
        ),
        encoding="utf-8",
    )
    evidence.sample(
        nav_path=nav_status,
        slam_path=missing_slam,
        traversability_path=missing_traversability,
    )

    assert evidence.samples == 2
    assert evidence.motion_health_samples == 1
    assert evidence.input_gate_ready_samples == 1
    assert evidence.max_odom_tf_rejections == 1
    assert evidence.max_cloud_pose_rejections == 2
    assert evidence.max_navigation_loop_overrun_ms == pytest.approx(4.0)


def test_native_evidence_starts_motion_health_only_after_command_is_accepted(tmp_path):
    nav_status = tmp_path / "nav.json"
    missing_slam = tmp_path / "missing-slam.json"
    missing_traversability = tmp_path / "missing-traversability.json"
    evidence = acceptance.NativeEvidence()

    nav_status.write_text(
        json.dumps({"input_gate": {"ready": True, "reason": "ready"}}),
        encoding="utf-8",
    )
    evidence.sample(
        nav_path=nav_status,
        slam_path=missing_slam,
        traversability_path=missing_traversability,
    )
    assert evidence.motion_health_samples == 0

    nav_status.write_text(
        json.dumps(
            {
                "input_gate": {"ready": True, "reason": "ready"},
                "command_boundary": {"last_accepted": True},
            }
        ),
        encoding="utf-8",
    )
    evidence.sample(
        nav_path=nav_status,
        slam_path=missing_slam,
        traversability_path=missing_traversability,
    )
    assert evidence.motion_health_samples == 1
    assert evidence.input_gate_ready_samples == 1


def test_native_evidence_can_read_final_status_without_polluting_motion_health(tmp_path):
    nav_status = tmp_path / "nav.json"
    missing_slam = tmp_path / "missing-slam.json"
    missing_traversability = tmp_path / "missing-traversability.json"
    evidence = acceptance.NativeEvidence()

    nav_status.write_text(
        json.dumps(
            {
                "input_gate": {"ready": True, "reason": "ready"},
                "command_boundary": {"last_accepted": True},
                "frame_gate": {"odom_rejected": 1},
            }
        ),
        encoding="utf-8",
    )
    evidence.sample(
        nav_path=nav_status,
        slam_path=missing_slam,
        traversability_path=missing_traversability,
    )

    nav_status.write_text(
        json.dumps(
            {
                "input_gate": {"ready": False, "reason": "driver_control_stale"},
                "frame_gate": {"odom_rejected": 99},
            }
        ),
        encoding="utf-8",
    )
    evidence.sample(
        nav_path=nav_status,
        slam_path=missing_slam,
        traversability_path=missing_traversability,
        collect_motion_health=False,
    )

    assert evidence.motion_health_samples == 1
    assert evidence.input_gate_ready_samples == 1
    assert evidence.max_odom_tf_rejections == 1
    assert evidence.last_nav["input_gate"]["reason"] == "driver_control_stale"


def test_motion_health_collection_stops_at_sensor_motion_complete_marker(tmp_path):
    marker = tmp_path / "motion_complete.json"

    assert acceptance._motion_health_collection_active(marker) is True
    marker.write_text('{"complete":true}\n', encoding="utf-8")
    assert acceptance._motion_health_collection_active(marker) is False


def test_native_evidence_freezes_motion_health_after_goal_reached_during_sensor_teardown(tmp_path):
    nav_status = tmp_path / "nav.json"
    missing_slam = tmp_path / "missing-slam.json"
    missing_traversability = tmp_path / "missing-traversability.json"
    evidence = acceptance.NativeEvidence()

    def sample(
        *,
        ready: bool,
        odom_rejected: int,
        cloud_pose_rejected: int,
        overrun_ms: float,
        goal_reached: bool = False,
    ) -> None:
        nav_status.write_text(
            json.dumps(
                    {
                        "input_gate": {"ready": ready, "reason": "ready" if ready else "odom_stale"},
                        "command_boundary": {"last_accepted": True},
                        "frame_gate": {"odom_rejected": odom_rejected},
                    "cloud_sync": {"pose_rejected": cloud_pose_rejected},
                    "timing_ms": {"overrun": overrun_ms},
                    "last_local": {"goal_reached": goal_reached},
                }
            ),
            encoding="utf-8",
        )
        evidence.sample(
            nav_path=nav_status,
            slam_path=missing_slam,
            traversability_path=missing_traversability,
        )

    sample(ready=True, odom_rejected=1, cloud_pose_rejected=2, overrun_ms=4.0)
    sample(ready=True, odom_rejected=1, cloud_pose_rejected=2, overrun_ms=6.0, goal_reached=True)
    # The sensor runner still owns shutdown. Its post-goal stream loss must not
    # pollute the completed motion interval's navigation health evidence.
    sample(ready=False, odom_rejected=80, cloud_pose_rejected=90, overrun_ms=570.0)
    sample(ready=False, odom_rejected=100, cloud_pose_rejected=110, overrun_ms=800.0)

    assert evidence.samples == 4
    assert evidence.motion_health_samples == 2
    assert evidence.input_gate_ready_samples == 2
    assert evidence.max_odom_tf_rejections == 1
    assert evidence.max_cloud_pose_rejections == 2
    assert evidence.max_consecutive_input_stale_s == 0.0
    assert evidence.max_navigation_loop_overrun_ms == pytest.approx(6.0)
    assert evidence.goal_reached_observed is True
    assert evidence.last_nav["input_gate"]["reason"] == "odom_stale"
    _, _, metrics = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": True},
        thresholds={"min_input_gate_ready_fraction": 0.98, "require_continuous_map_tracking": False},
        evidence=evidence,
        sensor_report={
            "motion": {
                "sim_start_xyz": [0.0, 0.0, 0.0],
                "sim_end_xyz": [0.0, 0.0, 0.0],
                "sim_path_length_xy_m": 0.0,
            }
        },
        goal=[0.0, 0.0, 0.0, 0.0],
    )
    assert metrics["motion_health_samples"] == 2
    assert metrics["input_gate_ready_fraction"] == pytest.approx(1.0)


def test_native_evidence_reports_loop_overrun_distribution_and_peak_context(tmp_path):
    nav_status = tmp_path / "nav.json"
    missing_slam = tmp_path / "missing-slam.json"
    missing_traversability = tmp_path / "missing-traversability.json"
    evidence = acceptance.NativeEvidence()
    overruns_ms = [2.0] * 98 + [80.0, 110.96]

    for sample_index, overrun_ms in enumerate(overruns_ms, start=1):
        nav_status.write_text(
            json.dumps(
                    {
                        "input_gate": {"ready": True, "reason": "ready"},
                        "command_boundary": {"last_accepted": True},
                        "frame_gate": {"odom_rejected": 0},
                    "cloud_sync": {"pose_rejected": 0},
                    "timing_ms": {"loop": round(50.0 + overrun_ms, 2), "overrun": overrun_ms},
                    "last_local": {"reason": "path_found", "goal_reached": False},
                    "stamp_s": float(sample_index),
                    "status_sequence": sample_index,
                }
            ),
            encoding="utf-8",
        )
        evidence.sample(
            nav_path=nav_status,
            slam_path=missing_slam,
            traversability_path=missing_traversability,
        )
        evidence.sample(
            nav_path=nav_status,
            slam_path=missing_slam,
            traversability_path=missing_traversability,
        )

    report = evidence.to_dict()
    assert report["samples"] == 200
    assert report["motion_health_samples"] == 200
    assert report["navigation_loop_overrun_samples_ms"] == overruns_ms
    assert report["navigation_loop_overrun_p95_ms"] == pytest.approx(2.0)
    assert report["navigation_loop_overrun_p99_ms"] == pytest.approx(80.0)
    assert report["max_navigation_loop_overrun_ms"] == pytest.approx(110.96)
    assert report["max_navigation_loop_overrun_context"] == {
        "evidence_sample": 199,
        "status_stamp_s": 100.0,
        "status_sequence": 100,
        "overrun_ms": 110.96,
        "timing_ms": {"loop": 160.96, "overrun": 110.96},
        "input_gate_ready": True,
        "input_gate_reason": "ready",
        "local_reason": "path_found",
        "goal_reached": False,
    }


def test_motion_rejects_native_input_sync_and_control_loop_health_regressions():
    evidence = acceptance.NativeEvidence(
        samples=100,
        motion_health_samples=100,
        input_gate_ready_samples=97,
        max_odom_tf_rejections=6,
        max_cloud_pose_rejections=8,
        max_consecutive_input_stale_s=1.2,
        navigation_loop_overrun_samples_ms=[570.0] * 100,
        max_navigation_loop_overrun_ms=570.0,
        max_navigation_loop_overrun_context={"evidence_sample": 100, "overrun_ms": 570.0},
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=120,
        max_local_path_points=40,
        max_cmd_vel_published=500,
        max_traversability_published=500,
        max_registered_clouds=500,
        max_path_follower_cmd_norm=0.4,
        max_computed_cmd_norm=0.4,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
        goal_reached_observed=True,
        last_slam={"state": "TRACKING", "track_against_map": {"successes": 10}},
    )
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "cmd_vel": {"nonzero_samples": 500},
        "motion": {
            "sim_start_xyz": [3.0, 4.0, 0.48],
            "sim_end_xyz": [56.0, 32.0, 0.48],
            "sim_path_length_xy_m": 70.0,
        },
    }

    ok, blockers, metrics = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": True},
        thresholds={
            "require_goal_reached": True,
            "max_goal_error_m": 0.5,
            "min_input_gate_ready_fraction": 0.98,
            "max_odom_tf_rejections": 5,
            "max_cloud_pose_rejections": 5,
            "max_consecutive_input_stale_s": 1.0,
            "max_navigation_loop_overrun_p99_ms": 100.0,
            "max_navigation_loop_overrun_peak_ms": 250.0,
        },
        evidence=evidence,
        sensor_report=sensor_report,
        goal=[56.0, 32.0, 0.3, 0.0],
    )

    assert not ok
    assert metrics["input_gate_ready_fraction"] == pytest.approx(0.97)
    assert metrics["odom_tf_rejections"] == 6
    assert metrics["cloud_pose_rejections"] == 8
    assert metrics["max_consecutive_input_stale_s"] == pytest.approx(1.2)
    assert metrics["navigation_loop_overrun_p95_ms"] == pytest.approx(570.0)
    assert metrics["navigation_loop_overrun_p99_ms"] == pytest.approx(570.0)
    assert metrics["max_navigation_loop_overrun_ms"] == pytest.approx(570.0)
    assert metrics["max_navigation_loop_overrun_context"]["overrun_ms"] == pytest.approx(570.0)
    assert "input_gate_ready_fraction_below_threshold" in blockers
    assert "odom_tf_rejections_above_threshold" in blockers
    assert "cloud_pose_rejections_above_threshold" in blockers
    assert "consecutive_input_stale_above_threshold" in blockers
    assert "navigation_loop_overrun_p99_above_threshold" in blockers
    assert "navigation_loop_overrun_peak_above_threshold" in blockers


def test_motion_loop_health_allows_one_bounded_spike_when_p99_is_healthy():
    overruns_ms = [4.0] * 199 + [110.96]
    evidence = acceptance.NativeEvidence(
        samples=200,
        motion_health_samples=200,
        input_gate_ready_samples=200,
        navigation_loop_overrun_samples_ms=overruns_ms,
        max_navigation_loop_overrun_ms=110.96,
        max_navigation_loop_overrun_context={"evidence_sample": 200, "overrun_ms": 110.96},
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=311,
        max_local_path_points=101,
        max_cmd_vel_published=500,
        max_traversability_published=500,
        max_registered_clouds=500,
        max_path_follower_cmd_norm=0.4,
        max_computed_cmd_norm=0.4,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
        goal_reached_observed=True,
        last_slam={"state": "TRACKING", "track_against_map": {"successes": 10}},
    )
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "cmd_vel": {"nonzero_samples": 500},
        "motion": {
            "sim_start_xyz": [3.0, 4.0, 0.48],
            "sim_end_xyz": [56.0, 32.0, 0.48],
            "sim_path_length_xy_m": 70.0,
        },
    }

    ok, blockers, metrics = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": True},
        thresholds={
            "require_goal_reached": True,
            "max_goal_error_m": 0.5,
            "min_input_gate_ready_fraction": 0.98,
            "max_navigation_loop_overrun_p99_ms": 100.0,
            "max_navigation_loop_overrun_peak_ms": 250.0,
        },
        evidence=evidence,
        sensor_report=sensor_report,
        goal=[56.0, 32.0, 0.3, 0.0],
    )

    assert ok, blockers
    assert metrics["navigation_loop_overrun_p95_ms"] == pytest.approx(4.0)
    assert metrics["navigation_loop_overrun_p99_ms"] == pytest.approx(4.0)
    assert metrics["max_navigation_loop_overrun_ms"] == pytest.approx(110.96)
    assert metrics["max_navigation_loop_overrun_context"]["overrun_ms"] == pytest.approx(110.96)
    assert "navigation_loop_overrun_p99_above_threshold" not in blockers
    assert "navigation_loop_overrun_peak_above_threshold" not in blockers


def test_navigation_isolation_can_disable_continuous_map_tracking_gate():
    evidence = acceptance.NativeEvidence(
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=8,
        max_local_path_points=8,
        max_cmd_vel_published=8,
        max_traversability_published=8,
        max_registered_clouds=8,
        max_path_follower_cmd_norm=0.2,
        max_computed_cmd_norm=0.2,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
        last_slam={
            "state": "TRACKING",
            "track_against_map": {"successes": 0, "consecutive_failures": 99},
        },
    )
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "cmd_vel": {"nonzero_samples": 8},
        "motion": {
            "sim_start_xyz": [0.0, 0.0, 0.48],
            "sim_end_xyz": [0.5, 0.0, 0.48],
            "sim_path_length_xy_m": 0.5,
        },
    }

    ok, blockers, _ = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": True},
        thresholds={"require_continuous_map_tracking": False},
        evidence=evidence,
        sensor_report=sensor_report,
        goal=[1.0, 0.0, 0.3, 0.0],
    )

    assert ok
    assert not blockers


def _navigation_isolation_degraded_slam_status():
    return {
        "state": "DEGRADED",
        "has_odom": True,
        "stamp_s": 123.0,
        "odom_prior_enabled": True,
        "odom_prior_active": True,
        "odometry": {
            "frame_id": "odom",
            "child_frame_id": "body",
            "pose": {
                "x": 3.0,
                "y": 4.0,
                "z": 0.48,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            },
        },
        "track_against_map": {"successes": 0, "consecutive_failures": 99},
    }


def _navigation_isolation_ready_evidence():
    return acceptance.NativeEvidence(
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=8,
        max_local_path_points=8,
        max_cmd_vel_published=8,
        max_traversability_published=8,
        max_registered_clouds=8,
        max_path_follower_cmd_norm=0.2,
        max_computed_cmd_norm=0.2,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
        last_nav={"has_odom": True, "counters": {"odom": 5}},
        last_slam=_navigation_isolation_degraded_slam_status(),
        last_traversability={"counters": {"published": 5}},
    )


def test_startup_allows_degraded_slam_only_for_explicit_valid_odom_prior_isolation(tmp_path):
    evidence = _navigation_isolation_ready_evidence()
    sensor = SimpleNamespace(poll=lambda: None)
    mapd_status = _complete_startup_dependencies(evidence, tmp_path)

    ok, reason = acceptance._wait_for_startup(
        sensor=sensor,
        evidence=evidence,
        nav_status=tmp_path / "missing-nav.json",
        slam_status=tmp_path / "missing-slam.json",
        traversability_status=tmp_path / "missing-traversability.json",
        mapd_status=mapd_status,
        timeout_s=0.1,
        thresholds={"allow_degraded_slam_for_navigation_isolation": True},
    )

    assert ok
    assert reason == "ready_navigation_isolation_odom_prior"


def test_startup_keeps_product_default_closed_for_degraded_slam(tmp_path):
    evidence = _navigation_isolation_ready_evidence()
    sensor = SimpleNamespace(poll=lambda: None)
    mapd_status = _complete_startup_dependencies(evidence, tmp_path)

    ok, reason = acceptance._wait_for_startup(
        sensor=sensor,
        evidence=evidence,
        nav_status=tmp_path / "missing-nav.json",
        slam_status=tmp_path / "missing-slam.json",
        traversability_status=tmp_path / "missing-traversability.json",
        mapd_status=mapd_status,
        timeout_s=0.1,
        thresholds={},
    )

    assert not ok
    assert reason == "native_runtime_startup_timeout"


def test_phase_allows_degraded_slam_only_with_complete_navigation_isolation_evidence():
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "cmd_vel": {"nonzero_samples": 8},
        "motion": {
            "sim_start_xyz": [0.0, 0.0, 0.48],
            "sim_end_xyz": [0.5, 0.0, 0.48],
            "sim_path_length_xy_m": 0.5,
        },
    }
    thresholds = {
        "allow_degraded_slam_for_navigation_isolation": True,
        "require_continuous_map_tracking": False,
    }

    ok, blockers, _ = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": True},
        thresholds=thresholds,
        evidence=_navigation_isolation_ready_evidence(),
        sensor_report=sensor_report,
        goal=[1.0, 0.0, 0.3, 0.0],
    )

    assert ok
    assert not blockers

    invalid_cases = (
        ("default_closed", lambda evidence: None, {}),
        (
            "prior_inactive",
            lambda evidence: evidence.last_slam.__setitem__("odom_prior_active", False),
            thresholds,
        ),
        (
            "slam_odometry_invalid",
            lambda evidence: evidence.last_slam["odometry"]["pose"].__setitem__("x", float("nan")),
            thresholds,
        ),
        (
            "nav_has_no_odom",
            lambda evidence: evidence.last_nav.__setitem__("has_odom", False),
            thresholds,
        ),
        (
            "nav_received_no_odom",
            lambda evidence: evidence.last_nav["counters"].__setitem__("odom", 0),
            thresholds,
        ),
    )
    for case_name, mutate, case_thresholds in invalid_cases:
        evidence = _navigation_isolation_ready_evidence()
        mutate(evidence)
        ok, blockers, _ = acceptance._evaluate_phase(
            phase="motion",
            phase_cfg={"publish_cmd_vel": True},
            thresholds=case_thresholds,
            evidence=evidence,
            sensor_report=sensor_report,
            goal=[1.0, 0.0, 0.3, 0.0],
        )
        assert not ok, case_name
        assert "slam_not_tracking" in blockers, case_name


def test_motion_strict_arrival_accepts_native_goal_and_pose_within_tolerance():
    evidence = acceptance.NativeEvidence(
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=120,
        max_local_path_points=40,
        max_cmd_vel_published=500,
        max_traversability_published=500,
        max_registered_clouds=500,
        max_path_follower_cmd_norm=0.4,
        max_computed_cmd_norm=0.4,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
        last_nav={"last_local": {"goal_reached": True}},
        last_slam={"state": "TRACKING", "track_against_map": {"successes": 10}},
    )
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "cmd_vel": {"nonzero_samples": 500},
        "motion": {
            "sim_start_xyz": [3.0, 4.0, 0.48],
            "sim_end_xyz": [55.8, 31.9, 0.48],
            "sim_path_length_xy_m": 61.0,
        },
    }

    ok, blockers, metrics = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": True},
        thresholds={
            "require_goal_reached": True,
            "max_goal_error_m": 0.5,
        },
        evidence=evidence,
        sensor_report=sensor_report,
        goal=[56.0, 32.0, 0.3, 0.0],
    )

    assert ok
    assert not blockers
    assert metrics["end_distance_m"] < 0.5


def test_motion_goal_latch_accepts_native_sensor_runner_terminal_observation():
    evidence = acceptance.NativeEvidence(
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=120,
        max_local_path_points=40,
        max_cmd_vel_published=500,
        max_traversability_published=500,
        max_registered_clouds=500,
        max_path_follower_cmd_norm=0.4,
        max_computed_cmd_norm=0.4,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
        last_nav={"last_local": {"goal_reached": False}},
        last_slam={"state": "TRACKING", "track_against_map": {"successes": 1}},
    )
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "goal_reached_early": True,
        "cmd_vel": {"nonzero_samples": 500},
        "motion": {
            "sim_start_xyz": [3.0, 4.0, 0.48],
            "sim_end_xyz": [55.8, 31.9, 0.48],
            "sim_path_length_xy_m": 61.0,
        },
    }

    ok, blockers, metrics = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": True},
        thresholds={"require_goal_reached": True, "max_goal_error_m": 0.5},
        evidence=evidence,
        sensor_report=sensor_report,
        goal=[56.0, 32.0, 0.3, 0.0],
    )

    assert ok
    assert "native_goal_not_reached" not in blockers
    assert metrics["native_goal_reached"] is True


def test_no_motion_accepts_embedded_command_stopped_by_final_safety():
    evidence = acceptance.NativeEvidence(
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=6,
        max_local_path_points=101,
        max_traversability_published=10,
        max_registered_clouds=10,
        pre_safety_command_samples=2,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
        last_slam={"state": "TRACKING", "track_against_map": {"successes": 1}},
    )
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "cmd_vel": {"nonzero_samples": 0},
    }

    ok, blockers, _ = acceptance._evaluate_phase(
        phase="no_motion",
        phase_cfg={"publish_cmd_vel": False},
        thresholds={"min_global_path_points": 2, "min_local_path_points": 2},
        evidence=evidence,
        sensor_report=sensor_report,
        goal=[1.0, 0.0, 0.3, 0.0],
    )

    assert ok
    assert not blockers


def test_no_motion_rejects_stale_map_tracking_and_static_pose_drift():
    evidence = acceptance.NativeEvidence(
        plan_accepted=True,
        local_path_found=True,
        max_global_path_points=60,
        max_local_path_points=101,
        max_traversability_published=10,
        max_registered_clouds=10,
        pre_safety_command_samples=2,
        local_path_role="dds_telemetry_and_preview",
        path_follower_role="embedded_before_cmd_vel_gate",
        command_transport="typed_dds_request_ack",
        max_command_requests=1,
        max_command_acks=1,
        command_last_accepted=True,
        last_slam={
            "state": "TRACKING",
            "track_against_map": {
                "successes": 2,
                "last_success_age_s": 12.0,
                "consecutive_failures": 8,
            },
        },
    )
    sensor_report = {
        "command_source": "dds",
        "policy_loaded": True,
        "cmd_vel": {"nonzero_samples": 0},
        "motion": {
            "slam_map_xy_error_m": 4.5,
            "slam_odom_xy_m": 4.2,
        },
    }

    ok, blockers, _ = acceptance._evaluate_phase(
        phase="no_motion",
        phase_cfg={"publish_cmd_vel": False},
        thresholds={
            "min_global_path_points": 2,
            "min_local_path_points": 2,
            "max_track_against_map_success_age_s": 3.0,
            "max_consecutive_track_against_map_failures": 3,
            "max_no_motion_slam_map_xy_error_m": 0.35,
            "max_no_motion_slam_odom_xy_m": 0.35,
        },
        evidence=evidence,
        sensor_report=sensor_report,
        goal=[7.4, 2.8, 4.15, 0.0],
    )

    assert not ok
    assert "continuous_map_tracking_stale" in blockers
    assert "continuous_map_tracking_degraded" in blockers
    assert "no_motion_slam_map_pose_drift" in blockers
    assert "no_motion_slam_odom_drift" in blockers


def test_motion_rejects_non_identity_map_odom_with_mislabeled_traversability():
    evidence = acceptance.NativeEvidence(
        last_slam={
            "map_odom_tf": {
                "tx": 0.308,
                "ty": 0.0,
                "tz": 0.0,
                "qw": 1.0,
            }
        }
    )
    contract = {
        "traversability_geometry_frame_current": "odom",
        "traversability_header_frame_current": "map",
        "navigation_query_frame": "map",
        "map_odom_identity_translation_tolerance_m": 0.05,
        "map_odom_identity_rotation_tolerance_rad": 0.05,
    }

    result = acceptance._traversability_frame_contract(evidence, contract)

    assert result["status"] == "mismatch"
    assert result["map_odom_non_identity"] is True
    assert result["mislabeled"] is True


def test_motion_accepts_non_identity_map_odom_after_traversability_transform():
    evidence = acceptance.NativeEvidence(
        last_slam={"map_odom_tf": {"tx": 0.308, "ty": 0.0, "tz": 0.0, "qw": 1.0}},
        last_traversability={
            "has_map_odom_tf": True,
            "frame_contract": {
                "odom_input_frame": "odom",
                "cloud_input_frame": "body",
                "geometry_frame": "map",
                "header_frame": "map",
                "map_odom_tf_age_s": 0.05,
            },
        },
    )
    contract = {
        "traversability_geometry_frame_current": "map",
        "traversability_header_frame_current": "map",
        "navigation_query_frame": "map",
        "max_map_odom_tf_age_s": 0.6,
        "map_odom_identity_translation_tolerance_m": 0.05,
        "map_odom_identity_rotation_tolerance_rad": 0.05,
    }

    result = acceptance._traversability_frame_contract(evidence, contract)

    assert result["status"] == "aligned"
    assert result["map_odom_non_identity"] is True
    assert result["mislabeled"] is False
    assert result["transform_ready"] is True


def test_evidence_treats_small_future_tf_age_as_fresh(tmp_path):
    traversability = tmp_path / "traversability.json"
    traversability.write_text(
        json.dumps(
            {
                "frame_contract": {"map_odom_tf_age_s": -0.007},
                "counters": {"published": 1},
            }
        ),
        encoding="utf-8",
    )
    evidence = acceptance.NativeEvidence()

    evidence.sample(
        nav_path=tmp_path / "missing-nav.json",
        slam_path=tmp_path / "missing-slam.json",
        traversability_path=traversability,
    )

    assert evidence.min_map_odom_tf_age_s == 0.007


def test_native_control_waits_for_business_ack_and_local_path_is_telemetry_only():
    client = (ROOT / "src" / "nav" / "cpp" / "client" / "client.cpp").read_text(encoding="utf-8")
    endpoint_loop = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "runtime" / "loop.cpp"
    ).read_text(encoding="utf-8")
    nav_loop = (ROOT / "src" / "nav" / "cpp" / "engine" / "nav_loop.cpp").read_text(encoding="utf-8")

    write_start = client.index("NavigationCommandReceipt writeCommandReceipt(")
    write_end = client.index("std::string writeCommand(", write_start)
    write_body = client[write_start:write_end]
    assert "kNavCommandRequest" in client
    assert "kNavCommandAck" in client
    register_index = write_body.index("registerNavigationAck(")
    publish_index = write_body.index("dds_write(command_writer", register_index)
    ack_index = write_body.index("waitForAckUntil(", publish_index)
    assert register_index < publish_index < ack_index
    ack_retry_start = write_body.index(
        "for (int ack_attempt = 0; ack_attempt < kNavigationCommandAckWriteAttempts"
    )
    ack_retry_end = write_body.index("if (!observation.has_value())", ack_retry_start)
    ack_retry_body = write_body[ack_retry_start:ack_retry_end]
    assert "command_ack_timeout_retry" in ack_retry_body
    assert "waitForAckUntil(" in ack_retry_body
    assert "active_request_id =" not in ack_retry_body
    assert "active_request_id," in ack_retry_body
    assert "final_ack_attempt" in ack_retry_body

    tick_index = endpoint_loop.index("autonomy_tick.tick(")
    telemetry_index = endpoint_loop.index("dds.writeLocalPath(out.local_path_map);", tick_index)
    assert tick_index < telemetry_index
    assert "drainLocalPath" not in endpoint_loop

    planner_index = nav_loop.index("local_planner_.plan(")
    follower_index = nav_loop.index("nav_kernel::computeControl(", planner_index)
    assert planner_index < follower_index

def test_policy_cpu_threads_are_bounded_and_reported(monkeypatch):
    state = {"cpu": 8, "interop": 8}

    def set_num_threads(value):
        state["cpu"] = int(value)

    def set_num_interop_threads(value):
        state["interop"] = int(value)

    fake_torch = SimpleNamespace(
        set_num_threads=set_num_threads,
        get_num_threads=lambda: state["cpu"],
        set_num_interop_threads=set_num_interop_threads,
        get_num_interop_threads=lambda: state["interop"],
    )
    monkeypatch.setitem(sys.modules, "torch", fake_torch)

    report = sensors._configure_policy_cpu_threads(Path("policy.pt"), 1)

    assert report == {
        "backend": "torchscript",
        "requested_cpu_threads": 1,
        "active_cpu_threads": 1,
        "active_interop_threads": 1,
    }
    with pytest.raises(ValueError, match="policy-cpu-threads"):
        sensors._configure_policy_cpu_threads(Path("policy.pt"), 0)


def test_sensor_runtime_performance_controls_are_explicit_and_validated():
    arguments = acceptance._sensor_runtime_args(
        {
            "sensor_runtime": {
                "physics_timestep_s": 0.002,
                "policy_cpu_threads": 1,
            }
        }
    )

    assert arguments[arguments.index("--physics-timestep-s") + 1] == "0.002"
    assert arguments[arguments.index("--policy-cpu-threads") + 1] == "1"

    with pytest.raises(ValueError, match="physics timestep"):
        acceptance._sensor_runtime_args(
            {"sensor_runtime": {"physics_timestep_s": 0.01}}
        )
    with pytest.raises(ValueError, match="policy_cpu_threads"):
        acceptance._sensor_runtime_args(
            {"sensor_runtime": {"policy_cpu_threads": 0}}
        )


def test_60m_acceptance_allows_visibility_simplified_global_route():
    manifest = acceptance._load_manifest(
        ROOT
        / "config"
        / "runtime_graph"
        / "acceptance"
        / "mujoco_industrial_park_60m_navigation_acceptance.json"
    )
    thresholds = dict(manifest["thresholds"])

    evidence = acceptance.NativeEvidence(max_global_path_points=9)
    _, blockers, _ = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": False},
        thresholds=thresholds,
        evidence=evidence,
        sensor_report={},
        goal=[56.0, 32.0, 0.3, 0.0],
    )
    assert "global_path_too_short" not in blockers

    evidence.max_global_path_points = 1
    _, blockers, _ = acceptance._evaluate_phase(
        phase="motion",
        phase_cfg={"publish_cmd_vel": False},
        thresholds=thresholds,
        evidence=evidence,
        sensor_report={},
        goal=[56.0, 32.0, 0.3, 0.0],
    )
    assert "global_path_too_short" in blockers
