from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from sim.engine.mujoco import robot_controller
from sim.scripts.mujoco import native_dds_sensors as sensors
from sim.scripts.mujoco import native_navigation_acceptance as acceptance
from sim.scripts.mujoco import native_navigation_video as navigation_video

ROOT = Path(__file__).resolve().parents[2]


class _FakeOnnxSession:
    def __init__(self, input_dim: int) -> None:
        self._input = SimpleNamespace(name="obs", shape=[1, input_dim])
        self._output = SimpleNamespace(name="actions", shape=[1, 16])

    def get_inputs(self):
        return [self._input]

    def get_outputs(self):
        return [self._output]


def test_onnx_policy_runner_is_selected_from_input_contract_not_directory(monkeypatch, tmp_path):
    sessions = iter([_FakeOnnxSession(285), _FakeOnnxSession(57)])
    monkeypatch.setitem(
        sys.modules,
        "onnxruntime",
        SimpleNamespace(InferenceSession=lambda *_args, **_kwargs: next(sessions)),
    )
    policy_dir = tmp_path / "thunderv4" / "policy"
    policy_dir.mkdir(parents=True)

    history_runner = robot_controller.load_policy_runner(str(policy_dir / "history.onnx"))
    frame_runner = robot_controller.load_policy_runner(str(policy_dir / "frame.onnx"))

    assert isinstance(history_runner, robot_controller.PolicyRunner)
    assert history_runner._history_len == 5
    assert history_runner.run_at_idle is True
    assert history_runner.zero_wheels_at_idle is True
    assert isinstance(frame_runner, robot_controller.ThunderV4OnnxPolicyRunner)


def test_cpp_cmd_vel_tap_consumes_canonical_typed_dds_topic():
    source = (ROOT / "sim" / "native_dds" / "cmd_vel_tap.cpp").read_text(encoding="utf-8")
    cmake = (ROOT / "sim" / "native_dds" / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "lingtu::message::kNavCmdVel" in source
    assert "lingtu_dds_TwistStamped_desc" in source
    assert "qos_for_topic(contract.dds_topic)" in source
    assert "LT_CMD_V1" in source
    assert "src/message/idl/lingtu_slam.idl" in cmake


def test_cpp_cmd_vel_tap_publishes_simulated_thunder_control_readiness():
    source = (ROOT / "sim" / "native_dds" / "cmd_vel_tap.cpp").read_text(encoding="utf-8")

    assert "lingtu::message::kDriverControlState" in source
    assert "lingtu_dds_DriverControlState_desc" in source
    assert "msg.connected = true" in source
    assert "msg.ready = true" in source
    assert "msg.motors_enabled = true" in source
    assert 'msg.owner = const_cast<char*>("grpc")' in source
    assert 'msg.owner_id = const_cast<char*>("lingtu-driver")' in source
    assert "dds_write(writer, &msg)" in source
    assert "writeControlState(control_state_writer, sequence)" in source


def test_native_navigation_phase_timeout_includes_wsl_shutdown_grace():
    assert acceptance._phase_runtime_timeout_s(20.0, 120.0) == pytest.approx(140.0)
    assert acceptance._phase_runtime_timeout_s(20.0, 10.0) == pytest.approx(80.0)


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


def test_sensor_bridge_parses_native_cmd_vel_tap_protocol():
    assert sensors.NativeCmdVelSource.parse_line("LT_CMD_V1\t7\t123.25\t0.3\t-0.1\t0.4\n") == (123.25, 0.3, -0.1, 0.4)
    assert sensors.NativeCmdVelSource.parse_line("unrelated log line") is None
    assert sensors.NativeCmdVelSource.parse_line("LT_CMD_V1\tbad") is None
    assert sensors.NativeCmdVelSource.parse_pid_line("LT_PID_V1\t123\n") == 123
    assert sensors.NativeCmdVelSource.parse_pid_line("LT_PID_V1\tbad") is None


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


def test_mujoco_sensor_records_are_restamped_inside_native_publisher():
    publisher = (ROOT / "src" / "drivers" / "real" / "lidar" / "sdk2_stream" / "main.cpp").read_text(encoding="utf-8")
    sensor_bridge = (ROOT / "sim" / "scripts" / "mujoco" / "native_dds_sensors.py").read_text(encoding="utf-8")

    assert "--restamp-stdin-records" in publisher
    assert "replay_restamper.stamp_ns(source_timestamp_ns, target_deadline)" in publisher
    assert "if (navigation_fixture)" in publisher
    assert "source_timestamp_ns, std::chrono::steady_clock::now()" in publisher
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
    assert '"--restamp-stdin-records"' in sensor_bridge


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


def test_motion_log_hydrates_deduplicated_planner_debug_sidecar(tmp_path):
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
                "candidates": [{"selected": False, "path": [[0, 0, 0], [2, 0, 0]]}],
            },
            "local_map": {"enabled": True, "obstacle_points": [[2, 0, 0, 1]]},
        },
    ]
    debug_log.write_text(
        "".join(json.dumps(snapshot) + "\n" for snapshot in snapshots),
        encoding="utf-8",
    )

    rows = navigation_video._load_jsonl(motion_log)

    assert len(rows) == 1
    assert rows[0]["planner_debug_id"] == 7
    assert rows[0]["nav_status_stamp_s"] == 12.4
    assert rows[0]["nav_status_hold_age_s"] == pytest.approx(0.1)
    assert rows[0]["local_candidates"][0]["selected"] is True
    assert rows[0]["local_map"]["enabled"] is True
    assert rows[0]["local_planner_debug"]["timestamp_s"] == 12.3
    assert rows[0]["local_reason"] == "control_ready"
    assert rows[0]["global_path"][-1] == [4, 0, 0]
    assert rows[0]["input_gate"]["reason"] == "ready"
    assert rows[0]["dynamic_objects"][0]["id"] == 3
    assert navigation_video._snapshot_age_s(rows[0]) == pytest.approx(0.2)


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
    assert '"inset_overlay": True' in renderer_source
    assert '"local_candidates"' in renderer_source
    assert '"planner_local_map"' in renderer_source
    assert '"--nav-status-json", str(nav_status)' in acceptance_source
    assert '"--local-planner-debug-candidates"' in acceptance_source
    assert '"--local-map-debug-points"' in acceptance_source
    assert "render_native_navigation_video(" in acceptance_source


def test_local_planner_visualization_uses_distinct_candidate_colors_and_selected_green():
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
        (ROOT / "config" / "runtime_graph" / "endpoints" / "mujoco_native_navigation_acceptance.json").read_text(
            encoding="utf-8"
        )
    )

    contracts = manifest["contracts"]
    assert contracts["sensor_inputs"] == ["rt/lidar/raw_frame", "rt/imu/raw"]
    assert contracts["terrain_output"] == "rt/nav/traversability"
    assert "rt/nav/command/request" in contracts["navigation_inputs"]
    assert "rt/nav/goal_pose" not in contracts["navigation_inputs"]
    assert contracts["navigation_command_ack"] == "rt/nav/command/ack"
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
    assert "t_il: [0.0, 0.0, 0.0]" in slam_config
    assert "navigation_body_from_imu_translation: [-0.30638, 0.0, 0.19417]" in slam_config
    assert manifest["runtime_tolerances"]["track_against_map_period_s"] == 5.0
    assert manifest["thresholds"]["min_track_against_map_successes"] == 1
    assert manifest["asset_builder"]["kind"] == "saved_map_plan_gate"
    assert manifest["asset_builder"]["map_source"] == "mujoco_lidar"
    assert manifest["world"] == ""
    assert manifest["map_dir"] == ""


def test_multifloor_merged_manifest_locks_runtime_octoplanner3d_constraints():
    manifest = acceptance._load_manifest(
        ROOT / "config" / "runtime_graph" / "endpoints" / "mujoco_multifloor_navigation_acceptance.json"
    )

    assert manifest["asset_builder"]["scene_preset"] == "multifloor_stack_3"
    assert manifest["asset_builder"]["resolution"] == 0.09
    assert manifest["start"][:3] == [7.0, -1.2, 0.55]
    assert manifest["goal"][:3] == [7.4, 2.8, 4.15]
    constraints = manifest["planner_constraints"]
    assert constraints["body_clearance_below_m"] == 0.18
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
    assert "--publish-odom-prior" not in sensor_args


def test_industrial_park_navigation_matches_global_and_local_clearance_contracts():
    manifest = acceptance._load_manifest(
        ROOT / "config" / "runtime_graph" / "endpoints" / "mujoco_industrial_park_60m_navigation_acceptance.json"
    )

    assert manifest["planner_constraints"]["robot_radius_m"] == 0.95
    assert manifest["navigation_runtime"]["corridor_lookahead_m"] == 1.2
    assert manifest["sensor_runtime"]["odom_prior_velocity_window_s"] == 0.1
    assert manifest["sensor_runtime"]["stop_on_nav_goal_reached"] is True
    assert manifest["sensor_runtime"]["mid360_samples_per_frame"] == 10000
    assert manifest["sensor_runtime"]["imu_hz"] == 100.0
    assert manifest["sensor_runtime"]["scan_time_profile"] == "instantaneous"
    assert "physical_rolling_sample_mode" not in manifest["sensor_runtime"]
    assert manifest["telemetry_log"]["hz"] == 10.0
    assert manifest["telemetry_log"]["lidar_points"] == 640
    assert manifest["slam_runtime"]["provider"] == "mujoco_navigation_fixture"
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
    assert manifest["thresholds"]["require_slam_tracking"] is False
    assert manifest["thresholds"]["require_slam_accuracy"] is False
    assert manifest["thresholds"]["require_navigation_safety_contracts"] is True
    assert manifest["thresholds"]["require_video_artifact"] is True
    assert acceptance.RUNTIME_EVIDENCE_SAMPLE_PERIOD_S == 0.20
    assert acceptance.SENSOR_PUBLISHER_PROBE_TIMEOUT_S == 60.0
    sensor_args = acceptance._sensor_runtime_args(manifest)
    assert "--stop-on-nav-goal-reached" in sensor_args
    assert sensor_args[sensor_args.index("--odom-prior-velocity-window-s") + 1] == "0.1"
    assert sensor_args[sensor_args.index("--scan-time-profile") + 1] == "instantaneous"
    assert sensor_args[sensor_args.index("--navigation-fixture-cloud-points") + 1] == "4000"
    assert sensor_args[sensor_args.index("--imu-hz") + 1] == "100.0"
    source = (ROOT / "sim" / "scripts" / "mujoco" / "native_navigation_acceptance.py").read_text(encoding="utf-8")
    assert '"--corridor-lookahead-m"' in source
    endpoint_source = (ROOT / "src" / "nav" / "services" / "endpoint" / "cpp" / "nav_native_endpoint.cpp").read_text(
        encoding="utf-8"
    )
    assert ("nav_config.local_planner.footprintPadding = safety_config.obstacle_margin_m;") in endpoint_source
    assert endpoint_source.count("const auto safety_config = commandSafetyConfig(cfg);") == 1


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
            video_report={"requested": True, "ok": True},
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


def test_navigation_fixture_readiness_uses_native_nav_odom_without_slam_status():
    readiness = acceptance._slam_navigation_readiness(
        slam={},
        nav={"has_odom": True, "counters": {"odom": 5}},
        thresholds={"navigation_state_provider": "mujoco_navigation_fixture"},
    )

    assert readiness == "mujoco_navigation_fixture"


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

    ok, _ = acceptance._wait_for_startup(
        sensor=sensor,
        evidence=evidence,
        nav_status=paths["nav_status"],
        slam_status=paths["slam_status"],
        traversability_status=paths["traversability_status"],
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

    ok, reason = acceptance._wait_for_startup(
        sensor=SimpleNamespace(poll=lambda: None),
        evidence=evidence,
        nav_status=tmp_path / "missing-nav.json",
        slam_status=tmp_path / "missing-slam.json",
        traversability_status=tmp_path / "missing-traversability.json",
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

    def sample(**_):
        evidence.last_nav["input_gate"]["ready"] = next(ready_sequence, True)

    evidence.sample = sample

    ok, reason = acceptance._wait_for_startup(
        sensor=SimpleNamespace(poll=lambda: None),
        evidence=evidence,
        nav_status=tmp_path / "missing-nav.json",
        slam_status=tmp_path / "missing-slam.json",
        traversability_status=tmp_path / "missing-traversability.json",
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


def test_native_endpoint_keeps_lidar_extrinsic_out_of_body_pose_local_planning():
    source = (
        ROOT / "src/nav/services/endpoint/cpp/nav_native_endpoint.cpp"
    ).read_text(encoding="utf-8")

    assert "sensorOriginFromBody" in source
    assert "cfg.sensor_offset_x_m" in source
    assert "cfg.sensor_offset_y_m" in source
    assert "nav_config.local_planner.sensorOffsetX = 0.0;" in source
    assert "nav_config.local_planner.sensorOffsetY = 0.0;" in source
    assert (
        "nav_config.local_planner.sensorOffsetX = cfg.sensor_offset_x_m;"
        not in source
    )
    assert (
        "nav_config.local_planner.sensorOffsetY = cfg.sensor_offset_y_m;"
        not in source
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
        (ROOT / "config" / "runtime_graph" / "endpoints" / "mujoco_multifloor_navigation_acceptance.json").read_text(
            encoding="utf-8"
        )
    )

    assert manifest["asset_builder"]["scene_preset"] == "multifloor_stack_3"
    assert manifest["asset_builder"]["resolution"] == 0.09
    assert manifest["start"][:3] == [7.0, -1.2, 0.55]
    assert manifest["goal"][:3] == [7.4, 2.8, 4.15]
    constraints = manifest["planner_constraints"]
    assert constraints["body_clearance_below_m"] == 0.18
    assert constraints["body_clearance_above_m"] == 0.30
    assert constraints["support_height_m"] == 0.55
    assert constraints["max_step_height_m"] == 0.23
    assert constraints["max_slope"] == 0.65
    args = acceptance._planner_constraint_args(manifest)
    assert args[args.index("--octo-max-step-height-m") + 1] == "0.23"
    assert args[args.index("--octo-support-height-m") + 1] == "0.55"


def test_native_acceptance_rejects_unknown_planner_constraint():
    try:
        acceptance._planner_constraint_args({"planner_constraints": {"invented": 1}})
    except ValueError as exc:
        assert "invented" in str(exc)
    else:
        raise AssertionError("unknown planner constraint must fail closed")


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


def test_preflight_validates_same_source_map_hashes(tmp_path, monkeypatch):
    map_dir = tmp_path / "map"
    map_dir.mkdir()
    map_pcd = map_dir / "map.pcd"
    octomap = map_dir / "octomap.ot"
    map_pcd.write_bytes(b"pcd")
    octomap.write_bytes(b"octomap")
    map_hash = hashlib.sha256(map_pcd.read_bytes()).hexdigest()
    octomap_hash = hashlib.sha256(octomap.read_bytes()).hexdigest()
    (map_dir / "metadata.json").write_text(
        json.dumps(
            {
                "artifacts": {
                    "map_pcd": {"sha256": map_hash},
                    "octomap": {
                        "sha256": octomap_hash,
                        "source_map_sha256": map_hash,
                    },
                }
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
    manifest = {
        "map_dir": str(map_dir),
        "map_files": {"slam": "map.pcd", "planner": "octomap.ot", "metadata": "metadata.json"},
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
                "cmd_vel_tap",
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
    assert provenance["metadata_planner_source_sha256"] == map_hash


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

    ok, reason = acceptance._wait_for_startup(
        sensor=sensor,
        evidence=evidence,
        nav_status=tmp_path / "missing-nav.json",
        slam_status=tmp_path / "missing-slam.json",
        traversability_status=tmp_path / "missing-traversability.json",
        timeout_s=0.1,
        thresholds={"allow_degraded_slam_for_navigation_isolation": True},
    )

    assert ok
    assert reason == "ready_navigation_isolation_odom_prior"


def test_startup_keeps_product_default_closed_for_degraded_slam(tmp_path):
    evidence = _navigation_isolation_ready_evidence()
    sensor = SimpleNamespace(poll=lambda: None)

    ok, reason = acceptance._wait_for_startup(
        sensor=sensor,
        evidence=evidence,
        nav_status=tmp_path / "missing-nav.json",
        slam_status=tmp_path / "missing-slam.json",
        traversability_status=tmp_path / "missing-traversability.json",
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


def test_native_runner_does_not_import_python_planners():
    source = (ROOT / "sim" / "scripts" / "mujoco" / "native_navigation_acceptance.py").read_text(encoding="utf-8")
    forbidden = (
        "GlobalPlannerService",
        "LocalPlannerModule",
        "PathFollowerModule",
        "octoplanner3d_planner.py",
        "nav.local",
    )
    for token in forbidden:
        assert token not in source


def test_native_control_waits_for_business_ack_and_local_path_is_telemetry_only():
    client = (ROOT / "src" / "nav" / "commands" / "cpp" / "client.cpp").read_text(encoding="utf-8")
    endpoint = (ROOT / "src" / "nav" / "services" / "endpoint" / "cpp" / "nav_native_endpoint.cpp").read_text(
        encoding="utf-8"
    )
    nav_loop = (ROOT / "src" / "nav" / "services" / "plan" / "cpp" / "nav_loop.cpp").read_text(encoding="utf-8")

    write_start = client.index("void writeCommand(")
    write_end = client.index("void writeReasonCommand(", write_start)
    write_body = client[write_start:write_end]
    assert "kNavCommandRequest" in client
    assert "kNavCommandAck" in client
    register_index = write_body.index("registerNavigationAck(")
    publish_index = write_body.index("dds_write(command_writer", register_index)
    ack_index = write_body.index("waitForAck(", publish_index)
    assert register_index < publish_index < ack_index
    assert "active_request_id, pending, timeout_ms" in write_body

    tick_index = endpoint.index("auto out = nav.tick(")
    telemetry_index = endpoint.index("dds.writeLocalPath(out.local_path_map);", tick_index)
    assert tick_index < telemetry_index
    assert "drainLocalPath" not in endpoint

    planner_index = nav_loop.index("local_planner_.plan(")
    follower_index = nav_loop.index("nav_kernel::computeControl(", planner_index)
    assert planner_index < follower_index
