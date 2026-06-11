from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = [pytest.mark.sim]

from sim.scripts.navigation_replay_deviation_gate import SCHEMA_VERSION
from sim.scripts.navigation_replay_deviation_gate import build_fixture_trace
from sim.scripts.navigation_replay_deviation_gate import build_report
from sim.scripts.navigation_replay_deviation_gate import build_routecheck_trace
from sim.scripts.navigation_replay_deviation_gate import build_topic_jsonl_trace
from sim.scripts.navigation_replay_deviation_gate import evaluate_trace


REPO_ROOT = Path(__file__).resolve().parents[2]


def _write_routecheck_report(tmp_path: Path) -> Path:
    plan_path = tmp_path / "routecheck" / "candidate" / "plan.json"
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(
        json.dumps(
            {
                "feasible": True,
                "goal": {"x": 2.0, "y": 0.4, "z": 0.0, "frame_id": "map"},
                "path": [
                    {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map"},
                    {"x": 1.0, "y": 0.2, "z": 0.0, "frame_id": "map"},
                    {"x": 2.0, "y": 0.4, "z": 0.0, "frame_id": "map"},
                ],
            }
        ),
        encoding="utf-8",
    )
    report_path = tmp_path / "routecheck" / "summary.json"
    report_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "mode": "routecheck_non_motion",
                "outcome": "pass",
                "simulation_only": True,
                "real_robot_motion": False,
                "cmd_vel_sent_to_hardware": False,
                "artifacts": {
                    "candidate_files": {"plan": str(plan_path)},
                },
            }
        ),
        encoding="utf-8",
    )
    return report_path


def _write_topic_jsonl(tmp_path: Path, *, encoding: str = "utf-8") -> Path:
    path = tmp_path / "recorded_topics.jsonl"
    events = [
        {
            "topic": "/nav/global_path",
            "t": 0.0,
            "msg": {
                "poses": [
                    {"pose": {"position": {"x": 0.0, "y": 0.0}}},
                    {"pose": {"position": {"x": 1.0, "y": 0.2}}},
                    {"pose": {"position": {"x": 2.0, "y": 0.4}}},
                ]
            },
        },
        {
            "topic": "/nav/local_path",
            "t": 0.01,
            "msg": {
                "poses": [
                    {"pose": {"position": {"x": 0.0, "y": 0.0}}},
                    {"pose": {"position": {"x": 0.7, "y": 0.14}}},
                    {"pose": {"position": {"x": 1.4, "y": 0.28}}},
                    {"pose": {"position": {"x": 2.0, "y": 0.4}}},
                ]
            },
        },
        {"topic": "/goal_pose", "t": 0.02, "msg": {"pose": {"position": {"x": 2.0, "y": 0.4}}}},
    ]
    for index in range(8):
        progress = index / 7.0
        events.extend(
            [
                {
                    "topic": "/nav/cmd_vel",
                    "t": 0.1 + index * 0.2,
                    "msg": {"linear": {"x": 0.35, "y": 0.0}, "angular": {"z": 0.02}},
                },
                {
                    "topic": "/Odometry",
                    "t": 0.11 + index * 0.2,
                    "msg": {
                        "pose": {
                            "pose": {
                                "position": {
                                    "x": progress * 2.0,
                                    "y": progress * 0.4,
                                }
                            }
                        }
                    },
                },
            ]
        )
    path.write_text("\n".join(json.dumps(event) for event in events), encoding=encoding)
    return path


def test_fixture_navigation_replay_deviation_passes():
    report = build_report(fixture=True)

    assert report["schema_version"] == SCHEMA_VERSION
    assert report["ok"] is True
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["global_path_count"] > 0
    assert report["local_path_count"] > 0
    assert report["cmd_vel_nonzero"] > 0
    assert report["final_distance_m"] <= report["thresholds"]["max_final_distance_m"]
    assert report["tracking_error_p95_m"] <= report["thresholds"]["max_tracking_error_p95_m"]


def test_navigation_replay_deviation_rejects_zero_commands_and_tracking_drift(tmp_path: Path):
    trace = build_fixture_trace()
    for sample in trace["samples"]:
        sample["cmd_vel"] = {"linear_x": 0.0, "angular_z": 0.0}
        sample["tracking_error_m"] = 1.2
    trace_path = tmp_path / "trace.json"
    trace_path.write_text(json.dumps(trace), encoding="utf-8")

    report = build_report(trace_path=trace_path)

    assert report["ok"] is False
    assert any("command_replay" in gap for gap in report["remaining_gaps"])
    assert any("tracking_deviation" in gap for gap in report["remaining_gaps"])


def test_navigation_replay_deviation_evaluator_handles_trace_dict_directly():
    trace = build_fixture_trace()

    evaluation = evaluate_trace(trace)

    assert evaluation["ok"] is True
    assert evaluation["sample_count"] == len(trace["samples"])
    assert evaluation["odom_motion_m"] >= evaluation["thresholds"]["min_odom_motion_m"]


def test_navigation_replay_deviation_builds_trace_from_routecheck_report(tmp_path: Path):
    routecheck_report = _write_routecheck_report(tmp_path)

    trace = build_routecheck_trace(routecheck_report)
    evaluation = evaluate_trace(trace)

    assert trace["source"] == "routecheck_preflight_plan_preview"
    assert trace["trace_kind"] == "non_motion_plan_replay"
    assert trace["simulation_only"] is True
    assert trace["cmd_vel_sent_to_hardware"] is False
    assert evaluation["ok"] is True
    assert evaluation["cmd_vel_nonzero"] > 0


def test_navigation_replay_deviation_report_can_write_routecheck_trace(tmp_path: Path):
    routecheck_report = _write_routecheck_report(tmp_path)
    trace_path = tmp_path / "navigation_replay_deviation" / "trace.json"

    report = build_report(routecheck_report_path=routecheck_report, write_trace=trace_path)

    assert report["ok"] is True
    assert report["trace_kind"] == "non_motion_plan_replay"
    assert report["trace_artifact"] == str(trace_path)
    trace = json.loads(trace_path.read_text(encoding="utf-8"))
    assert trace["source_plan"].endswith("plan.json")


def test_navigation_replay_deviation_builds_trace_from_topic_jsonl(tmp_path: Path):
    topic_jsonl = _write_topic_jsonl(tmp_path)

    trace = build_topic_jsonl_trace(topic_jsonl)
    evaluation = evaluate_trace(trace)

    assert trace["source"] == "recorded_topic_jsonl"
    assert trace["trace_kind"] == "recorded_topic_replay"
    assert trace["simulation_only"] is True
    assert trace["cmd_vel_sent_to_hardware"] is False
    assert evaluation["ok"] is True
    assert evaluation["global_path_count"] > 0
    assert evaluation["local_path_count"] > 0
    assert evaluation["cmd_vel_nonzero"] > 0


def test_navigation_replay_deviation_builds_trace_from_bom_topic_jsonl(tmp_path: Path):
    topic_jsonl = _write_topic_jsonl(tmp_path, encoding="utf-8-sig")

    trace = build_topic_jsonl_trace(topic_jsonl)
    evaluation = evaluate_trace(trace)

    assert trace["trace_kind"] == "recorded_topic_replay"
    assert evaluation["ok"] is True


def test_navigation_replay_deviation_cli_fixture_writes_report(tmp_path: Path):
    report_path = tmp_path / "report.json"

    probe = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "sim/scripts/navigation_replay_deviation_gate.py"),
            "--fixture",
            "--json-out",
            str(report_path),
            "--strict",
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        timeout=20,
    )

    assert probe.returncode == 0, probe.stderr
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["ok"] is True


def test_navigation_replay_deviation_cli_routecheck_report_writes_trace(tmp_path: Path):
    routecheck_report = _write_routecheck_report(tmp_path)
    trace_path = tmp_path / "trace.json"
    report_path = tmp_path / "report.json"

    probe = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "sim/scripts/navigation_replay_deviation_gate.py"),
            "--routecheck-report",
            str(routecheck_report),
            "--write-trace",
            str(trace_path),
            "--json-out",
            str(report_path),
            "--strict",
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        timeout=20,
    )

    assert probe.returncode == 0, probe.stderr
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["ok"] is True
    assert trace_path.exists()


def test_navigation_replay_deviation_cli_topic_jsonl_writes_trace(tmp_path: Path):
    topic_jsonl = _write_topic_jsonl(tmp_path)
    trace_path = tmp_path / "topic_trace.json"
    report_path = tmp_path / "report.json"

    probe = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "sim/scripts/navigation_replay_deviation_gate.py"),
            "--topic-jsonl",
            str(topic_jsonl),
            "--write-trace",
            str(trace_path),
            "--json-out",
            str(report_path),
            "--strict",
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        timeout=20,
    )

    assert probe.returncode == 0, probe.stderr
    report = json.loads(report_path.read_text(encoding="utf-8"))
    trace = json.loads(trace_path.read_text(encoding="utf-8"))
    assert report["ok"] is True
    assert report["trace_kind"] == "recorded_topic_replay"
    assert trace["source"] == "recorded_topic_jsonl"


def test_navigation_replay_deviation_cli_missing_trace_is_red(tmp_path: Path):
    report_path = tmp_path / "report.json"

    probe = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "sim/scripts/navigation_replay_deviation_gate.py"),
            "--trace",
            str(tmp_path / "missing_trace.json"),
            "--json-out",
            str(report_path),
            "--strict",
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        timeout=20,
    )

    assert probe.returncode == 1
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["ok"] is False
    assert any("failed to load trace" in gap for gap in report["remaining_gaps"])
