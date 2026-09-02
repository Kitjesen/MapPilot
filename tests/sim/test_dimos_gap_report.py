from __future__ import annotations

import json
from pathlib import Path

from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES
from sim.diagnostics.dataflow_report import (
    RUNTIME_DATAFLOW_GATES,
    build_runtime_dataflow_from_summary,
)
from sim.diagnostics.gap_report import _host_setup_plan, build_dimos_gap_report


def test_runtime_dataflow_reads_current_native_gate_report(tmp_path: Path) -> None:
    report_path = tmp_path / "saved-map.json"
    report_path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.saved_map_relocalize_runtime.v2",
                "ok": True,
                "runtime_dataflow": [{"id": "native_slam_status", "ok": True}],
            }
        ),
        encoding="utf-8",
    )
    summary = {"gates": {"saved_map_relocalize": {"path": str(report_path)}}}

    dataflow = build_runtime_dataflow_from_summary(
        summary,
        root=tmp_path,
        gates={"saved_map_relocalize"},
    )

    gate = dataflow["saved_map_relocalize"]
    assert gate["checked"] is True
    assert gate["ok"] is True
    assert gate["flow"] == [{"id": "native_slam_status", "ok": True}]


def test_runtime_dataflow_reads_current_navigation_gate_report(tmp_path: Path) -> None:
    report_path = tmp_path / "navigation.json"
    report_path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.navigation_replay_deviation_gate.v1",
                "ok": True,
                "runtime_dataflow": [
                    {
                        "id": "tracking_deviation",
                        "ok": True,
                        "evidence": {"tracking_error_p95_m": 0.1},
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    summary = {"gates": {"navigation_replay_deviation": {"path": str(report_path)}}}

    dataflow = build_runtime_dataflow_from_summary(
        summary,
        root=tmp_path,
        gates={"navigation_replay_deviation"},
    )

    gate = dataflow["navigation_replay_deviation"]
    assert gate["checked"] is True
    assert gate["ok"] is True
    assert gate["flow"][0]["id"] == "tracking_deviation"


def test_runtime_dataflow_rejects_current_schema_without_flow(tmp_path: Path) -> None:
    report_path = tmp_path / "navigation.json"
    report_path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.navigation_replay_deviation_gate.v1",
                "ok": True,
            }
        ),
        encoding="utf-8",
    )
    summary = {"gates": {"navigation_replay_deviation": {"path": str(report_path)}}}

    dataflow = build_runtime_dataflow_from_summary(
        summary,
        root=tmp_path,
        gates={"navigation_replay_deviation"},
    )

    gate = dataflow["navigation_replay_deviation"]
    assert gate["checked"] is False
    assert gate["ok"] is False
    assert gate["reason"] == "runtime_dataflow_missing"


def test_runtime_dataflow_rejects_unsupported_report_shape(tmp_path: Path) -> None:
    report_path = tmp_path / "unsupported.json"
    report_path.write_text(
        json.dumps({"schema_version": "lingtu.retired_gate.v1", "ok": True}),
        encoding="utf-8",
    )
    summary = {"gates": {"saved_map_relocalize": {"path": str(report_path)}}}

    dataflow = build_runtime_dataflow_from_summary(
        summary,
        root=tmp_path,
        gates={"saved_map_relocalize"},
    )

    gate = dataflow["saved_map_relocalize"]
    assert gate["checked"] is False
    assert gate["ok"] is False
    assert gate["reason"] == "unsupported_report_shape"
    assert gate["schema_detected"] == "lingtu.retired_gate.v1"


def test_gap_report_uses_only_current_gate_sequence() -> None:
    gates = {name: {"ok": True, "status": "passed"} for name in DIMOS_BENCHMARK_REQUIRED_GATES}
    runtime_dataflow = {
        name: {"checked": True, "ok": True, "flow": [{"id": "current_evidence", "ok": True}]}
        for name in RUNTIME_DATAFLOW_GATES
    }
    report = build_dimos_gap_report(
        {
            "ok": True,
            "gates": gates,
            "missing_or_failed": [],
            "algorithm_validation": {"claim_allowed": True, "flow_ok": True},
        },
        source="test",
        runtime_dataflow=runtime_dataflow,
        include_summary=False,
    )

    assert [row["gate"] for row in report["gap_matrix"]] == list(DIMOS_BENCHMARK_REQUIRED_GATES)
    assert report["gap_counts"]["failed"] == 0
    assert report["lingtu_readiness"]["ok"] is True
    assert "execution_plan" not in report


def test_host_setup_plan_contains_failures_without_commands() -> None:
    plan = _host_setup_plan(
        {
            "ok": False,
            "gates": {
                "saved_map_relocalize": {
                    "checks": {
                        "saved_map_input": {
                            "ok": False,
                            "blocker": "saved map missing",
                        }
                    }
                }
            },
        }
    )

    assert plan["failed_checks"][0]["check"] == "saved_map_input"
    assert "diagnostic_commands" not in plan["failed_checks"][0]
