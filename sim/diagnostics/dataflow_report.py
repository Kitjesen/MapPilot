"""Read current native simulation gate reports without launching runtimes."""

from __future__ import annotations

import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any

RUNTIME_DATAFLOW_GATES = {
    "bbs3d_kidnapped_relocalize",
    "navigation_replay_deviation",
    "saved_map_relocalize",
}
_SUPPORTED_SCHEMAS = {
    "bbs3d_kidnapped_relocalize": "lingtu.saved_map_relocalize_runtime.",
    "navigation_replay_deviation": "lingtu.navigation_replay_deviation_gate.",
    "saved_map_relocalize": "lingtu.saved_map_relocalize_runtime.",
}


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _report_path(gate: Mapping[str, Any], action: Mapping[str, Any], root: Path) -> Path | None:
    raw = gate.get("path") or action.get("report_path") or action.get("expected_report_path")
    if not raw:
        return None
    path = Path(str(raw))
    return path if path.is_absolute() else root / path


def _flow(report: Mapping[str, Any]) -> tuple[list[dict[str, Any]], str]:
    raw = report.get("runtime_dataflow")
    if not isinstance(raw, list) or not raw:
        return [], "runtime_dataflow_missing"

    flow: list[dict[str, Any]] = []
    for item in raw:
        if not isinstance(item, Mapping):
            return [], "runtime_dataflow_invalid"
        edge = dict(item)
        if not isinstance(edge.get("id"), str) or not edge["id"].strip():
            return [], "runtime_dataflow_invalid"
        if not isinstance(edge.get("ok"), bool):
            return [], "runtime_dataflow_invalid"
        if "evidence" in edge and not isinstance(edge["evidence"], Mapping):
            return [], "runtime_dataflow_invalid"
        flow.append(edge)
    return flow, ""


def runtime_dataflow_for_gate(
    gate_name: str,
    gate: Mapping[str, Any],
    action: Mapping[str, Any],
    *,
    root: Path,
) -> dict[str, Any]:
    """Return the dataflow evidence already published by one native gate."""

    path = _report_path(gate, action, root)
    if path is None or not path.is_file():
        return {
            "checked": False,
            "ok": False,
            "reason": "report_missing",
            "source_gate_report": "" if path is None else str(path),
            "source_report": "" if path is None else str(path),
            "candidate_reports": [],
        }

    try:
        report = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return {
            "checked": False,
            "ok": False,
            "reason": f"report_unreadable: {exc}",
            "source_gate_report": str(path),
            "source_report": str(path),
        }
    if not isinstance(report, Mapping):
        return {
            "checked": False,
            "ok": False,
            "reason": "report_not_object",
            "source_gate_report": str(path),
            "source_report": str(path),
        }

    schema = str(report.get("schema_version") or "")
    expected_schema = _SUPPORTED_SCHEMAS.get(gate_name, "")
    if expected_schema and not schema.startswith(expected_schema):
        return {
            "checked": False,
            "ok": False,
            "reason": "unsupported_report_shape",
            "schema_detected": schema,
            "source_gate_report": str(path),
            "source_report": str(path),
        }

    flow, flow_error = _flow(report)
    if flow_error:
        return {
            "checked": False,
            "ok": False,
            "reason": flow_error,
            "schema_detected": str(report.get("schema_version") or ""),
            "source_gate_report": str(path),
            "source_report": str(path),
            "candidate_reports": [],
        }

    blockers = [
        str(item) for item in [*(report.get("blockers") or []), *(report.get("remaining_gaps") or [])] if str(item)
    ]
    return {
        "checked": True,
        "ok": report.get("ok") is True and all(edge["ok"] for edge in flow),
        "schema_detected": schema,
        "primary_blocker": "" if report.get("ok") is True else (blockers[0] if blockers else "gate_failed"),
        "failed_edges": [
            str(edge.get("id") or "") for edge in flow if edge.get("ok") is False and str(edge.get("id") or "")
        ],
        "flow": flow,
        "claim_boundary": report.get("claim_boundary") or "",
        "environment": _mapping(report.get("environment")),
        "source_gate_report": str(path),
        "source_report": str(path),
        "candidate_reports": [],
    }


def build_runtime_dataflow_from_summary(
    summary: Mapping[str, Any],
    *,
    root: Path,
    gates: set[str] | None = None,
) -> dict[str, Any]:
    """Build gate-keyed dataflow from the current diagnostics summary."""

    summary_gates = _mapping(summary.get("gates"))
    validation = _mapping(summary.get("algorithm_validation"))
    actions = {
        str(action["gate"]): dict(action)
        for action in validation.get("next_actions") or []
        if isinstance(action, Mapping) and action.get("gate")
    }
    return {
        name: runtime_dataflow_for_gate(
            name,
            _mapping(summary_gates.get(name)),
            actions.get(name, {}),
            root=root,
        )
        for name in sorted(gates or RUNTIME_DATAFLOW_GATES)
    }
