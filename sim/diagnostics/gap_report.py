"""DimOS-style benchmark gap matrix for LingTu algorithm evidence."""

from __future__ import annotations

import json
import time
from typing import Any, Mapping

from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES
from sim.diagnostics.dataflow_report import RUNTIME_DATAFLOW_GATES

SCHEMA_VERSION = "lingtu.dimos_gap_report.v1"

DIMOS_REFERENCE_SURFACES: dict[str, dict[str, str]] = {
    "gateway_runtime_acceptance": {
        "surface": "operator data plane",
        "requirement": "Product-visible runtime streams, command whitelist, and non-motion acceptance are available before a demo claim.",
    },
    "navigation_replay_deviation": {
        "surface": "offline replay/deviation",
        "requirement": "Recorded global path, local path, cmd_vel, and odometry replay stay within endpoint and tracking deviation bounds without hardware output.",
    },
    "saved_map_relocalize": {
        "surface": "saved-map lifecycle",
        "requirement": "Saved map relocalization proves same-source map use and localization health before saved-map navigation.",
    },
    "bbs3d_kidnapped_relocalize": {
        "surface": "global relocalization",
        "requirement": "Native BBS3D recovers a displaced simulated robot against the saved map.",
    },
}

DIMOS_SOURCE_LINKS = {
    "dimos_repository": "https://github.com/dimensionalOS/dimos",
    "lingtu_gap_matrix": "docs/research/dimos_benchmark_matrix.md",
    "lingtu_validation_flow": "docs/07-testing/simulation/ALGORITHM_VALIDATION_FLOW.md",
}
PIPELINE_TRACE = {
    "schema_version": "lingtu.dimos_pipeline_trace.v1",
    "claim_boundary": ("Code path trace only. Runtime proof still requires passing DimOS gates."),
    "primary_chain": [
        {
            "id": "offline_navigation_replay",
            "role": "Routecheck-derived or recorded navigation traces prove path, cmd_vel, odometry, and deviation consistency without hardware output",
            "code": [
                "sim/scripts/navigation_replay_deviation_gate.py",
                "sim/scripts/sim_diagnostics.py",
            ],
            "runtime_evidence_gate": "navigation_replay_deviation",
        },
        {
            "id": "native_navigation",
            "role": "Native navd owns planning, local avoidance, command arbitration, and simulated motion",
            "code": [
                "src/nav/cpp/service/",
                "sim/scripts/mujoco/native_navigation_acceptance.py",
            ],
            "runtime_evidence_gate": "navigation_replay_deviation",
        },
        {
            "id": "saved_map_relocalize",
            "role": "Saved-map relocalization prepares the current product navigation map",
            "code": ["sim/scripts/saved_map_relocalize_runtime_gate.py"],
            "runtime_evidence_gate": "saved_map_relocalize",
        },
        {
            "id": "bbs3d_kidnapped_relocalize",
            "role": "Native BBS3D recovers map pose after a simulated displacement",
            "code": ["sim/scripts/saved_map_relocalize_runtime_gate.py"],
            "runtime_evidence_gate": "bbs3d_kidnapped_relocalize",
        },
    ],
    "gate_to_trace": {
        "navigation_replay_deviation": ["offline_navigation_replay", "native_navigation"],
        "saved_map_relocalize": ["saved_map_relocalize"],
        "bbs3d_kidnapped_relocalize": ["bbs3d_kidnapped_relocalize"],
    },
}

BLOCKER_ACTIONS = {
    "product_data_plane": "fix the Gateway/runtime data-plane report, then rerun the gate",
    "environment_runtime": "fix unavailable native runtime dependencies, then rerun host preflight",
    "artifact_contract": "generate a fresh report at the expected path with strict freshness enabled",
    "slam_localization": "debug native localization stability, then rerun the native relocalization gate",
    "planning_tracking": "debug route progress, local path tracking, checkpoint completion, and cmd_vel continuity",
    "command_safety": "fix the simulation-only command boundary before accepting the report",
    "simulation_integration": "fix the simulator adapter/topic/frame contract, then rerun the integration gate",
    "unclassified": "inspect the gate report and rerun after the blocker is made explicit",
}
HOST_PREFLIGHT_ACTION = "fix host preflight before running this gate"
HOST_PREFLIGHT_REPORT_ACTION = "regenerate a fresh host preflight report before using DimOS gate evidence"
SUMMARY_STALE_ACTION = "regenerate a fresh DimOS benchmark summary before claiming readiness"
SUMMARY_STALE_BLOCKER = "algorithm benchmark summary is stale"
HOST_PREFLIGHT_SCHEMA_VERSION = "lingtu.sim_diagnostics.host_preflight.v1"
HOST_PREFLIGHT_EXECUTION_MODE = "host_preflight_only"
HOST_CHECK_ACTIONS = {
    "mujoco_headless": ("install the MuJoCo Python runtime and set MUJOCO_GL to egl or osmesa"),
    "local_non_motion": "keep local non-motion gates fresh on the current Python host",
}
HOST_CHECK_PRIORITY = (
    "mujoco_headless",
)

CATEGORY_PRIORITY = (
    "product_data_plane",
    "environment_runtime",
    "artifact_contract",
    "slam_localization",
    "planning_tracking",
    "command_safety",
    "simulation_integration",
    "unclassified",
)

def _jsonable(value: Any) -> Any:
    return json.loads(json.dumps(value, ensure_ascii=False, default=str))


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _categories_for_gate(
    gate_name: str,
    gate: Mapping[str, Any],
    gate_categories: Mapping[str, Any],
    missing_or_failed: set[str],
) -> list[str]:
    categories = [str(item) for item in gate_categories.get(gate_name, []) if str(item)]
    if not categories and gate_name in missing_or_failed:
        blockers = " ".join(str(item).lower() for item in gate.get("blockers") or [])
        if any(
            item in blockers
            for item in ("abi", "native runtime", "python_tag", "mingw numpy", "numpy")
        ):
            categories = ["environment_runtime"]
        elif any(item in blockers for item in ("fast-lio", "fastlio", "slam", "localization")):
            categories = ["slam_localization"]
        elif any(
            item in blockers for item in ("replay", "deviation", "tracking", "cmd_vel", "replan", "blocked_route")
        ):
            categories = ["planning_tracking"]
        elif blockers:
            categories = ["artifact_contract"]
    if not categories and gate_name in missing_or_failed:
        categories = ["artifact_contract"]
    return categories


def _first_category(categories: list[str]) -> str:
    for category in CATEGORY_PRIORITY:
        if category in categories:
            return category
    return categories[0] if categories else "unclassified"


def _gate_status(gate_name: str, gate: Mapping[str, Any], missing_or_failed: set[str]) -> str:
    if gate.get("ok") is True and gate_name not in missing_or_failed:
        return "pass"
    status = str(gate.get("status") or "").lower()
    if status in {"missing", "failed", "stale", "error"}:
        return status
    if not gate:
        return "missing"
    return "failed"


def _priority(gate_name: str, status: str, categories: list[str]) -> str:
    if status == "pass":
        return "low"
    if "environment_runtime" in categories or "slam_localization" in categories:
        return "p1"
    if "artifact_contract" in categories:
        return "p2"
    return "p3"


def _priority_rank(priority: str) -> int:
    return {"p0": 0, "p1": 1, "p2": 2, "p3": 3, "low": 4}.get(priority, 9)


def _gate_order_rank(gate_name: str) -> int:
    try:
        return DIMOS_BENCHMARK_REQUIRED_GATES.index(gate_name)
    except ValueError:
        return len(DIMOS_BENCHMARK_REQUIRED_GATES)


def _host_check_rank(check_name: str) -> int:
    try:
        return HOST_CHECK_PRIORITY.index(check_name)
    except ValueError:
        return len(HOST_CHECK_PRIORITY)


def _stage_ids(validation_flow: list[Any], gate_name: str) -> list[str]:
    stages: list[str] = []
    for stage in validation_flow:
        if not isinstance(stage, dict):
            continue
        gates = {str(item) for item in stage.get("required_gates") or stage.get("all_gates") or []}
        if gate_name in gates:
            stages.append(str(stage.get("id") or stage.get("title") or ""))
    return [stage for stage in stages if stage]


def _next_action_by_gate(validation: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    actions: dict[str, dict[str, Any]] = {}
    for action in validation.get("next_actions") or []:
        if isinstance(action, dict) and action.get("gate"):
            actions[str(action["gate"])] = action
    return actions


def _host_setup_plan(host_preflight: Mapping[str, Any]) -> dict[str, Any]:
    if not host_preflight:
        return {"checked": False}

    gates = _mapping(host_preflight.get("gates"))
    checks_by_name: dict[str, dict[str, Any]] = {}
    for gate_name, raw_gate in gates.items():
        gate = _mapping(raw_gate)
        for check_name, raw_check in _mapping(gate.get("checks")).items():
            check = _mapping(raw_check)
            if check.get("ok") is not False:
                continue
            entry = checks_by_name.setdefault(
                str(check_name),
                {
                    "check": str(check_name),
                    "gates": [],
                    "blockers": [],
                    "evidence_samples": [],
                    "recommended_action": (
                        str(check.get("recommended_action") or "")
                        or HOST_CHECK_ACTIONS.get(
                            str(check_name),
                            "inspect the failed host check and fix the simulation host",
                        )
                    ),
                },
            )
            entry["gates"].append(str(gate_name))
            blocker = str(check.get("blocker") or "")
            if blocker:
                entry["blockers"].append(blocker)
            evidence = _mapping(check.get("evidence"))
            if evidence and len(entry["evidence_samples"]) < 3:
                entry["evidence_samples"].append(_jsonable(evidence))

    failed_checks = sorted(
        checks_by_name.values(),
        key=lambda item: (_host_check_rank(str(item["check"])), str(item["check"])),
    )
    for item in failed_checks:
        item["gates"] = list(dict.fromkeys(item["gates"]))
        item["blockers"] = list(dict.fromkeys(item["blockers"]))

    return {
        "checked": True,
        "source": "recomputed_from_host_preflight_gates",
        "ok": host_preflight.get("ok") is True and not failed_checks,
        "current_host": _jsonable(host_preflight.get("current_host") or {}),
        "ready_to_run_gates": list(host_preflight.get("runnable_gates") or []),
        "blocked_gates": list(host_preflight.get("blocked_gates") or []),
        "failed_check_count": len(failed_checks),
        "failed_checks": failed_checks,
        "stop_condition": (
            "host can run the selected DimOS gates"
            if not failed_checks
            else (
                "fix host checks before running blocked DimOS gates: "
                + ", ".join(str(item["check"]) for item in failed_checks)
            )
        ),
    }


def _host_preflight_contract_blockers(host_preflight: Mapping[str, Any]) -> list[str]:
    if not host_preflight:
        return []
    blockers: list[str] = []
    if host_preflight.get("ok") is not True:
        return blockers
    if host_preflight.get("report_contract_checked") is not True:
        blockers.append("host preflight report contract was not checked")
    if host_preflight.get("schema_version") != HOST_PREFLIGHT_SCHEMA_VERSION:
        blockers.append("schema_version is not lingtu.sim_diagnostics.host_preflight.v1")
    if host_preflight.get("execution_mode") != HOST_PREFLIGHT_EXECUTION_MODE:
        blockers.append("execution_mode is not host_preflight_only")
    required_sequence = tuple(str(item) for item in host_preflight.get("required_gate_sequence") or ())
    if required_sequence != tuple(DIMOS_BENCHMARK_REQUIRED_GATES):
        blockers.append("required_gate_sequence does not match DimOS required gates")
    if host_preflight.get("blocked_gates"):
        blockers.append("blocked_gates is not empty")
    freshness = _mapping(host_preflight.get("report_freshness"))
    if freshness.get("checked") is not True:
        blockers.append("host preflight report freshness was not checked")
    elif freshness.get("fresh") is not True:
        freshness_blockers = [str(item) for item in freshness.get("blockers") or [] if str(item)]
        if freshness_blockers:
            blockers.extend(freshness_blockers)
        else:
            blockers.append("host preflight report is stale")
    return list(dict.fromkeys(blockers))


def _host_preflight_with_contract_gate(
    host_preflight: Mapping[str, Any],
) -> dict[str, Any]:
    payload = dict(host_preflight)
    blockers = _host_preflight_contract_blockers(payload)
    if not blockers:
        return payload

    payload["ok"] = False
    payload["blocked_gates"] = list(
        dict.fromkeys(
            [
                *[str(item) for item in payload.get("blocked_gates") or []],
                *DIMOS_BENCHMARK_REQUIRED_GATES,
            ]
        )
    )
    gates = {str(gate_name): dict(_mapping(raw_gate)) for gate_name, raw_gate in _mapping(payload.get("gates")).items()}
    blocker_text = "; ".join(blockers)
    for gate_name in DIMOS_BENCHMARK_REQUIRED_GATES:
        gate = gates.setdefault(
            gate_name,
            {"ok": False, "status": "blocked", "blockers": []},
        )
        gate["ok"] = False
        gate["status"] = "blocked"
        gate_blockers = list(gate.get("blockers") or [])
        if blocker_text not in gate_blockers:
            gate_blockers.append(blocker_text)
        gate["blockers"] = gate_blockers
        checks = dict(_mapping(gate.get("checks")))
        checks["host_preflight_report_freshness"] = {
            "ok": False,
            "blocker": blocker_text,
            "recommended_action": HOST_PREFLIGHT_REPORT_ACTION,
        }
        gate["checks"] = checks
        gates[gate_name] = gate
    payload["gates"] = gates
    return payload


def _runtime_dataflow_blocker(brief: Mapping[str, Any]) -> str:
    if not brief:
        return ""
    reason = str(brief.get("reason") or "")
    primary = str(brief.get("primary_blocker") or "")
    failed_edges = [str(edge) for edge in brief.get("failed_edges") or [] if str(edge)]
    if brief.get("checked") is False:
        return reason or primary or (failed_edges[0] if failed_edges else "")
    if brief.get("ok") is True:
        return ""
    return primary or (failed_edges[0] if failed_edges else "") or reason or "runtime_dataflow_failed"


def _evidence_blockers(
    *,
    host_blockers: list[str],
    host_failed_checks: list[str],
    runtime_brief: Mapping[str, Any],
    gate_blockers: list[str],
) -> list[dict[str, Any]]:
    blockers: list[dict[str, Any]] = []
    if host_blockers or host_failed_checks:
        blockers.append(
            {
                "source": "host_preflight",
                "blockers": list(host_blockers),
                "failed_checks": list(host_failed_checks),
            }
        )
    runtime_blocker = _runtime_dataflow_blocker(runtime_brief)
    if runtime_blocker:
        blockers.append(
            {
                "source": "runtime_dataflow",
                "blocker": runtime_blocker,
                "failed_edges": [str(edge) for edge in runtime_brief.get("failed_edges") or [] if str(edge)],
                "candidate_reports": list(runtime_brief.get("candidate_reports") or []),
            }
        )
    if gate_blockers:
        blockers.append({"source": "gate_report", "blockers": list(gate_blockers)})
    return blockers


def _runtime_dataflow_gates(
    runtime_dataflow: Mapping[str, Any] | None,
) -> dict[str, dict[str, Any]]:
    payload = _mapping(runtime_dataflow)
    if not payload:
        return {}
    gates = _mapping(payload.get("gates"))
    if gates:
        return {str(name): _mapping(value) for name, value in gates.items()}
    return {str(name): _mapping(value) for name, value in payload.items()}


def _runtime_dataflow_brief(
    gate_name: str,
    runtime_dataflow_gates: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    payload = _mapping(runtime_dataflow_gates.get(gate_name))
    if not payload:
        return {"checked": False}
    flow = [item for item in payload.get("flow") or [] if isinstance(item, Mapping)]
    failed_edges = [
        str(edge.get("id") or "") for edge in flow if edge.get("ok") is not True and str(edge.get("id") or "")
    ]
    edge_status = {str(edge.get("id")): edge.get("ok") is True for edge in flow if str(edge.get("id") or "")}
    edge_evidence = {
        str(edge.get("id")): _jsonable(_mapping(edge.get("evidence"))) for edge in flow if str(edge.get("id") or "")
    }
    same_source = _mapping(payload.get("same_source_provenance"))
    return {
        "checked": payload.get("checked", True) is not False,
        "ok": payload.get("ok") is True,
        "schema_detected": payload.get("schema_detected") or "",
        "primary_blocker": payload.get("primary_blocker") or "",
        "failed_edges": failed_edges,
        "edge_status": edge_status,
        "edge_evidence": edge_evidence,
        "same_source_provenance": same_source,
        "claim_boundary": payload.get("claim_boundary") or "",
        "environment": _jsonable(_mapping(payload.get("environment"))),
        "source_gate_report": payload.get("source_gate_report") or "",
        "source_report": payload.get("source_report") or "",
        "reason": payload.get("reason") or "",
        "candidate_reports": list(payload.get("candidate_reports") or []),
        "child_report_freshness": _jsonable(_mapping(payload.get("child_report_freshness"))),
    }


def _runtime_dataflow_summary(
    runtime_dataflow_gates: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    expected_gates = set(RUNTIME_DATAFLOW_GATES)
    if not runtime_dataflow_gates:
        missing = {gate: "runtime dataflow report missing" for gate in sorted(expected_gates)}
        return {
            "checked": False,
            "complete": False,
            "all_gates_ok": False,
            "unchecked_gates": {},
            "missing_runtime_dataflow_gates": missing,
            "missing_runtime_dataflow_gate_count": len(missing),
            "gate_failures": missing,
        }
    checked = {
        gate: _mapping(payload)
        for gate, payload in runtime_dataflow_gates.items()
        if _mapping(payload).get("checked", True) is not False
    }
    unchecked = {
        gate: str(payload.get("primary_blocker") or payload.get("reason") or "unchecked")
        for gate, payload in runtime_dataflow_gates.items()
        if _mapping(payload).get("checked", True) is False
    }
    failing = {
        gate: str(payload.get("primary_blocker") or payload.get("reason") or "unknown")
        for gate, payload in checked.items()
        if payload.get("ok") is not True
    }
    missing = {gate: "runtime dataflow report missing" for gate in sorted(expected_gates - set(runtime_dataflow_gates))}
    gate_failures = {**missing, **unchecked, **failing}
    complete = bool(runtime_dataflow_gates) and not missing and not unchecked
    all_gates_ok = complete and not failing
    return {
        "schema_version": "lingtu.dimos_runtime_dataflow.v1",
        "checked": True,
        "complete": complete,
        "all_gates_ok": all_gates_ok,
        "gate_count": len(runtime_dataflow_gates),
        "checked_gate_count": len(checked),
        "ok_gate_count": sum(1 for payload in checked.values() if payload.get("ok") is True),
        "unchecked_gate_count": len(unchecked),
        "unchecked_gates": unchecked,
        "missing_runtime_dataflow_gates": missing,
        "missing_runtime_dataflow_gate_count": len(missing),
        "failing_primary_blockers": failing,
        "gate_failures": gate_failures,
        "gates": _jsonable(runtime_dataflow_gates),
    }


def build_dimos_gap_report(
    summary: Mapping[str, Any],
    *,
    source: str,
    generated_at: float | None = None,
    gate_metadata: Mapping[str, Mapping[str, Any]] | None = None,
    host_preflight: Mapping[str, Any] | None = None,
    host_preflight_source: str | None = None,
    summary_freshness: Mapping[str, Any] | None = None,
    runtime_dataflow: Mapping[str, Any] | None = None,
    include_summary: bool = True,
) -> dict[str, Any]:
    """Build the canonical DimOS-style gap matrix from a diagnostics summary."""
    generated_at = time.time() if generated_at is None else generated_at
    metadata = gate_metadata or {}
    validation = _mapping(summary.get("algorithm_validation"))
    gates = _mapping(summary.get("gates"))
    missing_or_failed = {str(item) for item in summary.get("missing_or_failed") or [] if str(item)}
    gate_categories = _mapping(validation.get("gate_categories"))
    validation_flow = list(validation.get("validation_flow") or [])
    next_actions = _next_action_by_gate(validation)
    explicit_host_preflight_payload = _mapping(host_preflight)
    host_preflight_payload = explicit_host_preflight_payload
    host_preflight_source = host_preflight_source or ("argument" if host_preflight_payload else "")
    if host_preflight_payload:
        host_preflight_payload = _host_preflight_with_contract_gate(host_preflight_payload)
    host_setup_plan = _host_setup_plan(host_preflight_payload)
    host_preflight_gates = _mapping(host_preflight_payload.get("gates"))
    freshness_payload = _mapping(summary_freshness)
    runtime_dataflow_by_gate = _runtime_dataflow_gates(runtime_dataflow)
    runtime_dataflow_summary = _runtime_dataflow_summary(runtime_dataflow_by_gate)
    runtime_dataflow_checked = runtime_dataflow_summary.get("checked") is True
    runtime_dataflow_complete = runtime_dataflow_summary.get("complete") is True
    runtime_dataflow_gate_failures = _mapping(runtime_dataflow_summary.get("gate_failures"))
    runtime_dataflow_ok = (
        runtime_dataflow_checked
        and runtime_dataflow_complete
        and not runtime_dataflow_gate_failures
    )
    summary_is_stale = freshness_payload.get("fresh") is False or freshness_payload.get("stale") is True
    summary_stale_blocker = str(freshness_payload.get("blocker") or SUMMARY_STALE_BLOCKER)

    rows: list[dict[str, Any]] = []
    for index, gate_name in enumerate(DIMOS_BENCHMARK_REQUIRED_GATES, start=1):
        gate = _mapping(gates.get(gate_name))
        meta = _mapping(metadata.get(gate_name))
        preflight_gate = _mapping(host_preflight_gates.get(gate_name))
        gate_evidence = _mapping(gate.get("evidence"))
        host_blockers = [str(item) for item in preflight_gate.get("blockers") or []]
        preflight_checks = _mapping(preflight_gate.get("checks"))
        host_failed_checks = [
            str(check_name) for check_name, check in preflight_checks.items() if _mapping(check).get("ok") is False
        ]
        categories = _categories_for_gate(
            gate_name,
            gate,
            gate_categories,
            missing_or_failed,
        )
        if host_blockers and "environment_runtime" not in categories:
            categories.append("environment_runtime")
        if summary_is_stale and "artifact_contract" not in categories:
            categories.append("artifact_contract")
        status = (
            "stale"
            if summary_is_stale
            else _gate_status(
                gate_name,
                gate,
                missing_or_failed,
            )
        )
        primary_category = _first_category(categories)
        action = next_actions.get(gate_name, {})
        reference = DIMOS_REFERENCE_SURFACES.get(gate_name, {})
        blockers = list(gate.get("blockers") or action.get("blockers") or [])
        if summary_is_stale and summary_stale_blocker not in blockers:
            blockers.insert(0, summary_stale_blocker)
        if status != "pass" and not blockers:
            blockers = ["report missing or not verified"]
        runtime_brief = _runtime_dataflow_brief(
            gate_name,
            runtime_dataflow_by_gate,
        )
        runtime_blocker = _runtime_dataflow_blocker(runtime_brief)
        runtime_failed_edges = [str(edge) for edge in runtime_brief.get("failed_edges") or [] if str(edge)]
        rows.append(
            {
                "order": index,
                "gate": gate_name,
                "status": status,
                "ok": status == "pass",
                "priority": _priority(gate_name, status, categories),
                "dimos_surface": reference.get("surface", "benchmark evidence"),
                "dimos_requirement": reference.get("requirement", ""),
                "lingtu_gate_description": meta.get("description", ""),
                "validation_stages": _stage_ids(validation_flow, gate_name),
                "pipeline_trace": list(_mapping(PIPELINE_TRACE.get("gate_to_trace")).get(gate_name, [])),
                "runtime_dataflow": runtime_brief,
                "runtime_dataflow_blocker": runtime_blocker,
                "runtime_dataflow_failed_edges": runtime_failed_edges,
                "execution_mode": gate_evidence.get("execution_mode") or "",
                "environment": _jsonable(_mapping(gate_evidence.get("environment"))),
                "categories": categories,
                "primary_category": primary_category,
                "recommended_action": (
                    "keep this report fresh before making a claim"
                    if status == "pass"
                    else HOST_PREFLIGHT_ACTION
                    if host_blockers
                    else SUMMARY_STALE_ACTION
                    if summary_is_stale
                    else BLOCKER_ACTIONS.get(primary_category, BLOCKER_ACTIONS["unclassified"])
                ),
                "expected_report_path": (action.get("expected_report_path") or meta.get("expected_report_path", "")),
                "report_path": gate.get("path") or action.get("report_path") or "",
                "blockers": blockers,
                "host_requirements": list(action.get("host_requirements") or meta.get("host_requirements") or []),
                "host_preflight": {
                    "checked": bool(host_preflight_payload or preflight_gate),
                    "ok": preflight_gate.get("ok") if preflight_gate else None,
                    "status": preflight_gate.get("status", "") if preflight_gate else "",
                    "blockers": host_blockers,
                    "failed_checks": host_failed_checks,
                    "checks": preflight_checks,
                },
                "evidence_blockers": _evidence_blockers(
                    host_blockers=host_blockers,
                    host_failed_checks=host_failed_checks,
                    runtime_brief=runtime_brief,
                    gate_blockers=blockers if status != "pass" else [],
                ),
            }
        )

    failed_rows = [row for row in rows if not row["ok"]]
    prioritized_failed_rows = sorted(
        failed_rows,
        key=lambda row: (_priority_rank(str(row["priority"])), int(row["order"])),
    )
    actionable_failed_rows = prioritized_failed_rows
    gate_ordered_failed_rows = sorted(
        failed_rows,
        key=lambda row: (
            _gate_order_rank(str(row["gate"])),
            _priority_rank(str(row["priority"])),
            int(row["order"]),
        ),
    )
    host_preflight_failed = host_setup_plan.get("checked") is True and host_setup_plan.get("ok") is False
    if summary_is_stale:
        stop_condition = (
            "do not claim DimOS-style algorithm health until the benchmark "
            "summary is regenerated within the freshness limit"
        )
    elif host_preflight_failed and not failed_rows:
        stop_condition = "do not claim DimOS-style algorithm health until host preflight passes: " + ", ".join(
            str(item.get("check") or "")
            for item in host_setup_plan.get("failed_checks") or []
            if str(item.get("check") or "")
        )
    elif not runtime_dataflow_checked and not failed_rows:
        stop_condition = (
            "do not claim DimOS-style algorithm health until runtime dataflow is checked with include_dataflow"
        )
    elif not runtime_dataflow_complete and not failed_rows:
        stop_condition = (
            "do not claim DimOS-style algorithm health until runtime dataflow "
            "evidence is complete for these gates: " + ", ".join(runtime_dataflow_gate_failures)
        )
    elif runtime_dataflow_gate_failures and not failed_rows:
        stop_condition = (
            "do not claim DimOS-style algorithm health until runtime dataflow "
            "passes for these gates: " + ", ".join(runtime_dataflow_gate_failures)
        )
    else:
        stop_condition = (
            "all DimOS benchmark gates passed; simulation-only algorithm-health claim is allowed"
            if not failed_rows
            else (
                "do not claim DimOS-style algorithm health until these gates pass: "
                + ", ".join(str(row["gate"]) for row in prioritized_failed_rows)
            )
        )
    report = {
        "schema_version": SCHEMA_VERSION,
        "generated_at": generated_at,
        "source": source,
        "dimos_reference": {
            "model": "DimOS-style evidence parity, not feature-for-feature copying",
            "required_gate_sequence": list(DIMOS_BENCHMARK_REQUIRED_GATES),
            "source_links": DIMOS_SOURCE_LINKS,
        },
        "pipeline_trace": _jsonable(PIPELINE_TRACE),
        "lingtu_readiness": {
            "ok": (
                summary.get("ok") is True
                and validation.get("claim_allowed") is True
                and not summary_is_stale
                and not failed_rows
                and not host_preflight_failed
                and runtime_dataflow_checked
                and runtime_dataflow_complete
                and runtime_dataflow_ok
            ),
            "summary_ok": summary.get("ok") is True,
            "summary_fresh": not summary_is_stale,
            "claim_allowed": validation.get("claim_allowed") is True,
            "flow_ok": validation.get("flow_ok") is True,
            "runtime_dataflow_checked": runtime_dataflow_checked,
            "runtime_dataflow_complete": runtime_dataflow_complete,
            "runtime_dataflow_ok": runtime_dataflow_ok,
            "runtime_dataflow_gate_failures": list(runtime_dataflow_gate_failures),
            "host_preflight_ok": (
                None if host_setup_plan.get("checked") is not True else host_setup_plan.get("ok") is True
            ),
            "missing_or_failed": [row["gate"] for row in failed_rows],
            "highest_priority_blocker": (prioritized_failed_rows[0]["gate"] if prioritized_failed_rows else ""),
            "highest_actionable_blocker": (actionable_failed_rows[0]["gate"] if actionable_failed_rows else ""),
            "stop_condition": stop_condition,
            "source_stop_condition": validation.get("stop_condition") or "",
            "claim_boundary": validation.get("claim_boundary") or {},
        },
        "gap_counts": {
            "required": len(rows),
            "passed": len(rows) - len(failed_rows),
            "failed": len(failed_rows),
            "p0": sum(1 for row in failed_rows if row["priority"] == "p0"),
            "p1": sum(1 for row in failed_rows if row["priority"] == "p1"),
            "p2": sum(1 for row in failed_rows if row["priority"] == "p2"),
            "p3": sum(1 for row in failed_rows if row["priority"] == "p3"),
        },
        "gap_matrix": rows,
        "next_steps": [
            {
                "gate": row["gate"],
                "priority": row["priority"],
                "category": row["primary_category"],
                "recommended_action": row["recommended_action"],
                "expected_report_path": row["expected_report_path"],
                "host_preflight_ok": row["host_preflight"]["ok"],
                "host_preflight_blockers": row["host_preflight"]["blockers"],
                "host_failed_checks": row["host_preflight"]["failed_checks"],
                "runtime_dataflow_blocker": row["runtime_dataflow_blocker"],
                "runtime_dataflow_failed_edges": row["runtime_dataflow_failed_edges"],
                "evidence_blockers": row["evidence_blockers"],
            }
            for row in gate_ordered_failed_rows
        ],
    }
    if host_preflight_payload:
        report["host_preflight"] = {
            "checked": True,
            "ok": host_preflight_payload.get("ok") is True,
            "schema_version": host_preflight_payload.get("schema_version"),
            "execution_mode": host_preflight_payload.get("execution_mode"),
            "source": host_preflight_source,
            "current_host": host_preflight_payload.get("current_host") or {},
            "runnable_gates": list(host_preflight_payload.get("runnable_gates") or []),
            "blocked_gates": list(host_preflight_payload.get("blocked_gates") or []),
        }
    else:
        report["host_preflight"] = {"checked": False}
    report["host_setup_plan"] = host_setup_plan
    if freshness_payload:
        report["summary_freshness"] = {
            "checked": True,
            "fresh": not summary_is_stale,
            "report_age_s": freshness_payload.get("report_age_s")
            if freshness_payload.get("report_age_s") is not None
            else freshness_payload.get("age_s"),
            "max_age_s": freshness_payload.get("max_age_s"),
            "blocker": summary_stale_blocker if summary_is_stale else "",
        }
    else:
        report["summary_freshness"] = {"checked": False}
    report["runtime_dataflow"] = runtime_dataflow_summary
    if include_summary:
        report["summary"] = _jsonable(summary)
    return report
