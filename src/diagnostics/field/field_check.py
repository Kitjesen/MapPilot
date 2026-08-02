"""Product-facing field readiness summary.

Aggregates validation from multiple independent sources: Gateway acceptance,
saved-map artifacts, algorithm benchmarks, and runtime switch plans. This is a
cross-cutting field validation concern, not a Gateway-specific function. All
non-core imports are lazy; module-level imports stay limited to core utilities
and stdlib.

The underlying evidence remains Gateway acceptance, saved-map artifact validation,
and real-runtime-evidence.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

from maps.paths import active_map_dir
from runtime.runtime_interface import REAL_RUNTIME_EVIDENCE_LABEL

FIELD_CHECK_SCHEMA_VERSION = "lingtu.product_field_check.v1"
HARDWARE_COMMAND_SINK = "driver"
ALGORITHM_REQUIRED_MODES = ("simulation", "field")


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _status(ok: Any, *, unchecked: bool = False) -> str:
    if unchecked:
        return "UNCHECKED"
    return "PASS" if ok is True else "FAIL"


def _artifact_status(
    map_gate: Mapping[str, Any] | None,
    name: str,
    *aliases: str,
) -> str:
    if not isinstance(map_gate, Mapping):
        return "UNCHECKED"
    artifacts = _mapping(map_gate.get("artifacts"))
    artifact = {}
    for artifact_name in (name, *aliases):
        artifact = _mapping(artifacts.get(artifact_name))
        if artifact:
            break
    if not artifact:
        return "UNCHECKED"
    return _status(artifact.get("exists") is True and artifact.get("sha256_ok") is not False)


def _string_list(value: Any) -> list[str]:
    return [str(item) for item in (value or ()) if item]


def _active_map_artifact_dir() -> Path | None:
    return active_map_dir()


def _algorithm_benchmark_status(
    algorithm_gate: Mapping[str, Any] | None,
    *,
    mode: str,
) -> dict[str, Any]:
    required = mode in ALGORITHM_REQUIRED_MODES
    checked = isinstance(algorithm_gate, Mapping)
    if not checked:
        return {
            "status": "UNCHECKED",
            "required": required,
            "ok": False,
            "claim_allowed": False,
            "missing_or_failed": [],
            "summary_path": None,
            "report_age_s": None,
            "max_age_s": None,
            "read_only": True,
            "ros2_topic_required": False,
            "publishes": [],
            "reason": "algorithm_benchmark_not_checked",
            "blockers": ["algorithm benchmark latest summary was not checked"],
            "claim_boundary": {},
            "blocking_categories": {},
            "source": "server_sim_closure",
            "preset": "dimos_benchmark",
            "active_benchmark_variant": "inspection_mvp",
            "strict_benchmark_variant": "dimos_benchmark",
            "benchmark_variants": {},
        }

    blockers = _string_list(algorithm_gate.get("blockers"))
    missing_or_failed = _string_list(algorithm_gate.get("missing_or_failed"))
    claim_allowed = algorithm_gate.get("claim_allowed") is True
    read_only = algorithm_gate.get("read_only") is not False
    publishes = _string_list(algorithm_gate.get("publishes"))
    ros2_topic_required = algorithm_gate.get("ros2_topic_required") is True
    if not read_only:
        blockers.append("algorithm benchmark latest endpoint is not read-only")
    if publishes:
        blockers.append("algorithm benchmark latest endpoint publishes: " + ",".join(publishes))
    if ros2_topic_required:
        blockers.append("algorithm benchmark latest endpoint requires ROS2 topics")
    ok = (
        algorithm_gate.get("ok") is True
        and claim_allowed
        and not missing_or_failed
        and not blockers
        and read_only
        and not publishes
        and not ros2_topic_required
    )
    return {
        "status": "PASS" if ok else "FAIL",
        "required": required,
        "ok": ok,
        "claim_allowed": claim_allowed,
        "missing_or_failed": missing_or_failed,
        "summary_path": algorithm_gate.get("summary_path"),
        "report_age_s": algorithm_gate.get("report_age_s"),
        "max_age_s": algorithm_gate.get("max_age_s"),
        "read_only": read_only,
        "ros2_topic_required": ros2_topic_required,
        "publishes": publishes,
        "reason": algorithm_gate.get("reason"),
        "blockers": blockers,
        "claim_boundary": _mapping(algorithm_gate.get("claim_boundary")),
        "blocking_categories": _mapping(algorithm_gate.get("blocking_categories")),
        "source": algorithm_gate.get("source") or "server_sim_closure",
        "preset": algorithm_gate.get("preset") or "dimos_benchmark",
        "active_benchmark_variant": (algorithm_gate.get("active_benchmark_variant") or "inspection_mvp"),
        "strict_benchmark_variant": (algorithm_gate.get("strict_benchmark_variant") or "dimos_benchmark"),
        "benchmark_variants": _mapping(algorithm_gate.get("benchmark_variants")),
    }


def _runtime_switch_preflight_status(
    switch_plan: Mapping[str, Any] | None,
    *,
    mode: str,
) -> dict[str, Any]:
    required = mode in ALGORITHM_REQUIRED_MODES
    checked = isinstance(switch_plan, Mapping)
    if not checked:
        return {
            "status": "UNCHECKED",
            "required": required,
            "ok": False,
            "read_only": None,
            "dry_run": None,
            "motion": None,
            "publishes": [],
            "lifecycle": None,
            "from": {},
            "to": {},
            "changed": [],
            "blockers": ["runtime switch preflight was not checked"],
        }

    blockers = _string_list(switch_plan.get("blockers"))
    publishes = _string_list(switch_plan.get("publishes"))
    read_only = switch_plan.get("read_only") is True
    dry_run = switch_plan.get("dry_run") is True
    motion = switch_plan.get("motion") is True
    if not read_only or not dry_run:
        blockers.append("runtime switch preflight is not dry-run/read-only")
    if motion:
        blockers.append("runtime switch preflight is motion-capable")
    if publishes:
        blockers.append("runtime switch preflight publishes: " + ",".join(publishes))
    ok = switch_plan.get("ok") is True and read_only and dry_run and not motion and not publishes and not blockers
    return {
        "status": "PASS" if ok else "FAIL",
        "required": required,
        "ok": ok,
        "read_only": read_only,
        "dry_run": dry_run,
        "motion": motion,
        "publishes": publishes,
        "lifecycle": switch_plan.get("lifecycle"),
        "from": _mapping(switch_plan.get("from")),
        "to": _mapping(switch_plan.get("to")),
        "changed": _string_list(switch_plan.get("changed")),
        "blockers": list(dict.fromkeys(blockers)),
    }


def _runtime_graph_status() -> dict[str, Any]:
    try:
        from runtime.graph import validate_runtime_graph
    except Exception as exc:  # pragma: no cover - defensive diagnostic path
        return {
            "status": "FAIL",
            "ok": False,
            "issue_count": 1,
            "issues": [
                {
                    "code": "runtime_graph_import_failed",
                    "message": str(exc),
                    "scope": "runtime_graph",
                    "severity": "error",
                }
            ],
        }

    issues = validate_runtime_graph()
    return {
        "status": "PASS" if not issues else "FAIL",
        "ok": not issues,
        "issue_count": len(issues),
        "issues": [issue.as_dict() for issue in issues],
    }


def _default_switch_plan_request(mode: str) -> dict[str, Any]:
    del mode
    # Gateway owns one fixed Env; the active RunPlan determines the
    # current Product and ProductControl resolves the target in that same Env.
    return {"target_product": "explore"}


def build_product_field_check(
    gateway_acceptance: Mapping[str, Any],
    *,
    map_gate: Mapping[str, Any] | None = None,
    algorithm_gate: Mapping[str, Any] | None = None,
    switch_plan: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Build a one-screen field readiness payload from existing gate results."""

    mode = str(gateway_acceptance.get("mode") or "unknown")
    checks = _mapping(gateway_acceptance.get("checks"))
    gateway = _mapping(checks.get("gateway_contract"))
    runtime_mode = _mapping(checks.get("runtime_mode"))
    observability = _mapping(checks.get("gateway_observability"))
    stage_evidence = _mapping(checks.get("stage_evidence"))
    frontier_preview = _mapping(checks.get("frontier_preview"))
    readiness = _mapping(checks.get("readiness"))
    localization = _mapping(checks.get("localization"))
    navigation = _mapping(checks.get("navigation"))
    routecheck = _mapping(checks.get("routecheck_latest"))
    evidence = _mapping(checks.get("real_runtime_evidence"))
    algorithm_gate = (
        algorithm_gate if isinstance(algorithm_gate, Mapping) else _mapping(checks.get("algorithm_benchmark_latest"))
    )
    algorithm = _algorithm_benchmark_status(algorithm_gate, mode=mode)
    runtime_switch = _runtime_switch_preflight_status(switch_plan, mode=mode)
    runtime_graph = _runtime_graph_status()

    command_boundary_ok = (
        observability.get("arbitrary_publish_supported") is False
        and not observability.get("missing_command_interfaces")
        and not observability.get("unexpected_command_interfaces")
        and (
            gateway_acceptance.get("mode") != "field"
            or (runtime_mode.get("ok") is True and runtime_mode.get("command_sink") == HARDWARE_COMMAND_SINK)
        )
    )
    map_checked = isinstance(map_gate, Mapping)
    map_ok = map_gate.get("ok") is True if map_checked else None
    blockers = [str(item) for item in (gateway_acceptance.get("blockers") or ()) if item]
    if map_checked and map_gate.get("ok") is not True:
        blockers.extend(f"map: {item}" for item in (map_gate.get("blockers") or ()) if item)
    field_mode = gateway_acceptance.get("mode") == "field"
    if field_mode and routecheck.get("ok") is not True:
        blockers.append("route preview is not passing or unavailable")
    if field_mode and evidence.get("ok") is not True:
        blockers.append(f"{REAL_RUNTIME_EVIDENCE_LABEL} is not passing")
    if field_mode and frontier_preview and frontier_preview.get("ok") is not True:
        blockers.append("traversable frontier preview is not passing")
    if algorithm["required"] and algorithm["ok"] is not True:
        blockers.append("algorithm strict benchmark is not passing")
        blockers.extend(f"algorithm: {item}" for item in algorithm["blockers"])
    if switch_plan and runtime_switch["ok"] is not True:
        blockers.append("runtime switch preflight is not passing")
        blockers.extend(str(item) for item in runtime_switch["blockers"])
    if runtime_graph["ok"] is not True:
        blockers.append("runtime graph contract is not passing")
        blockers.extend(f"runtime_graph: {issue.get('code')}" for issue in runtime_graph["issues"])
    advisories = [str(item) for item in (gateway_acceptance.get("advisories") or ()) if item]
    if not map_checked:
        advisories.append("map provenance not checked; provide map dir when saved map is used")
    if not algorithm["required"] and algorithm["ok"] is not True:
        advisories.append("algorithm strict benchmark is not currently passing")
    if not switch_plan:
        advisories.append("runtime switch preflight not checked")

    payload = {
        "schema_version": FIELD_CHECK_SCHEMA_VERSION,
        "ok": not blockers,
        "mode": mode,
        "summary": "PASS" if not blockers else "FAIL",
        "map": {
            "active": str(map_gate.get("map_dir")) if map_checked else "unchecked",
            "provenance": _status(map_ok, unchecked=not map_checked),
            "octomap": _artifact_status(map_gate, "octomap"),
            "occupancy": _artifact_status(map_gate, "occupancy_grid", "occupancy"),
        },
        "runtime": {
            "gateway": _status(gateway.get("ok") is True),
            "readiness": _status(readiness.get("ok") is True),
            "localization": _status(localization.get("ok") is True),
            "gateway_observability": _status(observability.get("ok") is True),
            "stages": _status(stage_evidence.get("ok") is True),
            "command_boundary": _status(command_boundary_ok),
            "frontier_preview": _status(
                frontier_preview.get("ok") is True,
                unchecked=not bool(frontier_preview),
            ),
            "runtime_switch": runtime_switch["status"],
            "runtime_graph": runtime_graph["status"],
            "ros2_topic_required": gateway_acceptance.get("ros2_topic_required"),
            "arbitrary_publish_supported": observability.get("arbitrary_publish_supported"),
        },
        "stage_evidence": {
            "required": stage_evidence.get("required"),
            "required_stages": list(stage_evidence.get("required_stages") or ()),
            "live_stages": list(stage_evidence.get("live_stages") or ()),
            "missing_stages": list(stage_evidence.get("missing_stages") or ()),
            "not_live_stages": list(stage_evidence.get("not_live_stages") or ()),
            "not_live_stage_tokens": _mapping(stage_evidence.get("not_live_stage_tokens")),
            "non_observable_stages": list(stage_evidence.get("non_observable_stages") or ()),
            "missing_tokens": _mapping(stage_evidence.get("missing_tokens")),
        },
        "frontier_preview": {
            "status": _status(
                frontier_preview.get("ok") is True,
                unchecked=not bool(frontier_preview),
            ),
            "required": frontier_preview.get("required"),
            "read_only": frontier_preview.get("read_only"),
            "live": frontier_preview.get("live"),
            "payload_available": frontier_preview.get("payload_available"),
            "candidate_source": frontier_preview.get("candidate_source"),
            "candidate": _mapping(frontier_preview.get("candidate")),
            "frontier_count": frontier_preview.get("frontier_count"),
            "command_published": frontier_preview.get("command_published"),
            "blockers": list(frontier_preview.get("blockers") or ()),
        },
        "runtime_switch": runtime_switch,
        "runtime_graph": runtime_graph,
        "navigation": {
            "can_send_goal": _status(navigation.get("can_send_goal") is True),
            "route_preview": _status(routecheck.get("ok") is True),
            "route_preview_non_motion": routecheck.get("non_motion"),
            "route_preview_published": _mapping(routecheck.get("published")),
            "local_path": _status(evidence.get("data_flow_ok") is True),
            "cmd_vel_mux": _status(evidence.get("cmd_vel_sent_to_hardware") is True),
        },
        "evidence": {
            "field_runtime": _status(evidence.get("ok") is True),
            "age_s": evidence.get("report_age_s"),
            "mode": mode,
            "runtime_contract": evidence.get("runtime_contract"),
            "map": dict(map_gate) if map_checked else None,
        },
        "algorithm": {
            "active_benchmark_variant": algorithm["active_benchmark_variant"],
            "strict_benchmark_variant": algorithm["strict_benchmark_variant"],
            "benchmark_variants": algorithm["benchmark_variants"],
            "strict_benchmark": algorithm,
        },
        "blockers": list(dict.fromkeys(blockers)),
        "advisories": list(dict.fromkeys(advisories)),
        "commands": {
            "runtime_audit": "python lingtu.py runtime-audit",
            "real_runtime_evidence": (
                "python lingtu.py real-runtime-evidence --collector gateway "
                "--gateway-url http://<robot>:5050 --duration-sec 20 "
                "--json-out artifacts/real_runtime/report.json"
            ),
            "gateway_field_acceptance": (
                "python lingtu.py gateway-runtime-acceptance --acceptance-mode field --gateway-url http://<robot>:5050"
            ),
            "gateway_sim_acceptance": (
                "python lingtu.py gateway-runtime-acceptance --acceptance-mode simulation "
                "--gateway-url http://<server>:5050"
            ),
            "algorithm_benchmark": (
                "PYTHONPATH=src:. python sim/scripts/server_sim_closure.py "
                "--preset dimos_benchmark "
                "--json-out artifacts/server_sim_closure/summary_dimos_benchmark_24h.json"
            ),
            "runtime_switch_plan": ("python lingtu.py switch-plan sim_mujoco_live explore"),
            "runtime_graph": "python -m pytest src/runtime/tests/test_runtime_graph_contract.py -q",
        },
    }
    return payload


def collect_product_field_check(
    *,
    gateway_url: str,
    timeout_sec: float,
    mode: str,
    map_dir: str | None = None,
    require_octomap: bool = False,
    require_occupancy: bool = False,
    expected_data_source: str | None = None,
    expected_source_profile: str | None = None,
    expected_frame_id: str | None = None,
) -> dict[str, Any]:
    """Collect existing gates and return the product field summary."""

    from diagnostics.field.gateway_acceptance import collect_gateway_runtime_acceptance
    from gateway.services.runtime_switch_plan import build_runtime_switch_plan

    # Note: gateway/service imports above are lazy (inside function body)
    # to keep core/ pure at module level.

    gateway_acceptance = collect_gateway_runtime_acceptance(
        gateway_url=gateway_url,
        timeout_sec=timeout_sec,
        mode=mode,
    )
    switch_plan = build_runtime_switch_plan(_default_switch_plan_request(mode))
    map_gate = None
    resolved_map_dir = map_dir or _active_map_artifact_dir()
    if resolved_map_dir:
        from diagnostics.field.gates import runtime_validation_gates
        from maps.artifacts import (
            validate_saved_map_artifact_dir,
        )

        map_gate = validate_saved_map_artifact_dir(
            Path(resolved_map_dir),
            require_octomap=require_octomap,
            require_occupancy=require_occupancy,
            expected_data_source=expected_data_source,
            expected_source_profile=expected_source_profile,
            expected_frame_id=expected_frame_id,
        )
        map_gate["validation_gate"] = runtime_validation_gates()["saved_map_artifact_gate"]
    return build_product_field_check(
        gateway_acceptance,
        map_gate=map_gate,
        switch_plan=switch_plan,
    )
