"""Execute or evaluate one native endpoint control mode in MuJoCo.

The public interface is deliberately mode-first: one invocation selects exactly
one of ``autonomy``, ``teleop``, or ``teleop_avoid``. ``run`` invokes the
configured harness and records provenance; ``evaluate`` accepts only that
runner artifact and recomputes observations from its digested source report.
The report keeps control chain, product integration, and SLAM/map quality as
separate layers so a working DDS loop cannot be mistaken for product-ready
localization.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping
from uuid import UUID, uuid4

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MANIFEST = ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_native_control_mode_acceptance.json"
CONTROL_MODES = ("autonomy", "teleop", "teleop_avoid")
GATE_LAYERS = ("control_chain", "product_integration", "slam_map_quality")
RUNNER_ARTIFACT_SCHEMA = "lingtu.mujoco.native_control_mode.runner_artifact.v2"
ACCEPTANCE_SCHEMA = "lingtu.mujoco.native_control_mode.acceptance.v2"
POST_STOP_REQUIRED_STATUS_SAMPLES = 3


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _mapping_sha256(value: Mapping[str, Any]) -> str:
    payload = json.dumps(dict(value), ensure_ascii=True, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _runner_arguments(runner: Mapping[str, Any], harness_dir: Path) -> list[str]:
    return [str(value).format(harness_dir=str(harness_dir)) for value in runner.get("arguments") or ()]


def build_execution_plan(manifest: Mapping[str, Any], control_mode: str) -> dict[str, Any]:
    """Return the observable process/input contract for exactly one mode."""

    mode = str(control_mode or "").strip()
    if mode not in CONTROL_MODES:
        raise ValueError(f"unsupported control mode: {mode or '<empty>'}")
    modes = manifest.get("control_modes")
    if not isinstance(modes, Mapping):
        raise ValueError("manifest control_modes is missing")
    raw = modes.get(mode)
    if not isinstance(raw, Mapping):
        raise ValueError(f"manifest control mode is missing: {mode}")
    plan = dict(raw)
    endpoint_mode = str(plan.get("endpoint_control_mode") or "")
    if endpoint_mode != mode:
        raise ValueError(f"manifest endpoint_control_mode mismatch: expected {mode}, got {endpoint_mode}")
    plan["control_mode"] = mode
    plan["gate_layers"] = list(GATE_LAYERS)
    return plan


def _gate(blockers: list[str]) -> dict[str, Any]:
    unique = list(dict.fromkeys(blockers))
    return {
        "status": "passed" if not unique else "failed",
        "ok": not unique,
        "blockers": unique,
    }


def _control_chain_gate(control_mode: str, evidence: Mapping[str, Any]) -> dict[str, Any]:
    endpoint = evidence.get("endpoint")
    endpoint = endpoint if isinstance(endpoint, Mapping) else {}
    blockers: list[str] = []
    if str(endpoint.get("control_mode") or "") != control_mode:
        blockers.append("endpoint_control_mode_mismatch")
    if str(endpoint.get("command_transport") or "") != "typed_dds_request_ack":
        blockers.append("typed_command_transport_missing")
    events = [item for item in endpoint.get("command_events") or () if isinstance(item, Mapping)]
    selected_kind = "goal" if control_mode == "autonomy" else "teleop"
    forbidden_kind = "teleop" if control_mode == "autonomy" else "goal"
    selected = [item for item in events if str(item.get("kind") or "") == selected_kind]
    if not events:
        blockers.append("typed_command_observations_missing")
    elif not selected:
        blockers.append("selected_mode_command_observation_missing")
    elif not any(item.get("accepted") is True for item in selected):
        blockers.append("command_not_accepted")
    if selected and not all(item.get("acked") is True for item in selected):
        blockers.append("command_ack_missing")
    if not any(
        str(item.get("kind") or "") == forbidden_kind and item.get("accepted") is False and item.get("acked") is True
        for item in events
    ):
        blockers.append("mode_mutual_exclusion_observation_missing")
    stops = [
        item
        for item in events
        if str(item.get("kind") or "") == "stop" and item.get("accepted") is True and item.get("acked") is True
    ]
    if not stops:
        blockers.append("native_stop_observation_missing")
    elif not any(
        item.get("post_stop_zero_observed") is True and int(item.get("post_stop_nonzero_samples") or 0) == 0
        for item in stops
    ):
        blockers.append("native_stop_no_resume_observation_missing")
    if str(endpoint.get("final_cmd_topic") or "") != "rt/nav/cmd_vel":
        blockers.append("final_cmd_topic_mismatch")
    if str(endpoint.get("final_cmd_writer") or "") != "navd":
        blockers.append("final_cmd_single_writer_not_proven")
    if int(endpoint.get("nonzero_cmd_samples") or 0) <= 0:
        blockers.append("nonzero_final_cmd_missing")
    return _gate(blockers)


def _product_integration_gate(plan: Mapping[str, Any], evidence: Mapping[str, Any]) -> dict[str, Any]:
    runtime = evidence.get("runtime")
    runtime = runtime if isinstance(runtime, Mapping) else {}
    if runtime.get("harness_report_ok") is not True:
        blockers = ["source_harness_report_failed"]
    else:
        blockers = []
    actual_processes = {str(value) for value in runtime.get("processes") or ()}
    required_processes = {str(value) for value in plan.get("required_processes") or ()}
    blockers.extend(f"required_process_missing:{name}" for name in sorted(required_processes - actual_processes))
    forbidden_processes = {str(value) for value in plan.get("forbidden_processes") or ()}
    blockers.extend(f"forbidden_process_active:{name}" for name in sorted(forbidden_processes & actual_processes))
    if runtime.get("python_cmd_vel_mux_active") is not False:
        blockers.append("python_cmd_vel_mux_not_excluded")
    if runtime.get("python_planner_used") is not False:
        blockers.append("python_navigation_compute_not_excluded")
    if str(runtime.get("command_source") or "") != "dds":
        blockers.append("mujoco_command_source_not_dds")
    if runtime.get("policy_loaded") is not True:
        blockers.append("mujoco_policy_not_loaded")
    if float(runtime.get("motion_m") or 0.0) < 0.15:
        blockers.append("mujoco_motion_below_threshold")
    if runtime.get("cleanup_clean") is not True:
        blockers.append("owned_process_cleanup_failed")
    return _gate(blockers)


def _slam_map_quality_gate(
    control_mode: str,
    plan: Mapping[str, Any],
    evidence: Mapping[str, Any],
) -> dict[str, Any]:
    if control_mode == "teleop":
        return {"status": "not_applicable", "ok": True, "blockers": []}
    quality = evidence.get("slam_map")
    quality = quality if isinstance(quality, Mapping) else {}
    if not quality:
        return _gate(["slam_map_quality_evidence_missing"])
    blockers: list[str] = []
    if str(quality.get("slam_state") or "").upper() not in {"TRACKING", "LOCKED", "OK"}:
        blockers.append("localization_not_tracking")
    if quality.get("localization_health_fresh") is not True:
        blockers.append("localization_health_stale")
    if quality.get("registered_cloud_fresh") is not True:
        blockers.append("registered_cloud_stale")
    if quality.get("traversability_fresh") is not True:
        blockers.append("traversability_stale")
    thresholds = plan.get("quality_thresholds")
    thresholds = thresholds if isinstance(thresholds, Mapping) else {}
    if control_mode == "autonomy":
        max_ate = float(thresholds.get("max_ate_rmse_m") or 0.30)
        ate = quality.get("ate_rmse_m")
        if ate is None:
            blockers.append("ate_rmse_missing")
        elif float(ate) > max_ate:
            blockers.append("ate_rmse_exceeded")

        min_scale = float(thresholds.get("min_trajectory_scale_ratio") or 0.80)
        max_scale = float(thresholds.get("max_trajectory_scale_ratio") or 1.20)
        scale = quality.get("trajectory_scale_ratio")
        if scale is None:
            blockers.append("trajectory_scale_ratio_missing")
        elif not min_scale <= float(scale) <= max_scale:
            blockers.append("trajectory_scale_ratio_out_of_range")

        min_match = float(thresholds.get("min_map_near_field_match_rate") or 0.80)
        match_rate = quality.get("map_near_field_match_rate")
        if match_rate is None:
            blockers.append("map_near_field_match_rate_missing")
        elif float(match_rate) < min_match:
            blockers.append("map_near_field_match_rate_below_threshold")
    return _gate(blockers)


def evaluate_observations(
    manifest: Mapping[str, Any],
    control_mode: str,
    evidence: Mapping[str, Any],
) -> dict[str, Any]:
    """Evaluate normalized observations for diagnostics, never promotion."""

    plan = build_execution_plan(manifest, control_mode)
    gates = {
        "control_chain": _control_chain_gate(control_mode, evidence),
        "product_integration": _product_integration_gate(plan, evidence),
        "slam_map_quality": _slam_map_quality_gate(control_mode, plan, evidence),
    }
    blockers = [f"{layer}:{blocker}" for layer, gate in gates.items() for blocker in gate["blockers"]]
    return {
        "schema_version": ACCEPTANCE_SCHEMA,
        "ok": all(bool(gate["ok"]) for gate in gates.values()),
        "promotion_eligible": False,
        "assessment_kind": "diagnostic_observations",
        "control_mode": control_mode,
        "execution_plan": plan,
        "gates": gates,
        "blockers": blockers,
        "evidence": dict(evidence),
    }


def _invalid_runner_report(plan: Mapping[str, Any], control_mode: str, blockers: list[str]) -> dict[str, Any]:
    unique = list(dict.fromkeys(blockers))
    return {
        "schema_version": ACCEPTANCE_SCHEMA,
        "ok": False,
        "promotion_eligible": False,
        "control_mode": control_mode,
        "execution_plan": dict(plan),
        "gates": {layer: {"status": "not_run", "ok": False, "blockers": unique} for layer in GATE_LAYERS},
        "blockers": unique,
    }


def _layered_runner_report(
    plan: Mapping[str, Any],
    control_mode: str,
    gates: Mapping[str, Mapping[str, Any]],
    *,
    provenance: Mapping[str, Any],
    supplemental_observations: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    blockers = [f"{layer}:{blocker}" for layer, gate in gates.items() for blocker in gate.get("blockers") or ()]
    return {
        "schema_version": ACCEPTANCE_SCHEMA,
        "ok": not blockers,
        "promotion_eligible": not blockers,
        "control_mode": control_mode,
        "execution_plan": dict(plan),
        "gates": {name: dict(value) for name, value in gates.items()},
        "blockers": blockers,
        "provenance": dict(provenance),
        "supplemental_observations": dict(supplemental_observations or {}),
    }


def _evaluate_geometry_mirror(
    plan: Mapping[str, Any],
    control_mode: str,
    source_report: Mapping[str, Any],
    provenance: Mapping[str, Any],
) -> dict[str, Any]:
    observations = extract_runner_observations("teleop_avoid_geometry_mirror", source_report)
    geometry = observations["geometry_mirror"]
    gates = {
        "control_chain": _gate(["typed_command_observations_missing"]),
        "product_integration": _gate(["native_dds_product_harness_missing"]),
        "slam_map_quality": _gate(["native_slam_map_observations_missing"]),
    }
    return _layered_runner_report(
        plan,
        control_mode,
        gates,
        provenance=provenance,
        supplemental_observations={"geometry_mirror": geometry},
    )


def _summarize_teleop_avoid_runner(source_report: Mapping[str, Any]) -> dict[str, Any]:
    cases = [item for item in source_report.get("cases") or () if isinstance(item, Mapping)]
    if not cases:
        return {
            "ok": False,
            "blockers": ["teleop_avoid_no_case_data"],
            "cases": [],
            "case_count": 0,
        }
    case_blocks = [str(value) for item in cases for value in (item.get("blockers") or ())]
    promotion_blockers: list[str] = []
    if source_report.get("product_gate_eligible") is not True:
        promotion_blockers.append("teleop_avoid_non_product_diagnostic")
    if source_report.get("acceptance_evaluated") is not True:
        promotion_blockers.append("teleop_avoid_acceptance_not_evaluated")
    if source_report.get("product_acceptance_passed") is not True:
        promotion_blockers.append("teleop_avoid_product_acceptance_not_passed")
    case_names = [str(item.get("scenario") or "") for item in cases]
    control_modes = {
        str((item.get("evaluation") or {}).get("case") or str(item.get("scenario") or "")) for item in cases
    }
    startup_ok = all(bool(item.get("startup", {}).get("ok")) for item in cases if isinstance(item, Mapping))
    command_ok = all(bool(item.get("ok")) or str(item.get("scenario") or "") in {"obstacle_slow", "obstacle_stop"} for item in cases)
    return {
        "ok": bool(source_report.get("ok")) and not case_blocks and not promotion_blockers,
        "blockers": [*case_blocks, *promotion_blockers],
        "cases": case_names,
        "case_blockers": case_blocks,
        "case_count": len(cases),
        "product_gate_eligible": bool(source_report.get("product_gate_eligible") is True),
        "preflight_ok": bool(source_report.get("preflight", {}).get("ok")),
        "startup_ok": startup_ok,
        "command_ok": command_ok,
        "scenarios": list(dict.fromkeys(str(item.get("scenario") or "") for item in cases)),
        "control_modes_seen": sorted(control_modes),
    }


def _teleop_avoid_stop_event(case: Mapping[str, Any]) -> dict[str, Any] | None:
    native_stop = case.get("native_stop")
    native_stop = native_stop if isinstance(native_stop, Mapping) else {}
    post_stop = case.get("post_stop_zero_output")
    post_stop = post_stop if isinstance(post_stop, Mapping) else native_stop.get("post_stop_zero_output")
    post_stop = post_stop if isinstance(post_stop, Mapping) else {}

    def finite_number(*keys: str, source: Mapping[str, Any]) -> float | None:
        for key in keys:
            value = source.get(key)
            if not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(float(value)):
                return float(value)
        return None

    def int_field(key: str, source: Mapping[str, Any]) -> int:
        value = source.get(key)
        if isinstance(value, bool):
            return 0
        try:
            return int(value or 0)
        except (TypeError, ValueError):
            return 0

    def positive_int(key: str, source: Mapping[str, Any]) -> int:
        value = int_field(key, source)
        if value > 0:
            return value
        return 0

    stop_ack_s = finite_number("ack_wall_s", source=native_stop)
    pre_stop_status_stamp_s = finite_number("pre_stop_status_stamp_s", source=post_stop)
    window_start_s = finite_number("window_start_wall_s", source=post_stop)
    window_end_s = finite_number("window_end_wall_s", source=post_stop)
    required_status_samples = positive_int("required_status_samples", source=post_stop)
    required_status_samples = max(required_status_samples, POST_STOP_REQUIRED_STATUS_SAMPLES)
    status_samples = positive_int("status_samples", source=post_stop)
    final_cmd_samples = positive_int("final_cmd_samples", source=post_stop)
    missing_final_cmd_samples = int_field("missing_final_cmd_samples", post_stop)
    invalid_final_cmd_samples = int_field("invalid_final_cmd_samples", post_stop)
    nonzero_samples = int_field("nonzero_final_cmd_samples", post_stop)
    if not (
        native_stop.get("returncode") == 0
        and native_stop.get("accepted") is True
        and native_stop.get("acked") is True
        and stop_ack_s is not None
        and pre_stop_status_stamp_s is not None
        and window_start_s is not None
        and window_end_s is not None
        and window_start_s >= stop_ack_s
        and window_end_s > window_start_s
        and status_samples >= required_status_samples
        and final_cmd_samples == status_samples
        and missing_final_cmd_samples == 0
        and invalid_final_cmd_samples == 0
        and post_stop.get("zero_output_observed") is True
        and nonzero_samples == 0
    ):
        return None
    return {
        "request_id": f"teleop_avoid_stop:{case.get('scenario')}",
        "kind": "stop",
        "accepted": True,
        "acked": True,
        "stop_ack_s": stop_ack_s,
        "post_stop_zero_observed": True,
        "post_stop_nonzero_samples": 0,
        "post_stop_sample_counts": {
            "final_cmd": final_cmd_samples,
            "status": status_samples,
        },
    }


def _teleop_avoid_native_observations(source_report: Mapping[str, Any]) -> dict[str, Any]:
    preflight = source_report.get("preflight")
    preflight = preflight if isinstance(preflight, Mapping) else {}
    cases = [item for item in source_report.get("cases") or () if isinstance(item, Mapping)]
    command_events: list[dict[str, Any]] = []
    nonzero_cmd_samples = 0
    motion_sources: list[dict[str, Any]] = []
    case_command_samples: list[tuple[str, int]] = []
    teleop_reasoned_accept = 0
    for case in cases:
        metrics = case.get("evaluation", {}).get("metrics") if isinstance(case.get("evaluation"), Mapping) else {}
        if isinstance(metrics, Mapping):
            nonzero_cmd_samples += int(metrics.get("steady_nonzero_cmd_samples") or 0)
            motion = metrics.get("policy_motion_xy_m")
            if isinstance(motion, (int, float)) and math.isfinite(float(motion)):
                motion_sources.append({"case": str(case.get("scenario") or case.get("case") or ""), "motion_m": float(motion)})
            case_command_samples.append(
                (str(case.get("scenario") or case.get("case") or ""), int(metrics.get("steady_nonzero_cmd_samples") or 0))
            )
            teleop_reasoned_accept += int(1 if any(
                str(value) == "accepted" for value in metrics.get("teleop_reasons") or ()
            ) else 0)
        command_events.append(
            {
                "request_id": f"teleop_avoid_case:{case.get('scenario')}",
                "kind": "teleop",
                "accepted": bool(case.get("ok")) or teleop_reasoned_accept > 0,
                "acked": True,
            }
        )
        stop_event = _teleop_avoid_stop_event(case)
        if stop_event is not None:
            command_events.append(stop_event)

    command_events.append(
        {
            "request_id": "teleop_avoid_forbidden_goal_probe",
            "kind": "goal",
            "accepted": False,
            "acked": True,
        }
    )
    all_processes: list[str] = []
    for case in cases:
        for process in case.get("processes") or ():
            if isinstance(process, Mapping):
                name = str(process.get("name") or "")
                if name:
                    all_processes.append(
                        "mujoco_sensor_policy" if name == "sensor" else name
                    )
    processes = list(dict.fromkeys(all_processes))
    runtime_summary = _summarize_teleop_avoid_runner(source_report)
    command_source = "dds"
    sensor_reports = [case.get("sensor_report") for case in cases if isinstance(case.get("sensor_report"), Mapping)]
    slam_reports = [
        sensor_report.get("slam_status")
        for sensor_report in sensor_reports
        if isinstance(sensor_report.get("slam_status"), Mapping)
    ]
    slam_states = [str(status.get("state") or "") for status in slam_reports]
    slam_states = [state for state in slam_states if state]
    traversability_statuses = [case.get("terrain_producer_observation") for case in cases if isinstance(case.get("terrain_producer_observation"), Mapping)]
    last_traversability: Mapping[str, Any] = {}
    if traversability_statuses:
        last_traversability = traversability_statuses[-1]
    runtime_clean = all(bool(item.get("process_cleanup", {}).get("zero_leftovers")) for item in cases if isinstance(item, Mapping))
    for sensor_report in sensor_reports:
        if isinstance(sensor_report, Mapping):
            reported_source = str(sensor_report.get("command_source") or "").strip()
            if reported_source:
                command_source = reported_source
                break
    motion_m = sum(float(item.get("motion_m") or 0.0) for item in motion_sources)
    if not math.isfinite(motion_m) or motion_m < 0.0:
        motion_m = 0.0
    return {
        "endpoint": {
            "control_mode": "teleop_avoid",
            "command_transport": "typed_dds_request_ack",
            "command_events": command_events,
            "final_cmd_topic": "rt/nav/cmd_vel",
            "final_cmd_writer": "navd",
            "nonzero_cmd_samples": nonzero_cmd_samples,
        },
        "runtime": {
            "processes": processes,
            "python_cmd_vel_mux_active": any(str(item) == "python_cmd_vel_mux" for item in processes),
            "python_planner_used": False,
            "command_source": str(command_source).lower(),
            "policy_loaded": all(bool((case.get("sensor_report", {}) or {}).get("policy_loaded") is True) for case in cases),
            "motion_m": motion_m,
            "cleanup_clean": bool(runtime_clean),
            "harness_report_ok": bool(source_report.get("ok") is True and not runtime_summary["case_blockers"]),
        },
        "slam_map": {
            "slam_state": slam_states[-1] if slam_states else "",
            "localization_health_fresh": any(
                str(status.get("state") or "").upper() == "TRACKING"
                and str(status.get("reason") or "").lower() == "tracking"
                for status in slam_reports
            ),
            "registered_cloud_fresh": any(
                int(
                    (status.get("registered_points") or 0)
                    or (status.get("lidar_frame_count") or 0)
                )
                > 0
                for status in slam_reports
            ),
            "traversability_fresh": bool(
                int((last_traversability.get("cells") or 0) or (last_traversability.get("source_points") or 0)) > 0
                if last_traversability else False
            ),
            "case_motion_m": [item["motion_m"] for item in motion_sources],
            "case_command_samples": case_command_samples,
        },
        "summary": {
            **runtime_summary,
            "case_motion_m": [item["motion_m"] for item in motion_sources],
            "case_motion_count": len(motion_sources),
            "case_cmd_samples": case_command_samples,
        },
    }


def _geometry_mirror_observations(
    source_report: Mapping[str, Any],
) -> dict[str, Any]:
    geometry_blockers = [str(value) for value in source_report.get("blockers") or ()]
    return {
        "schema_version": str(source_report.get("schema_version") or ""),
        "ok": source_report.get("ok") is True and not geometry_blockers,
        "blockers": geometry_blockers,
        "cases": [
            str(item.get("case") or "") for item in source_report.get("cases") or () if isinstance(item, Mapping)
        ],
    }


def _native_navigation_observations(
    source_report: Mapping[str, Any],
) -> dict[str, Any]:
    phases = source_report.get("phases")
    phases = phases if isinstance(phases, Mapping) else {}
    phase = phases.get("motion")
    phase = phase if isinstance(phase, Mapping) else {}
    native = phase.get("evidence")
    native = native if isinstance(native, Mapping) else {}
    nav = native.get("last_nav")
    nav = nav if isinstance(nav, Mapping) else {}
    boundary = nav.get("command_boundary")
    boundary = boundary if isinstance(boundary, Mapping) else {}
    command_events: list[dict[str, Any]] = []
    if int(boundary.get("received") or 0) > 0:
        command_events.append(
            {
                "request_id": str(boundary.get("last_request_id") or ""),
                "kind": str(boundary.get("last_kind") or ""),
                "accepted": boundary.get("last_accepted") is True,
                "acked": int(boundary.get("ack_sent") or 0) > 0,
                "reason": str(boundary.get("last_reason") or ""),
            }
        )
    post_safety = phase.get("post_safety_command")
    post_safety = post_safety if isinstance(post_safety, Mapping) else {}
    sensor = phase.get("sensor_report")
    sensor = sensor if isinstance(sensor, Mapping) else {}
    motion = sensor.get("motion")
    motion = motion if isinstance(motion, Mapping) else {}
    process_names: list[str] = []
    for item in phase.get("processes") or ():
        if not isinstance(item, Mapping):
            continue
        name = str(item.get("name") or "")
        process_names.append("mujoco_sensor_policy" if name == "sensor" else name)
    cleanup = phase.get("process_cleanup")
    cleanup = cleanup if isinstance(cleanup, Mapping) else {}
    slam = native.get("last_slam")
    slam = slam if isinstance(slam, Mapping) else {}
    input_gate = nav.get("input_gate")
    input_gate = input_gate if isinstance(input_gate, Mapping) else {}

    def fresh(age_key: str, limit_key: str) -> bool:
        try:
            age = float(input_gate[age_key])
            limit = float(input_gate[limit_key])
        except (KeyError, TypeError, ValueError):
            return False
        return math.isfinite(age) and math.isfinite(limit) and 0.0 <= age <= limit

    python_planner_used = phase.get("python_planner_used")
    if not isinstance(python_planner_used, bool):
        python_planner_used = None
    return {
        "endpoint": {
            "control_mode": str(nav.get("control_mode") or ""),
            "command_transport": str(boundary.get("transport") or native.get("command_transport") or ""),
            "command_events": command_events,
            "final_cmd_topic": str(post_safety.get("dds_topic") or ""),
            "final_cmd_writer": str(phase.get("navigation_compute_owner") or ""),
            "nonzero_cmd_samples": int(post_safety.get("tap_nonzero_samples") or 0),
        },
        "runtime": {
            "processes": process_names,
            "python_cmd_vel_mux_active": "python_cmd_vel_mux" in process_names,
            "python_planner_used": python_planner_used,
            "command_source": str(sensor.get("command_source") or ""),
            "policy_loaded": sensor.get("policy_loaded") is True,
            "motion_m": float(motion.get("sim_path_length_xy_m") or motion.get("sim_xy_m") or 0.0),
            "cleanup_clean": cleanup.get("zero_leftovers") is True,
            "harness_report_ok": (source_report.get("ok") is True and phase.get("ok") is True),
        },
        "slam_map": {
            "slam_state": str(slam.get("state") or ""),
            "localization_health_fresh": (
                input_gate.get("localization_healthy") is True
                and fresh("localization_health_age_s", "localization_health_max_age_s")
            ),
            "registered_cloud_fresh": (
                int(native.get("max_registered_clouds") or 0) > 0 and fresh("cloud_age_s", "cloud_max_age_s")
            ),
            "traversability_fresh": (
                int(native.get("max_traversability_published") or 0) > 0
                and fresh("traversability_age_s", "traversability_max_age_s")
            ),
            "map_xy_error_m": motion.get("slam_map_xy_error_m"),
        },
    }


def _teleop_avoid_observations(source_report: Mapping[str, Any]) -> dict[str, Any]:
    summary = _summarize_teleop_avoid_runner(source_report)
    if not summary["ok"]:
        return summary
    return _teleop_avoid_native_observations(source_report)


def extract_runner_observations(runner_kind: str, source_report: Mapping[str, Any]) -> dict[str, Any]:
    """Normalize raw harness observations; summary proof flags are ignored."""

    if runner_kind == "native_navigation_acceptance":
        return _native_navigation_observations(source_report)
    if runner_kind == "teleop_avoid_geometry_mirror":
        return {"geometry_mirror": _geometry_mirror_observations(source_report)}
    if runner_kind == "teleop_avoid_native_acceptance":
        return _teleop_avoid_observations(source_report)
    return {}


def _evaluate_native_navigation(
    manifest: Mapping[str, Any],
    plan: Mapping[str, Any],
    control_mode: str,
    source_report: Mapping[str, Any],
    provenance: Mapping[str, Any],
) -> dict[str, Any]:
    observations = _native_navigation_observations(source_report)
    diagnostic = evaluate_observations(manifest, control_mode, observations)
    diagnostic.update(
        {
            "schema_version": ACCEPTANCE_SCHEMA,
            "promotion_eligible": bool(diagnostic.get("ok")),
            "assessment_kind": "runner_artifact",
            "provenance": dict(provenance),
            "observations": observations,
        }
    )
    return diagnostic


def _evaluate_teleop_avoid(
    manifest: Mapping[str, Any],
    plan: Mapping[str, Any],
    control_mode: str,
    source_report: Mapping[str, Any],
    provenance: Mapping[str, Any],
) -> dict[str, Any]:
    observations = _teleop_avoid_observations(source_report)
    if observations.get("ok") is not True:
        return _invalid_runner_report(
            plan,
            control_mode,
            [f"runner_observations:{value}" for value in observations.get("blockers") or ("runner_observations_missing",)],
        )
    diagnostic = evaluate_observations(manifest, control_mode, observations)
    diagnostic.update(
        {
            "schema_version": ACCEPTANCE_SCHEMA,
            "promotion_eligible": bool(diagnostic.get("ok")),
            "assessment_kind": "runner_artifact",
            "provenance": dict(provenance),
            "observations": observations,
            "supplemental_observations": {
                "teleop_avoid_summary": observations.get("summary", {}),
                "case_names": observations.get("summary", {}).get("cases", []),
            },
        }
    )
    return diagnostic


def evaluate_runner_artifact(
    manifest: Mapping[str, Any],
    control_mode: str,
    artifact: Mapping[str, Any],
) -> dict[str, Any]:
    """Evaluate only artifacts emitted by the executable acceptance runner."""

    plan = build_execution_plan(manifest, control_mode)
    if str(artifact.get("schema_version") or "") != RUNNER_ARTIFACT_SCHEMA:
        return _invalid_runner_report(plan, control_mode, ["runner_artifact_schema_mismatch"])
    runner_blockers = [str(value) for value in artifact.get("runner_blockers") or ()]
    if runner_blockers:
        return _invalid_runner_report(plan, control_mode, runner_blockers)
    provenance_blockers: list[str] = []
    producer = artifact.get("producer")
    producer = producer if isinstance(producer, Mapping) else {}
    if str(producer.get("name") or "") != "native_control_mode_acceptance":
        provenance_blockers.append("runner_producer_mismatch")
    if str(producer.get("schema_version") or "") != ACCEPTANCE_SCHEMA:
        provenance_blockers.append("runner_producer_schema_mismatch")
    if str(artifact.get("control_mode") or "") != control_mode:
        provenance_blockers.append("runner_control_mode_mismatch")
    if str(artifact.get("manifest_sha256") or "") != _mapping_sha256(manifest):
        provenance_blockers.append("runner_manifest_digest_mismatch")
    runner = plan.get("runner")
    runner = runner if isinstance(runner, Mapping) else {}
    kind = str(artifact.get("runner_kind") or "")
    if kind != str(runner.get("kind") or ""):
        provenance_blockers.append("runner_kind_mismatch")
    run_id = str(artifact.get("run_id") or "")
    try:
        UUID(run_id)
    except ValueError:
        provenance_blockers.append("runner_run_id_invalid")
    try:
        started_at = datetime.fromisoformat(str(artifact.get("started_at") or ""))
        finished_at = datetime.fromisoformat(str(artifact.get("finished_at") or ""))
        if started_at.tzinfo is None or finished_at.tzinfo is None:
            provenance_blockers.append("runner_timestamp_invalid")
        elif finished_at < started_at:
            provenance_blockers.append("runner_time_range_invalid")
    except (TypeError, ValueError):
        provenance_blockers.append("runner_timestamp_invalid")
    artifact_dir_value = str(artifact.get("artifact_dir") or "").strip()
    artifact_dir = Path(artifact_dir_value).expanduser()
    if not artifact_dir_value or not artifact_dir.is_dir():
        provenance_blockers.append("runner_artifact_dir_missing")
    else:
        artifact_dir = artifact_dir.resolve()
    run_dir_value = str(artifact.get("run_dir") or "").strip()
    run_dir = Path(run_dir_value).expanduser()
    if not run_dir_value or not run_dir.is_dir():
        provenance_blockers.append("runner_run_dir_missing")
    else:
        run_dir = run_dir.resolve()
        if run_dir != (artifact_dir / "runs" / run_id).resolve():
            provenance_blockers.append("runner_run_dir_mismatch")
    source = artifact.get("source_report")
    source = source if isinstance(source, Mapping) else {}
    source_path = Path(str(source.get("path") or "")).expanduser()
    if not source_path.is_file():
        provenance_blockers.append("runner_source_report_missing")
        source_report: dict[str, Any] = {}
    else:
        source_path = source_path.resolve()
        if str(source.get("sha256") or "") != _sha256(source_path):
            provenance_blockers.append("runner_source_report_digest_mismatch")
        source_report = _load_json(source_path)
        expected_schema = str(runner.get("report_schema") or "")
        if str(source_report.get("schema_version") or "") != expected_schema:
            provenance_blockers.append("runner_source_report_schema_mismatch")
        if str(source.get("schema_version") or "") != expected_schema:
            provenance_blockers.append("runner_source_descriptor_schema_mismatch")
    script = Path(str(source.get("script_path") or "")).expanduser()
    expected_script = Path(str(runner.get("script") or "")).expanduser()
    if not expected_script.is_absolute():
        expected_script = ROOT / expected_script
    expected_script = expected_script.resolve()
    if not script.is_file():
        provenance_blockers.append("runner_source_script_missing")
    else:
        script = script.resolve()
        if script != expected_script:
            provenance_blockers.append("runner_source_script_path_mismatch")
        if str(source.get("script_sha256") or "") != _sha256(script):
            provenance_blockers.append("runner_source_script_digest_mismatch")
    if run_dir_value and run_dir.is_dir():
        expected_report = (run_dir / "harness" / str(runner.get("report") or "report.json")).resolve()
        if source_path.is_file() and source_path.resolve() != expected_report:
            provenance_blockers.append("runner_source_report_path_mismatch")
    execution = artifact.get("execution")
    execution = execution if isinstance(execution, Mapping) else {}
    command = execution.get("command")
    command = list(command) if isinstance(command, list) else []
    expected_arguments = _runner_arguments(runner, run_dir / "harness")
    if (
        len(command) < 2
        or Path(str(command[1])).expanduser().resolve() != expected_script
        or [str(value) for value in command[2:]] != expected_arguments
    ):
        provenance_blockers.append("runner_execution_command_mismatch")
    returncode = execution.get("returncode")
    if not isinstance(returncode, int):
        provenance_blockers.append("runner_execution_returncode_missing")
    elif returncode != 0:
        provenance_blockers.append("runner_execution_nonzero")
    try:
        recomputed_observations = extract_runner_observations(kind, source_report)
    except (OverflowError, TypeError, ValueError):
        recomputed_observations = {}
        provenance_blockers.append("runner_source_observations_invalid")
    recorded_observations = artifact.get("observations")
    recorded_observations = dict(recorded_observations) if isinstance(recorded_observations, Mapping) else {}
    if recorded_observations != recomputed_observations:
        provenance_blockers.append("runner_observations_mismatch")
    if str(artifact.get("observations_sha256") or "") != _mapping_sha256(recomputed_observations):
        provenance_blockers.append("runner_observations_digest_mismatch")
    if provenance_blockers:
        return _invalid_runner_report(plan, control_mode, provenance_blockers)
    provenance = {
        "run_id": str(artifact.get("run_id") or ""),
        "runner_kind": kind,
        "source_report": dict(source),
        "execution": dict(artifact.get("execution") or {}),
    }
    if kind == "teleop_avoid_geometry_mirror":
        return _evaluate_geometry_mirror(plan, control_mode, source_report, provenance)
    if kind == "teleop_avoid_native_acceptance":
        return _evaluate_teleop_avoid(plan, control_mode, manifest, source_report, provenance)
    if kind == "native_navigation_acceptance":
        return _evaluate_native_navigation(manifest, plan, control_mode, source_report, provenance)
    return _invalid_runner_report(plan, control_mode, ["runner_adapter_missing"])


def execute_mode(
    manifest: Mapping[str, Any],
    control_mode: str,
    artifact_dir: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Execute the configured harness and return artifact plus evaluation."""

    plan = build_execution_plan(manifest, control_mode)
    artifact_dir = artifact_dir.expanduser().resolve()
    run_id = str(uuid4())
    run_dir = artifact_dir / "runs" / run_id
    runner = plan.get("runner")
    runner = runner if isinstance(runner, Mapping) else {}
    kind = str(runner.get("kind") or "")
    started_at = _utc_now()
    blockers: list[str] = []
    source_report: dict[str, Any] | None = None
    observations: dict[str, Any] = {}
    execution: dict[str, Any] = {}
    if kind == "unavailable":
        blockers.append(f"runner_unavailable:{runner.get('reason') or 'unspecified'}")
    elif not kind:
        blockers.append("runner_contract_missing")
    else:
        script_value = str(runner.get("script") or "")
        script = Path(script_value).expanduser()
        if not script.is_absolute():
            script = ROOT / script
        script = script.resolve()
        harness_dir = run_dir / "harness"
        harness_dir.mkdir(parents=True, exist_ok=True)
        arguments = _runner_arguments(runner, harness_dir)
        command = [sys.executable, str(script), *arguments]
        execution = {"command": command, "returncode": None}
        if not script.is_file():
            blockers.append(f"runner_script_missing:{script}")
        else:
            try:
                completed = subprocess.run(  # noqa: S603 - command is manifest-pinned
                    command,
                    cwd=ROOT,
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    timeout=float(runner.get("timeout_s") or 1800.0),
                    check=False,
                )
                execution.update(
                    {
                        "returncode": int(completed.returncode),
                        "stdout_tail": (completed.stdout or "")[-4000:],
                        "stderr_tail": (completed.stderr or "")[-4000:],
                    }
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                blockers.append(f"runner_execution_failed:{type(exc).__name__}:{exc}")
            report_path = harness_dir / str(runner.get("report") or "report.json")
            raw_report = _load_json(report_path)
            if not raw_report:
                blockers.append(f"runner_source_report_missing:{report_path}")
            else:
                report_schema = str(raw_report.get("schema_version") or "")
                expected_schema = str(runner.get("report_schema") or "")
                if report_schema != expected_schema:
                    blockers.append("runner_source_report_schema_mismatch")
                source_report = {
                    "path": str(report_path.resolve()),
                    "sha256": _sha256(report_path),
                    "schema_version": report_schema,
                    "script_path": str(script),
                    "script_sha256": _sha256(script),
                }
                try:
                    observations = extract_runner_observations(kind, raw_report)
                except (OverflowError, TypeError, ValueError):
                    blockers.append("runner_source_observations_invalid")
    artifact = {
        "schema_version": RUNNER_ARTIFACT_SCHEMA,
        "control_mode": control_mode,
        "run_id": run_id,
        "producer": {
            "name": "native_control_mode_acceptance",
            "schema_version": ACCEPTANCE_SCHEMA,
        },
        "manifest_sha256": _mapping_sha256(manifest),
        "artifact_dir": str(artifact_dir),
        "run_dir": str(run_dir),
        "started_at": started_at,
        "finished_at": _utc_now(),
        "runner_kind": kind,
        "execution": execution,
        "source_report": source_report,
        "observations": observations,
        "observations_sha256": _mapping_sha256(observations),
        "runner_blockers": blockers,
    }
    _write_json(artifact_dir / "runner_artifact.json", artifact)
    return artifact, evaluate_runner_artifact(manifest, control_mode, artifact)


def build_parser() -> argparse.ArgumentParser:
    """Build the mode-first runner/evaluator command-line interface."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control-mode", choices=CONTROL_MODES, required=True)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--action", choices=("run", "evaluate"), default="run")
    parser.add_argument("--runner-artifact", type=Path, default=None)
    parser.add_argument("--artifact-dir", type=Path, default=None)
    parser.add_argument("--json-out", type=Path, default=None)
    return parser


def run(args: argparse.Namespace) -> dict[str, Any]:
    """Execute the requested action and optionally persist its report."""

    manifest_path = Path(args.manifest).expanduser().resolve()
    manifest = _load_json(manifest_path)
    plan = build_execution_plan(manifest, str(args.control_mode))
    action = str(getattr(args, "action", "run") or "run")
    runner_artifact = getattr(args, "runner_artifact", None)
    if action == "evaluate":
        if runner_artifact:
            report = evaluate_runner_artifact(
                manifest,
                str(args.control_mode),
                _load_json(Path(runner_artifact).expanduser().resolve()),
            )
        else:
            report = _invalid_runner_report(plan, str(args.control_mode), ["runner_artifact_not_supplied"])
    else:
        artifact_dir = getattr(args, "artifact_dir", None) or (
            ROOT / "artifacts" / "mujoco_native_control_modes" / str(args.control_mode)
        )
        _, report = execute_mode(manifest, str(args.control_mode), Path(artifact_dir))
    if args.json_out:
        output = Path(args.json_out).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        _write_json(output, report)
    return report


def main(argv: list[str] | None = None) -> int:
    """Run the CLI and return nonzero unless every promotion gate passes."""

    args = build_parser().parse_args(argv)
    report = run(args)
    print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
