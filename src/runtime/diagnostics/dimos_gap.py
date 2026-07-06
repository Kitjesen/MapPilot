"""DimOS-style benchmark gap matrix for LingTu algorithm evidence."""

from __future__ import annotations

import json
import time
from typing import Any, Mapping

from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES
from runtime.diagnostics.dimos_runtime_dataflow import RUNTIME_DATAFLOW_GATES

SCHEMA_VERSION = "lingtu.dimos_gap_report.v1"

DIMOS_REFERENCE_SURFACES: dict[str, dict[str, str]] = {
    "gateway_runtime_acceptance": {
        "surface": "operator data plane",
        "requirement": "Product-visible runtime streams, command whitelist, and non-motion acceptance are available before a demo claim.",
    },
    "routecheck_preflight": {
        "surface": "no-hardware route preview",
        "requirement": "Route preview and command counters prove planning intent without publishing motion commands.",
    },
    "blocked_route_replan_preflight": {
        "surface": "blocked-route replanning",
        "requirement": "A baseline route intersects a synthetic obstruction while the candidate preview replans around it without publishing motion commands.",
    },
    "navigation_replay_deviation": {
        "surface": "offline replay/deviation",
        "requirement": "Routecheck-derived or recorded global path, local path, cmd_vel, and odometry replay stay within endpoint and tracking deviation bounds without hardware output.",
    },
    "large_terrain": {
        "surface": "fixed waypoint mission matrix",
        "requirement": "Large saved terrain assets produce safe global routes over the map/tomogram source.",
    },
    "policy_nav": {
        "surface": "current product simulation route execution",
        "requirement": "OctoPlanner3D output reaches nanobind LocalPlanner, nav_kernel PathFollower, VelocityMux, and policy-mode MuJoCo motion without ROS2 local autonomy.",
    },
    "native_pct_mujoco": {
        "surface": "legacy PCT compatibility route execution",
        "requirement": "Legacy native PCT output reaches local planning, path following, and simulated motion without planner fallback; current product route execution is covered by policy_nav.",
    },
    "dynamic_obstacle_local_planner": {
        "surface": "local replanning unit gate",
        "requirement": "Local planner reacts to changing obstacle phases with measurable clearance and replan evidence.",
    },
    "fastlio2_dynamic_inspection": {
        "surface": "live sensor simulation",
        "requirement": "Raw LiDAR/IMU feed SLAM while global planning, local planning, and command output remain live.",
    },
    "moving_obstacle_sweep": {
        "surface": "dynamic replanning stress matrix",
        "requirement": "Moving-obstacle speed/density bins pass with live navigation-chain evidence and video artifacts.",
    },
    "large_loop_closure": {
        "surface": "long-range closed-loop route",
        "requirement": "A large same-source loop keeps localization drift bounded while PCT, local path, and cmd_vel remain active.",
    },
    "gazebo_runtime": {
        "surface": "ROS-native simulation integration",
        "requirement": "ROS/Gazebo adapter proves TF, map growth, navigation loop, frontier progress, and publisher identity.",
    },
    "saved_map_relocalize": {
        "surface": "saved-map lifecycle",
        "requirement": "Saved map relocalization proves same-source map use and localization health before saved-map navigation.",
    },
    "pct_saved_map_navigation": {
        "surface": "saved-map PCT execution",
        "requirement": "Saved map/tomogram drives PCT, localPlanner/pathFollower, and simulated motion after relocalization.",
    },
}

DIMOS_SOURCE_LINKS = {
    "dimos_repository": "https://github.com/dimensionalOS/dimos",
    "lingtu_gap_matrix": "docs/07-testing/DIMOS_BENCHMARK_GAP_MATRIX.md",
    "lingtu_validation_flow": "docs/07-testing/ALGORITHM_VALIDATION_FLOW.md",
}
PIPELINE_TRACE = {
    "schema_version": "lingtu.dimos_pipeline_trace.v1",
    "claim_boundary": (
        "Code path trace only. Runtime proof still requires passing DimOS gates."
    ),
    "primary_chain": [
        {
            "id": "global_planning_dispatch",
            "role": "Navigation selects GlobalPlanner and planner backend",
            "code": [
                "src/nav/mission/navigation.py",
                "src/nav/services/plan/global_planner/service.py",
            ],
            "runtime_evidence_gate": "large_terrain",
        },
        {
            "id": "pct_backend",
            "role": "PCT planner loads tomogram/native runtime and returns global path",
            "code": [
                "src/nav/services/plan/global_planner/algorithm/pct/planner.py",
                "src/nav/services/plan/global_planner/algorithm/pct/runtime/runtime.py",
            ],
            "runtime_evidence_gate": "large_terrain",
        },
        {
            "id": "octoplanner3d_backend",
            "role": "Navigation configures OctoPlanner3D as the current product global planner",
            "code": [
                "src/nav/services/plan/global_planner/service.py",
                "src/nav/services/plan/global_planner/algorithm/octoplanner3d.py",
                "src/nav/services/plan/global_planner/algorithm/OctoPlanner3D/",
            ],
            "runtime_evidence_gate": "policy_nav",
        },
        {
            "id": "blocked_route_replanning",
            "role": "Gateway plan preview proves route blocking changes the path without motion output",
            "code": [
                "sim/scripts/blocked_route_replan_gate.py",
                "src/gateway/gateway_module.py",
            ],
            "runtime_evidence_gate": "blocked_route_replan_preflight",
        },
        {
            "id": "offline_navigation_replay",
            "role": "Routecheck-derived or recorded navigation traces prove path, cmd_vel, odometry, and deviation consistency without hardware output",
            "code": [
                "sim/scripts/navigation_replay_deviation_gate.py",
                "sim/scripts/server_sim_closure.py",
            ],
            "runtime_evidence_gate": "navigation_replay_deviation",
        },
        {
            "id": "legacy_pct_to_local_autonomy",
            "role": "Legacy native PCT compatibility path is fed to localPlanner/pathFollower",
            "code": [
                "sim/scripts/mujoco/native_pct_gate.py",
                "src/nav/local/local_planner.py",
                "src/nav/local/path_follower.py",
            ],
            "runtime_evidence_gate": "native_pct_mujoco",
        },
        {
            "id": "octoplanner_to_inprocess_autonomy",
            "role": "OctoPlanner3D path is fed to in-process nanobind LocalPlanner and nav_kernel PathFollower",
            "code": [
                "sim/scripts/policy_nav_smoke.py",
                "src/nav/local/local_planner.py",
                "src/nav/local/path_follower.py",
            ],
            "runtime_evidence_gate": "policy_nav",
        },
        {
            "id": "mujoco_motion_executor",
            "role": "Path follower cmd_vel drives the MuJoCo policy-mode simulation endpoint",
            "code": [
                "src/drivers/sim/mujoco/driver.py",
                "src/drivers/sim/mujoco/runtime.py",
                "sim/scripts/mujoco/launch_fastlio2_live.sh",
            ],
            "runtime_evidence_gate": "policy_nav",
        },
        {
            "id": "live_fastlio_feedback",
            "role": "MuJoCo MID-360/IMU feed Fast-LIO2 and live navigation feedback",
            "code": [
                "sim/scripts/mujoco/live_gate.py",
                "sim/scripts/large_loop_closure_gate.py",
                "sim/scripts/moving_obstacle_sweep_gate.py",
            ],
            "runtime_evidence_gate": "fastlio2_dynamic_inspection",
        },
        {
            "id": "saved_map_relocalize_pct",
            "role": "Saved-map relocalization and same-source tomogram drive PCT navigation",
            "code": [
                "sim/scripts/saved_map_relocalize_runtime_gate.py",
                "sim/scripts/pct_saved_map_navigation_gate.py",
            ],
            "runtime_evidence_gate": "pct_saved_map_navigation",
        },
    ],
    "gate_to_trace": {
        "blocked_route_replan_preflight": ["blocked_route_replanning"],
        "navigation_replay_deviation": ["offline_navigation_replay"],
        "large_terrain": ["global_planning_dispatch", "pct_backend"],
        "native_pct_mujoco": [
            "pct_backend",
            "legacy_pct_to_local_autonomy",
            "mujoco_motion_executor",
        ],
        "policy_nav": [
            "octoplanner3d_backend",
            "octoplanner_to_inprocess_autonomy",
            "mujoco_motion_executor",
        ],
        "fastlio2_dynamic_inspection": [
            "live_fastlio_feedback",
            "pct_backend",
            "legacy_pct_to_local_autonomy",
        ],
        "moving_obstacle_sweep": [
            "live_fastlio_feedback",
            "pct_backend",
            "legacy_pct_to_local_autonomy",
            "mujoco_motion_executor",
        ],
        "large_loop_closure": [
            "live_fastlio_feedback",
            "pct_backend",
            "legacy_pct_to_local_autonomy",
            "mujoco_motion_executor",
        ],
        "saved_map_relocalize": ["saved_map_relocalize_pct", "live_fastlio_feedback"],
        "pct_saved_map_navigation": [
            "saved_map_relocalize_pct",
            "pct_backend",
            "legacy_pct_to_local_autonomy",
            "mujoco_motion_executor",
        ],
    },
}

BLOCKER_ACTIONS = {
    "product_data_plane": "fix the Gateway/runtime data-plane report, then rerun the gate",
    "environment_runtime": "fix host runtime first: Linux, ROS 2 Humble, CPython 3.10 PCT ABI, MuJoCo EGL",
    "artifact_contract": "generate a fresh report at the expected path with strict freshness enabled",
    "slam_localization": "debug Fast-LIO/localization stability, then rerun the live sensor gate",
    "dynamic_obstacle": "rerun the dynamic obstacle matrix with required speed/density bins and live-chain checks",
    "planning_tracking": "debug route progress, local path tracking, checkpoint completion, and cmd_vel continuity",
    "command_safety": "fix the simulation-only command boundary before accepting the report",
    "simulation_integration": "fix the simulator adapter/topic/frame contract, then rerun the integration gate",
    "unclassified": "inspect the gate report and rerun after the blocker is made explicit",
}
HOST_PREFLIGHT_ACTION = "fix host preflight before running this gate"
HOST_PREFLIGHT_REPORT_ACTION = (
    "regenerate a fresh host preflight report before using DimOS gate evidence"
)
SUMMARY_STALE_ACTION = "regenerate a fresh DimOS benchmark summary before claiming readiness"
SUMMARY_STALE_BLOCKER = "algorithm benchmark summary is stale"
HOST_PREFLIGHT_SCHEMA_VERSION = "lingtu.server_sim_host_preflight.v1"
HOST_PREFLIGHT_EXECUTION_MODE = "host_preflight_only"
HOST_CHECK_ACTIONS = {
    "pct_native": (
        "run on a Linux simulation host with the CPython 3.10 PCT native "
        "extension built and importable; start from pct_runtime_preflight.py "
        "and scripts/deploy/setup_server_ros_pct.sh"
    ),
    "ros2_humble": (
        "install/source ROS 2 Humble before running ROS-backed gates; reuse "
        "scripts/deploy/setup_server_ros_pct.sh before gate execution"
    ),
    "mujoco_headless": (
        "install the MuJoCo Python runtime and set MUJOCO_GL to egl or osmesa"
    ),
    "gazebo_runtime": (
        "install a Gazebo/Ignition runtime with gz or ign on PATH, then verify "
        "with sim/scripts/gazebo_runtime_gate.py"
    ),
    "isolated_ros_domain": (
        "set ROS_DOMAIN_ID to a nonzero isolated simulation domain before "
        "launching ROS simulation gates, then rerun server_sim_closure.py --host-preflight"
    ),
    "hardware_subscribers": (
        "prove hardware command topics have no physical robot subscribers on "
        "the simulation domain before using --run-missing"
    ),
    "localizer_runtime": "build/source the localizer runtime on the ROS 2 simulation host",
    "local_non_motion": "keep local non-motion gates fresh on the current Python host",
    "local_numeric_nav": "rerun local planner numeric gates on a Linux Python host with a stable NumPy/native planner runtime",
}
HOST_CHECK_PRIORITY = (
    "local_numeric_nav",
    "pct_native",
    "ros2_humble",
    "mujoco_headless",
    "gazebo_runtime",
    "isolated_ros_domain",
    "hardware_subscribers",
    "localizer_runtime",
)
PREFLIGHT_COMMAND = (
    "PYTHONPATH=src:. python3 sim/scripts/server_sim_closure.py "
    "--preset dimos_benchmark --required-only --host-preflight "
    "--json-out artifacts/server_sim_closure/host_preflight_dimos_benchmark.json"
)
SETUP_COMMAND = "bash scripts/deploy/setup_server_ros_pct.sh"
SAVED_MAP_RELOCALIZE_PREFLIGHT_COMMAND = (
    "PYTHONPATH=src:. python3 sim/scripts/saved_map_relocalize_runtime_gate.py "
    "--preflight-only "
    "--json-out artifacts/server_sim_closure/saved_map_relocalize_runtime_preflight/report.json"
)
HOST_CHECK_DIAGNOSTIC_COMMANDS = {
    "pct_native": (
        "PYTHONPATH=src:. python3 sim/scripts/pct_runtime_preflight.py "
        "--json-out artifacts/server_sim_closure/pct_runtime_preflight/report.json",
        SETUP_COMMAND,
        "PYTHONPATH=src:. python3 sim/scripts/pct_runtime_preflight.py "
        "--strict --json-out artifacts/server_sim_closure/pct_runtime_preflight/report.json",
        PREFLIGHT_COMMAND,
    ),
    "ros2_humble": (
        SAVED_MAP_RELOCALIZE_PREFLIGHT_COMMAND,
        SETUP_COMMAND,
        SAVED_MAP_RELOCALIZE_PREFLIGHT_COMMAND,
        PREFLIGHT_COMMAND,
    ),
    "localizer_runtime": (
        SAVED_MAP_RELOCALIZE_PREFLIGHT_COMMAND,
        SETUP_COMMAND,
        SAVED_MAP_RELOCALIZE_PREFLIGHT_COMMAND,
        PREFLIGHT_COMMAND,
    ),
}
SUMMARY_COMMAND = (
    "PYTHONPATH=src:. python3 sim/scripts/server_sim_closure.py "
    "--preset dimos_benchmark --required-only --strict --max-report-age-s 86400 "
    "--json-out artifacts/server_sim_closure/summary_dimos_benchmark_24h.json"
)
RUN_MISSING_COMMAND = (
    "PYTHONPATH=src:. python3 sim/scripts/server_sim_closure.py "
    "--preset dimos_benchmark --required-only --run-missing --skip-host-blocked --strict "
    "--max-report-age-s 86400 "
    "--json-out artifacts/server_sim_closure/summary_dimos_benchmark_24h.json"
)
GAP_REPORT_WITH_PREFLIGHT_COMMAND = (
    "PYTHONPATH=src:. python3 sim/scripts/dimos_gap_report.py "
    "--include-dataflow "
    "--host-preflight-report artifacts/server_sim_closure/host_preflight_dimos_benchmark.json "
    "--format json "
    "--json-out artifacts/server_sim_closure/dimos_gap_report_dimos_benchmark_24h.json"
)
LINUX_CLOSURE_RUNNER_COMMAND = "bash sim/scripts/run_dimos_linux_closure.sh --dry-run"
LINUX_SIM_ENV_COMMANDS = (
    'test "$(uname -s)" = Linux',
    "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-75}",
    "export MUJOCO_GL=${MUJOCO_GL:-egl}",
    "export PYOPENGL_PLATFORM=${PYOPENGL_PLATFORM:-egl}",
    "source /opt/ros/humble/setup.bash",
    "source install/setup.bash 2>/dev/null || true",
)

CATEGORY_PRIORITY = (
    "product_data_plane",
    "environment_runtime",
    "artifact_contract",
    "slam_localization",
    "dynamic_obstacle",
    "planning_tracking",
    "command_safety",
    "simulation_integration",
    "unclassified",
)

CRITICAL_STRESS_GATES = {
    "moving_obstacle_sweep",
    "large_loop_closure",
}

GATE_RUN_DEPENDENCIES: dict[str, tuple[str, ...]] = {
    "native_pct_mujoco": ("large_terrain",),
    "fastlio2_dynamic_inspection": ("large_terrain",),
    "moving_obstacle_sweep": ("fastlio2_dynamic_inspection",),
    "large_loop_closure": ("fastlio2_dynamic_inspection",),
    "pct_saved_map_navigation": ("saved_map_relocalize",),
}


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
    categories = [
        str(item)
        for item in gate_categories.get(gate_name, [])
        if str(item)
    ]
    if not categories and gate_name in missing_or_failed:
        blockers = " ".join(str(item).lower() for item in gate.get("blockers") or [])
        if any(item in blockers for item in ("abi", "native runtime", "python_tag", "no runnable pct", "mingw numpy", "numpy")):
            categories = ["environment_runtime"]
        elif any(item in blockers for item in ("fast-lio", "fastlio", "slam", "localization")):
            categories = ["slam_localization"]
        elif any(item in blockers for item in ("moving_obstacle", "moving obstacle", "clearance")):
            categories = ["dynamic_obstacle"]
        elif any(item in blockers for item in ("replay", "deviation", "tracking", "cmd_vel", "replan", "blocked_route")):
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
    if gate_name in CRITICAL_STRESS_GATES:
        return "p0"
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
        gates = {
            str(item)
            for item in stage.get("required_gates") or stage.get("all_gates") or []
        }
        if gate_name in gates:
            stages.append(str(stage.get("id") or stage.get("title") or ""))
    return [stage for stage in stages if stage]


def _next_action_by_gate(validation: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    actions: dict[str, dict[str, Any]] = {}
    for action in validation.get("next_actions") or []:
        if isinstance(action, dict) and action.get("gate"):
            actions[str(action["gate"])] = action
    return actions


def _gate_runs_by_gate(summary: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    runs: dict[str, dict[str, Any]] = {}
    for raw_run in summary.get("gate_runs") or []:
        run = _mapping(raw_run)
        name = str(run.get("name") or "")
        if name:
            runs[name] = run
    return runs


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
                    "diagnostic_commands": list(
                        check.get("diagnostic_commands")
                        or HOST_CHECK_DIAGNOSTIC_COMMANDS.get(str(check_name), ())
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
        blockers.append("schema_version is not lingtu.server_sim_host_preflight.v1")
    if host_preflight.get("execution_mode") != HOST_PREFLIGHT_EXECUTION_MODE:
        blockers.append("execution_mode is not host_preflight_only")
    required_sequence = tuple(
        str(item) for item in host_preflight.get("required_gate_sequence") or ()
    )
    if required_sequence != tuple(DIMOS_BENCHMARK_REQUIRED_GATES):
        blockers.append("required_gate_sequence does not match DimOS required gates")
    if host_preflight.get("blocked_gates"):
        blockers.append("blocked_gates is not empty")
    freshness = _mapping(host_preflight.get("report_freshness"))
    if freshness.get("checked") is not True:
        blockers.append("host preflight report freshness was not checked")
    elif freshness.get("fresh") is not True:
        freshness_blockers = [
            str(item) for item in freshness.get("blockers") or [] if str(item)
        ]
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
    gates = {
        str(gate_name): dict(_mapping(raw_gate))
        for gate_name, raw_gate in _mapping(payload.get("gates")).items()
    }
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
            "diagnostic_commands": [PREFLIGHT_COMMAND],
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
                "failed_edges": [
                    str(edge)
                    for edge in runtime_brief.get("failed_edges") or []
                    if str(edge)
                ],
                "candidate_reports": list(runtime_brief.get("candidate_reports") or []),
            }
        )
    if gate_blockers:
        blockers.append({"source": "gate_report", "blockers": list(gate_blockers)})
    return blockers


def _command_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    commands: list[dict[str, Any]] = []
    for row in rows:
        command = str(row.get("gate_command") or "")
        if not command:
            continue
        commands.append(
            {
                "gate": row["gate"],
                "priority": row["priority"],
                "command": command,
                "expected_report_path": row.get("expected_report_path", ""),
                "host_preflight_ok": row["host_preflight"]["ok"],
                "host_failed_checks": list(
                    row["host_preflight"].get("failed_checks") or []
                ),
                "host_preflight_blockers": list(
                    row["host_preflight"].get("blockers") or []
                ),
                "dependency_blockers": list(row.get("dependency_blockers") or []),
                "dependency_blocker_status": dict(
                    row.get("dependency_blocker_status") or {}
                ),
                "runtime_dataflow_blocker": row.get("runtime_dataflow_blocker", ""),
                "runtime_dataflow_failed_edges": list(
                    row.get("runtime_dataflow_failed_edges") or []
                ),
                "evidence_blockers": list(row.get("evidence_blockers") or []),
            }
        )
    return commands


def _host_setup_command_rows(host_setup_plan: Mapping[str, Any]) -> list[dict[str, Any]]:
    commands: list[dict[str, Any]] = []
    seen: set[str] = set()

    def add(
        *,
        command: str,
        check: str = "",
        gates: list[str] | None = None,
        blockers: list[str] | None = None,
        recommended_action: str = "",
    ) -> None:
        if not command or command in seen:
            return
        seen.add(command)
        commands.append(
            {
                "command": command,
                "check": check,
                "gates": list(gates or []),
                "host_preflight_blockers": list(blockers or []),
                "recommended_action": recommended_action,
            }
        )

    for raw_check in host_setup_plan.get("failed_checks") or []:
        check = _mapping(raw_check)
        check_name = str(check.get("check") or "")
        diagnostic_commands = [
            str(command)
            for command in check.get("diagnostic_commands") or []
            if str(command)
        ]
        for command in diagnostic_commands:
            add(
                command=command,
                check=check_name,
                gates=[str(gate) for gate in check.get("gates") or [] if str(gate)],
                blockers=[
                    str(blocker)
                    for blocker in check.get("blockers") or []
                    if str(blocker)
                ],
                recommended_action=str(check.get("recommended_action") or ""),
            )

    if not commands:
        add(command=SETUP_COMMAND)
        add(command=PREFLIGHT_COMMAND)
    return commands


def _linux_sim_closure_command_rows(
    rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    gates = [str(row.get("gate") or "") for row in rows if row.get("gate")]
    commands = [
        {
            "command": LINUX_CLOSURE_RUNNER_COMMAND,
            "check": "closure_runner_preview",
            "gates": gates,
            "recommended_action": (
                "preview the versioned target-host closure runner; switch it "
                "to --execute only on the prepared Linux simulation host"
            ),
        }
    ]
    commands.extend(
        [
            {
                "command": command,
                "check": "target_linux_sim_host" if index == 0 else "",
                "gates": gates if index == 0 else [],
                "recommended_action": (
                    "run this closure phase on the Linux ROS2/PCT/MuJoCo simulation host, "
                    "not on the current blocked host"
                    if index == 0
                    else ""
                ),
            }
            for index, command in enumerate(LINUX_SIM_ENV_COMMANDS)
        ]
    )
    commands.extend(
        [
            {
                "command": PREFLIGHT_COMMAND,
                "check": "host_preflight",
                "gates": gates,
                "recommended_action": (
                    "prove the target host is runnable before launching DimOS gates"
                ),
            },
            {
                "command": RUN_MISSING_COMMAND,
                "check": "run_missing",
                "gates": gates,
                "recommended_action": (
                    "run missing DimOS gates in dependency order with host-blocked gates skipped"
                ),
            },
            {
                "command": GAP_REPORT_WITH_PREFLIGHT_COMMAND,
                "check": "gap_report",
                "gates": gates,
                "recommended_action": (
                    "recompute the DimOS readiness claim from the target host preflight and runtime dataflow"
                ),
            },
        ]
    )
    return commands


def _annotate_dependency_blockers(rows: list[dict[str, Any]]) -> None:
    row_by_gate = {str(row.get("gate")): row for row in rows}
    blocked_cache: dict[str, bool] = {}

    def dependency_blocks_launch(gate_name: str) -> bool:
        if gate_name in blocked_cache:
            return blocked_cache[gate_name]
        row = row_by_gate.get(gate_name)
        if not row or row.get("ok") is True:
            blocked_cache[gate_name] = False
            return False
        if _mapping(row.get("host_preflight")).get("ok") is False:
            blocked_cache[gate_name] = True
            return True
        blockers: list[str] = []
        blocker_status: dict[str, str] = {}
        for dependency in GATE_RUN_DEPENDENCIES.get(gate_name, ()):
            dependency_row = row_by_gate.get(dependency)
            if dependency_row and dependency_row.get("ok") is True:
                continue
            blockers.append(dependency)
            if dependency_row:
                if _mapping(dependency_row.get("host_preflight")).get("ok") is False:
                    blocker_status[dependency] = "host_blocked"
                else:
                    dependency_blocks_launch(dependency)
                    blocker_status[dependency] = str(
                        dependency_row.get("status") or "failed"
                    )
            else:
                blocker_status[dependency] = "missing"
        row["dependency_blockers"] = blockers
        row["dependency_blocker_status"] = blocker_status
        blocked_cache[gate_name] = True
        return blocked_cache[gate_name]

    for row in sorted(rows, key=lambda item: _gate_order_rank(str(item.get("gate")))):
        row.setdefault("dependency_blockers", [])
        row.setdefault("dependency_blocker_status", {})
        dependency_blocks_launch(str(row.get("gate") or ""))


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
        str(edge.get("id") or "")
        for edge in flow
        if edge.get("ok") is not True and str(edge.get("id") or "")
    ]
    edge_status = {
        str(edge.get("id")): edge.get("ok") is True
        for edge in flow
        if str(edge.get("id") or "")
    }
    edge_evidence = {
        str(edge.get("id")): _jsonable(_mapping(edge.get("evidence")))
        for edge in flow
        if str(edge.get("id") or "")
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
        "pct_optimizer_mode": _jsonable(_mapping(payload.get("pct_optimizer_mode"))),
        "claim_boundary": payload.get("claim_boundary") or "",
        "environment": _jsonable(_mapping(payload.get("environment"))),
        "source_gate_report": payload.get("source_gate_report") or "",
        "source_report": payload.get("source_report") or "",
        "reason": payload.get("reason") or "",
        "candidate_reports": list(payload.get("candidate_reports") or []),
        "child_report_freshness": _jsonable(
            _mapping(payload.get("child_report_freshness"))
        ),
    }


def _runtime_dataflow_summary(
    runtime_dataflow_gates: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    expected_gates = set(RUNTIME_DATAFLOW_GATES)
    if not runtime_dataflow_gates:
        missing = {
            gate: "runtime dataflow report missing"
            for gate in sorted(expected_gates)
        }
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
    missing = {
        gate: "runtime dataflow report missing"
        for gate in sorted(expected_gates - set(runtime_dataflow_gates))
    }
    gate_failures = {**missing, **unchecked, **failing}
    cross_gate_chains = _runtime_dataflow_cross_gate_chains(runtime_dataflow_gates)
    failing_cross_gate_chains = {
        name: "; ".join(str(item) for item in chain.get("blockers") or ["unknown"])
        for name, chain in cross_gate_chains.items()
        if _mapping(chain).get("ok") is not True
    }
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
        "cross_gate_chains": cross_gate_chains,
        "failing_cross_gate_chains": failing_cross_gate_chains,
        "gates": _jsonable(runtime_dataflow_gates),
    }


def _runtime_dataflow_gate_brief(payload: Mapping[str, Any]) -> dict[str, Any]:
    value = _mapping(payload)
    if not value:
        return {
            "checked": False,
            "ok": False,
            "schema_detected": "",
            "primary_blocker": "dataflow missing",
            "source_report": "",
        }
    return {
        "checked": value.get("checked", True) is not False,
        "ok": value.get("ok") is True,
        "schema_detected": value.get("schema_detected") or "",
        "primary_blocker": value.get("primary_blocker") or value.get("reason") or "",
        "source_report": value.get("source_report") or "",
        "pct_provenance": _jsonable(_mapping(value.get("pct_provenance"))),
        "same_source_provenance": _jsonable(
            _mapping(value.get("same_source_provenance"))
        ),
        "planner_backend": _jsonable(_mapping(value.get("planner_backend"))),
        "local_planner_backend": value.get("local_planner_backend") or "",
        "path_follower_backend": value.get("path_follower_backend") or "",
        "candidate_reports": list(value.get("candidate_reports") or []),
        "child_report_freshness": _jsonable(
            _mapping(value.get("child_report_freshness"))
        ),
    }


def _runtime_dataflow_cross_gate_chains(
    runtime_dataflow_gates: Mapping[str, Mapping[str, Any]],
) -> dict[str, dict[str, Any]]:
    chains: dict[str, dict[str, Any]] = {}
    native = _runtime_dataflow_gate_brief(
        _mapping(runtime_dataflow_gates.get("native_pct_mujoco"))
    )
    if "policy_nav" in runtime_dataflow_gates:
        policy = _runtime_dataflow_gate_brief(
            _mapping(runtime_dataflow_gates.get("policy_nav"))
        )
        policy_blockers: list[str] = []
        if policy.get("checked") is not True:
            policy_blockers.append("policy_nav dataflow missing")
        elif policy.get("ok") is not True:
            policy_blockers.append("policy_nav dataflow is not ok")
        if _mapping(policy.get("planner_backend")).get("configured_backend") not in (
            "",
            "octoplanner3d",
        ):
            policy_blockers.append("policy_nav did not configure OctoPlanner3D")
        if str(policy.get("local_planner_backend") or "") not in ("", "nanobind"):
            policy_blockers.append("policy_nav did not use nanobind local planner")
        if str(policy.get("path_follower_backend") or "") not in ("", "nav_kernel"):
            policy_blockers.append("policy_nav did not use nav_kernel path follower")
        chains["product_octoplanner_inprocess_policy_nav"] = {
            "checked": True,
            "ok": not policy_blockers,
            "policy_nav_gate": policy,
            "claim_boundary": "current_product_inprocess_nav_no_ros2",
            "blockers": policy_blockers,
        }
    fastlio_gate_names = (
        "fastlio2_dynamic_inspection",
        "moving_obstacle_sweep",
        "large_loop_closure",
    )
    fastlio_gates = {
        name: _runtime_dataflow_gate_brief(_mapping(runtime_dataflow_gates.get(name)))
        for name in fastlio_gate_names
    }
    fastlio_candidate_rejections: list[dict[str, Any]] = []
    for gate_name, payload in fastlio_gates.items():
        for candidate in payload.get("candidate_reports") or []:
            if not isinstance(candidate, Mapping):
                continue
            fastlio_candidate_rejections.append(
                {
                    "gate": gate_name,
                    "path": candidate.get("path") or "",
                    "ok": candidate.get("ok") is True,
                    "schema_detected": candidate.get("schema_detected") or "",
                    "primary_blocker": (
                        candidate.get("primary_blocker")
                        or candidate.get("reason")
                        or ""
                    ),
                    "failed_edges": list(candidate.get("failed_edges") or []),
                    "rejection_reason": candidate.get("rejection_reason") or "",
                }
            )
    first_ok_fastlio_gate = next(
        (name for name, payload in fastlio_gates.items() if payload.get("ok") is True),
        "",
    )
    pct_proven_gate = next(
        (
            name
            for name, payload in fastlio_gates.items()
            if payload.get("ok") is True
            and _mapping(payload.get("pct_provenance")).get("ok") is True
        ),
        "",
    )
    same_run_gate = next(
        (
            name
            for name, payload in fastlio_gates.items()
            if payload.get("ok") is True
            and _mapping(payload.get("pct_provenance")).get("ok") is True
            and _mapping(payload.get("same_source_provenance")).get("ok") is True
        ),
        "",
    )
    selected_fastlio_gate = same_run_gate or first_ok_fastlio_gate
    any_fastlio_checked = any(
        payload.get("checked") is True for payload in fastlio_gates.values()
    )
    any_fastlio_ok = bool(first_ok_fastlio_gate)
    native_same_source_ok = (
        _mapping(native.get("same_source_provenance")).get("ok") is True
    )
    fastlio_same_run_proven = bool(same_run_gate)
    same_run_proven = fastlio_same_run_proven and native_same_source_ok
    blockers: list[str] = []
    if native.get("checked") is not True:
        blockers.append("native_pct_mujoco dataflow missing")
    elif native.get("ok") is not True:
        blockers.append("native_pct_mujoco dataflow is not ok")
    elif not native_same_source_ok:
        blockers.append(
            "native_pct_mujoco dataflow did not prove same-source map artifacts"
        )
    if not any_fastlio_checked:
        blockers.append("Fast-LIO live dataflow missing")
    elif not any_fastlio_ok:
        blockers.append("Fast-LIO live dataflow is not ok")
    elif not pct_proven_gate:
        blockers.append(
            "Fast-LIO live dataflow did not prove PCT planner in the same run"
        )
    elif not fastlio_same_run_proven:
        blockers.append(
            "Fast-LIO live dataflow did not prove same-source map artifacts"
        )
    chains["pct_mujoco_and_fastlio_live"] = {
        "checked": True,
        "ok": not blockers,
        "native_pct_gate": native,
        "fastlio_live_gates": fastlio_gates,
        "fastlio_candidate_rejections": fastlio_candidate_rejections,
        "selected_fastlio_gate": selected_fastlio_gate,
        "same_run_proven": same_run_proven,
        "native_same_source_proven": native_same_source_ok,
        "fastlio_same_source_proven": fastlio_same_run_proven,
        "claim_boundary": (
            "same_run_pct_fastlio_live"
            if same_run_proven
            else "cross_gate_evidence_not_single_run_fusion"
        ),
        "blockers": blockers,
    }
    return chains


def _execution_plan(
    *,
    failed_rows: list[dict[str, Any]],
    prioritized_failed_rows: list[dict[str, Any]],
    host_setup_plan: Mapping[str, Any],
    summary_is_stale: bool,
) -> dict[str, Any]:
    phases: list[dict[str, Any]] = [
        {
            "id": "host_preflight",
            "status": "required",
            "purpose": "prove this host can run the DimOS benchmark gates without launching them",
            "commands": [PREFLIGHT_COMMAND],
            "stop_condition": "host_preflight.ok=true or host_setup_plan explains blockers",
        }
    ]

    host_preflight_checked = host_setup_plan.get("checked") is True
    host_blocked = host_preflight_checked and host_setup_plan.get("ok") is False
    if host_blocked:
        phases.append(
            {
                "id": "host_setup",
                "status": "blocked",
                "purpose": "prepare the Linux ROS2/PCT simulation host before running blocked gates",
                "commands": _host_setup_command_rows(host_setup_plan),
                "failed_checks": [
                    item.get("check")
                    for item in host_setup_plan.get("failed_checks") or []
                ],
                "stop_condition": "all required host checks pass on the simulation host",
            }
        )

    unblocked_rows = [
        row
        for row in failed_rows
        if host_preflight_checked and row["host_preflight"]["ok"] is not False
        and not row.get("dependency_blockers")
    ]
    unblocked_rows_in_dependency_order = sorted(
        unblocked_rows,
        key=lambda row: (_gate_order_rank(str(row["gate"])), int(row["order"])),
    )
    blocked_rows = [
        row
        for row in prioritized_failed_rows
        if host_preflight_checked and row["host_preflight"]["ok"] is False
    ]
    blocked_rows_in_dependency_order = sorted(
        blocked_rows,
        key=lambda row: (_gate_order_rank(str(row["gate"])), int(row["order"])),
    )
    if host_blocked and blocked_rows_in_dependency_order:
        phases.append(
            {
                "id": "linux_sim_closure",
                "status": "target_host",
                "purpose": (
                    "run the full DimOS closure loop on the prepared Linux "
                    "ROS2/PCT/MuJoCo simulation host"
                ),
                "order": "host_preflight_then_dimos_dependency_order",
                "commands": _linux_sim_closure_command_rows(
                    blocked_rows_in_dependency_order,
                ),
                "gates": [
                    row["gate"] for row in blocked_rows_in_dependency_order
                ],
                "stop_condition": (
                    "target host preflight passes, run-missing completes, and "
                    "dimos gap report has lingtu_readiness.ok=true"
                ),
            }
        )
    dependency_blocked_rows = [
        row
        for row in failed_rows
        if host_preflight_checked
        and row["host_preflight"]["ok"] is not False
        and row.get("dependency_blockers")
    ]
    dependency_blocked_rows_in_order = sorted(
        dependency_blocked_rows,
        key=lambda row: (_gate_order_rank(str(row["gate"])), int(row["order"])),
    )

    if unblocked_rows_in_dependency_order:
        phases.append(
            {
                "id": "run_unblocked_gate_commands",
                "status": "pending",
                "purpose": "run missing DimOS gates that this host is allowed to execute, in dependency order",
                "order": "dimos_dependency_order",
                "commands": _command_rows(unblocked_rows_in_dependency_order),
                "gates": [row["gate"] for row in unblocked_rows_in_dependency_order],
                "stop_condition": "all runnable gate reports pass under strict evaluators",
            }
        )
    if blocked_rows_in_dependency_order:
        phases.append(
            {
                "id": "blocked_gate_commands",
                "status": "blocked",
                "purpose": "commands to run after host_setup passes; do not launch on the current blocked host",
                "order": "dimos_dependency_order",
                "commands": _command_rows(blocked_rows_in_dependency_order),
                "gates": [row["gate"] for row in blocked_rows_in_dependency_order],
                "stop_condition": "host_preflight passes for these gates before command execution",
            }
        )
    if dependency_blocked_rows_in_order:
        phases.append(
            {
                "id": "dependency_blocked_gate_commands",
                "status": "blocked",
                "purpose": "commands to run only after prerequisite gate reports pass in this run sequence",
                "order": "dimos_dependency_order",
                "commands": _command_rows(dependency_blocked_rows_in_order),
                "gates": [row["gate"] for row in dependency_blocked_rows_in_order],
                "stop_condition": "dependency_blockers is empty for every listed gate",
            }
        )
    if not host_preflight_checked and failed_rows:
        preflight_required_rows = sorted(
            failed_rows,
            key=lambda row: (_gate_order_rank(str(row["gate"])), int(row["order"])),
        )
        phases.append(
            {
                "id": "preflight_required_gate_commands",
                "status": "blocked",
                "purpose": (
                    "commands are listed for planning only; run host_preflight "
                    "first so blocked ROS/PCT/MuJoCo gates are not launched on "
                    "an unsuitable host"
                ),
                "order": "dimos_dependency_order",
                "commands": _command_rows(preflight_required_rows),
                "gates": [row["gate"] for row in preflight_required_rows],
                "stop_condition": "host_preflight checked before any gate command runs",
            }
        )

    if summary_is_stale:
        phases.append(
            {
                "id": "refresh_summary",
                "status": "required",
                "purpose": "replace stale benchmark evidence with a fresh strict summary",
                "commands": [SUMMARY_COMMAND],
                "stop_condition": "summary_freshness.fresh=true",
            }
        )

    phases.append(
        {
            "id": "final_summary",
            "status": "required" if failed_rows else "ready",
            "purpose": "recompute the strict DimOS benchmark claim boundary",
            "commands": [RUN_MISSING_COMMAND if failed_rows and not host_blocked else SUMMARY_COMMAND],
            "stop_condition": "lingtu_readiness.ok=true and gap_counts.failed=0",
        }
    )

    return {
        "schema_version": "lingtu.dimos_execution_plan.v1",
        "ok_to_run_missing": host_preflight_checked and not host_blocked,
        "phase_count": len(phases),
        "phases": phases,
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
    """Build the canonical DimOS-style gap matrix from a closure summary."""
    generated_at = time.time() if generated_at is None else generated_at
    metadata = gate_metadata or {}
    validation = _mapping(summary.get("algorithm_validation"))
    gates = _mapping(summary.get("gates"))
    missing_or_failed = {
        str(item)
        for item in summary.get("missing_or_failed") or []
        if str(item)
    }
    gate_categories = _mapping(validation.get("gate_categories"))
    validation_flow = list(validation.get("validation_flow") or [])
    next_actions = _next_action_by_gate(validation)
    summary_host_preflight_payload = _mapping(summary.get("run_missing_host_preflight"))
    explicit_host_preflight_payload = _mapping(host_preflight)
    host_preflight_payload = explicit_host_preflight_payload or summary_host_preflight_payload
    host_preflight_source = (
        host_preflight_source
        or "argument"
        if explicit_host_preflight_payload
        else "summary_run_missing_host_preflight"
        if summary_host_preflight_payload
        else ""
    )
    if host_preflight_payload:
        host_preflight_payload = _host_preflight_with_contract_gate(
            host_preflight_payload
        )
    host_setup_plan = _host_setup_plan(host_preflight_payload)
    host_preflight_gates = _mapping(host_preflight_payload.get("gates"))
    gate_runs = _gate_runs_by_gate(summary)
    freshness_payload = _mapping(summary_freshness)
    runtime_dataflow_by_gate = _runtime_dataflow_gates(runtime_dataflow)
    runtime_dataflow_summary = _runtime_dataflow_summary(runtime_dataflow_by_gate)
    runtime_dataflow_checked = runtime_dataflow_summary.get("checked") is True
    runtime_dataflow_complete = runtime_dataflow_summary.get("complete") is True
    runtime_dataflow_gate_failures = _mapping(
        runtime_dataflow_summary.get("gate_failures")
    )
    failing_cross_gate_chains = _mapping(
        runtime_dataflow_summary.get("failing_cross_gate_chains")
    )
    runtime_dataflow_ok = (
        runtime_dataflow_checked
        and runtime_dataflow_complete
        and not runtime_dataflow_gate_failures
        and not failing_cross_gate_chains
    )
    cross_gate_failures = list(failing_cross_gate_chains)
    summary_is_stale = (
        freshness_payload.get("fresh") is False
        or freshness_payload.get("stale") is True
    )
    summary_stale_blocker = str(
        freshness_payload.get("blocker") or SUMMARY_STALE_BLOCKER
    )

    rows: list[dict[str, Any]] = []
    for index, gate_name in enumerate(DIMOS_BENCHMARK_REQUIRED_GATES, start=1):
        gate = _mapping(gates.get(gate_name))
        meta = _mapping(metadata.get(gate_name))
        gate_run = _mapping(gate_runs.get(gate_name))
        gate_run_status = str(gate_run.get("status") or "").lower()
        gate_run_preflight = _mapping(gate_run.get("host_preflight"))
        preflight_gate = _mapping(host_preflight_gates.get(gate_name)) or gate_run_preflight
        gate_evidence = _mapping(gate.get("evidence"))
        host_blockers = [str(item) for item in preflight_gate.get("blockers") or []]
        if (
            not host_blockers
            and gate_run_status == "host_blocked"
            and str(gate_run.get("error") or "")
        ):
            host_blockers = [str(gate_run.get("error"))]
        preflight_checks = _mapping(preflight_gate.get("checks"))
        host_failed_checks = [
            str(check_name)
            for check_name, check in preflight_checks.items()
            if _mapping(check).get("ok") is False
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
        status = "stale" if summary_is_stale else _gate_status(
            gate_name,
            gate,
            missing_or_failed,
        )
        if not summary_is_stale and gate_run_status == "host_blocked" and status != "pass":
            status = "host_blocked"
        primary_category = _first_category(categories)
        action = next_actions.get(gate_name, {})
        reference = DIMOS_REFERENCE_SURFACES.get(gate_name, {})
        blockers = list(gate.get("blockers") or action.get("blockers") or [])
        gate_run_error = str(gate_run.get("error") or "")
        if gate_run_error and gate_run_error not in blockers:
            blockers.append(gate_run_error)
        if summary_is_stale and summary_stale_blocker not in blockers:
            blockers.insert(0, summary_stale_blocker)
        if status != "pass" and not blockers:
            blockers = ["report missing or not verified"]
        runtime_brief = _runtime_dataflow_brief(
            gate_name,
            runtime_dataflow_by_gate,
        )
        runtime_blocker = _runtime_dataflow_blocker(runtime_brief)
        runtime_failed_edges = [
            str(edge) for edge in runtime_brief.get("failed_edges") or [] if str(edge)
        ]
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
                "pipeline_trace": list(
                    _mapping(PIPELINE_TRACE.get("gate_to_trace")).get(gate_name, [])
                ),
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
                "gate_command": action.get("command") or meta.get("command", ""),
                "expected_report_path": (
                    action.get("expected_report_path")
                    or meta.get("expected_report_path", "")
                ),
                "report_path": gate.get("path") or action.get("report_path") or "",
                "blockers": blockers,
                "run_missing_attempt": {
                    "checked": bool(gate_run),
                    "status": gate_run.get("status", "") if gate_run else "",
                    "returncode": gate_run.get("returncode") if gate_run else None,
                    "executed_command": gate_run.get("executed_command") if gate_run else None,
                    "shell": gate_run.get("shell") if gate_run else None,
                    "error": gate_run_error,
                },
                "host_requirements": list(
                    action.get("host_requirements") or meta.get("host_requirements") or []
                ),
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
    _annotate_dependency_blockers(rows)
    for row in failed_rows:
        dependency_blockers = list(row.get("dependency_blockers") or [])
        if not dependency_blockers:
            continue
        blocker_text = "blocked by unmet prerequisite gate(s): " + ", ".join(
            dependency_blockers
        )
        if blocker_text not in row["blockers"]:
            row["blockers"].append(blocker_text)
        row["evidence_blockers"].append(
            {
                "source": "dependency",
                "blockers": list(dependency_blockers),
                "dependency_blocker_status": dict(
                    row.get("dependency_blocker_status") or {}
                ),
            }
        )
        row["recommended_action"] = (
            "run prerequisite gate reports first: " + ", ".join(dependency_blockers)
        )
    prioritized_failed_rows = sorted(
        failed_rows,
        key=lambda row: (_priority_rank(str(row["priority"])), int(row["order"])),
    )
    actionable_failed_rows = [
        row for row in prioritized_failed_rows if not row.get("dependency_blockers")
    ]
    dependency_ordered_failed_rows = sorted(
        failed_rows,
        key=lambda row: (
            bool(row.get("dependency_blockers")),
            _gate_order_rank(str(row["gate"])),
            _priority_rank(str(row["priority"])),
            int(row["order"]),
        ),
    )
    host_preflight_failed = (
        host_setup_plan.get("checked") is True and host_setup_plan.get("ok") is False
    )
    if summary_is_stale:
        stop_condition = (
            "do not claim DimOS-style algorithm health until the benchmark "
            "summary is regenerated within the freshness limit"
        )
    elif host_preflight_failed and not failed_rows:
        stop_condition = (
            "do not claim DimOS-style algorithm health until host preflight "
            "passes: "
            + ", ".join(
                str(item.get("check") or "")
                for item in host_setup_plan.get("failed_checks") or []
                if str(item.get("check") or "")
            )
        )
    elif not runtime_dataflow_checked and not failed_rows:
        stop_condition = (
            "do not claim DimOS-style algorithm health until runtime dataflow "
            "is checked with include_dataflow"
        )
    elif not runtime_dataflow_complete and not failed_rows:
        stop_condition = (
            "do not claim DimOS-style algorithm health until runtime dataflow "
            "evidence is complete for these gates: "
            + ", ".join(runtime_dataflow_gate_failures)
        )
    elif runtime_dataflow_gate_failures and not failed_rows:
        stop_condition = (
            "do not claim DimOS-style algorithm health until runtime dataflow "
            "passes for these gates: "
            + ", ".join(runtime_dataflow_gate_failures)
        )
    elif cross_gate_failures and not failed_rows:
        stop_condition = (
            "do not claim DimOS-style algorithm health until cross-gate "
            "dataflow passes: "
            + ", ".join(cross_gate_failures)
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
            "cross_gate_failures": cross_gate_failures,
            "host_preflight_ok": (
                None
                if host_setup_plan.get("checked") is not True
                else host_setup_plan.get("ok") is True
            ),
            "missing_or_failed": [row["gate"] for row in failed_rows],
            "highest_priority_blocker": (
                prioritized_failed_rows[0]["gate"] if prioritized_failed_rows else ""
            ),
            "highest_actionable_blocker": (
                actionable_failed_rows[0]["gate"] if actionable_failed_rows else ""
            ),
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
                "command": row["gate_command"],
                "expected_report_path": row["expected_report_path"],
                "dependency_blockers": list(row.get("dependency_blockers") or []),
                "dependency_blocker_status": dict(
                    row.get("dependency_blocker_status") or {}
                ),
                "host_preflight_ok": row["host_preflight"]["ok"],
                "host_preflight_blockers": row["host_preflight"]["blockers"],
                "host_failed_checks": row["host_preflight"]["failed_checks"],
                "runtime_dataflow_blocker": row["runtime_dataflow_blocker"],
                "runtime_dataflow_failed_edges": row["runtime_dataflow_failed_edges"],
                "evidence_blockers": row["evidence_blockers"],
            }
            for row in dependency_ordered_failed_rows
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
    report["execution_plan"] = _execution_plan(
        failed_rows=failed_rows,
        prioritized_failed_rows=prioritized_failed_rows,
        host_setup_plan=host_setup_plan,
        summary_is_stale=summary_is_stale,
    )
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
